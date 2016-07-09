//
//  testing a subd idea
//

#include <stdlib.h>
#include <iostream>
#include <cctype>     // std::tolower
#include <algorithm>
#include <vector>
// in project properties, add "../include" to the vc++ directories include path

#include "geometric.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "misc_gl.h"
#include "wingmesh.h"
#include "minixml.h"


void wmwire(const WingMesh &m)
{
	glBegin(GL_LINES);
	for (auto e : m.edges)
	{
		glVertex3fv(m.verts[e.v]);
		glVertex3fv(m.verts[m.edges[e.next].v]);
	}
	glEnd();
}
void wmdraw(const WingMesh &m)
{
	gldraw(m.verts, m.GenerateTris());
}


std::vector<float3> fvertsnew;
WingMesh WingMeshSubDiv(WingMesh wm)
{
	auto nv = (int) wm.verts.size();
	auto ne = (int) wm.edges.size();
	auto nf = (int) wm.faces.size();
	fvertsnew.clear();
	
	for (int fid = 0;fid < nf;fid++)
	{
		float3 com(0, 0, 0);
		auto fverts = wm.GenerateFaceVerts(fid);
		for (auto v : fverts)
			com += v;
		com /= (float)fverts.size();
		fvertsnew.push_back(com);
	}
	for (int i = 0;i < ne;i++)
	{
		if (wm.edges[i].v >= nv || wm.edges[i].Adj().v >=nv   )   // already split
			continue;
		auto &e = wm.edges[i];
		auto &a = wm.edges[e.adj];
		float3 v_new = (wm.verts[e.v] + wm.verts[a.v] + fvertsnew[e.face] + fvertsnew[a.face] ) / 4.0f;
		wm.SplitEdge(i, v_new);
	}
	for (int vid = 0;vid < nv;vid++)
	{
		int k = 0;  
		float3 fcom,ecom;
		for (auto &eid : wm.VertEdges(vid))
		{
			ecom += wm.verts[wm.edges[eid].Adj().v];
			fcom +=   fvertsnew[wm.edges[eid].face] ;
			k++;
		}
		wm.verts[vid] = wm.verts[vid] * ((k - 2.0f) / k) + ecom*(1.0f / k / k) + fcom*(1.0f / k / k);
	}

	for (int fid = 0;fid < nf;fid++)
	{
		std::vector<int> eids;
		for (auto edge : wm.FaceView(fid))
		{
			if (edge.v >= nv)
				eids.push_back(edge.id);
		}
		wm.BuildEdge( eids[1],eids[0]);
		wm.SplitEdge(wm.edges.size() - 1, fvertsnew[fid]);
		int eid_new = wm.edges.size() - 2; // this whould be the one with the correct face
		assert(wm.edges[eid_new].v == wm.verts.size() - 1);
		assert(wm.edges[eid_new].face == fid);
		for (int e = eids.size() - 1; e>=2;e--)
			wm.BuildEdge(eids[e], eid_new);
	}
	return wm;
}

std::vector<std::string> split(std::string line, std::string delimeter=" ")
{
	std::vector<std::string> tokens;
	size_t pos = 0;
	while ((pos = line.find(delimeter)) != std::string::npos)
	{
		auto token = line.substr(0, pos);
		line.erase(0, pos + delimeter.length());
		if (token.length())
			tokens.push_back(token);
	}
	if (line.length())
		tokens.push_back(line);
	return tokens;
}

WingMesh WingMeshLoad(const char *filename)
{
	WingMesh wm;
	std::ifstream filein(filename);
	if (!filein.is_open())
		throw "unable to open file";
	std::string line;
	while (std::getline(filein, line))
	{
		auto tokens = split(line);
		if (!tokens.size())
			continue;
		auto head = tokens.front(); 
		tokens.erase(tokens.begin());
		if (head == "v")
		{
			line.erase(0, 1);
			wm.verts.push_back(StringTo<float3>(line)*0.1f);
			continue;
		}
		if (head == "f")
		{
			std::vector<int> vids;
			for(auto &token:tokens)
				vids.push_back(StringTo<int>(split(token,"/")[0]) - 1);  // ugg obj index from one instead of zero
			int base = wm.edges.size();
			std::vector<float3> fverts;
			for (int i = 0; i < (int)vids.size(); i++)
			{
				int inext = (i + 1) % vids.size();
				int iprev = (i + vids.size()-1) % vids.size();
				wm.edges.push_back(WingMesh::HalfEdge(base + i, vids[i], -1, base + inext, base + iprev, wm.faces.size()));
				fverts.push_back(wm.verts[vids[i]]);
			}
			wm.faces.push_back(PolyPlane(fverts));
			continue;
		}
	}
	wm.LinkMesh();
	wm.InitBackLists();
	return wm;
}

int main(int argc, char *argv[])
{
	std::cout << "Test...\n";
	auto body = WingMeshLoad("BodyData/EntireBody.obj");
	auto body1 = WingMeshSubDiv(body);
	auto body2 = WingMeshSubDiv(body1);

	//auto box = WingMeshSubDiv(WingMeshSubDiv(WingMeshCube(0.5f)));

	GLWin glwin("Test CatmullClark");
	glwin.keyboardfunc = [](int key, int, int)
	{
			switch (std::tolower(key))
			{
			case ' ':

				break;
			case 27:   // ESC
			case 'q':
				exit(0);
				break;
			default:
				std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
				break;
			}

	};

	float3 mousevec_prev;
	float4 model_orientation(0, 0, 0, 1);
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			model_orientation = qmul(VirtualTrackBall(float3(0, 0, 2), float3(0, 0, 0), mousevec_prev, glwin.MouseVector), model_orientation);
		}
		mousevec_prev = glwin.MouseVector;



		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set up the viewport
		glViewport(0, 0, glwin.Width, glwin.Height);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width / glwin.Height, 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 2, 0, 0, 0, 0, 1, 0);

		float4x4 R = { { 1, 0, 0, 0 },{ 0, 1, 0, 0 },{ 0, 0, 1, 0 },{ 0, 0, 0, 1 } };
		R[0].xyz() = qxdir(model_orientation); R[1].xyz() = qydir(model_orientation); R[2].xyz() = qzdir(model_orientation);
		glMultMatrixf(R);

		glEnable(GL_DEPTH_TEST);


		glDisable(GL_BLEND);
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3f(0, 1, 0);
		for (auto &v : fvertsnew)
		{
			glVertex3fv(v);
		}
		glEnd();

		glColor3f(0, 1, 1);
		glEnable(GL_CULL_FACE);
		//glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		wmwire(body2);
		glTranslatef(0.3f,0,0 );
		wmwire(body1);
		glTranslatef( 0.3f,0,0);
		wmwire(body );



		// Restore state
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();  //should be currently in modelview mode
		glPopAttrib();

		glwin.PrintString({ 0, 0 }, "Press ESC to quit.");

		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

