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

float3 *HitCheckPoint(std::vector<float3> &points,const float3 &v0,float3 v1,float box_radius)  // typically used for vertex selection 
{
	float3 *point_hit = NULL;
	float w = -box_radius;
	std::vector<float4> planes = { { 1, 0, 0, w },{ 0, 1, 0, w },{ 0, 0, 1, w },{ -1, 0, 0, w },{ 0, -1, 0, w },{ 0, 0, -1, w } };
	for (float3 &p : points)
	{
		if (auto h = ConvexHitCheck(planes, Pose(p, { 0, 0, 0, 1 }), v0, v1))
		{
			point_hit = &p;
			v1 = h.impact;
		}
	}
	return point_hit;
}


int main(int argc, char *argv[])
{
	std::cout << "Test...\n";
	auto body = WingMeshLoad("EntireBody.obj");

	//auto box = WingMeshSubDiv(WingMeshSubDiv(WingMeshCube(0.5f)));
	float3 *selected = NULL;
	float3 *operand  = NULL; // for any two point operations
	GLWin glwin("Test CatmullClark");
	glwin.keyboardfunc = [&](int key, int, int)
	{
			switch (std::tolower(key))
			{
			case ' ':
				break;
			case 27:   // ESC
			case 'q':
				exit(0);
				break;
			case 'e':
			{	
				if (operand == selected)
				{
					operand = NULL;  // de-select 
					break;  
				}
				if (!operand || !selected)
				{
					operand = selected;
					break;
				}
				body.TryToggleEdge(operand - body.verts.data(), selected - body.verts.data());
				operand = NULL;
				selected = NULL;
				break;
			}
			default:
				std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
				break;
			}

	};

	float3 mousevec_prev;
	float camdist = 2.0f;
	Pose  camera({ 0,0,camdist }, { 0, 0, 0, 1 });
	float boxr = 0.002f;

	while (glwin.WindowUp())
	{
		if (!glwin.MouseState)
		{
			selected = HitCheckPoint(body.verts, camera.position, camera*(glwin.MouseVector*100.0f), boxr);
		}
		else if (selected)
		{
			*selected += (qrot(camera.orientation, glwin.MouseVector) - qrot(camera.orientation, glwin.OldMouseVector))  * length(*selected - camera.position);
			*selected = camera.position + (*selected - camera.position) * powf(1.1f, (float)glwin.mousewheel);
			glwin.mousewheel = 0;
		}
		else // on mouse drag 
		{
			camera.orientation = qmul(camera.orientation,qconj(VirtualTrackBall(float3(0, 0, 2), float3(0, 0, 0), mousevec_prev, glwin.MouseVector)));
			camera.position    = qzdir(camera.orientation)*camdist;
			camdist *= pow(1.1f, (float)glwin.mousewheel);
			glwin.mousewheel = 0;
		}
		mousevec_prev = glwin.MouseVector;

		auto body1 = WingMeshSubDiv(body);
		auto body2 = WingMeshSubDiv(body1);


		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set up the viewport
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.aspect_ratio(), 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glMultMatrixf(camera.inverse().matrix());

		glEnable(GL_DEPTH_TEST);


		glDisable(GL_BLEND);
		for (auto &p : body.verts)
			glcolorbox(float3((&p==operand||&p==selected)?2:1)*boxr, Pose(p, { 0,0,0,1 }));
		if (operand)
		{
			glBegin(GL_LINES);
			glColor3f(1, 1, 1);
			glVertex3fv(*operand);
			glVertex3fv(camera*(glwin.MouseVector*length(*operand - camera.position)));
			glEnd();
		}
		glColor3f(0, 1, 1);
		glEnable(GL_CULL_FACE);
		//glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		wmwire(body );
		glTranslatef(0.3f,0,0 );
		wmwire(body1);
		glTranslatef( 0.3f,0,0);
		wmwire(body2);


		// Restore state
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();  //should be currently in modelview mode
		glPopAttrib();

		glwin.PrintString({ 0, 0 }, "Press ESC to quit.");
		if (selected) glwin.PrintString({ 0, 1 }, "vertex current:  %d", (int)(selected - body.verts.data()));
		if (operand ) glwin.PrintString({ 0, 2 }, "vertex operand:  %d", (int)(operand  - body.verts.data()));
		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

