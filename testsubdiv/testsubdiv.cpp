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

int main(int argc, char *argv[])
{
	std::cout << "Test...\n";
	auto body = WingMeshLoad("EntireBody.obj");
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
	float camdist = 2.0f;
	Pose  camera({ 0,0,camdist }, { 0, 0, 0, 1 });
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			camera.orientation = qmul(camera.orientation,qconj(VirtualTrackBall(float3(0, 0, 2), float3(0, 0, 0), mousevec_prev, glwin.MouseVector)));
			camera.position    = qzdir(camera.orientation)*camdist;
		}
		mousevec_prev = glwin.MouseVector;



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

		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

