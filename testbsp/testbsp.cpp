//
// testbsp  by stan melax 2014
//
// a quick sample testing some bsp code. 
// perhaps even educational for someone.
// 
// 

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

// in project properties, add "../include" to the vc++ directories include path
#include "vecmatquat.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "wingmesh.h"    // fixme
#include "bsp.h"

void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }


std::vector<Face*> WingMeshToFaces(const WingMesh &m)
{
	std::vector<Face*> faces;
	assert(m.unpacked == 0);
    assert(m.fback.size() == m.faces.size());
	int k=0;
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		Face *face = new Face();
		faces.push_back(face);
		face->plane() = m.faces[i];
		//extern void texplanar(Face *face);
		//texplanar(face);
		face->vertex = m.GenerateFaceVerts(i); //int e0 = m.fback[i],e=e0; do{ face->vertex.push_back(m.verts[m.edges[e].v]);  e = m.edges[e].next;} while (e!=e0);
	}
	return faces;
}

void gldraw(const std::vector<float3> &verts, const std::vector<int3> &tris)
{
	glBegin(GL_TRIANGLES);
	glColor4f(1, 1, 1, 0.25f);
	for (auto t : tris)
	{
		auto n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		glNormal3fv(n); auto vn = vabs(n);
		int k = argmax(&vn.x, 3);
		for (int j = 0; j < 3; j++)
		{
			const auto &v = verts[t[j]];
			glTexCoord2f(v[(k + 1) % 3], v[(k + 2) % 3]);
			glVertex3fv(v);
		}
	}
	glEnd();
}
void fdraw(const Face &f)
{
	glBegin(GL_TRIANGLE_FAN);
	glNormal3fv(f.xyz());
	for (auto &v : f.vertex)
		glVertex3fv(v);
	glEnd();
}
void fdraw(const std::vector<Face*> &faces) { for (auto &f : faces) fdraw(*f); }

void wmwire(const WingMesh &m)
{
	glBegin(GL_LINES);
	for (auto e : m.edges)
		glVertex3fv(m.verts[e.v]), glVertex3fv(m.verts[m.edges[e.next].v]);  // note the comma
	glEnd();
}
void wmdraw(const WingMesh &m)
{
	gldraw(m.verts, m.GenerateTris());  // admittedly this generates tri list each time
}


// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "TestBSP\n";
	bool drawcells = 0;  // drawing mode: draw bsp cells or draw brep

	// create a couple boxes and subtract one from the other
	auto ac   = WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 });  // 2x2 cube
	auto bc   = WingMeshCube({ -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 2 });  // smaller box placed overlapping into top face
	auto co   = WingMeshTranslate(WingMeshDual(WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 }),0.85f),float3(0.8f,0,0.45f));  // octahedron offset a bit to the right
	auto af   = WingMeshToFaces(ac);
	auto bf   = WingMeshToFaces(bc);
	auto cf   = WingMeshToFaces(co);
	auto absp = BSPCompile(af, WingMeshCube({ -10.0f, -10.0f, -10.0f }, { 10.0f, 10.0f, 10.0f }));
	auto bbsp = BSPCompile(bf, WingMeshCube({ -10.0f, -10.0f, -10.0f }, { 10.0f, 10.0f, 10.0f }));
	auto cbsp = BSPCompile(cf, WingMeshCube({ -10.0f, -10.0f, -10.0f }, { 10.0f, 10.0f, 10.0f }));
	NegateTree(bbsp);  // turn it inside-out, so later  an intersection will be a subtraction
	NegateTree(cbsp);
	// note that there are quantization rules that the operands should follow to avoid numerical issues
	auto bsp = BSPIntersect(cbsp,BSPIntersect(bbsp, absp)); // after this point, dont use absp or bbsp or cbsp anymore

	BSPMakeBrep(bsp, BSPRipBrep(bsp));           // just regenerate the brep, ensures no T-intersections
	// BSPMakeBrep(bsp, BSPRipBrep(bsp));        // uncomment this line to test the ability to extract mat settings from the previous faces
	std::vector<Face*> faces = BSPRipBrep(bsp);  // brep moved into faces array

	// some extra tests if you are in the mood
	//	delete bsp;  // lets completely start over
	//	bsp = BSPCompile(faces, WingMeshCube({ -10.0f, -10.0f, -10.0f }, { 10.0f, 10.0f, 10.0f }));  // compiles from a single mesh that has more complexity

	GLWin glwin("TestBSP compile and intersect sample");
	glwin.keyboardfunc = [&drawcells](unsigned char key, int , int )
	{
		if (key == 'q' || key == 'Q' || key == 27)  // q or ESC
			exit(0);
		if(key=='d' || key == 'D') 
			drawcells = !drawcells;
	};
	float pitch=0, yaw=0;
	int mouseX=0, mouseY=0;
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			yaw   += (glwin.MouseX - mouseX) * 0.3f;  // poor man's trackball
			pitch += (glwin.MouseY - mouseY) * 0.3f;
		}
		mouseX = glwin.MouseX;
		mouseY = glwin.MouseY;


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height); // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION); 		// Set up matrices
		glPushMatrix(); glLoadIdentity();
		gluPerspective(60, (double)glwin.Width/ glwin.Height, 0.01, 10);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();

		gluLookAt(0, -5, 2, 0, 0, 0, 0, 0, 1);
		glRotatef(pitch, 1, 0, 0);             // poor man's trackball
		glRotatef(yaw, 0, 0, 1);

		glColor3f(0, 1, 0.5f);   // wireframe render the boolean operands
		wmwire(ac);  
		glColor3f(0, 0.5f, 1);
		wmwire(bc);
		glColor3f(0.5f, 0, 1);
		wmwire(co);
		glColor3f(1, 1, 1);

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		if (! drawcells)  // draw the brep
		{
			fdraw(faces);
		}
		else // draw the cells
		{
			std::vector<BSPNode*> stack = { bsp };  // for non-recursive tree traversal
			while (stack.size())
			{
				auto n = stack.back(); stack.pop_back();
				if (!n)
					continue;
				if (n->isleaf == UNDER)
				{
					float3 c(0, 0, 0);
					for (auto &v : n->convex.verts)
						c += v * (1.0f/n->convex.verts.size());  // approx center for convex cell
					glPushMatrix();
					glTranslatef(c.x*0.1f, c.y*0.1f, c.z*0.1f);  // expand outward to slightly separate the cells
					wmdraw(n->convex);
					glPopMatrix();
				}
				stack.push_back(n->under);
				stack.push_back(n->over);
			}
		}

		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString("Press esc to (q)uit.    ", 5, 0);
		char buf[256];
		sprintf_s(buf, "(d)rawmode %s ", drawcells?"cells":"brep" );
		glwin.PrintString(buf, 5, 1);

		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

