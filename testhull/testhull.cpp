
#include <stdlib.h>
#include <iostream>
#include <cctype>     // std::tolower
#include <algorithm>
#include <vector>
// in project properties, add "../include" to the vc++ directories include path

#include "vecmatquat.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "hull.h"


std::vector<float3> g_verts;
std::vector<int3> g_tris;
int g_vlimit = 64;
int g_cloudsize = 20;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }
int max_element(const int3& v) { return std::max(std::max(v.x, v.y),v.z); }

void Init()  // creates a random point cloud and generates initial hull for it.
{
	g_vlimit = 64;
	g_verts.resize(0);
	for (int i = 0; i < g_cloudsize; i++)
		g_verts.push_back(vrand() - float3(0.5f, 0.5f, 0.5f));
	g_tris = ::calchull(g_verts, g_vlimit);
	g_vlimit = 0;
	for (auto t : g_tris)
		g_vlimit = std::max(g_vlimit, 1+ max_element(t));
}


void OnKeyboard(unsigned char key, int x, int y)
{
	switch (std::tolower(key))
	{
	case ' ':
		Init();
		break;
	case 27:   // ESC
	case 'q':
		exit(0);
		break;
	case 'w':
	case 's':
		g_vlimit+=(key=='s')?-1:1;  g_vlimit = std::max(4, g_vlimit);  // same point cloud, just change the max allowable number of verts to use
		g_tris = ::calchull(g_verts, g_vlimit);
		break;
	default:
		std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
		break;
	}
}


void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }



int main(int argc, char *argv[])
{
	std::cout << "TestMath\n";

	Init();

	GLWin glwin("TestHull sample");
	glwin.keyboardfunc = OnKeyboard;
	float3 mousevec_prev;
	float4 model_orientation(0, 0, 0, 1);
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			model_orientation = qmul(VirtualTrackBall(float3(0, 0, 2), float3(0,0,0), mousevec_prev, glwin.MouseVector), model_orientation);
		}
		mousevec_prev = glwin.MouseVector;

		if (glwin.mousewheel)  // lets also support mouse wheel to increase/decrease number of points used to generate hull
		{
			g_vlimit += glwin.mousewheel;  g_vlimit = std::max(4, g_vlimit);
			g_tris = ::calchull(g_verts, g_vlimit);
		}


		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set up the viewport
		glViewport(0, 0, glwin.Width,glwin.Height);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width/ glwin.Height, 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 2, 0, 0, 0, 0, 1, 0);

		float4 R[4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
		R[0].xyz() = qxdir(model_orientation); R[1].xyz() = qydir(model_orientation); R[2].xyz() = qzdir(model_orientation);
		glMultMatrixf(&R[0].x);

		glEnable(GL_DEPTH_TEST);

		glDisable(GL_BLEND);
		glPointSize(4);
		glBegin(GL_POINTS);
		glColor3f(0, 1, 0);
		for (auto &v : g_verts)
		{
			glColor3fv((&v - g_verts.data() < g_vlimit) ? float3(0.75, 1, 0) : float3(0.75, 0, 0));
			glVertex3fv(v);
		}
		glEnd();

		glEnable(GL_CULL_FACE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glBegin(GL_TRIANGLES);
		glColor4f(1, 1, 1, 0.25f);
		for (auto t : g_tris)
		{
			glNormal3fv(TriNormal(g_verts[t[0]], g_verts[t[1]], g_verts[t[2]]));
			for (int j = 0; j < 3; j++)
				glVertex3fv(g_verts[t[j]]);
		}
		glEnd();


		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString({ 5, 0 },"Press q to quit.     Spacebar new pointcloud.");
		glwin.PrintString({ 5, 1 }, "w,s or mwheel to increase/decrease vlimit. ");
		glwin.PrintString({ 5, 2 }, "vlimit %d tris %d", g_vlimit, g_tris.size());

		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

