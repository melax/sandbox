//
// a quick sample testing the gjk convex-convex separation 
//
// 
// 

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cctype>  // std::tolower
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf
#include <vector>

// in project properties, add "../include" to the vc++ directories include path
#include "vecmatquat.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "hull.h"
#include "gjk.h"



float g_pitch, g_yaw;
int g_mouseX, g_mouseY;

std::vector<float3> a_verts;
std::vector<int3> a_tris;
std::vector<float3> b_verts;
std::vector<int3> b_tris;
int g_vcount = 10;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }
int max_element(const int3& v) { return std::max(std::max(v.x, v.y),v.z); }

std::pair<std::vector<float3>, std::vector<int3>> RandConvex()
{
	std::vector<float3> verts;
	auto position = (vrand() - float3(0.5f, 0.5f, 0.5f))*2.0f;
	for (int i = 0; i < g_vcount; i++)
		verts.push_back(position+  vrand() - float3(0.5f, 0.5f, 0.5f));
	auto tris = calchull(verts, g_vcount);
	return std::make_pair(verts, tris);
}
void gldraw(std::vector<float3> &verts, std::vector<int3> &tris)
{
	glBegin(GL_TRIANGLES);
	glColor4f(1, 1, 1, 0.25f);
	for (auto t : tris)
	{
		glNormal3fv(TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]));
		for (int j = 0; j < 3; j++)
			glVertex3fv(verts[t[j]]);
	}
	glEnd();
}

void Init()
{
	std::tie(a_verts, a_tris) = RandConvex();
	std::tie(b_verts, b_tris) = RandConvex();
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
		break;
	default:
		std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
		break;
	}
}



// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "TestMath\n";

	Init();

	GLWin glwin("TestGJK sample");
	glwin.keyboardfunc = OnKeyboard;
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			g_yaw   += (glwin.MouseX - g_mouseX) * 0.3f;  // poor man's trackball
			g_pitch += (glwin.MouseY - g_mouseY) * 0.3f;
		}
		g_mouseX = glwin.MouseX;
		g_mouseY = glwin.MouseY;


		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set up the viewport
		glViewport(0, 0, glwin.Width,glwin.Height);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Set up matrices
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(60, (double)glwin.Width/ glwin.Height, 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, -2, 1, 0, 0, 0, 0, 0, 1);

		glRotatef(g_pitch, 1, 0, 0);
		glRotatef(g_yaw, 0, 0, 1);

		glEnable(GL_DEPTH_TEST);
		//glEnable(GL_TEXTURE_2D);

		glDisable(GL_BLEND);
		glPointSize(3);
		glBegin(GL_POINTS);
		glColor3f(0, 1, 0.5f);
		for (auto &v : a_verts)
			glVertex3fv(v);
		glColor3f(0, 0.5f, 1.0f);
		for (auto &v : b_verts)
			glVertex3fv(v);
		glEnd();


		auto hitinfo = Separated(a_verts, b_verts, 1);

		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(1, 0.5f, 0.5f);
		glVertex3fv(hitinfo.p0w);
		glVertex3fv(hitinfo.p1w);
		glColor3f(1, 0, 0);
		for (int i = 0; i < 4;i++)
			hitinfo.v[i];
		glEnd();

		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3fv(hitinfo.p0w);
		glVertex3fv(hitinfo.p1w);
		glEnd();

		glEnable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		gldraw(a_verts, a_tris);
		gldraw(b_verts, b_tris);

		glEnable(GL_BLEND);
		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		auto q = RotationArc(float3(0, 0, 1), hitinfo.normal);
		glColor4f(((hitinfo)?0.6f:0), 0.0f, 1.0f, 0.50);
		glVertex3fv(hitinfo.impact + qxdir(q));
		glVertex3fv(hitinfo.impact + qydir(q));
		glVertex3fv(hitinfo.impact - qxdir(q));
		glVertex3fv(hitinfo.impact - qydir(q));
		glColor4f(((hitinfo)?0.6f:0), 1.0f, 0.0f, 0.50);
		glVertex3fv(hitinfo.impact - qydir(q));
		glVertex3fv(hitinfo.impact - qxdir(q));
		glVertex3fv(hitinfo.impact + qydir(q));
		glVertex3fv(hitinfo.impact + qxdir(q));
		glEnd();

		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString({ 3, 1 },"Press q to quit.     Spacebar new pointcloud.");
		glwin.PrintString({ 3, 2 }, "separation %5.2f", hitinfo.separation);

		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

