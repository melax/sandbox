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

#include "vecmatquat_minimal.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "hull.h"
#include "gjk.h"

void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }

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


int     argmax(const float a[], int n)
{
	int r = 0;
	for (int i = 1; i<n; i++)
	{
		if (a[i]>a[r])
		{
			r = i;
		}
	}
	return r;
}

// still in the process of rearranging basic math and geom routines, putting these here for now...
float3 vabs(const float3 &v)
{
	return float3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}
float3 Orth(const float3& v)
{
	float3 absv = vabs(v);
	float3 u(1, 1, 1);
	u[argmax(&absv[0], 3)] = 0.0f;
	return normalize(cross(u, v));
}
Quaternion RotationArc(float3 v0, float3 v1){
	static Quaternion q(0, 0, 0, 1);
	v0 = normalize(v0);  // Comment these two lines out if you know its not needed.
	v1 = normalize(v1);  // If vector is already unit length then why do it again?
	float3  c = cross(v0, v1);
	float   d = dot(v0, v1);
	if (d <= -1.0f) { float3 a = Orth(v0); return Quaternion(a.x, a.y, a.z, 0); } // 180 about any orthogonal axis axis
	float   s = sqrtf((1 + d) * 2);
	q.x = c.x / s;
	q.y = c.y / s;
	q.z = c.z / s;
	q.w = s / 2.0f;
	return q;
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

		Contact hitinfo;
//		auto supportfunc = [](std::vector<float3> &c){  return[&c](const float3& d) ->float3 { return c[maxdir(c.data(), c.size(), d)]; }; };
		int sep = Separated([](const float3& dir) { return a_verts[maxdir(a_verts.data(), a_verts.size(), dir)]; },  
			[](const float3& dir) { return b_verts[maxdir(b_verts.data(), b_verts.size(), dir)]; },
			hitinfo, 1);
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
		glColor4f(0.0f, 0.0f, 1.0f, 0.50);
		glVertex3fv(hitinfo.impact + qxdir(q));
		glVertex3fv(hitinfo.impact + qydir(q));
		glVertex3fv(hitinfo.impact - qxdir(q));
		glVertex3fv(hitinfo.impact - qydir(q));
		glColor4f(0.0f, 1.0f, 0.0f, 0.50);
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

		glwin.PrintString("Press q to quit.     Spacebar new pointcloud.", 5, 1);
		//glwin.PrintString("Keys w,s increase/decrease vlimit and recalc. ", 5, 2);
		//char buf[256];
		//sprintf_s(buf, "vlimit %d tris %d  y,p= %f,%f", g_vlimit, g_tris.size(), g_yaw, g_pitch);
		//glwin.PrintString(buf, 5, 3);

		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

