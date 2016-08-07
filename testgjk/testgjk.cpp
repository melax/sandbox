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
#include "linalg.h"   
using namespace linalg::aliases;

#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "hull.h"
#include "gjk.h"



std::vector<float3> g_verts;
std::vector<float3> a_verts;
std::vector<int3>   a_tris;
std::vector<float3> b_verts;
std::vector<int3>   b_tris;
int g_vcount = 10;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }
int max_element(const int3& v) { return std::max(std::max(v.x, v.y),v.z); }

void ReInit()
{
	g_verts.resize(0);
	for (int s = 0; s < 2; s++)
	{
		auto position = (vrand() - float3(0.5f, 0.5f, 0.5f))*2.0f;
		for (int i = 0; i < g_vcount; i++)
			g_verts.push_back(position + vrand() - float3(0.5f, 0.5f, 0.5f));
	}
	float3 com;
	for (auto v : g_verts)
		com += v;
	com *= (1.0f / g_verts.size());
	for (auto &v : g_verts)
		v -= com ;
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

void ReGen()
{
	a_verts.resize(0);
	b_verts.resize(0);
	for (int i = 0; i < g_vcount; i++)
	{
		a_verts.push_back(g_verts[i]);
		b_verts.push_back(g_verts[i + g_vcount]);
	}
	a_tris = calchull(a_verts, g_vcount);
	b_tris = calchull(b_verts, g_vcount);
}






// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "TestGJK\n";
	Pose camera;
	float3 *selected = NULL;
	float3 mousevec_prev;
	ReInit();
	float camdist = 2.0f;
	GLWin glwin("TestGJK sample");
	bool emode = false,wire=false,minkowdraw=false;
	glwin.keyboardfunc = [&](unsigned char key, int x, int y) -> void
	{
		switch (std::tolower(key))
		{
		case ' ':
			ReInit();
			selected = NULL;
			break;
		case 27:   // ESC
		case 'q':
			exit(0);
			break;
		case 'w':
			wire = !wire;
			break;
		case 'e':
			emode = !emode;
			break;
		case 'd':
			minkowdraw = !minkowdraw;
			break;
		default:
			std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
			break;
		}
	};
	while (glwin.WindowUp())
	{
		ReGen();

		auto hitinfo = Separated(a_verts, b_verts, 1);

		if (!emode)
			selected = NULL;
		else if (!glwin.MouseState)
		{
			float3 v = qrot(camera.orientation,glwin.MouseVector);  // assumes camera at 0,0,0 looking down -z axis
			float3 p = camera.position;
			selected = &(*std::max_element(g_verts.begin(), g_verts.end(), [&v,&p](const float3&a, const float3&b)->bool {return dot(v, normalize(a - p)) < dot(v, normalize(b-p)); }));
		}

		if (glwin.MouseState)
		{
			if (!emode || !selected)
			{
				camera.orientation = qmul(camera.orientation, qconj(VirtualTrackBall(float3(0, 0, 1), float3(0, 0, 0), mousevec_prev, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
			}
			else
			{
				*selected += (qrot(camera.orientation, glwin.MouseVector) - qrot(camera.orientation, mousevec_prev))  * length(*selected - camera.position);
				*selected = camera.position + (*selected - camera.position) * powf(1.1f, (float)glwin.mousewheel);
				glwin.mousewheel = 0;
			}
		}

		camdist *= powf(1.1f, (float)glwin.mousewheel);
		camera.position = qzdir(camera.orientation) * camdist;
		mousevec_prev = glwin.MouseVector;


		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set up the viewport
		glViewport(0, 0, glwin.res.x,glwin.res.y);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Set up matrices
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.aspect_ratio(), 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glMultMatrixf(camera.inverse().matrix());

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
		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(0, 1.0f, 1.0f);
		if (selected)
			glVertex3fv(*selected);
		glEnd();
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

		if (minkowdraw)
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_CULL_FACE);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
			std::vector<float3> verts;
			for (auto a : a_verts) for (auto b : b_verts)
			{
				verts.push_back(a - b);
			}
			auto tris = calchull(verts, (int)verts.size());
			glPolygonMode(GL_FRONT_AND_BACK, wire ? GL_LINE : GL_FILL);
			gldraw(verts, tris);
			glPopAttrib();

			glDisable(GL_LIGHTING);  // draw origin;
			glBegin(GL_LINES);
			for (int i = 0; i < 3; i++)
			{
				float3 v(0, 0, 0);
				v[i] = 1.0f;
				glColor3fv(v);
				glVertex3fv(-v); glVertex3fv(v);
			}
			glColor3f(1, 1, 1);
			glVertex3fv({ 0, 0, 0 });
			glVertex3fv(hitinfo.normal * hitinfo.separation * 2.0f);
			glEnd();

			glEnable(GL_BLEND);
			glDisable(GL_LIGHTING);
			glBegin(GL_QUADS);
			auto q = quat_from_to(float3(0, 0, 1), hitinfo.normal);
			glColor4f(((hitinfo) ? 0.6f : 0), 0.0f, 1.0f, 0.50);
			glVertex3fv(hitinfo.normal*hitinfo.separation + qxdir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation + qydir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation - qxdir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation - qydir(q));
			glColor4f(((hitinfo) ? 0.6f: 0), 1.0f, 0.0f, 0.50);
			glVertex3fv(hitinfo.normal*hitinfo.separation - qydir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation - qxdir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation + qydir(q));
			glVertex3fv(hitinfo.normal*hitinfo.separation + qxdir(q));
			glEnd();


		}
		else   // draw the two convex hulls and separation plane
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glPolygonMode(GL_FRONT_AND_BACK, wire?GL_LINE: GL_FILL);
			gldraw(a_verts, a_tris);
			gldraw(b_verts, b_tris);
			glPopAttrib();

			glEnable(GL_BLEND);
			glDisable(GL_LIGHTING);
			glBegin(GL_QUADS);
			auto q = quat_from_to(float3(0, 0, 1), hitinfo.normal);
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

		}



		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString({ 3, 1 }, "(q) to quit.     Spacebar new pointclouds.");
		glwin.PrintString({ 3, 2 }, "(w)ireframe %s    separation %5.3f  %5.3f",wire?"on":"off", hitinfo.separation,hitinfo.dist);
		glwin.PrintString({ 3, 3 }, "(e)dit mode: %s %d", emode ? "geo edit.  current vert:" : "camera navigation.        ", (selected)? selected-&g_verts[0] : -1);
		glwin.PrintString({ 3, 4 }, "(d)rawing: %s",minkowdraw?"full minkowski surface":"convex colliders");
		glwin.SwapBuffers();
	}


	std::cout << "\n";
	return 0;
}

