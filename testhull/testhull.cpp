
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cctype>  // std::tolower
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf
#include <GL/glut.h>
// in project properties, add "../include" to the vc++ directories include path

#include "vecmath.h"
#include "vecmath.cpp"

#include "hull.h"

float g_pitch, g_yaw;
int g_mouseX, g_mouseY;
int g_vlimit = 64;

std::vector<float3> g_verts;
std::vector<int3> g_tris;
int g_vcount = 20;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }
int max_element(const int3& v) { return std::max(std::max(v.x, v.y),v.z); }

void Init()
{
	g_vlimit = 64;
	g_verts.resize(0);
	for (int i = 0; i < g_vcount; i++)
		g_verts.push_back(vrand() - float3(0.5f, 0.5f, 0.5f));
	g_tris = ::calchull(g_verts.data(), g_verts.size(), g_vlimit);
	g_vlimit = 0;
	for (auto t : g_tris)
		g_vlimit = std::max(g_vlimit, 1+ max_element(t));
}

void OnMouse(int button, int state, int x, int y)
{
	g_mouseX = x;
	g_mouseY = y;
}
void OnMotion(int x, int y)
{
	g_yaw += (g_mouseX - x) * 0.3f;
	g_pitch += (y - g_mouseY) * 0.3f;
	g_mouseX = x;
	g_mouseY = y;
}

void OnIdle()
{
	glutPostRedisplay();
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
		g_vlimit+=(key=='s')?-1:1;  g_vlimit = std::max(4, g_vlimit);
		g_tris = ::calchull(g_verts.data(), g_verts.size(), g_vlimit);
		break;
	default:
		std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
		break;
	}
}

inline void DrawString(int x, int y, const char* format, ...)
{
	// Format output string
	va_list args;
	va_start(args, format);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	// Set up a pixel-aligned orthographic coordinate space
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0, -1, +1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();


	glColor3f(1, 1, 1);
	glRasterPos2i(x + 2, y + 12);
	const char *s = buffer;
	while(*s)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *s++);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}
void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }

void OnDisplay()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	// Set up the viewport
	int winWidth = glutGet(GLUT_WINDOW_WIDTH), winHeight = glutGet(GLUT_WINDOW_HEIGHT);
	glViewport(0, 0, winWidth, winHeight);
	glClearColor(0.1f, 0.1f, 0.15f, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set up matrices
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, (double)winWidth / winHeight, 0.01, 10);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, -2, 1, 0, 0, 0, 0, 0, 1);

	glRotatef(g_pitch, 1, 0, 0);
	glRotatef(g_yaw, 0, 0, 1);

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_TEXTURE_2D);

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
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();

	DrawString(10, 10, "Press q to quit.   w,s increase/decrease vlimit and recalc.  Spacebar new pointcloud.");
	DrawString(10, 28, "vlimit %d  tris %d    y,p = %f,%f", g_vlimit,g_tris.size(), g_yaw, g_pitch);

	glutSwapBuffers();
}

int main(int argc, char *argv[])
{
	std::cout << "TestMath\n";

	Init();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(800, 600);
	glutCreateWindow(argc?argv[0]:"TestMath sample");
	glutDisplayFunc(OnDisplay);
	glutKeyboardFunc(OnKeyboard);
	glutMotionFunc(OnMotion);
	glutMouseFunc(OnMouse);
	glutIdleFunc(OnIdle);
	glutMainLoop();

	std::cout << "\n";
	return 0;
}

