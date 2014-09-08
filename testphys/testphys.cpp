//
// a quick sample testing the physics code 
// create some boxes, let them drop.
// SPACE key starts the simulation.
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
#include "wingmesh.h"
#include "physics.h"

void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }
void glMultMatrix(const float4x4 &m) { glMultMatrixf(&m.x.x); }

float g_pitch, g_yaw;
int g_mouseX, g_mouseY;
bool g_simulate = 0;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }


void InitTex()  // create a checkerboard texture 
{
	const int imagedim = 16;
	ubyte3 checker_image[imagedim * imagedim];
	for (int y = 0; y < imagedim; y++) for (int x = 0; x < imagedim; x++)
		checker_image[y * imagedim + x] = ((x/4 + y/4) % 2) ? ubyte3(191, 255, 255) : ubyte3(63, 127, 127);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1); 
	glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imagedim, imagedim, 0, GL_RGB, GL_UNSIGNED_BYTE, checker_image);
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
	auto tris = WingMeshTris(m);
	gldraw(m.verts, tris);
}

void rbdraw(const RigidBody *rb,int wireframe)
{
	glPushMatrix();
	glMultMatrix(MatrixFromRotationTranslation(rb->orientation, rb->position));
	for (const auto &s :rb->shapes)
		((wireframe)?wmwire:wmdraw)(s->local_geometry);
	glPopMatrix();
}


template<class T> std::vector<T> ArrayOfOne(T t){ std::vector<T> a; a.push_back(t); return a; }

// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "Test Physics\n";

	extern std::vector<RigidBody*> g_rigidbodies;
	auto wma = WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 });
	auto wmb = WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 });
	auto rba = new RigidBody(ArrayOfOne(wma), {  1.5f, 0.0f, 1.5f });
	auto rbb = new RigidBody(ArrayOfOne(wmb), { -1.5f, 0.0f, 1.5f });
	rbb->orientation = normalize(float4(0.1f, 0.01f, 0.3f, 1.0f));
	for (float z = 5.5f; z < 14.0f; z += 3.0f)
		new RigidBody(ArrayOfOne(WingMeshCube({ -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f })), { 0.0f, 0.0f, z });
	for (float z = 15.0f; z < 20.0f; z += 3.0f)
		new RigidBody(ArrayOfOne(WingMeshDual( WingMeshCube({ -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f }),0.65f)), { 2.0f, -1.0f,  z }) ;

	std::vector<WingMesh*> world_geometry;
	WingMesh world_slab = WingMeshCube({ -10, -10, -5 }, { 10, 10, -2 });
	world_geometry.push_back(&world_slab);



	GLWin glwin("TestPhys sample");

	glwin.keyboardfunc = [&](unsigned char key, int x, int y)->void 
	{
			switch (std::tolower(key))
			{
			case ' ':
				g_simulate = !g_simulate;
				break;
			case 'q': case 27:   // ESC
				exit(0); break;  
			case 'r':
				for (auto &rb : g_rigidbodies)
				{
					rb->position = rb->position_start;
					//rb->orientation = rb->orientation_start;  // when commented out this provides some variation
					rb->momentum = float3(0, 0, 0);
					rb->rotation = float3(0, 0, 0);
				}
				break;
			default:
				std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
				break;
			}
	};

	InitTex();

	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			g_yaw   += (glwin.MouseX - g_mouseX) * 0.3f;  // poor man's trackball
			g_pitch += (glwin.MouseY - g_mouseY) * 0.3f;
		}
		g_mouseX = glwin.MouseX;
		g_mouseY = glwin.MouseY;

		extern void PhysicsUpdate(const std::vector<WingMesh*> &wgeom);
		if(g_simulate)
			PhysicsUpdate(world_geometry);


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height);  // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		// Set up matrices
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(60, (double)glwin.Width/ glwin.Height, 0.01, 50);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, -8, 5, 0, 0, 0, 0, 0, 1);
		glRotatef(g_pitch, 1, 0, 0);
		glRotatef(g_yaw, 0, 0, 1);

		// wireframe pass:
		glColor3f(0, 1, 0.5f);
		rbdraw(rba,1);
		glColor3f(0, 0.5f, 1.0f);
		rbdraw(rbb, 1);  // wireframe
		glColor3f(0, 0.5f, 0.5f);
		for (unsigned int i = 2; i < g_rigidbodies.size(); i++)
			rbdraw(g_rigidbodies[i], 1);  // wireframe

		for (auto &wm : world_geometry)
			wmdraw(*wm);

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1., 1. / (float)0x10000);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_TEXTURE_2D);
		glColor3f(0.5f, 0.5f, 0.5f);
		for (auto &rb : g_rigidbodies)
			rbdraw(rb, 0);

		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString("ESC/q quits. SPACE to simulate. r to restart", 5, 0);
		char buf[256];
		sprintf_s(buf, "simulation %s", (g_simulate)?"ON":"OFF");
		glwin.PrintString(buf, 5, 1);

		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

