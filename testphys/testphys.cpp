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



float g_pitch,  g_yaw;
bool  g_simulate = 0;



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
	gldraw(m.verts, m.GenerateTris());
}

void rbdraw(const RigidBody *rb)
{
	glPushMatrix();
	glMultMatrixf(MatrixFromRotationTranslation(rb->orientation, rb->position));
	for (const auto &s : rb->shapes)
		gldraw(s.verts, s.tris);
	glPopMatrix();
}



Shape AsShape(const WingMesh &m) { return Shape(m.verts, m.GenerateTris()); }


int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,LPSTR lpszCmdLine, int nCmdShow) // int main(int argc, char *argv[])
{
	std::cout << "Test Physics\n";

	std::vector<RigidBody*> rigidbodies;
	rigidbodies.push_back(new RigidBody({ AsShape(WingMeshCube(1.5f)) }, { 0.0f, 0.0f, 5.5f }));
//	rigidbodies.push_back(new RigidBody({ AsShape(WingMeshCube(1)) }, { -1.5f, 0.0f, 1.5f }));
//	rigidbodies.back()->orientation = normalize(float4(0.1f, 0.01f, 0.3f, 1.0f));
//	auto seesaw = new RigidBody({ AsShape(WingMeshBox( { 3, 0.5f, 0.1f })) }, { 0, -2.5, 0.25f });
//	rigidbodies.push_back(seesaw);
//	rigidbodies.push_back( new RigidBody({ AsShape(WingMeshCube(0.25f)) }, seesaw->position_start + float3( 2.5f, 0, 0.4f)));
//	rigidbodies.push_back( new RigidBody({ AsShape(WingMeshCube(0.50f)) }, seesaw->position_start + float3(-2.5f, 0, 5.0f)));
//	rbscalemass(rigidbodies.back(), 4.0f);
//	rigidbodies.push_back(new RigidBody({ AsShape(WingMeshBox({1,0.2f,0.2f})),AsShape(WingMeshBox({0.2f,1,0.2f})),AsShape(WingMeshBox({0.2f,0.2f,1})) }, { -1.5f, 0.5f, 7.5f }));
//	for (float z = 5.5f; z < 14.0f; z += 3.0f)
//		rigidbodies.push_back(new RigidBody({ AsShape(WingMeshCube(0.5f)) }, { 0.0f, 0.0f, z }));
//	for (float z = 15.0f; z < 20.0f; z += 3.0f)
//		rigidbodies.push_back(new RigidBody({ AsShape(WingMeshDual(WingMeshCube(0.5f), 0.65f)) }, { 2.0f, -1.0f, z }));

	WingMesh world_slab = WingMeshBox({ -10, -10, -5 }, { 10, 10, -2 }); // world_geometry



	GLWin glwin("TestPhys sample");
	glwin.ViewAngle = 60.0f;

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
				for (auto &rb : rigidbodies)
				{
					rb->position = rb->position_start;
					//rb->orientation = rb->orientation_start;  // when commented out this provides some variation
					rb->linear_momentum  = float3(0, 0, 0);
					rb->angular_momentum = float3(0, 0, 0);
				}
//				seesaw->orientation = { 0, 0, 0, 1 };
				break;
			default:
				std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
				break;
			}
	};

	InitTex();
	int2 mouseprev;
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			g_yaw   += (glwin.MouseX - mouseprev.x) * 0.3f;  // poor man's trackball
			g_pitch += (glwin.MouseY - mouseprev.y) * 0.3f;
		}
		mouseprev = { glwin.MouseX, glwin.MouseY };

		if (g_simulate)
		{
			std::vector<LimitAngular> angulars;
			std::vector<LimitLinear>  linears;
//			Append(linears , ConstrainPositionNailed(NULL, seesaw->position_start, seesaw, { 0, 0, 0 }));
//			Append(angulars, ConstrainAngularRange(NULL, seesaw, { 0, 0, 0, 1 }, { 0, -20, 0 }, { 0, 20, 0 }));
			PhysicsUpdate(rigidbodies, linears, angulars, { &world_slab.verts });
		}


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height);  // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		// Set up matrices
		glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width/ glwin.Height, 0.01, 50);

		glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
		gluLookAt(0, -8, 5, 0, 0, 0, 0, 0, 1);
		glRotatef(g_pitch, 1, 0, 0);
		glRotatef(g_yaw, 0, 0, 1);

		wmdraw(world_slab);  // world_geometry

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1., 1. / (float)0x10000);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_TEXTURE_2D);
		glColor3f(0.5f, 0.5f, 0.5f);
		for (auto &rb : rigidbodies)
			rbdraw(rb);

		
		glPopAttrib();   // Restore state
		glMatrixMode(GL_PROJECTION); glPopMatrix();
		glMatrixMode(GL_MODELVIEW);  glPopMatrix();  

		glwin.PrintString({ 5, 0 },"ESC/q quits. SPACE to simulate. r to restart");
		glwin.PrintString({ 5, 1 }, "simulation %s", (g_simulate) ? "ON" : "OFF");
		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

