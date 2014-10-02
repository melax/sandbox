//
// a quick sample testing physics code for tracking
// sample surface points and fit
// 

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cctype>    // std::tolower
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf
#include <vector>
#include <limits>    // std::numeric_limits

// in project properties, add "../include" to the vc++ directories include path
#include "vecmatquat.h"   
#include "glwin.h"  // minimal opengl for windows setup wrapper
#include "hull.h"
#include "gjk.h"
#include "wingmesh.h"
#include "../testphys/physics.h"  // fixme (location)

void glNormal3fv(const float3 &v) { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v) { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v) { glColor3fv(&v.x);  }
void glMultMatrix(const float4x4 &m) { glMultMatrixf(&m.x.x); }

float g_pitch, g_yaw;
bool  g_tracking = 0;

inline float randf(){ return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); }

float3 vrand(){ return {randf(),randf(),randf()}; }


float4 closestplane(const std::vector<float4> &planes, const float3 &v, const float3 &n)
{
	float4 r = { 0, 0, 0, -std::numeric_limits<float>::max() };
	for (auto p : planes)
	{
		if (dot(n, p.xyz()) < 0)  // to filter non-camera facing planes
			continue;
		if (dot(float4(v, 1), p) > dot(float4(v, 1), r))
			r = p;
	}
	return r;
}

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
void wmwire(const WingMesh &m, const Pose &pose) { glPushMatrix(); glMultMatrix(pose.Matrix()); wmwire(m); glPopMatrix(); }

void wmdraw(const WingMesh &m)
{
	gldraw(m.verts, m.GenerateTris());
}
void wmdraw(const WingMesh &m, const Pose &pose) { glPushMatrix(); glMultMatrix(pose.Matrix()); wmdraw(m); glPopMatrix(); }

void rbdraw(const RigidBody *rb)
{
	glPushMatrix();
	glMultMatrix(MatrixFromRotationTranslation(rb->orientation, rb->position));
	for (const auto &s : rb->shapes)
		gldraw(s.verts, s.tris);
	glPopMatrix();
}



Shape AsShape(const WingMesh &m) { return Shape(m.verts, m.GenerateTris()); }


int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,LPSTR lpszCmdLine, int nCmdShow) // int main(int argc, char *argv[])
{
	std::cout << "Test tracking\n";

	WingMesh box = WingMeshBox({ 1, 0.5f, 0.2f });  // our "real world" object used to generate computer vision or depth data input
	Pose boxpose({ 0, 0, 2 }, normalize(float4( 0.2f, 0.3f, 0.4f, 1.0f )));

	RigidBody trackmodel({ AsShape(box) }, { 0, -0.5, 2.25f });  // a tracking model based on the geometry of the real object we are tracking
	std::vector<RigidBody*> rigidbodies = { &trackmodel };

	WingMesh world_slab = WingMeshBox({ -3, -3, -1.25f }, { 3, 3, -1 }); // just some ground plane world_geometry



	GLWin glwin("Tracking single object from depth samples.");
	glwin.ViewAngle = 60.0f;

	glwin.keyboardfunc = [&](unsigned char key, int x, int y)->void 
	{
			switch (std::tolower(key))
			{
			case ' ':
				g_tracking = !g_tracking;
				break;
			case 'q': case 27:   // ESC
				exit(0); break;  
			case 'r':
				for (auto &rb : rigidbodies)
				{
					rb->position = rb->position_start;
					rb->orientation = rb->orientation_start;  
					rb->linear_momentum  = float3(0, 0, 0);
					rb->angular_momentum = float3(0, 0, 0);
				}
				break;
			default:
				std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
				break;
			}
	};

	InitTex();
	int2 mouseprev;
	int frame = 0;
	while (glwin.WindowUp())
	{
		frame++;
		if (glwin.MouseState)  // on mouse drag 
		{
			g_yaw   += (glwin.MouseX - mouseprev.x) * 0.3f;  // poor man's trackball
			g_pitch += (glwin.MouseY - mouseprev.y) * 0.3f;
		}
		mouseprev = { glwin.MouseX, glwin.MouseY };

		boxpose.orientation = normalize(float4(sinf(frame*0.01f),sin(frame*0.035f),sin(frame*0.045f),1.0f));  // animate the source object
		boxpose.position = float3(sinf(frame*0.01f)*0.75f, cosf(frame*0.01f)*0.75f, boxpose.position.z);
	
		std::vector<float3> depthdata; // generated pointcloud 
		for (float y = -1.0f; y <= 1.0f; y += 0.1f) for (float x = -1.0f; x <= 1.0f; x += 0.1f)
		{
			if (auto hit = ConvexHitCheck(box.faces, boxpose, { 0, 0, 0 }, float3(x, y, 1.0f)*5.0f))
				depthdata.push_back(hit.impact);
		}
		std::vector<std::pair<float3,float3>> match;
		if (g_tracking)
		{
			trackmodel.gravscale = 0;
			trackmodel.damping = 1;
			std::vector<LimitAngular> angulars;
			std::vector<LimitLinear>  linears;
			std::vector<float4> planesw;
			for (auto p : box.faces) // should be getting from shape, but oh well
				planesw.push_back(trackmodel.pose().TransformPlane(p));
			for (auto p0 : depthdata)
			{
				auto plane = closestplane(planesw, p0, { 0, 0, 0 } );  // could pass normal direction of -p0 to avoid backside planes
				if(plane.w<0) plane.w = std::min(0.0f, plane.w + 0.2f);  // small hack here (may add jitter)!! add thickness if we are using a backside plane

				auto p1w = p0 - plane.xyz()*dot(plane, float4(p0, 1));               // p1 is on the plane
				match.push_back(std::pair<float3,float3>(p0,p1w));
				linears.push_back(ConstrainAlongDirection(NULL, p0, &trackmodel, trackmodel.pose().Inverse()*p1w, plane.xyz(), -50, 50));
			}
			// trackmodel.angular_momentum = trackmodel.linear_momentum = { 0, 0, 0 };  // damping should be 1 already
			// Append(linears , ConstrainPositionNailed(NULL, seesaw->position_start, seesaw, { 0, 0, 0 }));
			PhysicsUpdate(rigidbodies, linears, angulars, {});
		}
		else
		{
			trackmodel.gravscale = 1;
			trackmodel.damping = 0.1f;
			PhysicsUpdate(rigidbodies, {}, std::vector<LimitAngular>(0), { &world_slab.verts });
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

		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, 0.75f, 0.5f);
		wmwire(box, boxpose);

		glColor3f(1.0f, 1.0f, 0.0f);
		glPointSize(2.0f);
		glBegin(GL_POINTS);
		for (auto p : depthdata)
			glVertex3fv(p);
		glEnd();
		glColor3f(0.7f, 0.0f, 0.0f);
		glPointSize(1.0f);
		glBegin(GL_LINES);
		for (auto p : match)
			glVertex3fv(p.first), glVertex3fv(p.second);  // yeah, no braces {} but note the comma
		glEnd();


		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1., 1. / (float)0x10000);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		wmdraw(world_slab);  // world_geometry
		glEnable(GL_TEXTURE_2D);
		glColor3f(0.5f, 0.5f, 0.5f);
		for (auto &rb : rigidbodies)
			rbdraw(rb);

		
		glPopAttrib();   // Restore state
		glMatrixMode(GL_PROJECTION); glPopMatrix();
		glMatrixMode(GL_MODELVIEW);  glPopMatrix();  

		glwin.PrintString("ESC/q quits. SPACE to toggle tracking.", 5, 0);
		char buf[256];
		sprintf_s(buf, "tracking %s", (g_tracking)?"ON":"OFF");
		glwin.PrintString(buf, 5, 1);

		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

