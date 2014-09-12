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
void glColor3fv(const float3 &v) { glColor3fv(&v.x); }
void glTexCoord2fv(const float2 &v) { glTexCoord2fv(&v.x); }


void InitTex()  // create a checkerboard texture 
{
	const int imagedim = 16;
	ubyte4 checker_image[imagedim * imagedim];
	for (int y = 0; y < imagedim; y++) for (int x = 0; x < imagedim; x++)
		checker_image[y * imagedim + x] = ((x / 4 + y / 4) % 2) ? ubyte4(191, 255, 255,255) : ubyte4(127, 191, 191,255);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imagedim, imagedim, 0, GL_RGBA, GL_UNSIGNED_BYTE, checker_image);
}

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

void glDraw(BSPNode *n,const float3 &p)
{
	if (n->isleaf)
		return;
	bool above = dot(float4(p, 1), n->plane()) > 0;
	glDraw((above) ? n->under : n->over , p);   // draw back to front
	WingMesh &wm = n->under->convex;
	int fid = std::find_if(wm.faces.begin(), wm.faces.end(), [&n](const float4 &p){return n->plane() == p || n->plane() == -p; }) - wm.faces.begin();
	assert(fid < (int) wm.faces.size());
	auto q = RotationArc(wm.faces[fid].xyz(),float3(0, 0, 1));
	glBegin(GL_POLYGON);
	glColor4f(1, 1, 1, 0.25f);
	glNormal3fv(wm.faces[fid].xyz());
	for (auto &v : wm.GenerateFaceVerts(fid))
	{
		glTexCoord2fv(qrot(q, v).xy()*1.0f);
		glVertex3fv(v);
	}
	glEnd();
	glDraw((above) ? n->over : n->under, p);  // draw back to front
}

// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "TestBSP\n";
	int drawmode = 0;  // drawing mode: draw bsp cells or draw brep

	// create a couple boxes and subtract one from the other
	auto ac   = WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 });  // 2x2 cube
	auto bc   = WingMeshCube({ -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 1.5f });  // smaller box placed overlapping into top face
	auto co   = WingMeshTranslate(WingMeshDual(WingMeshCube({ -1, -1, -1 }, { 1, 1, 1 }),0.85f),float3(0.8f,0,0.45f));  // octahedron offset a bit to the right
	auto af   = WingMeshToFaces(ac);
	auto bf   = WingMeshToFaces(bc);
	auto cf   = WingMeshToFaces(co);
	auto absp = BSPCompile(af, WingMeshCube({ -2.0f, -2.0f, -2.0f }, { 2.0f, 2.0f, 2.0f }));
	auto bbsp = BSPCompile(bf, WingMeshCube({ -2.0f, -2.0f, -2.0f }, { 2.0f, 2.0f, 2.0f }));
	auto cbsp = BSPCompile(cf, WingMeshCube({ -2.0f, -2.0f, -2.0f }, { 2.0f, 2.0f, 2.0f }));
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
	InitTex();
	glwin.keyboardfunc = [&drawmode](unsigned char key, int, int)
	{
		if (key == 'q' || key == 'Q' || key == 27)  // q or ESC
			exit(0);
		if(key=='d' || key == 'D') 
			++drawmode %= 3;
	};
	float4 cameraorientation = normalize(float4(sinf(60.0f*3.14f/180.0f/2),0,0,cosf(60.0f*3.14f/180.0f/2)));
	float  cameradist = 5;
	float3 mousevec_prev;
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			cameraorientation = qmul(cameraorientation, qconj(VirtualTrackBall(float3(0, 0, 2), float3(0, 0, 0), mousevec_prev, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
		}
		mousevec_prev = glwin.MouseVector;
		cameradist *= powf(0.9f, (float)glwin.mousewheel);
		float3 camerapos = qzdir(cameraorientation)*cameradist;
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height); // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_TEXTURE_2D);
		glMatrixMode(GL_PROJECTION); 		// Set up matrices
		glPushMatrix(); glLoadIdentity();
		gluPerspective(60, (double)glwin.Width/ glwin.Height, 0.01, 10);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();
		float4 R[4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
		R[0].xyz() = qxdir(qconj(cameraorientation)); R[1].xyz() = qydir(qconj(cameraorientation)); R[2].xyz() = qzdir(qconj(cameraorientation));
		glMultMatrixf(&R[0].x);  // inverse camera orientation
		glTranslatef(-camerapos.x, -camerapos.y, -camerapos.z);  // inverse camera position

		//gluLookAt(camerapos.x,camerapos.y,camerapos.z , 0, 0, 0, cameraup.x,cameraup.y,cameraup.z);

		glColor3f(0, 1, 0.5f);   // wireframe render the boolean operands
		wmwire(ac);  
		glColor3f(0, 0.5f, 1);
		wmwire(bc);
		glColor3f(0.5f, 0, 1);
		wmwire(co);
		glColor3f(1, 1, 1);

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		if (!drawmode)  // draw the brep
		{
			fdraw(faces);
		}
		else if (drawmode == 2)
		{
			//InitTex();
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glColor4f(1, 1, 1, 0.5f);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDraw(bsp,camerapos);
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
		char * drawmodename[] = { "brep", "cells", "tree" };
		sprintf_s(buf, "(d)rawmode %s ", drawmodename[drawmode]);
		glwin.PrintString(buf, 5, 1);

		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

