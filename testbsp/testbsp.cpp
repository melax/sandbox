//
// testbsp  by stan melax 2014
//
// a quick sample testing some bsp code. 
// perhaps even educational for someone.
// Shows how to CSG a couple of objects
// via merging spatial structures.
// Use mouse drag and mouse wheel to 
// orbit camera or move boolean operands.
// 

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

// in project properties, add "../include" to the vc++ directories include path
#include "vecmatquat.h"   
#include "glwin.h"         // minimal opengl for windows setup wrapper
#include "wingmesh.h"    
#include "bsp.h"

void glNormal3fv(const float3 &v)     { glNormal3fv(&v.x); }
void glVertex3fv(const float3 &v)     { glVertex3fv(&v.x); }
void glColor3fv (const float3 &v)     { glColor3fv(&v.x);  }
void glTexCoord2fv(const float2 &v)   { glTexCoord2fv(&v.x); }
void glTranslatefv(const float3 &v)   { glTranslatef(v.x, v.y, v.z); }
void glMultMatrixf(const float4x4 &m) { glMultMatrixf(&m.x.x); }

void InitTex()  // create a checkerboard texture 
{
	const int imagedim = 16;
	ubyte4 checker_image[imagedim * imagedim];
	for (int y = 0; y < imagedim; y++) for (int x = 0; x < imagedim; x++)
		checker_image[y * imagedim + x] = ((x / 4 + y / 4) % 2) ? ubyte4(255, 255, 255,255) : ubyte4(191, 191, 191,255);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imagedim, imagedim, 0, GL_RGBA, GL_UNSIGNED_BYTE, checker_image);
}

std::vector<Face> WingMeshToFaces(const WingMesh &m)
{
	std::vector<Face> faces;
	assert(m.unpacked == 0);
    assert(m.fback.size() == m.faces.size());
	int k=0;
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		Face face;
		face.plane() = m.faces[i];
		//extern void texplanar(Face *face);
		//texplanar(face);
		face.vertex = m.GenerateFaceVerts(i); //int e0 = m.fback[i],e=e0; do{ face->vertex.push_back(m.verts[m.edges[e].v]);  e = m.edges[e].next;} while (e!=e0);
		faces.push_back(face);
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
	glBegin(GL_POLYGON);
	glNormal3fv(f.xyz());
	for (auto &v : f.vertex)
		glVertex3fv(v);
	glEnd();
}
void fdraw(const std::vector<Face> &faces) { for (auto &f : faces) fdraw(f); }

void DrawWireframe(const WingMesh &m)
{
	glBegin(GL_LINES);
	for (auto e : m.edges)
		glVertex3fv(m.verts[e.v]), glVertex3fv(m.verts[m.edges[e.next].v]);  // note the comma
	glEnd();
}



void glDraw(BSPNode *root, const float3 &camera_position)
{
	const float3 rainbow[] = { { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 1, 1 }, { 0, 0, 1 }, { 1, 0, 1 } };
	for (auto n : treebacktofront(root, camera_position))
	{
		if (n->isleaf)
			continue;
		WingMesh &wm = n->under->convex;
		int fid = std::find_if(wm.faces.begin(), wm.faces.end(), [&n](const float4 &p){return n->plane() == p || n->plane() == -p; }) - wm.faces.begin();
		assert(fid < (int)wm.faces.size());
		auto q = RotationArc(wm.faces[fid].xyz(), float3(0, 0, 1));
		glBegin(GL_POLYGON);
		float3 both[2] = { n->xyz(), -n->xyz() };
		float3 c = float3(0.5f, 0.5f, 0.5f) + rainbow[argmax(&both[0][0],6)] * 0.5f;
		glColor4f(c.x,c.y,c.z, 0.35f);  
		glNormal3fv(wm.faces[fid].xyz());  // should test to see which side camera_position is on 
		for (auto &v : wm.GenerateFaceVerts(fid))
		{
			glTexCoord2fv(qrot(q, v).xy()*1.0f);
			glVertex3fv(v);
		}
		glEnd();
	}
}

float  Round(const float  x , const float p) { return roundf(x / p) * p; }
float3 Round(const float3 &v, const float p) { return float3(v.x/p, v.y/p, v.z/p) * p; }

// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
LPSTR lpszCmdLine, int nCmdShow)
{
	std::cout << "TestBSP\n";
	int drawmode = 0;  // drawing mode: draw bsp cells or draw brep

	// create a couple boxes and subtract one from the other
	float3 bpos = { 0, 0, 0.5f };
	float3 cpos = { 0.8f, 0, 0.45f };   // octahedron offset a bit to the right
	auto ac   = WingMeshBox({ -1, -1, -1 }, { 1, 1, 1 });  // 2x2 cube
	auto bc   = WingMeshBox({ -0.5f, -0.5f, -1.0f }, { 0.5f, 0.5f, 1.0f });  // skinny box placed overlapping into top face, starts at z=0.5
	auto co   = WingMeshDual(WingMeshBox({ -1, -1, -1 }, { 1, 1, 1 }),0.85f);  
	auto af   = WingMeshToFaces(ac);
	auto bf   = WingMeshToFaces(bc);
	auto cf   = WingMeshToFaces(co);
	std::unique_ptr<BSPNode> bsp;
	std::vector<Face> faces;


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
	float3 camerapos;
	int    dragmode = 0;   // for mouse motion.  1: orbit camera   2,3: move corresponding object 
	float  hitdist  = 0;   // if we select an operand this is how far it is from the viewpoint, so we know how much to translate for subsequent lateral mouse movement
	float3 mousevec_prev;  // direction of mouse vector from previous frame
	//float3 impact(0, 0, -100); // for debugging
	while (glwin.WindowUp())
	{
		if (glwin.MouseState)  // on mouse drag 
		{
			if (dragmode == 1)
				cameraorientation = qmul(cameraorientation, qconj(VirtualTrackBall(float3(0, 0, 2), float3(0, 0, 0), mousevec_prev, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
			if (dragmode >= 2)
			{
				float3 &pos = (dragmode == 2) ? bpos : cpos;
				pos += (qrot(cameraorientation, glwin.MouseVector) - qrot(cameraorientation, mousevec_prev))  * hitdist;
				pos += qrot(cameraorientation, glwin.MouseVector) * (float)glwin.mousewheel * 0.1f;
				glwin.mousewheel = 0;
				bsp = NULL; // force regeneration of bsp
			}
			if (!dragmode)  // mouse is down, but we dont know what we are manipulating
			{
				dragmode = 1;
				float3 v0 = camerapos, v1 = camerapos + qrot(cameraorientation, glwin.MouseVector*100.0f);
				if (bsp)
					HitCheck(bsp.get(), 0, v0, v1, &v1);                                        // shorten segment if we hit a face
				auto bhit = ConvexHitCheck(bc.faces, { bpos, { 0, 0, 0, 1 } }, v0, v1);
				v1 = bhit.impact;                                                         // shorten selection ray if necessary so we pick closest
				auto chit = ConvexHitCheck(co.faces, { cpos, { 0, 0, 0, 1 } }, v0, v1);
				hitdist = magnitude( chit.impact - v0 );
				//  impact = chit.impact;  // for debugging
				dragmode = (bhit) ? 2 : dragmode;
				dragmode = (chit) ? 3 : dragmode;   
				if (drawmode == 2)
					dragmode = 1;  // selection and object moving is too confusing when in this mode, so just allow camera navigation
			}
		}
		else // mouse is up
			dragmode = 0;
		mousevec_prev = glwin.MouseVector;
		cameradist *= powf(0.9f, (float)glwin.mousewheel);
		camerapos = qzdir(cameraorientation)*cameradist;

		if (!bsp)
		{
			auto absp = BSPCompile(af, WingMeshCube(2.0f));        // create spatial structures for our operands
			auto bbsp = BSPCompile(bf, WingMeshCube(2.0f));
			auto cbsp = BSPCompile(cf, WingMeshCube(2.0f));
			BSPTranslate(*bbsp, Round(bpos,0.05f));                 // move to current position for this operand
			BSPTranslate(*cbsp, Round(cpos,0.05f));
			NegateTree(*bbsp);                                      // turn it inside-out, so intersection will be a subtraction
			NegateTree(*cbsp);
			// note that there are 
			extern float qsnap; qsnap = 0.05f;                     // Important. quantization rules for operands to avoid numerical issues.
			bsp = BSPIntersect(move(cbsp), BSPIntersect(move(bbsp), move(absp)));    // after this point, dont use absp or bbsp or cbsp anymore
			BSPMakeBrep(bsp.get(), BSPRipBrep(bsp.get()));                     // just regenerate the brep, ensures no T-intersections
			// BSPMakeBrep(bsp, BSPRipBrep(bsp));                  // uncomment this line to test the ability to extract mat settings from the previous faces
			faces = BSPRipBrep(bsp.get());                               // brep moved into faces array

			// some extra tests if you are in the mood
			//	delete bsp;  // lets completely start over
			//	bsp = BSPCompile(faces, WingMeshCube({ -2.0f, -2.0f, -2.0f }, { 2.0f, 2.0f, 2.0f }));  // compiles from a single mesh that has more complexity
		}

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height); // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_TEXTURE_2D);
		glMatrixMode(GL_PROJECTION); 		// Set up matrices
		glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width/ glwin.Height, 0.01, 10);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();
		glMultMatrixf(Pose(camerapos, cameraorientation).Inverse().Matrix());

		//glColor3f(((dragmode>=2)?1.0f:0.0f), 0.5f, 0.5f);   // for debugging the user selection 
		//float3 axes[] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		//glBegin(GL_LINES);
		//for (auto a:axes)
		//	glVertex3fv(impact - a*0.1f), glVertex3fv(impact + a*0.1f); // note the comma

		// wireframe render the boolean operands
		glColor3f(0, 1, 0.5f); DrawWireframe(ac);  
		glColor3f(0, 0.5f, 1); glPushMatrix(); glTranslatefv(Round(bpos,0.05f)); DrawWireframe(bc); glPopMatrix();
		glColor3f(0.5f, 0, 1); glPushMatrix(); glTranslatefv(Round(cpos,0.05f)); DrawWireframe(co); glPopMatrix();
		glColor3f(1, 1, 1);

		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
		if (drawmode==0)  // draw the brep
		{
			fdraw(faces);
		}
		else if (drawmode == 1) // draw the cells
		{
			std::vector<BSPNode*> stack = { bsp.get() };  // for non-recursive tree traversal
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
					//glTranslatef(c.x*0.1f, c.y*0.1f, c.z*0.1f);  // expand outward to slightly separate the cells
					glTranslatefv(c);
					glScalef(0.95f, 0.95f, 0.95f);                 // shrink slightly about each cell center
					glTranslatefv(-c);
					gldraw(n->convex.verts,n->convex.GenerateTris());
					glPopMatrix();
				}
				stack.push_back(n->under.get());
				stack.push_back(n->over.get());
			}
		}
		else if (drawmode == 2)  // draw the tree planes
		{
			//InitTex();
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glColor4f(1, 1, 1, 0.5f);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDraw(bsp.get(), camerapos);
		}

		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString({ 0, 0 },"ESC to (q)uit.  Mouse left or wheel to manipulate");
		char * drawmodename[] = { "brep", "cells", "tree" };
		glwin.PrintString({ 0, 1 }, "(d)rawmode %s ", drawmodename[drawmode]);
#       ifdef _DEBUG
		if (dragmode>=2)
			glwin.PrintString({ 0, -1 },"DEBUG Version.  Perf may be SLOW.");
#       endif

		glwin.SwapBuffers();
	}

	std::cout << "\n";
	return 0;
}

