//
// testcloth  -  small sample using point-mass spring system
//
// The spring network is force based and uses implicit integration to handle stiff springs.
// See springnet.h for additional information.
// 
// debug build might be a bit slow.  Use release when possible.
// 

#include <iostream>
#include <algorithm>  // std::max_element
#include <cctype>     // std::tolower

// in project properties, add "../include" to the vc++ directories include path
#include "springnet.h"
#include "glwin.h"  // minimal opengl for windows setup wrapper

bool g_wireframe = 0;


void InitTex()  // create a checkerboard texture 
{
	const int imagedim = 16;
	ubyte3 checker_image[imagedim * imagedim];
	for (int y = 0; y < imagedim; y++)
	 for (int x = 0; x < imagedim; x++)
		checker_image[y * imagedim + x] = ((x + y) % 2) ? ubyte3(0, 255, 255) : ubyte3(0, 127, 127);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // use nearest to see actual uninterpolated image pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imagedim, imagedim, 0, GL_RGB, GL_UNSIGNED_BYTE, checker_image);
}


void OnKeyboard(unsigned char key, int x, int y)
{
	switch (std::tolower(key))
	{
	case ' ':
		break;
	case 27:   // ESC
	case 'q':
		exit(0);
		break;
	case 'w':
		g_wireframe = !g_wireframe;
		break;
	default:
		std::cout << "unassigned key (" << (int)key << "): '" << key << "'\n";
		break;
	}
}



// int main(int argc, char *argv[])
int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,LPSTR lpszCmdLine, int nCmdShow)
{
	SpringNetwork cloth = SpringNetworkCreateRectangular(17, 17, 1.0f);
	for (auto &v : cloth.X) v.z -= 1.75f;  // put cloth object at 0,0,-1.5 region,  view/camera will be at origin.
	cloth.gravity = float3(0, -10.0f, 0);  // normally i perfer z-up for any environment or "world" space.
	cloth.dt = 0.033f;                     // speed it up a bit (regardless of fps, each frame advances cloth 1/30th of a second instead of just 1/60th).
	GLWin glwin("TestCloth sample");
	glwin.keyboardfunc = OnKeyboard;
	InitTex();                             // just initializes a checkerboard default texture
	int selection = 0;                     // index of currently selected point
	while (glwin.WindowUp())
	{
		int point_to_unpin = -1;           // if we temporarily move pin a point, we have to unpin it later after simulation.
		if (!glwin.MouseState)             // on mouse drag  
		{
			float3 v = glwin.MouseVector;  // assumes camera at 0,0,0 looking down -z axis
			selection = std::max_element(cloth.X.begin(), cloth.X.end(), [&v](const float3&a, const float3&b)->bool{return dot(v, normalize(a)) < dot(v, normalize(b)); })- cloth.X.begin();
		}
		else 
		{
			if (!cloth.PointStatusSet(selection, -1))
				 cloth.PointStatusSet((point_to_unpin = selection), 1);
			const float3 &v = glwin.MouseVector;
			cloth.X[selection] = v * (dot(v, cloth.X[selection]) / dot(v, v) *(1.0f + glwin.mousewheel*0.1f));
		}
	
		cloth.Simulate();

		if(point_to_unpin >=0)
			cloth.PointStatusSet(point_to_unpin, 0);

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.Width,glwin.Height);           // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width/ glwin.Height, 0.01, 10);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 0, 0, 0, -1, 0, 1, 0);

		glEnable(GL_DEPTH_TEST);
		glDisable(GL_TEXTURE_2D);
		glPointSize(3);
		glBegin(GL_POINTS);
		for (unsigned int i = 0; i < cloth.X.size(); i++ )
			glColor3f((i==selection)?1.0f:0 , 1, 0.5f), glVertex3fv(cloth.X[i]);
		glEnd();

		if (g_wireframe)
		{
			glBegin(GL_LINES);
			SpringNetworkDrawSprings(&cloth, [](const float3 &a, const float3 &b, const float3 &c){glColor3fv(c); glVertex3fv(a); glVertex3fv(b); });
			glColor3f(1, 0, 0);
			glEnd();
		}
		else
		{
			glEnable(GL_TEXTURE_2D);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1., 1. / (float)0x10000);
			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glColor3f(0.5f, 0.5f, 0.5f);
			glBegin(GL_QUADS);
			for (auto const & q: cloth.quads)
			{
				for (int c = 0; c <4; c++)
					glTexCoord2f(q[c]%17/16.0f,q[c]/17/16.0f),glNormal3fv(cloth.N[q[c]]), glVertex3fv(cloth.X[q[c]]);
			}
			glEnd();
		}

		// Restore state
		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);  

		glwin.PrintString({ 0, 0 }, "Press ESC to quit.  w toggles wireframe. ");
		glwin.PrintString({ 0, 1 }, "Use left mouse motion and wheel to move points.");
		glwin.PrintString({ 0, 2 }, "(w)ireframe %s   vert selected %d", ((g_wireframe) ? "ON " : "OFF"), selection);
#       ifdef _DEBUG
			glwin.PrintString({ 2, -1 }, "Running DEBUG Version.  Performance may be SLoooow.", 2, -1);
#       endif
		glwin.SwapBuffers();
	}
	std::cout << "\n";
	return 0;
}

