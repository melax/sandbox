
/*
 *        Polygon Reduction Demo by Stan Melax (c) 1998
 *  Permission to use any of this code wherever you want is granted..
 *  Although, please do acknowledge authorship if appropriate.
 *
 *  This module contains the window setup code, mouse input, timing
 *  routines, and that sort of stuff.  The interesting modules
 *  to see are bunnygut.cpp and progmesh.cpp.
 *
 *  The windows 95 specific code for this application was taken from
 *  an example of processing mouse events in an OpenGL program using
 *  the Win32 API from the www.opengl.org web site. 
 *  
 *  Under Project->Settings, Link Options, General Category
 *  Add:  
 *        Opengl32.lib glu32.lib winmm.lib 
 *  to the Object/Library Modules
 *  
 *  You will need have OpenGL libs and include files to compile this
 *  Go to the www.opengl.org web site if you need help with this.
 */

// 2014 update, just inlined the needed vector things in the vecmatquat_minimal.h file.
// original code was from 1998 and wasn't using the best conventions.
// For example, quaternions are xyzw, not rxyz now.


#define NOMINMAX
#include <windows.h>	/* must include this before GL/gl.h */
#include <GL/gl.h>		/* OpenGL header file */
#include <GL/glu.h>		/* OpenGL utilities header file */
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <time.h>

#pragma comment(lib,"winmm.lib")  // for the timing functions fps deltat

#include "../include/vecmatquat_minimal.h"    // typical 3D math routines following hlsl style for the most part
//#include "../include/geometric.h"
#include "../include/glwin.h"                 // a minimial opengl on windows wrapper, just a header, no lib/dll.

// Functions and Variables from bunny module
extern void       InitModel();
extern char *     RenderModel();
extern float3     model_position;      // position of bunny 
extern float4 model_orientation;   // orientation of bunny 

// Global Variables
float   DeltaT = 0.1f;
float   FPS;






void CalcFPSDeltaT()
{
	static int timeinit=0;
	static int start,start2,current,last;
	static int frame=0, frame2=0;
	if(!timeinit){
		frame=0;
		start=timeGetTime();
		timeinit=1;
	}
	frame++;
	frame2++;
	current=timeGetTime(); // found in winmm.lib
	double dif=(double)(current-start)/CLOCKS_PER_SEC;
	double rv = (dif)? (double)frame/(double)dif:-1.0;
	if(dif>2.0 && frame >10) {
		start  = start2;
		frame  = frame2;
		start2 = timeGetTime();
		frame2 = 0;
	}		   
	DeltaT = (float)(current-last)/CLOCKS_PER_SEC;
	if(current==last) { 
		DeltaT = 0.1f / CLOCKS_PER_SEC;  // it just cant be 0
	}
	// if(DeltaT>1.0) DeltaT=1.0;
	FPS = (float)rv;
	last = current;
}



int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
	LPSTR lpszCmdLine, int nCmdShow)
{
 
	InitModel();  // initializes some data structures and does progressive mesh polygon reduction algorithm

	GLWin glwin("bunnylod by Stan Melax");
	float3 MouseVectorOld;

    while (glwin.WindowUp()) 
	{

		if(glwin.MouseState) 
			model_orientation=qmul(VirtualTrackBall(float3(0,0,0),model_position,MouseVectorOld,glwin.MouseVector),model_orientation);

		MouseVectorOld = glwin.MouseVector;
		CalcFPSDeltaT();

		// main drawing loop 
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushMatrix();
		glLoadIdentity();
		// camera at default (zero) position and orientation
		char * s=RenderModel();

		glLoadIdentity();
		glColor3f(1,1,0);
		glwin.PrintString(s,0,-2);  // print returned status string from rendermodel()  current vert and tri count
		glwin.PrintString("Demo by Stan Melax (c)1998",5,1);
		glwin.PrintString("Model by Viewpoint Datalabs (c)1996",5,2);
		char buf[1024];buf[0]='\0';
		sprintf_s(buf,"FPS: %5.2f   ",FPS);
		glwin.PrintString(buf,0,-1);

		glPopMatrix();
		glFlush();
		glwin.SwapBuffers();			
    }
	return 0;
}
