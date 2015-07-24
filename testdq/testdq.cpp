//
//    Dual Quaternion 
//
//  A minimal demo illustrating screw-motion pose interpolation using dual quaternion math
//  left mouse moves the camera or, if selected, changes the orientation of one of the two endpoints.
// 

#include <exception>
#include <iostream>

#include <geometric.h>
#include <glwin.h>
#include <mesh.h>

// Minimal c++11 implementation of dual quaternion as style agnostic as possible.  
// Assumes there is a float4 struct defined the obvious xyzw way and usual support quat functions.
// This implementation just uses 4x2 matrix where the second ([1]) column is the dual part.
// Feel free to convert to your own design/style preferences and struct names.
// The Pose class is just a position,orientation (vec3,quat) pair defined the obvious way.
// The functions dqmake/dqpose convert Pose to/from dual quat representation.
// 
float4x2 dqmul(const float4x2 &a, float4x2 &b) { return{ qmul(a[0], b[0]), qmul(a[0], b[1]) + qmul(a[1], b[0]) }; }
float4x2 dqnorm(const float4x2 &d) { return  d / magnitude(d[0]); }  // normalize based on non-dual part only
float4x2 dqmake(const Pose &p) { return dqmul(float4x2({ 0, 0, 0, 1 }, float4(p.position / 2.0f, 0)), float4x2(p.orientation, { 0, 0, 0, 0 })); }
Pose     dqpose(const float4x2 &d) { return{ qmul(d[1], qconj(d[0])).xyz()*2.0f, d[0] }; }
float4x2 dqinterp(const float4x2 &d0, const float4x2 &d1, float t) { return dqnorm(d0*(1 - t) + d1*t); }  // normalized lerp
Pose     dqinterp(const Pose &p0,const Pose &p1,float t) { return dqpose(dqinterp(dqmake(p0),dqmake(dot(p0.orientation,p1.orientation)<0?Pose(p1.position,-p1.orientation):p1),t)); }

// some typical opengl drawing support routines
//
void glGridxy(float r, float3 c = { 0, 1, 0 })
{
	glColor3fv(c);
	glBegin(GL_LINES);
	glColor3fv({ 0.25f, 0.25f, 0.25f });
	for (float t = -4; t <= 4; t += 1.0f)
	{
		glVertex3fv(float3(t, -4.0f, 0)*r / 4.0f); glVertex3fv(float3(t, 4.0f, 0)*r / 4.0f);
		glVertex3fv(float3(-4.0f, t, 0)*r / 4.0f); glVertex3fv(float3(4.0f, t, 0)*r / 4.0f);
	}
	glEnd();
}

void glAxis()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	for (int i : {0, 1, 2})
	{ 
		float3 v(0, 0, 0);
		v[i] = 1.0f;
		glColor3fv(v);
		glVertex3fv({ 0, 0, 0 }); 
		glVertex3fv(v);
	}
	glEnd();
	glPopAttrib();
}

void glcolorbox(const float3 &r, const Pose &p)   // p = { { 0, 0, 0 }, { 0, 0, 0, 1 } })
{
	glPushMatrix();
	glMultMatrixf(p.Matrix());
	glBegin(GL_QUADS);
	for (int m : {0, 1}) for (int i : {0, 1, 2})
	{
		int i1 = (i + 1 + m) % 3;
		int i2 = (i + 2 - m) % 3;
		float3 u, v, w;
		u[i1] = r[i1];
		v[i2] = r[i2];
		w[i]  = (m) ? -1.0f : 1.0f;
		float3 a((float)m, (float)m, (float)m);
		a[i] = 1-a[i];
		glColor3fv(a);
		glNormal3fv(w);
		float2 corners[] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };  // ccw order
		for (float2 t : corners)
			glTexCoord2fv(t), glVertex3fv(w*r[i] + u*t.x + v*t.y);
	}
	glEnd();
	glPopMatrix();
}
void glcolorbox(const float &r, const Pose &p) { glcolorbox({ r, r, r }, p); }

int main(int argc, char *argv[]) try
{
	std::cout << "TestDQ\n";
	Pose camera = { { 0, 0, 8 }, { 0, 0, 0, 1 } };
	bool showaxis = true;
	float3 focuspoint(0, 0, 0);
	float3 mousevec_prev;
	float4 model_orientation(0, 0, 0, 1);
	Pose p0 = { { -3, 0, 0 }, { 0, 0, 0, 1 } };
	Pose p1 = { {  3, 0, 0 }, { 0, 0, sqrtf(0.5f),sqrtf(0.5f) } };
	float dt = 0.01f, t = 0;
	Pose *selected = NULL;
	std::vector<float4> planes = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { -1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, -1, 0 } };
	for (auto &p : planes)
		p.w = -0.25f;

	GLWin glwin("Dual Quaternion Pose Interpolation");
	glwin.keyboardfunc = [&](int key, int, int)
	{
		showaxis = key == 'a' != showaxis;
	};
	while (glwin.WindowUp())
	{
		t = t + dt;  // advance our global time    t is in 0..1
		if (t > 1.0f)
			t = 0.0f;

		Pose pt  = dqinterp(p0,p1,t);     // And here we show our dual quaterion usage

		// some extras to help visualize the axis of rotation, not the best math to get the result, but oh well
		float4 aq = qmul(dot(p0.orientation, p1.orientation) < 0 ? -p1.orientation : p1.orientation, qconj(p0.orientation));
		float3 axis = normalize(aq.xyz()*(aq.w < 0 ? -1.0f : 1.0f));  // direction of the axis of rotation
		float3 axisp = cross(axis, p1.position - p0.position) / 2.0f * sqrtf(1/dot(aq.xyz(),aq.xyz())-1);  // origin projected onto the axis of rotation
		// user interaction: 
		float3 ray = qrot(camera.orientation, normalize(glwin.MouseVector));   // for mouse selection
		float3 v1 = camera.position + ray*100.0f;
		if (!glwin.MouseState)  // note that we figure out what is being selected only when the mouse is up
		{
			selected = NULL;
			for (Pose *p : { &p0, &p1 })
			{
				if (auto h = ConvexHitCheck(planes, *p, camera.position, v1))
				{
					selected = p;
					v1 = h.impact;
				}
			}
		}
		else // if (glwin.MouseState)  
		{
			if (selected)
				selected->orientation = qmul(VirtualTrackBall(camera.position, selected->position, qrot(camera.orientation, mousevec_prev), qrot(camera.orientation, glwin.MouseVector)), selected->orientation);
			else
				camera.orientation = qmul(camera.orientation, qconj(VirtualTrackBall(float3(0, 0, 1), float3(0, 0, 0), mousevec_prev, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
		}
		camera.position = focuspoint + qzdir(camera.orientation)*magnitude(camera.position - focuspoint);
		camera.position -= focuspoint;
		camera.position *= powf(1.1f, (float)glwin.mousewheel);
		camera.position += focuspoint;
		mousevec_prev = glwin.MouseVector;

		// Render the scene
		glPushAttrib(GL_ALL_ATTRIB_BITS); 
		glViewport(0, 0, glwin.Width, glwin.Height); // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.Width / glwin.Height, 0.25, 250);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();
		glMultMatrixf(camera.Inverse().Matrix());

		glDisable(GL_LIGHTING); 
		glAxis();  
		glGridxy(4.0f);
		if (showaxis)
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glLineWidth(3.0f);
			glBegin(GL_LINES);
			glColor3f(1, 1, 1);
			for (auto p : { p0.position, p1.position, pt.position ,axisp})
				glVertex3fv(p - axis*0.5f), glVertex3fv(p + axis*0.5f);  // note the comma
			glEnd();
			glPopAttrib();
			glColor3f(1, 1, 0);
			glBegin(GL_LINES);
			glVertex3fv(axisp + axis*dot(axis, p0.position)), glVertex3fv(axisp + axis*dot(axis, p1.position));
			glVertex3fv(axisp + axis*dot(axis, p0.position)), glVertex3fv(p0.position);
			glVertex3fv(axisp + axis*dot(axis, p1.position)), glVertex3fv(p1.position);
			glVertex3fv(axisp + axis*dot(axis, pt.position)), glVertex3fv(pt.position);
			glEnd();
		}

		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glEnable(GL_COLOR_MATERIAL);
		for (auto p : { p0, p1, pt })
			glcolorbox(0.25f, p);

		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopAttrib();// Restore state

		glwin.PrintString({ 0, 0 }, "ESC to quit.");
		glwin.PrintString({ 0, 1 }, "'a' show axis (%s)", showaxis ? "on" : "off");
		glwin.PrintString({ 0, 2 }, "%selected: %s", glwin.MouseState?"S":"s", (selected) ? ((selected==&p0)?"box0":"box1") : "none");
		glwin.SwapBuffers();
	}
	std::cout << "\n";
	return 0;
}
catch (std::exception e)
{
	std::cerr << e.what() << "\n";
}
