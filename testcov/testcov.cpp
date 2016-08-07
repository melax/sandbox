//
//    3D Covariance and Principal Axes
//
//  various applications of this.
//  Not shown here, but one practical example of this is whole object tracking of an unknown model based on depth camera data.
// 

#include <exception>
#include <iostream>
#include <random>

#include <geometric.h>
#include <glwin.h>
#include <mesh.h>
#include <misc_gl.h>




std::default_random_engine random_number_generator;
float3 vrand() // output range is from -1 to 1
{
	auto r = [](){return std::uniform_real<float>(-1.0f, 1.0f)(random_number_generator); };
	return{ r(), r(), r() };
}
float3 vrand(const float3 &scale) { return scale* vrand(); }


std::vector<float3> RandomPointCloud(float3 range = float3(1.0f, 0.5f, 0.25f), int n = 30)
{
	Pose randpose({ 1, 1, 1 }, normalize(float4(vrand(), 1.0f)));
	std::vector<float3> points(n);
	std::transform(points.begin(), points.end(), points.begin(), [&randpose, &range](const float3)->float3 {return randpose * (range*vrand()); });   // note this multiplication here not associative since scale 'range' is componentwise float3 instead of a matrix
	return points;
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
void glAxis(const Pose &pose)
{
	glPushMatrix();
	glMultMatrixf(pose.matrix());
	glAxis();
	glPopMatrix();
}

std::vector<int3> gridtriangulation(int2 tess)
{
	std::vector<int3> tris;
	for (int y = 0; y < tess.y - 1; y++) for (int x = 0; x < tess.x - 1; x++)
	{
		tris.push_back({ (y + 1)* tess.x + x + 0, (y + 0)* tess.x + x + 0, (y + 0)* tess.x + x + 1 });
		tris.push_back({ (y + 0)* tess.x + x + 1, (y + 1)* tess.x + x + 1, (y + 1)* tess.x + x + 0 });  // note the {2,0} edge is the one that cuts across the quad
	}
	return tris;
}
Mesh sphere(int2 tess)
{
	std::vector<Vertex> verts;
	for (int y = 0; y < tess.y ; y++) for (int x = 0; x < tess.x; x++)
	{
		float lat = 3.14159f * (y /(tess.y - 1.0f) - 0.5f);
		float lng = 3.14159f * 2.0f * x / (tess.x - 1.0f);
		float3 p(cos(lat)*cos(lng), cos(lat)*sin(lng), sin(lat));
		float3 u(-sin(lng), cos(lng), 0);
		verts.push_back({ p, quatfrommat({ u, cross(p, u), p }), { x / (tess.x - 1.0f), y / (tess.y - 1.0f) } });
	}
	return{ verts, gridtriangulation(tess), Pose(), "", { 1, 1, 1, 1 } };
}

Mesh scale(Mesh m, float3 r)
{
	auto stretch = [&r](Vertex v)->Vertex
	{ 
		float3 n = qzdir(v.orientation)/ r; 
		float3 u = qxdir(v.orientation)/ r;  
		return{ v.position* r, quatfrommat({ u, cross(n, u), n }), v.texcoord };
	};
	std::transform(m.verts.begin(), m.verts.end(), m.verts.begin(), stretch);
	return m;
}
Mesh ellipse(float3 r)
{
	return scale(sphere({ 23, 17 }), r);
}

bool show_ellipsoid_normals = false;  // just to make sure my mesh scale worked ok
inline void glellipsoid(const float3 &r)  // wire mesh version
{
	auto e = ellipse(r);
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for (auto t : e.tris) for (auto i : { 0, 1, 1, 2 })  // just draw first 2 edges since {0,2} is the diagonal that cuts across the quad
		glVertex3fv(e.verts[t[i]].position);
	glColor3f(0.0f, 0.5f, 0.5f);
	for (auto p : e.verts) for (auto n : { 0.0f, 0.02f }) if (show_ellipsoid_normals)  // just for debugging
		glVertex3fv(p.position + qzdir(p.orientation)*n);
	glEnd();
}
inline void glellipsoid(const float3 &r,const Pose &pose) { glPushMatrix(); glMultMatrixf(pose.matrix()); glellipsoid(r); glPopMatrix(); }



int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try
{
	std::cout << "TestCov\n";
	Pose camera = { { 0, 0, 6 }, { 0, 0, 0, 1 } };
	float3 focuspoint(0, 0, 0);
	float3 mousevec_prev;
	float4 model_orientation(0, 0, 0, 1);
	float dt = 0.01f, t = 0;
	float3 *selected = NULL;
	float boxr = 0.025f;
	std::vector<float4> planes = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { -1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, -1, 0 } };
	for (auto &p : planes)
		p.w = -boxr;
	std::vector<float3> points = RandomPointCloud();

	GLWin glwin("Point Cloud Covariance");
	glwin.keyboardfunc = [&](int key, int, int)
	{
		show_ellipsoid_normals = key == 'n' != show_ellipsoid_normals;
		if (key == ' ')
			points = RandomPointCloud();
	};
	while (glwin.WindowUp())
	{
		t = t + dt;  // advance our global time    t is in 0..1
		if (t > 1.0f)
			t = 0.0f;

		// user interaction: 
		float3 ray = qrot(camera.orientation, normalize(glwin.MouseVector));   // for mouse selection
		float3 v1 = camera.position + ray*100.0f;
		if (!glwin.MouseState)  // note that we figure out what is being selected only when the mouse is up
		{
			selected = NULL;
			for (float3 &p : points) 
			{
				if (auto h = ConvexHitCheck(planes, Pose(p, { 0, 0, 0, 1 }), camera.position, v1))
				{
					selected = &p;
					v1 = h.impact;
				}
			}
		}
		else // if (glwin.MouseState)
		{
			if (!selected)
				camera.orientation = qmul(camera.orientation, qconj(VirtualTrackBall(float3(0, 0, 1), float3(0, 0, 0), glwin.OldMouseVector, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
			else
			{
				*selected += (qrot(camera.orientation, glwin.MouseVector) - qrot(camera.orientation, glwin.OldMouseVector))  * length(*selected-camera.position);
				*selected = camera.position + (*selected - camera.position) * powf(1.1f, (float)glwin.mousewheel);
				glwin.mousewheel = 0;
			}
		}
		camera.position = focuspoint + qzdir(camera.orientation)*length(camera.position - focuspoint);
		camera.position -= focuspoint;
		camera.position *= powf(1.1f, (float)glwin.mousewheel);
		camera.position += focuspoint;
		mousevec_prev = glwin.MouseVector;

		// Render the scene
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.res.x, glwin.res.y); // Set up the viewport
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.aspect_ratio(), 0.25, 250);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();
		glMultMatrixf(camera.inverse().matrix());

		glDisable(GL_LIGHTING);
		glAxis();
		glGridxy(4.0f);

		Pose pa;
		float3 va;
		std::tie<Pose,float3>(pa,va) = PrincipalAxes(points);
		focuspoint = pa.position;

		auto s2 = sqrt(va)*2.0f;  // 2 * standard deviation
		glellipsoid(s2, pa);

		glPushMatrix(); glMultMatrixf(pa.matrix()); 
		glScalef(s2.x, s2.y, s2.z);
		glAxis();
		glScalef(-1, -1, -1);
		glAxis();
		glPopMatrix();

		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glEnable(GL_COLOR_MATERIAL);
		for(auto &p:points)
			glcolorbox(float3(selected==&p?boxr*1.5f:boxr), { p, pa.orientation });

		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopAttrib();// Restore state

		glwin.PrintString({ 0, 0 }, "ESC to quit.  Space for new pointcloud.");
		glwin.PrintString({ 0, 1 }, (!selected) ? ((glwin.MouseState)?"rotate cloud":"") : "%s: %d", glwin.MouseState ? "moving" : "selected",  selected - points.data());
		if (show_ellipsoid_normals)
			glwin.PrintString({ 0, 3 }, "[n] to disable useless showing of vertex normals.");
		glwin.SwapBuffers();
	}
	std::cout << "\n";
	return 0;
}
catch (std::exception e)
{
	MessageBoxA(GetActiveWindow(), e.what(), "FAIL", 0);
	return -1;
}
