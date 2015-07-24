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



std::pair<Pose, float3> PrincipalAxes(const std::vector<float3> &points)  // returns principal axes as a pose and population's variance along pose's local x,y,z
{
	float3   com(0, 0, 0);
	float3x3 cov;
	for (auto p : points)
		com += p;
	com /= (float)points.size();
	for (auto p : points)
		cov += outerprod(p - com, p - com);
	cov /= (float)points.size();
	auto q = Diagonalizer(cov);
	return std::make_pair<Pose, float3>({ com, q }, Diagonal(mul(transpose(qgetmatrix(q)), cov, qgetmatrix(q))));
}


std::default_random_engine random_number_generator;
float3 vrand() // output range is from -1 to 1
{
	auto r = [](){return std::uniform_real<float>(-1.0f, 1.0f)(random_number_generator); };
	return{ r(), r(), r() };
}
float3 vrand(const float3 &scale) { return cmul(scale, vrand()); }
float3 vsqrt(const float3 &v)     { return{ sqrtf(v.x), sqrtf(v.y), sqrtf(v.z) }; }

std::vector<float3> RandomPointCloud(float3 range = float3(1.0f, 0.5f, 0.25f), int n = 30)
{
	Pose randpose({ 1, 1, 1 }, normalize(float4(vrand(), 1.0f)));
	std::vector<float3> points(n);
	std::transform(points.begin(), points.end(), points.begin(), [&randpose, &range](const float3)->float3 {return randpose *cmul(range, vrand()); });
	return points;
}


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
void glAxis(const Pose &pose)
{
	glPushMatrix();
	glMultMatrixf(pose.Matrix());
	glAxis();
	glPopMatrix();
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
		w[i] = (m) ? -1.0f : 1.0f;
		float3 a((float)m, (float)m, (float)m);
		a[i] = 1 - a[i];
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
		float3 n = cdiv(qzdir(v.orientation), r); 
		float3 u = cdiv(qxdir(v.orientation), r);  
		return{ cmul(v.position, r), quatfrommat({ u, cross(n, u), n }), v.texcoord };
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
inline void glellipsoid(const float3 &r,const Pose &pose) { glPushMatrix(); glMultMatrixf(pose.Matrix()); glellipsoid(r); glPopMatrix(); }



int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try
{
	std::cout << "TestCov\n";
	Pose camera = { { 0, 0, 6 }, { 0, 0, 0, 1 } };
	float3 focuspoint(0, 0, 0);
	float3 mousevec_prev;
	float4 model_orientation(0, 0, 0, 1);
	float dt = 0.01f, t = 0;
	float3 *selected = NULL;
	bool moving_enabled = 0;  // to move vertices with Left mouse drag
	float boxr = 0.025f;
	std::vector<float4> planes = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { -1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, -1, 0 } };
	for (auto &p : planes)
		p.w = -boxr;
	std::vector<float3> points = RandomPointCloud();

	GLWin glwin("Point Cloud Covariance");
	glwin.keyboardfunc = [&](int key, int, int)
	{
		show_ellipsoid_normals = key == 'n' != show_ellipsoid_normals;
		moving_enabled = key == 'm' != moving_enabled;  // user vertex dragging
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
			for (float3 &p : points) if (moving_enabled)
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
				camera.orientation = qmul(camera.orientation, qconj(VirtualTrackBall(float3(0, 0, 1), float3(0, 0, 0), mousevec_prev, glwin.MouseVector))); // equation is non-typical we are orbiting the camera, not rotating the object
			else
			{
				*selected += (qrot(camera.orientation, glwin.MouseVector) - qrot(camera.orientation, mousevec_prev))  * magnitude(*selected-camera.position);
				*selected = camera.position + (*selected - camera.position) * powf(1.1f, (float)glwin.mousewheel);
				glwin.mousewheel = 0;
			}
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

		Pose pa;
		float3 va;
		std::tie<Pose,float3>(pa,va) = PrincipalAxes(points);
		focuspoint = pa.position;

		auto s2 = vsqrt(va)*2.0f;  // 2 * standard deviation
		glellipsoid(s2, pa);

		glPushMatrix(); glMultMatrixf(pa.Matrix()); 
		glScalef(s2.x, s2.y, s2.z);
		glAxis();
		glScalef(-1, -1, -1);
		glAxis();
		glPopMatrix();

		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glEnable(GL_COLOR_MATERIAL);
		for(auto p:points)
			glcolorbox(boxr, { p, pa.orientation });

		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopAttrib();// Restore state

		glwin.PrintString({ 0, 0 }, "ESC to quit.  Space for new pointcloud.");
		glwin.PrintString({ 0, 1 },"vertex [m]otion: %s", moving_enabled ? "(enabled) select drag with mouse" : "(disabled)");
		if(selected)
			glwin.PrintString({ 0, 2 }, "%s: %d", glwin.MouseState ? "moving" : "selected",  selected - points.data());
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
