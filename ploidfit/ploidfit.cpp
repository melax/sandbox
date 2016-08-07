//
//    Paraboloid Fitting of a point cloud
// 
//  first we center and line up the point cloud along its principle axes (higher two eigen values determine x,y plane)
//  then we fit parabaloid in local space using z as the up/height axis
//
//  Note:  when you move a point, it end up uniformly shifting all the points since we constantly re-center on 0,0,0, align as mentioned, and shift z to be tangent to xy plane.  this can seem a bit confusing. 
// 

#include <exception>
#include <iostream>
#include <random>

#include <geometric.h>
#include <glwin.h>
#include <mesh.h>



float4 ParabloidFit(const std::vector<float3> &pts)  // least squares fitting quadratic patch centered at x,y==0,0
{
	float4x4 m;
	float4 b;
	for (auto &p : pts)
	{
		float4 v(p.x*p.x, p.y*p.y, p.x*p.y, 1.0f);
		m += outerprod(v, v);
		b += v*p.z;
	}
	return mul(inverse(m), b);   // returns 'h' best fits   z = h.x * x*x + h.y * y*y + h.z * x*y + h.w  hessian and zoffset 
}


std::default_random_engine random_number_generator;
float3 vrand() // output range is from -1 to 1
{
	auto r = [](){return std::uniform_real<float>(-1.0f, 1.0f)(random_number_generator); };
	return{ r(), r(), r() };
}
float3 vrand(const float3 &scale) { return vrand()*scale; }

std::vector<float3> RandomParabolicCloud(float3 range = float3(1.5f, 1.0f, 0.25f))
{
	float3 k = vrand()*4.0f;   // we randomize the curvatures used to generate the point cloud.
	Pose randpose({ 1, 1, 1 }, normalize(float4(vrand(), 1.0f)));  // randomly pose the initial data to show that we are indeed normalizing a potentially generic point cloud 
	std::vector<float3> points;
	for (float y = -0.5f; y <= 0.5f; y += 0.25f) for (float x = -0.5f; x <= 0.5f; x += 0.25f)
		points.push_back(vrand()*0.1f+ float3(x,y,0.0f));
	std::transform(points.begin(), points.end(), points.begin(), [&randpose, &range,&k](const float3 v)->float3 { return randpose*  (float3(v.xy(), v.x*v.x*k.x + v.y*v.y*k.y +k.z*v.x*v.y)*range); });  // note not associative  mixing pose 'multiplication' with componentwise vector 'multiplication' 
	return points;
}


// some typical opengl drawing support routines
//

void glWirePatch(std::function<float3(float2)> m)
{
	auto f = [](std::function<float3(float2)> m)  
	{
		for (float x = 0; x <= 1.0f; x += 1.0f / 16)
		{
			glBegin(GL_LINE_STRIP);
			for (float y = 0; y <= 1.0f; y += 1.0f / 16)
				glVertex3fv(m({ x,y }));
			glEnd();
		}
	};
	f(m); 
	f([m](float2 c) {return m({ c.y,c.x });});
}

void glHeightField(float2 bmin, float2 bmax, std::function<float(float2)> h, float3 c = { 1,1,1 })
{
	glColor3fv(c);
	glWirePatch( [&h,&bmin,&bmax](float2 c)->float3 {auto p = bmin + (bmax - bmin)*c; return{ p.x,p.y,h(p) };} );
}


void glAxis()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(1.5f);
	glBegin(GL_LINES);
	for (int i : {0, 1, 2})
	{
		float3 v(0, 0, 0);
		v[i] = 0.6f;
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
	glMultMatrixf(p.matrix());
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



float DegToRad(float a) { return a*3.14159f / 180.0f; }

int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try
{ 
	Pose camera = Pose({0,0,0},{sin(DegToRad(60.0f/2.0f)),0,0,cos(DegToRad(60.0f/2.0f))}) * Pose({0,0,6},{0,0,0,1}) ;
	float3 focuspoint(0, 0, 0);
	float4 model_orientation(0, 0, 0, 1);
	float3 *selected = NULL;
	float boxr = 0.025f;
	std::vector<float4> planes = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { -1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, -1, 0 } };
	for (auto &p : planes)
		p.w = -boxr;
	std::vector<float3> points = RandomParabolicCloud();

	GLWin glwin("ParaboloidFit  least Squares quadratic surface fit",800,600);
	glwin.keyboardfunc = [&](int key, int, int)
	{
		if (key == ' ')
			points = RandomParabolicCloud();
	};
	while (glwin.WindowUp())
	{

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
		glHeightField({ -4.0f,-4.0f }, { 4.0f,4.0f }, [](float2){return 0.0f;} , { 0.25f, 0.25f, 0.25f });

		Pose pa;
		float3 va;
		std::tie<Pose,float3>(pa,va) = PrincipalAxes(points);
		auto sd = sqrt(va);  // standard deviation
		std::transform(points.begin(), points.end(), points.begin(), [&pa](const float3 &p) {return pa.inverse()*p; });   // for(auto &p : points) p = pa.inverse() * p; 
		auto h = ParabloidFit(points);
		std::transform(points.begin(), points.end(), points.begin(), [&h](const float3 &p) { return p - float3(0.0f,0.0f,h.w); });   // for (auto &p : points) p.z -= h.w; 
		float2x2 hess = { { h.x,h.z / 2.0f},{h.z / 2.0f,h.y} };
		float ha = Diagonalizer(hess);
		float2x2 hr = { {cosf(ha),sinf(ha)},{-sinf(ha),cosf(ha)} };  // axes of curvature
		auto hd = mul(mul(transpose(hr),hess),hr);   // diagonal entries are the maximum and minimum curvatures
		
		for (int i : {0, 1})  // showing two principal directions of curvature  (different than principal axes of the point cloud)
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glLineWidth(2.0f);
			glBegin(GL_LINE_STRIP);
			float3 c(0.5f,0.5f,0.5f);
			c[i] = 1.0f;
			glColor3fv(c);
			for (float d = -sd.x*2.5f; d <= sd.x*2.5f; d += sd.x / 32)
				glVertex3fv(float3(hr[i]*d, hd[i][i] * d*d));
			glEnd();
			glPopAttrib();
		}

		glHeightField( -sd.xy()*2.0f, sd.xy()*2.0f, [&hess](float2 p)->float{return dot(p,mul(hess,p));} );   // draws a quad patch using the hessian matrix

		glBegin(GL_LINES);  // draw z-aligned disparity between points and surface
		glColor3fv({ 0.1f,0.4f,0.7f });
		for (auto p : points)
			glVertex3fv(p), glVertex3fv({ p.x,p.y,dot(p.xy(),mul(hess,p.xy())) });
		glEnd();


		glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glEnable(GL_COLOR_MATERIAL);
		for (auto &p : points)
			glcolorbox(float3(selected == &p ? boxr*1.5f : boxr), { p,{ 0,0,0,1 } });

		glPopMatrix();  //should be currently in modelview mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopAttrib();// Restore state

		glwin.PrintString({ 0, 0 }, "Space for new pointcloud.");
		if(selected)
			glwin.PrintString({ 0,1 }, "%s: %d", glwin.MouseState ? "moving" : "selected",  selected - points.data());
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
