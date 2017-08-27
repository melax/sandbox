//
//  
//

#include "../include/glwin.h"
#include "../include/misc_gl.h"
#include "geo6.h"

#include "../include/wingmesh.h"


int main(int argc, char *argv[]) try
{
	GLWin glwin("test_icp");


	Pose camera({ 0,0,3.0f }, { 0,0,0,1 });

	float radius = 0.5f;
	auto box = WingMeshBox(float3(radius));
	Pose bpose = { { 0.02f,0,0 },QuatFromAxisAngle({ 0,1,0 },0.242f) };
	auto bmesh = MeshFlatShadeTex(box.verts, box.GenerateTris());

	bool editmode = false;
	glwin.keyboardfunc = [&](int key, int, int)
	{
		editmode = key == 'e' != editmode;
		std::cout << "editmode " << editmode << std::endl;
	};

	while (glwin.WindowUp())
	{
		if (glwin.MouseState)
		{
			if (editmode)
			{
				auto dq = camera.orientation*quat_from_to(glwin.OldMouseVector, glwin.MouseVector);
				bpose.orientation = normalize(qmul(bpose.orientation, dq));

			}
			else
			{
				auto dmouse = (glwin.mousepos - glwin.mousepos_previous);
				camera = Pose({ 0,0,0 }, QuatFromAxisAngle(qydir(camera.orientation), -dmouse.x*3.14f / 180.0f)) * camera;
				camera = Pose({ 0,0,0 }, QuatFromAxisAngle(qxdir(camera.orientation), -dmouse.y*3.14f / 180.0f)) * camera;
				camera.orientation = normalize(camera.orientation);
			}
		}
		camera.position *= powf(1.1f, (float)glwin.mousewheel);


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(0.1f, 0.1f, 0.15f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



		bmesh.pose = bpose;
		std::vector<float3> points;
		std::vector<icp_correspondence> correspondences;
		for (float2 p(-radius + 0.1f);p.y < radius;p.y += 0.1f) for (p.x = -radius + 0.1f;p.x < radius;p.x += 0.1f) for (int i : {0, 1, 2}) for (int sign : {-1, 1})
		{
			float3 normal(0.0f); normal[i] = 1.0f * sign;
			float3 v; v[i] = radius*sign; v[(i + 1) % 3] = p.x;v[(i + 2) % 3] = p.y;
			correspondences.push_back({ v,bpose.TransformPlane(float4{ normal,-radius }) });
		}

		auto dpose = ICP(correspondences);

		auto drawpoints = ColorVerts(Transform(correspondences, [&](icp_correspondence c) {return dpose*c.point;}), { 1,0,0 });

		std::vector<SegmentPC> lines;
		for (auto c : correspondences)
			lines.push_back(SegmentPC(c.point, c.point + c.plane.xyz()*-dot(c.plane, float4(c.point, 1.0f)), { 0,1,0,1 }));

		{
			render_scene scene(camera, { &bmesh }, lines, drawpoints);
			glPushMatrix();
			glMultMatrixf(dpose.matrix());
			glwirebox(float3(-radius), float3(radius));
			glPopMatrix();
			glColor3f(0, 1.0f,0);
			glwirebox(float3(-radius), float3(radius));
			glColor3f(0, 0, 1.0f);
			glMultMatrixf(bpose.matrix());
			glwirebox(float3(-radius), float3(radius));
		}
		glPopAttrib();
		glwin.SwapBuffers();
	}
	return 0;
}
catch (const char *c)
{
	std::cerr << c << std::endl;
	MessageBox(GetActiveWindow(), c, "FAIL", 0);
}
catch (std::exception e)
{
	std::cerr << e.what() << std::endl;
	MessageBox(GetActiveWindow(), e.what(), "FAIL", 0);
}


