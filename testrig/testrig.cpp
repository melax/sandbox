//
// An example of loading and simulating a rigged model consisting of a collection of rigidbodies and some joints connecting them.
// left mouse and mwheel can select and move individual rigidbodies using on-the-fly positional constraints.
//

#include <map>
#include <strstream>
#include <sstream>
#include <iostream>

#include <mesh.h>
#include <minixml.h>
#include <hull.h>
#include <wingmesh.h>  // just so i can quickly make a box
#include <physics.h>
#include <dxwin.h>




struct Joint 
{
	int rbi0 ;
	int rbi1 ;
	float3 p0;
	float3 p1;
	float3 jointlimitmin;
	float3 jointlimitmax;
};

std::vector<float4> Planes(const std::vector<float3> &verts, const std::vector<int3> &tris) { std::vector<float4> planes; for (auto &t : tris) planes.push_back(PolyPlane({verts[t[0]],verts[t[1]],verts[t[2]] }));  return planes; }

int main(int argc, const char *argv[]) try
{
	std::vector<Joint> joints;
	std::vector<RigidBody> rbs;
	std::map<std::string,unsigned int> rbindex;
	auto xml = XMLParseFile("./default_hand.chr");  // replace string with whatever model you want to test.   uses xml variation of John Ratcliff's easy mesh (ezm) file format.
	auto const &skx = xml.child("model").child("skeleton");
	for (auto const &b : skx.children)
	{
		rbindex[b.attribute("name")] = rbs.size();
		auto verts = ArrayImport<float3>(b.child("verts").body);
		auto tris = calchull(verts, verts.size());
		float3 pos = StringTo<float3>(b.attribute("position"));
		int parent = (b.hasAttribute("parent")) ? (int)rbindex[b.attribute("parent")] : -1;
		rbs.push_back(RigidBody({ Shape(verts,tris) }, pos + ((parent>=0)?rbs[parent].position-rbs[parent].com:float3(0,0,0))));
		if (parent>=0)
			joints.push_back({ parent, (int)rbs.size() - 1, pos, float3(0,0,0), StringTo<float3>(b.child("jointlimitmin").body), StringTo<float3>(b.child("jointlimitmax").body) });
	}
	rbscalemass(&rbs[0], 3.0f);
	rbscalemass(&rbs[1], 5.0f);

	DXWin mywin("DX testing articulated rigged model");
	//OVRWin mywin("VR testing articulated rigged model");
	std::vector<Mesh> meshes;
	for (auto &rb : rbs)
	{
		meshes.push_back(MeshSmoothish(rb.shapes[0].verts, rb.shapes[0].tris)); //  1 shape each is known
		rb.damping = 0.8f;
		//rb.gravscale = 0;
	}
	for (auto &joint : joints)
	{
		rbs[joint.rbi0].ignore.push_back(&rbs[joint.rbi1]);
		rbs[joint.rbi1].ignore.push_back(&rbs[joint.rbi0]);
		joint.p0 -= rbs[joint.rbi0].com;
		joint.p1 -= rbs[joint.rbi1].com;
	}
	for (auto &ja : joints) for (auto &jb : joints) if (ja.rbi0 == jb.rbi0 && ja.rbi1 != jb.rbi1)  // ignore siblings 
	{
		rbs[ja.rbi1].ignore.push_back(&rbs[jb.rbi1]);
		rbs[jb.rbi1].ignore.push_back(&rbs[ja.rbi1]);
	}
	for (auto &ja : joints) for (auto &jb : joints) if (ja.rbi1 == jb.rbi0 )  // ignore grandparents 
	{
		rbs[ja.rbi0].ignore.push_back(&rbs[jb.rbi1]);
		rbs[jb.rbi1].ignore.push_back(&rbs[ja.rbi0]);
	}

	std::vector<float3> groundpoints = { { -5.0f, -5.0f, -5.0f }, { 5.0f, -5.0f, -5.0f }, { 5.0f, 10.0f, -5.0f }, { -5.0f, 10.0f, -5.0f }, { -5.0f, -5.0f, -10.0f }, { 5.0f, -5.0f, -10.0f }, { 5.0f, 10.0f, -10.0f }, { -5.0f, 10.0f, -10.0f } };
	Mesh ground = MeshSmoothish(groundpoints, { { 0, 1, 2 }, { 2, 3,0 } });
	ground.hack = { 1, 1, 0 ,1};
	WingMesh cube_wm = WingMeshCube(0.025f);
	auto mesh_cube = MeshFlatShadeTex(cube_wm.verts, WingMeshTris(cube_wm));   
	mesh_cube.hack = { 0, 1, 0, 1 };

	Pose camera = { { 0, -10, 0 }, normalize(float4(1, 0, 0, 1)) };
	RigidBody *selected = NULL;
	float3 spoint=camera * float3(0,0,-10);
	float3 rbpoint;
	
	struct Pin{ float3 w; RigidBody* rb; float3 p; };
	std::vector<Pin> pins;   
	
	mywin.keyboardfunc = [&](int key, int, int)
	{
		if (key == 'g') for (auto &rb : rbs) rb.gravscale = 1.0f - rb.gravscale;
		if (key == 'p' && selected)
			Append<Pin>(pins, { spoint, selected, rbpoint });
	};

	while (mywin.WindowUp())
	{
		float3 ray = qrot(camera.orientation, normalize(mywin.MouseVector));
		if (!selected)
		{
			for (auto &rb : rbs)
			{
				float3 v1 = camera.position + ray*100.0f;
				if (auto h=ConvexHitCheck(Planes(rb.shapes[0].verts, rb.shapes[0].tris),rb.pose(),camera.position,v1))
				{
					v1 = h.impact;
					selected = &rb;
					spoint = h.impact;
					rbpoint = rb.pose().Inverse()*h.impact;
				}
			}
		}
		spoint = camera.position + ray * magnitude(spoint - camera.position)*powf(1.025f, (float)mywin.mousewheel);
		mesh_cube.pose.position = spoint;
		if (!mywin.MouseState)
			selected = NULL;

		std::vector<LimitAngular> angulars;
		std::vector<LimitLinear>  linears;
		for (auto const &joint : joints)
		{
			Append(linears, ConstrainPositionNailed(&rbs[joint.rbi0], joint.p0, &rbs[joint.rbi1], joint.p1));
			Append(angulars, ConstrainAngularRange(&rbs[joint.rbi0], &rbs[joint.rbi1], { 0, 0, 0, 1 }, joint.jointlimitmin, joint.jointlimitmax));
		}
		if (selected)
			Append(linears, ConstrainPositionNailed(NULL, spoint, selected, rbpoint));
		for(auto &p:pins)
			Append(linears, ConstrainPositionNailed(NULL, p.w,p.rb,p.p));
		PhysicsUpdate(Addresses(rbs), linears, angulars, { &groundpoints });

		for (unsigned int i = 0; i < rbs.size(); i++)
		{
			meshes[i].pose = rbs[i].pose();
		}

		mywin.RenderScene(camera, Append(Addresses(meshes),std::vector<Mesh*>({ &ground, &mesh_cube })));
	}
}
catch (std::exception e)
{
	MessageBoxA(GetActiveWindow(), e.what(), "FAIL", 0);
	return -1;
}


