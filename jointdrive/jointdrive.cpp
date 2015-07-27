//
// JointDrive - sample showing powered ragdoll rigidbody physics
//
//  Animation influences the angular drive which is the desired relative orientation of two jointed bodies.
//  The physics simulation tries to put the rigidbodies in the desired pose by applying up to the allowable torque on a joint.
//  This sample uses a generated circular animation for the upper limbs to follow the path of a cone.
//  Typically one would use interpolated animation keyframe to set the joint drive.
//
//  this sample is a stripped down reimplementation from original spider seen in Teeter 2007 (http://melax.github.io/minigames/)
//  

#include <mesh.h>
#include <hull.h>
#include <wingmesh.h>  // to quickly make a box
#include <physics.h>
#include <geometric.h>
#include <dxwin.h>


// a simple 6 legged creature with boxes for bones
// note that typing numbers in code is the absolute worst way to create content. 
//
float3 bodysizes[] = { // box radius
	{ 0.25f, 0.50f, 0.10f },   // torso
	{ 0.25f, 0.05f, 0.05f },   // limb upper bones
	{ 0.25f, 0.05f, 0.05f },
	{ 0.25f, 0.05f, 0.05f },
	{ 0.25f, 0.05f, 0.05f },
	{ 0.25f, 0.05f, 0.05f },
	{ 0.25f, 0.05f, 0.05f },
	{ 0.05f, 0.05f, 0.25f },   // limb lower bones
	{ 0.05f, 0.05f, 0.25f },
	{ 0.05f, 0.05f, 0.25f },
	{ 0.05f, 0.05f, 0.25f },
	{ 0.05f, 0.05f, 0.25f },
	{ 0.05f, 0.05f, 0.25f },
};
struct
{
	int b0, b1;   // body0 and body1 (parent and child) indices
	float a;    // multiplier to generated animation 
	float3 p0, p1;  // attachment points
} joints[]=
{
	{ 0,  1,  0.2f, {  0.25f, -0.5f, 0 }, { -0.25f, 0, 0 } },     // attach upper limbs to torso
	{ 0,  2, -0.2f, {  0.25f,  0.0f, 0 }, { -0.25f, 0, 0 } },     
	{ 0,  3,  0.2f, {  0.25f,  0.5f, 0 }, { -0.25f, 0, 0 } },     
	{ 0,  4,  0.2f, { -0.25f, -0.5f, 0 }, {  0.25f, 0, 0 } },     
	{ 0,  5, -0.2f, { -0.25f,  0.0f, 0 }, {  0.25f, 0, 0 } },     
	{ 0,  6,  0.2f, { -0.25f,  0.5f, 0 }, {  0.25f, 0, 0 } },     
    { 1,  7,  0.0f, {  0.25f,     0, 0 } ,{  0, 0 ,0.25f } },     // attaches lower limb to corresponding upper limb
    { 2,  8,  0.0f, {  0.25f,     0, 0 } ,{  0, 0 ,0.25f } },
    { 3,  9,  0.0f, {  0.25f,     0, 0 } ,{  0, 0 ,0.25f } },
    { 4, 10,  0.0f, { -0.25f,     0, 0 } ,{  0, 0 ,0.25f } },
    { 5, 11,  0.0f, { -0.25f,     0, 0 } ,{  0, 0 ,0.25f } },
    { 6, 12,  0.0f, { -0.25f,     0, 0 } ,{  0, 0 ,0.25f } },
};


std::vector<float3> genboxverts(float3 r){std::vector<float3> verts; for (auto z : { -1.0f, 1.0f }) for (auto y : { -1.0f, 1.0f }) for (auto x : { -1.0f, 1.0f }) verts.push_back(cmul(r,float3(x,y,z))); return verts;}

int main(int argc, const char *argv[]) try
{
	std::vector<RigidBody> rbs;
	for (auto const &b : bodysizes)
	{
		auto verts = genboxverts(b);
		auto tris = calchull(verts, 8);
		rbs.push_back(RigidBody({ Shape(verts, tris) }, float3(0, 0, 0)));
	}
	rbscalemass(&rbs[0], 5.0f); // make torso heavier than limb bones
	rbs[0].position.z = 1.0f;  // lift a meter off the ground.
	DXWin mywin("Joint Drive - powered rag doll model");
	std::vector<Mesh> meshes;
	for (auto &rb : rbs)
	{
		meshes.push_back(MeshSmoothish(rb.shapes[0].verts, rb.shapes[0].tris)); //  1 shape each is known
		rb.damping = 0.8f;   //rb.gravscale = 0;
	}
	for (auto &joint : joints)
	{
		rbs[joint.b0].ignore.push_back(&rbs[joint.b1]);
		rbs[joint.b1].ignore.push_back(&rbs[joint.b0]);
		rbs[joint.b1].position = rbs[joint.b0].pose() * joint.p0 - qrot(rbs[joint.b1].orientation,joint.p1);
	}

	WingMesh ground_wm = WingMeshBox({ -5, -5, -2.0f }, { 5, 5, -1.0f });
	auto ground = MeshFlatShadeTex(ground_wm.verts, WingMeshTris(ground_wm));
	ground.hack = { 0.25f, 0.75f, 0.25f, 1 };
	
	Pose camera = { { 0, -8, 0 }, normalize(float4(0.9f, 0, 0, 1)) };   // where we view the rendered scene from.
	float time = 0;             // our global clock, used to generate the circular animation for upper limbs to follow
	float torquelimit = 38.0f;  // how much torque we let each joint apply each frame

	while (mywin.WindowUp())
	{
		time += 0.06f;

		std::vector<LimitAngular> angulars;
		std::vector<LimitLinear>  linears;
		for (auto const &joint : joints)
		{
			Append(linears, ConstrainPositionNailed(&rbs[joint.b0], joint.p0, &rbs[joint.b1], joint.p1));
			Append(angulars, ConstrainAngularDrive(&rbs[joint.b0], &rbs[joint.b1], (float4(0, joint.a*cos(time), joint.a*sin(time), sqrt(1.0f - joint.a*joint.a))), torquelimit));
		}
		PhysicsUpdate(Addresses<RigidBody>(rbs), linears, angulars, { &ground_wm.verts });

		for (unsigned int i = 0; i < rbs.size(); i++)
			meshes[i].pose = rbs[i].pose();

		mywin.RenderScene(camera, Append(Addresses(meshes), std::vector<Mesh*>({ &ground })));
	}
}
catch (std::exception e)
{
	MessageBoxA(GetActiveWindow(), e.what(), "FAIL", 0);
	return -1;
}


