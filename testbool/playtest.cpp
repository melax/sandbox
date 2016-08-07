//  

#include <exception> 
#include <iostream>
#include <fstream>
#include <cctype>    // std::tolower

#include <geometric.h>
#include <glwin.h>
#include "../testbsp/bsp.h"
#include "misc.h"
#include "misc_gl.h"
#include <mesh.h>

void InitTex()  // create a checkerboard texture   (duplicated function, move to misc_gl.h)
{
	const int imagedim = 16;
	byte3 checker_image[imagedim * imagedim];
	for (int y = 0; y < imagedim; y++)
		for (int x = 0; x < imagedim; x++)
			checker_image[y * imagedim + x] = ((x + y) % 2) ? byte3(255, 255, 255) : byte3(127, 127, 127);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // use nearest to see actual uninterpolated image pixels
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imagedim, imagedim, 0, GL_RGB, GL_UNSIGNED_BYTE, checker_image);
}


std::vector<Face> WingMeshToFaces(const WingMesh &m)
{
	std::vector<Face> faces;
	assert(m.unpacked == 0);
	assert(m.fback.size() == m.faces.size());
	int k = 0;
	for (unsigned int i = 0; i<m.faces.size(); i++)
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


void glDraw(BSPNode *root, const float3 &camera_position)
{
	const float3 rainbow[] = { { 1, 0, 0 },{ 1, 1, 0 },{ 0, 1, 0 },{ 0, 1, 1 },{ 0, 0, 1 },{ 1, 0, 1 } };
	for (auto n : treebacktofront(root, camera_position))
	{
		if (n->isleaf)
			continue;
		WingMesh &wm = n->under->convex;
		int fid = std::find_if(wm.faces.begin(), wm.faces.end(), [&n](const float4 &p) {return n->plane() == p || n->plane() == -p; }) - wm.faces.begin();
		assert(fid < (int)wm.faces.size());
		auto q = quat_from_to(wm.faces[fid].xyz(), float3(0, 0, 1));
		glBegin(GL_POLYGON);
		float3 both[2] = { n->xyz(), -n->xyz() };
		float3 c = float3(0.5f, 0.5f, 0.5f) + rainbow[argmax(&both[0][0], 6)] * 0.5f;
		glColor4f(c.x, c.y, c.z, 0.35f);
		glNormal3fv(wm.faces[fid].xyz());  // should test to see which side camera_position is on 
		for (auto &v : wm.GenerateFaceVerts(fid))
		{
			glTexCoord2fv(qrot(q, v).xy()*0.1f);
			glVertex3fv(v);
		}
		glEnd();
	}
}
void fdraw(const Face &f)
{
	glBegin(GL_POLYGON);
	glNormal3fv(f.xyz());
	int k = argmax(abs(f.xyz()));
	for (auto &v : f.vertex)
		glTexCoord2fv(float2(v[(k+1)%3], v[(k+2)%3])*0.05f) , glVertex3fv(v);
	glEnd();
}
void fdraw(const std::vector<Face> &faces) { for (auto &f : faces) fdraw(f); }
void fdraw(BSPNode *root,const float3 &camera_position) 
{ 
	for (auto n : treebacktofront(root, camera_position))
		for (auto &f : n->brep) 
			fdraw(f); 
}

//----- player nav code ---------


class Player : public Pose //: public Entity
{
public:

	Pose&      pose() { return *this; }
	const Pose& pose() const { return *this; }
	Pose       eyepose() { return pose() * Pose({ 0,0,height*0.9f }, QuatFromAxisAngle({ 1,0,0 }, 3.14f / 180.0f*(90 + headtilt))); }
	float3     positionnew;
	float3     positionold;
	float3     velocity;
	float      height;
	float      radius;
	float      headtilt=0.0f;
	float3     groundnormal = { 0,0,1 };
	bool       groundcontact = true;
	Player() :height(1.5f), radius(0.3f) //,Entity("player")
	{
		position = float3(0, 0, 0.1f); // default a few cm off the ground if it happens to be z==0 plane
	}
	void areacheck( BSPNode *bsp);
	void wasd_mlook(const float2 &dmouse, const float3 &thrust, const BSPNode *bsp);
};


float3 gravity(0, 0, -10.0f);
float  maximumspeed = 10.0f;
float  jumpspeed = 10.0f;
float  dampingground = 10.0f;
float  dampingair = 1.0f;
float DeltaT = 0.016f;

float mouse_sensitivity = 0.5f;


extern float3 HitCheckImpactNormal;


void Player::areacheck( BSPNode *bsp)
{
	float grounddist = 0;
	int wallcontact = 0;
	float3 target = positionnew;
	float3 impact;
	int hit = 0;
	while (hit<5 && HitCheckCylinder(radius, height, bsp, 0, position, positionnew, &impact, float3(0, 0, 0))) {
		hit++;
		float3 norm = HitCheckImpactNormal;
		if (norm.z > 0.5f) {
			groundcontact = true;
			groundnormal = norm;
			grounddist = -dot(groundnormal, impact);
		}
		else if (norm.z > -0.5f) {
			wallcontact = 1;
		}
		// slide along plane of impact
		positionnew = ProjectOntoPlane(float4(norm, -dot(norm, impact)), positionnew) + norm*0.00001f;
		if (groundcontact && dot(norm, groundnormal)<0 && dot(groundnormal, positionnew) + grounddist <= 0) {
			float3 slide = ProjectOntoPlane(float4(norm, 0), groundnormal);
			slide *= -(dot(groundnormal, positionnew) + grounddist) / (dot(slide, groundnormal));
			slide += safenormalize(slide)*0.000001f;
			positionnew += slide;
		}
	}
	if (hit == 5) {
		// couldn't resolve it;
		positionnew = position ;  // TO TEST: adjusmment based on room brush moving.
	}
	if (groundcontact && wallcontact) {
		hit = 1;
		float3 positionup = positionnew + float3(0, 0, 0.4f);
		while (hit<5 && HitCheckCylinder(radius, height, bsp, 0, positionnew, positionup, &positionup, float3(0, 0, 0))) {
			positionup.z -= 0.00001f;
			hit++;
		}
		float3 targetup = target + (positionup - positionnew);
		while (hit<5 && HitCheckCylinder(radius, height, bsp, 0, positionup, targetup, &impact, float3(0, 0, 0))) {
			hit++;
			float3 norm = HitCheckImpactNormal;
			// slide along plane of impact
			targetup = ProjectOntoPlane(float4(norm, -dot(norm, impact)), targetup) + norm*0.00001f;
		}
		float3 positiondrop = targetup - (positionup - positionnew);
		while (hit<5 && HitCheckCylinder(radius, height, bsp, 0, targetup, positiondrop, &positiondrop, float3(0, 0, 0))) {
			positiondrop.z += 0.00001f;
			hit++;
		}
		if (hit != 5) {
			// couldn't resolve it;
			positionnew = positiondrop;
		}
	}
	if (hit) {
		velocity = (positionnew - position) / DeltaT;
	}
}

void Player::wasd_mlook(const float2 &dmouse,const float3 &thrust, const BSPNode *bsp)
{

	float  damping = (groundcontact) ? dampingground : dampingair;
	float4 thrustdomain = (groundcontact) ? qmul(quat_from_to(float3(0, 0, 1), groundnormal),orientation) : orientation;
	float3 accelleration = (qydir(thrustdomain)*thrust.y + qxdir(thrustdomain)*thrust.x)*maximumspeed*damping;
	accelleration += gravity;
	float3 contact_velocity(0, 0, 0);  // if the ground is moving put its velocity here

	float3 accdamping = (velocity - contact_velocity ) * -damping;

	float3 microimpulse(0,0,0);
	if (groundcontact && thrust==float3(0.0f)) {
		microimpulse = groundnormal * (gravity.z*groundnormal.z) - float3(0, 0, gravity.z);
	}
	if (groundcontact) {
		velocity.z = std::min(0.0f, velocity.z);
		if (thrust.z>0) {
			velocity.z = jumpspeed;
			groundcontact = false;
		}
	}
	accelleration += accdamping;
	accelleration += microimpulse;
	velocity = velocity + accelleration * DeltaT;
	positionnew = position + velocity *DeltaT;

	orientation = normalize(qmul( orientation , QuatFromAxisAngle(float3(0, 0, 1), -dmouse.x* mouse_sensitivity)));
	headtilt = clamp(headtilt+ 180.0f/3.14f*dmouse.y* mouse_sensitivity , -90.0f,90.0f);


	//-- the move part
	positionold=position;
	float3 unused; // dummy variable to store returned unused impact point

	groundcontact=false;

	areacheck((BSPNode*)bsp);  // do this bsp last since we have to remain in a valid position wrt this space.

	position = positionnew;	

}

//------------------------------

const float qsnap = 0.5f; // turns out to be 1/4th this value in practice for axial aligned planes
const int   qcuberadius = 4;

unsigned int CubeSide(const float3 &v)
{
	auto side = linalg::argmax(abs(v));
	return side;
}

float  round(float  x,float precision){ return precision*round(x/precision); }
float3 round(float3 x,float precision){ return precision*round(x/precision); }
float3 CubeProjected(const float3 &v) { return v * ((float)qcuberadius / maxelem(abs(v      ))); }
float4 CubeProjected(const float4 &p) { return p * ((float)qcuberadius / maxelem(abs(p.xyz()))); }
float3 Quantized(const float3 &v)     { return normalize(linalg::round(CubeProjected(v)));       }
float  QuantumDist(const float3 &n)   { return qsnap / length(linalg::round(CubeProjected(n)));  }
float4 Quantized(const float4 &p)     { auto n= CubeProjected(p.xyz()); float mag = length(n); return {n/mag,round(p.w,qsnap/mag)}; }

std::vector<std::unique_ptr<BSPNode>> mashers;
void blast_init()
{
	if (mashers.size())
		return;
	for (int m = 0; m < 10; m++)
	{
		WingMesh mashbox = WingMeshCube(0.5f);
		for (int i = 0; i < 8; i++) 
		{
			float3 rn((rand() % 9 - 4) / 4.0f, (rand() % 9 - 4) / 4.0f, (rand() % 9 - 4) / 4.0f);
			//float3 rn((rand()%5-2)/2.0f,(rand()%5-2)/2.0f,(rand()%5-2)/2.0f);
			rn = safenormalize(rn);
			float rd = -1.00f* (((rand() % 10000) / 10000.0f) * 0.5f + 0.125f);
			mashbox = WingMeshCrop(mashbox, Quantized(float4(rn, rd)));
		}
		auto b = move(NegateTree(move(BSPCompile(WingMeshToFaces(mashbox), WingMeshCube(16.0f)))));
		BSPDeriveConvex(*b, WingMeshCube(30.51f));
		BSPMakeBrep(b.get(), {});
		mashers.emplace_back(move(b));
	}
}
std::unique_ptr<BSPNode> bsp_blast(std::unique_ptr<BSPNode> bsp, const float3 &p,float s)
{
	if (mashers.size() == 0)
		blast_init();
	static int k = 0;
	k = (k + 1) % mashers.size();
	auto b = BSPDup(mashers[k].get());
	BSPScale(*b, s);
	BSPTranslate(*b, round(p, qsnap));
	return BSPIntersect(move(b),move(bsp) );
}

//---------------

int main(int argc,const char argv[]) try
{
	GLWin glwin("testnav", 800, 600);
	Pose camera({ 0, 0, 20 }, { 0, 0, 0, 1 });

	InitTex();


	// create an initial arena:
	auto arena = WingMeshBox({ -10, -10, -5 }, { 10, 10, 5 });  // arena volume
	auto bsp = BSPCompile(WingMeshToFaces(arena),WingMeshBox(float3(-32.0f),float3(32.0f)));
	NegateTree(*bsp );
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({ -11.0f, -11.0f,-0.25f }, {  11.0f, 11.0f, 0.0f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({   4.0f, -11.0f, -6.0f }, {   4.5f, 11.0f, 6.0f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({  -4.5f, -11.0f, -6.0f }, {  -4.0f, 11.0f, 6.0f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({ -11.0f,   4.0f, -6.0f }, {  11.0f,  4.5f, 6.0f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({ -11.0f,  -4.5f, -6.0f }, {  11.0f, -4.0f, 6.0f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	bsp = BSPUnion( move(BSPCompile(WingMeshToFaces(WingMeshBox({   2.5f,   1.5f,  2.0f }, {   3.5f,  3.5f, 4.5f })), WingMeshBox(float3(-16.0f), float3(16.0f)))),move(bsp));
	for (float x : {-7.0f, 0.0f, 7.0f})
	{
		auto doorsx = BSPCompile(WingMeshToFaces(WingMeshBox({ x-1.0f,  -9.0f, 0.1f }, { x+1.0f, 9.0f, 2.5f })), WingMeshBox(float3(-16.0f), float3(16.0f)));
		NegateTree(*doorsx);
		bsp = BSPIntersect(move(doorsx),move(bsp));
	}
	for (float y : {-7.0f, 7.0f})
	{
		auto doorsy = BSPCompile(WingMeshToFaces(WingMeshBox({  -9.0f,y-1.0f,  0.1f }, { 9.0f,y+1.0f,  2.5f })), WingMeshBox(float3(-16.0f), float3(16.0f)));
		NegateTree(*doorsy);
		bsp = BSPIntersect(move(doorsy), move(bsp));
	}
	BSPMakeBrep(bsp.get(), BSPRipBrep(bsp.get()));                     // just regenerate the brep, ensures no T-intersections
	//auto faces = BSPRipBrep(bsp.get());                               // brep moved into faces array
	// done arena creation

	bool planeview = false;
	bool cammove = false;
	bool camsnap = true;
	glwin.keyboardfunc =[&](int key, int , int)
	{
		planeview = planeview != (key == 'p');
		glwin.centermouse = glwin.centermouse != (key == 'm');
		cammove = cammove != (key == 'c');
		camsnap = camsnap != (key == 'l');
	};
	Player player;

	while(glwin.WindowUp())
	{
		if (glwin.downevent)
			glwin.centermouse = !glwin.centermouse;

		std::vector<bool> keyheld(128, 0);
		for (int i = 0; i < 128; i++)  // update key states
			keyheld[i] = (glwin.focus && (GetAsyncKeyState(i), GetAsyncKeyState(i)));   // call twice hack since getasynckeystate detects later

		if (keyheld['B']) // (key == 'b')
		{
			float3 hitpoint;
			if (HitCheck(bsp.get(), 0, camera.position, camera*float3(0, 0, -100.0f), &hitpoint) && length(hitpoint)<14.0f)
				bsp = bsp_blast(move(bsp), hitpoint, 4.0f);
			//BSPMakeBrep(bsp.get(), BSPRipBrep(bsp.get()));               // just regenerate the brep, ensures no T-intersections
			//faces = BSPRipBrep(bsp.get());                               // brep moved into faces array
		}


		float3 thrust = float3((float)(keyheld['D'] - keyheld['A']), (float)(keyheld['W'] - keyheld['S']), (float)(keyheld[' '] - keyheld['Z']));
		player.wasd_mlook(glwin.dmouse, thrust, bsp.get());
		if (camsnap)
		{
			camera = player.pose() * Pose({ 0,0,player.height*0.9f }, QuatFromAxisAngle({ 1,0,0 }, 3.14f / 180.0f*(90 + player.headtilt)));
		}
		else if (cammove)
		{
			camera.position += mul(qmat(camera.orientation), float3(keyheld['D'] - keyheld['A'], keyheld['Q'] - keyheld['Z'], keyheld['S'] - keyheld['W']));
			camera.orientation = normalize(qmul(camera.orientation, quat_from_to(float3(0, 0, 1), float3(glwin.dmouse, 1))));
		}
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(0,0,0, 1);
		glEnable(GL_CULL_FACE);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_FOG);
		float fogden = 0.1f;
		glFogfv(GL_FOG_DENSITY, &fogden);
		
		glMatrixMode(GL_PROJECTION);
		glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.aspect_ratio(), 0.02, 50.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();

		glMultMatrixf(camera.inverse().matrix());

		if (planeview)
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glColor4f(1, 1, 1, 0.90f);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			fdraw(bsp.get(), camera.position);  // use  glDraw() for planes
			glPopAttrib();
		}
		else
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glEnable(GL_TEXTURE_2D);
			glColor4f(1, 1, 1, 1);
			glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
			float4 ambient(0.50f, 0.75f, 1.0f, 1.0f);
			glMaterialfv(GL_FRONT, GL_AMBIENT, &ambient.x);
			fdraw(bsp.get(),camera.position); //  faces);
			glPopAttrib();
		}
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPushMatrix();
		glcolorbox(float3(player.radius, player.radius, player.height / 2.0f), player.pose()*Pose({ 0,0,player.height / 2.0f }, { 0,0,0,1 }));
		glPopMatrix();
		glPopAttrib();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();  
		glPopAttrib();
		glwin.PrintString({ 0, 0 }, "[esc] to quit, [b] to blow holes in stuff");
#       ifdef _DEBUG
		 glwin.PrintString({ 0, -1 }, "DEBUG Version.  CSG Boolean Ops may be SLOW.");
#       endif
		glwin.SwapBuffers();

	}
	return 0;
}
catch (const char *c)
{
	std::cerr << c << "\n";  
	MessageBox(GetActiveWindow(), c, "FAIL", 0); //throw(std::exception(c));
}
catch (const std::exception & e)
{
	std::cerr << e.what() << "\n"; 
	MessageBox(GetActiveWindow(), e.what(), "FAIL", 0);
}
