// 
//     The limit to marching cubes type of Algorithm 
//
//     by Stan Melax (c) 1998   bsd licence
//
//  backstory - In the mid 90s software heightfield rasterizers, typically used to render terrain, were being called "voxel" engines.  
//              So, I came up with this in response to suggestions to do another voxel engine.  (wrote a separate trimesh based heightfield renderer too)  
//              Had less RAM back then, so the idea was to have a *binary* voxel grid, and generate mesh from that, 
//              but let generated vertices have freedom to position within voxel volume.
//              Tesselation turns out to be what you might get if you took the limit of MCA when specifying a surface right at 1.0 (at the gridpoints). 
//              Avoids the ambiguous cases in MCA too.
//
//  keeping around since was useful for a couple of mod-able soft-body-ish geometry demos in the recent past.
//
//  update 2016 - attempted some code cleanups and updates and keeping minimal sized demo with zero effort on the rendering aspects
//  still lots of very "legacy" looking code here, 
//

#define NOMINMAX
#include <windows.h>
#include <gl/gl.h>

#include <assert.h>
#include <stdio.h>  
#include <stdlib.h>
#include <math.h>

#include "geometric.h"

#include "glwin.h"
#include "mesh.h"
#include "misc_gl.h"

//--------- some redundant drawing utils ------------

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


//------ some redundant geometric routines --------- 


struct quadratic_roots { int num; float ta, tb; operator bool() { return (num!=0); } };
quadratic_roots SolveQuadratic(float a, float b, float c)  // a*x^2+b*x+c==0  if true returns roots x={ta,tb} where ta<=tb
{
	float d = b*b - 4.0f*a*c; // discriminant
	if (d < 0.0f) return{ 0,0.0f,0.0f };
	float sqd = sqrtf(d);
	return  { ((d == 0.0f) ? 1 : 2), (-b - sqd) / (2.0f * a)  , (-b + sqd) / (2.0f * a) };
}

int HitCheckRaySphere(const float3& sphereposition, float radius, const float3& _v0, const float3& _v1, float3 *impact, float3 *normal)
{
	assert(impact);
	assert(normal);
	float3 dv = _v1 - _v0;
	float3 v0 = _v0 - sphereposition; // solve in coord system of the sphere
	if (radius <= 0.0f || _v0 == _v1) return 0; // only true if point moves from outside to inside sphere.
	float a = dot(dv, dv);
	float b = 2.0f * dot(dv, v0);
	float c = dot(v0, v0) - radius*radius;
	if (c<0.0f) return 0; // we are already inside the sphere.

	auto intersections = SolveQuadratic(a, b, c);

	if (!intersections) return 0;

	if (intersections.ta >= 0.0f && intersections.ta <= 1.0f && (intersections.ta <= intersections.tb || intersections.tb <= 0.0f))
	{
		if (impact)
			*impact = _v0 + dv * intersections.ta;
		if (normal)
			*normal = (v0 + dv*intersections.ta) / radius;
		return 1;
	}
	if (intersections.tb >= 0.0f && intersections.tb <= 1.0f)
	{
		assert(intersections.tb <= intersections.ta || intersections.ta <= 0.0f);  // tb must be better than ta
		if (impact)
			*impact = _v0 + dv * intersections.tb;
		if (normal)
			*normal = (v0 + dv*intersections.tb) / radius;
		return 1;
	}
	return 0;
}

int HitCheckRayCylinder(const float3 &p0, const float3 &p1, float radius, const float3& _v0, const float3& _v1, float3 *impact, float3 *normal)
{
	assert(impact);
	assert(normal);
	// only concerned about hitting the sides, not the caps for now
	float3x3 m = qmat(quat_from_to(p1 - p0, float3(0, 0, 1.0f)));
	float h = mul(m, p1 - p0).z;
	float3 v0 = mul(m, _v0 - p0);
	float3 v1 = mul(m, _v1 - p0);
	if (v0.z <= 0.0f && v1.z <= 0.0f) return 0;  // entirely below cylinder
	if (v0.z >= h    && v1.z >= h) return 0;  // ray is above cylinder
	if (v0.z <0.0f)  v0 = PlaneLineIntersection(float3(0, 0, 1.0f), 0, v0, v1);  // crop to cylinder range
	if (v1.z <0.0f)  v1 = PlaneLineIntersection(float3(0, 0, 1.0f), 0, v0, v1);
	if (v0.z > h)  v0 = PlaneLineIntersection(float3(0, 0, 1.0f), -h, v0, v1);
	if (v1.z > h)  v1 = PlaneLineIntersection(float3(0, 0, 1.0f), -h, v0, v1);
	if (v0.x == v1.x && v0.y == v1.y) return 0;
	float3 dv = v1 - v0;

	float a = dv.x*dv.x + dv.y*dv.y;
	float b = 2.0f * (dv.x*v0.x + dv.y*v0.y);
	float c = (v0.x*v0.x + v0.y*v0.y) - radius*radius;
	if (c<0.0f) return 0; // we are already inside the cylinder .

	auto intersections = SolveQuadratic(a, b, c);

	if (!intersections) return 0;

	if (intersections.ta >= 0.0f && intersections.ta <= 1.0f && (intersections.ta <= intersections.tb || intersections.tb <= 0.0f))
	{
		*impact = mul(transpose(m), v0 + dv * intersections.ta) + p0;
		*normal = mul(transpose(m), (float3(v0.x, v0.y, 0.0f) + float3(dv.x, dv.y, 0) * intersections.ta) / radius);
		return 1;
	}
	if (intersections.tb >= 0.0f && intersections.tb <= 1.0f)
	{
		assert(intersections.tb <= intersections.ta || intersections.ta <= 0.0f);  // tb must be better than ta
		*impact = mul(transpose(m), v0 + dv * intersections.tb) + p0;  // compute intersection in original space
		*normal = mul(transpose(m), (float3(v0.x, v0.y, 0.0f) + float3(dv.x, dv.y, 0) * intersections.tb) / radius);
		return 1;
	}
	return 0;
}

int HitCheckPoly(const float3 *verts, const int n, const float3 &v0, const float3 &v1, float3 *impact, float3 *normal)  // legacy function call
{
	// PolyHitCheck(const std::vector<float3>& verts, const float3 &v0, const float3 &v1) { return PolyHitCheck(verts, PolyPlane(verts), v0, v1); }
	auto hit = PolyHitCheck({ verts,verts + n }, v0, v1);
	if (!hit)
		return 0;
	if (impact) *impact = hit.impact;
	if (normal) *normal = hit.normal;
	return hit.hit;  
}

int HitCheckSweptSphereTri(const float3 &p0, const float3 &p1, const float3 &p2, float radius, const float3& v0, const float3& _v1, float3 *impact, float3 *normal)
{
	float3 unused;
	if (!normal) normal = &unused;
	float3 v1 = _v1;  // so we can update v1 after each sub intersection test if necessary
	int hit = 0;
	float3 cp = cross(p1 - p0, p2 - p0);
	if (dot(cp, v1 - v0) >= 0.0f) return 0; // coming from behind and/or moving away
	float3 n = normalize(cp);
	float3 tv[3];
	tv[0] = p0 + n*radius;
	tv[1] = p1 + n*radius;
	tv[2] = p2 + n*radius;
	hit += HitCheckPoly(tv, 3, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p0, p1, radius, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p1, p2, radius, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p2, p0, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p0, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p1, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p2, radius, v0, v1, &v1, normal);
	if (hit && impact) *impact = v1 + *normal * 0.001f;
	return hit;
}


//--------

static int blobinitialized=0;
int blobenable =1;
int useoffset=1;
int brain=0;
#define SIZEX (64)
#define SIZEY (64)
#define SIZEZ (24)
unsigned char blob[SIZEX][SIZEY][SIZEZ];
float3 offset[SIZEX][SIZEY][SIZEZ];
float3 nurmal[SIZEX][SIZEY][SIZEZ];


float3 blobposition(0.0f,0.0f,-0.0f);
float blobscale=1.00f;
float blobtexscale=0.2f;
int3 voxhit;
float3 blobhitpoint;
int blobishit = 0;

unsigned char withit[SIZEX+1][SIZEY+1];


int perimeter=255;
int allowsingularhole=0;
int allowsingularvoxel=0;

float3 globedir[27];
float3 globecolor[27];

// the writeup explains there are only 12 base cases
// this is still true but one of them requires reflection
// to generate its symmetries - I just generated the mirror by hand here.
#define NUM_BASE_CUBE (13)
class basecubenode {
public:
	unsigned char id;
	unsigned char n;
	unsigned char t[3][3];
}basecube[NUM_BASE_CUBE] = { 
	{  7,1,{{0,1,2},{0,0,0},{0,0,0}}},	// bc==3 sym==24
	{ 15,2,{{0,1,3},{0,3,2},{0,0,0}}},	// bc==4 sym==6  (square)
	{ 23,1,{{1,2,4},{0,0,0},{0,0,0}}},	// bc==4 sym==8
	{ 27,2,{{0,4,3},{1,3,4},{0,0,0}}},	// bc==4 sym==12
	{ 29,2,{{0,3,4},{2,4,3},{0,0,0}}},	// bc==4 sym==12  (mirror of 27 to avoid mirroring and winding hassles)
	{ 30,1,{{1,3,2},{0,0,0},{0,0,0}}},	// bc==4 sym==24
	{ 31,2,{{1,3,2},{1,2,4},{0,0,0}}},	// bc==5 sym==24
	{ 61,3,{{0,3,4},{2,4,3},{0,4,5}}},	// bc==5 sym==24
	{107,1,{{0,5,3},{0,0,0},{0,0,0}}},	// bc==5 sym==8
	{ 63,2,{{2,4,5},{2,5,3},{0,0,0}}},	// bc==6 sym==12 (square)
	{111,2,{{0,5,3},{0,3,6},{0,0,0}}},	// bc==6 sym==12
	{126,2,{{1,4,2},{3,6,5},{0,0,0}}},	// bc==6 sym==4
	{127,1,{{3,6,5},{0,0,0},{0,0,0}}},	// bc==7 sym==8
};
unsigned char RX[8] = {2,3,6,7,0,1,4,5}; // rotate 90 on X axis
unsigned char RY[8] = {4,0,6,2,5,1,7,3};
unsigned char RZ[8] = {1,3,0,2,5,7,4,6};
unsigned char MX[8] = {1,0,3,2,5,4,7,6}; // mirror along X axis

class MCube :public basecubenode{
public:
	//unsigned char	n;
	//unsigned char	t[3][3];
	float3 nrml[3];
	float3 udir[3];
	float3 vdir[3];
	float3 color[3];
	int    dir[3];
	unsigned char	ID();
	        MCube();
	void    Init(unsigned char _n,unsigned char _t[3][3]);
	void    Render(const int3 &c, Mesh &blobmesh);
	int     HitCheck(float3 v,float3 v1,float3 v2,float3 *impact);
	int     GetTri(int i,const float3 &v,float3 *p0,float3 *p1,float3 *p2);
	void    Massage(const int3 &v);
};
MCube mcube[256];
unsigned char MCube::ID() {
	int id = (int)(this - mcube);
	assert(id>=0 && id <256);
	return (unsigned char) id;
}
MCube::MCube() {
	id = ID();
	n=0;
}
void MCube::Init(unsigned char _n,unsigned char _t[3][3]) {
	n=_n;
	int i,j;
	for(i=0;i<3;i++) for(j=0;j<3;j++) {
		t[i][j] = _t[i][j];
	}
	for(i=0;i<n;i++) {
		float3 p[3];
		for(j=0;j<3;j++) {
			p[j] = float3((t[i][j]&1)?1.0f:0.0f,(t[i][j]&2)?1.0f:0.0f,(t[i][j]&4)?1.0f:0.0f);
		}
		nrml[i]  = normalize(cross(p[1]-p[0],p[2]-p[0]));
		udir[i] = Orth(nrml[i]);
		vdir[i] = safenormalize(cross(nrml[i],udir[i]));
		udir[i]= udir[i]; // optimization to multiply coef now
		vdir[i]= vdir[i];
		int x = (nrml[i].x >0.5f)?1:((nrml[i].x <-0.5f)?-1:0);
		int y = (nrml[i].y >0.5f)?1:((nrml[i].y <-0.5f)?-1:0);
		int z = (nrml[i].z >0.5f)?1:((nrml[i].z <-0.5f)?-1:0);
		dir[i] = 13 + x + 3*y + 9*z;
		assert(dir[i]>=0 && dir[i]<27 && dir[i] !=13);
		globedir[dir[i]]=nrml[i]; // just in case it aint initialized
		color[i] = (nrml[i]+float3(1.0f,1.0f,1.0f))*0.5f;
	}
}
unsigned char Permutation(unsigned char oldid,unsigned char t[3][3],unsigned char *permutation){
	int i,j;
	unsigned char newid=0;
	for(j=0;j<8;j++) {
		if((oldid>>j)&1) newid |= (1<<(permutation[j]));
	}
	for(i=0;i<3;i++) {
		for(j=0;j<3;j++){
			t[i][j] = permutation[t[i][j]];
		}
	}
	return newid;
}

int bitcount(unsigned char a) {
	int j,count=0;
	for(j=0;j<8;j++) {
		if((a>>j)&1)count++;
	}
	return count;
}

void symmetries()  // creates the global table 
{
	int cs[256];
	int i;
	for(i=0;i<256;i++) { cs[i] =-1;}

	int sx,sy,sz;
	for(i=0;i<256;i++) 
	 if(-1==cs[i])
	{
		int id=i;
		basecubenode *base;
		unsigned char t[3][3] = { { 0,0,0},{0,0,0},{0,0,0}};
		unsigned char n=0;
		assert(n==0);
		assert(t[0][0]==0);
		for(base= basecube;base<basecube+NUM_BASE_CUBE;base++) {
			if(base->id == i) {
				n=base->n;
				int j,k;
				for(j=0;j<3;j++) for(k=0;k<3;k++) {
					t[j][k] = base->t[j][k];
				}
				break;
			}
		}
		if(base==basecube+NUM_BASE_CUBE){ base=NULL;}
		for(sx=0;sx<4;sx++) {
	 		for(sy=0;sy<4;sy++) {
		 		for(sz=0;sz<4;sz++) {
					if(cs[id] == -1) {
						cs[id] = i;
						mcube[id].Init(n,t);
					}
					assert(cs[id]==i);
					id = Permutation(id,t,RZ);
				}
				id = Permutation(id,t,RY);
			}
			id = Permutation(id,t,RX);
		}
		assert(id==i);
	}
	for(int bc=0;bc<=8;bc++) {
		for(i=0;i<256;i++) {
			assert(cs[i]>=0 && cs[i]<256);
			assert(mcube[i].n<=3);
			if(cs[i]==i && bitcount(i)==bc) {
				int j;
				int count=0;
				for(j=0;j<256;j++) {
					if(cs[j]==i){
						count ++;
						assert(mcube[j].n == mcube[i].n);
					}
				}
				//printf("%d (%2d): ",bc,count);
				for(j=0;j<256;j++) {
					//if(cs[j]==i) printf("%3d ",(int)j);
				}
				//printf("\n");
			}
		}
	}
}

float3 normalhack;
int trihit(float3 vert[3],float3 v1,float3 v2,float3 *impact)
{
	float3 normal = cross(vert[1]-vert[0],vert[2]-vert[1]);
	if(length(normal)<=0.0001f)return 0;
	normal = normalize(normal);
	float dist = -dot(normal,vert[0]);
	if(dot(v1,normal)+dist <0  ||  dot(v2,normal)+dist >0) return 0;
	float3 the_point = PlaneLineIntersection(normal,dist,v1,v2);
	int inside=1;
	for(int j=0;inside && j<3;j++) {
		// let inside = 0 if outside
		float3 pp1,pp2,side;
		pp1 = vert[j] ;
		pp2 = vert[(j+1)%3];
		side = cross(pp2-pp1,the_point-pp1);
		inside = (dot(normal,side) > 0.0);
	}
	if(inside && impact) {
		normalhack=normal;
		*impact=the_point+normal*0.001f;
	}
	return inside;
}

int3 voxlocalhithack;
int MCube::HitCheck(float3 v,float3 v1,float3 v2,float3 *impact){
	int hit=0;
	int i;
	for(i=0;i<n;i++) {
		float3 vert[3];
		unsigned char *r=t[i];
		vert[0] =v+float3((r[0]&1)?1.0f:0.0f,(r[0]&2)?1.0f:0.0f,(r[0]&4)?1.0f:0.0f);
		vert[1] =v+float3((r[1]&1)?1.0f:0.0f,(r[1]&2)?1.0f:0.0f,(r[1]&4)?1.0f:0.0f);
		vert[2] =v+float3((r[2]&1)?1.0f:0.0f,(r[2]&2)?1.0f:0.0f,(r[2]&4)?1.0f:0.0f);
		if(useoffset) {
			vert[0] = vert[0]+offset[(int)vert[0].x][(int)vert[0].y][(int)vert[0].z];
			vert[1] = vert[1]+offset[(int)vert[1].x][(int)vert[1].y][(int)vert[1].z];
			vert[2] = vert[2]+offset[(int)vert[2].x][(int)vert[2].y][(int)vert[2].z];
		}
		if(trihit(vert,v1,v2,&v2)) {
			hit=1;
			if(impact) *impact=v2;
			float3 m ( length(v2 - vert[0]), length(v2 - vert[1]), length(v2 - vert[2]) );
			int k= argmin(m);
			voxlocalhithack=int3((r[k]&1)?1:0,(r[k]&2)?1:0,(r[k]&4)?1:0);
		}
	}
	return hit;
}
int MCube::GetTri(int i,const float3 &v,float3 *p0,float3 *p1,float3 *p2)
{
	assert(i<n);
	float3 vert[3];
	unsigned char *r=t[i];
	vert[0] =v+float3((r[0]&1)?1.0f:0.0f,(r[0]&2)?1.0f:0.0f,(r[0]&4)?1.0f:0.0f);
	vert[1] =v+float3((r[1]&1)?1.0f:0.0f,(r[1]&2)?1.0f:0.0f,(r[1]&4)?1.0f:0.0f);
	vert[2] =v+float3((r[2]&1)?1.0f:0.0f,(r[2]&2)?1.0f:0.0f,(r[2]&4)?1.0f:0.0f);
	// (useoffset) 
	*p0 = vert[0]+offset[(int)vert[0].x][(int)vert[0].y][(int)vert[0].z];
	*p1 = vert[1]+offset[(int)vert[1].x][(int)vert[1].y][(int)vert[1].z];
	*p2 = vert[2]+offset[(int)vert[2].x][(int)vert[2].y][(int)vert[2].z];
	return 1;
}




unsigned char getblob(int x,int y,int z){
	if(x<0 || x>=SIZEX) return perimeter;
	if(y<0 || y>=SIZEY) return perimeter;
	if(z<0 || z>=SIZEZ) return perimeter;
	return blob[x][y][z];
}
unsigned char blobat(int x,int y,int z){
	return (getblob(x,y,z) &1);
}
unsigned char computblob(int x,int y,int z) {
	return (
			((blobat(x  ,y  ,z  ))? 1  :0) |
			((blobat(x+1,y  ,z  ))? 2  :0) |
			((blobat(x  ,y+1,z  ))? 4  :0) |
			((blobat(x+1,y+1,z  ))? 8  :0) |
			((blobat(x  ,y  ,z+1))? 16 :0) |
			((blobat(x+1,y  ,z+1))? 32 :0) |
			((blobat(x  ,y+1,z+1))? 64 :0) |
			((blobat(x+1,y+1,z+1))? 128:0) 
		   );
}

void deblob(int dx,int dy,int dz) 
{
	if(!blobat(dx,dy,dz) ) return;
	if(dx<=1 ||dy<=1 || dz<=1 || dx >=SIZEX-2|| dy>=SIZEY-2 || dz>=SIZEZ-2) return;
	withit[dx][dy]=0;
	withit[dx-1][dy]=0;
	withit[dx][dy-1]=0;
	withit[dx-1][dy-1]=0;
	blob[dx][dy][dz]--; 
	int x,y,z;
	for(x= std::max(dx-1,0) ; x<=dx;x++)
	 for(y= std::max(dy-1,0) ; y<=dy;y++)
	  for(z= std::max(dz-1,0) ; z<=dz;z++)
	{
		blob[x][y][z] = computblob(x,y,z);
	}
	if(!allowsingularvoxel) 
	{
		if(!blobat(dx-2,dy,dz)) deblob(dx-1,dy,dz);
		if(!blobat(dx+2,dy,dz)) deblob(dx+1,dy,dz);
		if(!blobat(dx,dy-2,dz)) deblob(dx,dy-1,dz);
		if(!blobat(dx,dy+2,dz)) deblob(dx,dy+1,dz);
		if(!blobat(dx,dy,dz-2)) deblob(dx,dy,dz-1);
		if(!blobat(dx,dy,dz+2)) deblob(dx,dy,dz+1);
	}
	if(!allowsingularhole) 
	{
		if(blobat(dx+((dx%2)?1:-1),dy,dz) ) deblob(dx-((dx%2)?1:-1),dy,dz);
		if(blobat(dx,dy+((dy%2)?1:-1),dz) ) deblob(dx,dy-((dy%2)?1:-1),dz);
		if(blobat(dx,dy,dz+((dz%2)?1:-1)) ) deblob(dx,dy,dz-((dz%2)?1:-1));
	}
}
void terraform(int dx,int dy,int dz) {
	if(blobat(dx,dy,dz) ) return;
	if(dx<=1 ||dy<=1 || dz<=1 || dx >=SIZEX-2|| dy>=SIZEY-2 || dz>=SIZEZ-2) return;
	withit[dx][dy]=0;
	withit[dx-1][dy]=0;
	withit[dx][dy-1]=0;
	withit[dx-1][dy-1]=0;
	blob[dx][dy][dz]++; 
	int x,y,z;
	for(x= std::max(dx-1,0) ; x<=dx;x++)
	 for(y= std::max(dy-1,0) ; y<=dy;y++)
	  for(z= std::max(dz-1,0) ; z<=dz;z++) {
		blob[x][y][z] = computblob(x,y,z);
	}
	if(!allowsingularhole) {
		if(blobat(dx-2,dy,dz)) terraform(dx-1,dy,dz);
		if(blobat(dx+2,dy,dz)) terraform(dx+1,dy,dz);
		if(blobat(dx,dy-2,dz)) terraform(dx,dy-1,dz);
		if(blobat(dx,dy+2,dz)) terraform(dx,dy+1,dz);
		if(blobat(dx,dy,dz-2)) terraform(dx,dy,dz-1);
		if(blobat(dx,dy,dz+2)) terraform(dx,dy,dz+1);
	}
	if(!allowsingularvoxel) {
		if(!blobat(dx+((dx%2)?1:-1),dy,dz) ) terraform(dx-((dx%2)?1:-1),dy,dz);
		if(!blobat(dx,dy+((dy%2)?1:-1),dz) ) terraform(dx,dy-((dy%2)?1:-1),dz);
		if(!blobat(dx,dy,dz+((dz%2)?1:-1)) ) terraform(dx,dy,dz-((dz%2)?1:-1));
	}
}

int tid[3];
int startinghole(int x,int y,int z){
	if (z>=6 && z<= 14 && x>=18 && x<=22 && y>=18 && y<=22) return 1;
	if (z>=8 && z<= 11 && x>=14 && x<=26 && y>=14 && y<=26) return 1;
	return 0;

}
void initblob() {
	symmetries();
	int x,y,z;
	for(x=0;x<SIZEX;x++)
 	 for(y=0;y<SIZEY;y++)
	  for(z=0;z<SIZEZ;z++) {
		//blob[x][y][z] = (z<2+(int)(3.0*(1+cos(720.0* 3.14/180.0*x/16.0))*(1+cos(720.0* 3.14/180.0*y/16.0))))?1:0;
		blob[x][y][z] = 1;
		if(startinghole(x,y,z)) blob[x][y][z]=0;
		if(z<=1||x<=1 || y<=1 || x>=SIZEX-2 || y>=SIZEY-2 || z>=SIZEZ-2) blob[x][y][z] = (perimeter)?1:0;
	}

	for(x=0;x<SIZEX;x++)
 	 for(y=0;y<SIZEY;y++)
	  for(z=0;z<SIZEZ;z++) {
		blob[x][y][z] = computblob(x,y,z); 
	}
	blobinitialized = 1;
}


static int3 hithack;

int blobHitCheckPolySweptSphere(float radius,float3 v0,float3 v1,float3 *impact,float3 *normal)
{
	// replace this with a 3d brezenham thing
	int x,y,z;
	float3 dv=v1-v0;
	int n=(int)length(v1-v0);
	if(n==0) {n=1;}
	dv = dv * (1.0f/(float)n);
	int i;
	if(useoffset) 
	{
		int hit=0;
		for(i=0;i<n;i+=4)
		{
			float3 vmid = v0+dv*(float)i;
			for(x=std::max(0,(int)vmid.x-2);x<=std::min(SIZEX-1,(int)vmid.x+2);x++)
			 for(y= std::max(0,(int)vmid.y-2);y<= std::min(SIZEY-1,(int)vmid.y+2);y++)
			  for(z= std::max(0,(int)vmid.z-2);z<= std::min(SIZEZ-1,(int)vmid.z+2);z++)
			{
				int b=getblob(x,y,z);
				if(mcube[b].n==0) continue;
				for(int j=0;j<mcube[b].n;j++)
				{
					float3 p[3];
					mcube[b].GetTri(j,float3((float)x,(float)y,(float)z),p,p+1,p+2);
					hit += HitCheckSweptSphereTri(p[0],p[1],p[2],radius,v0,v1,&v1,normal);
					// dont return yet, see if any other tris happen to be closer than the hit one here 
				}
			}
			if(hit) 
			{
				if(impact) *impact=v1;
				return hit;
			}
		}
	}
	return 0; // assume that we are using offsets here regardless.
}

int blobHitCheckPoly(float3 v1,float3 v2,float3 *impact)
{
	// replace this with a 3d brezenham thing
	int x,y,z;
	float3 dv=v2-v1;
	int n=(int)length(v2-v1);
	if(n==0) {n=1;}
	dv = dv * (1.0f/(float)n);
	int i;
	if(useoffset) 
	{
		int hit=0;
		for(i=0;i<n;i+=4)
		{
			float3 vmid = v1+dv*(float)i;
			for(x= std::max(0,(int)vmid.x-2);x<= std::min(SIZEX-1,(int)vmid.x+2);x++)
			 for(y= std::max(0,(int)vmid.y-2);y<= std::min(SIZEY-1,(int)vmid.y+2);y++)
			  for(z= std::max(0,(int)vmid.z-2);z<= std::min(SIZEZ-1,(int)vmid.z+2);z++)
			{
				int b=getblob(x,y,z);
				if(mcube[b].n==0) continue;
				if(mcube[b].HitCheck(float3((float)x,(float)y,(float)z),v1,v2,&v2)) 
				{
					hithack[0]=x+voxlocalhithack.x;
					hithack[1]=y+voxlocalhithack.y;
					hithack[2]=z+voxlocalhithack.z;
					hit ++;
					// dont return yet, see if any other tris happen to be closer than the hit one here 
				}
			}
			if(hit) 
			{
				if(impact) *impact=v2;
				return hit;
			}
		}
	}
	else for(i=0;i<=n;i++)// yup - less or equal to n
	{  
		x=(int) ( (v1.x+dv.x*i));
		y=(int) ( (v1.y+dv.y*i));
		z=(int) ( (v1.z+dv.z*i));
		int b=getblob(x,y,z);
		//printf("blobHitcheck at %d,%d,%d  %d:%d\n",
		//	x,y,z,b,mcube[b].n);
		if(mcube[b].n==0) continue;
		//printf("mcube[%d].Hitcheck((%d,%d,%d),(%f,%f,%f),(%f,%f,%f),impact)\n",
		//	x,y,z,v1.x,v1.y,v1.z,v2.x,v2.y,v2.z);
		if(mcube[b].HitCheck(float3((float)x,(float)y,(float)z),v1,v2,impact)) {
			//printf("Hit %f,%f,%f\n",impact->x,impact->y,impact->z);
			hithack[0]=x+voxlocalhithack.x;
			hithack[1]=y+voxlocalhithack.y;
			hithack[2]=z+voxlocalhithack.z;
			return 1;
		}
	}
	return 0;
}
int blobHitCheckVoxel(float3 v1,float3 v2,float3 *impact,int hittype=1){
	// not exactly 3d brezenham thing
	int x,y,z;
	float3 dv=v2-v1;
	int n=(int)length(v2-v1);
	if(n==0) {n=1;}
	dv = dv * (1.0f/(float)n);
	int i;
	for(i=0;i<=n;i++) {  // yup - less or equal to n
		x=(int) (0.5 + (v1.x+dv.x*i));
		y=(int) (0.5 + (v1.y+dv.y*i));
		z=(int) (0.5 + (v1.z+dv.z*i));
		int b=blobat(x,y,z);
		if(b==hittype) {
			hithack[0]=x;
			hithack[1]=y;
			hithack[2]=z;
			//deblob(x,y,z);
			if(impact) {*impact=float3((float)x,(float)y,(float)z);}
			return 1;
		}
	}
	return 0;
}

int BlobHitCheckRelative(const float3 &_v0, float3 v1, float3 *impact)
{
	if (!blobenable) return 0;
	float3 v0 = _v0 - blobposition;
	v1 -= blobposition;
	v0 /= blobscale;
	v1 /= blobscale;
	int hit = blobHitCheckPoly(v0, v1, &v1);
	blobishit = hit;
	if (hit && impact)
	{
		blobhitpoint = *impact = v1 * blobscale + blobposition;
		voxhit = hithack;
	}
	return hit;
}

void blobHit(int hittype=1) {
	if(hittype) {
		deblob(hithack[0],hithack[1],hithack[2]);
	}
	else {
		terraform(hithack[0],hithack[1],hithack[2]);
	}
}



void MCube::Render(const int3 &p,Mesh &blobmesh)
{
	// appends verts and tris to the datamesh 'blobmesh'
	float3 v = float3(p);
	int i;
	if(!n ) return;
	for(i=0;i<n;i++) {
		unsigned char *r=t[i];
		int v0x=p.x+((r[0]&1)?1:0);
		int v0y=p.y+((r[0]&2)?1:0);
		int v0z=p.z+((r[0]&4)?1:0);
		int v1x=p.x+((r[1]&1)?1:0);
		int v1y=p.y+((r[1]&2)?1:0);
		int v1z=p.z+((r[1]&4)?1:0);
		int v2x=p.x+((r[2]&1)?1:0);
		int v2y=p.y+((r[2]&2)?1:0);
		int v2z=p.z+((r[2]&4)?1:0);
		float3 v0((float)v0x,(float)v0y,(float)v0z);// =v+float3((r[0]&1)?1:0,(r[0]&2)?1:0,(r[0]&4)?1:0);
		float3 v1((float)v1x,(float)v1y,(float)v1z);// =v+float3((r[1]&1)?1:0,(r[1]&2)?1:0,(r[1]&4)?1:0);
		float3 v2((float)v2x,(float)v2y,(float)v2z);// =v+float3((r[2]&1)?1:0,(r[2]&2)?1:0,(r[2]&4)?1:0);
		float2 t0 = float2(dot(v0,udir[i])*blobtexscale,dot(v0,vdir[i])*blobtexscale);
		float2 t1 = float2(dot(v1,udir[i])*blobtexscale,dot(v1,vdir[i])*blobtexscale);
		float2 t2 = float2(dot(v2,udir[i])*blobtexscale,dot(v2,vdir[i])*blobtexscale);
		if(useoffset) {
			v0 = v0+offset[v0x][v0y][v0z];
			v1 = v1+offset[v1x][v1y][v1z];
			v2 = v2+offset[v2x][v2y][v2z];
		}
		float3 facenrml = safenormalize(cross(v1-v0,v2-v1));
		std::vector<Vertex> &verts = blobmesh.verts;
		int vc = verts.size();
		blobmesh.tris.push_back(int3(vc,vc+1,vc+2));
		verts.push_back({v0*blobscale,quatfrommat({udir[i],vdir[i],facenrml}),t0});		
		verts.push_back({v1*blobscale,quatfrommat({udir[i],vdir[i],facenrml}),t1});		
		verts.push_back({v2*blobscale,quatfrommat({udir[i],vdir[i],facenrml}),t2});		
	}																	
}
void ClampMag(float3 &v,float m=1.0f) {
	float d=length(v);
	if(d<m) return;
	v= v * (m/d);
}
void MCube::Massage(const int3 &v){
	int i;
	if(!n ) return;
	for(i=0;i<n;i++) {
		unsigned char *r=t[i];
		int v0x=v.x+((r[0]&1)?1:0);
		int v0y=v.y+((r[0]&2)?1:0);
		int v0z=v.z+((r[0]&4)?1:0);
		int v1x=v.x+((r[1]&1)?1:0);
		int v1y=v.y+((r[1]&2)?1:0);
		int v1z=v.z+((r[1]&4)?1:0);
		int v2x=v.x+((r[2]&1)?1:0);
		int v2y=v.y+((r[2]&2)?1:0);
		int v2z=v.z+((r[2]&4)?1:0);
		float3 v0((float)v0x,(float)v0y,(float)v0z);// =v+float3((r[0]&1)?1:0,(r[0]&2)?1:0,(r[0]&4)?1:0);
		float3 v1((float)v1x,(float)v1y,(float)v1z);// =v+float3((r[1]&1)?1:0,(r[1]&2)?1:0,(r[1]&4)?1:0);
		float3 v2((float)v2x,(float)v2y,(float)v2z);// =v+float3((r[2]&1)?1:0,(r[2]&2)?1:0,(r[2]&4)?1:0);

		//float3 v0 =v+float3((r[0]&1)?1:0,(r[0]&2)?1:0,(r[0]&4)?1:0);
		//float3 v1 =v+float3((r[1]&1)?1:0,(r[1]&2)?1:0,(r[1]&4)?1:0);
		//float3 v2 =v+float3((r[2]&1)?1:0,(r[2]&2)?1:0,(r[2]&4)?1:0);
		float3 v0o = v0+offset[(int)v0.x][(int)v0.y][(int)v0.z];
		float3 v1o = v1+offset[(int)v1.x][(int)v1.y][(int)v1.z];
		float3 v2o = v2+offset[(int)v2.x][(int)v2.y][(int)v2.z];
		offset[v0x][v0y][v0z] =
			offset[v0x][v0y][v0z]*0.98f + 
			(v1o-v0)* 0.01f + (v2o-v0) *0.01f;
		offset[v1x][v1y][v1z] = 
			offset[v1x][v1y][v1z]*0.98f +
			(v2o-v1)* 0.01f + (v0o-v1) *0.01f;
		offset[v2x][v2y][v2z] = 
			offset[v2x][v2y][v2z]*0.98f +
			(v0o-v2)* 0.01f + (v1o-v2) *0.01f;

		ClampMag(offset[v0x][v0y][v0z]);
		ClampMag(offset[v1x][v1y][v1z]);
		ClampMag(offset[v2x][v2y][v2z]);

	}
}

void blob_settle()
{
	for (auto v : vol_iteration(int3(SIZEX, SIZEY, SIZEZ)))
		mcube[getblob(v.x, v.y, v.z)].Massage(v);
}

Mesh blobtomesh()
{
	Mesh blobmesh;
	blobmesh.verts.clear();
	blobmesh.tris.clear();
	int3 v;  // voxel
	for(v.x=0; v.x<SIZEX; v.x++)
 	 for(v.y=0; v.y<SIZEY; v.y++)
	  for(v.z=0; v.z<SIZEZ; v.z++)
	{
		mcube[getblob(v.x, v.y, v.z)].Render(v,blobmesh);
	}
	return blobmesh;
}


int blobHitCheckPolySweptSphereW(float r,const float3 &_v0,const float3 &_v1,float3 *_impact,float3 *_normal)
{
	if(!within_range((_v1-blobposition)/blobscale,float3(0,0,0),float3((float)SIZEX,(float)SIZEY,(float)SIZEZ))) return 0;
	float3 impact,normal;
	int hit = blobHitCheckPolySweptSphere(r/blobscale,(_v0-blobposition)/blobscale,(_v1-blobposition)/blobscale,&impact,&normal);
	if(hit && _impact) *_impact = impact*blobscale+blobposition;
	if(hit && _normal) *_normal = normal;
	return hit;
}

int blobnav(float r,float h,const float3 &_v0,const float3 &_v1,float3 *impact)
{
	if(!blobinitialized || !blobenable) return 0;
	float3 v0 = (_v0 - blobposition) / blobscale;
	float3 v1 = (_v1 - blobposition) / blobscale;
	h/=blobscale;
	r/=blobscale;
	if(!within_range(v1,float3(0,0,0),float3((float)SIZEX,(float)SIZEY,(float)SIZEZ))) return 0;
	int hitcount=0;
	float3 hitpoint;
	float3 normal;

	if(blobHitCheckPoly(v0+float3(0,0,h*0.5f),v0+float3(0,0,h),&hitpoint))
	{
		hitcount++;
		float dz = hitpoint.z-(v0.z+h);		
		v1.z += dz;
		v0.z += dz;
	}
	if(blobHitCheckPoly(v0+float3(0,0,h),v0+float3(0,0,h*0.5f),&hitpoint))
	{
		hitcount++;
		float dz = hitpoint.z-(v0.z+h*0.5f);		
		v1.z += dz;
		v0.z += dz;
	}

	float3 offsetlist[3];
	offsetlist[0] = float3(0,0,h*0.5f+r);
	offsetlist[1] = float3(0,0, (h-r));

	//blobHitCheckPolySweptSphere(r,v0+offset,v1+offset,&hitpoint,normal)
	for(int i=0;i<2;i++)
	{
		float3 offset=offsetlist[i];
 		int hit=0;
		while(hit<3 && blobHitCheckPolySweptSphere(r,v0+offset,v1+offset,&hitpoint,&normal) )
		{
			v1= ProjectOntoPlane(float4(normal,-dot(hitpoint,normal)),v1+offset)-offset;
			hit++;
		}
		while(hit<6 && blobHitCheckPolySweptSphere(r,v0+offset,v1+offset,&hitpoint,&normal)) 
		{
			v1=hitpoint-offset;
			hit++;
		}
		if(hit==6)
		{
			v1=v0;
		}
		hitcount+=hit;
	}	

	if(hitcount && impact)
	{
		*impact = v1*blobscale + blobposition;
	}
	return hitcount;
}

int blobnavold(float r,float h,const float3 &_v0,const float3 &_v1,float3 *impact)
{
	if(!blobinitialized || !blobenable) return 0;
	float3 v0 = (_v0 - blobposition) / blobscale;
	float3 v1 = (_v1 - blobposition) / blobscale;
	if(!within_range(v1,float3(0,0,0),float3((float)SIZEX,(float)SIZEY,(float)SIZEZ))) return 0;
	int hitcount=0;
	float3 hitpoint;
	if(blobHitCheckPoly(v0+float3(0,0,h/blobscale),v0,&hitpoint))
	{
		hitcount++;
		float dz = hitpoint.z-v0.z;		
		v1.z += dz;
		v0.z += dz;
	}
	if(blobHitCheckPoly(v0,v0+float3(0,0,h/blobscale),&hitpoint))
	{
		hitcount++;
		float dz = hitpoint.z-(v0.z+h/blobscale);		
		v1.z += dz;
		v0.z += dz;
	}
	for(int i=0;i<3;i++)
	{
		float3 offset(0,0,h/blobscale * (float)i / 2.0f );
		int hit=0;
		while(hit<3 && blobHitCheckPoly(v0+offset,v1+offset,&hitpoint))
		{
			v1= ProjectOntoPlane(float4(normalhack,-dot(hitpoint,normalhack)),v1+offset)-offset;
			hit++;
		}
		while(hit<6 && blobHitCheckPoly(v0+offset,v1+offset,&hitpoint)) 
		{
			v1=hitpoint-offset;
			hit++;
		}
		if(hit==6)
		{
			v1=v0;
		}
		hitcount+=hit;
	}
	if(hitcount && impact)
	{
		*impact = v1*blobscale + blobposition;
	}
	return hitcount;
}



/*
class BlobManipulator : public Manipulator
{
public:
	BlobManipulator();
	//~BlobManipulator();
	virtual int  HitCheck(const float3 &v0, float3 v1,float3 *impact);
	virtual int  Drag(const float3 &v0, float3 v1,float3 *impact,int ctrldown,int shiftdown) override;
	int KeyPress(int k);
};
BlobManipulator *blobmanipulator;
BlobManipulator::BlobManipulator(){blobmanipulator=this;}
int BlobManipulator::HitCheck(const float3 &_v0, float3 v1,float3 *impact)
{
	if(!blobenable) return 0;
	float3 v0 = _v0 - blobposition;
	v1-=blobposition;
	v0 /= blobscale;
	v1 /= blobscale;
	int hit = blobHitCheckPoly(v0,v1,&v1);
	blobishit=hit;
	if(hit && impact)
	{
		blobhitpoint = *impact = v1 * blobscale + blobposition;
		voxhit = hithack;
	}
	return hit;
}
int BlobManipulator::Drag(const float3 &_v0, float3 v1,float3 *impact,int ctrldown,int shiftdown)
{
	currentmanipulator=NULL;

	deblob(voxhit.x,voxhit.y,voxhit.z); // blobHit(1);
	if(voxhit.z<SIZEZ-2)
		deblob(voxhit.x,voxhit.y,voxhit.z+1);
	return 0;
}
void slingblob(float3 v0,float3 v1)
{
	if(!blobmanipulator) return;
	if(blobmanipulator->HitCheck(v0,v1,&v1))
		blobmanipulator->Drag(v0,v1,&v1,0,0);
}
std::string blobstatus()
{
	if(!blobenable) return "";
	if(!blobishit) return "";
	if(!dynamic_cast<BlobManipulator*>(currentmanipulator)) return "";
	return "left-click to deform surface";
}
EXPORTFUNCTION(blobstatus);

int BlobManipulator::KeyPress(int k)
{
	switch(k)
	{
		case '[':
		case '{':
			blobmesh->matid = MaterialPrevRegular(blobmesh->matid); 
			return 1;
		case ']':
		case '}':
			blobmesh->matid = MaterialNextRegular(blobmesh->matid);
			return 1;
		case 'u':
			terraform(voxhit.x,voxhit.y,voxhit.z);
			terraform(voxhit.x-1,voxhit.y,voxhit.z);
			terraform(voxhit.x,voxhit.y-1,voxhit.z);
			terraform(voxhit.x,voxhit.y,voxhit.z-1);
			terraform(voxhit.x+1,voxhit.y,voxhit.z);
			terraform(voxhit.x,voxhit.y+1,voxhit.z);
			terraform(voxhit.x,voxhit.y,voxhit.z+1);
	}
	return 0;
}
*/

float mouse_sensitivity = 1.0f;

int main(int argc, const char argv[]) try
{
	GLWin glwin("testnav", 800, 600);
	Pose camera({ 20, 20, 10.0f }, { 0, 0, 0, 1 });
	float yaw = 0.0f;
	float pitch = 0.0f;
	float3 velocity(0,0,0);

	InitTex();
	initblob();


	while (glwin.WindowUp())
	{
		if (glwin.downevent)
			glwin.centermouse = !glwin.centermouse;

		std::vector<bool> keyheld(128, 0);
		for (int i = 0; i < 128; i++)  // update key states
			keyheld[i] = (glwin.focus && (GetAsyncKeyState(i), GetAsyncKeyState(i)));   // call twice hack since getasynckeystate detects later

		yaw = yaw + -glwin.dmouse.x* 180.0f / 3.14f* mouse_sensitivity ;
		while(yaw <   0.0f) yaw += 360.0f;
		while(yaw > 360.0f) yaw -= 360.0f;
		pitch = clamp(pitch + 180.0f / 3.14f*glwin.dmouse.y* mouse_sensitivity, -90.0f, 90.0f);

		camera.orientation = qmul(QuatFromAxisAngle({ 0,0,1 }, yaw*3.14f / 180.0f), QuatFromAxisAngle({ 1,0,0 }, (90.0f+pitch)*3.14f / 180.0f))  ;
		velocity = velocity*0.8f + mul(qmat(camera.orientation), float3(float(keyheld['D'] - keyheld['A']), float(keyheld['Q'] - keyheld['Z']), float(keyheld['S'] - keyheld['W']))*0.1f);  // in distance per frame - not per sec
		float3 pnew = camera.position + velocity;
		float3 impact, normal;
		if (blobHitCheckPolySweptSphere(0.5f, camera.position, pnew, &impact, &normal))
		{
			pnew = ProjectOntoPlane({ normal,-dot(impact,normal) }, pnew);
		}
		blobHitCheckPolySweptSphere(0.5f, camera.position, pnew, &pnew, &normal);  // will stick pnew at impact point 

		velocity = pnew - camera.position; // writeback velocity for next frame
		camera.position = pnew;
		float3 ray = qrot(camera.orientation, normalize(glwin.MouseVector));   // for mouse selection
		float3 v1 = camera.position + ray*100.0f;
		
		if (keyheld['B']&& BlobHitCheckRelative(camera.position, v1, &v1))
			deblob(hithack[0], hithack[1], hithack[2]); // deblob(v1.x, v1.y, v1.z);

		if (keyheld['R'] && BlobHitCheckRelative(camera.position, v1, &v1))
		{
			terraform(hithack[0], hithack[1], hithack[2]); // deblob(v1.x, v1.y, v1.z);
			for (int a : {0, 1, 2}) for(int s: {-1,1})
			{
				int3 v=hithack;
				v[a] += s;
				terraform(v.x, v.y, v.z);
			}
		}

		blob_settle();

		auto blobmesh = blobtomesh();


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, glwin.res.x, glwin.res.y);
		glClearColor(0, 0, 0, 1);
		glEnable(GL_CULL_FACE);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_FOG);
		float fogden = 0.02f;
		glFogfv(GL_FOG_DENSITY, &fogden);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix(); glLoadIdentity();
		gluPerspective(glwin.ViewAngle, (double)glwin.aspect_ratio(), 0.02, 70.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix(); glLoadIdentity();

		glMultMatrixf(camera.inverse().matrix());

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glBegin(GL_TRIANGLES);
		for (auto t : blobmesh.tris) for (auto i : t)
		{
			auto &v = blobmesh.verts[i];
			glNormal3fv(qzdir(v.orientation)); glTexCoord2fv(v.texcoord); glVertex3fv(v.position);
		}
		glEnd();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();  
		glPopAttrib();
		glwin.PrintString({ 0, 0 }, "[esc] to quit, [b] to blast holes,  [r] to refill holes");
		glwin.PrintString({ 0, 1 }, glwin.centermouse? "Mouse and WASD to move" :"LMouse click for navigation");
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
