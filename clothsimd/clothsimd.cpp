//
//  This is originally from a sample program i wrote at Intel during 2010 to illustrate the
//  SIMD processing on the x86 cpu and utilize the upcoming AVX instruction set.
//  Unfortunately, this sample is no longer supported, and, furthermore, 
//  the downloadable .zip on their website doesn't compile out of the box.
//  Below, i made some modifications so this sample would actually run.
//  Also migrated the code utilize more modern c++ conventions. 
//
//  This particular version below uses 128 bit intrinsics and thus can be compiled for either instruction set AVX or SSE.
//


//original copyright


// Copyright 2010 Intel Corporation
// All Rights Reserved
//
// Permission is granted to use, copy, distribute and prepare derivative works of this
// software for any purpose and without fee, provided, that the above copyright notice
// and this statement appear in all copies. Intel makes no representations about the
// suitability of this software for any purpose. THIS SOFTWARE IS PROVIDED "AS IS."
// INTEL SPECIFICALLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, AND ALL LIABILITY,
// INCLUDING CONSEQUENTIAL AND OTHER INDIRECT DAMAGES, FOR THE USE OF THIS SOFTWARE,
// INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PROPRIETARY RIGHTS, AND INCLUDING THE
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  Intel does not
// assume any responsability for any errors which may appear in this software nor any
// responsibility to update it.


#include "vecmatquat.h"
#include "mesh.h"
#include "dxwin.h"

#include "wingmesh.h"

//--------------- for convenience -----------------

inline float  Round(float a, float precision) { return floorf(0.5f + a / precision)*precision; }
template <class T> inline T Max(const T &a, const T &b) { return (a > b) ? a : b; }
template <class T> inline T Min(const T &a, const T &b) { return (a < b) ? a : b; }

//--------------- 2D ----------------------




//--------- 3D ---------

template<class T> class vec3
{
public:
	T x, y, z;
	__forceinline vec3() { x = 0; y = 0; z = 0; };
	__forceinline vec3(const T &_x, const T &_y, const T &_z) { x = _x; y = _y; z = _z; };
	__forceinline T& operator[](int i) { return ((T*)this)[i]; }
	__forceinline const T& operator[](int i) const { return ((T*)this)[i]; }
};


//typedef vec3<int>   int3;
//typedef vec3<short> short3;
//typedef vec3<float> float3;

// due to ambiguity there is no overloaded operators for v3*v3 use dot,cross,outerprod,cmul 
template<class T> __forceinline int operator==(const vec3<T> &a, const vec3<T> &b) { return (a.x == b.x && a.y == b.y && a.z == b.z); }
template<class T> __forceinline int operator!=(const vec3<T> &a, const vec3<T> &b) { return !(a == b); }
template<class T> __forceinline vec3<T> operator+(const vec3<T>& a, const vec3<T>& b) { return vec3<T>(a.x + b.x, a.y + b.y, a.z + b.z); }
template<class T> __forceinline vec3<T> operator-(const vec3<T>& a, const vec3<T>& b) { return vec3<T>(a.x - b.x, a.y - b.y, a.z - b.z); }
template<class T> __forceinline vec3<T> operator-(const vec3<T>& v) { return vec3<T>(-v.x, -v.y, -v.z); }
template<class T> __forceinline vec3<T> operator*(const vec3<T>& v, const T &s) { return vec3<T>(v.x*s, v.y*s, v.z*s); }
template<class T> __forceinline vec3<T> operator*(T s, const vec3<T>& v) { return v*s; }
template<class T> __forceinline vec3<T> operator/(const vec3<T>& v, T s) { return vec3<T>(v.x / s, v.y / s, v.z / s); }
template<class T> __forceinline T       dot(const vec3<T>& a, const vec3<T>& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
template<class T> __forceinline vec3<T> cmul(const vec3<T>& a, const vec3<T>& b) { return vec3<T>(a.x*b.x, a.y*b.y, a.z*b.z); }
template<class T> __forceinline vec3<T> cross(const vec3<T>& a, const vec3<T>& b) { return vec3<T>(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); }
template<class T> __forceinline vec3<T>& operator+=(vec3<T>& a, const vec3<T>& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
template<class T> __forceinline vec3<T>& operator-=(vec3<T>& a, const vec3<T>& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
template<class T> __forceinline vec3<T>& operator*=(vec3<T>& v, T s) { v.x *= s; v.y *= s; v.z *= s; return v; }
template<class T> __forceinline vec3<T>& operator/=(vec3<T>& v, T s) { v.x /= s; v.y /= s; v.z /= s; return v; }

template<class T> __forceinline vec3<T>VectorMin(const  vec3<T> &a, const  vec3<T> &b) { return  vec3<T>(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z)); }
template<class T> __forceinline vec3<T>VectorMax(const  vec3<T> &a, const  vec3<T> &b) { return  vec3<T>(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z)); }


//-------- 4D  --------

template<class T> class vec4
{
public:
	T x, y, z, w;
	__forceinline vec4() { x = 0; y = 0; z = 0; w = 0; };
	__forceinline vec4(const T &_x, const T &_y, const T &_z, const T &_w) { x = _x; y = _y; z = _z; w = _w; }
	__forceinline vec4(const vec3<T> &v, const T &_w) { x = v.x; y = v.y; z = v.z; w = _w; }
	T& operator[](int i) { return ((T*)this)[i]; }
	const T& operator[](int i) const { return ((T*)this)[i]; }
};

//typedef vec4<float> float4;
//typedef vec4<unsigned char> byte4; 
//typedef vec4<int>   int4;


//--------------------------- SIMD Operator Overloads and Convenience Routines -------------------------------
__forceinline __m128 operator *(const __m128 &a, const __m128 &b) { return _mm_mul_ps(a, b); }
__forceinline __m128 operator +(const __m128 &a, const __m128 &b) { return _mm_add_ps(a, b); }
__forceinline __m128 operator -(const __m128 &a, const __m128 &b) { return _mm_sub_ps(a, b); }
__forceinline __m128 operator -(const __m128 &b) { return _mm_sub_ps(_mm_setzero_ps(), b); }
__forceinline __m128 operator |(const __m128 &a, const __m128 &b) { return _mm_or_ps(a, b); }
__forceinline __m128 operator &(const __m128 &a, const __m128 &b) { return _mm_and_ps(a, b); }
__forceinline __m128 rsqrt(const __m128 &a) { return _mm_rsqrt_ps(a); }
__forceinline __m128 &operator+=(__m128 &a, const __m128 &b) { a = (_mm_add_ps(a, b)); return a; }
__forceinline __m128 &operator-=(__m128 &a, const __m128 &b) { a = (_mm_sub_ps(a, b)); return a; }
__forceinline __m128 Max(const __m128 &a, const __m128 &b) { return _mm_max_ps(a, b); }
__forceinline __m128 Min(const __m128 &a, const __m128 &b) { return _mm_min_ps(a, b); }





//--------------------------- SOA SIMD 3D Vector math library routines -------------------------------

template<> class vec3<__m128>
{
public:
	__m128 x, y, z;
	__forceinline vec3() {} // note - not initializing to 0
	__forceinline vec3(const __m128 &_x, const __m128 &_y, const __m128 &_z) { x = _x; y = _y; z = _z; };
	__forceinline explicit vec3(const float3 &v) :x(_mm_set1_ps(v.x)), y(_mm_set1_ps(v.y)), z(_mm_set1_ps(v.z)) {}
	__forceinline explicit vec3(float _x, float _y, float _z) :x(_mm_set1_ps(_x)), y(_mm_set1_ps(_y)), z(_mm_set1_ps(_z)) {}
	__forceinline __m128& operator[](int i) { return ((__m128*)this)[i]; }
	__forceinline const __m128& operator[](int i) const { return ((__m128*)this)[i]; }
};



//
// The soafloat3 convenience accessor class returns an object that makes it possible to write code to get/set 
// one of the xyz tuples within an SOA 3D vector as if it was an AOS float3 vector. 
// 
//  
class soafloat3  // note this is essentially a strided array view
{
public:
	float *a;
	soafloat3(vec3<__m128> &V, int s) :a((float*)(&V) + s) {}
	operator const float3() const { return float3(a[0], a[4], a[8]); }
	soafloat3& operator= (const float3 &v) { a[0] = v.x; a[4] = v.y; a[8] = v.z; return *this; }
	soafloat3& operator+=(const float3 &v) { a[0] += v.x; a[4] += v.y; a[8] += v.z; return *this; }
	soafloat3& operator-=(const float3 &v) { a[0] -= v.x; a[4] -= v.y; a[8] -= v.z; return *this; }
};
__forceinline float3 operator-(const soafloat3 &a, const soafloat3 &b) { return (float3)a - (float3)b; }

//------- global cloth simulation parameters ---------

static float3 cloth_gravity(0, 0, -20.0f);
static float  cloth_timestep = 0.016f;  // default 60fps - fixed delta time for constraintnetwork
static float  cloth_velocity_damp = 0.9f;  // a damping/friction factor to slow cloth object motion or sliding over time.
static float  cloth_bias = 0.3f;           // how much to adjust constraints to move them toward their restlength
static float  cloth_bias_decay = 0.95f;    // each successive solver iteration does a smaller adjustment to minimize any frame-to-frame vibrations  
static float3 cloth_rmin(-10.0f, -10.0f, 0.0f);  // limits of environment (box room) cloth is contained within
static float3 cloth_rmax(10.0f, 32.0f, 6.0f);  // 
static float  cloth_separation_epsilon = 0.01f;    // minimum distance used for collision with walls and floor - prevents z fighting. 
static int    cloth_solver_count = 4;  // typical range 8-32,  lower values can make long cloth too stretchy
static int    cloth_simdwidth = 4;      // 1 - serial  4 - 128 bit  8 - 256 bit avx processing




//-------------------------------------------------------------------------------------------
// ConstraintNetwork - main physics internal class for cloth objects
// Each ConstraintNetwork object is 8 cloth pieces
// The physics data (position,massinv,velocity,normal) is sometimes cast to things other than vec3<__m256>* since we also provide serial and 128bit implementations.
//
class ConstraintNetwork 
{
public:
	// 
	// Each constraint is defined by its endpoints and a restlength that is the natural distance between those points
	class Constraint
	{
	public:
		float restlen;
		short int a, b;      // constraint endpoints, indices into vertex array
		Constraint() {}
		Constraint(int _a, int _b, float _restlen) :a((short)_a), b((short)_b), restlen(_restlen) {}
	};
	Mesh mesh;
	// the following are inherited from parent class Mesh
	//  VertexC*  vertices;
	//  int       vertices_count;
	//  short3*   tris;
	//  int       tris_count;
	//  ID3D11Buffer *vbuf;
	//  ID3D11Buffer *tbuf;

	int           points_count; // number of points per cloth section, total number of vertices will be 8*points_count
	int           mesh_width;   // the width and height of each cloth mesh  note width==height and mesh_width*mesh_width==points_count
	float         simcycles;
	float         damp_air;
	vec3<__m128>* sbmin;  // bounding box minimum, updated by simulater each frame  
	vec3<__m128>* sbmax;  // bounding box maximum
	vec3<__m128>* swind;  // local wind direction (one for each cloth piece)

	Constraint *  constraints;
	int           constraints_count;
	int4 *        quads;          // physics uses these quads instead of the triangles - better for calculating vertex normals
	int           quads_count;
	vec3<__m128> *position;       // positions of all points
	vec3<__m128> *posn_old;       // positions from previous frame
	__m128 *      massinv;        // inverse mass of each point
	vec3<__m128> *velocity;       // velocities of each point
	vec3<__m128> *normal;         // normals of each point

	ConstraintNetwork(int _w, int _h, float size);
	~ConstraintNetwork();
	void          ConstraintNetworkCreate(int w, int h, float size);  // Add constraints and layout points given physical size
	void          SimulateSerial();  // No SIMD
	void          SimulateSIMD4();   // 128-bit AVX
	void          PhysicsToGraphics(); // SOA to AOS vertex buffer format
	void          UpdateBBoxLimits();    // bounding box for each of 8 sections
	void          RoomCollision();
	void          CalcNormals();
	void          CalcVelocity();
	void          Simulate();
	void          PointSetInvMass(int index, float invm);  // note an inverse mass of 0 is a fixed point.
	soafloat3     Point(int i) { return soafloat3(position[i%points_count], i / points_count); }  // get or set value of a point 
};

ConstraintNetwork::ConstraintNetwork(int _w, int _h, float physical_size) : points_count(_w*_h), constraints_count(0), quads_count(0)
{
	
	mesh_width = _w;
	assert(_w == _h);

	position = reinterpret_cast<vec3<__m128> *>(_mm_malloc(points_count * sizeof(__m128) * 3, 64));
	posn_old = reinterpret_cast<vec3<__m128> *>(_mm_malloc(points_count * sizeof(__m128) * 3, 64));
	normal = reinterpret_cast<vec3<__m128> *>(_mm_malloc(points_count * sizeof(__m128) * 3, 64));
	velocity = reinterpret_cast<vec3<__m128> *>(_mm_malloc(points_count * sizeof(__m128) * 3, 64));
	massinv = reinterpret_cast<__m128*>(_mm_malloc(points_count * sizeof(__m128), 64));
	mesh.verts.resize(points_count * 4); //  vertices = reinterpret_cast<Vertex*>(_mm_malloc(points_count * 8 * sizeof(VertexC), 64));

	for (int i = 0; i < points_count * 4 * 3; i++)
	{
		reinterpret_cast<float*>(position)[i] = 0.0f;
		reinterpret_cast<float*>(posn_old)[i] = 0.0f;
		reinterpret_cast<float*>(normal)[i] = 0.0f;
		reinterpret_cast<float*>(velocity)[i] = 0.0f;
	}
	for (int i = 0; i < points_count; i++) for (int s = 0; s < 4;s++)
	{
		reinterpret_cast<float*>(massinv)[i*4+s] = 1.0f;
		mesh.verts[i*4+s].texcoord =  {(float)(i % mesh_width) / (mesh_width-1), (float)(i/mesh_width) / (mesh_width-1) };
	}

	assert(mesh_width*mesh_width == points_count);
	mesh.tris.reserve((mesh_width - 1) * (mesh_width - 1) * 4 * 2);
	for (int i = 0; i < mesh_width - 1; i++)  for (int j = 0; j < mesh_width - 1; j++)  for (int s = 0; s < 4; s++)
	{
				int3 t = { s + 4 * ((i + 0)*mesh_width + (j + 0)), s + 4 * ((i + 1)*mesh_width + (j + 0)), s + 4 * ((i + 1)*mesh_width + (j + 1)) };
				mesh.tris.push_back( t);
				mesh.tris.push_back({ t.x,t.z,t.y });  // backside
				int3 r = { s + 4 * ((i + 0)*mesh_width + (j + 0)), s + 4 * ((i + 0)*mesh_width + (j + 1)), s + 4 * ((i + 1)*mesh_width + (j + 1)) };
				mesh.tris.push_back(r);
				mesh.tris.push_back({ r.x,r.z,r.y });  // backside
	}

	damp_air = 60.0f;  // max wind assuming 60fps this should be overrridden
					   // since sbmin, sbmax and swind need to be aligned we allocate all 3 of them here with a single _mm_malloc 
	sbmin = reinterpret_cast<vec3<__m128> *>(_mm_malloc(3 * 3 * sizeof(__m128), 64));
	sbmax = sbmin + 1;
	swind = sbmin + 2;
	*sbmin = vec3<__m128>(0, 0, 0);
	*sbmax = vec3<__m128>(0, 0, 0);
	ConstraintNetworkCreate(_w, _h, physical_size);  // add all the constraints, add the quads
}

ConstraintNetwork::~ConstraintNetwork()
{
	_mm_free(position);
	_mm_free(posn_old);
	_mm_free(normal);
	_mm_free(velocity);
	_mm_free(massinv);
	_mm_free(sbmin);
}

void ConstraintNetwork::ConstraintNetworkCreate(int w, int h, float size)
{
	// simple constraint network generation routine that creates a typical square patch.
	// note that we order the constraints to later minimize dependency of one loop iteration on the previous.
	int i, j;
	for (i = 0; i < h; i++)
		for (j = 0; j < w; j++)
		{
			for (int s = 0; s < 4; s++)
				this->Point(i*w + j + s*w*h) = (float3(-0.5f, -0.5f, 0) + float3((float)j / (w - 1.0f), 1.0f - (float)i / (w - 1.0f), 0)) * size;
		}

	float r = size / (w - 1.0f); // magnitude(points[0]-points[1]); // constraint restlength 

	constraints = new Constraint[6 * (w*h)];
	for (i = 0; i < h; i++) for (j = 0; j < w; j++)
		if (i < h - 1)        constraints[constraints_count++] = (Constraint(i*w + j, (i + 1)*w + j, r));     // structural
	for (j = 0; j < w; j++) for (i = 0; i < h; i++)
		if (j < w - 1)        constraints[constraints_count++] = (Constraint(i*w + j, i*w + (j + 1), r));     // structural
	for (i = 0; i < h; i++) for (j = 0; j < w; j++)
		if (j < w - 1 && i < h - 1) constraints[constraints_count++] = (Constraint(i*w + j, (i + 1)*w + (j + 1), r*sqrtf(2.0f))); // shear
	for (i = 0; i < h; i++) for (j = 0; j < w; j++)
		if (j>0 && i < h - 1) constraints[constraints_count++] = (Constraint(i*w + j, (i + 1)*w + (j - 1), r*sqrtf(2.0f))); // shear
	for (i = 0; i < h; i++) for (j = 0; j < w; j++)
		if (i < h - 2)        constraints[constraints_count++] = (Constraint(i*w + j, (i + 2)*w + j, r*2.0f));     // benders
	for (j = 0; j < w; j++) for (i = 0; i < h; i++)
		if (j < w - 2)        constraints[constraints_count++] = (Constraint(i*w + j, i*w + (j + 2), r*2.0f));     // benders
	assert(constraints_count <= 6 * w*h);

	quads_count = (h - 1)*(w - 1);
	quads = new int4[quads_count];
	for (i = 0; i < h - 1; i++)
		for (j = 0; j < w - 1; j++)
		{
			quads[i*(w - 1) + j] = int4((i + 0)*w + (j + 0), (i + 0)*w + (j + 1), (i + 1)*w + (j + 1), (i + 1)*w + (j + 0));
		}
	this->UpdateBBoxLimits();
}

void ConstraintNetwork::PointSetInvMass(int _index, float invm)
{
	assert(_index >= 0 && _index < points_count * 4);
	int index = _index / points_count +4 * (_index%points_count);
	float *m = reinterpret_cast<float*>(massinv);
	m[index] = invm;
}


void ConstraintNetwork::UpdateBBoxLimits()
{
	// sbmin and sbmax are the bounding box limits for the 4 cloth sections.
	*sbmin = vec3<__m128>(FLT_MAX, FLT_MAX, FLT_MAX);
	*sbmax = vec3<__m128>(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (int i = 0; i < points_count; i++)
	{
		vec3<__m128> v = position[i];       // position of i'th point for each of the 8 cloth sections.
		*sbmin = VectorMin(*sbmin, v);  // takes the min for each of the 24 floats xxxxxxxxyyyyyyyyzzzzzzzz from v and from sbmin
		*sbmax = VectorMax(*sbmax, v);
	}
}

//void inline Transpose8x4(__m128* dst, const __m128 &row0, const __m128 &row1, const __m128 &row2, const __m128 &row3, const __m128 &row4, const __m128 &row5, const __m128 &row6, const __m128 &row7)
//{
//	// for clarity refer to the 'columns' as x,y,z,w instead of [0][1][2][3].  eg row0 has 4 elements x0y0z0w0
//	// this is a generic transpose routine, the xyzw have no correspondence with position x y or z values in the application
//	__m128 r0r4 = _mm256_insertf128_ps(_mm256_castps128_ps256(row0), row4, 0x1); // x0y0z0w0x4y4z4w4 
//	__m128 r2r6 = _mm256_insertf128_ps(_mm256_castps128_ps256(row2), row6, 0x1); // x2y2z2w2x6y6z6w6 
//	__m128 r1r5 = _mm256_insertf128_ps(_mm256_castps128_ps256(row1), row5, 0x1); // x1y1z1w1x5y5z5w5 
//	__m128 r3r7 = _mm256_insertf128_ps(_mm256_castps128_ps256(row3), row7, 0x1); // x3y3z3w3x7y7z7w7 
//
//	__m128 xyeven = _mm256_unpacklo_ps(r0r4, r2r6); // x0x2y0y2x4x6y4y6
//	__m128 zweven = _mm256_unpackhi_ps(r0r4, r2r6); // z0z2w0w2z4z6w4w6
//	__m128 xyodd = _mm256_unpacklo_ps(r1r5, r3r7); // x1x3y1y3x5x7y5y7
//	__m128 zwodd = _mm256_unpackhi_ps(r1r5, r3r7); // z1z3w1w3z5z7w5w7
//
//	dst[0] = _mm256_unpacklo_ps(xyeven, xyodd); // x0x1x2x3x4x5x6x7
//	dst[1] = _mm256_unpackhi_ps(xyeven, xyodd); // y0y1y2y3y4y5y6y7							   
//	dst[2] = _mm256_unpacklo_ps(zweven, zwodd); // z0z1z2z3z4z5z6z7
//	dst[3] = _mm256_unpackhi_ps(zweven, zwodd); // w0w1w2w3w4w5w6w7
//}

void ConstraintNetwork::PhysicsToGraphics()
{
	// convert SOA to AOS vertex buffer format
	// From: position and normal arrays xxxxxxxxyyyyyyyyzzzzzzzz  To: vertices array px,py,pz,nx,ny,nz,tx,ty, px,py,pz,nx,...  
	// We just regenerate the texture coordinates each time this function is called tx,ty.
	// The 8x8 transpose is achieved by doing two 8x4 transposes
	// Need to do some casting since we need to pull data out by 128 bit chuncks (4 columns at a time) - not 256 bit chunks.
	__m128 (*normal)[3][2] = reinterpret_cast<__m128 (*)[3][2]>(this->normal);  // instead of 8 at a time, we pull out 4 at a time, so cast each vec3<_m256> as 6 _m128 vars
	__m128 (*position)[3][2] = reinterpret_cast<__m128 (*)[3][2]>(this->position);  // union __m256 doesn't have an elegant way of accessing its two _m128 parts
	for (int i = 0; i < points_count; i++) for (int s = 0; s < 4;s++)
	{
		mesh.verts[i * 4 + s].position.x = ((float*)(&this->position[i].x))[s];
		mesh.verts[i * 4 + s].position.y = ((float*)(&this->position[i].y))[s];
		mesh.verts[i * 4 + s].position.z = ((float*)(&this->position[i].z))[s];
		//float3 n = {  ((float*)(&this->normal[i].x))[s], ((float*)(&this->normal[i].y))[s], ((float*)(&this->normal[i].z))[s] };
		mesh.verts[i * 4 + s].orientation = { 0,0,0,1 }; // RotationArc({ 0,0,1 }, n);
		mesh.verts[i * 4 + s].texcoord = { i%mesh_width / (mesh_width-1.0f) , i / mesh_width / (mesh_width-1.0f) };
	}
	//__m256 *vertices = reinterpret_cast<__m256 *>(this->vertices);    // each vertex fits nicely into a __m256
	//for (int i = 0; i < points_count; i++)
	//{
	//	__m128 tx = _mm_set1_ps(i%mesh_width / (float)mesh_width);  // texture coordinates 'u'
	//	__m128 ty = _mm_set1_ps(i / mesh_width / (float)mesh_width);  // texture coordinates 'v'
	//	Transpose8x4(vertices + i * 8, position[i][0][0], position[i][1][0], position[i][2][0], normal[i][0][0], normal[i][1][0], normal[i][2][0], tx, ty);
	//	Transpose8x4(vertices + i * 8 + 4, position[i][0][1], position[i][1][1], position[i][2][1], normal[i][0][1], normal[i][1][1], normal[i][2][1], tx, ty);
	//}
}

void ConstraintNetwork::CalcVelocity()
{
	// Calculate actual velocity V for each point by looking at the difference between old and new position  posn_old and position
	// This is not a significant hotspot, so just use avx256 for everybody.
	__m128 invdt = _mm_set1_ps(1.0f / cloth_timestep);
	for (int i = 0; i < points_count; i++)
	{
		velocity[i] = (position[i] - posn_old[i]) * invdt;  // calculate the velocity for the i'th point on each of the 8 cloth sections
	}
}

void ConstraintNetwork::CalcNormals()
{
	// Calculate the normal of each point on the cloth sections by using all the quad normals for each quad the point is a part of.
	// not a major hotspot, let everybody use avx256 bit
	for (int i = 0; i < points_count; i++)
		normal[i] = vec3<__m128>(0, 0, 0);
	for (int i = 0; i < quads_count; i++)
	{
		vec3<__m128> &v0 = position[quads[i][0]];
		vec3<__m128> &v1 = position[quads[i][1]];
		vec3<__m128> &v2 = position[quads[i][2]];
		vec3<__m128> &v3 = position[quads[i][3]];
		vec3<__m128> n = cross(v1 - v0, v2 - v1) + cross(v3 - v2, v0 - v3);  // using face area weighted contribution to normal
		normal[quads[i][0]] += n;
		normal[quads[i][1]] += n;
		normal[quads[i][2]] += n;
		normal[quads[i][3]] += n;
	}
	__m128 epsilon = _mm_set1_ps(0.00000001f);  // for safety this prevents taking rsqrt of 0
	for (int i = 0; i < points_count; i++)
		normal[i] = normal[i] * _mm_rsqrt_ps(epsilon + dot(normal[i], normal[i]));   // set normal for i'th point for each of the 8 cloth sections
}

void ConstraintNetwork::RoomCollision()
{
	// Confine cloth point position position to the global bounding box cloth_rmin,cloth_rmax.
	// Note that we slightly scale the separation differently for each of the 8.  
	// Since we dont have cloth-cloth collision, this significantly reduces the z-fighting when rendering.
	// Noticed compiler doing some register spilling, but this isn't a major hotspot. 
	__m128       ce = _mm_set1_ps(cloth_separation_epsilon) * _mm_set_ps(1.0f, 1.1f, 1.2f, 1.3f); // prevents z-fighting
	vec3<__m128> mn = vec3<__m128>(cloth_rmin) + vec3<__m128>(ce, ce, ce);
	vec3<__m128> mx = vec3<__m128>(cloth_rmax) - vec3<__m128>(ce, ce, ce);
	for (int i = 0; i < points_count; i++)
	{
		position[i] = VectorMax(mn, VectorMin(mx, position[i])); // limit position of point i on each of the 8 cloth sections to be within global bounds
	}
}

// next 3 routines contain the serial, 128bit and 256bit versions of the solver which is the main computational section.
// These routines also use the cpu cycle counter to measure the number of cpu cycles per constraint update.
// In this case we want to determine best case time - i.e. the performance of the AVX instructions themselves.  
// Therefore timing this specific timing ignores removing first loop which is most likely to be an outlier where L1 data cache is not primed.
// 

void ConstraintNetwork::SimulateSerial()
{
	float *position = reinterpret_cast<float *>(this->position);
	float *posn_old = reinterpret_cast<float *>(this->posn_old);
	float *velocity = reinterpret_cast<float *>(this->velocity);
	float *normal = reinterpret_cast<float *>(this->normal);
	float *massinv = reinterpret_cast<float *>(this->massinv);

	float3 g = cloth_gravity;
	float  a = this->damp_air;
	float* w = reinterpret_cast<float *>(swind); // absolute direction of the wind
	float  d = (cloth_velocity_damp*cloth_timestep);
	float  t = (cloth_timestep);

	for (int i = 0; i < points_count; i++)
		for (int j = 0; j < 4; j++)
		{
			float  mdp_adamp =
				((velocity[i * 24 + 0 + j] - w[j + 0]) * normal[i * 24 + 0 + j] +
					(velocity[i * 24 + 4 + j] - w[j + 4]) * normal[i * 24 + 4 + j] +
					(velocity[i * 24 + 8 + j] - w[j + 8]) * normal[i * 24 + 8 + j])
				* -a;

			vec3<float>  f(
				normal[i * 12 + 0 + j] * mdp_adamp,
				normal[i * 12 + 4 + j] * mdp_adamp,
				normal[i * 12 + 8 + j] * mdp_adamp);
			float mdt = (massinv[i * 4 + j] * t);
			velocity[i * 12 + 0 + j] = velocity[i * 12 + 0 + j] + (g.x + f.x) * mdt;
			velocity[i * 12 + 4 + j] = velocity[i * 12 + 4 + j] + (g.y + f.y) * mdt;
			velocity[i * 12 + 8 + j] = velocity[i * 12 + 8 + j] + (g.z + f.z) * mdt;
			posn_old[i * 12 + 0 + j] = position[i * 12 + 0 + j];
			posn_old[i * 12 + 4 + j] = position[i * 12 + 4 + j];
			posn_old[i * 12 + 8 + j] = position[i * 12 + 8 + j];
			position[i * 12 + 0 + j] = position[i * 12 + 0 + j] + velocity[i * 12 + 0 + j] * d;
			position[i * 12 + 4 + j] = position[i * 12 + 4 + j] + velocity[i * 12 + 4 + j] * d;
			position[i * 12 + 8 + j] = position[i * 12 + 8 + j] + velocity[i * 12 + 8 + j] * d;
		}

	float bias = (cloth_bias);
	__int64 timestart = 0;
	for (int iter = 0; iter < cloth_solver_count; iter++) // even
	{
		if (iter == 1)timestart = __rdtsc();
		for (int i = 0; i < this->constraints_count; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				Constraint &s = constraints[i];
				int a = s.a;
				int b = s.b;
				vec3<float> v(
					position[b * 12 + 0 + j] - position[a * 12 + 0 + j],
					position[b * 12 + 4 + j] - position[a * 12 + 4 + j],
					position[b * 12 + 8 + j] - position[a * 12 + 8 + j]
					);
				// some hand tuning to avoid div and sqrt instruction and 
				// provide a fair apples to apples performance comparison.
				// 64 bit compilation seems to be optimal, 32 bit compilation might not be
				__m128 d = _mm_set_ss(dot(v, v));
				__m128 invmag = _mm_rsqrt_ss(d);
				float  m = (_mm_mul_ss(d, invmag)).m128_f32[0];  // dont want to have to write out by hand, hopefully this doesn't cause a register spill
				vec3<float> u = v * invmag.m128_f32[0];
				float dif = (m - (s.restlen))*bias;
				float difa = dif * massinv[a * 4 + j];
				float difb = dif * massinv[b * 4 + j];
				position[a * 12 + 0 + j] = position[a * 12 + 0 + j] + u.x* difa;
				position[a * 12 + 4 + j] = position[a * 12 + 4 + j] + u.y* difa;
				position[a * 12 + 8 + j] = position[a * 12 + 8 + j] + u.z* difa;
				position[b * 12 + 0 + j] = position[b * 12 + 0 + j] - u.x* difb;
				position[b * 12 + 4 + j] = position[b * 12 + 4 + j] - u.y* difb;
				position[b * 12 + 8 + j] = position[b * 12 + 8 + j] - u.z* difb;
			}
		}
		bias = bias * (cloth_bias_decay);
	}
	this->simcycles = Round((float)(__rdtsc() - timestart) / (cloth_solver_count - 1) / constraints_count / 4, 0.1f);
}

void ConstraintNetwork::SimulateSIMD4()
{

	vec3<__m128> *position = this->position;    // putting into local variables avoids readback performance issue
	vec3<__m128> *posn_old = this->posn_old;
	vec3<__m128> *velocity = this->velocity;
	vec3<__m128> *normal = this->normal;
	__m128       *massinv = this->massinv;

	__m128       a = _mm_set1_ps(this->damp_air);
	vec3<__m128> g = vec3<__m128>(cloth_gravity);
	vec3<__m128> w = *swind;  // absolute direction of the wind
	__m128       d = _mm_set1_ps(cloth_velocity_damp*cloth_timestep);
	__m128       t = _mm_set1_ps(cloth_timestep);
	for (int i = 0; i < points_count; i++)
	{
		vec3<__m128> f = -normal[i] * (dot(velocity[i] - w, normal[i]) * a);  // calculate wind force at point 
		velocity[i] = velocity[i] + (g + f) * (massinv[i] * t);   // update velocity based on wind force and gravity
		posn_old[i] = position[i];                        // save old point position
		position[i] = position[i] + velocity[i] * d;               // update potential new position based on velocity
	}

	__m128 bias = _mm_set1_ps(cloth_bias);
	__m128 epsilon = _mm_set1_ps(0.000001f);
	__int64 timestart = 0;
	for (int iter = 0; iter < cloth_solver_count; iter++)   // main solver loop for AVX 256-bit version
	{
		if (iter == 1) timestart = __rdtsc();
		for (int i = 0; i < this->constraints_count; i++)
		{
			Constraint &s = constraints[i];
			short int a = s.a; // endpoint indices cached in temporaries to avoid compiler generating code to re-read these values from memory
			short int b = s.b;

			vec3<__m128> v = (position[b] - position[a]);   // vector between positions 
			__m128 dp = dot(v, v) + epsilon;     // add an epsilon to avoid rsqrt of 0
			__m128 invm = _mm_rsqrt_ps(dp);  // inverse of distance between endpoints
			v = v * invm;   // v = v/||v||   unit length vector 

			__m128 mag = invm * dp;  // distance between endpoints.  this works by mathematics:  sqrt(dp) == dp * 1/sqrt(dp)
			__m128 dif = (mag - _mm_set1_ps(s.restlen))*bias;  // how much we have to adjust based on difference with restlength and bias

																  //for some reason compiler was spilling registers with the following 2 lines of code:
																  //  position[a] += v * dif * massinv[a];
																  //  position[b] -= v * dif * massinv[b];
																  // Sigh, So instead of using  vec3<__m128> class, explicitly write out x,y,z separately:
			__m128 difma = dif * massinv[a];
			__m128 difmb = dif * massinv[b];
			if (a == 65 || b == 65)
				a = a;
			position[a].x = position[a].x + v.x* (difma);
			position[a].y = position[a].y + v.y* (difma);
			position[a].z = position[a].z + v.z* (difma);
			position[b].x = position[b].x - v.x* (difmb);
			position[b].y = position[b].y - v.y* (difmb);
			position[b].z = position[b].z - v.z* (difmb);
		}
		bias = bias * _mm_set1_ps(cloth_bias_decay);
	}
	this->simcycles = Round((float)(__rdtsc() - timestart) / (cloth_solver_count - 1) / constraints_count / 4, 0.1f);

}


//void ConstraintNetwork::SimulateSIMD4()
//{
//	__m128 *position = reinterpret_cast<__m128 *>(this->position);
//	__m128 *posn_old = reinterpret_cast<__m128 *>(this->posn_old);
//	__m128 *velocity = reinterpret_cast<__m128 *>(this->velocity);
//	__m128 *normal = reinterpret_cast<__m128 *>(this->normal);
//	__m128 *massinv = reinterpret_cast<__m128 *>(this->massinv);
//
//	vec3<__m128>  g = vec3<__m128>(cloth_gravity);
//	__m128        a = _mm_set1_ps(this->damp_air);
//	__m128       *w = reinterpret_cast<__m128 *>(swind); // absolute direction of the wind
//	__m128        d = _mm_set1_ps(cloth_velocity_damp*cloth_timestep);
//	__m128        t = _mm_set1_ps(cloth_timestep);
//
//	// the next two loops aren't a perforamnce hotspot, so didn't worry too much about line-by-line optimization
//	for (int i = 0; i < points_count; i++)
//	{
//		__m128  mdp_adamp =
//			((velocity[i * 6 + 0] - w[0]) * normal[i * 6 + 0] +
//				(velocity[i * 6 + 2] - w[2]) * normal[i * 6 + 2] +
//				(velocity[i * 6 + 4] - w[4]) * normal[i * 6 + 4])
//			* -a;
//
//		vec3<__m128>  f(
//			normal[i * 6 + 0] * mdp_adamp,
//			normal[i * 6 + 2] * mdp_adamp,
//			normal[i * 6 + 4] * mdp_adamp);
//		__m128 mdt = (massinv[i * 2] * t);
//		velocity[i * 6 + 0] = velocity[i * 6 + 0] + (g.x + f.x) * mdt;
//		velocity[i * 6 + 2] = velocity[i * 6 + 2] + (g.y + f.y) * mdt;
//		velocity[i * 6 + 4] = velocity[i * 6 + 4] + (g.z + f.z) * mdt;
//		posn_old[i * 6 + 0] = position[i * 6 + 0];
//		posn_old[i * 6 + 2] = position[i * 6 + 2];
//		posn_old[i * 6 + 4] = position[i * 6 + 4];
//		position[i * 6 + 0] = position[i * 6 + 0] + velocity[i * 6 + 0] * d;
//		position[i * 6 + 2] = position[i * 6 + 2] + velocity[i * 6 + 2] * d;
//		position[i * 6 + 4] = position[i * 6 + 4] + velocity[i * 6 + 4] * d;
//	}
//	for (int i = 0; i < points_count; i++)  //odd
//	{
//		__m128  mdp_adamp =
//			((velocity[i * 6 + 1] - w[1]) * normal[i * 6 + 1] +
//				(velocity[i * 6 + 3] - w[3]) * normal[i * 6 + 3] +
//				(velocity[i * 6 + 5] - w[5]) * normal[i * 6 + 5])
//			* -a;
//
//		vec3<__m128>  f(
//			normal[i * 6 + 1] * mdp_adamp,
//			normal[i * 6 + 3] * mdp_adamp,
//			normal[i * 6 + 5] * mdp_adamp);
//		__m128 mdt = (massinv[i * 2 + 1] * t);
//		velocity[i * 6 + 1] = velocity[i * 6 + 1] + (g.x + f.x) * mdt;
//		velocity[i * 6 + 3] = velocity[i * 6 + 3] + (g.y + f.y) * mdt;
//		velocity[i * 6 + 5] = velocity[i * 6 + 5] + (g.z + f.z) * mdt;
//		posn_old[i * 6 + 1] = position[i * 6 + 1];
//		posn_old[i * 6 + 3] = position[i * 6 + 3];
//		posn_old[i * 6 + 5] = position[i * 6 + 5];
//		position[i * 6 + 1] = position[i * 6 + 1] + velocity[i * 6 + 1] * d;
//		position[i * 6 + 3] = position[i * 6 + 3] + velocity[i * 6 + 3] * d;
//		position[i * 6 + 5] = position[i * 6 + 5] + velocity[i * 6 + 5] * d;
//	}
//
//	// main solver loop 128 bit version
//	__m128 bias = _mm_set1_ps(cloth_bias);
//	__m128 epsilon = _mm_set1_ps(0.000001f);
//	__int64 timestart = 0;
//	for (int iter = 0; iter < cloth_solver_count; iter++)
//	{
//		if (iter == 1) timestart = __rdtsc();
//		for (int group = 0; group < 2; group++)
//		{
//			__m128 *positionss = position + group;
//			__m128 *massinvs = massinv + group;
//			for (int i = 0; i < this->constraints_count; i++)
//			{
//				Constraint &s = constraints[i];
//				int a = s.a;
//				int b = s.b;
//				vec3<__m128> v(
//					positionss[b * 6 + 0] - positionss[a * 6 + 0],
//					positionss[b * 6 + 2] - positionss[a * 6 + 2],
//					positionss[b * 6 + 4] - positionss[a * 6 + 4]
//					);
//				__m128  dp = (v.x*v.x + v.y*v.y + v.z*v.z + epsilon);
//				__m128  invm = _mm_rsqrt_ps(dp);
//				vec3<__m128> u = v * invm;
//				__m128 m = (dp*invm);
//				__m128 dif = (m - _mm_set1_ps(s.restlen))*bias;
//				__m128 difa = dif * massinvs[a * 2];
//				__m128 difb = dif * massinvs[b * 2];
//				positionss[a * 6 + 0] = positionss[a * 6 + 0] + u.x* difa;
//				positionss[a * 6 + 2] = positionss[a * 6 + 2] + u.y* difa;
//				positionss[a * 6 + 4] = positionss[a * 6 + 4] + u.z* difa;
//				positionss[b * 6 + 0] = positionss[b * 6 + 0] - u.x* difb;
//				positionss[b * 6 + 2] = positionss[b * 6 + 2] - u.y* difb;
//				positionss[b * 6 + 4] = positionss[b * 6 + 4] - u.z* difb;
//			}
//		}
//		bias = bias * _mm_set1_ps(cloth_bias_decay);
//	}
//	this->simcycles = Round((float)(__rdtsc() - timestart) / (cloth_solver_count - 1) / constraints_count / 8, 0.1f);
//}

#pragma optimize("",off)

void ConstraintNetwork::Simulate()
{
	if (cloth_simdwidth == 1)
		SimulateSerial();
	else if (cloth_simdwidth == 4)
		SimulateSIMD4();
	RoomCollision();
	CalcVelocity();
	UpdateBBoxLimits();
	CalcNormals();
	PhysicsToGraphics();  // transpose SOA data into AOS vertexbuffer in parent Mesh class 
}

//------------------------------------------------------------------------------------------------------
// 
//  The following code section manages the objects in our cloth physics scene.  
//  Also we provide a simple ad hoc wind environment.
//

std::vector<ConstraintNetwork *> cloth_meshes;
float              cloth_windspot = 0.0f;
float3             cloth_winddir(5, 5, 4);

void windupdate()
{
	// The idea is that there is a fan that moves forward or backward along the y axis.
	// The direction of the wind changes after each pass.
	static float3 w[16] =
	{
		float3(5,10,3),
		float3(30,10,5),
		float3(30,-10,5),
		float3(-30,10,10),

		float3(-30,-10,10),
		float3(30,10,10),
		float3(30,10,5),
		float3(30,10,10),

		float3(-20,-40,10),
		float3(-20,-20,10),
		float3(20,10,10),
		float3(30, 10,10),

		float3(0,-20,5),
		float3(-10,-1,5),
		float3(30,10,10),
		float3(-100,-200,10),
	};
	cloth_windspot += 0.05f;
	if (cloth_windspot > 32)
	{
		cloth_windspot = -10;
		static int k = 0;
		k++;
		k %= 16;
		cloth_winddir = w[k];
	}
}

float3 windat(const float3& p)
{
	// the windat() function returns the wind velocity at a specific spot.  
	float y = (cloth_winddir.y > 0) ? cloth_windspot : 22.0f - cloth_windspot;
	float e = powf(0.5f, 1 + fabsf(p.y - y));
	return cloth_winddir * e;
}

void spawncloths(int width, float size,
	int groups_count,
	float3 spawnpoint,
	float  damp_air,
	float3 spacing = float3(0, 2.0f, 0),
	int pin = 3
	)
{
	// simple fabric generation routine that creates a typical square fabric.
	// better to use a real pipeline to generate these, this is just for testing.
	for (int g = 0; g < groups_count; g++)
	{
		ConstraintNetwork *c = new ConstraintNetwork(width, width, size);

		for (int j = 0; j < 4; j++)
			if (pin) // pin the corners:
			{
				if (pin & 1) c->PointSetInvMass(j*(width*width) + 0, 0);
				if (pin & 2) c->PointSetInvMass(j*(width*width) + width - 1, 0);
				if (pin & 4) c->PointSetInvMass(j*(width*width) + ((width - 1)*width), 0);
				if (pin & 8) c->PointSetInvMass(j*(width*width) + ((width - 1)*width) + width - 1, 0);
			}
		for (int i = 0; i < width*width * 4; i++)
		{
			c->Point(i) += spawnpoint + spacing*(float)(i / (width*width));
		}
		spawnpoint += spacing*4.0f;
		c->damp_air = damp_air;
		cloth_meshes.push_back(c);
	}
}


void ClothInit()
{
	_mm_setcsr(_mm_getcsr() | _MM_FLUSH_ZERO_MASK);

	//  Quick ad hoc generation of a scene full of cloth pieces.
	//  This wouldn't normally be done in code, but rather use a data file.
	spawncloths(16, 2.0f, 1, float3(2.f, 0.0f, 5.0f), 10.0f);
	spawncloths(12, 2.0f, 2, float3(2.f, 8.0f, 5.0f), 10.0f);

	spawncloths(16, 2.0f, 1, float3(-2.f, 0.0f, 5.0f), 10.0f);
	spawncloths(12, 2.0f, 2, float3(-2.f, 8.0f, 5.0f), 10.0f);

	spawncloths(12, 2.0f, 2, float3(-2.f, 0.0f, 0.5f), 30.0f, float3(0, 1, 0), 0);
	spawncloths(12, 2.0f, 3, float3(2.f, 0.0f, 0.5f), 30.0f, float3(0, 1, 0), 0);

	spawncloths(10, 2.0f, 3, float3(-5.f, 0.0f, 1.0f), 60.0f, float3(0, 1, 0), 0);
	spawncloths(10, 2.0f, 3, float3(5.f, 0.0f, 1.0f), 60.0f, float3(0, 1, 0), 0);

	spawncloths(16, 2.5f, 3, float3(-8.5f, -2.0f, 4.0f), 15.0f, float3(0, 1.5f, 0), 1);
	spawncloths(16, 2.5f, 3, float3(8.5f, -2.0f, 4.0f), 15.0f, float3(0, 1.5f, 0), 8);

	spawncloths(8, 1.25f, 3, float3(-9.0f, -3.5f, 5.5f), 10.0f, float3(0, 1.5f, 0), 5);
	spawncloths(8, 1.25f, 3, float3(9.0f, -3.5f, 5.5f), 10.0f, float3(0, 1.5f, 0), 10);

	spawncloths(8, 0.5f, 5, float3( 0.0f, 2.0f, 1.0f), 60.0f, float3(0.0f, 0.03f, 0.03f), 0);
	spawncloths(8, 0.5f, 5, float3(-0.1f, 2.0f, 1.2f), 60.0f, float3(0.0f, 0.03f, 0.03f), 0);
	spawncloths(8, 0.5f, 5, float3( 0.1f, 2.0f, 1.2f), 60.0f, float3(0.0f, 0.03f, 0.03f), 0);
}

float ClothSimulation()
{
	// the main loop update for our cloth physics scene
	assert(cloth_meshes.size()); // forget to call ClothInit()?

								// update our wind scene and gather local wind vectors for each cloth section from each group
	windupdate();
	for(auto &m : cloth_meshes) 
	{
		vec3<__m128> c = (*m->sbmin + *m->sbmax) * _mm_set1_ps(0.5f);
		for (int i = 0; i < 4; i++)
			soafloat3(*m->swind, i) = windat(soafloat3(c, i));
	}

	for (auto &m : cloth_meshes)
	{
		m->Simulate();
	}
	return cloth_meshes.size() ? cloth_meshes[0]->simcycles : 0;  // for displaying cpu cycles per constraint update - just using first one
}

int &ClothSolverSIMDWidth()
{
	return cloth_simdwidth;
}






Mesh MeshFlatShadeTex(const WingMesh &m)  // procedurally generate normals and texture coords mesh
{
	return MeshFlatShadeTex(m.verts, WingMeshTris(m));
}

int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try  // int main(int argc, char *argv[])
{
	DXWin dxwin("updated intel simd cloth demo", { 1920,1080 });
	dxwin.ViewAngle = 70.0f;
	dxwin.vsync = 0;
	WingMesh cube_wm = WingMeshCube(1.0f);
	WingMesh oct_wm = WingMeshDual(cube_wm, 1.4f);
	auto cube_mesh = MeshFlatShadeTex(cube_wm);
	auto oct_mesh  = MeshFlatShadeTex(oct_wm );
	auto room = MeshFlatShadeTex(WingMeshBox(cloth_rmin, cloth_rmax));
	room.hack = float4(0.15f, 0.3f, 0.6f, 1.0f);
	for (auto &t : room.tris)
		std::swap(t.x, t.y);
	for (auto &v : room.verts)
		v.texcoord *= 0.1f;
	int frame = 0;
	bool stereo = 0;
	ClothInit();
	for (auto &m : cloth_meshes)
		m->mesh.hack = float4( 1.0f,1.0f,1.0f,1.0f );

	float3 cspawn(0.0f, 2.0f, 1.0f);
	dxwin.keyboardfunc = [&cspawn](int k, int, int)
	{
		if (k == 'k')
		{
			auto b = cloth_meshes.size();
			spawncloths(8, 0.5f, 5, cspawn+=float3(0,0,0.1f), 60.0f, float3(0.0f, 0.03f, 0.03f), 0);
			while (b < cloth_meshes.size())
				cloth_meshes[b++]->mesh.hack = { 1,1,0.25f,1 };
		}
	};
	while (dxwin.WindowUp())
	{
		frame++;
		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		frame++;
		ClothSimulation();

		cube_mesh.hack = { 0.5f + 0.5f*sinf(frame*0.0002f), 1, 1, 1 };
		cube_mesh.pose = { {0,10,0.0f}, { 0, 0, sinf(frame*0.0001f), cosf(frame*0.0001f) } };

		oct_mesh.pose = { { -2.5f, 16.3f, -0.25f }, { 0, 0, -sinf(frame*0.0001f), cosf(frame*0.0001f) } };
		oct_mesh.hack = { 1, 0, 1, 1 };

		std::vector<Mesh*> meshes;
		meshes.push_back(&cube_mesh);
		meshes.push_back(&oct_mesh);
		meshes.push_back(&room);
		for (auto &m : cloth_meshes)
			meshes.push_back(&m->mesh);
		(dxwin.*(stereo ? &DXWin::RenderStereo : &DXWin::RenderScene))({ { 0, -7, 1.5f }, normalize(float4(1.0f, 0, 0, 1.0f)) }, meshes);
	}
}
catch (const char *c) 
{	
	MessageBox(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}

