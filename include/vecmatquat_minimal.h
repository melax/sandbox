//
// minimal set of well understood vec,quat,mat 3d math routines as needed.
// Following/extending hlsl conventions.
// 
// 2014 update, just inlined the needed vector things in this file.
// original code was from 1998 and wasn't using the best conventions.
// For example, quaternions are xyzw, not rxyz now.

#ifndef GENERIC_VECMATQUAT_MINIMAL_H
#define GENERIC_VECMATQUAT_MINIMAL_H

#include <stdio.h>
#include <math.h>
#include <tuple>  // for std::pair
#include <algorithm> 

struct int3 
{
	int x, y, z;
	int &operator[](int i){ return (&x)[i]; }
	const int &operator[](int i)const { return (&x)[i]; }
};
inline bool operator==(const int3 &a, const int3 &b)      { return a.x == b.x && a.y == b.y && a.z == b.z; }

struct int4 
{
	int x, y, z, w;
	int &operator[](int i){ return (&x)[i]; }
	const int &operator[](int i)const { return (&x)[i]; }
};


class float3 {
  public:
	float x,y,z;
	float3(float x,float y,float z):x(x),y(y),z(z){}
	float3():x(0),y(0),z(0){}
	float &operator[](int i){ return (&x)[i]; }
	const float &operator[](int i)const { return (&x)[i]; }
};


inline bool operator==(const float3 &a, const float3 &b)  { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline float3 operator+(const float3 &a, const float3 &b) { return{ a.x + b.x, a.y + b.y, a.z + b.z }; }
inline float3 operator-(const float3 &v)                  { return { -v.x , -v.y, -v.z }; }
inline float3 operator-(const float3 &a, const float3 &b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline float3 operator*(const float3 &v,float s)          { return { v.x*s , v.y*s, v.z*s }; }  
inline float3 operator*(float s,const float3 &v)          { return v*s;}
inline float3 operator/(const float3 &v,float s)          { return v * (1.0f/s) ;}
inline float  dot  (const float3 &a, const float3 &b)     { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline float3 cross(const float3 &a, const float3 &b)     { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline float  magnitude(const float3 &v) { return sqrtf(dot(v, v)); }
inline float3 normalize(const float3 &v) { return v / magnitude(v); }
inline float3 cmin(const float3 &a, const float3 &b)      { return{ std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) }; }
inline float3 cmax(const float3 &a, const float3 &b)      { return{ std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) }; }
inline float3 planelineintersection(const float3 &n, float d, const float3 &p1, const float3 &p2)
{
	// returns the point where the line p1-p2 intersects the plane n&d
	float3 dif = p2 - p1;
	float dn = dot(n, dif);
	float t = -(d + dot(n, p1)) / dn;
	return p1 + (dif*t);
}
inline int maxdir(const float3 *p, int count, const float3 &dir)
{
	if (count == 0) 
		return -1;
	int m = 0;
	for (int i = 1; i<count; i++)
	{
		if (dot(p[i], dir)>dot(p[m], dir)) m = i;
	}
	return m;
}
inline float3 TriNormal(const float3 &v0, const float3 &v1, const float3 &v2)  // normal of the triangle with vertex positions v0, v1, and v2
{
	float3 cp = cross(v1 - v0, v2 - v1);
	float m = magnitude(cp);
	if (m == 0) return float3(1, 0, 0);
	return cp*(1.0f / m);
}

/*
class float3x3
{
 public:
	float3 x,y,z;
	float3x3(){x=float3(1.0f,0.0f,0.0f);
	         y=float3(0.0f,1.0f,0.0f);
	         z=float3(0.0f,0.0f,1.0f);};
	float3x3(float3 x, float3 y, float3 z) :x(x), y(y), z(z){}
};
inline float3x3 transpose(const float3x3 &m) {
	return float3x3(float3(m.x.x, m.y.x, m.z.x),
		float3(m.x.y, m.y.y, m.z.y),
		float3(m.x.z, m.y.z, m.z.z));
}
inline float3 mul(const float3x3 &m, const float3 &v){  // m is assumed to be column major
	return m.x*v.x + m.y*v.y + m.z*v.z; 
}

*/

class Quaternion
{
 public:
	 float x,y,z,w;
	 const float3& xyz() const { return *((float3*)&x); }
	 float3&       xyz()       { return *((float3*)&x); }
	 //Quaternion(){ x = y = z = 0.0f; w = 1.0f; };
	 Quaternion(float _x,float _y,float _z,float _w){x=_x;y=_y;z=_z;w=_w;};
	 
};
inline Quaternion QuatFromAxisAngle(const float3 &axis, float t) { auto v=normalize(axis)*sinf(t / 2.0f); return{v.x,v.y,v.z,cosf(t/2.0f)}; }
inline std::pair<float3, float> AxisAngleFromQuat(const Quaternion &q) { auto a = acos(q.w)*2.0f; return std::make_pair(q.xyz() / sinf(a / 2.0f), a); }
inline float3 qxdir(const Quaternion &q) {return float3(1-2*(q.y*q.y+q.z*q.z),  2*(q.x*q.y+q.w*q.z),  2*(q.x*q.z-q.w*q.y));}
inline float3 qydir(const Quaternion &q) {return float3(  2*(q.x*q.y-q.w*q.z),1-2*(q.x*q.x+q.z*q.z),  2*(q.y*q.z+q.w*q.x));}
inline float3 qzdir(const Quaternion &q) {return float3(  2*(q.x*q.z+q.w*q.y),  2*(q.y*q.z-q.w*q.x),1-2*(q.x*q.x+q.y*q.y));}
//inline float3x3  qmatrix(const Quaternion &q) { return float3x3(qxdir(q), qydir(q), qzdir(q)); }
inline Quaternion qmul(const Quaternion &a, const Quaternion &b)
{
	return{
		a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
		a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
		a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
		a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
	};
}

inline Quaternion operator*(const Quaternion &a, float b) { return{ a.x*b, a.y*b, a.z*b, a.w*b }; }
inline Quaternion operator+(const Quaternion &a, const Quaternion &b) { return{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
inline float dot(const Quaternion &a, const Quaternion &b) { return  a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
inline Quaternion qconj(const Quaternion &q) { return {-q.x,-q.y,-q.z,q.w}; } 
inline float3 qrot(const Quaternion &q, const float3 &v) { return qmul(qmul(q , Quaternion(v.x,v.y,v.z,0)), qconj(q)).xyz() ; }  // q*v*conj(q) 


inline Quaternion slerp(const Quaternion &a_, const Quaternion &b, float interp)
{
	Quaternion a = (dot(a, b) <0.0) ? qconj(a_) : a_;
	float theta = (float)acos(dot(a, b));
	if (theta == 0.0f) { return(a); }
	return a*(float)(sin(theta - interp*theta) / sin(theta)) + b*(float)(sin(interp*theta) / sin(theta));
}


#endif
