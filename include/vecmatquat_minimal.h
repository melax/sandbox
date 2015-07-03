//
// minimal set of well understood vec,quat,mat 3d math routines as needed.
// Following/extending hlsl conventions.
// 
//  defines the same set of structs as the alternative file vecmatquat.h.   
//  This vecmatquat_minimal.h version is meant to enable copy-paste just the code snippets that are needed.
//  

#ifdef VECMATQUAT_FULL_H
#error
#endif


#ifndef VECMATQUAT_H
#define VECMATQUAT_H
#define VECMATQUAT_MINIMAL_H

#include <stdio.h>
#include <math.h>
#include <tuple>  // for std::pair
#include <algorithm> 

struct int2
{
	int x, y;
	int &operator[](int i){ return (&x)[i]; }
	const int &operator[](int i)const { return (&x)[i]; }
};
inline bool operator==(const int2 &a, const int2 &b)      { return a.x == b.x && a.y == b.y; }

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
	float x, y, z;
	float3(float x, float y, float z) :x(x), y(y), z(z){}
	float3() :x(0), y(0), z(0){}
	float &operator[](int i){ return (&x)[i]; }
	const float &operator[](int i)const { return (&x)[i]; }
};



inline bool   operator==(const float3 &a, const float3 &b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool   operator!=(const float3 &a, const float3 &b) { return !(a==b); }
inline float3 operator+(const float3 &a, const float3 &b)  { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline float3 operator-(const float3 &v)                   { return { -v.x , -v.y, -v.z }; }
inline float3 operator-(const float3 &a, const float3 &b)  { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline float3 operator*(const float3 &v, float s)          { return { v.x*s, v.y*s, v.z*s }; }
inline float3 operator*(float s,const float3 &v)           { return v*s;}
inline float3 operator/(const float3 &v,float s)           { return v * (1.0f/s) ;}
inline float3 operator+=(float3 &a, const float3 &b)       { return a = a + b; }
inline float3 operator-=(float3 &a, const float3 &b)       { return a = a - b; }
inline float3 operator*=(float3 &v, const float &s )       { return v = v * s; }
inline float3 operator/=(float3 &v, const float &s )       { return v = v / s; }
inline float  dot(const float3 &a, const float3 &b)        { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline float3 cross(const float3 &a, const float3 &b)      { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline float  magnitude(const float3 &v)                   { return sqrtf(dot(v, v)); }
inline float3 normalize(const float3 &v)                   { return v / magnitude(v); }
inline float3 cmin(const float3 &a, const float3 &b)       { return { std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) }; }
inline float3 cmax(const float3 &a, const float3 &b)       { return { std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) }; }
inline float3 vabs(const float3 &v)                        { return { std::abs(v.x)     , std::abs(v.y)     , std::abs(v.z) }; }

class float3x3
{
 public:
	float3 x,y,z;
	float3x3(){x=float3(1.0f,0.0f,0.0f);
	         y=float3(0.0f,1.0f,0.0f);
	         z=float3(0.0f,0.0f,1.0f);};
	float3x3(float3 x, float3 y, float3 z) :x(x), y(y), z(z){}
	float3 &operator[](int i){ return (&x)[i]; }
	const float3 &operator[](int i)const { return (&x)[i]; }
};

inline float3x3 operator*(const float3x3 &m, float s)         { return{ m.x*s, m.y*s, m.z*s }; }
inline float3x3 transpose(const float3x3 &m) {
	return float3x3(float3(m.x.x, m.y.x, m.z.x),
		float3(m.x.y, m.y.y, m.z.y),
		float3(m.x.z, m.y.z, m.z.z));
}
inline float3 mul(const float3x3 &m, const float3 &v){ return m.x*v.x + m.y*v.y + m.z*v.z; } // m is assumed to be column major
inline float  determinant(const float3x3& a)  { return a.x.x*(a.y.y*a.z.z - a.z.y*a.y.z) + a.x.y*(a.y.z*a.z.x - a.z.z*a.y.x) + a.x.z*(a.y.x*a.z.y - a.z.x*a.y.y); }
inline float3x3 adjoint(const float3x3 & a)
{
	return{ { a.y.y*a.z.z - a.z.y*a.y.z, a.z.y*a.x.z - a.x.y*a.z.z, a.x.y*a.y.z - a.y.y*a.x.z },
	{ a.y.z*a.z.x - a.z.z*a.y.x, a.z.z*a.x.x - a.x.z*a.z.x, a.x.z*a.y.x - a.y.z*a.x.x },
	{ a.y.x*a.z.y - a.z.x*a.y.y, a.z.x*a.x.y - a.x.x*a.z.y, a.x.x*a.y.y - a.y.x*a.x.y } };
}
inline float3x3 inverse(float3x3 & a)  { return adjoint(a) *(1.0f / determinant(a)); }
inline float3x3 outerprod(const float3 &a, const float3 &b) { return {a*b.x, a*b.y, a*b.z }; }





class float4
{
 public:
	 float x,y,z,w;
	 const float3& xyz() const { return *((float3*)&x); }
	 float3&       xyz()       { return *((float3*)&x); }
	 //float4(){ x = y = z = 0.0f; w = 1.0f; };
	 float4(float _x,float _y,float _z,float _w){x=_x;y=_y;z=_z;w=_w;};
	 float &operator[](int i){ return (&x)[i]; }
	 const float &operator[](int i)const { return (&x)[i]; }

};

inline float4 operator*(const float4 &a, float b)         { return { a.x*b, a.y*b, a.z*b, a.w*b                 }; }
inline float4 operator+(const float4 &a, const float4 &b) { return { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
inline float4 operator-(const float4 &v)                  { return { -v.x, -v.y, -v.z, -v.w };                     }
inline float  dot(const float4 &a, const float4 &b)       { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;          }
inline float4 slerp(const float4 &v0, const float4 &v1, float t)
{
	float a = (float)acos(dot(v0, v1));
	if (a == 0.0f) { return(v0); }
	return v0*(float)(sin(a - t*a) / sin(a)) + v1*(float)(sin(t*a) / sin(a));
}

// Quaternion library:
inline float4 qmul(const float4 &a, const float4 &b)
{
	return{
		a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
		a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
		a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
		a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
	};
}inline float4 qconj(const float4 &q) { return {-q.x,-q.y,-q.z,q.w}; } 
inline float3 qrot (const float4 &q, const float3 &v) { return qmul(qmul(q , float4(v.x,v.y,v.z,0)), qconj(q)).xyz() ; }  // q*v*conj(q) 
inline float3 qxdir(const float4 &q) {return float3(1-2*(q.y*q.y+q.z*q.z),  2*(q.x*q.y+q.w*q.z),  2*(q.x*q.z-q.w*q.y));}
inline float3 qydir(const float4 &q) {return float3(  2*(q.x*q.y-q.w*q.z),1-2*(q.x*q.x+q.z*q.z),  2*(q.y*q.z+q.w*q.x));}
inline float3 qzdir(const float4 &q) {return float3(  2*(q.x*q.z+q.w*q.y),  2*(q.y*q.z-q.w*q.x),1-2*(q.x*q.x+q.y*q.y));}
//inline float3x3  qmatrix(const float4 &q) { return float3x3(qxdir(q), qydir(q), qzdir(q)); }

inline float4 qslerp(const float4 &q0, const float4 &q1, float t)
{
	return slerp(((dot(q0, q1) < 0.0) ? -q0 : q0), q1, t);   // slerp between two on the same hemisphere (common usage)
}
inline float4 QuatFromAxisAngle(const float3 &axis, float t) { auto v = normalize(axis)*sinf(t / 2.0f); return{ v.x, v.y, v.z, cosf(t / 2.0f) }; }
inline std::pair<float3, float> AxisAngleFromQuat(const float4 &q) { auto a = acos(q.w)*2.0f; return std::make_pair(q.xyz() / sinf(a / 2.0f), a); }


//-------- copied from geometric.h -------

inline float3 PlaneLineIntersection(const float3 &n, const float d, const float3 &p0, const float3 &p1)   // returns the point where the line p0-p2 intersects the plane n&d
{
	float3 dif = p1 - p0;
	float dn = dot(n, dif);
	float t = -(d + dot(n, p0)) / dn;
	return p0 + (dif*t);
}
inline int     argmax(const float a[], int n)
{
	int r = 0;
	for (int i = 1; i<n; i++)
	{
		if (a[i]>a[r])
		{
			r = i;
		}
	}
	return r;
}
inline float3 Orth(const float3& v)
{
	float3 absv = vabs(v);
	float3 u(1, 1, 1);
	u[argmax(&absv[0], 3)] = 0.0f;
	return normalize(cross(u, v));
}
inline float4 RotationArc(const float3 &v0_, const float3 &v1_)
{
	auto v0 = normalize(v0_);  // Comment these two lines out if you know its not needed.
	auto v1 = normalize(v1_);  // If vector is already unit length then why do it again?
	auto  c = cross(v0, v1);
	auto  d = dot(v0, v1);
	if (d <= -1.0f) { float3 a = Orth(v0); return float4(a.x, a.y, a.z, 0); } // 180 about any orthogonal axis axis
	auto  s = sqrtf((1 + d) * 2);
	return{ c.x / s, c.y / s, c.z / s, s / 2.0f };
}
inline float4 VirtualTrackBall(const float3 &cop, const float3 &cor, const float3 &dir1, const float3 &dir2)
{
	// Simple track ball functionality to spin stuf on the screen.
	//  cop   center of projection   cor   center of rotation
	//  dir1  old mouse direction    dir2  new mouse direction
	// Pretend there is a sphere around cor.    Take rotation
	// between apprx points where dir1 and dir2 intersect sphere.  
	float3 nrml = cor - cop; // compute plane 
	float fudgefactor = 1.0f / (magnitude(nrml) * 0.25f); // since trackball proportional to distance from cop
	nrml = normalize(nrml);
	float dist = -dot(nrml, cor);
	float3 u = (PlaneLineIntersection(nrml, dist, cop, cop + dir1) - cor) * fudgefactor;
	float m = magnitude(u);
	u = (m > 1) ? u / m : u - (nrml * sqrtf(1 - m*m));
	float3 v = (PlaneLineIntersection(nrml, dist, cop, cop + dir2) - cor) * fudgefactor;
	m = magnitude(v);
	v = (m>1) ? v / m : v - (nrml * sqrtf(1 - m*m));
	return RotationArc(u, v);
}
//--------------------------------



#endif // VECMATQUAT_H
