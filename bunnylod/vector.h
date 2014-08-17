//
// This module contains a bunch of well understood functions
// I apologise if the conventions used here are slightly
// different than what you are used to.
// 
 
// 2014 update, just inlined the needed vector things in this file.
// original code was from 1998 and wasn't using the best conventions.
// For example, quaternions are xyzw, not rxyz now.

#ifndef GENERIC_VECTOR_H
#define GENERIC_VECTOR_H

#include <stdio.h>
#include <math.h>


class float3 {
  public:
	float x,y,z;
	float3(float x,float y,float z):x(x),y(y),z(z){}
	float3():x(0),y(0),z(0){}
	operator float *() { return &x;};  // this lets glVertex3fv work with this class
};


inline float3 operator+(const float3 &a, const float3 &b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline float3 operator-(const float3 &v)                  { return { -v.x , -v.y, -v.z }; }
inline float3 operator-(const float3 &a, const float3 &b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline float3 operator*(const float3 &v,float s)          { return { v.x*s , v.y*s, v.z*s }; }  
inline float3 operator*(float s,const float3 &v)          { return v*s;}
inline float3 operator/(const float3 &v,float s)          { return v * (1.0f/s) ;}
inline float  dot  (const float3 &a, const float3 &b)     { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline float3 cross(const float3 &a, const float3 &b)     { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline float  magnitude(const float3 &v) { return sqrtf(dot(v, v)); }
inline float3 normalize(const float3 &v) { return v / magnitude(v); }

inline float3 planelineintersection(const float3 &n, float d, const float3 &p1, const float3 &p2)
{
	// returns the point where the line p1-p2 intersects the plane n&d
	float3 dif = p2 - p1;
	float dn = dot(n, dif);
	float t = -(d + dot(n, p1)) / dn;
	return p1 + (dif*t);
}

class matrix   // float3x3
{
 public:
	float3 x,y,z;
	matrix(){x=float3(1.0f,0.0f,0.0f);
	         y=float3(0.0f,1.0f,0.0f);
	         z=float3(0.0f,0.0f,1.0f);};
	matrix(float3 x,float3 y,float3 z):x(x),y(y),z(z){}
};
inline matrix transpose(const matrix &m) {
	return matrix(float3(m.x.x, m.y.x, m.z.x),
		float3(m.x.y, m.y.y, m.z.y),
		float3(m.x.z, m.y.z, m.z.z));
}
inline float3 operator*(const matrix &m_, const float3 &v){
	auto m = transpose(m_); // since column ordered
	return float3(dot(m.x, v), dot(m.y, v), dot(m.z, v));
}
inline matrix operator*(const matrix &a, const matrix &b){
	auto m = transpose(a);
	return matrix(m*b.x, m*b.y, m*b.z);
}

class Quaternion{
 public:
	 float x,y,z,w;
	 Quaternion(){x=y=z=0.0f;w=1.0f;};
	 Quaternion(float3 v,float t){v=normalize(v);w=(float)cos(t/2.0);v=v*(float)sin(t/2.0);x=v.x;y=v.y;z=v.z;};
	 Quaternion(float _x,float _y,float _z,float _w){x=_x;y=_y;z=_z;w=_w;};
	 float angle()const {return (float)(acos(w)*2.0);}
	 float3 axis()const {float3 a(x,y,z); return a*(float)(1/sin(angle()/2.0));}
	 float3 xdir()const {return float3(1-2*(y*y+z*z),  2*(x*y+w*z),  2*(x*z-w*y));}
	 float3 ydir()const {return float3(  2*(x*y-w*z),1-2*(x*x+z*z),  2*(y*z+w*x));}
	 float3 zdir()const {return float3(  2*(x*z+w*y),  2*(y*z-w*x),1-2*(x*x+y*y));}
	 matrix  getmatrix() const {return matrix(xdir(),ydir(),zdir());}
	 //operator matrix(){return getmatrix();}
};
inline Quaternion operator*(const Quaternion &a, const Quaternion &b)
{
	Quaternion c;
	c.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
	c.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
	c.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
	c.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
	return c;
}

inline Quaternion operator*(const Quaternion &a, float b) { return{ a.x*b, a.y*b, a.z*b, a.w*b }; }
inline Quaternion operator+(const Quaternion &a, const Quaternion &b) { return{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
inline float dot(const Quaternion &a, const Quaternion &b) { return  a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
inline Quaternion qconj(const Quaternion &q) { return {-q.x,-q.y,-q.z,q.w}; } 

inline float3 qrot(const Quaternion &q, const float3 &v) { return q.getmatrix() * v; }  // q*v*conj(q) 


inline Quaternion slerp(const Quaternion &a_, const Quaternion &b, float interp)
{
	Quaternion a = (dot(a, b) <0.0) ? qconj(a_) : a_;
	float theta = (float)acos(dot(a, b));
	if (theta == 0.0f) { return(a); }
	return a*(float)(sin(theta - interp*theta) / sin(theta)) + b*(float)(sin(interp*theta) / sin(theta));
}


#endif
