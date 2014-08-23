//
//
// Collection of useful 3d geometric routines such as projections, intersection, etc.
//
// this module doesn't implement any geometric types, i.e. no 'classes' or 'structs'.
//
// note:  these evolved over the years.
// todo:  still doing some reorg to make naming more consistent and pick the best and minimal set of routines here
//

#pragma once
#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <algorithm>   // for std::max_element()
#include "vecmatquat_minimal.h"

// still reorganizing little geometry functions, putting these here for now:


inline float3 PlaneLineIntersection(const float3 &n, const float d, const float3 &p0, const float3 &p1)   // returns the point where the line p0-p2 intersects the plane n&d
{
	float3 dif = p1 - p0;
	float dn = dot(n, dif);
	float t = -(d + dot(n, p0)) / dn;
	return p0 + (dif*t);
}

inline float3 LineProject(const float3 &p0, const float3 &p1, const float3 &a)
{
	// project point a on segment [p0,p1]
	float3 d = p1 - p0;
	float t = dot(d, (a - p0)) / dot(d, d);
	return p0 + d*t;
}


inline float LineProjectTime(const float3 &p0, const float3 &p1, const float3 &a)
{
	// project point a on segment [p0,p1]
	float3 d = p1 - p0;
	float t = dot(d, (a - p0)) / dot(d, d);
	return t;
}
inline float3 BaryCentric(const float3 &v0, const float3 &v1, const float3 &v2, float3 s)
{
	float3x3 m(v0, v1, v2);
	if (determinant(m) == 0)
	{
		int k = (magnitude(v1 - v2)>magnitude(v0 - v2)) ? 1 : 0;
		float t = LineProjectTime(v2, m[k], s);
		return float3((1 - k)*t, k*t, 1 - t);
	}
	return mul(inverse(m), s);
}
inline int tri_interior(const float3& v0, const float3& v1, const float3& v2, const float3& d)
{
	float3 b = BaryCentric(v0, v1, v2, d);
	return (b.x >= 0.0f && b.y >= 0.0f && b.z >= 0.0f);
}


inline  float3 PlaneProjectOf(const float3 &v0, const float3 &v1, const float3 &v2, const float3 &point)
{
	float3 cp = cross(v2 - v0, v2 - v1);
	float dtcpm = -dot(cp, v0);
	float cpm2 = dot(cp, cp);
	if (cpm2 == 0.0f)
	{
		return LineProject(v0, (magnitude(v1 - v0)>magnitude(v2 - v0)) ? v1 : v2, point);
	}
	return point - cp * (dot(cp, point) + dtcpm) / cpm2;
}



inline int maxdir(const float3 *p, int count, const float3 &dir)
{
	if (count == 0)
		return -1;
	return std::max_element(p, p + count, [dir](const float3 &a, const float3 &b){return dot(a, dir) < dot(b, dir); }) - p;
}
inline float3 TriNormal(const float3 &v0, const float3 &v1, const float3 &v2)  // normal of the triangle with vertex positions v0, v1, and v2
{
	float3 cp = cross(v1 - v0, v2 - v1);
	float m = magnitude(cp);
	if (m == 0) return float3(1, 0, 0);
	return cp*(1.0f / m);
}


int     argmax(const float a[], int n)
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

// still in the process of rearranging basic math and geom routines, putting these here for now...

float3 Orth(const float3& v)
{
	float3 absv = vabs(v);
	float3 u(1, 1, 1);
	u[argmax(&absv[0], 3)] = 0.0f;
	return normalize(cross(u, v));
}
float4 RotationArc(const float3 &v0_, const float3 &v1_)
{
	auto v0 = normalize(v0_);  // Comment these two lines out if you know its not needed.
	auto v1 = normalize(v1_);  // If vector is already unit length then why do it again?
	auto  c = cross(v0, v1);
	auto  d = dot(v0, v1);
	if (d <= -1.0f) { float3 a = Orth(v0); return float4(a.x, a.y, a.z, 0); } // 180 about any orthogonal axis axis
	auto  s = sqrtf((1 + d) * 2);
	return{ c.x / s, c.y / s, c.z / s, s / 2.0f };
}


float4 VirtualTrackBall(const float3 &cop, const float3 &cor, const float3 &dir1, const float3 &dir2)
{
	// from game programming gems.  track ball functionality to spin stuf on the screen
	//  cop   center of projection   cor   center of rotation
	//  dir1  old mouse direction   dir2  new mouse direction
	// Pretend there is a sphere around cor.  Then find the points
	// where dir1 and dir2 intersect that sphere.  Find the
	// rotation that takes the first point to the second.
	float3 nrml = cor - cop; // compute plane 
	float fudgefactor = 1.0f / (magnitude(nrml) * 0.25f); // since trackball proportional to distance from cop
	nrml = normalize(nrml);
	float dist = -dot(nrml, cor);
	float3 u = ( PlaneLineIntersection(nrml, dist, cop, cop + dir1) - cor) * fudgefactor;
	float m = magnitude(u);
	u = (m > 1) ? u / m : u - (nrml * sqrtf(1 - m*m));
	float3 v = ( PlaneLineIntersection(nrml, dist, cop, cop + dir2) - cor) * fudgefactor;
	m = magnitude(v);
	v = (m>1) ? v / m : v - (nrml * sqrtf(1 - m*m));
	return RotationArc(u, v);
}

#endif //GEOMETRIC_H