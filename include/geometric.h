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

#include <algorithm>   // for std::max_element() used by maxdir and supportmap funcs
#include <limits>      // for std::numeric_limits used by Extents()
#include <vector>
#include "vecmatquat.h"

// still reorganizing little geometry functions, putting these here for now:

#define COPLANAR   (0)
#define UNDER      (1)
#define OVER       (2)
#define SPLIT      (OVER|UNDER)
#define PAPERWIDTH (0.0001f)  

inline float4x4 MatrixFromRotationTranslation(const float4 & rotationQuat, const float3 & translationVec)    { return{ { qxdir(rotationQuat), 0 }, { qydir(rotationQuat), 0 }, { qzdir(rotationQuat), 0 }, { translationVec, 1 } }; }


struct Pose // Value type representing a rigid transformation consisting of a translation and rotation component
{
	float3      position;
	float4      orientation;

	Pose(const float3 & p, const float4 & q) : position(p), orientation(q) {}
	Pose() : Pose({ 0, 0, 0 }, { 0, 0, 0, 1 }) {}

	Pose        Inverse() const                             { auto q = qconj(orientation); return{ qrot(q, -position), q }; }
	float4x4    Matrix() const                              { return MatrixFromRotationTranslation(orientation, position); }

	float3      operator * (const float3 & point) const     { return position + qrot(orientation, point); }
	Pose        operator * (const Pose & pose) const        { return{ *this * pose.position, qmul(orientation, pose.orientation) }; }
};


inline float3 PlaneLineIntersection(const float3 &n, const float d, const float3 &p0, const float3 &p1)   // returns the point where the line p0-p2 intersects the plane n&d
{
	float3 dif = p1 - p0;
	float dn = dot(n, dif);
	float t = -(d + dot(n, p0)) / dn;
	return p0 + (dif*t);
}
inline float3 PlaneLineIntersection(const float4 &plane, const float3 &p0, const float3 &p1) { return PlaneLineIntersection(plane.xyz(), plane.w, p0, p1); } // returns the point where the line p0-p2 intersects the plane n&d

inline int PlaneTest(const float4 &plane, const float3 &v, float epsilon = PAPERWIDTH) {
	float a = dot(v, plane.xyz()) + plane.w;
	int   flag = (a>epsilon) ? OVER : ((a<-epsilon) ? UNDER : COPLANAR);
	return flag;
}

inline float LineProjectTime(const float3 &p0, const float3 &p1, const float3 &a)
{
	// project point a on segment [p0,p1]
	float3 d = p1 - p0;
	float t = dot(d, (a - p0)) / dot(d, d);
	return t;
}
inline float3 LineProject(const float3 &p0, const float3 &p1, const float3 &a) { return p0 + (p1 - p0) * LineProjectTime(p0, p1, a); }

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
inline bool tri_interior(const float3& v0, const float3& v1, const float3& v2, const float3& d)
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



inline int maxdir(const float3 *p, int count, const float3 &dir)  // returns index
{
	if (count == 0)
		return -1;
	return std::max_element(p, p + count, [dir](const float3 &a, const float3 &b){return dot(a, dir) < dot(b, dir); }) - p;
}
inline float3 TriNormal(const float3 &v0, const float3 &v1, const float3 &v2)  // normal of the triangle with vertex positions v0, v1, and v2
{
	float3 cp = cross(v1 - v0, v2 - v1);
	float m = magnitude(cp);
	if (m == 0) return float3(0, 0, 1);
	return cp*(1.0f / m);
}
inline float4 PolyPlane(const std::vector<float3>& verts)
{
	float4 p(0, 0, 0, 0);
	float3 c(0, 0, 0);
	for (const auto &v : verts)
		c += v*(1.0f / verts.size());
	for (unsigned int i = 0; i < verts.size(); i++)
		p.xyz() += cross(verts[i] - c, verts[(i + 1) % verts.size()] - c);
	if (p == float4(0, 0, 0, 0)) 
		return p;
	p.xyz() = normalize(p.xyz());
	p.w = -dot(c, p.xyz());
	return p;
}
struct HitInfo { bool hit; float3 impact; float3 normal; operator bool(){ return hit; };  };

inline HitInfo PolyHitCheck(const std::vector<float3>& verts, const float4 &plane, const float3 &v0, const float3 &v1)
{
	float d0, d1;
	HitInfo hitinfo = { ((d0 = dot(float4(v0, 1), plane)) > 0 && (d1 = dot(float4(v1, 1), plane)) < 0), { 0, 0, 0 }, { 0, 0, 0 } };  // if segment crosses into plane
	hitinfo.normal = plane.xyz();
	hitinfo.impact = v0 + (v1 - v0)* d0 / (d0 - d1);  //  if both points on plane this will be 0/0, if parallel you might get infinity
	for (unsigned int i = 0; hitinfo&& i < verts.size(); i++)
		hitinfo.hit = hitinfo && (determinant(float3x3(verts[(i + 1) % verts.size()] - v0, verts[i] - v0, v1 - v0)) >= 0);  // use v0,v1 winding instead of impact to prevent mesh edge tunneling
	return hitinfo;  
}
inline HitInfo PolyHitCheck(const std::vector<float3>& verts, const float3 &v0, const float3 &v1) { return PolyHitCheck(verts, PolyPlane(verts), v0, v1); }

inline HitInfo ConvexHitCheck(const std::vector<float4>& planes, float3 v0, const float3 &v1_)
{
	float3 v1 = v1_;
	float3 n;
	for (auto plane : planes)
	{
		float d0 = dot(float4(v0, 1), plane);
		float d1 = dot(float4(v1, 1), plane);
		if (d0 >= 0 && d1>=0)  // segment above plane
			return{ false, v1_, { 0, 0, 0 } }; // hitinfo;
		if (d0 <= 0 && d1<=0 )
			continue; //  start and end point under plane
		auto c = v0 + (v1 - v0)* d0 / (d0 - d1);
		if (d0 >= 0)
		{
			n = plane.xyz();
			v0 = c;
		}
		else
			v1 = c;
	}
	return{ true, v0, n };
}
inline HitInfo ConvexHitCheck(const std::vector<float4>& planes, const Pose &pose, float3 v0, const float3 &v1)
{
	auto h = ConvexHitCheck(planes, pose.Inverse()*v0, pose.Inverse()*v1);
	return { h.hit, pose*h.impact, qrot(pose.orientation, h.normal) };
}

inline int argmax(const float a[], int count)  // returns index
{
	if (count == 0) return -1;
	return std::max_element(a,a+count)-a;
}

// still in the process of rearranging basic math and geom routines, putting these here for now...

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
	float3 u = ( PlaneLineIntersection(nrml, dist, cop, cop + dir1) - cor) * fudgefactor;
	float m = magnitude(u);
	u = (m > 1) ? u / m : u - (nrml * sqrtf(1 - m*m));
	float3 v = ( PlaneLineIntersection(nrml, dist, cop, cop + dir2) - cor) * fudgefactor;
	m = magnitude(v);
	v = (m>1) ? v / m : v - (nrml * sqrtf(1 - m*m));
	return RotationArc(u, v);
}



template<class T, int N>
inline std::pair<linalg::vec<T, N>, linalg::vec<T, N> > Extents(const std::vector<linalg::vec<T, N> > &verts)
{
	linalg::vec<T, N> bmin(std::numeric_limits<T>::max()), bmax(std::numeric_limits<T>::lowest());
	for (auto v : verts)
	{
		bmin = cmin(bmin, v);
		bmax = cmax(bmax, v);
	}
	return std::make_pair(bmin, bmax);  // typical useage:   std::tie(mymin,mymax) = Extents(myverts);  
}

inline float Volume(const float3 *vertices, const int3 *tris, const int count)
{
	// count is the number of triangles (tris) 
	float  volume = 0;
	for (int i = 0; i<count; i++)  // for each triangle
	{
		volume += determinant(float3x3(vertices[tris[i][0]], vertices[tris[i][1]], vertices[tris[i][2]])); //divide by 6 later for efficiency
	}
	return volume / 6.0f;  // since the determinant give 6 times tetra volume
}

inline float3 CenterOfMass(const float3 *vertices, const int3 *tris, const int count)
{
	// count is the number of triangles (tris) 
	float3 com(0, 0, 0);
	float  volume = 0; // actually accumulates the volume*6
	for (int i = 0; i<count; i++)  // for each triangle
	{
		float3x3 A(vertices[tris[i][0]], vertices[tris[i][1]], vertices[tris[i][2]]);
		float vol = determinant(A);  // dont bother to divide by 6 
		com += vol * (A.x + A.y + A.z);  // divide by 4 at end
		volume += vol;
	}
	com /= volume*4.0f;
	return com;
}
inline float3x3 Inertia(const float3 *vertices, const int3 *tris, const int count, const float3& com)
{
	// count is the number of triangles (tris) 
	// The moments are calculated based on the center of rotation (com) which defaults to [0,0,0] if unsupplied
	// assume mass==1.0  you can multiply by mass later.
	// for improved accuracy the next 3 variables, the determinant d, and its calculation should be changed to double
	float  volume = 0;                          // technically this variable accumulates the volume times 6
	float3 diag(0, 0, 0);                       // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
	float3 offd(0, 0, 0);                       // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]
	for (int i = 0; i<count; i++)  // for each triangle
	{
		float3x3 A(vertices[tris[i][0]] - com, vertices[tris[i][1]] - com, vertices[tris[i][2]] - com);  // matrix trick for volume calc by taking determinant
		float    d = determinant(A);  // vol of tiny parallelapiped= d * dr * ds * dt (the 3 partials of my tetral triple integral equasion)
		volume += d;                   // add vol of current tetra (note it could be negative - that's ok we need that sometimes)
		for (int j = 0; j<3; j++)
		{
			int j1 = (j + 1) % 3;
			int j2 = (j + 2) % 3;
			diag[j] += (A[0][j] * A[1][j] + A[1][j] * A[2][j] + A[2][j] * A[0][j] +
				A[0][j] * A[0][j] + A[1][j] * A[1][j] + A[2][j] * A[2][j]) *d; // divide by 60.0f later;
			offd[j] += (A[0][j1] * A[1][j2] + A[1][j1] * A[2][j2] + A[2][j1] * A[0][j2] +
				A[0][j1] * A[2][j2] + A[1][j1] * A[0][j2] + A[2][j1] * A[1][j2] +
				A[0][j1] * A[0][j2] * 2 + A[1][j1] * A[1][j2] * 2 + A[2][j1] * A[2][j2] * 2) *d; // divide by 120.0f later
		}
	}
	diag /= volume*(60.0f / 6.0f);  // divide by total volume (vol/6) since density=1/volume
	offd /= volume*(120.0f / 6.0f);
	return{ { diag.y + diag.z, -offd.z, -offd.y },
	{ -offd.z, diag.x + diag.z, -offd.x },
	{ -offd.y, -offd.x, diag.x + diag.y } };
}



#endif //GEOMETRIC_H
