

#pragma once
#ifndef MESH_H
#define MESH_H

#include <vector>
#include <string>
#include <algorithm>  // for std::sort which should take a lambda
#include "geometric.h"

std::vector<int3> adjacencies(const std::vector<int3> &tris)
{
	int vcount = 0;
	for (auto t : tris) for (int j = 0; j < 3; j++)
		vcount = std::max(vcount, t[j] + 1);
	// implicit edge# is tri*3 plus index (from 0 to 2) of vertex that is left out
	// so edge 0 is triangle 0's v[1] to v[2], whereas edge 5 is triangle 1's v0 to v1
	// assumes range of values is within 0 to vcount-1.  this puts a limit on number of reverse indexes 
	struct halfedge { short id, adj, v0, v1; halfedge(){} halfedge(short id, short adj, short v0, short v1) :id(id), adj(adj), v0(v0), v1(v1){} };
	std::vector<halfedge> edges;
	for (unsigned int i = 0; i<tris.size() * 3; i++)
		edges.push_back(halfedge(i, -1, tris[i / 3][(i + 1) % 3], tris[i / 3][(i + 2) % 3]));
	std::sort(edges.begin(), edges.end(), [](const halfedge &a, const halfedge &b){return a.v1<b.v1; });  // sort by 2nd vertex v1 so these are grouped
	std::vector<short> v1tab;  // want v1tab[v] to be first edge e such that e.v1==v
	v1tab.resize(vcount);
	for (unsigned int i = edges.size(); i--;) // by going in backwards order we end up with v1tab indexing back so that 
		v1tab[edges[i].v1] = i;  // the index of the first edge that has this particular 2nd vertex (v1)
	for (unsigned int i = 0; i<edges.size(); i++)
		for (unsigned int k = v1tab[edges[i].v0]; k<edges.size() && edges[k].v1 == edges[i].v0; k++)  // check all edges that end at have i'th edge's start v0  
			if (edges[k].v0 == edges[i].v1)
				edges[i].adj = edges[k].id;   // found a match, only set one side, other should get set later
	std::vector<int3> adj_out;
	adj_out.resize(tris.size());
	for (const auto & e : edges)
		adj_out[e.id / 3][e.id % 3] = e.adj;
	int borders_found = 0, non_manifold_edges_found = 0;
	for (unsigned int i = 0; i<adj_out.size() * 3; i++)
	{
		int j = adj_out[i / 3][i % 3];
		if (j == -1)
			borders_found++;  // just counting for curiousity, should be 0 for most of my stuff
		else if (i != adj_out[j / 3][j % 3])
		{
			non_manifold_edges_found++;
			adj_out[i / 3][i % 3] = -1;  //make it a border edge so shadowing will work
		}
	}
	return adj_out;
}

struct Vertex  // pretty much just need one vertex layout to do anything.  
{
	float3 position;    // : POSITION;
	float4 orientation; // : TEXCOORD1;  quaternion for tangent,binormal,normal
	float2 texcoord;    // : TEXCOORD0;
};

inline std::istream &operator >>(std::istream &s, Vertex &v) { return s >> v.position >> v.orientation >> v.texcoord; }

//----------- skinning support ----------------
struct VertexS : public Vertex  
{
	int4 bones;   
	float4 weights;
};
inline std::istream &operator >>(std::istream &s, VertexS &v) { return s >> (Vertex&)v >> v.bones >> v.weights; }

inline Pose skin_dualquat(int4 bones, float4 weights, const std::vector<Pose> &pose)  // dual quat skinning (screw motion)
{
	Pose p;
	float4 b0 = pose[bones[0]].orientation;
	float4 b1 = pose[bones[1]].orientation;
	float4 b2 = pose[bones[2]].orientation;
	float4 b3 = pose[bones[3]].orientation;
	if (dot(b0, b1)<0) b1 = -(b1);
	if (dot(b0 + b1, b2)<0) b2 = -(b2);
	if (dot(b0 + b1 + b2, b3)<0) b3 = -(b3);
	float4 q = (
		b0*weights[0] +
		b1*weights[1] +
		b2*weights[2] +
		b3*weights[3]);
	p.position =         // convert translational inputs to dualquat's dual part and back.
		qmul(
			qmul(float4(pose[bones[0]].position, 0), b0) * weights[0] +
			qmul(float4(pose[bones[1]].position, 0), b1) * weights[1] +
			qmul(float4(pose[bones[2]].position, 0), b2) * weights[2] +
			qmul(float4(pose[bones[3]].position, 0), b3) * weights[3]
			, qconj(q)
			).xyz() / dot(q, q); // this works since q hasn't been normalized yet
	p.orientation = normalize(q);
	return p;
}

inline Vertex Skin(const VertexS &s, const std::vector<Pose> &pose)
{
	Pose p = skin_dualquat(s.bones, s.weights, pose);
	return { p*s.position,qmul(p.orientation,s.orientation),s.texcoord };
}
inline std::vector<Vertex> Skin(const std::vector<VertexS> &sverts, const std::vector<Pose> &pose)
{
	return Transform(sverts, [&pose](const VertexS &s) {return Skin(s, pose); });
}

//------------------------------------------

struct Mesh
{
	std::vector<Vertex> verts;
	std::vector<int3>   tris;
	Pose pose;
	std::string material;
	float4 hack; //  = { 1, 1, 1, 1 };
};

inline Mesh MeshFromHeightField(unsigned short *image, int dimx, int dimy, std::function<float3(int, int,short)> f, short cont = 30, short rmin = 1, short rmax = 2000) // assumes mm to m 
{
	std::vector<Vertex> verts;
	std::vector<int3>   tris;
	std::vector<int> vid(dimx*dimy, -1);
	for (int i = 0; i < dimx*dimy; i++)
	{
		if (image[i]<rmin || image[i]>rmax)
			continue;
		int x = i%dimx, y = i / dimx;
		vid[i] = (int)verts.size();
		verts.push_back({ f(x, y,image[i]), { 0, 0, 0, 1 }, { (float)x / dimx, (float)y / dimy } });
	}
	for (int i = 0; i < dimx*dimy; i++)
	{
		if (vid[i] == -1)
			continue;
		int x = i%dimx, y = i / dimx;
		int xp = (x && vid[i - 1] >= 0) ? x - 1 : x;
		int xn = (x < dimx - 1 && vid[i + 1] >= 0) ? x + 1 : x;
		int yp = (y && vid[i - dimx] >= 0) ? y - 1 : y;
		int yn = (y < dimy - 1 && vid[i + dimx] >= 0) ? y + 1 : y;
		auto norm = cross(verts[vid[xn + y*dimx]].position - verts[vid[xp + y*dimx]].position, verts[vid[x + yn*dimx]].position - verts[vid[x + yp*dimx]].position);
		norm =  (norm==float3(0, 0, 0)) ? float3(0, 0, 1) : normalize(norm);
		verts[vid[i]].orientation = quat_from_to(float3(0, 0, 1), norm);
		if (xp != x && yp != y)
			tris.push_back(int3(vid[i], vid[i - 1], vid[i - dimx]));
		if (xn != x && yn != y)
			tris.push_back(int3(vid[i], vid[i + 1], vid[i + dimx]));
		if (xn != x && yp != y && vid[i + 1 - dimx] == -1)
			tris.push_back(int3(vid[i + 1], vid[i], vid[i - dimx]));
		if (xp != x && yn != y && vid[i - 1 + dimx] == -1)
			tris.push_back(int3(vid[i - 1], vid[i], vid[i + dimx]));
	}
	return{ verts, tris, Pose(), "", { 1, 1, 1, 1 } };
}

inline Mesh MeshFlatShadeTex(const std::vector<float3> &verts, const std::vector<int3> &tris)  // procedurally generate normals and texture coords mesh
{
	std::vector<Vertex> vout;
	std::vector<int3> tout;
	for (auto t : tris)
	{
		float3 n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		auto vn = abs(n);
		int k = argmax(&vn.x, 3);
		int c = (int)vout.size();
		tout.push_back({ c, c + 1, c + 2 });
		float3 st = gradient(verts[t[0]], verts[t[1]], verts[t[2]], verts[t[0]][(k + 1) % 3], verts[t[1]][(k + 1) % 3], verts[t[2]][(k + 1) % 3]);
		float3 sb = gradient(verts[t[0]], verts[t[1]], verts[t[2]], verts[t[0]][(k + 2) % 3], verts[t[1]][(k + 2) % 3], verts[t[2]][(k + 2) % 3]);
		bool flipu = dot(sb, cross(n, st))<0.0f;
		st = cross(sb, n);
		auto q = quatfrommat(float3x3(st, sb, n)); //  quat_from_to(float3(0, 0, 1), n);
		for (int j = 0; j < 3; j++)
		{
			const float3 &v = verts[t[j]];
			vout.push_back({ v, q, { v[(k + 1) % 3] * (flipu ? -1.0f : 1.0f), v[(k + 2) % 3] } });
		}
	}
	return{ vout, tout, Pose(), "", { 1, 1, 1, 1 } };
}

inline Mesh MeshSmoothish(const std::vector<float3> &points, const std::vector<int3> tris)
{
	std::vector<Vertex> verts;
	for (auto p : points)
		verts.push_back({ p, { 0, 0, 0, 0 }, p.xy() });
	for (auto t : tris)
	{
		float3 sn = normalize(cross(points[t[1]] - points[t[0]], points[t[2]] - points[t[0]]));
		float3 st = gradient(points[t[0]], points[t[1]], points[t[2]], verts[t[0]].texcoord.x, verts[t[1]].texcoord.x, verts[t[2]].texcoord.x);
		float3 sb = cross(sn, st);
		auto q = quatfrommat(float3x3(st, sb, sn)); //  quat_from_to(float3(0, 0, 1), n);
		for (int i = 0; i < 3; i++)
		{
			verts[t[i]].orientation += q;
		}
	}
	for (auto &v : verts)
	{
		v.orientation = normalize(v.orientation);
		float3 n = qrot(v.orientation, float3(0, 0, 1));
		//v.texcoord = { atan2(n.y, n.x), 0.5f + n.y / 2.0f };  // quick sphereical hack
	}
	return{ verts, tris, Pose(), "", { 1, 1, 1, 1 } };
}





#endif
