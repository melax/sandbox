//
//   WingMesh class 
//  (c) stan melax 2007    bsd licence
//
//
// A mesh class designed to support many general mesh operations and be efficient.
// This class uses halfedge structures and is designed to work with manifold meshes.
// The eventual goal is to consolidate all the different structures and functions currently in use.
//
// Many winged mesh implementations make excessive use of new and delete resulting
// in a rat's nest of pointers and bad memory fragmentation.
// The idea is to use dynamic arrays for storing vertices, edges and faces.
// Obviously, indices are used instead of pointers.  
// Pointers to faces,edges,or vertices are discouraged since these can become invalid as
// elements are added to the array which might cause a reallocation.
//
// The wingmesh structure can be repacked/compressed to fill in space within the arrays.
// Care must be taken since any references (by index) to elements of the winged mesh from
// places outside of the class may become incorrect or invalid after packing.
// Therefore, depending on the application, it may be better to repacked the mesh 
// in constant time immediately after each operation, or wait and repack the
// entire mesh in linear time after a mesh algorithm has completed.
//
// Programming can be especially tricky using integer indices instead of pointers which 
// offer greater safety due to type checking.  For example, there's nothing to safeguard 
// the programmer against using the id of an edge when indexing the vertex list.
//
//
#ifndef WINGMESH_H
#define WINGMESH_H

#include <vector>
#include <assert.h>

#include "vecmatquat.h"
#include "geometric.h"


struct WingMesh
{
	struct HalfEdge
	{
		short id;
		short v;
		short adj;
		short next;
		short prev;
		short face;
		HalfEdge(){id=v=adj=next=prev=face=-1;}
		HalfEdge(short _id,short _v,short _adj,short _next,short _prev,short _face):id(_id),v(_v),adj(_adj),next(_next),prev(_prev),face(_face){}

		HalfEdge &Next(){assert(next>=0 && id>=0); return this[next-id];}  // convenience for getting next halfedge 
		HalfEdge &Prev(){assert(prev>=0 && id>=0); return this[prev-id];}
		HalfEdge &Adj (){assert(adj >=0 && id>=0); return this[adj -id];}
	};
	std::vector<HalfEdge> edges;
	std::vector<float3>   verts;
	std::vector<float4>   faces;
	std::vector<short>    vback;
	std::vector<short>    fback;
	int unpacked; // flag indicating if any unused elements within arrays
	WingMesh():unpacked(0){}
};

inline float3    SupportPoint(const WingMesh *m, const float3& dir) { return m->verts[maxdir(m->verts.data(), m->verts.size(), dir)]; }

WingMesh  WingMeshDual(const WingMesh &m,float r=1.0f,const float3 &p=float3(0,0,0));
WingMesh  WingMeshCreate(const float3 *verts,const int3 *tris,int n);
WingMesh  WingMeshCreate(const float3 *verts,const int3 *tris,int n,const int *hidden_edges,int hidden_edges_count);
WingMesh  WingMeshCube(const float3 &bmin,const float3 &bmax);
WingMesh  WingMeshCrop(const WingMesh &_m,const float4 &slice);
int       WingMeshSplitTest(const WingMesh &m,const float4 &plane);
WingMesh& WingMeshTranslate(WingMesh &m, const float3 &offset);     // non-const tranlates mesh passed in
WingMesh& WingMeshRotate(WingMesh &m, const float4 &rot);           // non-const rotates mesh passed in
float     WingMeshVolume(const WingMesh &m);

std::vector<int3> WingMeshTris(const WingMesh &m);  // generates a list of indexed triangles from the wingmesh's faces

float3x3  Inertia(const std::vector<WingMesh*> &meshes, const float3& com=float3(0,0,0));
float3    CenterOfMass(const std::vector<WingMesh*> &meshes);
float     Volume(const std::vector<WingMesh*> &meshes);

// int       WingMeshToFaces(WingMesh *m,std::vector<Face*> &faces);






#endif
