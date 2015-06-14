//
//   WingMesh class 
//  (c) stan melax 2007    bsd licence
//
// A mesh class designed to support n-gon mesh operations and be efficient.
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
// primary usage of this module is to represent and operate 
// on convex cells of space in the spatial partition tree.
//

#ifndef WINGMESH_H
#define WINGMESH_H

#include <vector>
#include <assert.h>
#include <iterator>
#include <algorithm>
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
		HalfEdge(){ id = v = adj = next = prev = face = -1; }
		HalfEdge(short _id, short _v, short _adj, short _next, short _prev, short _face) :id(_id), v(_v), adj(_adj), next(_next), prev(_prev), face(_face){}

		HalfEdge &Next(){ assert(next >= 0 && id >= 0); return this[next - id]; }  // convenience for getting next halfedge 
		HalfEdge &Prev(){ assert(prev >= 0 && id >= 0); return this[prev - id]; }
		HalfEdge &Adj(){ assert(adj >= 0 && id >= 0); return this[adj - id]; }
	};
	std::vector<HalfEdge> edges;
	std::vector<float3>   verts;
	std::vector<float4>   faces;
	std::vector<short>    vback;
	std::vector<short>    fback;
	int unpacked; // flag indicating if any unused elements within arrays


	WingMesh() :unpacked(0){}

	int     VertDegree(int v) const { int e0 = vback[v], e = e0, k = 0;  if (e != -1) do{ k++; e = edges[edges[e].adj].next; } while (e != e0); return k; }



	void SanityCheck() const 
	{
		for (unsigned int e = 0; e<edges.size(); e++)
		{
			if (unpacked && edges[e].v == -1)
			{
				assert(edges[e].face == -1);
				assert(edges[e].next == -1);
				assert(edges[e].prev == -1);
				continue;
			}
			assert(edges[e].id == e);
			assert(edges[e].v >= 0);
			assert(edges[e].face >= 0);
			assert(edges[edges[e].next].prev == e);
			assert(edges[edges[e].prev].next == e);
			assert(edges[e].adj != e);  // antireflexive
			assert(edges[edges[e].adj].adj == e);  // symmetric
			assert(edges[e].v == edges[edges[edges[e].adj].next].v);
			assert(edges[e].v != edges[edges[e].adj].v);
		}
		for (unsigned int i = 0; i<vback.size(); i++) assert((unpacked && vback[i] == -1) || edges[vback[i]].v == i);
		for (unsigned int i = 0; i<fback.size(); i++) assert((unpacked && fback[i] == -1) || edges[fback[i]].face == i);
	}


	class FaceViewC  // example how to write an iterator for what is a circular indexed list
	{
	public:
		class Iterator : public std::iterator< std::bidirectional_iterator_tag, WingMesh::HalfEdge, std::size_t, WingMesh::HalfEdge*, WingMesh::HalfEdge& >
		{
		public:
			WingMesh &wm;
			int start;
			int i; // current
			bool eq(const Iterator& other) const { return (i == other.i); }
			bool operator==(const Iterator& rhs) { return  eq(rhs); }
			bool operator!=(const Iterator& rhs) { return !eq(rhs); }
			reference operator*() { assert(i >= 0); assert(i == wm.edges[i].id); return  wm.edges[i]; }
			pointer  operator->() { assert(i >= 0); assert(i == wm.edges[i].id); return &wm.edges[i]; }
			Iterator& operator++() { if (i >= 0) i = wm.edges[i].next; if (i == start) i = -1; return *this; }
			Iterator& operator--() { if (i == start) i = -1; else if (i == -1) i = start; if (i >= 0) i = wm.edges[i].prev; return *this; }
			Iterator operator--(int){ Iterator temp(*this); --(*this); return temp; }
			Iterator operator++(int){ Iterator temp(*this); ++(*this); return temp; }
			explicit Iterator(WingMesh &wm, int start, int i) :wm(wm), start(start), i(i){};
			friend class FaceViewC;
		};
		WingMesh &wm;
		int fid;
		Iterator begin() { return Iterator(wm, wm.fback[fid], wm.fback[fid]); }
		Iterator end()   { return Iterator(wm, wm.fback[fid], -1); }
		FaceViewC(const WingMesh *wm, int fid) :wm(*(WingMesh*) wm), fid(fid){}  // const cheating here
	};
	FaceViewC  FaceView(int fid) const { return FaceViewC(this, fid); }  // const cheating here

	std::vector<float3> GenerateFaceVerts(int fid) const
	{
		std::vector<float3> fverts;
		for (auto &e : FaceView(fid))
			fverts.push_back(verts[e.v]);
		return fverts;
	}

	float4& ComputeFaceNormal(int f) { return (faces[f] = PolyPlane(GenerateFaceVerts(f))); }


	void EdgeSwap(int a, int b)  // untested, using packslotedge typically
	{
		std::swap(edges[a], edges[b]);
		edges[edges[a].prev].next = edges[edges[a].next].prev = edges[a].id = a;
		edges[edges[b].prev].next = edges[edges[b].next].prev = edges[b].id = b;
		if (vback[edges[a].v] == b) vback[edges[a].v] = a;
		if (vback[edges[b].v] == a) vback[edges[b].v] = b;
		if (fback[edges[a].face] == b) fback[edges[a].face] = a;
		if (fback[edges[b].face] == a) fback[edges[b].face] = b;
	}

	void LinkMesh()
	{
		// Computes adjacency information for edges.
		// Assumes all edges have valid prev,next, and vertex.
		unsigned int i;
		std::vector<short> edgesv;
		edgesv.reserve(edges.size());  // edges indexes sorted into ascending edge's vertex index order
		for (i = 0; i < edges.size(); i++) edgesv.push_back((short)i);
		std::sort(edgesv.begin(), edgesv.end(), [this](const short &a, const short &b){ return this->edges[a].v < this->edges[b].v; });
		std::vector<short> veback;
		veback.resize(verts.size());
		i = edges.size();
		while (i--)
			veback[edges[edgesv[i]].v] = (short)i;
		for (i = 0; i < edges.size(); i++)
		{
			WingMesh::HalfEdge &e = edges[i];
			assert(e.id == i);
			if (e.adj != -1) continue;
			short a = e.v;
			short b = edges[e.next].v;
			unsigned int k = veback[b];
			while (k < edges.size() && edges[edgesv[k]].v == b)
			{
				if (edges[edges[edgesv[k]].next].v == a)
				{
					e.adj = edgesv[k];
					edges[e.adj].adj = e.id;
					break;
				}
				k++;
			}
			assert(e.adj != -1);
		}
	}

	void InitBackLists()
	{
		vback.assign(verts.size(), -1);
		fback.assign(faces.size(), -1);
		unsigned int i = edges.size();
		while (i--)  // going backward so back pointers will point to the first applicable edge
		{
			if (unpacked && edges[i].v == -1) continue;
			vback[edges[i].v] = i;
			fback[edges[i].face] = i;
		}
	}
	int BuildEdge(int ea, int eb)  // puts an edge from ea.v to eb.v  and splits a face into two
	{
		assert(edges[ea].next != eb);  // make sure they aren't already beside each other
		assert(edges[eb].next != ea);
		assert(edges[eb].face == edges[ea].face);
		int e = ea;
		while (e != eb)
		{
			e = edges[e].next;
			assert(e != ea);  // make sure eb is on the same poly
		}
		int newface = faces.size();
		WingMesh::HalfEdge sa(edges.size() + 0, edges[ea].v, edges.size() + 1, eb, edges[ea].prev, newface);  // id,v,adj,next,prev,face
		WingMesh::HalfEdge sb(edges.size() + 1, edges[eb].v, edges.size() + 0, ea, edges[eb].prev, edges[ea].face);
		edges[sa.prev].next = edges[sa.next].prev = sa.id;
		edges[sb.prev].next = edges[sb.next].prev = sb.id;
		edges.push_back(sa);
		edges.push_back(sb);
		faces.push_back(faces[edges[ea].face]);
		if (fback.size()) {
			fback.push_back(sa.id);
			fback[sb.face] = sb.id;
		}
		e = edges[sa.id].next;
		while (e != sa.id)
		{
			assert(e != ea);
			edges[e].face = newface;
			e = edges[e].next;
		}
		return sa.id;
	}


	void SplitEdge(int e, const float3 &vpos)
	{
		// Add's new vertex vpos to mesh 
		// Adds two new halfedges to 
		int ea = edges[e].adj;
		int v = verts.size();
		WingMesh::HalfEdge s0(edges.size() + 0, v, edges.size() + 1, edges[e].next, e, edges[e].face);  // id,v,adj,next,prev,face
		WingMesh::HalfEdge sa(edges.size() + 1, edges[ea].v, edges.size() + 0, ea, edges[ea].prev, edges[ea].face);
		edges[s0.prev].next = edges[s0.next].prev = s0.id;
		edges[sa.prev].next = edges[sa.next].prev = sa.id;
		edges[ea].v = v;
		edges.push_back(s0);
		edges.push_back(sa);
		verts.push_back(vpos);
		if (vback.size())
		{
			vback.push_back(s0.id);
			vback[sa.v] = sa.id;
		}
	}

	void SplitEdges(const float4 &split)
	{
		for (unsigned int e = 0; e < edges.size(); e++)
		{
			int ea = edges[e].adj;
			if ((PlaneTest(split, verts[edges[e].v]) | PlaneTest(split, verts[edges[ea].v])) == SPLIT)
			{
				float3 vpos = PlaneLineIntersection(split, verts[edges[e].v], verts[edges[ea].v]);
				assert(PlaneTest(split, vpos) == COPLANAR);
				SplitEdge(e, vpos);
			}
		}
	}

	int findcoplanaredge(int v, const float4 &slice)  // v should be coplanar
	{
		// assumes the mesh straddles the plane slice.
		// adds edge if required.  edges should have been split already
		int e = vback[v];
		int es = e;
		while (PlaneTest(slice, verts[edges[e].Adj().v]) != UNDER)
		{
			e = edges[e].Prev().Adj().id;  // next edge coming out of v going ccw
			assert(e != es); // if(e==es) return -1; // all edges point over!
		}
		es = e;
		while (PlaneTest(slice, verts[edges[e].Adj().v]) == UNDER)
		{
			e = edges[e].Adj().Next().id;  // next cw edge coming out of v
			assert(e != es); // if(e==es) return -1; // all edges point UNDER!
		}
		int ec = edges[e].next;
		while (PlaneTest(slice, verts[edges[ec].v]) != COPLANAR)
		{
			ec = edges[ec].next;
			assert(ec != e);
		}
		if (ec == edges[e].next) { return e; }  // a coplanar edge already exists 
		assert(ec != edges[e].prev);
		return BuildEdge(e, ec);

	}

	std::vector<int> SliceMesh(const float4 &slice)  // also creates new edges along the split plane
	{
		std::vector<int> loop;
		SplitEdges(slice);
		auto it = std::find_if(verts.begin(), verts.end(), [&slice](const float3 &v){return (PlaneTest(slice, v) == COPLANAR); });
		if (it == verts.end())
			return loop;
		int v0 = it - verts.begin(), v = v0;
		assert(v0 >= 0 && v0 < (int)verts.size());
		do{
			int e = findcoplanaredge(v, slice);
			v = edges[e].Adj().v;
			loop.push_back(e);
		} while (v != v0);
		return loop;
	}


	void PackSlotEdge(int s)
	{
		// used to relocate last edge into slot s
		WingMesh::HalfEdge e = edges.back(); edges.pop_back();
		if (edges.size() == s) return;
		assert(s < (int)edges.size());
		assert(e.v >= 0); // last edge is invalid
		if (vback.size() && vback[e.v] == e.id) vback[e.v] = s;
		if (fback.size() && fback[e.face] == e.id) fback[e.face] = s;
		e.id = s;
		edges[s] = e;
		edges[e.next].prev = e.id;
		edges[e.prev].next = e.id;
		edges[e.adj].adj = e.id;
	}

	inline void PackSlotVert(int s)
	{
		// When you no longer need verts[s], compress away unused vert
		assert(vback.size() == verts.size());
		assert(vback[s] == -1);
		int last = verts.size() - 1;
		if (s == last)
		{
			verts.pop_back();
			vback.pop_back();
			return;
		}
		vback[s] = vback[last];
		verts[s] = verts[last];
		int e = vback[s];
		assert(e != -1);
		do{ assert(edges[e].v == last); edges[e].v = s; e = edges[edges[e].adj].next; } while (e != vback[s]);  // iterate over edges coming out of vert 'last' and change to 's'
		verts.pop_back();
		vback.pop_back();
	}

	inline void PackSlotFace(int s)
	{
		// When you no longer need faces[s], compress away unused face
		assert(fback.size() == faces.size());
		assert(fback[s] == -1);
		int last = faces.size() - 1;
		if (s == last)
		{
			faces.pop_back();
			fback.pop_back();
			return;
		}
		fback[s] = fback[last];
		faces[s] = faces[last];
		int e = fback[s];
		assert(e != -1);
		do{ assert(edges[e].face == last); edges[e].face = s; e = edges[e].next; } while (e != fback[s]);
		faces.pop_back();
		fback.pop_back();
	}
	void SwapFaces(int a, int b)
	{
		std::swap(faces[a], faces[b]);
		std::swap(fback[a], fback[b]);
		if (fback[a] != -1)
		{
			int e = fback[a];
			do{ assert(edges[e].face == b); edges[e].face = a; e = edges[e].next; } while (e != fback[a]);
		}
		if (fback[b] != -1)
		{
			int e = fback[b];
			do{ assert(edges[e].face == a); edges[e].face = b; e = edges[e].next; } while (e != fback[b]);
		}
	}
	inline void PackFaces()  // removes faces with no edges using them
	{
		assert(fback.size() == faces.size());
		unsigned int s = 0;
		for (unsigned int i = 0; i < faces.size(); i++)
		{
			if (fback[i] == -1) continue;
			if (s < i) SwapFaces( s, i);
			s++;
		}
		fback.resize(s);
		faces.resize(s);
	}

	void Compress()	// get rid of unused faces and verts
	{
		int i;
		assert(vback.size() == verts.size());
		assert(fback.size() == faces.size());
		i = edges.size();
		while (i--) if (edges[i].v == -1) PackSlotEdge(i);
		i = vback.size();
		while (i--) if (vback[i] == -1) PackSlotVert(i);
		PackFaces();
		//	i=m.fback.size();
		//	while(i--) if(m.fback[i]==-1) PackSlotFace(m,i);
		unpacked = 0;
		SanityCheck();
	}

	void RemoveEdges(const int *cull, int cull_count)
	{
		vback.clear();
		fback.clear();
		unpacked = 1;
		for (int i = 0; i<cull_count; i++)
		{
			int e = cull[i];
			WingMesh::HalfEdge &Ea = edges[e];
			if (Ea.v == -1) continue; // already snuffed
			WingMesh::HalfEdge &Eb = edges[Ea.adj];
			edges[Ea.prev].next = edges[Eb.next].id;
			edges[Eb.prev].next = edges[Ea.next].id;
			edges[Ea.next].prev = edges[Eb.prev].id;
			edges[Eb.next].prev = edges[Ea.prev].id;
			Ea.next = Ea.prev = Ea.face = Ea.adj = Ea.v = -1;
			Eb.next = Eb.prev = Eb.face = Eb.adj = Eb.v = -1;
		}
		for (unsigned int i = 0; i<edges.size(); i++)  // share faces now
		{
			WingMesh::HalfEdge &E = edges[i];
			if (E.v == -1) continue; // dead edge
			if (E.face == E.Next().face) continue; // 
			for (int e = E.next; e != i; e = edges[e].next)
			{
				edges[e].face = E.face;
			}
		}
		InitBackLists();
		SanityCheck();
		Compress();
	}


	void Crop(const std::vector<int> &edgeloop)  // removes everything above/adjacent to the loop
	{
		std::vector<int> killstack;
		for (unsigned int i = 0; i<edgeloop.size(); i++)
		{
			// dont want to delete vertices along the loop.
			// detach vertices along loop from to-be-deleted edges  
			int ec = edgeloop[i];
			int en = edgeloop[(i + 1) % edgeloop.size()];
			int e = edges[ec].next;
			while (e != en)
			{
				assert(edges[e].v == edges[en].v);
				edges[e].v = -1;
				e = edges[e].Adj().next;
			}
			vback[edges[en].v] = en; // ensure vertex points to edge that wont be deleted.
			fback[edges[en].face] = -1;  // delete all faces 
			edges[en].face = edges[edgeloop[0]].face; // assign all loop faces to same which we ressurrect later
		}
		for (unsigned int i = 0; i<edgeloop.size(); i++)
		{
			// detatch tobedeleted edges from loop edges.
			// and fix loop's next/prev links.
			int ec = edgeloop[i];
			int en = edgeloop[(i + 1) % edgeloop.size()];
			int ep = edgeloop[(i) ? i - 1 : edgeloop.size() - 1];
			WingMesh::HalfEdge &E = edges[ec];
			if (E.next != en)
			{
				WingMesh::HalfEdge &K = E.Next();
				if (K.id >= 0) killstack.push_back(E.next);
				K.id = -1;
				assert(K.v == -1);
				assert(K.prev == E.id);
				K.prev = -1;
				E.next = en;
			}
			if (E.prev != ep)
			{
				WingMesh::HalfEdge &K = E.Prev();
				if (K.id >= 0) killstack.push_back(E.prev);
				K.id = -1;
				assert(K.next == E.id);
				K.next = -1;
				E.prev = ep;
			}
		}
		while (killstack.size())
		{
			// delete (make "-1") all unwanted edges faces and verts
			int k = killstack.back(); killstack.pop_back(); //  Pop(killstack);
			assert(k >= 0);
			WingMesh::HalfEdge &K = edges[k];
			assert(K.id == -1);
			if (K.next != -1 && edges[K.next].id != -1) { killstack.push_back(K.next); edges[K.next].id = -1; }
			if (K.prev != -1 && edges[K.prev].id != -1) { killstack.push_back(K.prev); edges[K.prev].id = -1; }
			if (K.adj  != -1 && edges[K.adj].id  != -1) { killstack.push_back(K.adj ); edges[K.adj].id  = -1; }
			if (K.v != -1) vback[K.v] = -1;
			if (K.face != -1) fback[K.face] = -1;
			K.next = K.prev = K.adj = K.v = K.face = -1;
		}
		assert(fback[edges[edgeloop[0]].face] == -1);
		fback[edges[edgeloop[0]].face] = edgeloop[0];
		ComputeFaceNormal(edges[edgeloop[0]].face);
		SwapFaces(edges[edgeloop[0]].face, 0);
		unpacked = 1;
		SanityCheck();
		Compress();
		SanityCheck();
	}


	std::vector<int3> GenerateTris() const  // generates a list of indexed triangles from the wingmesh's faces
	{
		std::vector<int3> tris;
		tris.reserve(edges.size() - faces.size() * 2); // predicted size
		for (int e0 : fback) // for each face, e0 is first halfedge  // int e0 = fback[i]; 
		{
			if (e0 == -1) continue;
			int ea = e0, eb = edges[ea].next;
			while ((eb = edges[ea = eb].next) != e0)
				tris.push_back(int3(edges[e0].v, edges[ea].v, edges[eb].v));
		}
		return tris;
	}

	int    SplitTest(const float4 &plane) const { int flag = 0; for (auto &v : verts) flag |= PlaneTest(plane, v); return flag; }

	float  CalcVolume() const { auto tris = GenerateTris(); return Volume(verts.data(), tris.data(), tris.size()); }

};



inline float3    SupportPoint(const WingMesh *m, const float3& dir) { return m->verts[maxdir(m->verts.data(), m->verts.size(), dir)]; }

inline void WingMeshTranslate(WingMesh & m, const float3 & translation) { for(auto & v : m.verts) v += translation; for(auto & f : m.faces) PlaneTranslate(f, translation); }
inline void WingMeshRotate(WingMesh & m, const float4 & rotation) { for(auto & v : m.verts) v = qrot(rotation, v); for(auto & f : m.faces) PlaneRotate(f, rotation); }
inline void WingMeshScale(WingMesh & m, float scaling) { for(auto & v : m.verts) v *= scaling; for(auto & f : m.faces) PlaneScale(f, scaling); }

inline std::vector<int3> WingMeshTris(const WingMesh &m) // generates a list of indexed triangles from the wingmesh's faces
{
	return m.GenerateTris();
}


inline float   WingMeshVolume(const WingMesh &m) { return m.CalcVolume(); }












inline void WingMeshAvoidBackRefs(WingMesh &m,int eid)  // ensure vback and fback reference someone other than eid, i.e. just pick another edge to point to
{
	WingMesh::HalfEdge &E = m.edges[eid];
	assert(E.id==eid);
	assert(E.prev!=eid);
	assert(m.edges[m.edges[E.prev].adj].v==E.v);
	assert(m.edges[E.prev].face==E.face);
	if(m.vback[E.v   ]==eid) m.vback[E.v   ]= m.edges[E.prev].adj;
	if(m.fback[E.face]==eid) m.fback[E.face]= E.prev;
}

inline void WingMeshCollapseEdge(WingMesh &m,int ea,int pack=0)
{
	int eb = m.edges[ea].adj;
	WingMesh::HalfEdge &Ea  = m.edges[ea];
	WingMesh::HalfEdge &Eb  = m.edges[eb];
	WingMesh::HalfEdge &Eap = m.edges[Ea.prev];
	WingMesh::HalfEdge &Ean = m.edges[Ea.next];
	WingMesh::HalfEdge &Ebp = m.edges[Eb.prev];
	WingMesh::HalfEdge &Ebn = m.edges[Eb.next];
	assert(Ea.v>=0);
	WingMeshAvoidBackRefs(m, Ea.id);
	WingMeshAvoidBackRefs(m, Eb.id);
	int oldv = m.edges[ea].v;
	int newv = m.edges[eb].v;
	assert(Ean.v==newv);
	assert(Ean.face==Ea.face);
	assert(Ebn.face==Eb.face);
	if(m.vback[newv]==eb) m.vback[newv]=Ean.id;
	if(m.fback[Ea.face]==ea) m.fback[Ea.face]=Ean.id;
	if(m.fback[Eb.face]==eb) m.fback[Eb.face]=Ebn.id;
	int e=ea;
	do{assert(m.edges[e].v==oldv);m.edges[e].v=newv;e=m.edges[m.edges[e].adj].next;} while(e!=ea);
	Eap.next = Ean.id;
	Ean.prev = Eap.id;
	Ebp.next = Ebn.id;
	Ebn.prev = Ebp.id;
	m.vback[oldv]=-1;
	Ea.next=Ea.prev=Ea.face=Ea.adj=Ea.v=-1;
	Eb.next=Eb.prev=Eb.face=Eb.adj=Eb.v=-1;
	if(pack && m.unpacked==0)
	{	
		m.PackSlotEdge(std::max(Ea.id,Eb.id));
		m.PackSlotEdge(std::min(Ea.id,Eb.id));
		m.PackSlotVert(oldv);
	}
	else m.unpacked=1;
	m.SanityCheck();
}



inline WingMesh & WingMeshSort(WingMesh &m) // not sure if this routine works now
{
	if(m.unpacked) m.Compress();
    std::vector<WingMesh::HalfEdge> s;
    s.reserve(m.edges.size());
	int k=0;
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		int e=m.fback[i]; assert(e!=-1);
        do{ s.push_back(m.edges[e]); m.edges[e].id = s.size() - 1; e = m.edges[e].next; } while (e != m.fback[i]);
	}
	for(unsigned int i=0;i<s.size();i++)
	{
		s[i].id=i;
		s[i].next = m.edges[s[i].next].id;
		s[i].prev = m.edges[s[i].prev].id;
		s[i].adj  = m.edges[s[i].adj ].id;
	}
    s.swap(m.edges);
	m.InitBackLists();
	m.SanityCheck();
	for(unsigned int i=0;i<m.edges.size();i++) {assert(m.edges[i].next<=(int)i+1);}
	return m;
}











inline WingMesh WingMeshCrop(const WingMesh &src,const float4 &slice)
{
	int s = src.SplitTest(slice);
	if (s == OVER) return WingMesh(); //  NULL;
	if(s==UNDER) return src;          // returning this will make a copy
	WingMesh m = src;
	auto coplanar = m.SliceMesh( slice);   // 	std::vector<int>  the coplanar edges (i believe these are the above ones)
	std::vector<int> reverse;              // the coplanar edges below (todo verify)  reverse is probably the wrong name
    for (unsigned int i = 0; i<coplanar.size(); i++) reverse.push_back(m.edges[coplanar[coplanar.size() - 1 - i]].adj);

    if (coplanar.size()) 
		m.Crop(reverse);
	m.SanityCheck();
	assert(dot(m.faces[0].xyz(), slice.xyz()) > 0.99f);
	m.faces[0] = slice;
	return m;
}




/*
int VertFindOrAdd(std::vector<float3> &array, const float3& v,float epsilon=0.001f)
{
	for(unsigned int i=0;i<array.size();i++)
	{
		if(magnitude(array[i]-v)<epsilon) 
			return i;
	}
	array.push_back(v);
    return array.size() - 1;
}
WingMesh WingMeshCreate(std::vector<Face*> &faces)
{
	int i,j;
	WingMesh *wmesh = new WingMesh();

	std::vector<WingMesh::HalfEdge> &edges = wmesh->edges;
	// make em
    wmesh->faces.resize(faces.size());
	for(i=0;i<faces.size();i++)
	{
		int base = edges.size();
		Face *f0 = faces[i];
        assert(f0->vertex.size());
		wmesh->faces[i] = Plane(f0->normal(),f0->dist());
		for(j=0;j<f0->vertex.size();j++)
		{
			int j1 = (j+1)%f0->vertex.size();
            int jm1 = (j + f0->vertex.size() - 1) % f0->vertex.size();
			WingMesh::HalfEdge e;
            assert(base + j == edges.size());
			e.id = base+j;
			e.next = base+j1;
			e.prev = base+jm1;
			e.face = i;
			e.v = VertFindOrAdd(wmesh->verts, f0->vertex[j]);
			edges.push_back(e);
		}
	}
	WingMeshInitBackLists(*wmesh);
	int rc=WingMeshLinkMesh(*wmesh); 	// connect em
	assert(rc);
	WingMeshCheck(*wmesh);
	return wmesh;
}
*/


inline WingMesh WingMeshDual(const WingMesh &m, float r=1.0f, const float3 &p=float3(0,0,0))
{
	WingMesh d;  // new WingMesh
	d.faces.resize(m.verts.size());
	d.fback.resize(m.vback.size());
	for (unsigned int i = 0; i<m.verts.size(); i++)
	{
		d.faces[i] = float4(normalize(m.verts[i]), -r*r / magnitude(m.verts[i]));
		d.fback[i] = m.vback[i];
	}
	d.verts.resize(m.faces.size());
	d.vback.resize(m.fback.size());
	for (unsigned int i = 0; i<m.faces.size(); i++)
	{
		d.verts[i] = m.faces[i].xyz()*(-r*r / m.faces[i].w);
		d.vback[i] = m.edges[m.fback[i]].adj;
	}
	d.edges.resize(m.edges.size());
	for (unsigned int i = 0; i<m.edges.size(); i++)
	{
		d.edges[i] = m.edges[i];
		d.edges[i].face = m.edges[i].v;
		d.edges[i].v = m.edges[m.edges[i].adj].face;
		d.edges[i].next = m.edges[m.edges[i].prev].adj;
		d.edges[i].prev = m.edges[m.edges[i].adj].next;
	}
	for (int f = 0; f < (int)d.faces.size(); f++)
		d.ComputeFaceNormal(f);    // this shouldn't change the plane equation by a significant amount

	d.SanityCheck();
	return d;
}




inline WingMesh WingMeshBox(const float3 &bmin,const float3 &bmax)
{
	WingMesh wm;
	wm.verts = { { bmin.x, bmin.y, bmin.z },{ bmin.x, bmin.y, bmax.z },{ bmin.x, bmax.y, bmin.z },{ bmin.x, bmax.y, bmax.z },
	             { bmax.x, bmin.y, bmin.z },{ bmax.x, bmin.y, bmax.z },{ bmax.x, bmax.y, bmin.z },{ bmax.x, bmax.y, bmax.z }, };
	wm.faces = { {-1,0,0,  bmin.x},{ 1,0,0, -bmax.x},{0,-1,0,  bmin.y},{0, 1,0, -bmax.y},{0,0,-1,  bmin.z},{0,0, 1, -bmax.z}, };
	wm.edges = 
	{
		{ 0,0,11, 1, 3,0},{ 1,1,23, 2, 0,0},{ 2,3,15, 3, 1,0},{ 3,2,16, 0, 2,0},
		{ 4,6,13, 5, 7,1},{ 5,7,21, 6, 4,1},{ 6,5, 9, 7, 5,1},{ 7,4,18, 4, 6,1},
		{ 8,0,19, 9,11,2},{ 9,4, 6,10, 8,2},{10,5,20,11, 9,2},{11,1, 0, 8,10,2},
		{12,3,22,13,15,3},{13,7, 4,14,12,3},{14,6,17,15,13,3},{15,2, 2,12,14,3},
		{16,0, 3,17,19,4},{17,2,14,18,16,4},{18,6, 7,19,17,4},{19,4, 8,16,18,4},
		{20,1,10,21,23,5},{21,5, 5,22,20,5},{22,7,12,23,21,5},{23,3, 1,20,22,5},
	};
	wm.InitBackLists();
	wm.SanityCheck();
	return wm;
}
inline WingMesh WingMeshBox(const float3 &r) { return WingMeshBox(-r, r); }
inline WingMesh WingMeshCube(const float  r) { return WingMeshBox({ -r, -r, -r }, { r, r, r }); } // r (radius) is half-extent of box


inline WingMesh WingMeshCreate(const float3 *verts, const int3 *tris, int n)
{
	WingMesh m;
	if (n == 0) return m;
	int verts_count = -1;
	for (int i = 0; i<n; i++) for (int j = 0; j<3; j++)
	{
		verts_count = std::max(verts_count, tris[i][j]);
	}
	verts_count++;
	m.verts.resize(verts_count);
	for (unsigned int i = 0; i<m.verts.size(); i++)
	{
		m.verts[i] = verts[i];
	}
	for (int i = 0; i<n; i++)
	{
		const int3 &t = tris[i];
		WingMesh::HalfEdge e0, e1, e2;
		e0.face = e1.face = e2.face = i;
		int k = m.edges.size();
		e0.id = e1.prev = e2.next = k + 0;
		e1.id = e2.prev = e0.next = k + 1;
		e2.id = e0.prev = e1.next = k + 2;
		e0.v = (short)tris[i][0];
		e1.v = (short)tris[i][1];
		e2.v = (short)tris[i][2];
		m.edges.push_back(e0);
		m.edges.push_back(e1);
		m.edges.push_back(e2);
	}
	m.faces.resize(n);
	for (int i = 0; i<n; i++)
	{
		float3 normal = TriNormal(verts[tris[i][0]], verts[tris[i][1]], verts[tris[i][2]]);
		float  dist = -dot(normal, verts[tris[i][0]]);
		m.faces[i] = float4(normal, dist);
	}
	m.LinkMesh();
	m.InitBackLists();
	return m;
}


inline WingMesh WingMeshCreate(const float3 *verts, const int3 *tris, int n, const int *hidden_edges, int hidden_edges_count)
{
	WingMesh m = WingMeshCreate(verts, tris, n);
	m.RemoveEdges(hidden_edges, hidden_edges_count);
	return m;
}


// int       WingMeshToFaces(WingMesh *m,std::vector<Face*> &faces);  // currently in testbsp.cpp 






#endif
