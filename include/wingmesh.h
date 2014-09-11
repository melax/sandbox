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

class FaceView  // example how to write an iterator for what is a circular indexed list
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
		Iterator& operator++() { if(i>=0) i=wm.edges[i].next; if(i==start) i=-1; return *this;}
		Iterator& operator--() { if(i==start) i=-1; else if(i==-1) i=start; if(i>=0) i=wm.edges[i].prev; return *this;}
		Iterator operator--(int){ Iterator temp(*this); --(*this); return temp; }
		Iterator operator++(int){ Iterator temp(*this); ++(*this); return temp; }
		explicit Iterator(WingMesh &wm,int start, int i) :wm(wm), start(start), i(i){};
		friend class FaceView;
	};
	WingMesh &wm;
	int fid;
	Iterator begin() { return Iterator(wm, wm.fback[fid], wm.fback[fid]); }
	Iterator end()   { return Iterator(wm, wm.fback[fid], - 1); }
	FaceView(WingMesh &wm, int fid) :wm(wm), fid(fid){}
};

// fixme, make the non-const casting not necessary
inline std::vector<float3> WingMeshFaceVerts(const WingMesh &m, int fid) { std::vector<float3> verts; for (auto &e : FaceView((WingMesh&)m, fid))verts.push_back(m.verts[e.v]); return verts; }

inline float3    SupportPoint(const WingMesh *m, const float3& dir) { return m->verts[maxdir(m->verts.data(), m->verts.size(), dir)]; }



inline WingMesh& WingMeshTranslate(WingMesh &m, const float3 &offset) // non-const tranlates mesh passed in
{for (auto &v : m.verts) v += offset;   for (auto &p : m.faces) p.w -= dot(p.xyz(), offset);  return m; }

inline WingMesh& WingMeshRotate(WingMesh &m, const float4 &rot) // non-const tranlates mesh passed in
{ for (auto &v : m.verts) v = qrot(rot, v);  for (auto &p : m.faces) p.xyz() = qrot(rot, p.xyz()); return m; }

inline std::vector<int3> WingMeshTris(const WingMesh &m) // generates a list of indexed triangles from the wingmesh's faces
{
	std::vector<int3> tris;
	tris.reserve(m.edges.size() - m.faces.size() * 2); // predicted size
	for (int e0 : m.fback) // for each face, e0 is first halfedge  // int e0 = m.fback[i]; 
	{
		if (e0 == -1) continue;
		int ea = e0, eb = m.edges[ea].next;
		while ((eb = m.edges[ea = eb].next) != e0)
			tris.push_back(int3(m.edges[e0].v, m.edges[ea].v, m.edges[eb].v));
	}
	return tris;
}

inline int     WingMeshSplitTest(const WingMesh &m, const float4 &plane) { int flag=0; for(auto &v:m.verts)	flag|=PlaneTest(plane,v); return flag;}

inline float   WingMeshVolume(const WingMesh &m) { auto tris = WingMeshTris(m); return Volume(m.verts.data(), tris.data(), tris.size()); }

inline float4& WingMeshComputeFaceNormal(WingMesh &m, int f) { return (m.faces[f] = PolyPlane(WingMeshFaceVerts(m, f))); }

inline int     WingMeshVertDegree(WingMesh &m,int v) {int e0=m.vback[v],e=e0,k=0;  if(e!=-1) do{k++; e=m.edges[m.edges[e].adj].next;}while (e!=e0); return k; }


inline void WingMeshLinkMesh(WingMesh &m)
{
	// Computes adjacency information for edges.
	// Assumes all edges have valid prev,next, and vertex.
	unsigned int i;
	std::vector<short> edgesv;
	edgesv.reserve(m.edges.size());  // edges indexes sorted into ascending edge's vertex index order
	for (i = 0; i<m.edges.size(); i++) edgesv.push_back((short)i);
	std::sort(edgesv.begin(), edgesv.end(), [&m](const short &a, const short &b){ return m.edges[a].v < m.edges[b].v; });
	std::vector<short> veback;
	veback.resize(m.verts.size());
	i = m.edges.size();
	while (i--)
		veback[m.edges[edgesv[i]].v] = (short)i;
	for (i = 0; i<m.edges.size(); i++)
	{
		WingMesh::HalfEdge &e = m.edges[i];
		assert(e.id == i);
		if (e.adj != -1) continue;
		short a = e.v;
		short b = m.edges[e.next].v;
		unsigned int k = veback[b];
		while (k<m.edges.size() && m.edges[edgesv[k]].v == b)
		{
			if (m.edges[m.edges[edgesv[k]].next].v == a)
			{
				e.adj = edgesv[k];
				m.edges[e.adj].adj = e.id;
				break;
			}
			k++;
		}
		assert(e.adj != -1);
	}
}

inline void WingMeshInitBackLists(WingMesh &m)
{
	m.vback.assign(m.verts.size(), -1);
	m.fback.assign(m.faces.size(), -1);
	unsigned int i = m.edges.size();
	while (i--)  // going backward so back pointers will point to the first applicable edge
	{
		if (m.unpacked && m.edges[i].v == -1) continue;
		m.vback[m.edges[i].v] = i;
		m.fback[m.edges[i].face] = i;
	}
}

inline void WingMeshCheck(const WingMesh &wmesh)
{
	const std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	for (unsigned int e = 0; e<edges.size(); e++)
	{
		if (wmesh.unpacked && edges[e].v == -1)
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
	for (unsigned int i = 0; i<wmesh.vback.size(); i++) assert((wmesh.unpacked && wmesh.vback[i] == -1) || edges[wmesh.vback[i]].v == i);
	for (unsigned int i = 0; i<wmesh.fback.size(); i++) assert((wmesh.unpacked && wmesh.fback[i] == -1) || edges[wmesh.fback[i]].face == i);
}


inline void WingMeshEdgeSwap(WingMesh &m, int a, int b)  // untested, using packslotedge typically
{
	std::swap(m.edges[a], m.edges[b]);
	m.edges[m.edges[a].prev].next = m.edges[m.edges[a].next].prev = m.edges[a].id = a;
	m.edges[m.edges[b].prev].next = m.edges[m.edges[b].next].prev = m.edges[b].id = b;
	if (m.vback[m.edges[a].v] == b) m.vback[m.edges[a].v] = a;
	if (m.vback[m.edges[b].v] == a) m.vback[m.edges[b].v] = b;
	if (m.fback[m.edges[a].face] == b) m.fback[m.edges[a].face] = a;
	if (m.fback[m.edges[b].face] == a) m.fback[m.edges[b].face] = b;
}


inline void PackSlotEdge(WingMesh &m, int s)
{
	// used to relocate last edge into slot s
	WingMesh::HalfEdge e = m.edges.back(); m.edges.pop_back();
	if (m.edges.size() == s) return;
	assert(s< (int)m.edges.size());
	assert(e.v >= 0); // last edge is invalid
	if (m.vback.size() && m.vback[e.v] == e.id) m.vback[e.v] = s;
	if (m.fback.size() && m.fback[e.face] == e.id) m.fback[e.face] = s;
	e.id = s;
	m.edges[s] = e;
	m.edges[e.next].prev = e.id;
	m.edges[e.prev].next = e.id;
	m.edges[e.adj].adj = e.id;
}

inline void PackSlotVert(WingMesh &m, int s)
{
	// When you no longer need verts[s], compress away unused vert
	assert(m.vback.size() == m.verts.size());
	assert(m.vback[s] == -1);
	int last = m.verts.size() - 1;
	if (s == last)
	{
		m.verts.pop_back();
		m.vback.pop_back();
		return;
	}
	m.vback[s] = m.vback[last];
	m.verts[s] = m.verts[last];
	int e = m.vback[s];
	assert(e != -1);
	do{ assert(m.edges[e].v == last); m.edges[e].v = s; e = m.edges[m.edges[e].adj].next; } while (e != m.vback[s]);  // iterate over edges coming out of vert 'last' and change to 's'
	m.verts.pop_back();
	m.vback.pop_back();
}

inline void PackSlotFace(WingMesh &m, int s)
{
	// When you no longer need faces[s], compress away unused face
	assert(m.fback.size() == m.faces.size());
	assert(m.fback[s] == -1);
	int last = m.faces.size() - 1;
	if (s == last)
	{
		m.faces.pop_back();
		m.fback.pop_back();
		return;
	}
	m.fback[s] = m.fback[last];
	m.faces[s] = m.faces[last];
	int e = m.fback[s];
	assert(e != -1);
	do{ assert(m.edges[e].face == last); m.edges[e].face = s; e = m.edges[e].next; } while (e != m.fback[s]);
	m.faces.pop_back();
	m.fback.pop_back();
}
inline void SwapFaces(WingMesh &m, int a, int b)
{
	std::swap(m.faces[a], m.faces[b]);
	std::swap(m.fback[a], m.fback[b]);
	if (m.fback[a] != -1)
	{
		int e = m.fback[a];
		do{ assert(m.edges[e].face == b); m.edges[e].face = a; e = m.edges[e].next; } while (e != m.fback[a]);
	}
	if (m.fback[b] != -1)
	{
		int e = m.fback[b];
		do{ assert(m.edges[e].face == a); m.edges[e].face = b; e = m.edges[e].next; } while (e != m.fback[b]);
	}
}
inline void PackFaces(WingMesh &m)  // removes faces with no edges using them
{
	assert(m.fback.size() == m.faces.size());
	unsigned int s = 0;
	for (unsigned int i = 0; i<m.faces.size(); i++)
	{
		if (m.fback[i] == -1) continue;
		if (s<i) SwapFaces(m, s, i);
		s++;
	}
	m.fback.resize(s);
	m.faces.resize(s);
}


inline void WingMeshCompress(WingMesh &m)
{
	// get rid of unused faces and verts
	int i;
	assert(m.vback.size() == m.verts.size());
	assert(m.fback.size() == m.faces.size());
	i = m.edges.size();
	while (i--) if (m.edges[i].v == -1) PackSlotEdge(m, i);
	i = m.vback.size();
	while (i--) if (m.vback[i] == -1) PackSlotVert(m, i);
	PackFaces(m);
	//	i=m.fback.size();
	//	while(i--) if(m.fback[i]==-1) PackSlotFace(m,i);
	m.unpacked = 0;
	WingMeshCheck(m);
}



inline void WingMeshAvoidBackRefs(WingMesh &m, int eid)  // ensure vback and fback reference someone other than eid, i.e. just pick another edge to point to
{
	WingMesh::HalfEdge &E = m.edges[eid];
	assert(E.id == eid);
	assert(E.prev != eid);
	assert(m.edges[m.edges[E.prev].adj].v == E.v);
	assert(m.edges[E.prev].face == E.face);
	if (m.vback[E.v] == eid) m.vback[E.v] = m.edges[E.prev].adj;
	if (m.fback[E.face] == eid) m.fback[E.face] = E.prev;
}

inline void WingMeshCollapseEdge(WingMesh &m, int ea, int pack = 0)
{
	int eb = m.edges[ea].adj;
	WingMesh::HalfEdge &Ea = m.edges[ea];
	WingMesh::HalfEdge &Eb = m.edges[eb];
	WingMesh::HalfEdge &Eap = m.edges[Ea.prev];
	WingMesh::HalfEdge &Ean = m.edges[Ea.next];
	WingMesh::HalfEdge &Ebp = m.edges[Eb.prev];
	WingMesh::HalfEdge &Ebn = m.edges[Eb.next];
	assert(Ea.v >= 0);
	WingMeshAvoidBackRefs(m, Ea.id);
	WingMeshAvoidBackRefs(m, Eb.id);
	int oldv = m.edges[ea].v;
	int newv = m.edges[eb].v;
	assert(Ean.v == newv);
	assert(Ean.face == Ea.face);
	assert(Ebn.face == Eb.face);
	if (m.vback[newv] == eb) m.vback[newv] = Ean.id;
	if (m.fback[Ea.face] == ea) m.fback[Ea.face] = Ean.id;
	if (m.fback[Eb.face] == eb) m.fback[Eb.face] = Ebn.id;
	int e = ea;
	do{ assert(m.edges[e].v == oldv); m.edges[e].v = newv; e = m.edges[m.edges[e].adj].next; } while (e != ea);
	Eap.next = Ean.id;
	Ean.prev = Eap.id;
	Ebp.next = Ebn.id;
	Ebn.prev = Ebp.id;
	m.vback[oldv] = -1;
	Ea.next = Ea.prev = Ea.face = Ea.adj = Ea.v = -1;
	Eb.next = Eb.prev = Eb.face = Eb.adj = Eb.v = -1;
	if (pack && m.unpacked == 0)
	{
		PackSlotEdge(m, std::max(Ea.id, Eb.id));
		PackSlotEdge(m, std::min(Ea.id, Eb.id));
		PackSlotVert(m, oldv);
	}
	else m.unpacked = 1;
	WingMeshCheck(m);
}


inline void WingMeshRemoveEdges(WingMesh &m, const int *cull, int cull_count)
{
	m.vback.clear();
	m.fback.clear();
	m.unpacked = 1;
	for (int i = 0; i<cull_count; i++)
	{
		int e = cull[i];
		WingMesh::HalfEdge &Ea = m.edges[e];
		if (Ea.v == -1) continue; // already snuffed
		WingMesh::HalfEdge &Eb = m.edges[Ea.adj];
		m.edges[Ea.prev].next = m.edges[Eb.next].id;
		m.edges[Eb.prev].next = m.edges[Ea.next].id;
		m.edges[Ea.next].prev = m.edges[Eb.prev].id;
		m.edges[Eb.next].prev = m.edges[Ea.prev].id;
		Ea.next = Ea.prev = Ea.face = Ea.adj = Ea.v = -1;
		Eb.next = Eb.prev = Eb.face = Eb.adj = Eb.v = -1;
	}
	for (unsigned int i = 0; i<m.edges.size(); i++)  // share faces now
	{
		WingMesh::HalfEdge &E = m.edges[i];
		if (E.v == -1) continue; // dead edge
		if (E.face == E.Next().face) continue; // 
		for (int e = E.next; e != i; e = m.edges[e].next)
		{
			m.edges[e].face = E.face;
		}
	}
	WingMeshInitBackLists(m);
	WingMeshCheck(m);
	WingMeshCompress(m);
}



inline void WingMeshSort(WingMesh &m)
{
	if (m.unpacked) WingMeshCompress(m);
	std::vector<WingMesh::HalfEdge> s;
	s.reserve(m.edges.size());
	int k = 0;
	for (unsigned int i = 0; i<m.faces.size(); i++)
	{
		int e = m.fback[i]; assert(e != -1);
		do{ s.push_back(m.edges[e]); m.edges[e].id = s.size() - 1; e = m.edges[e].next; } while (e != m.fback[i]);
	}
	for (unsigned int i = 0; i<s.size(); i++)
	{
		s[i].id = i;
		s[i].next = m.edges[s[i].next].id;
		s[i].prev = m.edges[s[i].prev].id;
	}
	s.swap(m.edges);
	WingMeshCheck(m);
	for (unsigned int i = 0; i<m.edges.size(); i++) { assert(m.edges[i].next <= (int)i + 1); }
}








inline void WingMeshSeparate(WingMesh &m, const std::vector<int> &edgeloop)
{
	std::vector<int> killstack;
	for (unsigned int i = 0; i<edgeloop.size(); i++)
	{
		// dont want to delete vertices along the loop.
		// detach vertices along loop from to-be-deleted edges  
		int ec = edgeloop[i];
		int en = edgeloop[(i + 1) % edgeloop.size()];
		int e = m.edges[ec].next;
		while (e != en)
		{
			assert(m.edges[e].v == m.edges[en].v);
			m.edges[e].v = -1;
			e = m.edges[e].Adj().next;
		}
		m.vback[m.edges[en].v] = en; // ensure vertex points to edge that wont be deleted.
		m.fback[m.edges[en].face] = -1;  // delete all faces 
		m.edges[en].face = m.edges[edgeloop[0]].face; // assign all loop faces to same which we ressurrect later
	}
	for (unsigned int i = 0; i<edgeloop.size(); i++)
	{
		// detatch tobedeleted edges from loop edges.
		// and fix loop's next/prev links.
		int ec = edgeloop[i];
		int en = edgeloop[(i + 1) % edgeloop.size()];
		int ep = edgeloop[(i) ? i - 1 : edgeloop.size() - 1];
		WingMesh::HalfEdge &E = m.edges[ec];
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
		WingMesh::HalfEdge &K = m.edges[k];
		assert(K.id == -1);
		if (K.next != -1 && m.edges[K.next].id != -1) { killstack.push_back(K.next); m.edges[K.next].id = -1; }
		if (K.prev != -1 && m.edges[K.prev].id != -1) { killstack.push_back(K.prev); m.edges[K.prev].id = -1; }
		if (K.adj != -1 && m.edges[K.adj].id != -1) { killstack.push_back(K.adj); m.edges[K.adj].id = -1; }
		if (K.v != -1) m.vback[K.v] = -1;
		if (K.face != -1) m.fback[K.face] = -1;
		K.next = K.prev = K.adj = K.v = K.face = -1;
	}
	assert(m.fback[m.edges[edgeloop[0]].face] == -1);
	m.fback[m.edges[edgeloop[0]].face] = edgeloop[0];
	WingMeshComputeFaceNormal(m, m.edges[edgeloop[0]].face);
	SwapFaces(m, m.edges[edgeloop[0]].face, 0);
	m.unpacked = 1;
	WingMeshCheck(m);
	WingMeshCompress(m);
}


inline int WingMeshBuildEdge(WingMesh& wmesh, int ea, int eb)
{
	// puts an edge from ea.v to eb.v  
	std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	assert(edges[ea].next != eb);  // make sure they aren't too close
	assert(edges[eb].next != ea);
	assert(edges[eb].face == edges[ea].face);
	int e = ea;
	while (e != eb)
	{
		e = edges[e].next;
		assert(e != ea);  // make sure eb is on the same poly
	}
	int newface = wmesh.faces.size();
	WingMesh::HalfEdge sa(edges.size() + 0, edges[ea].v, edges.size() + 1, eb, edges[ea].prev, newface);  // id,v,adj,next,prev,face
	WingMesh::HalfEdge sb(edges.size() + 1, edges[eb].v, edges.size() + 0, ea, edges[eb].prev, edges[ea].face);
	edges[sa.prev].next = edges[sa.next].prev = sa.id;
	edges[sb.prev].next = edges[sb.next].prev = sb.id;
	edges.push_back(sa);
	edges.push_back(sb);
	wmesh.faces.push_back(wmesh.faces[edges[ea].face]);
	if (wmesh.fback.size()) {
		wmesh.fback.push_back(sa.id);
		wmesh.fback[sb.face] = sb.id;
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


inline void SplitEdge(WingMesh &m, int e, const float3 &vpos)
{
	// Add's new vertex vpos to mesh m
	// Adds two new halfedges to m
	std::vector<WingMesh::HalfEdge> &edges = m.edges;
	int ea = edges[e].adj;
	int v = m.verts.size();
	WingMesh::HalfEdge s0(edges.size() + 0, v, edges.size() + 1, edges[e].next, e, edges[e].face);  // id,v,adj,next,prev,face
	WingMesh::HalfEdge sa(edges.size() + 1, edges[ea].v, edges.size() + 0, ea, edges[ea].prev, edges[ea].face);
	edges[s0.prev].next = edges[s0.next].prev = s0.id;
	edges[sa.prev].next = edges[sa.next].prev = sa.id;
	edges[ea].v = v;
	edges.push_back(s0);
	edges.push_back(sa);
	m.verts.push_back(vpos);
	if (m.vback.size())
	{
		m.vback.push_back(s0.id);
		m.vback[sa.v] = sa.id;
	}
}

inline void SplitEdges(WingMesh &wmesh, const float4 &split)
{
	std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	for (unsigned int e = 0; e<edges.size(); e++)
	{
		int ea = edges[e].adj;
		if ((PlaneTest(split, wmesh.verts[edges[e].v]) | PlaneTest(split, wmesh.verts[edges[ea].v])) == SPLIT)
		{
			float3 vpos = PlaneLineIntersection(split, wmesh.verts[edges[e].v], wmesh.verts[edges[ea].v]);
			assert(PlaneTest(split, vpos) == COPLANAR);
			SplitEdge(wmesh, e, vpos);
		}
	}
}

inline int findcoplanaredge(WingMesh &m, int v, const float4 &slice)
{
	// assumes the mesh straddles the plane slice.
	// tesselates the mesh if required.
	int e = m.vback[v];
	int es = e;
	while (PlaneTest(slice, m.verts[m.edges[e].Adj().v]) != UNDER)
	{
		e = m.edges[e].Prev().Adj().id;
		assert(e != es); // if(e==es) return -1; // all edges point over!
	}
	es = e;
	while (PlaneTest(slice, m.verts[m.edges[e].Adj().v]) == UNDER)
	{
		e = m.edges[e].Adj().Next().id;
		assert(e != es); // if(e==es) return -1; // all edges point UNDER!
	}
	int ec = m.edges[e].next;
	while (PlaneTest(slice, m.verts[m.edges[ec].v]) != COPLANAR)
	{
		ec = m.edges[ec].next;
		assert(ec != e);
	}
	if (ec == m.edges[e].next) { return e; }
	assert(ec != m.edges[e].prev);
	return WingMeshBuildEdge(m, e, ec);

}

inline int findcoplanarvert(const WingMesh &m, const float4 &slice)
{
	for (unsigned int i = 0; i<m.verts.size(); i++)
	{
		if (PlaneTest(slice, m.verts[i]) == COPLANAR)
		{
			return i;
		}
	}
	return -1;
}

inline void WingMeshTess(WingMesh &m, const float4 &slice, std::vector<int> &loop)
{
	SplitEdges(m, slice);
	loop.clear();
	int v0 = findcoplanarvert(m, slice);
	if (v0 == -1) return; //??
	int v = v0;
	do{
		int e = findcoplanaredge(m, v, slice);
		v = m.edges[e].Adj().v;
		loop.push_back(e);
	} while (v != v0);

}

inline WingMesh WingMeshCrop(const WingMesh &_m, const float4 &slice)
{
	std::vector<int> coplanar; // edges
	int s = WingMeshSplitTest(_m, slice);
	if (s == OVER) return WingMesh(); //  NULL;
	if (s == UNDER) return _m;
	WingMesh m = _m;
	WingMeshTess(m, slice, coplanar);
	std::vector<int> reverse;
	for (unsigned int i = 0; i<coplanar.size(); i++) reverse.push_back(m.edges[coplanar[coplanar.size() - 1 - i]].adj);

	if (coplanar.size()) WingMeshSeparate(m, reverse);
	WingMeshCheck(m);
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
		WingMeshComputeFaceNormal(d, f);    // this shouldn't change the plane equation by a significant amount

	WingMeshCheck(d);
	return d;
}




inline WingMesh WingMeshCube(const float3 &bmin, const float3 &bmax)
{
	WingMesh wm;
	wm.verts = { { bmin.x, bmin.y, bmin.z }, { bmin.x, bmin.y, bmax.z }, { bmin.x, bmax.y, bmin.z }, { bmin.x, bmax.y, bmax.z },
	{ bmax.x, bmin.y, bmin.z }, { bmax.x, bmin.y, bmax.z }, { bmax.x, bmax.y, bmin.z }, { bmax.x, bmax.y, bmax.z }, };
	wm.faces = { { -1, 0, 0, bmin.x }, { 1, 0, 0, -bmax.x }, { 0, -1, 0, bmin.y }, { 0, 1, 0, -bmax.y }, { 0, 0, -1, bmin.z }, { 0, 0, 1, -bmax.z }, };
	wm.edges =
	{
		{ 0, 0, 11, 1, 3, 0 }, { 1, 1, 23, 2, 0, 0 }, { 2, 3, 15, 3, 1, 0 }, { 3, 2, 16, 0, 2, 0 },
		{ 4, 6, 13, 5, 7, 1 }, { 5, 7, 21, 6, 4, 1 }, { 6, 5, 9, 7, 5, 1 }, { 7, 4, 18, 4, 6, 1 },
		{ 8, 0, 19, 9, 11, 2 }, { 9, 4, 6, 10, 8, 2 }, { 10, 5, 20, 11, 9, 2 }, { 11, 1, 0, 8, 10, 2 },
		{ 12, 3, 22, 13, 15, 3 }, { 13, 7, 4, 14, 12, 3 }, { 14, 6, 17, 15, 13, 3 }, { 15, 2, 2, 12, 14, 3 },
		{ 16, 0, 3, 17, 19, 4 }, { 17, 2, 14, 18, 16, 4 }, { 18, 6, 7, 19, 17, 4 }, { 19, 4, 8, 16, 18, 4 },
		{ 20, 1, 10, 21, 23, 5 }, { 21, 5, 5, 22, 20, 5 }, { 22, 7, 12, 23, 21, 5 }, { 23, 3, 1, 20, 22, 5 },
	};
	WingMeshInitBackLists(wm);
	WingMeshCheck(wm);
	return wm;
}

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
	WingMeshLinkMesh(m);
	WingMeshInitBackLists(m);
	return m;
}


inline WingMesh WingMeshCreate(const float3 *verts, const int3 *tris, int n, const int *hidden_edges, int hidden_edges_count)
{
	WingMesh m = WingMeshCreate(verts, tris, n);
	WingMeshRemoveEdges(m, hidden_edges, hidden_edges_count);
	return m;
}


// int       WingMeshToFaces(WingMesh *m,std::vector<Face*> &faces);  // currently in testbsp.cpp 






#endif
