//
// half-edge winged mesh data structure
// (c) stan melax 2007    bsd licence
//
// primary usage of this module is to represent and operate 
// on convex cells of space in the spatial partition tree.
// Can also be used to represent convex shapes for rigidbody simulation.
// 

#include "wingmesh.h"
#include <algorithm>
#include <assert.h>



/*
class WingMesh
{
  public:
	class HalfEdge
	{
	  public:
	    short id;
		short v;
		short adj;
		short next;
		short prev;
		short face;
		short vm;
		HalfEdge(){vm=v=adj=next=prev=face=-1;}
	};
	std::vector<HalfEdge> edges;
	std::vector<float3> verts;
	std::vector<Plane>  faces;
	std::vector<short>  vback;
	std::vector<short>  fback;
	int unpacked; // flag indicating if any unused elements within arrays
};
*/





static int linkmesh(WingMesh &m)
{
	// Computes adjacency information for edges.
	// Assumes prev/next pointers are assigned for each edge.
	// Assumes edges have vertices assigned to them.
	unsigned int i;
    std::vector<short> edgesv;
    edgesv.reserve(m.edges.size());  // edges indexes sorted into ascending edge's vertex index order
	for(i=0;i<m.edges.size();i++) edgesv.push_back((short)i);
	std::sort(edgesv.begin(), edgesv.end(), [&m](const short &a, const short &b){ return m.edges[a].v < m.edges[b].v; });
	std::vector<short> veback;
    veback.resize(m.verts.size());
	i=m.edges.size();
	while(i--)
	{
		veback[m.edges[edgesv[i]].v]=(short)i;
	}
	for(i=0;i<m.edges.size();i++)
	{
		WingMesh::HalfEdge &e = m.edges[i];
		assert(e.id==i);
		if(e.adj!=-1) continue;
		short a = e.v;
		short b = m.edges[e.next].v;
		unsigned int k=veback[b];
        while (k<m.edges.size() && m.edges[edgesv[k]].v == b)
		{
			if(m.edges[m.edges[edgesv[k]].next].v == a)
			{
				e.adj = edgesv[k];
				m.edges[e.adj].adj=e.id;
				break;
			}
			k++;
		}
		assert(e.adj!=-1);
	}
	return 1;
}

static void WingMeshInitBackLists(WingMesh &m)
{
	unsigned int i;
    m.vback.resize(m.verts.size());
	for(i=0;i<m.vback.size();i++) m.vback[i]=-1;
    m.fback.resize(m.faces.size());
	for(i=0;i<m.fback.size();i++) m.fback[i]=-1;
	i=m.edges.size();
	while(i--)  // going backward so back pointers will point to the first applicable edge
	{
		if(m.unpacked && m.edges[i].v==-1) continue;
		m.vback[m.edges[i].v]=i;
		m.fback[m.edges[i].face]=i;
	}
}

void checkit(WingMesh &wmesh)
{
	std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	unsigned int e,i;
	for(e=0;e<edges.size();e++)
	{
		if(wmesh.unpacked && edges[e].v==-1)
		{
			assert(edges[e].face==-1);
			assert(edges[e].next==-1);
			assert(edges[e].prev==-1);
			continue;
		}
		assert(edges[e].id==e);
		assert(edges[e].v>=0);
		assert(edges[e].face>=0);
		assert(edges[edges[e].next].prev==e);
		assert(edges[edges[e].prev].next==e);
		assert(edges[e].adj !=e);  // antireflexive
		assert(edges[edges[e].adj ].adj ==e);  // symmetric
		assert(edges[e].v == edges[edges[edges[e].adj].next].v);
		assert(edges[e].v != edges[edges[e].adj].v);
	}
	for(i=0;i<wmesh.vback.size();i++) assert((wmesh.unpacked && wmesh.vback[i]==-1) || edges[wmesh.vback[i]].v   ==i);
	for(i=0;i<wmesh.fback.size();i++) assert((wmesh.unpacked && wmesh.fback[i]==-1) || edges[wmesh.fback[i]].face==i);
}

float4 &ComputeNormal(WingMesh &m,int f)
{
	float3 c(0,0,0);
	float3 n(0,0,0);
	int e0 = m.fback[f];
	int e=e0;
	int k=0;
	do
	{
		int en = m.edges[e].next;
		float3 &v0 = m.verts[m.edges[e ].v];
		float3 &v1 = m.verts[m.edges[en].v];
		n.x += (v0.y - v1.y) * (v0.z + v1.z);
		n.y += (v0.z - v1.z) * (v0.x + v1.x);
		n.z += (v0.x - v1.x) * (v0.y + v1.y);
		c += v0;
		e = en;
		k++;
	} while(e!=e0);
	float4 &p = m.faces[f];
	p.xyz() = normalize(n);
	p.w     = -dot(c,p.xyz())/(float)k;
	return p;
}

WingMesh WingMeshCreate(float3 *verts,int3 *tris,int n)
{
	WingMesh m;
	if (n == 0) return m;
	int verts_count = -1;
	for(int i=0;i<n;i++) for(int j=0;j<3;j++)
	{
		verts_count = std::max(verts_count,tris[i][j]);
	}
	verts_count++;
	m.verts.resize(verts_count);
	for(unsigned int i=0;i<m.verts.size();i++)
	{
		m.verts[i]=verts[i];
	}
	for(int i=0;i<n;i++)
	{
		int3 &t=tris[i];
		WingMesh::HalfEdge e0,e1,e2;
		e0.face=e1.face=e2.face=i;
		int k=m.edges.size();
		e0.id = e1.prev = e2.next = k+0;
		e1.id = e2.prev = e0.next = k+1;
		e2.id = e0.prev = e1.next = k+2;
		e0.v = (short)tris[i][0];
		e1.v = (short)tris[i][1];
		e2.v = (short)tris[i][2];
		m.edges.push_back(e0);
		m.edges.push_back(e1);
		m.edges.push_back(e2);
	}
	m.faces.resize(n);
	for(int i=0;i<n;i++)
	{
		float3 normal = TriNormal(verts[tris[i][0]],verts[tris[i][1]],verts[tris[i][2]]);
		float  dist   = -dot(normal,verts[tris[i][0]]);
		m.faces[i] = float4(normal,dist);
	}
	linkmesh(m);
	WingMeshInitBackLists(m);
	return m;
}
static void SwapSlots(WingMesh &m,int a,int b)
{
	std::swap(m.edges[a], m.edges[b]);
	m.edges[m.edges[a].prev].next=m.edges[m.edges[a].next].prev=m.edges[a].id=a;
	m.edges[m.edges[b].prev].next=m.edges[m.edges[b].next].prev=m.edges[b].id=b;
}


static void PackSlotEdge(WingMesh &m,int s)
{
	// used to relocate last edge into slot s
	WingMesh::HalfEdge e = m.edges.back(); m.edges.pop_back();
    if (m.edges.size() == s) return;
    assert(s< (int)m.edges.size());
	assert(e.v>=0); // last edge is invalid
    if (m.vback.size() && m.vback[e.v] == e.id) m.vback[e.v] = s;
    if (m.fback.size() && m.fback[e.face] == e.id) m.fback[e.face] = s;
	e.id=s;
	m.edges[s] = e;
	m.edges[e.next].prev = e.id;
	m.edges[e.prev].next = e.id;
	m.edges[e.adj ].adj  = e.id;
}

static void PackSlotVert(WingMesh &m,int s)
{
	// When you no longer need verts[s], compress away unused vert
    assert(m.vback.size() == m.verts.size());
	assert(m.vback[s]==-1);
	int last = m.verts.size()-1;
	if(s==last) 
	{
		m.verts.pop_back();
		m.vback.pop_back();
		return;
	}
	m.vback[s] = m.vback[last];
	m.verts[s] = m.verts[last];
	int e=m.vback[s];
	assert(e!=-1);
	do{assert( m.edges[e].v==last); m.edges[e].v=s;e=m.edges[m.edges[e].adj].next;} while (e!=m.vback[s]);
    m.verts.pop_back();
    m.vback.pop_back();
}

static void PackSlotFace(WingMesh &m,int s)
{
	// When you no longer need faces[s], compress away unused face
    assert(m.fback.size() == m.faces.size());
	assert(m.fback[s]==-1);
	int last = m.faces.size()-1;
	if(s==last) 
	{
        m.faces.pop_back();
        m.fback.pop_back();
		return;
	}
	m.fback[s] = m.fback[last];
	m.faces[s] = m.faces[last];
	int e=m.fback[s];
	assert(e!=-1);
	do{assert( m.edges[e].face==last); m.edges[e].face=s;e=m.edges[e].next;} while (e!=m.fback[s]);
    m.faces.pop_back();
    m.fback.pop_back();
}
static void SwapFaces(WingMesh &m,int a,int b)
{
	std::swap(m.faces[a], m.faces[b]);
	std::swap(m.fback[a], m.fback[b]);
	if(m.fback[a] != -1)
	{
		int e=m.fback[a];
		do{assert( m.edges[e].face==b); m.edges[e].face=a;e=m.edges[e].next;} while (e!=m.fback[a]);
	}
	if(m.fback[b] != -1)
	{
		int e=m.fback[b];
		do{assert( m.edges[e].face==a); m.edges[e].face=b;e=m.edges[e].next;} while (e!=m.fback[b]);
	}
}
static void PackFaces(WingMesh &m)
{
    assert(m.fback.size() == m.faces.size());
	unsigned int s=0;
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		if(m.fback[i] == -1) continue;
		if(s<i) SwapFaces(m,s,i);
		s++;
	}
	m.fback.resize(s);
    m.faces.resize(s);
}


void WingMeshCompress(WingMesh &m)
{
	// get rid of unused faces and verts
	int i;
    assert(m.vback.size() == m.verts.size());
    assert(m.fback.size() == m.faces.size());
	i=m.edges.size();
	while(i--) if(m.edges[i].v==-1) PackSlotEdge(m,i);
	i=m.vback.size();
	while(i--) if(m.vback[i]==-1) PackSlotVert(m,i);
	PackFaces(m);
//	i=m.fback.size();
//	while(i--) if(m.fback[i]==-1) PackSlotFace(m,i);
	m.unpacked=0;
	checkit(m);
}



int WingMeshIsDegTwo(const WingMesh &m,int v)
{
    assert(m.vback.size());
	int e = m.vback[v];
	return m.edges[e].prev == m.edges[m.edges[m.edges[e].adj].next].adj;
}

static void BackPointElsewhere(WingMesh &m,int e)
{
	// ensure vback and fback reference someone other than e
	WingMesh::HalfEdge &E = m.edges[e];
	assert(E.id==e);
	assert(E.prev!=e);
	assert(m.edges[m.edges[E.prev].adj].v==E.v);
	assert(m.edges[E.prev].face==E.face);
	if(m.vback[E.v   ]==e) m.vback[E.v   ]= m.edges[E.prev].adj;
	if(m.fback[E.face]==e) m.fback[E.face]= E.prev;
}

void WingMeshCollapseEdge(WingMesh &m,int ea,int pack=0)
{
	int eb = m.edges[ea].adj;
	WingMesh::HalfEdge &Ea  = m.edges[ea];
	WingMesh::HalfEdge &Eb  = m.edges[eb];
	WingMesh::HalfEdge &Eap = m.edges[Ea.prev];
	WingMesh::HalfEdge &Ean = m.edges[Ea.next];
	WingMesh::HalfEdge &Ebp = m.edges[Eb.prev];
	WingMesh::HalfEdge &Ebn = m.edges[Eb.next];
	assert(Ea.v>=0);
	BackPointElsewhere(m,Ea.id);
	BackPointElsewhere(m,Eb.id);
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
		PackSlotEdge(m,std::max(Ea.id,Eb.id));
		PackSlotEdge(m,std::min(Ea.id,Eb.id));
		PackSlotVert(m,oldv);
	}
	else m.unpacked=1;
	checkit(m);
}

void RemoveEdge(WingMesh &m,int e,int pack=0)
{
	WingMesh::HalfEdge &Ea = m.edges[e];
	WingMesh::HalfEdge &Eb = m.edges[Ea.adj];
	BackPointElsewhere(m,Ea.id);
	BackPointElsewhere(m,Eb.id);
	int oldface = Ea.face;
	while(m.edges[e].face != Eb.face)
	{
		m.edges[e].face = Eb.face;
		e = m.edges[e].next;
	}
	assert(e==Ea.id);
	m.edges[Ea.prev].next = m.edges[Eb.next].id;
	m.edges[Eb.prev].next = m.edges[Ea.next].id;
	m.edges[Ea.next].prev = m.edges[Eb.prev].id;
	m.edges[Eb.next].prev = m.edges[Ea.prev].id;
	Ea.next=Ea.prev=Ea.face=Ea.adj=Ea.v=-1;
	Eb.next=Eb.prev=Eb.face=Eb.adj=Eb.v=-1;
	m.fback[oldface]=-1;
	if(pack)
	{
		PackSlotFace(m,oldface);
		PackSlotEdge(m,std::max(Ea.id,Eb.id));
		PackSlotEdge(m,std::min(Ea.id,Eb.id));
	}
	else m.unpacked=1;
	checkit(m);
}

void RemoveEdges(WingMesh &m,int *cull,int cull_count)
{
	m.vback.clear();
	m.fback.clear();
	m.unpacked=1;
	for(int i=0;i<cull_count;i++)
	{
		int e = cull[i];
		WingMesh::HalfEdge &Ea = m.edges[e];
		if(Ea.v==-1) continue; // already snuffed
		WingMesh::HalfEdge &Eb = m.edges[Ea.adj];
		m.edges[Ea.prev].next = m.edges[Eb.next].id;
		m.edges[Eb.prev].next = m.edges[Ea.next].id;
		m.edges[Ea.next].prev = m.edges[Eb.prev].id;
		m.edges[Eb.next].prev = m.edges[Ea.prev].id;
		Ea.next=Ea.prev=Ea.face=Ea.adj=Ea.v=-1;
		Eb.next=Eb.prev=Eb.face=Eb.adj=Eb.v=-1;
	}
	for(unsigned int i=0;i<m.edges.size();i++)  // share faces now
	{
		WingMesh::HalfEdge &E = m.edges[i];
		if(E.v==-1) continue; // dead edge
		if(E.face==E.Next()->face) continue; // 
		for(int e=E.next; e!=i ; e=m.edges[e].next)
		{
			m.edges[e].face = E.face;
		}
	}
	WingMeshInitBackLists(m);
	checkit(m);
	WingMeshCompress(m);
}

int RemoveD2(WingMesh &m)
{
	int k=0;
	unsigned int  i = m.edges.size();
	if(m.faces.size()<3) return 0;
	while(i--)
	{
        if (i >= m.edges.size()) continue;
		if(m.edges[i].prev == m.edges[i].Adj()->Next()->adj)
		{
			WingMeshCollapseEdge(m,i,1);
			k++;
		}
	}
	return k;
}


void WingMeshSort(WingMesh &m)
{
	if(m.unpacked) WingMeshCompress(m);
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
	}
    s.swap(m.edges);
	checkit(m);
	for(unsigned int i=0;i<m.edges.size();i++) {assert(m.edges[i].next<=(int)i+1);}
}





WingMesh WingMeshCreate(float3 *verts,int3 *tris,int n,int *hidden_edges,int hidden_edges_count)
{
	WingMesh m=WingMeshCreate(verts,tris,n);
	RemoveEdges(m,hidden_edges,hidden_edges_count);
	return m;
}

WingMesh WingMeshDual(const WingMesh &m, float r, const float3 &p)
{
	WingMesh d;  // new WingMesh
    d.faces.resize(m.verts.size());
    d.fback.resize(m.vback.size());
	for(unsigned int i=0;i<m.verts.size();i++)
	{
		d.faces[i] = float4(normalize(m.verts[i]),r*r/magnitude(m.verts[i]));
		d.fback[i] = m.vback[i];
	}
    d.verts.resize(m.faces.size());
    d.vback.resize(m.fback.size());
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		d.verts[i] = m.faces[i].xyz()*(-r*r/m.faces[i].w);
		d.vback[i] = m.edges[m.fback[i]].adj;
	}
    d.edges.resize(m.edges.size());
	for(unsigned int i=0;i<m.edges.size();i++)
	{
		d.edges[i] = m.edges[i];
		d.edges[i].face = m.edges[i].v;
		d.edges[i].v = m.edges[m.edges[i].adj].face;
		d.edges[i].next = m.edges[m.edges[i].prev].adj ;
		d.edges[i].prev = m.edges[m.edges[i].adj ].next;
	}
	checkit(d);
	return d;
}


WingMesh WingMeshCube(const float3 &bmin,const float3 &bmax)
{
	WingMesh wm;
	wm.verts.resize(8);
	wm.verts[0] = float3(bmin.x,bmin.y,bmin.z);
	wm.verts[1] = float3(bmin.x,bmin.y,bmax.z);
	wm.verts[2] = float3(bmin.x,bmax.y,bmin.z);
	wm.verts[3] = float3(bmin.x,bmax.y,bmax.z);
	wm.verts[4] = float3(bmax.x,bmin.y,bmin.z);
	wm.verts[5] = float3(bmax.x,bmin.y,bmax.z);
	wm.verts[6] = float3(bmax.x,bmax.y,bmin.z);
	wm.verts[7] = float3(bmax.x,bmax.y,bmax.z);
	wm.faces.resize(6);
	wm.faces[0] = float4(float3(-1,0,0), bmin.x);
	wm.faces[1] = float4(float3(1,0,0), -bmax.x);
	wm.faces[2] = float4(float3(0,-1,0), bmin.y);
	wm.faces[3] = float4(float3(0,1,0), -bmax.y);
	wm.faces[4] = float4(float3(0,0,-1), bmin.z);
	wm.faces[5] = float4(float3(0,0,1), -bmax.z);
	wm.edges.resize(24);
	wm.edges[0 ] = WingMesh::HalfEdge( 0,0,11, 1, 3,0);
	wm.edges[1 ] = WingMesh::HalfEdge( 1,1,23, 2, 0,0);
	wm.edges[2 ] = WingMesh::HalfEdge( 2,3,15, 3, 1,0);
	wm.edges[3 ] = WingMesh::HalfEdge( 3,2,16, 0, 2,0);
										 
	wm.edges[4 ] = WingMesh::HalfEdge( 4,6,13, 5, 7,1);
	wm.edges[5 ] = WingMesh::HalfEdge( 5,7,21, 6, 4,1);
	wm.edges[6 ] = WingMesh::HalfEdge( 6,5, 9, 7, 5,1);
	wm.edges[7 ] = WingMesh::HalfEdge( 7,4,18, 4, 6,1);

	wm.edges[8 ] = WingMesh::HalfEdge( 8,0,19, 9,11,2);
	wm.edges[9 ] = WingMesh::HalfEdge( 9,4, 6,10, 8,2);
	wm.edges[10] = WingMesh::HalfEdge(10,5,20,11, 9,2);
	wm.edges[11] = WingMesh::HalfEdge(11,1, 0, 8,10,2);

	wm.edges[12] = WingMesh::HalfEdge(12,3,22,13,15,3);
	wm.edges[13] = WingMesh::HalfEdge(13,7, 4,14,12,3);
	wm.edges[14] = WingMesh::HalfEdge(14,6,17,15,13,3);
	wm.edges[15] = WingMesh::HalfEdge(15,2, 2,12,14,3);

	wm.edges[16] = WingMesh::HalfEdge(16,0, 3,17,19,4);
	wm.edges[17] = WingMesh::HalfEdge(17,2,14,18,16,4);
	wm.edges[18] = WingMesh::HalfEdge(18,6, 7,19,17,4);
	wm.edges[19] = WingMesh::HalfEdge(19,4, 8,16,18,4);

	wm.edges[20] = WingMesh::HalfEdge(20,1,10,21,23,5);
	wm.edges[21] = WingMesh::HalfEdge(21,5, 5,22,20,5);
	wm.edges[22] = WingMesh::HalfEdge(22,7,12,23,21,5);
	wm.edges[23] = WingMesh::HalfEdge(23,3, 1,20,22,5);
	WingMeshInitBackLists(wm);
	checkit(wm);
	return wm;
}

void Polyize(WingMesh &m,float angle=2.0f)
{
	unsigned int i=m.edges.size();
	while(i--) 
	{
        if (i >= m.edges.size() || m.edges[i].v<0) continue; // unused
		WingMesh::HalfEdge &Ea = m.edges[i];
		WingMesh::HalfEdge &Eb = m.edges[Ea.adj];
		if(Ea.next==Ea.prev)
		{
			RemoveEdge(m,i,1);
		}
		else if(Eb.next==Eb.prev)
		{
			RemoveEdge(m,Ea.adj,1);
		}
		else if(Ea.prev==Eb.Next()->adj)
		{
			WingMeshCollapseEdge(m,Ea.id,1);
		}
		else if(Eb.prev==Ea.Next()->adj)
		{
			WingMeshCollapseEdge(m,Eb.id,1);
		}
		else if(dot(m.faces[Ea.face].xyz(),m.faces[Eb.face].xyz())> cosf(angle*3.14f/180.0f))
		{
			RemoveEdge(m,i,1);
		}
	}
}



int degree(WingMesh &m,int v)
{
	int e=m.vback[v];
	if(e==-1) return 0;
	int k=0;
	do{ k++ ; e = m.edges[m.edges[e].adj].next;} while (e!=m.vback[v]);
	return k;
}



void WingMeshSeparate(WingMesh &m,const std::vector<int> &edgeloop)
{
	std::vector<int> killstack;
	for(unsigned int i=0;i<edgeloop.size();i++)
	{
		// dont want to delete vertices along the loop.
		// detach vertices along loop from to-be-deleted edges  
		int ec = edgeloop[i];
        int en = edgeloop[(i + 1) % edgeloop.size()];
		int e  = m.edges[ec].next;
		while(e!=en)
		{
			assert(m.edges[e].v == m.edges[en].v);
			m.edges[e].v=-1;
			e = m.edges[e].Adj()->next;
		}
		m.vback[m.edges[en].v]=en; // ensure vertex points to edge that wont be deleted.
		m.fback[m.edges[en].face] = -1;  // delete all faces 
		m.edges[en].face = m.edges[edgeloop[0]].face; // assign all loop faces to same which we ressurrect later
	}
	for(unsigned int i=0;i<edgeloop.size();i++)
	{
		// detatch tobedeleted edges from loop edges.
		// and fix loop's next/prev links.
		int ec = edgeloop[i];
        int en = edgeloop[(i + 1) % edgeloop.size()];
        int ep = edgeloop[(i) ? i - 1 : edgeloop.size() - 1];
		WingMesh::HalfEdge &E = m.edges[ec];
		if(E.next != en)
		{
			WingMesh::HalfEdge *K = E.Next();
			if(K->id>=0) killstack.push_back(E.next);
			K->id=-1;
			assert(K->v==-1);
			assert(K->prev==E.id);
			K->prev=-1;
			E.next=en;
		}
		if(E.prev != ep)
		{
			WingMesh::HalfEdge *K = E.Prev();
			if(K->id>=0) killstack.push_back(E.prev);
			K->id=-1;
			assert(K->next==E.id);
			K->next=-1;
			E.prev=ep;
		}
	}
    while (killstack.size())
	{
		// delete (make "-1") all unwanted edges faces and verts
		int k = killstack.back(); killstack.pop_back(); //  Pop(killstack);
		assert(k>=0);
		WingMesh::HalfEdge &K = m.edges[k];
		assert(K.id==-1);
		if(K.next!=-1 && m.edges[K.next].id!=-1) {killstack.push_back(K.next);m.edges[K.next].id=-1;}
		if(K.prev!=-1 && m.edges[K.prev].id!=-1) {killstack.push_back(K.prev);m.edges[K.prev].id=-1;}
		if(K.adj !=-1 && m.edges[K.adj ].id!=-1) {killstack.push_back(K.adj );m.edges[K.adj ].id=-1;}
		if(K.v   !=-1) m.vback[K.v   ]=-1;
		if(K.face!=-1) m.fback[K.face]=-1;
		K.next=K.prev=K.adj=K.v=K.face = -1;
	}
	assert(m.fback[m.edges[edgeloop[0]].face]==-1);
	m.fback[m.edges[edgeloop[0]].face]=edgeloop[0];
	ComputeNormal(m,m.edges[edgeloop[0]].face);
	SwapFaces(m,m.edges[edgeloop[0]].face,0);
	m.unpacked=1;
	checkit(m);
	WingMeshCompress(m);
}

int FaceSideCount(WingMesh &m,int f)
{
	int k=0;
	int e=m.fback[f];
	do{k++;e=m.edges[e].next;} while (e!=m.fback[f]);
	return k;
}






int Tess(WingMesh& wmesh,int ea,int eb)
{
	// puts an edge from ea.v to eb.v  
	std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	assert(edges[ea].next != eb);  // make sure they aren't too close
	assert(edges[eb].next != ea);
	assert(edges[eb].face == edges[ea].face);
	int e=ea;
	while(e!=eb)
	{
		e=edges[e].next;
		assert(e!=ea);  // make sure eb is on the same poly
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
		wmesh.fback[sb.face]=sb.id;
	}
	e=edges[sa.id].next;
	while(e!=sa.id)
	{
		assert(e!=ea);
		edges[e].face = newface;
		e = edges[e].next;
	}
	return sa.id;
}


void SplitEdge(WingMesh &m,int e,const float3 &vpos)
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
		m.vback[sa.v]=sa.id;
	}
}

void SplitEdges(WingMesh &wmesh,const float4 &split)
{
	std::vector<WingMesh::HalfEdge> &edges = wmesh.edges;
	for(unsigned int e=0;e<edges.size();e++)
	{
		int ea = edges[e].adj;
		if((PlaneTest(split,wmesh.verts[edges[e].v])|PlaneTest(split,wmesh.verts[edges[ea].v]))==SPLIT)
		{
			float3 vpos = PlaneLineIntersection(split,wmesh.verts[edges[e].v],wmesh.verts[edges[ea].v]);
			assert(PlaneTest(split,vpos)==COPLANAR);
			SplitEdge(wmesh,e,vpos);
		}
	}
}

int findcoplanaredge(WingMesh &m,int v,const float4 &slice)
{
	// assumes the mesh straddles the plane slice.
	// tesselates the mesh if required.
	int e=m.vback[v];
	int es=e;
	while(PlaneTest(slice,m.verts[m.edges[e].Adj()->v])!=UNDER)
	{
		e = m.edges[e].Prev()->Adj()->id;
		assert(e!=es); // if(e==es) return -1; // all edges point over!
	}
	es=e;
	while(PlaneTest(slice,m.verts[m.edges[e].Adj()->v])==UNDER)
	{
		e = m.edges[e].Adj()->Next()->id;
		assert(e!=es); // if(e==es) return -1; // all edges point UNDER!
	}
	int ec=m.edges[e].next;
	while(PlaneTest(slice,m.verts[m.edges[ec].v])!=COPLANAR)
	{
		ec = m.edges[ec].next;
		assert(ec!=e);
	}
	if(ec==m.edges[e].next) {return e;}
	assert(ec!=m.edges[e].prev);
	return Tess(m,e,ec);

}

int findcoplanarvert(const WingMesh &m,const float4 &slice)
{
	for(unsigned int i=0;i<m.verts.size();i++)
	{
		if(PlaneTest(slice,m.verts[i])==COPLANAR)
		{
			return i;
		}
	}
	return -1;
}

void WingMeshTess(WingMesh &m,const float4 &slice,std::vector<int> &loop)
{
	SplitEdges(m,slice);
	loop.clear();
	int v0 = findcoplanarvert(m,slice);
	if(v0==-1) return; //??
	int v=v0;
	do{
		int e=findcoplanaredge(m,v,slice);
		v = m.edges[e].Adj()->v;
		loop.push_back(e);
	} while (v!=v0);

}

WingMesh WingMeshCrop(const WingMesh &_m,const float4 &slice)
{
	std::vector<int> coplanar; // edges
	int s = WingMeshSplitTest(_m,slice);
	if (s == OVER) return WingMesh(); //  NULL;
	if(s==UNDER) return _m;
	WingMesh m = _m;
	WingMeshTess(m,slice,coplanar);
	std::vector<int> reverse;
    for (unsigned int i = 0; i<coplanar.size(); i++) reverse.push_back(m.edges[coplanar[coplanar.size() - 1 - i]].adj);

    if (coplanar.size()) WingMeshSeparate(m, reverse);
	checkit(m);
	assert(dot(m.faces[0].xyz(), slice.xyz()) > 0.99f);
	m.faces[0] = slice;
	return m;
}

int WingMeshSplitTest(const WingMesh &m, const float4 &plane) {
	int flag=0;
	for (unsigned int i = 0; i<m.verts.size(); i++) {
		flag |= PlaneTest(plane,m.verts[i]);
	}
	return flag;
}
void WingMeshTranslate(WingMesh *m,const float3 &offset){
	for(unsigned int i=0;i<m->verts.size();i++) {
		m->verts[i] = m->verts[i]+offset;
	}
	for(unsigned int i=0;i<m->faces.size();i++) {
		m->faces[i].w = m->faces[i].w - dot(m->faces[i].xyz(),offset);
	}
}

void      WingMeshRotate(WingMesh *m,const float4 &rot)
{
	for(unsigned int i=0;i<m->verts.size();i++) {
		m->verts[i] = qrot(rot , m->verts[i]);
	}
	for(unsigned int i=0;i<m->faces.size();i++) {
		m->faces[i].xyz() = qrot(rot, m->faces[i].xyz());
	}
}


std::vector<int3> WingMeshTris(const WingMesh &m)
{
	std::vector<int3> tris;
    int predict = m.edges.size() - m.faces.size() * 2;
	if (predict>0) tris.resize(predict);
	tris.clear();
	for(unsigned int i=0;i<m.faces.size();i++)
	{
		int e0 = m.fback[i];
		if(e0==-1) continue;
		int ea = m.edges[e0].next;
		int eb = m.edges[ea].next;
		while(eb!=e0)
		{
			tris.push_back(int3(m.edges[e0].v,m.edges[ea].v,m.edges[eb].v));
			ea=eb;
			eb = m.edges[ea].next;
		}
	}
	return tris;
}


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

/*
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
	int rc=linkmesh(*wmesh); 	// connect em
	assert(rc);
	checkit(*wmesh);
	return wmesh;
}
*/


//--------- mass properties ----------

float WingMeshVolume(const WingMesh &mesh)
{
	auto tris= WingMeshTris(mesh);
	const float3 *verts = mesh.verts.data();
    return Volume(verts, tris.data(), tris.size());
}

float Volume(const std::vector<WingMesh*> &meshes)
{
	float  vol=0;
	for(auto &m : meshes)
		vol += WingMeshVolume(*m);
	return vol;
}

float3 CenterOfMass(const std::vector<WingMesh*> &meshes)
{
	float3 com(0,0,0);
	float  vol=0;
	for (auto &m : meshes)
	{
		auto tris = WingMeshTris(*m);
		const float3 *verts = m->verts.data();
        float3 cg = CenterOfMass(verts, tris.data(), tris.size());
        float   v = Volume(verts, tris.data(), tris.size());
		vol+=v;
		com+= cg*v;
	}
	com /= vol;
	return com;
}


float3x3 Inertia(const std::vector<WingMesh*> &meshes, const float3& com)
{
	float  vol=0;
	float3x3 inertia;
	for (auto &m : meshes)
	{
		auto tris = WingMeshTris(*m);
		const float3 *verts = m->verts.data();
        float v = Volume(verts, tris.data(), tris.size());
        inertia += Inertia(verts, tris.data(), tris.size(), com) * v;
		vol+=v;
	}
	inertia *= 1.0f/vol;
	return inertia;
}

//-----------------------


