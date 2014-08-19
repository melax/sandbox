
//
// hull and convex utility routines 
//  (c) Stan Melax 2004
// 
// 2014 update: got rid of all the little new/delete calls.  replaced array of Tri* with array of Tri objects.
// Note: some style updates reauire c++11 now
// 
// Note:  The code here actually predates any code used by novodex as well as 
// the 'stanhull'  version released on the web at
// codesuppoitory.blogspot.com, see that version for something slightly more robust.
// btw someone else picked the name 'stanhull' - no egos here.  :)
// Although, I was pleasantly surprised to see many people that found it useful.  
// Thanks for all the positive feedback.
// Erwin may now (2011) have more recent hull code on the Bullet site that seamlessly handles 
// both 2D and 3D cases.  
// 
// My main objective when writing this code was to just do 3D convex hulls, but only 
// up to a certain number of vertices.  
// Starting from a tetrahedron, the algorithm iteratively selects a vertex and 
// expands the convex hull.
// I use a greedy approach to pick the volume maximizing vertex at each step.
// This is potentially O(n*n) or O(n*c) for final hull with c verts.
// quickhull is about doing the complete convex hull O(n lg(n)).  
// For offline processing the runtime wasn't an issue.  
// Furthermore, prefer a significantly reduced hull especially since a convex hull is already
// an approximation of the actual geometry anyways.  16 or 32 verts is usually more than enough.
//
// 
// 

#include <utility>      // for std::swap()
#include <algorithm>
#include <assert.h>

#include "vecmatquat_minimal.h"  // hull.h header currently expects int3 and float3 and a few related functions to be implemented in the obvious way





inline int3 roll3(const int3 &a) { return{ a[1], a[2], a[0] }; }
inline bool isa(const int3 &a, const int3 &b) { return (a == b || roll3(a) == b || a == roll3(b)); }
inline bool b2b(const int3 &a, const int3 &b) { return isa(a, { b[2], b[1], b[0] }); }

inline bool above(float3* vertices, const int3& t, const float3 &p, float epsilon)
{
	float3 n=TriNormal(vertices[t[0]],vertices[t[1]],vertices[t[2]]);
	return (dot(n,p-vertices[t[0]]) > epsilon); // EPSILON???
}
inline bool hasedge(const int3 &t, int a, int b)
{
	for(int i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(t[i]==a && t[i1]==b) return 1;
	}
	return 0;
}
inline bool hasvert(const int3 &t, int v)
{
	return (t[0]==v || t[1]==v || t[2]==v) ;
}
inline bool shareedge(const int3 &a, const int3 &b)
{
	int i;
	for(i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(hasedge(a,b[i1],b[i])) return 1;
	}
	return 0;
}



class Tri 
{
public:
	int3 v;   // indices into a vertex array
	int3 n;
	int id;   // should only need this for assert statements
	int vmax;
	float rise;
	Tri(int a, int b, int c, int id, int3 n = { -1, -1, -1 }) :v({ a, b, c }), id(id), n(n)
	{
		vmax=-1;
		rise = 0.0f;
	}
	~Tri()
	{
	}
	bool dead(){ return n[0] == -1; }

	int &neib(int va, int vb)
	{
		int i;
		for(i=0;i<3;i++) 
		{
			int i1=(i+1)%3;
			int i2=(i+2)%3;
			if (v[i] == va && v[i1] == vb) return n[i2];
			if (v[i] == vb && v[i1] == va) return n[i2];  // in case caller flipped edge order,
		}
		assert(0);
		throw("badness");
	}
};



inline void nnfix(std::vector<Tri> &tris,  int k)  // make valid neighbors of k point back to k;
{
	if (tris[k].id == -1)
		return;
	assert(tris[k].id == k);
	for (int i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		if (tris[k].n[i] != -1)
		{
			int &nn = tris[tris[k].n[i]].neib(tris[k].v[i2], tris[k].v[i1]);
			nn = k;
		}
	}
}
inline void swapn(std::vector<Tri> &tris, int a, int b)
{
	std::swap(tris[a], tris[b]);
	std::swap(tris[a].id, tris[b].id);
	nnfix(tris,a);
	nnfix(tris,b);
}

inline void detach(std::vector<Tri> &tris,int k)
{
	assert(tris[k].id == k);
	tris[k].n = { -1, -1, -1 };
	if (0) for (int i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		if (tris[k].n[i] != -1)
		{
			try {
				int &nn = tris[tris[k].n[i]].neib(tris[k].v[i2], tris[k].v[i1]);
				if (nn == k)
					nn = -1;
			}
			catch (...){}
		}
	}
	swapn(tris, k, tris.size()-1);
	tris.pop_back();
}


inline void b2bfix(std::vector<Tri> &tris, int s, int t)
{
	// b2bfix 
	for(int i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int va = tris[s].v[i1];
		int vb = tris[s].v[i2];
		assert(tris[tris[s].neib(va, vb)].neib(vb, va) == tris[s].id);
		assert(tris[tris[t].neib(va, vb)].neib(vb, va) == tris[t].id);
		tris[tris[s].neib(va, vb)].neib(vb, va) = tris[t].neib(vb, va);
		tris[tris[t].neib(vb, va)].neib(va, vb) = tris[s].neib(va, vb);
	}
	tris[s].n = tris[t].n = { -1, -1, -1 };  // these will get cleaned up later
}



inline void checkit( std::vector<Tri> &tris, const Tri &t)
{
	assert(tris[t.id].id==t.id);
	assert(&t == &tris[t.id]);
	for (int i = 0; i<3; i++)
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = t.v[i1];
		int b = t.v[i2];
		assert(a!=b);
		assert( tris[t.n[i]].neib(b,a) == t.id);
	}
}

inline void extrude(std::vector<Tri> &tris, int t0, int v)
{
	int3 t= tris[t0].v;
	int b = tris.size();
	int3 n = tris[t0].n;

	tris.push_back(Tri(v,t[1],t[2],b+0, {n[0],b+1,b+2}));
	tris[n[0]].neib(t[1],t[2]) = b+0;
	tris.push_back(Tri(v,t[2],t[0],b+1, {n[1],b+2,b+0}));
	tris[n[1]].neib(t[2],t[0]) = b+1;
	tris.push_back(Tri(v,t[0],t[1],b+2, {n[2],b+0,b+1}));
	tris[n[2]].neib(t[0],t[1]) = b+2;
	tris[t0].n = { -1, -1, -1 };
	checkit(tris, tris[b + 0]);
	checkit(tris, tris[b + 1]);
	checkit(tris, tris[b + 2]);
	if(hasvert(tris[n[0]].v,v)) { b2bfix(tris,b+0,n[0 ]); }
	if(hasvert(tris[n[1]].v,v)) { b2bfix(tris,b+1,n[1 ]); }
	if(hasvert(tris[n[2]].v,v)) { b2bfix(tris,b+2,n[2 ]); }
}

inline int extrudable(std::vector<Tri> &tris,   float epsilon)
{
	int t=-1;
	for(unsigned int i=0;i<tris.size();i++)
	{
		assert(tris[i].id >= 0);
		assert(tris[i].id ==i);
		assert(!tris[i].dead()); // array should be clear of junk when this is called
		if(t<0 || (tris[t].rise<tris[i].rise))
		{
			t = i;
		}
	}
	return (tris[t].rise >epsilon)?t:-1 ;
}



inline int4 FindSimplex(float3 *verts,int verts_count)
{
	float3 basis[3];
	basis[0] = float3( 0.01f, 0.02f, 1.0f );      
	int p0 = maxdir(verts,verts_count, basis[0]);   
	int	p1 = maxdir(verts,verts_count,-basis[0]);
	basis[0] = verts[p0]-verts[p1];
	if(p0==p1 || basis[0]==float3(0,0,0)) 
		return{ -1, -1, -1, -1 };
	basis[1] = cross(float3(1,0,0),basis[0]);
	basis[2] = cross(float3(0,1,0),basis[0]);
	basis[1] = normalize( (magnitude(basis[1])>magnitude(basis[2])) ? basis[1]:basis[2]);
	int p2 = maxdir(verts,verts_count,basis[1]);
	if(p2 == p0 || p2 == p1)
	{
		p2 = maxdir(verts,verts_count,-basis[1]);
	}
	if(p2 == p0 || p2 == p1) 
		return{ -1, -1, -1, -1 };
	basis[1] = verts[p2] - verts[p0];
	basis[2] = cross(basis[1],basis[0]);
	int p3 = maxdir(verts,verts_count,basis[2]);
	if(p3==p0||p3==p1||p3==p2) p3 = maxdir(verts,verts_count,-basis[2]);
	if(p3==p0||p3==p1||p3==p2) 
		return{ -1, -1, -1, -1 };
	assert(!(p0==p1||p0==p2||p0==p3||p1==p2||p1==p3||p2==p3));
	if(dot(verts[p3]-verts[p0],cross(verts[p1]-verts[p0],verts[p2]-verts[p0])) <0) {std::swap(p2,p3);}
	return {p0,p1,p2,p3};
}


inline std::vector<int3> calchull(float3 *verts,int verts_count, int vlimit) 
{
	if(verts_count <4) return std::vector<int3>();
	if(vlimit==0) vlimit=1000000000;
	float3 bmin(*verts),bmax(*verts);
	std::vector<int> isextreme;
    isextreme.reserve(verts_count);
	for(int j=0;j<verts_count;j++) 
	{
		isextreme.push_back(0);
		bmin = cmin(bmin,verts[j]);
		bmax = cmax(bmax,verts[j]);
	}
	float epsilon = magnitude(bmax-bmin) * 0.001f;


	int4 p = FindSimplex(verts,verts_count);
	if(p.x==-1) return std::vector<int3>(); // simplex failed

	std::vector<Tri> tris;
	assert(tris.size() == 0);
	float3 center = (verts[p[0]]+verts[p[1]]+verts[p[2]]+verts[p[3]]) /4.0f;  // a valid interior point
	tris.push_back(Tri(p[2],p[3],p[1],tris.size(),{2,3,1})); 
	tris.push_back(Tri(p[3],p[2],p[0],tris.size(),{3,2,0})); 
	tris.push_back(Tri(p[0],p[1],p[3],tris.size(),{0,1,3})); 
	tris.push_back(Tri(p[1],p[0],p[2],tris.size(),{1,0,2})); 
	isextreme[p[0]]=isextreme[p[1]]=isextreme[p[2]]=isextreme[p[3]]=1;
	checkit(tris,tris[0]);checkit(tris,tris[1]);checkit(tris,tris[2]);checkit(tris,tris[3]);

	for(auto &t : tris ) 
	{
		assert(t.id>=0);
		assert(t.vmax<0);
		float3 n=TriNormal(verts[(t.v)[0]],verts[(t.v)[1]],verts[(t.v)[2]]);
		t.vmax = maxdir(verts,verts_count,n);
		t.rise = dot(n,verts[t.vmax]-verts[(t.v)[0]]);
	}
	int te;
	vlimit-=4;
	while(vlimit >0 && (te=extrudable(tris,epsilon))>=0)
	{
		int3 ti=tris[te].v;
		int v = tris[te].vmax;
		assert(!isextreme[v]);  // wtf we've already done this vertex
		isextreme[v]=1;
		//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
        unsigned int j = tris.size();
		int newstart=j;
		while(j--) {
			if(tris[j].dead()) 
				continue;
			int3 t=tris[j].v;
			if(above(verts,t,verts[v],0.01f*epsilon)) 
			{
				extrude(tris,j,v);
			}
		}
		// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
		j=tris.size();
		while(j--)
		{
			if(tris[j].dead()) continue;
			if(!hasvert(tris[j].v,v)) break;
			int3 nt=tris[j].v;
			if(above(verts,nt,center,0.01f*epsilon)  || magnitude(cross(verts[nt[1]]-verts[nt[0]],verts[nt[2]]-verts[nt[1]]))< epsilon*epsilon*0.1f )
			{
				int nb = tris[j].n[0];
				assert(nb>=0);assert(!tris[nb].dead());assert(!hasvert(tris[nb].v,v));assert(tris[nb].id<(int)j);
				extrude(tris,nb,v);
				j=tris.size(); 
			}
		} 
		j=tris.size();
		while(j--)
		{
			Tri &t=tris[j];
			if(tris[j].dead()) continue;
			if(t.vmax>=0) break;
			float3 n=TriNormal(verts[(t.v)[0]],verts[(t.v)[1]],verts[(t.v)[2]]);
			t.vmax = maxdir(verts,verts_count,n);
			if(isextreme[t.vmax]) 
			{
				t.vmax=-1; // already done that vertex - algorithm needs to be able to terminate.
			}
			else
			{
				t.rise = dot(n,verts[t.vmax]-verts[(t.v)[0]]);
			}
		}
		j = tris.size();
		while (j--)  // compress, remove non-tris
		{
			if (!tris[j].dead()) continue;
			swapn(tris, j, tris.size() - 1);  // this routine fixes neighbor's neighbor indices
			tris.pop_back();
		}
		vlimit--;
	}
	std::vector<int3> ts;
	for(unsigned int i=0;i<tris.size();i++) 
	{
		assert(!tris[i].dead());
		ts.push_back(tris[i].v);
		//delete tris[i];
	}
    tris.clear();
    std::vector<int> used;
	std::vector<int> map;
	int n=0;
	for(int i=0;i<verts_count;i++){ used.push_back(0);map.push_back(0);}
	for(unsigned int i=0;i<ts.size();i++  )for(unsigned int j=0;j<3;j++){used[(ts[i])[j]]++;}
	for(unsigned int i=0;i<used.size();i++){if(used[i]) std::swap(verts[map[i]=n++],verts[i]);else map[i]=-1;}
	for(unsigned int i=0;i<ts.size();i++  )for(unsigned int j=0;j<3;j++){(ts[i])[j] = map[(ts[i])[j]];}
	return ts;
}

