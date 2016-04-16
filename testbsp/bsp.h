//
//      BSP
//  (c) Stan Melax 1998-2007  bsd licence
//  
// BSPs? really? are they still useful?  
// Clearly, Spatial structures are still necessary to implement anything useful in a 3D 
// interactive application.  
// Admittedly overlapping hierarchies and polygon soup approaches are easier to implement.
// An advantage with BSP is the ability to do interesting things  
// from just knowing if a point xyz is in empty space or inside a solid volume all the way to 
// operations such as computational solid geometry (intersections and unions).
// 
// 

#ifndef SMBSP_H
#define SMBSP_H


#include "geometric.h"
#include "wingmesh.h"  
#include <functional>
#include <memory>

//#define COPLANAR   (0)   <= these found in geometric.h
//#define UNDER      (1)
//#define OVER       (2)
//#define SPLIT      (OVER|UNDER)
//#define PAPERWIDTH (0.0001f)


struct Face : public float4 
{
	float4&         plane() {return *this;}  
	const float4&   plane() const { return *this; }  // hmmmm is-a vs has-a
	int				matid;
	std::vector<float3>	vertex;
	float3			gu;
	float3			gv;
	float3			ot;

    Face() : matid(0) {}
    Face(const Face & r) = default;
    Face(Face && r) : Face() { *this = std::move(r); }
    Face & operator = (const Face & r) = default;
    Face & operator = (Face && r) { plane()=r.plane(); matid=r.matid; vertex=move(r.vertex); gu=r.gu; gv=r.gv; ot=r.ot; return *this; }
};


class BSPNode :public float4  
{
  public:
	float4&         plane() {return *this;}  // hmmmm is-a vs has-a
	const float4&   plane() const { return *this; }  
	std::unique_ptr<BSPNode> under;
	std::unique_ptr<BSPNode> over;
	int				isleaf;
	WingMesh 		convex;    // the volume of space occupied by this node
	std::vector<Face> brep;
	explicit		BSPNode(const float4 &p);
	explicit		BSPNode(const float3 &n=float3(0,0,0),float d=0);
					~BSPNode();
};

struct treetraverse  // preorder
{
	BSPNode *root;
	treetraverse(BSPNode *root) :root(root){}
	struct iterator
	{
		std::vector<BSPNode *> stack;
		BSPNode * operator *() const { return stack.size() ? stack.back() : NULL; }
		iterator & operator++(){ assert(stack.size()); BSPNode *n = **this; stack.pop_back(); if (n && n->under) stack.push_back(n->under.get()); if (n&&n->over) stack.push_back(n->over.get());  return *this; }
		bool operator !=(const iterator &b){ return stack != b.stack; }
	};
	iterator begin() { iterator b; if (root)b.stack.push_back(root); return b; }
	iterator end()   { iterator e; return e; }
};

struct treebacktofront
{
	BSPNode *root;
	float3 p;
	treebacktofront(BSPNode *root, const float3 &p) :root(root), p(p) {}
	struct iterator
	{
		std::vector<BSPNode *> stack;
		float3 p;
		void toleaf(BSPNode *n) { while (n) { stack.push_back(n); n = (dot(float4(p, 1), n->plane()) > 0) ? n->under.get() : n->over.get(); } }
		BSPNode * operator *() const { return stack.size() ? stack.back() : NULL; }
		iterator & operator++(){ assert(stack.size()); BSPNode *n = **this; stack.pop_back(); assert(n); toleaf((dot(float4(p, 1), n->plane()) > 0) ? n->over.get() : n->under.get());  return *this; }
		bool operator !=(const iterator &b){ return stack != b.stack; }
	};
	iterator begin() { iterator b; b.p = p;  b.toleaf(root); return b; }
	iterator end()   { iterator e; e.p = p;  return e; }
};

inline std::pair<float3, float3> Extents(const Face &face){ return Extents(face.vertex); }
inline std::pair<float3, float3> Extents(const std::vector<Face*> &faces)
{
    auto bbox = Extents(std::vector<float3>());  // initializes to empty limits
    for (auto f : faces)
    {
        auto b = Extents(*f);
        bbox.first  = min(b.first, bbox.first);
        bbox.second = max(b.second, bbox.second);
    }
    return bbox;
}

Face     FaceClip(const Face & face, const float4 & clip);
Face     FaceClip(Face && face, const float4 & clip);
float    FaceArea(const Face & face);
float3   FaceCenter(const Face & face);
int	     FaceSplitTest(const Face & face,const float4 &splitplane,float epsilon=PAPERWIDTH);
void     FaceSliceEdge(Face *face,int edge,BSPNode *n);
void     FaceEmbed(BSPNode *node, Face && face);
void     FaceExtractMatVals(Face *face,const float3 &v0,const float3 &v1,const float3 &v2,const float2 &t0,const float2 &t1,const float2 &t2);
void     FaceTranslate(std::vector<Face> & faces, const float3 & offset);
void     FaceTranslate(Face & face ,const float3 & offset);
void     FaceRotate(Face & face, const float4 & r);
void     FaceScale(Face & face, const float3 & scaling);
void     FaceScale(Face & face, float scaling);
int      FaceClosestEdge(Face *face,const float3 &sample_point);
Face     FaceNewQuad(const float3 &v0,const float3 &v1,const float3 &v2,const float3 &v3);
Face     FaceNewTri(const float3 &v0,const float3 &v1,const float3 &v2);
Face     FaceNewTriTex(const float3 &v0,const float3 &v1,const float3 &v2,const float2 &t0,const float2 &t1,const float2 &t2);
float2   FaceTexCoord(Face *f,int i);  // uv texture coords of i'th vertex
float2   FaceTexCoord(Face *f,const float3 &v); // uv texture coord of point v on face
int      FaceSplitifyEdges(BSPNode *root);

void     AssignTex(BSPNode *node,int matid=0);
void     AssignTex(Face & face);


std::unique_ptr<BSPNode> BSPCompile(const std::vector<Face> & inputfaces,WingMesh convex_space,int side=0); 
std::unique_ptr<BSPNode> BSPCompile(std::vector<Face> && inputfaces,WingMesh convex_space,int side=0); 
std::unique_ptr<BSPNode> BSPUnion(std::unique_ptr<BSPNode> a, std::unique_ptr<BSPNode> b);
std::unique_ptr<BSPNode> BSPIntersect(std::unique_ptr<BSPNode> a, std::unique_ptr<BSPNode> b);
std::unique_ptr<BSPNode> BSPClean(std::unique_ptr<BSPNode> n); 

std::unique_ptr<BSPNode> BSPDup(BSPNode *n);

void     BSPDeriveConvex(BSPNode &node, WingMesh convex);
void     BSPMakeBrep(BSPNode *r, std::vector<Face> && faces);  // only uses faces to sample for texture and material
std::vector<Face> BSPRipBrep(BSPNode *r);
void     BSPTranslate(BSPNode & n,const float3 & translation);
void     BSPRotate(BSPNode & n, const float4 & rotation);
void     BSPScale(BSPNode & n, const float3 & scaling);
void     BSPScale(BSPNode & n, float scaling);
BSPNode& NegateTree(BSPNode & n);
inline   std::unique_ptr<BSPNode> NegateTree(std::unique_ptr<BSPNode> n) { NegateTree(*n.get()); return n; }
inline   std::unique_ptr<BSPNode> BSPTranslate(std::unique_ptr<BSPNode> n,const float3 &translation) { BSPTranslate(*n.get(), translation); return n; }

int      HitCheck(BSPNode *node,int solid,float3 v0,float3 v1,float3 *impact);
int      HitCheckSolidReEnter(BSPNode *node,float3 v0,float3 v1,float3 *impact); // wont just return v0 if you happen to start in solid
int      HitCheckCylinder(float r,float h,BSPNode *node,int solid,float3 v0,float3 v1,float3 *impact,const float3 &nv0); 
int      HitCheckConvex(std::vector<float3> &verts,BSPNode *node,int solid,
				   float3 v0,float3 v1,float3 *impact,
				   const float4 &q0, const float4 &q1, float4 *impactq, const float3 &nv0, const int vrtv0);

int HitCheckConvexGJKm(std::function<float3(const float3&)> support_map_function, BSPNode *bsp);
template<class T> int HitCheckConvexGJK(T collidable, BSPNode *bsp) {return HitCheckConvexGJKm(SupportPointFunc<T>(collidable), bsp); }
int      HitCheckSphere(float r, BSPNode *node, int solid, float3 v0, float3 v1, float3 *impact, const float3 &nv0);
int      ConvexHitCheck(WingMesh *convex,float3 v0,float3 v1,float3 *impact); 
std::vector<WingMesh*> ProximityCellsm(std::function<float3(const float3&)> support_map_function, BSPNode *bsp, float padding = 0.0f);
template<class T> std::vector<WingMesh*> ProximityCells(T collidable, BSPNode *bsp, float padding = 0.0f) { return ProximityCellsm(SupportPointFunc<T>(collidable), bsp, padding); }
std::vector<WingMesh*> BSPGetSolids(BSPNode *bsp);
void     BSPPartition(BSPNode *n, const float4 &p, BSPNode * &nodeunder, BSPNode * &nodeover);
int      BSPCount(BSPNode *n);
int      BSPFinite(BSPNode *bsp);
inline int maxdir(const std::vector<float3> &a,const float3 &dir) {return maxdir(a.data(),a.size(),dir);}

extern int bspnodecount;  // just a running count of all nodes created, for monitoring purposes


#endif
