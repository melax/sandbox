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
#include "../testphys/wingmesh.h"  // fixme
#include <functional>

//#define COPLANAR   (0)
//#define UNDER      (1)
//#define OVER       (2)
//#define SPLIT      (OVER|UNDER)
//#define PAPERWIDTH (0.0001f)


class Face : public float4 
{
  public:
	int				matid;
	std::vector<float3>	vertex;
	float3			gu;
	float3			gv;
	float3			ot;
	Face() {matid=0;}
};


class BSPNode :public float4  
{
  public:
	BSPNode *		under;
	BSPNode *		over;
	int				isleaf;
	int				flag;      // using this for GC
	WingMesh 		convex;    // the volume of space occupied by this node
	std::vector<Face *>	brep;
	explicit		BSPNode(const float4 &p);
	explicit		BSPNode(const float3 &n=float3(0,0,0),float d=0);
					~BSPNode();
};


inline std::pair<float3, float3> Extents(const Face &face){ return Extents(face.vertex); }
inline std::pair<float3, float3> Extents(const std::vector<Face*> &faces)
{
    auto bbox = Extents(std::vector<float3>());  // initializes to empty limits
    for (auto f : faces)
    {
        auto b = Extents(*f);
        bbox.first = cmin(b.first, bbox.first);
        bbox.second = cmax(b.second, bbox.second);
    }
    return bbox;
}

Face *   FaceDup(Face *face);
Face *   FaceClip(Face *face,const float4 &clip);
float    FaceArea(Face *face);
float3   FaceCenter(Face *face);
int	     FaceSplitTest(Face *face,float3 splitnormal,float splitdist,float epsilon=PAPERWIDTH);
void     FaceSliceEdge(Face *face,int edge,BSPNode *n);
void     FaceEmbed(BSPNode *node,Face *face);
void     FaceExtractMatVals(Face *face,const float3 &v0,const float3 &v1,const float3 &v2,const float2 &t0,const float2 &t1,const float2 &t2);
void     FaceTranslate(std::vector<Face *> &faces,const float3 &offset);
void     FaceTranslate(Face *face,const float3 &offset);
void     FaceRotate(Face *face,const float4 &r);
int      FaceClosestEdge(Face *face,const float3 &sample_point);
Face *   FaceNewQuad(const float3 &v0,const float3 &v1,const float3 &v2,const float3 &v3);
Face *   FaceNewTri(const float3 &v0,const float3 &v1,const float3 &v2);
Face *   FaceNewTriTex(const float3 &v0,const float3 &v1,const float3 &v2,const float2 &t0,const float2 &t1,const float2 &t2);
float2   FaceTexCoord(Face *f,int i);  // uv texture coords of i'th vertex
float2   FaceTexCoord(Face *f,const float3 &v); // uv texture coord of point v on face
int      FaceSplitifyEdges(BSPNode *root);

void     AssignTex(BSPNode *node,int matid=0);
void     AssignTex(Face* face);



BSPNode *BSPCompile(std::vector<Face *> &inputfaces,WingMesh convex_space,int side=0); 
void     BSPMakeBrep(BSPNode *r,std::vector<Face*> &faces);
void     BSPDeriveConvex(BSPNode *node,WingMesh *convex);
BSPNode* BSPDup(BSPNode *n);
void     BSPTranslate(BSPNode *n,const float3 &offset);
void     BSPRotate(BSPNode *n, const float4 &r);
void     BSPScale(BSPNode *n,float s);
void     BSPFreeBrep(BSPNode *);
void     BSPRipBrep(BSPNode *r,std::vector<Face*> &faces);
void     BSPGetBrep(BSPNode *r,std::vector<Face*> &faces);
inline   std::vector<Face*> BSPGetBrep(BSPNode *n) {std::vector<Face*> faces;BSPGetBrep(n,faces);return faces;}
void     BSPMakeBrep(BSPNode *);
BSPNode *BSPUnion(BSPNode *a,BSPNode *b);
BSPNode *BSPIntersect(BSPNode *a,BSPNode *b);
void     NegateTree(BSPNode *n);
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
BSPNode *BSPClean(BSPNode *n); 
int      BSPCount(BSPNode *n);
int      BSPFinite(BSPNode *bsp);
inline int maxdir(const std::vector<float3> &a,const float3 &dir) {return maxdir(a.data(),a.size(),dir);}

extern int bspnodecount;  // just a running count of all nodes created, for monitoring purposes


#endif
