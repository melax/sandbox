
//
//              BSP 
// 
// (c) Stan Melax 1998-2007  bsd licence
//
// bsp compiler
//


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>


#define FUZZYWIDTH (PAPERWIDTH*100)

#include "bsp.h"

int facetestlimit=50;
int allowaxial=1;
int solidbias =0;
int usevolcalc=1;
int allowhull =0;
BSPNode *currentbsp=NULL;



int bspnodecount=0;
BSPNode::BSPNode(const float3 &n,float d):float4(n,d){
	isleaf= 0;
	bspnodecount++;
}
BSPNode::BSPNode(const float4 &p) : float4(p)
{
	isleaf=0;
	bspnodecount++;
}

BSPNode::~BSPNode() {
	bspnodecount--;
}

int BSPCount(BSPNode *n)
{
	if(!n) return 0;
	return 1+BSPCount(n->under.get())+BSPCount(n->over.get());
}

struct SortPerm { unsigned int i; float v; };
static int FaceAreaCompare(const void *_a,const void *_b) {
	SortPerm *a = ((SortPerm *) _a); 
	SortPerm *b = ((SortPerm *) _b); 
	return ((a->v < b->v)?-1:1); 
}
void ReorderFaceArray(std::vector<Face> & face){
	std::vector<SortPerm> sp;
	for(unsigned int i=0;i<face.size();i++) {
        sp.push_back(SortPerm{i, FaceArea(face[i])});
	}
    assert(sp.size() == face.size());
    qsort(sp.data(), (unsigned long)sp.size(), sizeof(SortPerm), FaceAreaCompare);

    std::vector<Face> newFaces;
    newFaces.reserve(face.size());
	for(unsigned int i=0;i<face.size();i++) {
        newFaces.push_back(std::move(face[sp[i].i]));
		if(i) {
			assert(FaceArea(newFaces[i]) >= FaceArea(newFaces[i-1]));
		}
	}
    face.swap(newFaces);
}

static int count[4];


float sumbboxdim(const WingMesh &convex)
{
	float3 bmin,bmax;
	std::tie(bmin, bmax) = Extents<float,3>(convex.verts); // (convex->verts.data(), convex->verts.size(), bmin, bmax);
	return dot(float3(1,1,1),bmax-bmin);
}

float PlaneCost(const std::vector<Face> &inputfaces,const float4 &split,const WingMesh &space,int onbrep)
{
	count[COPLANAR] = 0;
	count[UNDER]    = 0;
	count[OVER]     = 0;
	count[SPLIT]    = 0;
	for(unsigned int i=0;i<inputfaces.size();i++) {
		count[FaceSplitTest(inputfaces[i],split,FUZZYWIDTH)]++;
	}
    if (space.verts.size() == 0) {
		// The following formula isn't that great.
		// Better to use volume as well eh.
		return (float)(abs(count[OVER]-count[UNDER]) + count[SPLIT] - count[COPLANAR]);
	}
	float volumeover =(float)1.0;
	float volumeunder=(float)1.0;
	float volumetotal=WingMeshVolume(space);
	WingMesh spaceunder= WingMeshCrop(space,float4( split.xyz(), split.w));
	WingMesh spaceover = WingMeshCrop(space,float4(-split.xyz(),-split.w));
	if(usevolcalc==1)
	{
		volumeunder = WingMeshVolume(spaceunder);
		volumeover  = WingMeshVolume(spaceover );
	}
	else if (usevolcalc==2)
	{
		volumeunder = sumbboxdim(spaceunder);
		volumeover  = sumbboxdim(spaceover );
	}
	assert(volumeover/volumetotal>=-0.01);
	assert(volumeunder/volumetotal>=-0.01);
	if(fabs((volumeover+volumeunder-volumetotal)/volumetotal)>0.01)	{
		// ok our volume equations are starting to break down here
		// lets hope that we dont have too many polys to deal with at this point.
		volumetotal=volumeover+volumeunder;
	}
	if(solidbias && onbrep && count[OVER]==0 && count[SPLIT]==0)
	{
		return volumeunder;
	}
	return volumeover *powf(count[OVER] +1.5f*count[SPLIT],0.9f) + 
	       volumeunder*powf(count[UNDER]+1.5f*count[SPLIT],0.9f);
}



void DividePolys(const float4 &splitplane,std::vector<Face> && inputfaces,
				 std::vector<Face> &under,std::vector<Face> &over,std::vector<Face> &coplanar){
	int i=inputfaces.size();
	while(i--) {
		int flag = FaceSplitTest(inputfaces[i],splitplane,FUZZYWIDTH);

		if(flag == OVER) {
			over.push_back(std::move(inputfaces[i]));
		}
		else if(flag == UNDER) {
			under.push_back(std::move(inputfaces[i]));
		}
		else if(flag == COPLANAR) {
			coplanar.push_back(std::move(inputfaces[i]));
		}
		else {
			assert(flag == SPLIT);
			over.push_back(FaceClip(inputfaces[i], -splitplane));
			under.push_back(FaceClip(std::move(inputfaces[i]), splitplane));
		}
	}
}


std::unique_ptr<BSPNode> BSPCompile(const std::vector<Face> & inputfaces,WingMesh convex_space,int side) { return BSPCompile(std::vector<Face>(inputfaces), convex_space, side); }
std::unique_ptr<BSPNode> BSPCompile(std::vector<Face> && inputfaces,WingMesh space,int side) 
{
    if (inputfaces.size() == 0) {
        std::unique_ptr<BSPNode> node(new BSPNode);
		node->convex = space;
		node->isleaf=side; 
		return node;
	}
	std::vector<Face> over;
	std::vector<Face> under;
	std::vector<Face> coplanar;
	ReorderFaceArray(inputfaces);
	// select partitioning plane
	float minval=FLT_MAX;
	float4 split(float3(0,0,0),0);

//    if (inputfaces.size()>1 && inputfaces.size() <= allowhull)
//	{
//		int j;
//		std::vector<float3> verts;
//		for(i=0;i<inputfaces.size();i++) 
//			for(j=0;j<inputfaces[i]->vertex.size();j++)
//                AddUnique(verts, inputfaces[i]->vertex[j]);
//
//
//        std::vector<int3> tris=calchull(verts.data(), verts.size(), 50);  // passing in array members might not be the best thing here!!
//
//		for(i=0;i<tris.size();i++)
//		{
//			float4 p(TriNormal(verts[tris[i][0]],verts[tris[i][1]],verts[tris[i][2]]),0);
//			if(p.xyz()==float3(0,0,0))continue;
//			p.w = -dot(verts[tris[i][0]],p.xyz());
//			float3 c = (verts[tris[i][0]]+verts[tris[i][1]]+verts[tris[i][2]]) /3.0f + p.xyz() * 0.001f;
//			if(solidbias && !currentbsp) continue; 
//			if(solidbias && HitCheck(currentbsp,1,c,c,&c)) continue;
//			if(WingMeshSplitTest(space,p)!= SPLIT) continue;
//			float val = PlaneCost(inputfaces,p,space,1)*1.01f;
//			if(val<minval) 
//			{
//				minval=val;
//				split = p;
//			}
//		}
//	}

	if(!solidbias || split.xyz()==float3(0,0,0))
	{
        for (unsigned int i = 0; i<inputfaces.size() && (int)i<facetestlimit; i++) {
			float val=PlaneCost(inputfaces, inputfaces[i].plane(), space, 1);
			if(val<minval) {
				minval=val;
				split = inputfaces[i].plane();
			}
		}
		assert(split.xyz() != float3(0,0,0));
		PlaneCost(inputfaces,split,space,1);
        if (allowaxial && inputfaces.size() > 8) {
			// consider some other planes:
            for (unsigned int i = 0; i<inputfaces.size() && (int)i<facetestlimit; i++) {
				for(unsigned int j=0;j<inputfaces[i].vertex.size();j++ ) {
					float val;
					if(allowaxial & (1<<0))
					{
						val = PlaneCost(inputfaces,float4(float3(1,0,0),-inputfaces[i].vertex[j].x),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(1, 0, 0);
							split.w   = -inputfaces[i].vertex[j].x;
						}
					}
					if(allowaxial & (1<<1))
					{
						val = PlaneCost(inputfaces,float4(float3(0,1,0),-inputfaces[i].vertex[j].y),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(0, 1, 0);
							split.w   = -inputfaces[i].vertex[j].y;
						}
					}
					if(allowaxial & (1<<2))
					{
						val = PlaneCost(inputfaces,float4(float3(0,0,1),-inputfaces[i].vertex[j].z),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(0, 0, 1);
							split.w   = -inputfaces[i].vertex[j].z;
						}
					}
				}
			}
		}
	}
	// Divide the faces
    std::unique_ptr<BSPNode> node(new BSPNode);
    node->plane() = split;
	node->convex = space;

	DividePolys(float4(split.xyz(), split.w), std::move(inputfaces), under, over, coplanar);

	for(unsigned int i=0;i<over.size();i++) {
		for(unsigned int j=0;j<over[i].vertex.size();j++) {
			assert(dot(node->xyz(),over[i].vertex[j])+node->w >= -FUZZYWIDTH);
		}
	}
	for(unsigned int i=0;i<under.size();i++) {
		for(unsigned int j=0;j<under[i].vertex.size();j++) {
			assert(dot(node->xyz(),under[i].vertex[j])+node->w <= FUZZYWIDTH);
		}
	}

	node->under = BSPCompile(std::move(under), WingMeshCrop(space, split), UNDER);
	node->over = BSPCompile(std::move(over), WingMeshCrop(space, -split), OVER);
	return node;
}



void BSPDeriveConvex(BSPNode & node, WingMesh cnvx) 
{
	if (cnvx.edges.size() && cnvx.verts.size())
	{
		assert(cnvx.verts.size());
		assert(cnvx.edges.size());
		assert(cnvx.faces.size());
	}
	node.convex = std::move(cnvx);
	if(node.isleaf) return;

	// if we are "editing" a bsp then the current plane may become coplanar to one of its parents (boundary of convex) or outside of the current volume (outside the convex)
	WingMesh cu;
	WingMesh co;
	if(node.convex.verts.size())  // non empty
	{
		int f = node.convex.SplitTest(node.plane());
		if(f==SPLIT)
		{
			cu = WingMeshCrop(node.convex, node.plane());
			co = WingMeshCrop(node.convex, -node.plane());
		}
		else if(f==OVER)
		{
			co = node.convex;
		}
		else if(f==UNDER)
		{
			cu = node.convex;
		}
		else
		{
			assert(0); // hunh? the 3d convex has 0 volume
		}
	}

	// Under SubTree
	assert(node.under);
	BSPDeriveConvex(*node.under,cu); 
	// Over  SubTree
	assert(node.over);
	BSPDeriveConvex(*node.over ,co);
}

 
std::vector<WingMesh*> BSPGetSolids(BSPNode *root)
{
	std::vector<WingMesh*> meshes;
	for (auto n : treetraverse(root))
		if(n->isleaf == UNDER)
			meshes.push_back(&n->convex);
	return meshes;
}

void BSPTranslate(BSPNode & n, const float3 & translation)
{
    PlaneTranslate(n.plane(), translation);
	WingMeshTranslate(n.convex, translation);
	for(auto & face : n.brep) FaceTranslate(face, translation);
	if(n.under) BSPTranslate(*n.under, translation);
    if(n.over) BSPTranslate(*n.over, translation);
}

void BSPRotate(BSPNode & n, const float4 & rotation)
{
    PlaneRotate(n.plane(), rotation);
	WingMeshRotate(n.convex, rotation);
	for(auto & face : n.brep) FaceRotate(face, rotation);
	if(n.under) BSPRotate(*n.under, rotation);
	if(n.over) BSPRotate(*n.over, rotation);
}

void BSPScale(BSPNode & n, const float3 & scaling)
{
    PlaneScale(n.plane(), scaling);
    WingMeshScale(n.convex, scaling);
    for(auto & face : n.brep) FaceScale(face, scaling);
	if(n.under) BSPScale(*n.under, scaling);
	if(n.over) BSPScale(*n.over, scaling);
}

void BSPScale(BSPNode & n, float scaling)
{
    PlaneScale(n.plane(), scaling);
    WingMeshScale(n.convex, scaling);
    for(auto & face : n.brep) FaceScale(face, scaling);
	if(n.under) BSPScale(*n.under, scaling);
	if(n.over) BSPScale(*n.over, scaling);
}

void NegateFace(Face & f)
{
    f.plane() = -f.plane();
    std::reverse(begin(f.vertex), end(f.vertex));
}

void NegateTreePlanes(BSPNode * root) 
{
	for (auto n : treetraverse(root))
	{
		if (!n)
			continue;  // shouldn't happen
		for (auto &f : n->brep)
			NegateFace(f);
		n->isleaf = (3 - n->isleaf) % 3; //	if(n->isleaf) n->isleaf = (n->isleaf==UNDER)?OVER:UNDER;
		n->xyz() = -n->xyz();
		n->w     = -n->w;
		std::swap(n->under, n->over);
	}
}

void NegateTree(BSPNode & root) 
{
	NegateTreePlanes(&root);  // this flips the faces too
	for (auto & f : BSPRipBrep(&root))
		FaceEmbed(&root, std::move(f)); 
}

int BSPFinite(BSPNode *bsp)
{
	return !HitCheck(bsp,1,float3(999999,9999999,999999),float3(999999,9999999,999999),NULL);
}

