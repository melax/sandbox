
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
	under = NULL;
	over  = NULL;
	isleaf= 0;
	convex= NULL;
	flag = 0;
	bspnodecount++;
}
BSPNode::BSPNode(const float4 &p) : float4(p)
{
	under=over=NULL;
	isleaf=0;
	flag=0;
	convex=NULL;
	bspnodecount--;
}

BSPNode::~BSPNode() {
	delete under;
	delete over;
	delete convex;
	while(brep.size()) {
		delete brep.back();
		brep.pop_back();
	}
	bspnodecount--;
}

int BSPCount(BSPNode *n)
{
	if(!n) return 0;
	return 1+BSPCount(n->under)+BSPCount(n->over);
}

class SortPerm{
  public:
	int   i;
	float  v;
	Face *f;
	SortPerm(){}
	SortPerm(int _i,float _v,Face *_f):i(_i),v(_v),f(_f){}
};
static int FaceAreaCompare(const void *_a,const void *_b) {
	SortPerm *a = ((SortPerm *) _a); 
	SortPerm *b = ((SortPerm *) _b); 
	return ((a->v < b->v)?-1:1); 
}
void ReorderFaceArray(std::vector<Face *> &face){
	std::vector<SortPerm> sp;
	for(unsigned int i=0;i<face.size();i++) {
		sp.push_back(SortPerm(i,FaceArea(face[i]),face[i]));
	}
    assert(sp.size() == face.size());
    qsort(sp.data(), (unsigned long)sp.size(), sizeof(SortPerm), FaceAreaCompare);
	for(unsigned int i=0;i<face.size();i++) {
		face[i] = sp[i].f;
		if(i) {
			assert(FaceArea(face[i])>=FaceArea(face[i-1]));
		}
	}
}

static int count[4];

//int PlaneTest(float3 normal,float dist,float3 v,float epsilon) {
//	float a  = dot(v,normal)+dist;
//	int   flag = (a>epsilon)?OVER:((a<-epsilon)?UNDER:COPLANAR);
//	return flag;
//}
//
//int PlaneTest(const float4 &p, const float3 &v,float epsilon) {
//	float a  = dot(v,p.xyz())+p.w;
//	int   flag = (a>epsilon)?OVER:((a<-epsilon)?UNDER:COPLANAR);
//	return flag;
//}

float sumbboxdim(WingMesh *convex)
{
	float3 bmin,bmax;
	std::tie(bmin, bmax) = Extents<float,3>(convex->verts); // (convex->verts.data(), convex->verts.size(), bmin, bmax);
	return dot(float3(1,1,1),bmax-bmin);
}

float PlaneCost(std::vector<Face *> &inputfaces,const float4 &split,WingMesh *space,int onbrep)
{
	count[COPLANAR] = 0;
	count[UNDER]    = 0;
	count[OVER]     = 0;
	count[SPLIT]    = 0;
	for(unsigned int i=0;i<inputfaces.size();i++) {
		count[FaceSplitTest(inputfaces[i],split.xyz(),split.w,FUZZYWIDTH)]++;
	}
    if (space->verts.size() == 0) {
		// The following formula isn't that great.
		// Better to use volume as well eh.
		return (float)(abs(count[OVER]-count[UNDER]) + count[SPLIT] - count[COPLANAR]);
	}
	float volumeover =(float)1.0;
	float volumeunder=(float)1.0;
	float volumetotal=WingMeshVolume(space);
	WingMesh *spaceunder= WingMeshCrop(space,float4( split.xyz(), split.w));
	WingMesh *spaceover = WingMeshCrop(space,float4(-split.xyz(),-split.w));
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
	delete spaceover;
	delete spaceunder;
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



void DividePolys(const float4 &splitplane,std::vector<Face *> &inputfaces,
				 std::vector<Face *> &under,std::vector<Face *> &over,std::vector<Face *> &coplanar){
	int i=inputfaces.size();
	while(i--) {
		int flag = FaceSplitTest(inputfaces[i],splitplane.xyz(),splitplane.w,FUZZYWIDTH);

		if(flag == OVER) {
			over.push_back(inputfaces[i]);
		}
		else if(flag == UNDER) {
			under.push_back(inputfaces[i]);
		}
		else if(flag == COPLANAR) {
			coplanar.push_back(inputfaces[i]);
		}
		else {
			assert(flag == SPLIT);
			Face *und=FaceClip(FaceDup(inputfaces[i]),splitplane);
			Face *ovr=FaceClip(FaceDup(inputfaces[i]),float4(-splitplane.xyz(),-splitplane.w));
			assert(ovr);
			assert(und);
			over.push_back(ovr);
			under.push_back(und);
		}
	}
}



BSPNode * BSPCompile(std::vector<Face *> &inputfaces,WingMesh *space,int side) 
{
    if (inputfaces.size() == 0) {
		BSPNode *node = new BSPNode();
		node->convex = space;
		node->isleaf=side; 
		return node;
	}
	std::vector<Face *> over;
	std::vector<Face *> under;
	std::vector<Face *> coplanar;
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
			float val=PlaneCost(inputfaces,*inputfaces[i],space,1);
			if(val<minval) {
				minval=val;
				split.xyz() = inputfaces[i]->xyz();
				split.w   = inputfaces[i]->w;
			}
		}
		assert(split.xyz() != float3(0,0,0));
		PlaneCost(inputfaces,split,space,1);
        if (allowaxial && inputfaces.size() > 8) {
			// consider some other planes:
            for (unsigned int i = 0; i<inputfaces.size() && (int)i<facetestlimit; i++) {
				for(unsigned int j=0;j<inputfaces[i]->vertex.size();j++ ) {
					float val;
					if(allowaxial & (1<<0))
					{
						val = PlaneCost(inputfaces,float4(float3(1,0,0),-inputfaces[i]->vertex[j].x),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(1, 0, 0);
							split.w   = -inputfaces[i]->vertex[j].x;
						}
					}
					if(allowaxial & (1<<1))
					{
						val = PlaneCost(inputfaces,float4(float3(0,1,0),-inputfaces[i]->vertex[j].y),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(0, 1, 0);
							split.w   = -inputfaces[i]->vertex[j].y;
						}
					}
					if(allowaxial & (1<<2))
					{
						val = PlaneCost(inputfaces,float4(float3(0,0,1),-inputfaces[i]->vertex[j].z),space,0);
						if(val<minval && (count[OVER]*count[UNDER]>0 || count[SPLIT]>0)) { 
							minval=val;
							split.xyz() = float3(0, 0, 1);
							split.w   = -inputfaces[i]->vertex[j].z;
						}
					}
				}
			}
		}
	}
	// Divide the faces
	BSPNode *node=new BSPNode();
	node->xyz() = split.xyz();
	node->w   = split.w;
	node->convex   = space;

	DividePolys(float4(split.xyz(), split.w), inputfaces, under, over, coplanar);

	for(unsigned int i=0;i<over.size();i++) {
		for(unsigned int j=0;j<over[i]->vertex.size();j++) {
			assert(dot(node->xyz(),over[i]->vertex[j])+node->w >= -FUZZYWIDTH);
		}
	}
	for(unsigned int i=0;i<under.size();i++) {
		for(unsigned int j=0;j<under[i]->vertex.size();j++) {
			assert(dot(node->xyz(),under[i]->vertex[j])+node->w <= FUZZYWIDTH);
		}
	}

	WingMesh* space_under = WingMeshCrop(space, float4( split.xyz(),  split.w));
	WingMesh* space_over  = WingMeshCrop(space, float4(-split.xyz(), -split.w));

	node->under = BSPCompile(under,space_under,UNDER);
	node->over  = BSPCompile(over ,space_over ,OVER );
	return node;
}

/*
void DeriveCells(BSPNode *node,Polyhedron *cell) {
	assert(node);
	node->cell = cell;
	cell->volume = Volume(cell);
	if(node->isleaf) {
		return;
	}
	// Under SubTree
	Polyhedron *under = PolyhedronDup(cell);
	Poly *sf = under->Crop(Plane(node->normal,node->dist));
	assert(node->under); 
	DeriveCells(node->under,under);
	// Over  SubTree
	Polyhedron *over  = PolyhedronDup(cell);
	over->Crop(Plane(-node->normal,-node->dist));
	assert(node->over );
	DeriveCells(node->over ,over );

}
*/

void BSPDeriveConvex(BSPNode *node,WingMesh *convex) {
	assert(node);
    if (convex && convex->edges.size() && convex->verts.size())
	{
		assert(convex->verts.size());
		assert(convex->edges.size());
		assert(convex->faces.size());
	}
	else 
	{
		convex=NULL;
	}
	if(node->convex)  // clean up previous convex if there is one.  
	{
		delete node->convex;
		node->convex=NULL;
	}

	node->convex = convex;
	if(node->isleaf) {
		return;
	}
	// if we are "editing" a bsp then the current plane may become coplanar to one of its parents (boundary of convex) or outside of the current volume (outside the convex)
	WingMesh *cu=NULL;
	WingMesh *co=NULL;
	if(convex)
	{
		int f=WingMeshSplitTest(convex,*node);
		if(f==SPLIT)
		{
			cu = WingMeshCrop(convex,*node);
			co = WingMeshCrop(convex, float4(-node->xyz(), -node->w));
		}
		else if(f==OVER)
		{
			co = WingMeshDup(convex);
		}
		else if(f==UNDER)
		{
			cu = WingMeshDup(convex);
		}
		else
		{
			assert(0); // hunh? the 3d convex has 0 volume
		}
	}

	// Under SubTree
	assert(node->under);
	BSPDeriveConvex(node->under,cu); 
	// Over  SubTree
	assert(node->over );
	BSPDeriveConvex(node->over ,co);
}


std::vector<WingMesh*> BSPGetSolids(BSPNode *bsp)
{
	std::vector<WingMesh*> meshes;
	std::vector<BSPNode *> stack;
	stack.push_back(bsp);
    while (stack.size())
	{
		BSPNode *n = stack.back(); stack.pop_back();
		if(!n) continue;
		stack.push_back(n->under);
		stack.push_back(n->over);
		if(n->isleaf == UNDER)
		{
			meshes.push_back(n->convex);
		}
	}
	return meshes;
}

 
void BSPGetSolidsr(BSPNode *n,std::vector<WingMesh*> &meshes)
{
		if(!n) return;
		if(n->isleaf == UNDER)
		{
			meshes.push_back(n->convex);
		}
		BSPGetSolidsr(n->under,meshes);
		BSPGetSolidsr(n->over,meshes);
	
}



void BSPTranslate(BSPNode *n,const float3 &_offset)
{
	float3 offset(_offset);
	if(!n ) {
		return;
	}
	n->w = n->w - dot(n->xyz(), offset);
	if(n->convex) {
		WingMeshTranslate(n->convex,_offset);
	}
	for(auto & f : n->brep) {
		FaceTranslate(f,_offset);
	}
	BSPTranslate(n->under,_offset);
	BSPTranslate(n->over,_offset);
}


void BSPRotate(BSPNode *n,const float4 &r)
{
	if(!n ) {
		return;
	}
	n->xyz() = qrot(r, n->xyz());
	if(n->convex) 
	{
		WingMeshRotate(n->convex,r);
	}
	for (auto & f : n->brep) {
		FaceRotate(f, r);
	}
	BSPRotate(n->under,r);
	BSPRotate(n->over,r);
}


void BSPScale(BSPNode *n,float s)
{
	if(!n) return;
	n->w = n->w * s;
	if(n->convex) {
		for(unsigned int i=0;i<n->convex->verts.size();i++){
			n->convex->verts[i] *= s;
		}
		for(unsigned int i=0;i<n->convex->faces.size();i++) {
			n->convex->faces[i].w *=s;
		}
	}
	for(unsigned i=0;i<n->brep.size();i++) {
		Face *f = n->brep[i];
		f->w *= s;
		// Scale(f->vertex,s);
		for(unsigned int j=0;j<f->vertex.size();j++){
			f->vertex[j] *= s;
		}
			
	}
	BSPScale(n->under,s);
	BSPScale(n->over,s);
}

void NegateFace(Face *f)
{
	f->w *=-1.0f;
	f->xyz() *= -1.0f;
	std::vector<float3> tmp;
	for(unsigned int i=0;i<f->vertex.size();i++)
	{
        tmp.push_back(f->vertex[f->vertex.size() - i - 1]);
	}
	for(unsigned int i=0;i<f->vertex.size();i++)
	{
		f->vertex[i] = tmp[i];
	}
}

void NegateTreePlanes(BSPNode *n) 
{
	if(!n) {
		return;
	}
	for(unsigned int i=0;i<n->brep.size();i++)
	{
		NegateFace(n->brep[i]);
	}
	if(n->isleaf) {
		n->isleaf = (n->isleaf==UNDER)?OVER:UNDER;
		return;
	}
	n->xyz() = -n->xyz();
	n->w   = -n->w;
	NegateTreePlanes(n->under);
	NegateTreePlanes(n->over);
	BSPNode *tmp = n->over;
	n->over  = n->under;
	n->under = tmp;
}


void NegateTree(BSPNode *root) 
{
	std::vector<Face*> faces;
	NegateTreePlanes(root);
	BSPRipBrep(root,faces);
	for(unsigned int i=0;i<faces.size();i++) {
		extern void FaceEmbed(BSPNode *node,Face *face);
		FaceEmbed(root,faces[i]);
	}
}

BSPNode *BSPDup(BSPNode *n) 
{
	if(!n) {
		return NULL;
	}
	BSPNode* a = new BSPNode();
	a->xyz() = n->xyz();
	a->w   = n->w;
	a->isleaf = n->isleaf;
	if(n->convex) {
		a->convex = WingMeshDup(n->convex);
	}
	for(unsigned int i=0;i<n->brep.size();i++) {
		a->brep.push_back(FaceDup(n->brep[i]));
	}
	a->under= BSPDup(n->under);
	a->over = BSPDup(n->over);
	return a;
}

int BSPFinite(BSPNode *bsp)
{
	return !HitCheck(bsp,1,float3(999999,9999999,999999),float3(999999,9999999,999999),NULL);
}

