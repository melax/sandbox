//
//      BSP
//  (c) Stan Melax 1998-2007  bsd licence
//  see file bsp.h 
//
// this module provides csg boolean operation (geomod) support
//
//

#include "bsp.h"

template<class T> T Pop(std::vector<T> &a){ T t = a.back(); a.pop_back(); return t; }

static int fusenodes=0;

BSPNode *BSPClean(BSPNode *n)
{
	// removes empty cells.
	if(n->convex.verts.size() == 0) 
	{
		delete n;
		return NULL;
	}
	if(n->isleaf)
	{
		n->xyz() = float3(0,0,0); 
		n->w=0;
		assert(n->over==NULL);
		assert(n->under==NULL);
		return n;
	}
	n->under = BSPClean(n->under);
	n->over  = BSPClean(n->over );
	if(!n->over) 
	{
		BSPNode *r= n->under; 
		n->under=NULL;
		delete n;
		return r;
	}
	if(!n->under)
	{
		BSPNode *r= n->over; 
		n->over=NULL;
		delete n;
		return r;
	}
	if(n->over->isleaf && n->over->isleaf==n->under->isleaf)  
	{
		// prune when both children are leaf cells of the same type
		n->isleaf = n->over->isleaf;
		delete n->over;
		delete n->under;
		n->over=n->under=NULL;
		n->xyz() = float3(0, 0, 0);
		n->w  =0;
	}
	assert(n->convex.verts.size());
	return n;
}

void BSPPartition(BSPNode *n, const float4 &p, BSPNode * &nodeunder, BSPNode * &nodeover) {
	nodeunder=NULL;
	nodeover =NULL;
	if(!n) {
		return;
	}
//	assert(n->cell);
	int flag;
	//flag = SplitTest(n->cell,p);
	//assert(flag==SplitTest(*n->convex,p));
	flag=WingMeshSplitTest(n->convex,p);
	if(flag == UNDER) {
		nodeunder = n;
		return;
	}
	if(flag==OVER) {
		nodeover = n;
		return;
	}
	assert(flag==SPLIT);
//	Polyhedron *cellover  = PolyhedronDup(n->cell);
//	Polyhedron *cellunder = PolyhedronDup(n->cell);
//	cellunder->Crop(p);
//	cellover->Crop(Plane(-p.normal,-p.dist));
	nodeunder = new BSPNode(n->xyz(), n->w);
	nodeover  = new BSPNode(n->xyz(),n->w);
	nodeunder->isleaf = n->isleaf;
	nodeover->isleaf = n->isleaf;
//	nodeunder->cell= cellunder;
//	nodeover->cell = cellover;
	nodeunder->convex = WingMeshCrop(n->convex,p);
	nodeover->convex = WingMeshCrop(n->convex, float4(-p.xyz(), -p.w));
	if(n->isleaf==UNDER) { 
		int i;
		BSPNode fake(p.xyz(), p.w);
		fake.under=nodeunder;
		fake.over=nodeover;
		i=n->brep.size();
		while(i--){
			Face *face = n->brep[i];
			FaceEmbed(&fake,face);
		}
		n->brep.clear();
		fake.under=fake.over=NULL;

	}
	BSPPartition(n->under,p,nodeunder->under,nodeover->under);
	BSPPartition(n->over ,p,nodeunder->over ,nodeover->over );
	if(n->isleaf) { 
		assert(nodeunder->isleaf);
		assert(nodeover->isleaf);
		n->over=n->under=NULL;
		delete n;
		return;
	} 
	assert(nodeunder->over || nodeunder->under);
	assert(nodeover->over  || nodeover->under);
	n->over=n->under=NULL;
	delete n;
	n=NULL;
	if(!nodeunder->under) {
//		assert(SplitTest(nodeunder->cell,*nodeunder)==OVER);
		BSPNode *r = nodeunder;
		nodeunder = nodeunder->over; 
		r->over=NULL;
		delete r;
	}
	else if(!nodeunder->over) {
//		assert(SplitTest(nodeunder->cell,*nodeunder)==UNDER);
		BSPNode *r = nodeunder;
		nodeunder = nodeunder->under;
		r->under=NULL; 
		delete r;
	}
	assert(nodeunder);
	assert(nodeunder->isleaf || (nodeunder->under && nodeunder->over));
	if(!nodeover->under) {
//		assert(SplitTest(nodeover->cell,*nodeover)==OVER);
		BSPNode *r = nodeover;
		nodeover = nodeover->over; 
		r->over=NULL;
		delete r;
	}
	else if(!nodeover->over) {
//		assert(SplitTest(nodeover->cell,*nodeover)==UNDER);
		BSPNode *r = nodeover;
		nodeover = nodeover->under;
		r->under = NULL; 
		delete r;
	}
	assert(nodeover);
	assert(nodeover->isleaf || (nodeover->under && nodeover->over));
	if(!nodeunder->isleaf && nodeunder->over->isleaf && nodeunder->over->isleaf==nodeunder->under->isleaf) {
		nodeunder->isleaf = nodeunder->over->isleaf; // pick one of the children
		int i;
		i=nodeunder->under->brep.size();
		while(i--){
			nodeunder->brep.push_back(nodeunder->under->brep[i]);
		}
		nodeunder->under->brep.clear();
		i=nodeunder->over->brep.size();
		while(i--){
			nodeunder->brep.push_back(nodeunder->over->brep[i]);
		}
		nodeunder->over->brep.clear();
		delete nodeunder->under;
		delete nodeunder->over;
		nodeunder->over = nodeunder->under = NULL;
	}
	// wtf:	if(!nodeover->isleaf && nodeover->over->isleaf==nodeover->under->isleaf) {
	if(!nodeover->isleaf && nodeover->over->isleaf && nodeover->over->isleaf==nodeover->under->isleaf) {
		nodeover->isleaf = nodeover->over->isleaf; // pick one of the children
		int i;
		i=nodeover->under->brep.size();
		while(i--){
			nodeover->brep.push_back(nodeover->under->brep[i]);
		}
		nodeover->under->brep.clear();
		i=nodeover->over->brep.size();
		while(i--){
			nodeover->brep.push_back(nodeover->over->brep[i]);
		}
		nodeover->over->brep.clear();
		delete nodeover->under;
		delete nodeover->over;
		nodeover->over = nodeover->under = NULL;
	}
/*	if(fusenodes) {
		if(0==nodeunder->isleaf) {
			if(nodeunder->over->isleaf==UNDER) {
				DeriveCells(nodeunder->under,nodeunder->cell);
				nodeunder = nodeunder->under; // memleak
			}
			else if(nodeunder->under->isleaf==OVER) {
				DeriveCells(nodeunder->over,nodeunder->cell);
				nodeunder = nodeunder->over; // memleak
			}
		}
		assert(nodeunder);
		if(0==nodeover->isleaf) {
			if(nodeover->over->isleaf==UNDER) {
				DeriveCells(nodeover->under,nodeover->cell);
				nodeover = nodeover->under; // memleak
			}
			else if(nodeover->under->isleaf==OVER) {
				DeriveCells(nodeover->over,nodeover->cell);
				nodeover = nodeover->over;  // memleak
			}
		}
		assert(nodeover);
	}
*/
}

void FaceCutting(BSPNode *n,std::vector<Face*> &faces)
{
	if(n->isleaf==OVER)
	{
		return;
	}
	if(n->isleaf==UNDER)
	{
		for(auto &f : faces)
		{
			delete f;
		}
		faces.clear();
		return;
	}
	std::vector<Face*> faces_over;
	std::vector<Face*> faces_under;
	std::vector<Face*> faces_coplanar;
    while (faces.size())
	{
		Face *f;
		f= Pop(faces);
		int s = FaceSplitTest(f, n->xyz(), n->w);
		if(s==COPLANAR)
			faces_coplanar.push_back(f);
		else if(s==UNDER)
			faces_under.push_back(f);
		else if(s==OVER)
			faces_over.push_back(f);
		else
		{
			assert(s==SPLIT);
			Face *ovr = FaceDup(f);
			FaceClip(f,(*n));
			FaceClip(ovr, float4(-n->xyz(), -n->w));
			faces_under.push_back(f);
			faces_over.push_back(ovr);
		}
	}
	FaceCutting(n->under,faces_under);
	FaceCutting(n->over,faces_over);
	for(unsigned int i=0;i<faces_under.size();i++)
		faces.push_back(faces_under[i]);
	for (unsigned int i = 0; i<faces_over.size(); i++)
		faces.push_back(faces_over[i]);
	for (unsigned int i = 0; i<faces_coplanar.size(); i++)
		faces.push_back(faces_coplanar[i]);
}

BSPNode *BSPUnion(BSPNode *a,BSPNode *b) {
	if(!a || b->isleaf == UNDER || a->isleaf==OVER) {
		if(a && b->isleaf==UNDER)
		{
			FaceCutting(a,b->brep);
		}
		return b;
	}
	if(a->isleaf == UNDER || b->isleaf==OVER) {
		return a;
	}
	BSPNode *aover;
	BSPNode *aunder;
	// its like "b" is the master, so a should be the little object and b is the area's shell
	assert(!a->isleaf);
	BSPPartition(a, float4(b->xyz(), b->w), aunder, aover);
	assert(aunder || aover);
	b->under = BSPUnion(aunder,b->under);
	b->over  = BSPUnion(aover ,b->over );
/*	if(fusenodes) {
		if(b->over->isleaf == UNDER) {
			DeriveCells(b->under,b->cell);
			return b->under;
		}
		if(b->under->isleaf == OVER) {
			DeriveCells(b->over,b->cell);
			return b->over;
		}
	}
*/
	return b;
}


int bspmergeallowswap=0;
BSPNode *BSPIntersect(BSPNode *a,BSPNode *b) 
{
	int swapflag;
	if(!a||a->isleaf == UNDER || b->isleaf==OVER) {
		if(a&&a->isleaf==UNDER ) {
            while (a->brep.size()) {
				FaceEmbed(b,Pop(a->brep));
			}
		}
		delete a;
		return b;
	}
	assert(b);
	if(b->isleaf == UNDER || a->isleaf==OVER) {
		if(b->isleaf==UNDER ) {
            while (b->brep.size()) {
				FaceEmbed(a,Pop(b->brep));
			}
		}
		delete b;
		return a;
	}
	// I'm not sure about the following bit - it only works if booleaning bsp's cells cover entire area volume too
	if(bspmergeallowswap)if( SPLIT != (swapflag = WingMeshSplitTest(b->convex,*a))) {
		if(swapflag == OVER) {
			a->over = BSPIntersect(a->over,b);
			return a;
		}
		if(swapflag == UNDER) {
			a->under= BSPIntersect(a->under,b);
			return a;
		}
	}
	BSPNode *aover;
	BSPNode *aunder;
	// its like "b" is the master, so a should be the little object and b is the area's shell
	BSPPartition(a, float4(b->xyz(), b->w), aunder, aover);
	b->under = BSPIntersect(aunder,b->under);
	b->over  = BSPIntersect(aover ,b->over );
	if(b->over->isleaf && b->over->isleaf==b->under->isleaf) {  // both children are leaves of same type so merge them into parent
        while (b->over->brep.size()) {
			b->brep.push_back(Pop(b->over->brep));
		}
		assert(b->over->brep.size()==0);
        while (b->under->brep.size()) {
			b->brep.push_back(Pop(b->under->brep));
		}
		b->isleaf = b->over->isleaf;
		delete b->over;
		delete b->under;
		b->over=b->under=NULL;
	}
/*	if(fusenodes) {
		if(b->over->isleaf == UNDER) {
			DeriveCells(b->under,b->cell);
			return b->under;
		}
		if(b->under->isleaf == OVER) {
			DeriveCells(b->over,b->cell);
			return b->over;
		}
	}
*/
	return b;
}

//int HitCheckConvexGJK(const Collidable *dude, BSPNode *n)
//  int HitCheckConvexGJKm(std::function<float3(const float3&)> collidersupportmap, BSPNode *n)
//  {
//  	Contact hitinfo;
//  	if(n->isleaf==OVER)  return 0;
//  	if(n->isleaf==UNDER) return (!Separated(collidersupportmap,(n->convex),hitinfo,1));
//  	int f=0;
//  	int h=0;
//  
//  	auto u = collidersupportmap(-n->xyz());
//  	auto o = collidersupportmap(n->xyz());
//  	f |= PlaneTest(n->xyz(), n->w, u, 0);
//  	f |= PlaneTest(n->xyz(), n->w, o, 0);
//  	if(f&UNDER)
//  	{
//  		h = HitCheckConvexGJKm(collidersupportmap, n->under);
//  		if(h) return 1;
//  	}
//  	if(f&OVER)
//  	{
//  		h = HitCheckConvexGJKm(collidersupportmap, n->over);
//  		if(h) return 1;
//  	}
//  	return 0;
//  }

//  std::vector<WingMesh*> ProximityCellsm(std::function<float3(const float3&)> supportmap, BSPNode *bsp, float padding)
//  {
//  	std::vector<WingMesh*> cells;
//  	std::vector<BSPNode *> stack;
//  	stack.push_back(bsp);
//  	Contact hitinfo;
//  	while (stack.size())
//  	{
//  		BSPNode *n = Pop(stack);
//  		if (!n) continue;
//  		if (n->isleaf == UNDER)
//  		{
//  			cells.push_back(n->convex);
//  			continue;
//  		}
//  		if (n->isleaf == OVER)
//  			continue;
//  		int f = 0;
//  		auto u = supportmap(-n->xyz());
//  		auto o = supportmap(n->xyz());
//  		f |= PlaneTest(n->xyz(), n->w - padding, u, 0);  // verify sign for padding, seems right
//  		f |= PlaneTest(n->xyz(), n->w + padding, o, 0);
//  		if (f&UNDER)
//  		{
//  			stack.push_back(n->under); // ProximityCellsm(dude, n->under, cells, padding);
//  		}
//  		if (f&OVER)
//  		{
//  			stack.push_back(n->over);  // ProximityCellsm(dude, n->over, cells, padding);
//  		}
//  	}
//  	return cells;
//  }
