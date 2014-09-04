//
//      BSP
//  (c) Stan Melax 1998-2007  bsd licence
//  see file bsp.h 
//
// this module provides collision support
//
//


#include "bsp.h"



float3   HitCheckImpactNormal;  // global variable storing the normal of the last hit 
int      HitCheckVertexHit;     // global variable storing the vertex id of the last convex obj collision
BSPNode *HitCheckNodeHitLeaf=NULL; // global variable storing the BSP node leaf hit.
BSPNode *HitCheckNodeHitOverLeaf=NULL; // global variable storing the BSP over leaf node that was just beforehit.
BSPNode *HitCheckNode=NULL; // global variable storing the BSP node plane hit.


int PointInsideFace(Face* f,const float3 &s)
{
	int inside=1;
	for(unsigned int j=0;inside && j<f->vertex.size();j++) 
    {
			float3 &pp1 = f->vertex[j] ;
            float3 &pp2 = f->vertex[(j + 1) % f->vertex.size()];
			float3 side = cross((pp2-pp1),(s-pp1));
			inside = (dot(f->xyz(),side) >= 0.0);
	}
	return inside;
}

Face* FaceHit(BSPNode *leaf,const float4 &plane,const float3 &s)
{
	for(unsigned int i=0;i<leaf->brep.size();i++)
	{
		Face *f = leaf->brep[i];
		if(*f==plane || *f==-plane ) continue;  // was coplanar
		static int craptest=0;   // sometimes the normal is facing the other way, but that's ok.
		if(craptest)
		{ 
			craptest=0;
			assert(f->xyz() == plane.xyz());
			assert(f->w     == plane.w    );
		}
		if(PointInsideFace(f,s)) 
		{
			return f;
		}
	}
	return NULL;
}

Face* FaceHit(const float3& s)
{
	if(!HitCheckNodeHitLeaf || !HitCheckNode) return NULL;
	BSPNode *leaf=HitCheckNodeHitLeaf;
	float4 &plane = *HitCheckNode;
	return FaceHit(leaf,plane,s);
}

static int bypass_first_solid=0;  // put into hitinfo
int HitCheck(BSPNode *node,int solid,float3 v0,float3 v1,float3 *impact) {
	assert(node);
	if(node->isleaf ){
		if(node->isleaf==UNDER ){
			HitCheckNodeHitLeaf=node;
			if(impact) *impact = v0;
		}
		else
		{
			HitCheckNodeHitOverLeaf = node;
			bypass_first_solid=0;
		}
		return (node->isleaf==UNDER  && !bypass_first_solid );
	}
	int f0 = (dot(v0,node->xyz())+node->w>0)?1:0;  // if v0 above plane
	int f1 = (dot(v1, node->xyz()) + node->w>0) ? 1 : 0;  // if v1 above plane
	if(f0==0 && f1==0) {
		return HitCheck(node->under,1,v0,v1,impact);
	}
	if(f0==1 && f1==1) {
		return HitCheck(node->over,0,v0,v1,impact);
	}
	float3 vmid = PlaneLineIntersection(*node,v0,v1);
	if(f0==0) {
		assert(f1==1);
		// perhaps we could pass 'solid' instead of '1' here:
		if(HitCheck(node->under,1,v0,vmid,impact)) {
			return 1;
		}
		HitCheckImpactNormal = -node->xyz();
		HitCheckNode = node;
		return HitCheck(node->over,0,vmid,v1,impact);
	}
	assert(f0==1 && f1==0);
	if(HitCheck(node->over,0,v0,vmid,impact)) {
		return 1;
	}
	HitCheckImpactNormal = node->xyz();
	HitCheckNode = node;
	return HitCheck(node->under,1,vmid,v1,impact);
}

int HitCheckSolidReEnter(BSPNode *node,float3 v0,float3 v1,float3 *impact) 
{
	bypass_first_solid =1;
	int	h=HitCheck(node,1,v0,v1,impact);
	bypass_first_solid =0; // make sure you turn this back off in case the query never hit empty space.
	return h;
}


int SegmentUnder(const float4 &plane, const float3 &v0, const float3 &v1, const float3 &nv0, float3 *w0, float3 *w1, float3 *nw0){
	float d0,d1;
	d0 = dot(plane.xyz(), v0) + plane.w;
	d1 = dot(plane.xyz(), v1) + plane.w;
	if(d0>0.0f && d1>0.0f) {
		return 0;
	}
	if(d0<=0.0f && d1<=0.0f) {
		*w0 =v0;
		*w1 =v1;
		*nw0=nv0;
		return 3;
	}
	float3 vmid=PlaneLineIntersection(plane,v0,v1);
	if(d0>0.0f) {
		assert(d1<=0.0f);
		*w1=v1;
		*w0=vmid;
		*nw0 = plane.xyz();
		return 2;
	}
	else{
		assert(d1>0.0f);
		*w1 = vmid;
		*w0 = v0;
		*nw0=nv0;
		return 1;
	}
}
int SegmentOver(const float4 &plane, const float3 &v0, const float3 &v1, const float3 &nv0, float3 *w0, float3 *w1, float3 *nw0){
	return SegmentUnder(float4(-plane.xyz(), -plane.w), v0, v1, nv0, w0, w1, nw0);
}
int ConvexHitCheck(WingMesh *convex,float3 v0,float3 v1,float3 *impact) 
{
	float3 nml;
	for(unsigned int i=0;i<convex->faces.size();i++) {
		if(!SegmentUnder(convex->faces[i],v0,v1,nml,&v0,&v1,&nml)){
			return 0;
		}
	}
	if(impact) *impact=v0;
	return 1;
}


int HitCheckSphere(float r,BSPNode *node,int solid,float3 v0,float3 v1,float3 *impact,const float3 &nv0) {
	assert(node);
	if(node->isleaf ){
		if(node->isleaf==UNDER ){
			if(impact) *impact = v0;
			HitCheckImpactNormal = nv0;
		}
		return (node->isleaf==UNDER);
	}
	float3 w0,w1,nw0;
	int hit=0;
	if(SegmentUnder(float4(node->xyz(),node->w-r),v0,v1,nv0,&w0,&w1,&nw0)) {
		hit |= HitCheckSphere(r,node->under,1,w0,w1,&v1,nw0);
	}
	if(SegmentOver( float4(node->xyz(),node->w+r),v0,v1,nv0,&w0,&w1,&nw0)) {
		hit |= HitCheckSphere(r,node->over,0,w0,w1,&v1,nw0);
	}
	if(hit) {
		*impact = v1;
	}
	return hit;
}

float3 TangentPointOnCylinder(const float r,const float h,const float3 &n) {
	float3 p;
	float xymag = sqrtf(n.x*n.x+n.y*n.y);
	if(xymag==0.0f) {xymag=1.0f;} // yup this works
    p.x = r * n.x /xymag;
    p.y = r * n.y /xymag;
    p.z = (n.z>0) ?h:0;  // reference point at base of cylinder 
    // p.z = (n.z>0) ?h/2:-h/2; // use if reference point at mid height of cylinder (h/2)
	return p;
}

int usebevels=1;

// hmmm this isn't working right now for some reason:
int HitCheckBevelsCylinder(float r,float h,WingMesh *convex,float3 v0,float3 v1,float3 *impact,float3 nv0) 
{
    if (!convex || !convex->edges.size())
	{
		return 0; // nothing here to hit (likely an orphan plane due to bediter)
	}
	assert(convex->edges.size());
	for(unsigned int i=0;i<convex->edges.size();i++) 
	{
		WingMesh::HalfEdge &edge0 = convex->edges[i];
		WingMesh::HalfEdge &edgeA = convex->edges[edge0.adj];
		if((int)i>edge0.adj) {
			continue; // no need to test edge twice
		}
		if(dot(convex->faces[edge0.face].xyz(),convex->faces[edgeA.face].xyz())> -0.03f ) {
			continue;
		}
		float4 bev(normalize(convex->faces[edge0.face].xyz() + convex->faces[edgeA.face].xyz()),0);
		bev.w =  -dot(bev.xyz(),convex->verts[edge0.v]);
		for(unsigned int j=0;j<convex->verts.size();j++) {
			extern int PlaneTest(const float4 &p, const float3 &v, float epsilon = PAPERWIDTH);
			assert(PlaneTest(bev,convex->verts[j],PAPERWIDTH*10) != OVER);
		}
	    bev.w += -dot(TangentPointOnCylinder(r,h,-bev.xyz()),-bev.xyz()); 
		if(0==SegmentUnder(bev,v0,v1,nv0,&v0,&v1,&nv0)) {
			return 0;
		}
	}
	*impact = v0;
	HitCheckImpactNormal = nv0;
	return 1;
}


int HitCheckCylinder(float r,float h,BSPNode *node,int solid,float3 v0,float3 v1,float3 *impact,const float3 &nv0) 
{
	assert(node);
	if(node->isleaf )
	{
		if(usebevels && node->isleaf==UNDER) {
			return HitCheckBevelsCylinder(r,h,node->convex,v0,v1,impact,nv0);
		}
		if(node->isleaf==UNDER){
			if(impact) *impact = v0;
			HitCheckImpactNormal = nv0;
		}
		return (node->isleaf==UNDER);
	}

    float offset_up   = -dot(TangentPointOnCylinder(r,h,-node->xyz()),-node->xyz()); 
    float offset_down =  dot(TangentPointOnCylinder(r,h, node->xyz()), node->xyz()); 

	float3 w0,w1,nw0;
	int hit=0;
	if(SegmentUnder(float4(node->xyz(),node->w+offset_up  ),v0,v1,nv0,&w0,&w1,&nw0)) {
		hit |= HitCheckCylinder(r,h,node->under,1,w0,w1,&v1,nw0);
	}
	if(SegmentOver( float4(node->xyz(),node->w+offset_down),v0,v1,nv0,&w0,&w1,&nw0)) {
		hit |= HitCheckCylinder(r,h,node->over,0,w0,w1,&v1,nw0);
	}
	if(hit) {
		*impact = v1;
	}
	return hit;
}

class Collision
{
public:
	int hit;
	float3 impact;
	float3 normal;
	operator int(){return hit;}
	Collision(int _hit=0):hit(_hit){}
};
Collision hittest()
{
	return Collision();
}
static void testc()
{
	int x;
	if(hittest())
	{
		 x++;
	}
	Collision c = hittest();
	x++;
}


int PortionUnder(float4 plane,std::vector<float3> &verts,
				 const float3 &v0,const float3 &v1,const float3 &nv0,int vrtv0,
				 const float4 &q0,const float4 &q1,
				 float3 *w0,float3 *w1,float3 *nw0,int *vrtw0,
				 float4 *wq0,float4 *wq1
				)
{
	std::vector<float3> verts0;
	std::vector<float3> verts1;
	int under0=0;
	int under1=0;
	int closest0=-1;
	int closest1=-1;
	if(dot(plane.xyz(),v0)+plane.w <0)
	{
		under0=1;
	}
	else if(dot(plane.xyz(),v0)+plane.w <5.0f)
	{
		for(unsigned int i=0;i<verts.size();i++) {
			verts0.push_back( qrot(q0,verts[i]));
		}
		closest0 = maxdir(verts0,-plane.xyz());
		if(dot(plane.xyz(),v0+verts0[closest0])+plane.w <0){
			under0=1;
		}
	}
	if(dot(plane.xyz(),v1)+plane.w <0)
	{
		under1=1;
	}
	else if(dot(plane.xyz(),v1)+plane.w <5.0f)
	{
		for(unsigned int i=0;i<verts.size();i++) {
			verts1.push_back(qrot(q1,verts[i]));
		}
		closest1 = maxdir(verts1,-plane.xyz());
		if(dot(plane.xyz(),v1+verts1[closest1])+plane.w <0){
			under1=1;
		}
	}
	
	if(!under0 && !under1) {
		return 0;
	}
	if(under0) {
		*nw0  = nv0;
		*wq0  = q0;
		*w0   = v0;
		*vrtw0= vrtv0;
	}
	else {
		if(closest1==-1) {
			// this code already exists above, but wasnt executed then // FIXME
			for(unsigned int i=0;i<verts.size();i++) {
				verts1.push_back(qrot(q1,verts[i]));
			}
			closest1 = maxdir(verts1,-plane.xyz());
		}
		assert(under1);
		// could be solved analytically, but im lazy and just implementing iterative thingy
		float ta=0.0f,tb=1.0f;
		for(unsigned int i=0;i<10;i++) {
			float tmid = (ta+tb)/2.0f;
			float4 qmid = qlerp(q0, q1, tmid);
			float3     vmid = lerp(v0,v1,tmid);
			float dmid = dot(plane.xyz(),vmid+qrot(qmid,verts[closest1]))+plane.w;
			*((dmid>0)?&ta:&tb)=tmid;
		}
		*nw0 = plane.xyz();
		*w0  = lerp(v0,v1,ta);
		*wq0 = qlerp(q0,q1,ta);
		*vrtw0= closest1;
		// lets hope that closest1 is still the closest.
		assert(dot(plane.xyz(),*w0+qrot(*wq0,verts[closest1]))+plane.w >0);
		for(unsigned int i=0;i<verts.size();i++) {
			if(dot(plane.xyz(),*w0+qrot(*wq0,verts[i]))+plane.w <=0){
				// call this function again with shorter
				float3 vshort     = *w0;  // shortened
				float4 qshort = *wq0; // shortened
				int rc= PortionUnder(plane,verts,v0,vshort,nv0,vrtv0,q0,qshort,w0,w1,nw0,vrtw0,wq0,wq1);
				assert(rc);
				return rc;
			}
		}
	}
	if(under1) {
		*wq1 = q1;
		*w1  = v1;
	}
	else {
		assert(under0);
		if(closest0==-1) {
			// yes this code already exists above but it wasn't called // FIXME
			for(unsigned int i=0;i<verts.size();i++) {
				verts0.push_back(qrot( q0,verts[i]));
			}
			closest0 = maxdir(verts0,-plane.xyz());
		}
		float ta=0.0f,tb=1.0f;
		for(unsigned int i=0;i<5;i++) {
			float tmid = (ta+tb)/2.0f;
			float4     qmid = qlerp(q0,q1,tmid);
			float3     vmid = lerp(v0,v1,tmid);
			float dmid = dot(plane.xyz(), vmid + qrot(qmid, verts[closest0])) + plane.w;
			*((dmid<0)?&ta:&tb)=tmid;
		}
		*w1  = lerp(v0,v1,tb);
		*wq1 = qlerp(q0,q1,tb);
		// lets hope that closest0 is still the closest.
		//assert(dot(plane.normal,*w1+*wq1*verts[closest0])+plane.dist <0);
	}
	return 1;
}


int HitCheckConvex(std::vector<float3> &verts,BSPNode *node,int solid,
				   float3 v0,float3 v1,float3 *impact,
				   const float4 &q0, const float4 &_q1, float4 *impactq, const float3 &nv0, const int vrtv0) {
	assert(node);
	if( node->isleaf ){
		if(node->isleaf==UNDER ){
			if(impact)  *impact = v0;
			if(impactq) *impactq= q0;
			HitCheckImpactNormal = nv0;
			HitCheckVertexHit    = vrtv0;
		}
		return (node->isleaf==UNDER);
	}

	float3 w0,w1,nw0;
    float4 wq0(0, 0, 0, 1), wq1(0,0,0,1);
	float4 q1 = _q1;
	int vrtw0;
	int hit=0;
	if(PortionUnder(float4(node->xyz(),node->w),verts,v0,v1,nv0,vrtv0,q0,q1,&w0,&w1,&nw0,&vrtw0,&wq0,&wq1)) {
		hit |= HitCheckConvex(verts,node->under,1, w0,w1,&v1, wq0,wq1,&q1,nw0,vrtw0);
	}
	if(PortionUnder(float4(-node->xyz(),-node->w),verts,v0,v1,nv0,vrtv0,q0,q1,&w0,&w1,&nw0,&vrtw0,&wq0,&wq1)) {
		hit |= HitCheckConvex(verts,node->over,0, w0,w1,&v1, wq0,wq1,&q1,nw0,vrtw0);
	}
	if(hit) {
		*impact = v1;
		*impactq= q1;
	}
	return hit;
}



