//
// Rigidbody Physics Stuff
// Sequential Iterative Impulse approach
// (c) Stan Melax 1998-2008   bsd licence
// with some recent minor code cleanups 2011
// in process of packing the tech into a single header file 2014
//
// Feel free to use and/or learn from the following code.  Code provided as is.
// Author assumes no obligation for correctness of support.
//
// What's here is typical of a small basic 3D physics engine.  
// Primary references include Baraff&Witkin's notes and many threads on Erwin's bullet website.
// I might do some things slightly different than other engines such as
// using a runge-kutta integrator for the position updates so i can preserve angular momentum
// instead of keeping angular spin constant.  
// No broadphase - i was only doing small demos exploring other concepts at this point.
// 
// I've been working on this sort of tech for a while often just implementing what I need at the time.
// i.e. this and related modules could be characterized as a personal research sandbox.
// I try to keep code minimal yet have some degree of generality (without "overdesign")
// and have the code do everthing I need.  Not claiming the architecture is yet perfect.
// This module maintains a number of rigidbodies and implements the update solver.
//

#include <stdio.h>
#include <float.h>
#include "physics.h"
#include "wingmesh.h"
#include "gjk.h"

#define EXPORTVAR(V)

void Line(const float3 &, const float3 &, const float3 &color_rgb){};

inline float3 safenormalize(const float3 &v) { return (v == float3(0,0,0)) ? float3(0, 0, 1) : normalize(v); }
inline float clamp(float a, float mn = 0.0f, float mx = 1.0f) { return std::min(std::max(a, mn), mx); }

inline float3 operator*(const float3x3 &m, const float3& v) { return mul(m, v); }

float   physics_deltaT=(1.0f/60.0f);



extern float3 HitCheckImpactNormal;



float  physics_restitution=0.4f;  // coefficient of restitution
float3 gravity(0,0,-10.0f);
float  physics_coloumb=0.6f; // used as default friction value and for static geometry    
EXPORTVAR(physics_restitution);
EXPORTVAR(physics_coloumb);  // between 0 and 1


float physics_biasfactorjoint = 0.3f;
float physics_biasfactorpositive = 0.3f;
float physics_biasfactornegative = 0.3f;
EXPORTVAR(physics_biasfactorjoint);
EXPORTVAR(physics_biasfactorpositive);
EXPORTVAR(physics_biasfactornegative);
float physics_falltime_to_ballistic=0.2f;
EXPORTVAR(physics_falltime_to_ballistic);

// global list of physics objects
std::vector<RigidBody *> g_rigidbodies;



inline float4 DiffQ(const float4 &orientation, const float3x3 &tensorinv, const float3 &angular)
{
	float4 s_orientation_normed = normalize(orientation);
	float3x3 s_orientation_matrix = qgetmatrix(s_orientation_normed);
	float3x3 Iinv = mul(s_orientation_matrix, tensorinv, transpose(s_orientation_matrix));
	float3 halfspin = mul(Iinv, angular) * 0.5f;
	return qmul(float4(halfspin.x, halfspin.y, halfspin.z, 0), s_orientation_normed);
}

inline float4 rkupdateq(const float4 &s, const float3x3 &tensorinv, const float3 &angular, float dt)
{
	float4 d1 = DiffQ(s, tensorinv, angular);
	float4 s2(s + d1*(dt / 2));

	float4 d2 = DiffQ(s2, tensorinv, angular);
	float4 s3(s + d2*(dt / 2));

	float4 d3 = DiffQ(s3, tensorinv, angular);
	float4 s4(s + d3*(dt));

	float4 d4 = DiffQ(s4, tensorinv, angular);
	return normalize(s + d1*(dt / 6) + d2*(dt / 3) + d3*(dt / 3) + d4*(dt / 6));
}



void ApplyImpulse(RigidBody *rb,float3 r,float3 n,float impulse) 
{
	rb->momentum = rb->momentum + (n*(impulse));
	float3 drot = cross(r,(n*impulse));
	rb->rotation = rb->rotation + drot ;

	rb->spin = rb->Iinv * rb->rotation; // recompute auxilary variable spin
	//assert(magnitude(rb->spin) <10000);
}

void ApplyImpulseIm(RigidBody *rb,float3 r,float3 n,float impulse) 
{
	float3 drot = cross(r,(n*impulse));
	rb->orientation = normalize(rb->orientation + DiffQ(rb->orientation,rb->tensorinv_massless * rb->massinv ,drot)*physics_deltaT);
	//rb->Iinv = transpose(qgetmatrix(rb->orientation)) * rb->tensorinv * (qgetmatrix(rb->orientation)); // recompute Iinv
	rb->Iinv = mul(qgetmatrix(rb->orientation), rb->tensorinv_massless * rb->massinv, transpose(qgetmatrix(rb->orientation)));
	rb->spin = rb->Iinv * rb->rotation; // recompute auxilary variable spin
	rb->position +=(n*(impulse)) * rb->massinv * physics_deltaT;
	//assert(magnitude(rb->spin) <10000);
}


/*
Spring *CreateSpring(RigidBody *a,float3 av,RigidBody *b,float3 bv,float k) {
	Spring *s  = new Spring();
	s->bodyA   = a;
	s->anchorA = av;
	s->bodyB   = b;
	s->anchorB = bv;
	s->k       = k;
	if(a)a->springs.push_back(s);
	if(b)b->springs.push_back(s);
	return s;
}
void DeleteSpring(Spring *s) {
    if (s->bodyA && Contains(s->bodyA->springs,s)) {
		Remove(s->bodyA->springs, s);
	}
    if (s->bodyB && Contains(s->bodyB->springs,s)) {
		Remove(s->bodyB->springs, s);
	}
	delete s;
}
*/

class Limit
{
public:
	RigidBody *rb0;
	RigidBody *rb1;
	Limit(RigidBody *_rb0,RigidBody *_rb1):rb0(_rb0),rb1(_rb1){}
	virtual ~Limit(){}
	virtual void Finalize(){}
};


class LimitAngular : public Limit
{
public:
	float  limitangle;
	float3 axis;  // world-space
	float  torque;  // rotational impulse
	float  targetspin;
	float  mintorque;
	float  maxtorque;
	LimitAngular():Limit(NULL,NULL){}
	LimitAngular(RigidBody *_rb0,RigidBody *_rb1=NULL,const float3 &_axis=float3(0,0,1),float _targetspin=0.0f,float _mintorque=-FLT_MAX,float _maxtorque=FLT_MAX);
	void Iter();
	void PostIter(){targetspin=(mintorque<0)?0:std::min(targetspin,0.0f);}  // not zero since its ok to let it fall to the limit;
};

LimitAngular::LimitAngular(RigidBody *_rb0,RigidBody *_rb1,const float3 &_axis,float _targetspin,float _mintorque,float _maxtorque):Limit(_rb0,_rb1),axis(_axis),targetspin(_targetspin),mintorque(_mintorque),maxtorque(_maxtorque)
{
	torque=0.0f;
}
void LimitAngular::Iter()
{
	if(targetspin==-FLT_MAX) return;
	float currentspin = ((rb1)?dot(rb1->spin,axis):0.0f) - ((rb0)?dot(rb0->spin,axis):0.0f) ;  // how we are rotating about the axis 'normal' we are dealing with
	float dspin = targetspin - currentspin;  // the amount of spin we have to add to satisfy the limit.
	float spintotorque = 1.0f / (  ((rb0)?dot(axis,mul(rb0->Iinv,axis)):0.0f) + ((rb1)?dot(axis,mul(rb1->Iinv,axis)):0.0f)  );
	float dtorque = dspin * spintotorque ;  // how we have to change the angular impulse
	dtorque = std::min(dtorque,maxtorque*physics_deltaT-torque); // total torque cannot exceed maxtorque
	dtorque = std::max(dtorque,mintorque*physics_deltaT-torque); // total torque cannot fall below -maxtorque
	if(rb0)
	{
		rb0->rotation += axis*-dtorque;  // apply impulse
		rb0->spin = rb0->Iinv * rb0->rotation; // recompute auxilary variable spin
	}
	if(rb1)
	{
		rb1->rotation += axis*dtorque;  // apply impulse
		rb1->spin = rb1->Iinv * rb1->rotation; // recompute auxilary variable spin
	}
	torque += dtorque;
}

class LimitLinear : public Limit
{
 public:
	float3 position0;  // position of contact on rb0 in rb0 local space
	float3 position1;  // position of contact on rb1 in rb1 local space
	float3 normal;     // direction in world in which limit is applied
	float  targetspeed; 
	float  targetspeednobias;
	float  minforce;
	float  maxforce;
	int    friction_master;   // true if a friction limit
	LimitLinear *master;
	float  impulsesum;
	void Iter();
	void PostIter();
	virtual void Finalize();
	LimitLinear():Limit(NULL,NULL){}
	LimitLinear(RigidBody *_rb0,RigidBody *_rb1,const float3 &_position0,const float3 &_position1,
	            const float3 &_normal=float3(0,0,1),float _targetspeed=0.0f,float _targetspeednobias=0.0f,float _maxforce=FLT_MAX,float _minforce=-FLT_MAX);
};

LimitLinear::LimitLinear(RigidBody *_rb0,RigidBody *_rb1,const float3 &_position0,const float3 &_position1,const float3 &_normal,float _targetspeed,float _targetspeednobias,float _maxforce,float _minforce):
		Limit(_rb0,_rb1),
		position0(_position0),
		position1(_position1),
		normal(_normal),
		targetspeed(_targetspeed),
		targetspeednobias(_targetspeednobias),
		maxforce(_maxforce),
		minforce(_minforce),
		friction_master(0),
		master(NULL)
{
	assert(maxforce>minforce);
	impulsesum=0.0f;
	impulsesum=0;
}
float showimpulse=0.0f;
EXPORTVAR(showimpulse);
void LimitLinear::PostIter()
{
	if(showimpulse>0.0f)
	{
		Pose p = (this->rb0) ? this->rb0->pose() : Pose() ;
		Line(p*this->position0-this->normal*impulsesum * showimpulse,p*position0+this->normal* impulsesum * showimpulse, float3(1,0.25f,0));
	}
	targetspeed=std::min(targetspeed,targetspeednobias); 
}

void LimitLinear::Finalize()
{
}
void LimitLinear::Iter()  
{
// Want to consolidate what is currently in different structures to make a consistent low level api.  
// Immediate mode style works nicely for higher level joint objects such as limit cone.  
// With an immediate mode api there needs to be a mechanism for optional feedback to higher level for warmstarting and what forces being applied eg sound effects or reading teeter's weight
	if(friction_master)
		minforce = -(  maxforce = std::max( ((rb0)?rb0->friction:0),((rb1)?rb1->friction:0) ) * ((this)+friction_master)->impulsesum / physics_deltaT   );
	float3 r0  = (rb0) ? qrot(rb0->orientation , position0) :  position0;
	float3 r1  = (rb1) ? qrot(rb1->orientation , position1) :  position1;
	float3 v0  = (rb0) ? cross(rb0->spin,r0) + rb0->momentum*rb0->massinv : float3(0,0,0); // instantaneioius linear velocity at point of constraint
	float3 v1  = (rb1) ? cross(rb1->spin,r1) + rb1->momentum*rb1->massinv : float3(0,0,0); 
	float  vn  = dot(v1-v0,normal);  // velocity of rb1 wrt rb0
	float impulsen = -targetspeed - vn;
	float impulsed =  ((rb0)? rb0->massinv + dot( cross((rb0->Iinv*cross(r0,normal)),r0),normal):0) 
	                + ((rb1)? rb1->massinv + dot( cross((rb1->Iinv*cross(r1,normal)),r1),normal):0) ;
	float impulse = impulsen/impulsed;
	impulse = std::min( maxforce*physics_deltaT-impulsesum,impulse);
	impulse = std::max( minforce*physics_deltaT-impulsesum,impulse);
	if(rb0) ApplyImpulse(rb0,r0,normal ,-impulse ); 
	if(rb1) ApplyImpulse(rb1,r1,normal , impulse ); 
	impulsesum += impulse;
}


void createdrive(std::vector<LimitAngular> &Angulars_out, RigidBody *rb0,RigidBody *rb1,float4 target,float maxtorque)
{
	//target=Quaternion(0,0,0,1.0f);  // TEST
//	Quaternion r1 = rb1->orientation ;
//	Quaternion r0 = qconj(rb0->orientation*target);
//	if(dot(r1,r0)>0.0f) r1 *= -1.0f;
//	Quaternion dq = r1*r0  ;  // quat that takes a direction in r0+target orientation into r1 orientation
	float4 q0 = (rb0)?rb0->orientation:float4(0,0,0,1);
	float4 q1 = (rb1)?rb1->orientation:float4(0,0,0,1);
    float4 dq = qmul(q1, qconj(qmul(q0, target)));  // quat that takes a direction in r0+target orientation into r1 orientation
	if(dq.w<0) dq=-dq;
	float3 axis     = safenormalize(dq.xyz()); // dq.axis();
	float3 binormal = Orth(axis);
	float3 normal   = cross(axis,binormal);
	Angulars_out.push_back( LimitAngular(rb0,rb1,axis,-physics_biasfactorjoint*(acosf(clamp(dq.w,-1.0f,1.0f))*2.0f)/physics_deltaT,-maxtorque,maxtorque) );
	Angulars_out.push_back( LimitAngular(rb0,rb1,binormal,0,-maxtorque,maxtorque) );
	Angulars_out.push_back( LimitAngular(rb0,rb1,normal  ,0,-maxtorque,maxtorque) );
}

LimitLinear limitlinearaxis(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
{
	return LimitLinear( rb0,rb1,p0,p1,axisw, dot( ((rb1)?rb1->pose()*p1:p1) - ((rb0)?rb0->pose()*p0:p0) ,axisw)/physics_deltaT,0.0f,maxforce,minforce);
}
void createnail(std::vector<LimitLinear> &Linears_out,RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1)
{
	float3 d = ((rb1)?rb1->pose()*p1:p1) - ((rb0)?rb0->pose()*p0:p0);
	Linears_out.push_back( LimitLinear(rb0,rb1,p0,p1,float3(1.0f,0.0f,0.0f),d.x / physics_deltaT ,0.0f) );
	Linears_out.push_back( LimitLinear(rb0,rb1,p0,p1,float3(0.0f,1.0f,0.0f),d.y / physics_deltaT ,0.0f) );
	Linears_out.push_back( LimitLinear(rb0,rb1,p0,p1,float3(0.0f,0.0f,1.0f),d.z / physics_deltaT ,0.0f) );
}

void createangularlimitsW(std::vector<LimitAngular> &Angulars_out, RigidBody *rb0, const float4 &jb0, RigidBody *rb1, const float4 &jf1, const float3& _jointlimitmin, const float3& _jointlimitmax)
{
	// used for hard limits such as joint ranges.
	// an attempt to specify joint limits a bit better using arbitrary axis limits  
	float3 jmin = _jointlimitmin * 3.14f/180.0f; 
	float3 jmax = _jointlimitmax * 3.14f/180.0f; 

	if(jmin.x==0 && jmax.x==0 && jmin.y==0 && jmax.y==0 && jmin.z<jmax.z)
	{
		float4 cb = normalize(float4(0,-1,0,1));
		return createangularlimitsW(Angulars_out,rb0,  qmul(jb0,cb) , rb1, qmul(jf1,cb) , float3(_jointlimitmin.z,0,0),float3(_jointlimitmax.z,0,0)); 
	}
	float4 r = qmul(qconj(jb0), jf1);
	float4 s = RotationArc(float3(0, 0, 1.0f), qzdir(r));
	float4 t = qmul(qconj(s), r);
	
	if(jmax.x==jmin.x)
	{
		Angulars_out.push_back(LimitAngular(rb0,rb1, qxdir(jf1), 2 * (-s.x+sin(jmin.x/2.0f)) /physics_deltaT));   
	}
	else if(jmax.x-jmin.x< 360.0f * 3.14f/180.0f )
	{
		Angulars_out.push_back(LimitAngular(rb0,rb1, qxdir(jf1), 2 * (-s.x+sin(jmin.x/2.0f)) /physics_deltaT,0));   
		Angulars_out.push_back(LimitAngular(rb0,rb1,-qxdir(jf1), 2 * ( s.x-sin(jmax.x/2.0f)) /physics_deltaT,0));  
	}
	if(jmax.y==jmin.y)
	{
		Angulars_out.push_back(LimitAngular(rb0,rb1,qydir(jf1),physics_biasfactorjoint * 2 * (-s.y+jmin.y) /physics_deltaT)); // btw '2' is because of quat angle is t/2
	}
	else
	{
		Angulars_out.push_back(LimitAngular(rb0,rb1, qydir(jf1), 2 * (-s.y+sin(jmin.y/2.0f)) /physics_deltaT,0));   
		Angulars_out.push_back(LimitAngular(rb0,rb1,-qydir(jf1), 2 * ( s.y-sin(jmax.y/2.0f)) /physics_deltaT,0));  
	}
	Angulars_out.push_back(LimitAngular(rb0,rb1,qzdir(jf1),physics_biasfactorjoint * 2 * -t.z /physics_deltaT)); // btw '2' is because of quat angle is t/2
}

void createangularlimits(std::vector<LimitAngular> &Angulars_out, RigidBody *rb0, RigidBody *rb1, const float4 &_jointframe, const float3& _jointlimitmin, const float3& _jointlimitmax)
{
	return createangularlimitsW(Angulars_out, rb0, (rb0) ? qmul(rb0->orientation, _jointframe) : _jointframe, rb1, (rb1) ? rb1->orientation : float4(0, 0, 0, 1), _jointlimitmin, _jointlimitmax);
}


LimitAngular ConeLimit(RigidBody* rb0,const float3 &n0,RigidBody* rb1,const float3 &n1,float limitangle_degrees)  // a hinge is a cone with 0 limitangle
{
	// note only limiting 1dof, so if limitangle is close to 0 then there is not anything constraining it on 2nd free axis.  
	assert(rb1);
	int equality = (limitangle_degrees==0);
	float3 a0 = (rb0)? qrot(rb0->orientation,n0) : n0;
	float3 a1 = (rb1)? qrot(rb1->orientation,n1) : n1;
	float3 axis = safenormalize(cross(a1,a0));  // could this be an issue if a0 and a1 are collinear and we start enforcing a two sided constraint with limitangle 0 on a bogus axis
	float rbangle = acosf(clamp(dot(a0,a1),0.0f,1.0f));
	float dangle = rbangle - (limitangle_degrees)*3.14f/180.0f;
	float targetspin = ((equality)?physics_biasfactorjoint:1.0f) * dangle / physics_deltaT; // enforce all the way to limit if one-sided
	return LimitAngular( rb0, rb1, axis, targetspin ,  (limitangle_degrees>0.0f)? 0 : -FLT_MAX,  FLT_MAX );
}


Shape::Shape(RigidBody *_rb, WingMesh local_geometry) :rb(_rb),local_geometry(local_geometry)
{
}
Shape::~Shape()
{
}


RigidBody::RigidBody(std::vector<WingMesh*> &_meshes,const float3 &_position) : orientation_next(0,0,0,1), orientation_old(0,0,0,1), orientation_start(0,0,0,1)
{
	position_start=position_old=position_next=position=_position;
	rest=0;   
	hittime=0.0f;
	g_rigidbodies.push_back(this);
	collide=(_meshes.size())?3:0;
	resolve=(_meshes.size())?1:0;
	usesound = 0;
	mass=1;
	gravscale=1.0f;
	damping = 0.0f;  // rigidbody::damping   currently i just take the max of this and the global damping
	friction = physics_coloumb;

	std::vector<WingMesh*> shapemeshes;
    for (auto m : _meshes)
		shapemeshes.push_back(& (shapes.push_back(new Shape(this,*m)),shapes.back())->local_geometry);


	com = CenterOfMass(shapemeshes);
	position += com;
	position_start=position_old=position_next=position;
	for(auto & s : shapemeshes)
		WingMeshTranslate(s,-com);
	//BSPTranslate(bsp,-com);

	float3x3 tensor = Inertia(shapemeshes);
	massinv = 1.0f / mass;
	Iinv = (tensorinv_massless = inverse(tensor))  * massinv;

	std::vector<float3> allverts;
	for (auto &m : shapemeshes)
		allverts.insert(allverts.end(), m->verts.begin(), m->verts.end());
	radius = magnitude(*std::max_element(allverts.begin(), allverts.end(), [](const float3 &a, const float3 &b){return dot(a, a) < dot(b, b); }));
	std::tie(bmin, bmax) = Extents(allverts);
}


RigidBody::~RigidBody()
{
	// I still have other memory leaks here

//	while(springs.size()) {
//		DeleteSpring(springs[springs.size()-1]);
//	}
	g_rigidbodies.erase(std::find(g_rigidbodies.begin(), g_rigidbodies.end(), this));  // 	Remove(g_rigidbodies, this);
	for(unsigned int i=0;i<shapes.size();i++)
	{
		delete shapes[i];
	}
	shapes.clear();
}



float springk=12.56f; // spring constant position - default to about half a second for unit mass weight
float springr=1.0f;   // spring constant rotation 
float physics_damping=0.15f;  // 1 means critically damped,  0 means no damping
EXPORTVAR(springk);
EXPORTVAR(physics_damping);
float3 SpringForce(State &s,const float3 &home,float mass) {
	return (home-s.position)*springk +   // spring part
		   (s.momentum/mass)* (-sqrtf(4 * mass * springk) * physics_damping);    // physics_damping part
}




int AddSpringForces(RigidBody *rba,const State &state,float3 *force,float3 *torque)
{

	return 1;
}



float physics_driftmax = 0.03f;
EXPORTVAR(physics_driftmax);
float showcontactnormal=0.1f;
EXPORTVAR(showcontactnormal);



int suspendpairupdate=0;
EXPORTVAR(suspendpairupdate);

/*
int HitCheck(std::vector<BSPNode *> bsps,const float3 &v0,float3 v1,float3 *impact)
{
	int hit=0;
	for(int i=0;i<bsps.size();i++)
		hit |= HitCheck(bsps[i],1,v0,v1,&v1);
	if(hit && impact) 
		*impact=v1;
	return hit;
}


void CCD(RigidBody *rb,const std::vector<BSPNode*> &area_bsps)
{
	if(!rb->resolve)
		return;
	float3 hitpoint;
	for(int i=0;i<area_bsps.size();i++)
        if(HitCheck(area_bsps,rb->position_old,rb->position_old,&hitpoint)) return; // we were inside a bad spot last frame anyways.
	int k=0;
	while(k<4 && HitCheck(area_bsps,rb->position_old,rb->position_next,&hitpoint)) 
	{
		k++;
		extern float3 HitCheckImpactNormal;  // global variable storing the normal of the last hit 
		rb->position = rb->position_next = PlaneProject(Plane(HitCheckImpactNormal,-dot(hitpoint,HitCheckImpactNormal)-0.001f),rb->position_next);
		rb->momentum =  float3(0,0,0); //  rb->mass * (rb->position_next-rb->position_old)/physics_deltaT;
	}
	if(k==4)
	{
		rb->position_next = rb->position = rb->position_old;
		rb->momentum = float3(0,0,0);
	}
}

*/

float showspin=0.0f;

void rbinitvelocity(RigidBody *rb) 
{
	// gather weak forces being applied to body at beginning of timestep
	// forward euler update of the velocity and rotation/spin 
	rb->old.position    = rb->position;
	rb->old.orientation = rb->orientation;
	float dampleftover = powf((1.0f-std::max(rb->damping,physics_damping)),physics_deltaT);
	rb->momentum *= dampleftover; 
	rb->rotation *= dampleftover;
	State s = rb->state(); // (State) *rb;
	float3 force;
	float3 torque;
	force  = gravity*rb->mass*rb->gravscale;
	torque = float3(0,0,0);
	AddSpringForces(rb,s,&force,&torque);

	rb->momentum += force*physics_deltaT; 
	rb->rotation += torque*physics_deltaT;
	//rb->Iinv = transpose(qgetmatrix(rb->orientation)) * rb->tensorinv * (qgetmatrix(rb->orientation));
	rb->Iinv = mul(qgetmatrix(rb->orientation), rb->tensorinv_massless * rb->massinv, transpose(qgetmatrix(rb->orientation)));
	rb->spin = rb->Iinv * rb->rotation;
}

int physics_euler_integration=0;
EXPORTVAR(physics_euler_integration);

void rbcalcnextpose(RigidBody *rb) 
{
	if(!rb->resolve) return;
	// after an acceptable velocity and spin are computed, a forward euler update is applied ot the position and orientation.
	rb->position_next     = rb->position + rb->momentum * rb->massinv * physics_deltaT; 
	rb->orientation_next  = (physics_euler_integration)? normalize(rb->orientation+DiffQ(rb->orientation,rb->tensorinv_massless * rb->massinv,rb->rotation )*physics_deltaT) :rkupdateq(rb->orientation,rb->tensorinv_massless * rb->massinv,rb->rotation,physics_deltaT);
	for(int i=0;i<3;i++)
		if(rb->orientation_next[i]< FLT_EPSILON/4.0 && rb->orientation_next[i]> -FLT_EPSILON/4.0 ) 
			rb->orientation_next[i]=0.0f; // dont keep needless precision, somehow this causes serious slowdown on Intel's centrino processors perhaps becuase as extra shifting is required during multiplications
}

void rbscalemass(RigidBody *rb,float s)
{
	rb->mass      *= s;
	rb->momentum  *= s;
	rb->massinv   *= 1.0f/s;
	rb->rotation  *= s;
	rb->Iinv      *= 1.0f/s;
}

void rbupdatepose(RigidBody *rb) 
{
	if(!rb->resolve) return;
	// after an acceptable velocity and spin are computed, a forward euler update is applied ot the position and orientation.
	rb->position_old    = rb->position;  // should do this at beginning of physics loop in case someone teleports the body.
	rb->orientation_old = rb->orientation;
	rb->position        = rb->position_next ; 
	rb->orientation     = rb->orientation_next;
	rb->Iinv = mul(qgetmatrix(rb->orientation), rb->tensorinv_massless * rb->massinv, transpose(qgetmatrix(rb->orientation)));
	rb->spin = rb->Iinv * rb->rotation;
	if(showspin>0.0f)
	{
		extern void Line(const float3 &,const float3 &,const float3 &color_rgb);
		Line(rb->position,rb->position+safenormalize(rb->spin)*showspin ,float3(0,1,1));
		Line(rb->position,rb->position+safenormalize(rb->rotation)*showspin ,float3(1,0,1));
	}
}







//----------------  new contact code -------------
float physics_contactpatchjiggle=2.0f;// rotations in degrees   ideally this should be a shear
EXPORTVAR(physics_contactpatchjiggle);

struct Patch
{
	gjk_implementation::Contact hitinfo[5];
	gjk_implementation::Contact &operator[](int i){ return hitinfo[i]; }
	int count;
	operator bool(){return (count != 0);}
	Patch():count(0){}
};


template<class COLLIDABLE>
Patch ContactPatch(Shape *s0,const COLLIDABLE *_s1)
{
	// using the separated convention right now   points from s1 to s0   would have prefered point from s0 to s1
	//
	Patch hitinfo;

	hitinfo[0] = Separated(s0, _s1);


	if (hitinfo[0].separation>physics_driftmax)
		return hitinfo;  // too far away to care

	//float3 n0 = qrot(qconj(s0->Orientation()) , n);
	//float3 n1 = qrot(qconj(s1->Orientation()) ,-n);
	//int f0=best(s0->local_geometry->faces,[&n0](const Plane &p)  {return  dot(n0,p.normal());} );
	//int f1=best(s1->local_geometry->faces,[&n1](const Plane &p)  {return  dot(n1,p.normal());} );
	//int p0=best(s0->local_geometry->verts,[&n0](const float3 &v) {return  dot(n0,v); } );
	//int p1=best(s1->local_geometry->verts,[&n1](const float3 &v) {return  dot(n1,v); } );
	//std::vector<float3> v0 = WingMeshFaceVerts(s0->local_geometry,f0);
	//std::vector<float3> v1 = WingMeshFaceVerts(s1->local_geometry,f1);
	//std::vector<float3> v(2*(v0.size()+v1.size()));

	int &hc=hitinfo.count;
	hitinfo[hc].C[0]=s0;
	hitinfo[hc].C[1]=NULL;
	const float3 &n=hitinfo[hc].normal;  // since this needs to be flipped than what was in my head today
	hitinfo[hc].p0 = (s0)?qrot(qconj(s0->rb->orientation) , hitinfo[hc].p0w - s0->rb->position) : hitinfo[hc].p0w;
	hitinfo[hc].p1 =  hitinfo[hc].p1w;
	hc=1;

	Pose s0_pose_save = s0->rb->pose();
	s0->rb->position += n*0.1f;  // move s0 away from s1
	float3 tangent = Orth(n);
	float3 bitangent = cross(n,tangent); // tangent X bitangent == n
	float3 rolls[4] = { tangent , bitangent, -tangent , -bitangent };
	for(int i=0;i<4;i++)
	{
		float4 jiggle = normalize(float4( rolls[i]*sinf(3.14f/180.0f*(physics_contactpatchjiggle)),1));
		s0->rb->orientation =  qmul(jiggle, s0_pose_save.orientation);
		s0->rb->position    =  hitinfo[0].p0w  +  qrot(jiggle ,  s0_pose_save.position - hitinfo[0].p0w  );
		//Separated(s0,_s1,hitinfo[hc],1); 
		hitinfo[hc]=Separated(s0, _s1);
		hitinfo[hc].normal = n;// all parallel to the initial separating plane;
		hitinfo[hc].p0 = (s0)?qrot(qconj(s0->rb->orientation) , hitinfo[hc].p0w - s0->rb->position) : hitinfo[hc].p0w;
		hitinfo[hc].p1 =  hitinfo[hc].p1w;   // dont know if its a 'shape' or not - let caller figure that out.
		int match=0;
		for(int j=0;!match && j<hc;j++)
			match = magnitude(hitinfo[hc].p0w-hitinfo[j].p0w) < 0.05f || magnitude(hitinfo[hc].p1w-hitinfo[j].p1w) < 0.05f ;
		if(match)
			continue;
		hitinfo[hc].p0w = s0_pose_save * hitinfo[hc].p0;
		hitinfo[hc].C[0]= s0;
		hitinfo[hc].C[1] = NULL; // currently dont know if its a shape
		hc++;
	}
	s0->rb->pose() = s0_pose_save;

	for(int i=0;i<hc;i++)
	{
		Line(hitinfo[i].p0w,hitinfo[i].p1w,float3(0.7f,1.0f,0));
	}
	return hitinfo;
}

void FindShapeWorldContacts(std::vector<gjk_implementation::Contact> &contacts_out, const std::vector<RigidBody*>& rigidbodies, const std::vector<WingMesh*>& cells)
{
	//foreach(rigidbodies,[&contacts_out](RigidBody* rb){ foreach(rb->shapes,[&contacts_out](Shape *s){FindShapeWorldContacts(s,contacts_out);});});  // compiler failed to make this work
	//foreach(rigidbodies,[&contacts_out](RigidBody* rb){std::vector<Contact> &c2=contacts_out ;foreach(rb->shapes,[&](Shape *s){FindShapeWorldContacts(s,c2);});}); // works
	//auto f = [&](Shape *s){FindShapeWorldContacts(s,contacts_out);};
	//foreach(rigidbodies,[f](RigidBody* rb){ foreach(rb->shapes,f);});
	for(unsigned int i=0;i<rigidbodies.size();i++)
	 for(unsigned int j=0 ; j<rigidbodies[i]->shapes.size() ; j++)  // foreach rigidbody shape
	{
		Shape *shape = rigidbodies[i]->shapes[j];
		if(!(shape->rb->collide&1)) continue;
		//auto cells=ProximityCells(shape,area_bsps[i],physics_driftmax);
		for (auto & cell : cells) //  int i = 0; i < cells.size(); i++)
		{
			if(!cell) continue; // it was null, so dont collide. /// This is probably a bug!!
			Patch hitinfo = ContactPatch(shape,cell);
			for(int i=0;i<hitinfo.count;i++)
			{
				contacts_out.push_back(hitinfo[i]);
			}
		}
	}
}

void FindShapeShapeContacts(std::vector<gjk_implementation::Contact> &contacts_out_append, const std::vector<RigidBody*> & rigidbodies)  // Dynamic-Dynamic contacts
{
	unsigned int i_rb,j_rb;
	for(i_rb=0;i_rb<rigidbodies.size();i_rb++)
	  for(j_rb=i_rb+1;j_rb<rigidbodies.size();j_rb++)
	{
		RigidBody *rb0 = rigidbodies[i_rb];
		RigidBody *rb1 = rigidbodies[j_rb];
		if(!(rb0->collide&2)) continue; 
		if(!(rb1->collide&2)) continue;
		if(magnitude(rb1->position - rb0->position) > rb0->radius + rb1->radius) continue;
		int ignore=0;
		for(unsigned int ign=0;ign<rb0->ignore.size();ign++)
		{
			ignore |= (rb0->ignore[ign]==rb1);
		}
		if(ignore)continue;
		for(unsigned int i_s=0 ; i_s<rb0->shapes.size() ; i_s++)
		 for(unsigned int j_s=0 ; j_s<rb1->shapes.size() ; j_s++)
		{
			Shape *s0=rb0->shapes[i_s];
			Shape *s1=rb1->shapes[j_s];
			assert(s0->rb!=s1->rb);
			Patch hitinfo = ContactPatch(s0,s1);
			for(int i=0;i<hitinfo.count;i++)
			{
				hitinfo[i].C[1] = s1;
				hitinfo[i].p1 = s1->rb->pose().Inverse() * hitinfo[i].p1w;
				contacts_out_append.push_back(hitinfo[i]);
			}
		}
	}
}

void ConstrainContact(std::vector<LimitLinear> &Linears_out, const gjk_implementation::Contact &c)
{
	RigidBody *rb0 = dynamic_cast<Shape*>(c.C[0]) ?  dynamic_cast<Shape*>(c.C[0])->rb  : NULL;
	RigidBody *rb1 = dynamic_cast<Shape*>(c.C[1]) ?  dynamic_cast<Shape*>(c.C[1])->rb  : NULL;

	float3  n =c.normal;
	float3 v,vn,r0,v0,r1,v1;
	r0 = (rb0) ? c.p0w - rb0->position : float3(0,0,0);  
	v0 = (rb0) ? cross(rb0->spin,r0) + rb0->momentum*rb0->massinv : float3(0,0,0); // instantaneioius linear velocity at point of collision
	r1 = (rb1) ? c.p1w - rb1->position : float3(0,0,0);  
	v1 = (rb1) ? cross(rb1->spin,r1) + rb1->momentum*rb1->massinv : float3(0,0,0); 
	v = v0-v1;  // todo would look cleaner if it was v1-v0.
	vn = n*dot(v,n);
	float minsep = physics_driftmax*0.25f;
	float separation = dot(n,c.p0w-c.p1w);  // todo verify direction here
	float extra = (minsep-separation);
	float bouncevel = 0.0f;
	float targvel = (extra/physics_deltaT)*std::min(1.0f,((extra>0.0f)? physics_biasfactorpositive *physics_deltaT*60 : physics_biasfactornegative*physics_deltaT*60));  // "bias" fudgefactor to move past minseparation
	if(separation<=0)  // attempt to immediately pull completely out of penetration
	{
		targvel += -separation/physics_deltaT * (1.0f - std::min(1.0f, physics_biasfactorpositive*physics_deltaT*60 ));  // ensure we go at least fast enough to stop penetration
	}
	if(dot(v,c.normal) < 0 && magnitude(vn) > magnitude(gravity)*physics_falltime_to_ballistic )
	{
		// take off any contribution of gravity on the current/recent frames to avoid adding energy to the system.
		bouncevel = std::max(0.0f, (magnitude(vn)-magnitude(gravity)*physics_falltime_to_ballistic) * physics_restitution);  // ballistic non-resting contact, allow elastic response
		targvel   = std::max(0.0f,targvel-bouncevel);
	}

	//solver.AddContact(rb0,rb1,c.p0,c.p1, -c.normal, Bounce(bouncevel, targvel), physics_coloumb);
				
				Linears_out.push_back(LimitLinear(rb0,rb1,c.p0,c.p1,   -c.normal,-(targvel+bouncevel),-bouncevel,FLT_MAX,0.0f));
				LimitLinear &lim = Linears_out.back();
				float3 tangent  = Orth(-c.normal);
				float3 binormal = cross(c.normal,tangent);
				LimitLinear fb(rb0,rb1,c.p0,c.p1,   binormal,0,0,0); fb.master=&lim; fb.friction_master=-1;
				LimitLinear ft(rb0,rb1,c.p0,c.p1,   tangent,0,0,0); ft.master=&lim; ft.friction_master=-2;
				Linears_out.push_back(fb);
				Linears_out.push_back(ft);
}
//------------------

int physics_iterations=16;
EXPORTVAR(physics_iterations);
int physics_iterations_post=4;
//EXPORTVAR(physics_iterations_post);

int physics_ccd=1;
EXPORTVAR(physics_ccd);

int physics_enable=1;
EXPORTVAR(physics_enable);

void PhysicsUpdate(std::vector<RigidBody*> &rigidbodies, std::vector<LimitLinear> &Linears, std::vector<LimitAngular> &Angulars, const std::vector<WingMesh*> &wgeom) 
{
	if(!physics_enable) return;

	for(auto &rb:rigidbodies) 
	{
		if(!rb->resolve) continue;
		rbinitvelocity(rb); // based on previous and current force/torque
	}

	std::vector<gjk_implementation::Contact> contacts;
	FindShapeWorldContacts(contacts, rigidbodies, wgeom);
	FindShapeShapeContacts(contacts,rigidbodies);
    for (const auto & c : contacts)
		ConstrainContact(Linears,c);  // currently uses aux variables on the rigidbodies (spin, Iinv) to calculate bounce velocity etc.

	for(int s=0;s<physics_iterations;s++)  // iteration steps
	{
		for(auto &ln  : Linears) 
			ln.Iter();
		for(auto &a : Angulars) 
			a.Iter();
	}

	for(auto rb : rigidbodies)
		rbcalcnextpose(rb); 

	// The objective of this step is to take away any velocity that was added strictly for purposes of reaching the constraint or contact.
	// i.e. we added some velocity to a point to pull it away from something that was 
	// interpenetrating but that velocity shouldn't stick around for the next frame,
	// otherwise we will have lots of jitter from our contacts and occilations from constraints.
	// this is accomplished by clearing the targetvelocities to zero and invoking the solver.
	for (auto &ln : Linears)
		ln.PostIter();
	for (auto &a : Angulars)
		a.PostIter();
	for(int s=0;s<physics_iterations_post;s++)  // iteration steps
	{
		for (auto &ln : Linears)
			ln.Iter();
		for (auto &a : Angulars)
			a.Iter();
	}
	for (auto &ln : Linears)
		ln.Finalize();
	for (auto &a : Angulars)
		a.Finalize();

	for(auto rb : rigidbodies)
		rbupdatepose(rb); 


//	if(physics_ccd) 
//		for(auto rb : rigidbodies)
//			CCD(rb,area_bsps);

}


//--  retained mode API --

std::vector<LimitAngular> Angulars;
std::vector<LimitLinear> Linears;


void addlimitlinearaxis(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
{
	Linears.push_back(  limitlinearaxis(rb0,p0,rb1,p1,axisw, minforce, maxforce)  );
}
void addlimitlinearaxisflow(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
{
	Linears.push_back(limitlinearaxis(rb0, p0, rb1, p1, axisw, minforce, maxforce));
	LimitLinear &lim=Linears.back();
	lim.targetspeednobias=lim.targetspeed;
}
void createnail(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1)
{
	float maxforce = FLT_MAX;
	addlimitlinearaxis(rb0,p0,rb1,p1,float3(1,0,0),-maxforce,maxforce);
	addlimitlinearaxis(rb0,p0,rb1,p1,float3(0,1,0),-maxforce,maxforce);
	addlimitlinearaxis(rb0,p0,rb1,p1,float3(0,0,1),-maxforce,maxforce);
	//return createnail(Linears,rb0,p0,rb1,p1);
}
void createdrive(RigidBody *rb0,RigidBody *rb1,float4 target,float maxtorque)
{
	return createdrive(Angulars,rb0,rb1,target,maxtorque);
}
void createangularlimits(RigidBody *rb0,RigidBody *rb1, const float4 &_jointframe, const float3& _jointlimitmin, const float3& _jointlimitmax)
{
	createangularlimits(Angulars,rb0,rb1,_jointframe,_jointlimitmin,_jointlimitmax);
}
void createconelimit(RigidBody* rb0,const float3 &n0,RigidBody* rb1,const float3 &n1,float limitangle_degrees)  // a hinge is a cone with 0 limitangle
{
	Angulars.push_back(ConeLimit(rb0,n0,rb1,n1,limitangle_degrees));
}
void PhysicsUpdate(const std::vector<WingMesh*> &wgeom)
{
	//GetEnvBSPs(area_bsps);
	PhysicsUpdate(g_rigidbodies,Linears,Angulars,wgeom);
	Linears.clear();
	Angulars.clear();
}

