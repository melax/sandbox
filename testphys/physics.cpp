

#include <stdio.h>
#include <float.h>
#include "physics.h"
#include "gjk.h"

#define EXPORTVAR(V)

void Line(const float3 &, const float3 &, const float3 &color_rgb){};

inline float3 safenormalize(const float3 &v) { return (v == float3(0,0,0)) ? float3(0, 0, 1) : normalize(v); }
inline float clamp(float a, float mn = 0.0f, float mx = 1.0f) { return std::min(std::max(a, mn), mx); }

inline float3 operator*(const float3x3 &m, const float3& v) { return mul(m, v); }

float   physics_deltaT=(1.0f/60.0f);


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



inline void ApplyImpulse(RigidBody *rb,const float3 &r, const float3& impulse)  // r is impact point positionally relative to rb's origin but in 'world' orientation
{
	rb->momentum +=  impulse;
	rb->rotation += cross(r, impulse);
	rb->spin = rb->Iinv * rb->rotation; // recompute auxilary variable spin
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
};


class LimitAngular : public Limit
{
public:
	float3 axis;  // world-space
	float  torque;  // rotational impulse
	float  targetspin;
	float  mintorque;
	float  maxtorque;
	LimitAngular():Limit(NULL,NULL){}
	LimitAngular(RigidBody *rb0, RigidBody *rb1, const float3 &axis, float targetspin=0, float mintorque=-FLT_MAX, float maxtorque=FLT_MAX) 
		:Limit(rb0, rb1), axis(axis), targetspin(targetspin), mintorque(mintorque), maxtorque(maxtorque), torque(0) {}	
	void RemoveBias(){ targetspin = (mintorque<0) ? 0 : std::min(targetspin, 0.0f); }  // not zero since its ok to let one-sided constraints fall to their bound;
	void Iter()
	{
		if (targetspin == -FLT_MAX) return;
		float currentspin = ((rb1) ? dot(rb1->spin, axis) : 0.0f) - ((rb0) ? dot(rb0->spin, axis) : 0.0f);  // how we are rotating about the axis 'normal' we are dealing with
		float dspin = targetspin - currentspin;  // the amount of spin we have to add to satisfy the limit.
		float spintotorque = 1.0f / (((rb0) ? dot(axis, mul(rb0->Iinv, axis)) : 0.0f) + ((rb1) ? dot(axis, mul(rb1->Iinv, axis)) : 0.0f));
		float dtorque = dspin * spintotorque;  // how we have to change the angular impulse
		dtorque = std::min(dtorque, maxtorque*physics_deltaT - torque); // total torque cannot exceed maxtorque
		dtorque = std::max(dtorque, mintorque*physics_deltaT - torque); // total torque cannot fall below -maxtorque
		if (rb0)
		{
			rb0->rotation += axis*-dtorque;  // apply impulse
			rb0->spin = rb0->Iinv * rb0->rotation; // recompute auxilary variable spin
		}
		if (rb1)
		{
			rb1->rotation += axis*dtorque;  // apply impulse
			rb1->spin = rb1->Iinv * rb1->rotation; // recompute auxilary variable spin
		}
		torque += dtorque;
	}
};



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
	float  impulsesum;
	LimitLinear() :Limit(NULL, NULL){}
	LimitLinear(RigidBody *rb0,RigidBody *rb1,const float3 &position0,const float3 &position1,
		const float3 &normal = float3(0, 0, 1), float _targetspeed = 0.0f, float _targetspeednobias = 0.0f, const float2 forcerange = { -FLT_MAX, FLT_MAX })
		:Limit(rb0, rb1), position0(position0), position1(position1), normal(normal), targetspeed(_targetspeed), targetspeednobias(_targetspeednobias), 
		 minforce(std::min(forcerange.x, forcerange.y)), maxforce(std::max(forcerange.x, forcerange.y)), friction_master(0), impulsesum(0)
	{}
	void RemoveBias() { targetspeed = std::min(targetspeed, targetspeednobias); }
	void Iter()
	{
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
		if(rb0) ApplyImpulse(rb0,r0,normal *-impulse ); 
		if(rb1) ApplyImpulse(rb1,r1,normal * impulse ); 
		impulsesum += impulse;
	}
};


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
	return LimitLinear( rb0,rb1,p0,p1,axisw, dot( ((rb1)?rb1->pose()*p1:p1) - ((rb0)?rb0->pose()*p0:p0) ,axisw)/physics_deltaT,0.0f,{minforce,maxforce});
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


LimitAngular conelimit(RigidBody* rb0,const float3 &n0,RigidBody* rb1,const float3 &n1,float limitangle_degrees)  // a hinge is a cone with 0 limitangle
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




RigidBody::RigidBody(std::vector<Shape> shapes_, const float3 &_position) : shapes(shapes_), orientation_next(0, 0, 0, 1), orientation_old(0, 0, 0, 1), orientation_start(0, 0, 0, 1)
{
	position_start = position_old = position_next = position = _position;
	rest = 0;
	hittime = 0.0f;
	collide = (shapes.size()) ? 3 : 0;
	resolve = (shapes.size()) ? 1 : 0;
	usesound = 0;
	mass = 1;
	gravscale = 1.0f;
	damping = 0.0f;  // rigidbody::damping   currently i just take the max of this and the global damping
	friction = physics_coloumb;

	com = CenterOfMass(shapes);
	position += com;
	position_start = position_old = position_next = position;
	for (auto & s : shapes)
		for (auto &v : s.verts)
			v -= com;

	float3x3 tensor = Inertia(shapes, { 0, 0, 0 });
	massinv = 1.0f / mass;
	Iinv = (tensorinv_massless = inverse(tensor))  * massinv;

	std::vector<float3> allverts;
	for (auto &m : shapes)
		allverts.insert(allverts.end(), m.verts.begin(), m.verts.end());
	radius = magnitude(*std::max_element(allverts.begin(), allverts.end(), [](const float3 &a, const float3 &b){return dot(a, a) < dot(b, b); }));
	std::tie(bmin, bmax) = Extents(allverts);
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


struct PhysContact : public gjk_implementation::Contact
{
	RigidBody *rb0,*rb1;
	float3     p0 , p1 ;
	PhysContact(RigidBody *rb0, RigidBody *rb1, const gjk_implementation::Contact &c) : rb0(rb0), rb1(rb1), gjk_implementation::Contact(c)
	{
		p0 = (rb0) ? rb0->pose().Inverse() * p0w : p0w;
		p1 = (rb1) ? rb1->pose().Inverse() * p1w : p1w;
	}
};

inline  std::function<float3(const float3&)> SupportFunc(RigidBody *rb,const Shape& shape) { return SupportFuncTrans(SupportFunc(shape.verts), rb->position, rb->orientation); }

inline void FindShapeWorldContacts(std::vector<PhysContact> &contacts_out, const std::vector<RigidBody*>& rigidbodies, const std::vector<std::vector<float3> *> & cells)
{
	for (auto rb : rigidbodies) for (auto &shape : rb->shapes)   // foreach rigidbody shape
	{
		if(!(rb->collide&1)) continue;
		for (auto  cell : cells)
			for(auto &c : ContactPatch(SupportFunc(rb,shape), SupportFunc(*cell), physics_driftmax) ) 
				contacts_out.push_back(PhysContact(rb, NULL, c));  
	}
}


inline void FindShapeShapeContacts(std::vector<PhysContact> &contacts_out_append, const std::vector<RigidBody*> & rigidbodies)  // Dynamic-Dynamic contacts
{
	for (auto rb0 : rigidbodies) for (auto rb1 : rigidbodies) if (rb0<rb1)
	{
		if (!(rb0->collide & rb1->collide & 2)) continue;             // 2nd bit means dont collide with other rigidbodies
		if (magnitude(rb1->position - rb0->position) > rb0->radius + rb1->radius) continue;
		if (std::find(rb0->ignore.begin(), rb0->ignore.end(), rb1) != rb0->ignore.end()) continue;
		for (auto &s0 : rb0->shapes) for (auto &s1 : rb1->shapes)
		for (auto &c : ContactPatch(SupportFunc(rb0,s0), SupportFunc(rb1,s1), physics_driftmax))
			contacts_out_append.push_back(PhysContact(rb0, rb1, c));
	}
}
void ConstrainContact(std::vector<LimitLinear> &Linears_out, const PhysContact &c)
{
	RigidBody *rb0 = c.rb0 , *rb1 = c.rb1;

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

	Linears_out.push_back(LimitLinear(rb0, rb1, c.p0, c.p1, -c.normal, -(targvel + bouncevel), -bouncevel, { 0, FLT_MAX }));
				LimitLinear &lim = Linears_out.back();
				float3 tangent  = Orth(-c.normal);
				float3 binormal = cross(c.normal,tangent);
				LimitLinear fb(rb0,rb1,c.p0,c.p1,   binormal,0,0,{0,0});  fb.friction_master=-1;
				LimitLinear ft(rb0,rb1,c.p0,c.p1,   tangent ,0,0,{0,0});  ft.friction_master=-2;
				Linears_out.push_back(fb);
				Linears_out.push_back(ft);
}
//------------------


void PhysicsUpdate(std::vector<RigidBody*> &rigidbodies, std::vector<LimitLinear> &Linears, std::vector<LimitAngular> &Angulars, const std::vector<std::vector<float3> *> &wgeom)
{
	const int physics_iterations = 16;
	const int physics_iterations_post = 4;

	for (auto &rb : rigidbodies)
	{
		if(!rb->resolve) continue;
		rbinitvelocity(rb); // based on previous and current force/torque
	}

	std::vector<PhysContact> contacts;
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
	// i.e. we added some velocity to a point to pull it away from something that was interpenetrating but that velocity shouldn't stick around for the next frame,
	// otherwise we will have lots of jitter from our contacts and occilations from constraints.   do this by clearing the targetvelocities and reinvoking the solver.
	for (auto &ln : Linears)
		ln.RemoveBias();
	for (auto &a : Angulars)
		a.RemoveBias();

	for(int s=0;s<physics_iterations_post;s++)  
	{
		for (auto &ln : Linears)
			ln.Iter();
		for (auto &a : Angulars)
			a.Iter();
	}

	for(auto rb : rigidbodies)
		rbupdatepose(rb);   // setting position,orientation based on rbcalcnextpose


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
	Angulars.push_back(conelimit(rb0, n0, rb1, n1, limitangle_degrees));
}

void PhysicsUpdate(std::vector<RigidBody*> &rigidbodies,const std::vector<std::vector<float3> *> &wgeom)
{
	//GetEnvBSPs(area_bsps);
	PhysicsUpdate(rigidbodies,Linears,Angulars,wgeom);
	Linears.clear();
	Angulars.clear();
}

