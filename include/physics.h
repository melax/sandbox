//
// Rigidbody Physics Stuff
// Sequential Iterative Impulse approach
// (c) Stan Melax 1998-2008   bsd licence 
// with some recent minor code cleanups 2011
// packing the tech into a single header file 2014
//
// Feel free to use and/or learn from the following code.  Code provided as is.
// Author assumes no obligation for correctness of support.
//
// What's here is typical of a small basic 3D physics engine.  But without the retained-mode design.
// This module maintains a number of rigidbodies and implements the update solver.
// Primary references include Baraff&Witkin's notes and many threads on Erwin's bullet website.
// 
// I've been working on this sort of tech for a while often just implementing what I need at the time thus enabling experimentation.  
// For example, i use a runge-kutta integrator for the position updates so i can preserve angular momentum instead of keeping angular spin constant.  
// I try to keep code minimal yet have some degree of generality (without "overdesign").  not claiming perfection. 
// For example, there's no broadphase - i was only doing small demos exploring other concepts at this point.
//


#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdio.h>
#include <float.h>

#include "vecmatquat.h"
#include "gjk.h"

const float   physics_deltaT = (1.0f / 60.0f);
const float   physics_restitution = 0.4f;  // coefficient of restitution
const float3  physics_gravity(0, 0, -10.0f);
const float   physics_coloumb = 0.6f; // used as default friction value and for static geometry    
const float   physics_biasfactorjoint = 0.3f;
const float   physics_biasfactorpositive = 0.3f;
const float   physics_biasfactornegative = 0.3f;
const float   physics_falltime_to_ballistic = 0.2f;
const float   physics_driftmax = 0.03f;
const float   physics_damping = 0.15f;  // 1 means critically damped,  0 means no damping

inline float3 safenormalize(const float3 &v) { return (v == float3(0, 0, 0)) ? float3(0, 0, 1) : normalize(v); }
inline float  clamp(float a, float mn = 0.0f, float mx = 1.0f) { return std::min(std::max(a, mn), mx); }


class Spring;



struct Shape // rigidbody has an array of shapes.  all meshes are stored in the coord system of the rigidbody, no additional shape transform.
{
	std::vector<float3> verts;
	std::vector<int3> tris;
	Shape(std::vector<float3> verts, std::vector<int3> tris) : verts(verts), tris(tris){}
};

inline float Volume(const std::vector<Shape> &meshes)
{
	float  vol = 0;
	for (auto &m : meshes)
		vol += Volume(m.verts.data(), m.tris.data(), (int)m.tris.size());
	return vol;
}

inline float3 CenterOfMass(const std::vector<Shape> &meshes)
{
	float3 com(0, 0, 0);
	float  vol = 0;
	for (auto &m : meshes)
	{
		const auto &tris = m.tris;
		const float3 *verts = m.verts.data();
		float3 cg = CenterOfMass(verts, tris.data(), tris.size());
		float   v = Volume(verts, tris.data(), tris.size());
		vol += v;
		com += cg*v;
	}
	com /= vol;
	return com;
}


inline float3x3 Inertia(const std::vector<Shape> &meshes, const float3& com)
{
	float  vol = 0;
	float3x3 inertia;
	for (auto &m : meshes)
	{
		const auto &tris = m.tris;
		const float3 *verts = m.verts.data();
		float v = Volume(verts, tris.data(), tris.size());
		inertia += Inertia(verts, tris.data(), tris.size(), com) * v;
		vol += v;
	}
	inertia *= 1.0f / vol;
	return inertia;
}

class State : public Pose
{
  public:
					State(const float3 &p, const float4 &q, const float3 &v, const float3 &r) : Pose(p, q), linear_momentum(v), angular_momentum(r){}
					State():linear_momentum(0,0,0),angular_momentum(0,0,0) {};
	float3			linear_momentum ;   
	float3			angular_momentum;

    Pose&           pose() { return *this; }
    const Pose&     pose() const { return *this; }

	State&          state() { return *this; }
	const State&    state() const { return *this; }
};

class RigidBody : public State 
{
  public:
	float			mass;
	float			massinv;  
	//float3x3		tensor;  // inertia tensor
	float3x3		tensorinv_massless;  
	float3x3		Iinv;    // Inverse Inertia Tensor rotated into world space 
	float3			spin() { return mul(Iinv, angular_momentum); }     // often called Omega in most references
	float			radius;
	float3          bmin,bmax; // in local space
	float3			position_next;
	float4          orientation_next;
	float3			position_old;  // used by penetration rollback
	float4   		orientation_old;  // used by penetration rollback
	float3			position_start;
	float4          orientation_start;
	float           damping;
	float			gravscale;
	float			friction;  // friction multiplier
	State			old;
	int				collide;  // collide&1 body to world  collide&2 body to body
	float3          com; // computed in constructor, but geometry gets translated initially 
	float3          PositionUser() {return pose()* -com; }  // based on original origin
	std::vector<Spring*>	springs;
	std::vector<Shape>      shapes;
	std::vector<RigidBody*> ignore; // things to ignore during collision checks
	RigidBody(std::vector<Shape> shapes_, const float3 &_position) : shapes(shapes_), orientation_next(0, 0, 0, 1), orientation_old(0, 0, 0, 1), orientation_start(0, 0, 0, 1)
	{
		position_start = position_old = position_next = position = _position;
		collide = (shapes.size()) ? 3 : 0;
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
};


inline void rbscalemass(RigidBody *rb,float s)  // scales the mass and all relevant inertial properties by multiplier s
{
	rb->mass *= s;
	rb->linear_momentum *= s;
	rb->massinv *= 1.0f / s;
	rb->angular_momentum *= s;
	rb->Iinv *= 1.0f / s;
}
 
class Spring 
{
  public:
	RigidBody *		bodyA;
	float3			anchorA;
	RigidBody *		bodyB;
	float3			anchorB;
	float			k; // spring constant
};



//Spring *   CreateSpring(RigidBody *a,float3 av,RigidBody *b,float3 bv,float k);
//void       DeleteSpring(Spring *s);



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
	float4 d1 = DiffQ(s              , tensorinv, angular);
	float4 d2 = DiffQ(s + d1*(dt / 2), tensorinv, angular);
	float4 d3 = DiffQ(s + d2*(dt / 2), tensorinv, angular);
	float4 d4 = DiffQ(s + d3*(dt)    , tensorinv, angular);
	return normalize(s + d1*(dt / 6) + d2*(dt / 3) + d3*(dt / 3) + d4*(dt / 6));
}



inline void ApplyImpulse(RigidBody *rb,const float3 &r, const float3& impulse)  // r is impact point positionally relative to rb's origin but in 'world' orientation
{
	rb->linear_momentum  +=  impulse;
	rb->angular_momentum += cross(r, impulse);
}



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
	float3 axis;        // world-space
	float  torque;      // rotational impulse
	float  targetspin;  // usually this includes any spin required to get things into desired alignment
	float  mintorque;   // note this is usually -maxtorque when axis is fully constrained or  0 angle range is limited 
	float  maxtorque;
	LimitAngular():Limit(NULL,NULL){}
	LimitAngular(RigidBody *rb0, RigidBody *rb1, const float3 &axis, float targetspin=0, float mintorque=-FLT_MAX, float maxtorque=FLT_MAX) 
		:Limit(rb0, rb1), axis(axis), targetspin(targetspin), mintorque(mintorque), maxtorque(maxtorque), torque(0) {}	
	void RemoveBias(){ targetspin = (mintorque<0) ? 0 : std::min(targetspin, 0.0f); }  // not zero since its ok to let one-sided constraints fall to their bound;
	void Iter()
	{
		if (targetspin == -FLT_MAX) return;
		float currentspin = ((rb1) ? dot(rb1->spin(), axis) : 0.0f) - ((rb0) ? dot(rb0->spin(), axis) : 0.0f);  // how we are rotating about the axis 'normal' we are dealing with
		float dspin = targetspin - currentspin;  // the amount of spin we have to add to satisfy the limit.
		float spintotorque = 1.0f / (((rb0) ? dot(axis, mul(rb0->Iinv, axis)) : 0.0f) + ((rb1) ? dot(axis, mul(rb1->Iinv, axis)) : 0.0f));
		float dtorque = dspin * spintotorque;  // how we have to change the angular impulse
		dtorque = std::min(dtorque, maxtorque*physics_deltaT - torque); // total torque cannot exceed maxtorque
		dtorque = std::max(dtorque, mintorque*physics_deltaT - torque); // total torque cannot fall below mintorque
		if (rb0)
			rb0->angular_momentum -= axis*dtorque;  // apply impulse
		if (rb1)
			rb1->angular_momentum += axis*dtorque;  // apply impulse
		torque += dtorque;
	}
};



class LimitLinear : public Limit
{
 public:
	float3 position0;  // position of contact on rb0 in rb0 local space
	float3 position1;  // position of contact on rb1 in rb1 local space
	float3 normal;     // direction in world in which limit is applied
	float  targetdist;
	float  targetspeednobias;
	float2 forcelimit;
	int    friction_master;   // true if a friction limit
	float  targetspeed; 
	float  impulsesum;
	LimitLinear() :Limit(NULL, NULL){}
	LimitLinear(RigidBody *rb0,RigidBody *rb1,const float3 &position0,const float3 &position1,
		const float3 &normal = float3(0, 0, 1), float targetdist = 0.0f, float targetspeednobias = 0.0f, const float2 forcelimit = { -FLT_MAX, FLT_MAX })
		:Limit(rb0, rb1), position0(position0), position1(position1), normal(normal), targetdist(targetdist), targetspeednobias(targetspeednobias),
		forcelimit({ std::min(forcelimit.x, forcelimit.y), std::max(forcelimit.x, forcelimit.y) }), friction_master(0), impulsesum(0)
	{}
	void RemoveBias() { targetspeed = std::min(targetspeed, targetspeednobias); }
	void Iter()
	{
		if(friction_master)
			forcelimit.x = -(  forcelimit.y = std::max( ((rb0)?rb0->friction:0),((rb1)?rb1->friction:0) ) * ((this)+friction_master)->impulsesum / physics_deltaT   );
		float3 r0  = (rb0) ? qrot(rb0->orientation , position0) :  position0;
		float3 r1  = (rb1) ? qrot(rb1->orientation , position1) :  position1;
		float3 v0  = (rb0) ? cross(rb0->spin(),r0) + rb0->linear_momentum*rb0->massinv : float3(0,0,0); // instantaneioius linear velocity at point of constraint
		float3 v1  = (rb1) ? cross(rb1->spin(),r1) + rb1->linear_momentum*rb1->massinv : float3(0,0,0); 
		float  vn  = dot(v1-v0,normal);  // velocity of rb1 wrt rb0
		float impulsen = -targetspeed - vn;
		float impulsed =  ((rb0)? rb0->massinv + dot( cross(mul(rb0->Iinv,cross(r0,normal)),r0),normal):0) 
						+ ((rb1)? rb1->massinv + dot( cross(mul(rb1->Iinv,cross(r1,normal)),r1),normal):0) ;
		float impulse = impulsen/impulsed;
		impulse = std::min( forcelimit.y*physics_deltaT-impulsesum,impulse);
		impulse = std::max( forcelimit.x*physics_deltaT-impulsesum,impulse);
		if(rb0) ApplyImpulse(rb0,r0,normal *-impulse ); 
		if(rb1) ApplyImpulse(rb1,r1,normal * impulse ); 
		impulsesum += impulse;
	}
};


// various examples of limits/constraints that can be created

inline std::vector<LimitAngular> ConstrainAngularDrive( RigidBody *rb0, RigidBody *rb1, float4 target, float maxtorque)  // for driving powered-ragdoll 
{
	float4 q0 = (rb0)?rb0->orientation:float4(0,0,0,1);
	float4 q1 = (rb1)?rb1->orientation:float4(0,0,0,1);
    float4 dq = qmul(q1, qconj(qmul(q0, target)));  // quat that takes a direction in r0+target orientation into r1 orientation
	if(dq.w<0) dq=-dq;
	float3 axis     = safenormalize(dq.xyz()); // dq.axis();
	float3 binormal = Orth(axis);
	float3 normal   = cross(axis,binormal);
	return{
		LimitAngular(rb0, rb1, axis    , -physics_biasfactorjoint*(acosf(clamp(dq.w, -1.0f, 1.0f))*2.0f) / physics_deltaT, -maxtorque, maxtorque),
		LimitAngular(rb0, rb1, binormal, 0, -maxtorque, maxtorque),
		LimitAngular(rb0, rb1, normal  , 0, -maxtorque, maxtorque) };
}

inline LimitLinear ConstrainAlongDirection(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
{
	return LimitLinear( rb0,rb1,p0,p1,axisw, dot( ((rb1)?rb1->pose()*p1:p1) - ((rb0)?rb0->pose()*p0:p0) ,axisw),0.0f,{minforce,maxforce});
}
inline std::vector<LimitLinear> ConstrainPositionNailed( RigidBody *rb0, const float3 &p0, RigidBody *rb1, const float3 &p1)
{
	float3 d = (((rb1)?rb1->pose()*p1:p1) - ((rb0)?rb0->pose()*p0:p0) );
	return {LimitLinear(rb0,rb1,p0,p1,float3(1,0,0),d.x), LimitLinear(rb0,rb1,p0,p1,float3(0,1,0),d.y), LimitLinear(rb0,rb1,p0,p1,float3(0,0,1),d.z) };
}

inline std::vector<LimitAngular> ConstrainAngularRangeW(RigidBody *rb0, const float4 &jb0, RigidBody *rb1, const float4 &jf1, const float3& _jointlimitmin, const float3& _jointlimitmax)
{
	// a generic configurable 6dof style way to specify angular limits.  used for hard limits such as joint ranges.
	std::vector<LimitAngular> Angulars_out;
	float3 jmin = _jointlimitmin * 3.14f / 180.0f;
	float3 jmax = _jointlimitmax * 3.14f/180.0f; 

	if(jmin.x==0 && jmax.x==0 && jmin.y==0 && jmax.y==0 && jmin.z<jmax.z)
	{
		float4 cb = normalize(float4(0,-1,0,1));
		return ConstrainAngularRangeW(rb0, qmul(jb0, cb), rb1, qmul(jf1, cb), float3(_jointlimitmin.z, 0, 0), float3(_jointlimitmax.z, 0, 0));
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
	return Angulars_out;
}

inline std::vector<LimitAngular> ConstrainAngularRange( RigidBody *rb0, RigidBody *rb1, const float4 &_jointframe, const float3& _jointlimitmin, const float3& _jointlimitmax)
{
	// a generic configurable 6dof style way to specify angular limits.  used for hard limits such as joint ranges.
	return ConstrainAngularRangeW(rb0, (rb0) ? qmul(rb0->orientation, _jointframe) : _jointframe, rb1, (rb1) ? rb1->orientation : float4(0, 0, 0, 1), _jointlimitmin, _jointlimitmax);
}


inline LimitAngular ConstrainConeAngle(RigidBody* rb0,const float3 &n0,RigidBody* rb1,const float3 &n1,float limitangle_degrees)  // a hinge is a cone with 0 limitangle
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







//----------------  new contact code -------------


struct PhysContact : public gjk_implementation::Contact
{
	RigidBody *rb0,*rb1;   // rigid bodies involved in the contact
	float3     p0 , p1 ;   // the points of contact in the local space of the rigidbodies
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
		float distance_range = std::max(physics_driftmax, magnitude(rb->linear_momentum) *physics_deltaT / rb->mass);  // dont need to create potential contacts if beyond this range
		for (auto  cell : cells)
			for(auto &c : ContactPatch(SupportFunc(rb,shape), SupportFunc(*cell), distance_range) )
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
inline std::vector<LimitLinear> ConstrainContacts(const std::vector<PhysContact> &contacts)
{
	std::vector<LimitLinear> linearconstraints;
	for (auto &c : contacts)
	{
		RigidBody *rb0 = c.rb0 , *rb1 = c.rb1;
		float3 r0 = (rb0) ? c.p0w - rb0->position : float3(0,0,0);  
		float3 v0 = (rb0) ? cross(rb0->spin(), r0) + rb0->linear_momentum*rb0->massinv : float3(0,0,0); // instantaneioius linear velocity at point of collision
		float3 r1 = (rb1) ? c.p1w - rb1->position : float3(0,0,0);  
		float3 v1 = (rb1) ? cross(rb1->spin(), r1) + rb1->linear_momentum*rb1->massinv : float3(0,0,0); 
		float3 v = v0-v1;  // todo would look cleaner if it was v1-v0.  
		
		float minsep = physics_driftmax*0.25f;
		float separation = c.separation; //  verify direction here 
		float bouncevel = std::max(0.0f, (-dot(c.normal, v) - magnitude(physics_gravity)*physics_falltime_to_ballistic) * physics_restitution);  // ballistic non-resting contact, allow elastic response
		linearconstraints.push_back(LimitLinear(rb0, rb1, c.p0, c.p1, -c.normal, std::min((separation-minsep)*physics_biasfactorpositive,separation) , -bouncevel, { 0, FLT_MAX }));  // could also add (-bouncevel*physics_deltaT) to targetdist too
		auto q = RotationArc({ 0,0,1 }, -c.normal);   // to get orthogonal axes on the plane
		float3 normal = qzdir(q);
		float3 tangent = qxdir(q); //  Orth(-c.normal);
		float3 binormal = qydir(q); //  cross(c.normal, tangent);
		LimitLinear fb(rb0,rb1,c.p0,c.p1,   binormal,0,0,{0,0});  fb.friction_master=-1;
		LimitLinear ft(rb0,rb1,c.p0,c.p1,   tangent ,0,0,{0,0});  ft.friction_master=-2;
		linearconstraints.push_back(fb);
		linearconstraints.push_back(ft);
	}
	return linearconstraints;
}

inline std::vector<LimitLinear> CollisionConstraints(std::vector<RigidBody*> &rigidbodies, const std::vector<std::vector<float3> *> &wgeom)
{
	std::vector<PhysContact> contacts;
	FindShapeWorldContacts(contacts, rigidbodies, wgeom);
	FindShapeShapeContacts(contacts, rigidbodies);
	return ConstrainContacts(contacts);
}
//------------------

inline void rbinitvelocity(RigidBody *rb)
{
	// gather weak forces being applied to body at beginning of timestep
	// forward euler update of the velocity and rotation/spin 
	rb->old.position = rb->position;
	rb->old.orientation = rb->orientation;
	float dampleftover = powf((1.0f - std::max(rb->damping, physics_damping)), physics_deltaT);
	rb->linear_momentum *= dampleftover;
	rb->angular_momentum *= dampleftover;
	State s = rb->state(); // (State) *rb;
	float3 force;
	float3 torque;
	force = physics_gravity*rb->mass*rb->gravscale;
	torque = float3(0, 0, 0);
	//AddSpringForces(rb,s,&force,&torque);

	rb->linear_momentum += force*physics_deltaT;
	rb->angular_momentum += torque*physics_deltaT;
	rb->Iinv = mul(qgetmatrix(rb->orientation), rb->tensorinv_massless * rb->massinv, transpose(qgetmatrix(rb->orientation)));
}


inline void rbcalcnextpose(RigidBody *rb)
{
	const int physics_euler_integration = 0;
	// after an acceptable velocity and spin are computed, a forward euler update is applied ot the position and orientation.
	rb->position_next = rb->position + rb->linear_momentum * rb->massinv * physics_deltaT;
	rb->orientation_next = (physics_euler_integration) ? normalize(rb->orientation + DiffQ(rb->orientation, rb->tensorinv_massless * rb->massinv, rb->angular_momentum)*physics_deltaT) : rkupdateq(rb->orientation, rb->tensorinv_massless * rb->massinv, rb->angular_momentum, physics_deltaT);
	for (int i = 0; i<3; i++)
	if (rb->orientation_next[i]< FLT_EPSILON / 4.0 && rb->orientation_next[i]> -FLT_EPSILON / 4.0)
		rb->orientation_next[i] = 0.0f; // dont keep needless precision, somehow this causes serious slowdown on Intel's centrino processors perhaps becuase as extra shifting is required during multiplications
}

inline void rbupdatepose(RigidBody *rb)
{
	// after an acceptable velocity and spin are computed, a forward euler update is applied ot the position and orientation.
	rb->position_old = rb->position;  // should do this at beginning of physics loop in case someone teleports the body.
	rb->orientation_old = rb->orientation;
	rb->position = rb->position_next;
	rb->orientation = rb->orientation_next;
	rb->Iinv = mul(qgetmatrix(rb->orientation), rb->tensorinv_massless * rb->massinv, transpose(qgetmatrix(rb->orientation)));
}

inline void PhysicsUpdate(std::vector<RigidBody*> &rigidbodies, std::vector<LimitLinear> Linears, std::vector<LimitAngular> &Angulars, const std::vector<std::vector<float3> *> &wgeom)
{
	const int physics_iterations = 16;
	const int physics_iterations_post = 4;

	for (auto &rb : rigidbodies)
		rbinitvelocity(rb);          // based on previous and current force/torque

	auto collionconstraints = CollisionConstraints(rigidbodies, wgeom);
	for (const auto & c : collionconstraints)
		Linears.push_back(c);  // currently uses aux variables on the rigidbodies (orientation,angularmomentum, Iinv) to calculate bounce velocity etc.

	for (auto &ln : Linears)
		ln.targetspeed = ln.targetdist / physics_deltaT;

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

	// might want a CCD or tunneling check here

	for(auto rb : rigidbodies)
		rbupdatepose(rb);   // setting position,orientation based on rbcalcnextpose
}



#endif

