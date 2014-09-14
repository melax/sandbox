

#include "physics.h"
#include "gjk.h"



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
//float springk=12.56f; // spring constant position - default to about half a second for unit mass weight
//float springr=1.0f;   // spring constant rotation 

//EXPORTVAR(springk);
//EXPORTVAR(physics_damping);
//float3 SpringForce(State &s,const float3 &home,float mass) {
//	return (home-s.position)*springk +   // spring part
//		   (s.momentum/mass)* (-sqrtf(4 * mass * springk) * physics_damping);    // physics_damping part
//}

//int AddSpringForces(RigidBody *rba,const State &state,float3 *force,float3 *torque)
//{
//
//	return 1;
//}



//--  retained mode API --

//std::vector<LimitAngular> Angulars;
//std::vector<LimitLinear> Linears;
//
//
//void addlimitlinearaxis(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
//{
//	Linears.push_back(  limitlinearaxis(rb0,p0,rb1,p1,axisw, minforce, maxforce)  );
//}
//void addlimitlinearaxisflow(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1,const float3 &axisw,float minforce,float maxforce)
//{
//	Linears.push_back(limitlinearaxis(rb0, p0, rb1, p1, axisw, minforce, maxforce));
//	LimitLinear &lim=Linears.back();
//	lim.targetspeednobias=lim.targetspeed;
//}
//void createnail(RigidBody *rb0,const float3 &p0,RigidBody *rb1,const float3 &p1)
//{
//	float maxforce = FLT_MAX;
//	addlimitlinearaxis(rb0,p0,rb1,p1,float3(1,0,0),-maxforce,maxforce);
//	addlimitlinearaxis(rb0,p0,rb1,p1,float3(0,1,0),-maxforce,maxforce);
//	addlimitlinearaxis(rb0,p0,rb1,p1,float3(0,0,1),-maxforce,maxforce);
//	//return createnail(Linears,rb0,p0,rb1,p1);
//}
//void createdrive(RigidBody *rb0,RigidBody *rb1,float4 target,float maxtorque)
//{
//	return createdrive(Angulars,rb0,rb1,target,maxtorque);
//}
//void createangularlimits(RigidBody *rb0,RigidBody *rb1, const float4 &_jointframe, const float3& _jointlimitmin, const float3& _jointlimitmax)
//{
//	createangularlimits(Angulars,rb0,rb1,_jointframe,_jointlimitmin,_jointlimitmax);
//}
//void createconelimit(RigidBody* rb0,const float3 &n0,RigidBody* rb1,const float3 &n1,float limitangle_degrees)  // a hinge is a cone with 0 limitangle
//{
//	Angulars.push_back(conelimit(rb0, n0, rb1, n1, limitangle_degrees));
//}
//
//void PhysicsUpdate(std::vector<RigidBody*> &rigidbodies,const std::vector<std::vector<float3> *> &wgeom)
//{
//	//GetEnvBSPs(area_bsps);
//	PhysicsUpdate(rigidbodies,Linears,Angulars,wgeom);
//	Linears.clear();
//	Angulars.clear();
//}
//
//