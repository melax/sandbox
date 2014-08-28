//
// Rigidbody Physics
// (c) Stan Melax 1998-2008   bsd licence
//
// See physics.cpp for more info
//


#ifndef PHYSICS_H
#define PHYSICS_H


#include "vecmatquat.h"
#include "wingmesh.h" 
#include "gjk.h"

class Shape;
class RigidBody;
class Spring;

struct Pose // Value type representing a rigid transformation consisting of a translation and rotation component
{
	float3      position;
	float4      orientation;

	Pose(const float3 & p, const float4 & q) : position(p), orientation(q) {}
	Pose() : Pose({ 0, 0, 0 }, { 0, 0, 0, 1 }) {}

	Pose        Inverse() const                             { auto q = qconj(orientation); return{ qrot(q, -position), q }; }
	float4x4    Matrix() const                              { return MatrixFromRotationTranslation(orientation, position); }

	float3      operator * (const float3 & point) const     { return position + qrot(orientation, point); }
	Pose        operator * (const Pose & pose) const        { return{ *this * pose.position, qmul(orientation, pose.orientation) }; }
};

class State : public Pose
{
  public:
					State(const float3 &p,const float4 &q,const float3 &v,const float3 &r): Pose(p,q), momentum(v),rotation(r){}
					State() {};
	float3			momentum;  
	float3			rotation;  // angular momentum

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
	float3			spin;     // often called Omega in most references
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
					RigidBody(std::vector<WingMesh*> &wmeshes,const float3 &_position);
					~RigidBody();
	//WingMesh *		local_geometry;
	//WingMesh *		world_geometry;
	float			hittime;
	int				rest;
	State			old;
	int				collide;  // collide&1 body to world  collide&2 body to body
	int				resolve;
	int				usesound;
	float3          com; // computed in constructor, but geometry gets translated initially 
	float3          PositionUser() {return pose()* -com; }  // based on original origin
	std::vector<Spring*>	springs;
	//virtual int		Support(const float3& dir)const{return local_geometry->Support(qrot(qconj(orientation),dir)); } // return world_geometry->Support(dir); }
	//virtual float3	GetVert(int v)const {return position + qrot(orientation,local_geometry->GetVert(v));} // world_geometry->GetVert(v);}
	std::vector<Shape*>   shapes;
	std::vector<RigidBody*> ignore; // things to ignore during collision checks
};


class Shape 
{
  public:
	// idea is for rigidbody to have an array of these.
	// all meshes are stored in the coord system of the rigidbody, no additional shape transform.
	RigidBody *rb;
	const float3 &Position() const {return rb->position;}
	const float4 &Orientation() const {return rb->orientation;}
	const Pose& pose() const {return rb->pose();}
	WingMesh local_geometry;
	//virtual float3	Support(const float3& dir)const{return rb->pose() * local_geometry->Support(qrot(qconj(rb->orientation),dir)); } // return world_geometry->Support(dir); }
	//virtual float3	GetVert(int v)const {return rb->position + qrot(rb->orientation, local_geometry->GetVert(v));} // world_geometry->GetVert(v);}
	Shape(RigidBody *_rb,WingMesh _geometry);
	~Shape();
};
inline float3    SupportPoint(const Shape *s, const float3& dir) { return s->pose() * SupportPoint(&s->local_geometry, qrot(qconj(s->Orientation()),dir)); }

inline gjk_implementation::Contact Separated(const Shape *a, const Shape *b)
{
	return Separated(a->local_geometry.verts, a->Position(), a->Orientation(), b->local_geometry.verts, b->Position(), b->Orientation());
}
inline gjk_implementation::Contact Separated(const Shape *a, const WingMesh *b)
{
	return Separated(SupportFunc(a->local_geometry.verts, a->Position(), a->Orientation()), SupportFunc(b->verts), 1);
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

void rbscalemass(RigidBody *rb,float s);  // scales the mass and all relevant inertial properties by multiplier s
//Spring *   CreateSpring(RigidBody *a,float3 av,RigidBody *b,float3 bv,float k);
//void       DeleteSpring(Spring *s);






#endif

