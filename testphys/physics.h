//
// Rigidbody Physics
// (c) Stan Melax 1998-2008   bsd licence
//
// See physics.cpp for more info
//


#ifndef PHYSICS_H
#define PHYSICS_H


#include "vecmatquat.h"
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
					RigidBody(std::vector<Shape> shapes, const float3 &_position);
					~RigidBody();
	float			hittime;
	int				rest;
	State			old;
	int				collide;  // collide&1 body to world  collide&2 body to body
	int				resolve;
	int				usesound;
	float3          com; // computed in constructor, but geometry gets translated initially 
	float3          PositionUser() {return pose()* -com; }  // based on original origin
	std::vector<Spring*>	springs;
	std::vector<Shape>      shapes;
	std::vector<RigidBody*> ignore; // things to ignore during collision checks
};


class Shape 
{
  public:
	// idea is for rigidbody to have an array of these.
	// all meshes are stored in the coord system of the rigidbody, no additional shape transform.
	std::vector<float3> verts;
	std::vector<int3> tris;
	RigidBody *rb;
	const float3 &Position() const {return rb->position;}
	const float4 &Orientation() const {return rb->orientation;}
	const Pose& pose() const {return rb->pose();}
	Shape(std::vector<float3> verts, std::vector<int3> tris, RigidBody *rb = NULL) : verts(verts), tris(tris), rb(rb){}
	~Shape(){};
};
// inline float3    SupportPoint(const Shape *s, const float3& dir) { return s->pose() * SupportPoint(&s->verts, qrot(qconj(s->Orientation()),dir)); }

inline gjk_implementation::Contact Separated(const Shape *a, const Shape *b)
{
	return Separated(a->verts, a->Position(), a->Orientation(), b->verts, b->Position(), b->Orientation());
}
inline gjk_implementation::Contact Separated(const Shape *a, const std::vector<float3> *b)
{
	return Separated(SupportFunc(a->verts, a->Position(), a->Orientation()), SupportFunc(*b), 1);
}


inline float Volume(const std::vector<Shape> &meshes)
{
	float  vol = 0;
	for (auto &m : meshes)
		vol += Volume(m.verts.data(),m.tris.data(),(int)m.tris.size());
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

