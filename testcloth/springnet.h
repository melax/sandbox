//
// by Stan Melax  (c) 2004   
// some updates 2014 to be more c++11 friendly
//
// Free demo and source, but do give credit where due when appropriate. 
// The core backward integration functionality has all been extracted into the SpringNet class.
// Attempted to provide a minimal implementation for those who just want to see the math and algorithm.
//
// Hooke Spring Simulation Code with Implicit integration
// Can be used for simulating soft surfaces such as cloth
// Based on "Large Steps in Cloth Simulation" by Baraff and Witkin siggraph 98
//
// Regarding stability (usually compression stability) as mentioned in section 7 (adaptive time stepping) of their paper...
// I noticed as springs become compressed close to zero length, although the forces still have
// reasonable values, the jacobian (force derivates) can become large and only accurate for a shrinking neighborhood.
// Obviously this is a prime candidate for numerical issues.  I only noticed numerical badness in such compression cases.  
// Therefore rather than adaptive stepping or some sort of clamping scheme,
// I limit the denominator for computing dfdx and dfdv for compressed springs, eg max(length,rest).  
// This kinda just makes everything work (by which i mean non-exploding) even when things get bunched up.  
// 

#pragma once
#ifndef SPRINGNET_H
#define SPRINGNET_H

#include <stdio.h>
#include <float.h>  
#include <assert.h>
#include <vector>

#include "vecmatquat.h"

//-------  vec3n large vector library -----------


class float3Nx3N
{
public:
	struct Block
	{
		struct {
			float3x3 m;
			int r, c;
		};
		Block(){}
		Block(short _r, short _c) :r(_r), c(_c){ m.x = m.y = m.z = float3(0, 0, 0); }
	};
	std::vector<Block> blocks;  // the first n blocks use as the diagonal.
	const int  n;               // number of points in spring network
	void Zero();
	void InitDiagonal(float d);
	void Identity(){ InitDiagonal(1.0f); }
	float3Nx3N() :n(0){}
	float3Nx3N(int _n) :n(_n) { for (int i = 0; i<n; i++) blocks.push_back(Block((short)i, (short)i)); }
};
inline void Msub(float3Nx3N &r, const float3Nx3N &a, float s, const float3Nx3N &b, float t)
{
	for (unsigned int i = 0; i<r.blocks.size(); i++) r.blocks[i].m -= a.blocks[i].m*s + b.blocks[i].m*t;
}

class float3N : public std::vector<float3>
{
public:
	float3N(int _count = 0)
	{
		resize(_count);
	}
	void Zero();
	void Init(const float3 &v);  // sets each subvector to v
};


inline float3N& Mul(float3N &r, const float3Nx3N &m, const float3N &v)
{
	for (unsigned int i = 0; i<r.size(); i++) r[i] = float3(0, 0, 0);
	for (unsigned int i = 0; i<m.blocks.size(); i++)
	{
		r[m.blocks[i].r] += mul(m.blocks[i].m, v[m.blocks[i].c]);
	}
	return r;
}



inline float dot(const float3N &a, const float3N &b)
{
	float d = 0;
	for (unsigned int i = 0; i<a.size(); i++)
	{
		d += dot(a[i], b[i]);
	}
	return d;
}

inline void float3Nx3N::Zero()
{
	for (unsigned int i = 0; i<blocks.size(); i++)
	{
		blocks[i].m = float3x3();
	}
}

inline void float3Nx3N::InitDiagonal(float d)
{

	for (unsigned int i = 0; i<blocks.size(); i++)
	{
		blocks[i].m = (blocks[i].c == blocks[i].r) ? float3x3({ d, 0, 0 }, { 0, d, 0 }, { 0, 0, d }) : float3x3();
	}
}

inline void float3N::Zero()
{
	auto element = data();
	for (unsigned int i = 0; i<size(); i++)
	{
		element[i] = float3(0, 0, 0);
	}
}

inline void float3N::Init(const float3 &v)
{
	auto element = data();
	for (unsigned int i = 0; i<size(); i++)
	{
		element[i] = v;
	}
}

// Unoptimized Basic Version of big vector operators +,/,-,*,=
// c++11 move semantics reduces some (but not all) of the memory allocation, and copying.


inline float3N operator +(const float3N &a, const float3N &b)
{
	float3N r(a.size());
	for (unsigned int i = 0; i<a.size(); i++) r[i] = a[i] + b[i];
	return r;
}

inline float3N operator *(const float3N &a, const float &s)
{
	float3N r(a.size());
	for (unsigned int i = 0; i<a.size(); i++) r[i] = a[i] * s;
	return r;
}
inline float3N operator /(const float3N &a, const float &s)
{
	return a * (1.0f / s);
}
inline float3N operator -(const float3N &a, const float3N &b)
{
	float3N r(a.size());
	for (unsigned int i = 0; i<a.size(); i++) r[i] = a[i] - b[i];
	return r;
}
inline float3N operator -(const float3N &a)
{
	float3N r(a.size());
	for (unsigned int i = 0; i<a.size(); i++) r[i] = -a[i];
	return r;
}

inline float3N operator *(const float3Nx3N &m, const float3N &v)
{
	float3N r(v.size());
	return Mul(r, m, v);
}
inline float3N &operator-=(float3N &A, const float3N &B)
{
	assert(A.size() == B.size());
	for (unsigned int i = 0; i<A.size(); i++) A[i] -= B[i];
	return A;
}
inline float3N &operator+=(float3N &A, const float3N &B)
{
	assert(A.size() == B.size());
	for (unsigned int i = 0; i<A.size(); i++) A[i] += B[i];
	return A;
}



struct HalfConstraint {
	float3 n; int vi;
	float s, t;
	HalfConstraint(const float3& _n, int _vi, float _t) :n(_n), vi(_vi), s(0), t(_t){}
	HalfConstraint() :vi(-1){}
};

inline int  ConjGradientFiltered(float3N &X, const float3Nx3N &A, const float3N &B, const float3Nx3N &S, std::vector<HalfConstraint> &H)
{
	float conjgrad_epsilon = 0.02f;
	int   conjgrad_looplimit = 100;
	auto filter = [](float3N &V, const float3Nx3N &S)  // inline to this function only
	{
		for (unsigned int i = 0; i<S.blocks.size(); i++)
		{
			V[S.blocks[i].r] = mul(S.blocks[i].m, V[S.blocks[i].r]);
		}
	};
	auto filterH = [](float3N &V, std::vector<HalfConstraint> &H)
	{
		for (unsigned int i = 0; i<H.size(); i++)
		{
			float3& v = V[H[i].vi];
			float d = dot(v, H[i].n) - H[i].t;
			if (d<0.0f)
				v += H[i].n * -d;
		}
	};
	// Solves for unknown X in equation AX=B
	int conjgrad_loopcount = 0;
	int n = B.size();
	float3N q(n), d(n), tmp(n), r(n);
	//r = B - A*X;   // just set r to B if X known to be zero
	tmp = A*X;
	r = B - tmp;
	filter(r, S);
	d = r;
	float s = dot(r, r);
	float starget = s * conjgrad_epsilon*conjgrad_epsilon;
	while (s>starget && conjgrad_loopcount++ < conjgrad_looplimit)
	{
		q = A*d;
		filter(q, S);
		float a = s / dot(d, q);
		X = X + d*a;
		filterH(X, H);
		if (H.size() || conjgrad_loopcount % 50 == 0)
		{
			tmp = A*X;  // r = B - A*X   // Mul(tmp,A,X); 
			r = B - tmp;
			filter(r, S);
		}
		else
		{
			r = r - q*a;
		}
		float s_prev = s;
		s = dot(r, r);
		d = r + d*(s / s_prev);
		filter(d, S);
	}
	//conjgrad_lasterror = s;
	return conjgrad_loopcount<conjgrad_looplimit;  // true means we reached desired accuracy in given time - ie stable
}



// ---------- vec3n end of big vector section  --------------


#define SPRING_STRUCT (0)
#define SPRING_SHEAR  (1)
#define SPRING_BEND   (2)

static const float3x3 Identity({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 });

inline float3x3 dfdx_spring(const float3 &dir, float length, float rest, float k)
{
	// dir is unit length direction, rest is spring's restlength, k is spring constant.
	return  ((Identity - outerprod(dir, dir))*std::min(1.0f, rest / length) - Identity) * -k;
}
inline float3x3 dfdx_damp(const float3 &dir, float length, const float3& vel, float rest, float damping)
{
	// inner spring damping   vel is the relative velocity  of the endpoints.  
	return (Identity - outerprod(dir, dir)) * (-damping * -(dot(dir, vel) / std::max(length, rest)));
}
inline float3x3 dfdv_damp(const float3 &dir, float damping)
{
	// derivative of force wrt velocity.  
	return outerprod(dir, dir) * damping;
}



static const float3 cloth_gravity_default(0,0,-10);
static const float  cloth_dt_default = 0.016f;  // default 60fps - delta time for springnetwork



class SpringNetwork
{
 public:
	class Spring
	{
	 public:
		int type;           // index into coefficients spring_k[] 
		float restlen;     
		short int a,b;      // spring endpoints, indices into vertex array
		short int iab,iba;  // INTERNAL: indices into off-diagonal blocks of sparse matrix  
		Spring(){}
		Spring(int _type,int _a,int _b,float _restlen):type(_type),a(_a),b(_b),restlen(_restlen){iab=iba=-1;}
	};

	union 
	{
		struct 
		{
			float spring_struct;
			float spring_shear;
			float spring_bend;
		};
		float spring_k[3];
	};
	float3 gravity;
	int    awake;
	float3 bmin;              // bounding box, updated by simulater each frame
	float3 bmax;              // 
	float  damp_spring;
	float  damp_air;
	float  dt;                // time delta to simulate at each Simulate() call.
	float  sleepthreshold;
	int    sleepcount;
	float3 wind;
	int    simd;
	float  collision_epsilon; // thickness of cloth, default is 0.01f or 1cm

	std::vector<Spring> springs;
	float3N     X;            // positions of all points
	std::vector<float3> Xb;   // positions 
	std::vector<float> M;     // inverse mass 
	float3N     V;            // velocities
	float3N     N;            // normals
	float3N     P;            // pressure
	float3N     F;            // force on each point
	float3N     dV;           // change in velocity
	float3Nx3N  A;            // big matrix we solve system with
	float3Nx3N  dFdX;         // big matrix of derivative of force wrt position
	float3Nx3N  dFdV;         // big matrix of derivative of force wrt velocity
	float3Nx3N  S;            // used for our constraints - contains only some diagonal blocks as needed S[i,i]
	std::vector<HalfConstraint> H;
	std::vector<int3>  tris;
	std::vector<int4>  quads;

	Spring &CreateSpring(int type,int a,int b,float restlen){springs.push_back( Spring(type,a,b,restlen));return AddBlocks(springs.back());}
	Spring &CreateSpring(int type,int a,int b){return CreateSpring(type,a,b,magnitude(X[b]-X[a]));}
	//void    UpdateLimits() { BoxLimits(X.data(),X.size(),(float3&)bmin,(float3&)bmax);}
	void    Wake(){awake=sleepcount;}
	float3* GetPoints(){ return X.data(); }

	//         SpringNetwork(int _n);
	// Spring& AddBlocks(Spring &s);    
	// void    Simulate();
	// //int     ProcessCollisionConstraints();
	// void    PreSolveSpring(const Spring &s);
	// void    CalcForces();
	// void    CalcNormals();
	// int     PointStatusSet(int index,int op);

	SpringNetwork::SpringNetwork(int _n):X(_n),V(_n),F(_n),dV(_n),A(_n),N(_n),P(_n),dFdX(_n),dFdV(_n),gravity(cloth_gravity_default)
	{
		Xb.reserve(_n);
		for(int i=0;i<_n;i++) 
		{
			M.push_back(1.0f);
			Xb.push_back(float3(0,0,0));
		}
		assert(SPRING_STRUCT==0);
		assert(&spring_shear == &spring_struct +SPRING_SHEAR);
		assert(&spring_bend  == &spring_struct +SPRING_BEND);
		assert(&spring_struct== &spring_k[SPRING_STRUCT]);
		assert(&spring_shear == &spring_k[SPRING_SHEAR ]);
		assert(&spring_bend  == &spring_k[SPRING_BEND  ]);
		spring_struct=100000.0f;
		spring_shear=5000.0f;
		spring_bend=1000.0f;
		damp_spring=10.0f;
		damp_air=5.0f;
		dt = cloth_dt_default ; // 
		sleepthreshold = 0.001f;
		sleepcount = 100;
		awake = sleepcount;
		(float3&)bmin=float3(0,0,0);
		(float3&)bmax=float3(0,0,0);
		(float3&)wind=float3(0,0,0);
		collision_epsilon = 0.01f;
		simd=0;
	}

	SpringNetwork::Spring &SpringNetwork::AddBlocks(Spring &s)
	{
		// Called during initial creation of springs in our spring network.  
		// Sets up the sparse matrices corresponding to connections.
		// Note the indices (s.iab,s.iba) are also stored with spring to avoid looking them up each time a spring is applied
		// All 3 matrices A,dFdX, and dFdV are contstructed identically so the block array layout will be the same for each.
		s.iab = A.blocks.size();   // added 'ab' blocks will have this index.
		A.blocks.push_back(float3Nx3N::Block(s.a,s.b)); 
		dFdX.blocks.push_back(float3Nx3N::Block(s.a,s.b));
		dFdV.blocks.push_back(float3Nx3N::Block(s.a,s.b));
		s.iba = A.blocks.size();   // added 'ba' blocks will have this index.
		A.blocks.push_back(float3Nx3N::Block(s.b,s.a)); 
		dFdX.blocks.push_back(float3Nx3N::Block(s.b,s.a));
		dFdV.blocks.push_back(float3Nx3N::Block(s.b,s.a));
		return s;
	}

	void SpringNetwork::PreSolveSpring(const SpringNetwork::Spring &s)
	{
		// Adds this spring's contribution into force vector F and force derivitves dFdX and dFdV
		// One optimization would be premultiply dfdx by dt*dt and F and dFdV by dt right here in this function. 
		// However, for educational purposes we wont do that now and intead just follow the paper directly.
		//assert(dFdX.blocks[s.a].c==s.a);  // delete this assert, no bugs here
		//assert(dFdX.blocks[s.a].r==s.a);
		float3 extent = X[s.b] - X[s.a];
		float  length = magnitude(extent);
		float3 dir    = (length==0)?float3(0,0,0): extent * 1.0f/length;
		float3 vel    = V[s.b] - V[s.a];
		float  k      = spring_k[s.type];
		float3 f =  dir * ((k * (length-s.restlen) ) +  damp_spring * dot(vel,dir));  // spring force + damping force
		F[s.a] += f;
		F[s.b] -= f;
		float3x3 dfdx = dfdx_spring(dir,length,s.restlen,k) + dfdx_damp(dir,length,vel,s.restlen,damp_spring);
		dFdX.blocks[s.a].m   -= dfdx;  // diagonal chunk dFdX[a,a] 
		dFdX.blocks[s.b].m   -= dfdx;  // diagonal chunk dFdX[b,b]
		dFdX.blocks[s.iab].m += dfdx;  // off-diag chunk dFdX[a,b]
		dFdX.blocks[s.iba].m += dfdx;  // off-diag chunk dFdX[b,a]
		float3x3 dfdv = dfdv_damp(dir,damp_spring);
		dFdV.blocks[s.a].m   -= dfdv;  // diagonal chunk dFdV[a,a]
		dFdV.blocks[s.b].m   -= dfdv;  // diagonal chunk dFdV[b,b]
		dFdV.blocks[s.iab].m += dfdv;  // off-diag chunk dFdV[a,b]
		dFdV.blocks[s.iba].m += dfdv;  // off-diag chunk dFdV[b,a]
	}

	void SpringNetwork::CalcForces()
	{
		CalcNormals();
		// Collect forces and derivatives:  F,dFdX,dFdV
		dFdX.Zero();
		// was dFdV.InitDiagonal(-damp_air);
		dFdV.InitDiagonal(0.0f);
		F.Init(gravity);
		//was F -= V * damp_air;
		//for(int i=0;i<F.size();i++) {F[i] -= (V[i]-wind) * fabsf(dot(V[i]-wind,N[i]))* damp_air;}
		for(unsigned int i=0;i<F.size();i++) {F[i] -= N[i] * (dot(V[i]-(float3&)wind,N[i]) * damp_air);}
		for(unsigned int i=0;i<springs.size();i++) 
		{
			PreSolveSpring(springs[i]); // will add to F,dFdX,dFdV
		}
	}

	void SpringNetwork::CalcNormals()
	{
		for (auto &n : N) n = float3(0, 0, 0);
		for (unsigned int i = 0; i<tris.size(); i++)
		{
			float3 &v0 = X[tris[i][0]];
			float3 &v1 = X[tris[i][1]];
			float3 &v2 = X[tris[i][2]];
			float3 n = cross(v1-v0,v2-v1);
			N[tris[i][0]] += n;
			N[tris[i][1]] += n;
			N[tris[i][2]] += n;
		}
		for(unsigned int i=0;i<quads.size();i++)
		{
			float3 &v0 = X[quads[i][0]];
			float3 &v1 = X[quads[i][1]];
			float3 &v2 = X[quads[i][2]];
			float3 &v3 = X[quads[i][2]];
			float3 n = cross(v1-v0,v2-v1) + cross(v3-v2,v0-v3);
			N[quads[i][0]] += n;
			N[quads[i][1]] += n;
			N[quads[i][2]] += n;
			N[quads[i][3]] += n;
		}
		for (auto &n : N) n = normalize(n);
	}

	int SpringNetwork::PointStatusSet(int index, int op)
	{
		if (index<0 || index >(int)X.size()) return -1;// "no current point";
		// it mike be better to have one key for fixing and a different one for freeing 
		int status = 0;
		for (unsigned int i = 0; i<S.blocks.size(); i++) //PinCheck
		if (S.blocks[i].r == index)
			status = 1;
		//if(op==-1) return status;
		if (status && (op == 0 || op == 2)) // PinClear
		{
			int i = S.blocks.size();
			while (i--)
			if (S.blocks[i].r == index)
				S.blocks.erase(S.blocks.begin() + i); // Deletes entry at index i
			status = 0;
		}
		if (!status && (op == 1 || op == 2)) // PinSet
		{
			S.blocks.insert(S.blocks.begin(), float3Nx3N::Block(index, index));   // put at beginning
			V[index] = float3(0, 0, 0);
			status = 1;
		}
		M[index] = (status) ? 0.0f : 1.0f;  // invmass to zero if point is "set"
		return status; // no change;
	}

	void SpringNetwork::Simulate()
	{
		// Get ready for conjugate gradient iterative solver step.
		// Initialize operands.
		//	if(!awake) return;
		//float dt = (dt_fixed<=0.0f)?dt_fixed : _dt ;  // use value specified here
		if(dt<=0.0f) return;
		CalcForces(); 
		int n=X.size();  // all our big vectors are of this size
		float3N dFdXmV(n);  // temp to store result of matrix multiply
		float3N B(n);  
		dV.Zero();
		for(unsigned int i=0;i<S.blocks.size();i++)
			V[S.blocks[i].c] = float3(0,0,0);
		//int scount=ProcessCollisionConstraints();    // Adds constraints to S
		A.Identity();                  // build up the big matrix we feed to solver
		//A -= dFdV * dt +  dFdX * (dt*dt) ;  
		Msub(A, dFdV, dt, dFdX, dt*dt); //  A -= dFdV * dt + dFdX * (dt*dt);
		dFdXmV = dFdX * V;
		B = F * dt + dFdXmV * (dt*dt);
	//	if(simd) ConjGradientFilteredSIMD(dV,A,B,S);   // Call solver to compute dV = inverse(A)*B subject to S
	//	else 
		ConjGradientFiltered(dV,A,B,S,H);   // Call solver to compute dV = inverse(A)*B subject to S
		//S.blocks.resize(S.blocks.size() - scount);           // remove any temporary constraints
		H.clear();
		V = V + dV;
		X = X + V*dt;
		for(unsigned int i=0;i<S.blocks.size();i++)
			V[S.blocks[i].c] = float3(0,0,0);
		//UpdateLimits();  // extents of object
		awake = (dot(V,V)<sleepthreshold)?awake-1:awake=sleepcount; 
	}
};




inline SpringNetwork SpringNetworkCreate(float points[][3], int points_count,
	SpringNetwork::Spring springs[], int springs_count,
	int tris[][3], int tris_count,
	int quads[][4], int quads_count)
{
	// Handmade 
	SpringNetwork sn(points_count);
	assert(sn.X.size() == points_count);
	for (int i = 0; i<points_count; i++)
		sn.X[i] = (float3&)points[i];
	for (int i = 0; i<springs_count; i++)
		sn.CreateSpring(springs[i].type, springs[i].a, springs[i].b, springs[i].restlen);
	for (int i = 0; i<tris_count; i++)
		sn.tris.push_back((int3&)tris[i]);
	for (int i = 0; i<quads_count; i++)
		sn.quads.push_back((int4&)quads[i]);
	//sn->UpdateLimits();
	return sn;
}


inline SpringNetwork* SpringNetworkCreate(float points[][3], int points_count,
	int tris[][3], int tris_count, SpringNetwork* sn)
{
	if (!sn) sn = new SpringNetwork(points_count);
	for (int i = 0; i<points_count; i++)
		sn->X[i] = (float3&)points[i];
	for (int i = 0; i<tris_count; i++)
		sn->tris.push_back((int3&)tris[i]);
	for (int i = 0; i<tris_count; i++) // generates springs from triangles
	 for (int j = 0; j<3; j++)
		sn->CreateSpring(SPRING_STRUCT, tris[i][j], tris[i][(j + 1) % 3]);
	for (int ia = 0; ia<tris_count; ia++) // generates bend springs from adjacent triangles
	 for (int ja = 0; ja<3; ja++)
	  for (int ib = 0; ib<tris_count; ib++)
	   for (int jb = 0; jb<3; jb++)
	    if (tris[ia][ja] == tris[ib][(jb + 1) % 3] && tris[ia][(ja + 1) % 3] == tris[ib][jb])
	     sn->CreateSpring(SPRING_BEND, tris[ia][(ja + 2) % 3], tris[ib][(jb + 2) % 3]);
	return sn;
}



inline SpringNetwork SpringNetworkCreateRectangular(int w, int h, float size)
{
	int cloth_pincorners = 4 + 8;  // could be tweakable parameters 
	float cloth_pretension = 1.0f;
	// simple spring network generation routine that creates a typical square patch.
	int i, j;
	std::vector<SpringNetwork::Spring> springs;
	std::vector<float3> points;
	std::vector<int4> quads;
	for (i = 0; i<h; i++)
	 for (j = 0; j<w; j++)
	{
		points.push_back(float3(0, 0, 0) + (float3(-0.5f, -0.5f, -0.0f) + float3((float)j / (w - 1.0f), (float)i / (w - 1.0f), 0)) * size);
		//points.push_back(float3(0,0,0)+(float3(0,-0.5f,-0.0f)+float3(0,(float)j/(w-1.0f),0.0f-(float)i/(w-1.0f))) * size);
	}
	float r = magnitude(points[0] - points[1])*cloth_pretension; // spring restlength 

	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (i<h - 1)        springs.push_back(SpringNetwork::Spring(SPRING_STRUCT, i*w + j, (i + 1)*w + j, r));     // structural
	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (j<w - 1)        springs.push_back(SpringNetwork::Spring(SPRING_STRUCT, i*w + j, i*w + (j + 1), r));     // structural
	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (j<w - 1 && i<h - 1) springs.push_back(SpringNetwork::Spring(SPRING_SHEAR, i*w + j, (i + 1)*w + (j + 1), r*sqrtf(2.0f))); // shear
	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (j>0 && i<h - 1) springs.push_back(SpringNetwork::Spring(SPRING_SHEAR, i*w + j, (i + 1)*w + (j - 1), r*sqrtf(2.0f))); // shear
	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (i<h - 2)        springs.push_back(SpringNetwork::Spring(SPRING_BEND, i*w + j, (i + 2)*w + j, r*2.0f));     // benders
	for (i = 0; i<h; i++) for (j = 0; j<w; j++)
	if (j<w - 2)        springs.push_back(SpringNetwork::Spring(SPRING_BEND, i*w + j, i*w + (j + 2), r*2.0f));     // benders

	for (i = 0; i<h - 1; i++)
	 for (j = 0; j<w - 1; j++)
	{
		quads.push_back(int4((i + 0)*w + (j + 0), (i + 0)*w + (j + 1), (i + 1)*w + (j + 1), (i + 1)*w + (j + 0)));
	}
	SpringNetwork sn = SpringNetworkCreate((float(*)[3])points.data(), points.size(), springs.data(), springs.size(), 0, 0, (int(*)[4])quads.data(), quads.size());
	if (cloth_pincorners & 1) sn.PointStatusSet(0, 1); // S.blocks.push_back(float3Nx3N::Block(0,0)).m=float3x3(0,0,0,0,0,0,0,0,0);
	if (cloth_pincorners & 2) sn.PointStatusSet(w - 1, 1); // S.blocks.push_back(float3Nx3N::Block(w-1,w-1)).m=float3x3(0,0,0,0,0,0,0,0,0);
	if (cloth_pincorners & 4) sn.PointStatusSet((h - 1)*w, 1);
	if (cloth_pincorners & 8) sn.PointStatusSet(h*w - 1, 1);
	return sn;
}

inline void SpringNetworkDrawSprings(SpringNetwork *_c, void(*Line)(const float3 &v0, const float3 &v1, const float3 &color))
{
	SpringNetwork *c = dynamic_cast<SpringNetwork*>(_c);
	if(!c) 
		return;
	static const float3 color[3]={float3(1,1,0),float3(1,0,1),float3(0,1,1)};
	float3N &X = c->X;
	for(unsigned int i=0;i<c->springs.size();i++)
	{
		SpringNetwork::Spring &s = c->springs[i];
		Line(X[s.a],X[s.b],color[s.type]);
	}
}

#endif // SPRINGNET_H

