//
//  extention of linalg and geo library for 6D and 6x6 systems
//  includes a common icp use case 
//

#ifndef GEO6_H
#define GEO6_H

#include "../include/geometric.h"  //  this will also include linalg.h 


namespace linalg
{
	template<class T> struct vec<T, 6>
	{
		T                           x, y, z, w, q, r;
		vec() : x(), y(), z(), w(), q(), r() {}
		vec(T x, T y, T z, T w, T q, T r) : x(x), y(y), z(z), w(w), q(q), r(r) {}
		explicit                    vec(T s) : x(s), y(s), z(s), w(s), q(s), r(s) {}
		explicit                    vec(const T * p) : vec(p[0], p[1], p[2], p[3], p[4], p[5]) {}
		template<class U> explicit  vec(const vec<U, 6> & v) : vec(static_cast<T>(v.x), static_cast<T>(v.y), static_cast<T>(v.z), static_cast<T>(v.w), static_cast<T>(v.q), static_cast<T>(v.r)) {}
		vec(const vec<T, 3> & xyz, const vec<T, 3> & wqr) : vec(xyz.x, xyz.y, xyz.z, wqr.x, wqr.y, wqr.z) {}
		const T &                   operator[] (int i) const { return (&x)[i]; }
		T &                         operator[] (int i) { return (&x)[i]; }
		const vec<T, 2> &            xy()  const { return *reinterpret_cast<const vec<T, 2> *>(this); }
		vec<T, 2> &                  xy() { return *reinterpret_cast<vec<T, 2> *>(this); }
		const vec<T, 3> &            xyz() const { return *reinterpret_cast<const vec<T, 3> *>(this); }
		vec<T, 3> &                  xyz() { return *reinterpret_cast<vec<T, 3> *>(this); }
		const vec<T, 3> &            wqr() const { return *reinterpret_cast<const vec<T, 3> *>(&w); }
		vec<T, 3> &                  wqr() { return *reinterpret_cast<vec<T, 3> *>(&w); }
	};
	template<class T, int M> struct mat<T, M, 6>
	{
		typedef vec<T, M>            V;
		V                            x, y, z, w, q, r;
		mat() : x(), y(), z(), w(), q(), r() {}
		mat(V x, V y, V z, V w, V q, V r) : x(x), y(y), z(z), w(w), q(q), r(r) {}
		explicit                    mat(T s) : x(s), y(s), z(s), w(s), q(s), r(s) {}
		explicit                    mat(const T * p) : x(p + M * 0), y(p + M * 1), z(p + M * 2), w(p + M * 3), q(p + M * 4), r(p + M * 5) {}
		template<class U> explicit  mat(const mat<U, M, 6> & m) : mat(V(m.x), V(m.y), V(m.z), V(m.w), V(m.q), V(m.r)) {}
		vec<T, 6>                   row(int i) const { return{ x[i], y[i], z[i], w[i],q[i],r[i] }; }
		const V &                   operator[] (int j) const { return (&x)[j]; }
		V &                         operator[] (int j) { return (&x)[j]; }
	};
	template<class T, int M> mat<T, M, 6> outerprod(const vec<T, M> & a, const vec<T, 6> & b) { return{ a*b.x, a*b.y, a*b.z, a*b.w ,a*b.q,a*b.r }; }

	template<class T, class F> T fold(const vec<T, 6> & a, F f) { return f(f(f(f(f(a.x, a.y), a.z), a.w), a.q), a.r); }
	template<class T, class F> auto zip(const vec<T, 6  > & a, const vec<T, 6  > & b, F f) -> vec<decltype(f(T(), T())), 6  > { return{ f(a.x,b.x), f(a.y,b.y), f(a.z,b.z), f(a.w,b.w), f(a.q,b.q), f(a.r,b.r) }; }
	template<class T, int M, class F> auto zip(const mat<T, M, 6> & a, const mat<T, M, 6> & b, F f) -> mat<decltype(f(T(), T())), M, 6> { return{ zip(a.x,b.x,f), zip(a.y,b.y,f), zip(a.z,b.z,f), zip(a.w,b.w,f), zip(a.q,b.q,f), zip(a.r,b.r,f) }; }
	template<class T, int M> vec<T, M> mul(const mat<T, M, 6> & a, const vec<T, 6> & b) { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w + a.q*b.q + a.r*b.r; }
	template<class T, int M, int N> mat<T, M, 6> mul(const mat<T, M, N> & a, const mat<T, N, 6> & b) { return{ mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w),mul(a,b.q),mul(a,b.r) }; }
	template<class T, int M> mat<T, M, 6> transpose(const mat<T, 6, M> & m) { return{ m.row(0), m.row(1), m.row(2), m.row(3), m.row(4), m.row(5) }; }

}; // namespace linalg

typedef linalg::vec<float , 6   > float6;
typedef linalg::mat<float , 6, 6> float6x6;
typedef linalg::vec<double, 6   > double6;
typedef linalg::mat<double, 6, 6> double6x6;


inline float6 ConjGradient(const float6x6 &A, const float6 &B) // Solves for unknown x in equation Ax=B
{
	float6 x(0.0f);
	float conjgrad_epsilon = 0.00000002f;
	int   conjgrad_looplimit = 200;

	int conjgrad_loopcount = 0;
	float6 q(0.0f), d(0.0f), tmp(0.0f), r(0.0f);
	//r = B - A*X;   // just set r to B if X known to be zero
	tmp = mul(A, x);
	r = B - tmp;
	d = r;
	float s = dot(r, r);
	float starget = s * conjgrad_epsilon*conjgrad_epsilon;
	while (s>starget && conjgrad_loopcount++ < conjgrad_looplimit)
	{
		q = mul(A, d);
		float a = s / dot(d, q);
		x = x + d*a;
		if (conjgrad_loopcount % 10 == 0)
		{
			tmp = mul(A, x);  // r = B - A*X   // Mul(tmp,A,X); 
			r = B - tmp;

		}
		else
			r = r - q*a;

		float s_prev = s;
		s = dot(r, r);
		d = r + d*(s / s_prev);
	}
	//conjgrad_lasterror = s;
	return x;  // true means we reached desired accuracy in given time - ie stable
}

template<class T>
linalg::vec<T, 6> Cholesky(const linalg::mat<T, 6, 6> &A, const linalg::vec<T, 6> &b)
{
	// equiv to LLT from Eigen. Adapted from dlib,  this routine adapted from Leo's code
	// theory  https://en.wikipedia.org/wiki/Cholesky_decomposition 
	linalg::mat<T, 6, 6> L;
	
	linalg::vec<T, 6> z ;

	linalg::vec<T, 6> xA;

	T maxd = 0.0f;
	for (int i = 0; i < 6; i++)
		maxd = std::max<T>(maxd, std::abs(A[i][i]));

	const double eps = maxd*std::sqrt(std::numeric_limits<T>::epsilon()) / 100; //  1e-3;
	bool isspd = true;

	// Main loop.
	for (int j = 0; j < 6; j++) {
		double d(0.0);
		for (int k = 0; k < j; k++) {
			T s(0.0);
			for (int i = 0; i < k; i++) {
				s += L[k][i] * L[j][i];
			}

			// if L_(k,k) != 0
			if (std::abs(L[k][k]) > eps)
			{
				s = (A[j][k] - s) / L[k][k];
			}
			else {
				s = (A[j][k] - s);
				isspd = false;
			}
			L[j][k] = s;
			d = d + s*s;
			// this is approximately doing: isspd = isspd && ( A(k,j) == A(j,k))
			isspd = isspd && (std::abs(A[k][j] - A[j][k]) < eps);
		}
		d = A[j][j] - d;
		isspd = isspd && (d > eps);
		L[j][j] = (T)sqrt(d > 0.0f ? d : 0.0f);  // cast to templated precision type  float vs double
		for (int k = j + 1; k < 6; k++)
		{
			L[j][k] = 0.0;
		}
	}
	//forw sub
	for (int i = 0; i < 6; i++) {
		z[i] = b[i];
		for (int j = 0; j < i; j++)
		{
			z[i] -= L[i][j] * z[j];
		}
		z[i] /= L[i][i];
	}

	auto LT = linalg::transpose(L);

	//back sub
	for (int i = 6 - 1; i >= 0; i--) {
		xA[i] = z[i];
		for (int j = i + 1; j < 6; j++) {
			xA[i] -= LT[i][j] * xA[j];
		}
		xA[i] /= LT[i][i];
	}
	if (!isspd)
		return{ 0,0,0,0,0,0 };
	return xA; 
}

static float2 g_separation_change;  // just for testing

struct icp_correspondence { float3 point;float4 plane; };
inline Pose ICP(const std::vector<icp_correspondence> &correspondences)  // single iteration of ICP, caller provides the correspondences
{
	float6   b = { 0,0,0,0,0,0 };   // xyz or indices 0 1 2 used for rotation part  wqr or indices 3 4 5 used for trans part
	float6x6 m(0.0f);               // that accumulated matrix  transpose(A) * A 
	float    sep = 0.0f;            // a geometric separation distance check just for curiousity and testing
	for (auto &c : correspondences)
	{
		auto  &t = c.plane; auto v = c.point;
		auto   s = dot(t, float4(v, 1.0f));            // distance v is above plane.  here s is like the 'b' in kinfu paper equation 26
		auto cvr = float6(cross(v, t.xyz()), t.xyz()); // nice that a lot of that complicated maths simply boils down to just this 
		m   += outerprod(cvr, cvr);                      
		b   += cvr * s;
		sep += std::abs(s) / correspondences.size();  // just for testing
	}
	auto t = Cholesky(m, b);       // solves for t where  m t = b    most papers use 'x' instead of 't' when writing this equation 
	if (!dot(t, t))
		return Pose(); // t = ConjGradient(m, b);
	auto dpose = Pose(float3(t.w, t.q, t.r), normalize(float4(linalg::sin(float3(t.xyz()) / 2.0f), 1.0f))).inverse();  
	// remainder of this function is just for testing - not essential to algorithm
	auto test = mul(m, t);  
	auto diff = b - test;
	auto t2 = ConjGradient(m, b);
	auto test2 = mul(m, t2);
	auto diff2 = b - test2;
	float postsep = 0.0f;
	for(auto &c:correspondences)
		postsep += std::abs(dot(dpose.inverse().TransformPlane(c.plane), float4(/* dpose* */c.point, 1.0f))) / correspondences.size();;
	g_separation_change = { sep,postsep };
	return dpose;
}


#endif  // GEO6_H
