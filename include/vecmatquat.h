//
//  vecmatquat (complete version)
//
//  defines the same set of structs as the alternative file vecmatquat_minimal.h.   
//  The vecmatquat_minimal.h version is meant to enable copy-paste just the code snippets that are needed.
//
// Common math types and functions, based closely on Sterling Orsten's BSD-licensed C++ Utilities library (https://github.com/sgorsten/copper)
// As a general rule, this file should only contain types which are generic, mathematical in nature, and free of non-mathematical semantics. 
// For instance, we do not have specific subtypes for concepts like coordinates, directions, or colors. They are all supported via basic vectors, 
// with a rich family of supporting functions. Even quaternions are not represented by their own type, but rather by a family of functions which
// can be applied to vec<T,4>s.

#ifdef VECMATQUAT_MINIMAL_H
#error
#endif

#pragma once
#ifndef VECMATQUAT_H
#define VECMATQUAT_H
#define VECMATQUAT_FULL_H

#include <cmath>
#include <cstdint>
#include <functional>
#include <istream>
#include <ostream>

namespace linalg
{
    template<class T, int M> struct vec; // An M element vector, represented as exactly M instances of type T. It is considered to be equivalent to an Mx1 (column) matrix.
    template<class T, int M, int N> struct mat; // An MxN matrix, represented as exactly N columns of type vec<T,M>. Note that this is a COLUMN MAJOR representation.

    template<class T> struct vec<T,2>
    {
        T                       x, y;
                                vec()                               : x(), y() {}
                                vec(T x, T y)                       : x(x), y(y) {}
                                explicit vec(T s)                   : vec(s, s) {}
        T &                     operator [] (int i)                 { return (&x)[i]; } // v[i] retrieves the i'th row
        const T &               operator [] (int i) const           { return (&x)[i]; } // v[i] retrieves the i'th row 
        bool                    operator == (const vec & r) const   { return x == r.x && y == r.y; }
        template<class F> vec   apply(const vec & r, F f) const     { return {f(x,r.x), f(y,r.y)}; }
        template<class F> vec   apply(T r, F f) const               { return {f(x,r), f(y,r)}; }
    };

    template<class T> struct vec<T,3>
    {
        T                       x, y, z;
                                vec()                               : x(), y(), z() {}
                                vec(T x, T y, T z)                  : x(x), y(y), z(z) {}
                                vec(const vec<T,2> & xy, T z)       : vec(xy.x, xy.y, z) {}
                                explicit vec(T s)                   : vec(s, s, s) {}
        T &                     operator [] (int i)                 { return (&x)[i]; } // v[i] retrieves the i'th row
        const T &               operator [] (int i) const           { return (&x)[i]; } // v[i] retrieves the i'th row 
        bool                    operator == (const vec & r) const   { return x == r.x && y == r.y && z == r.z; }
        const vec<T,2> &        xy() const                          { return reinterpret_cast<const vec<T,2> &>(x); }
        vec<T,2> &              xy()                                { return reinterpret_cast<vec<T,2> &>(x); }
        template<class F> vec   apply(const vec & r, F f) const     { return {f(x,r.x), f(y,r.y), f(z,r.z)}; }
        template<class F> vec   apply(T r, F f) const               { return {f(x,r), f(y,r), f(z,r)}; }
    };

    template<class T> struct vec<T,4>
    {
        T                       x, y, z, w;
                                vec()                               : x(), y(), z(), w() {}
                                vec(T x, T y, T z, T w)             : x(x), y(y), z(z), w(w) {}
                                vec(const vec<T,2> & xy, T z, T w)  : vec(xy.x, xy.y, z, w) {}
                                vec(const vec<T,3> & xyz, T w)      : vec(xyz.x, xyz.y, xyz.z, w) {}
                                explicit vec(T s)                   : vec(s, s, s, s) {}
        T &                     operator [] (int i)                 { return (&x)[i]; } // v[i] retrieves the i'th row
        const T &               operator [] (int i) const           { return (&x)[i]; } // v[i] retrieves the i'th row 
        bool                    operator == (const vec & r) const   { return x == r.x && y == r.y && z == r.z && w == r.w; }
        const vec<T,2> &        xy() const                          { return reinterpret_cast<const vec<T,2> &>(x); }
        vec<T,2> &              xy()                                { return reinterpret_cast<vec<T,2> &>(x); }
        const vec<T,3> &        xyz() const                         { return reinterpret_cast<const vec<T,3> &>(x); }
        vec<T,3> &              xyz()                               { return reinterpret_cast<vec<T,3> &>(x); }
        template<class F> vec   apply(const vec & r, F f) const     { return {f(x,r.x), f(y,r.y), f(z,r.z), f(w,r.w)}; }
        template<class F> vec   apply(T r, F f) const               { return {f(x,r), f(y,r), f(z,r), f(w,r)}; }
    };

    template<class T, int M> bool       operator != (const vec<T, M> & a, const vec<T, M> & b)  { return !(a == b); }
    template<class T, int M> vec<T,M>   operator -  (const vec<T,M> & a)                        { return a.apply(T(), [](T x, T) { return -x; }); }
    template<class T, int M> vec<T,M>   vabs        (const vec<T,M> & a)                        { return a.apply(T(), [](T x, T) { return std::abs(x); }); }
    template<class T, int M> vec<T,M>   operator +  (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, std::plus      <T>()); }
    template<class T, int M> vec<T,M>   operator -  (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, std::minus     <T>()); }
    template<class T, int M> vec<T,M>   operator *  (const vec<T,M> & a, T b)                   { return a.apply(b, std::multiplies<T>()); }
    template<class T, int M> vec<T,M>   operator /  (const vec<T,M> & a, T b)                   { return a.apply(b, std::divides   <T>()); }
    template<class T, int M> vec<T,M>   operator *  (T a, const vec<T,M> & b)                   { return b*a; }
    template<class T, int M> vec<T,M> & operator += (vec<T,M> & a, const vec<T,M> & b)          { return a=a+b; }
    template<class T, int M> vec<T,M> & operator -= (vec<T,M> & a, const vec<T,M> & b)          { return a=a-b; }
    template<class T, int M> vec<T,M> & operator *= (vec<T,M> & a, T b)                         { return a=a*b; }
    template<class T, int M> vec<T,M> & operator /= (vec<T,M> & a, T b)                         { return a=a/b; }

    template<class T, int M> vec<T,M>   cmax        (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, [](T x, T y) { return x > y ? x : y; }); }
    template<class T, int M> vec<T,M>   cmin        (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, [](T x, T y) { return x < y ? x : y; }); }
    template<class T, int M> vec<T,M>   cmul        (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, std::multiplies<T>()); }
    template<class T, int M> vec<T,M>   cdiv        (const vec<T,M> & a, const vec<T,M> & b)    { return a.apply(b, std::divides   <T>()); }
    template<class T>        T          cross       (const vec<T,2> & a, const vec<T,2> & b)    { return a.x*b.y - a.y*b.x; }
    template<class T>        vec<T,3>   cross       (const vec<T,3> & a, const vec<T,3> & b)    { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
    template<class T>        T          dot         (const vec<T,2> & a, const vec<T,2> & b)    { return a.x*b.x + a.y*b.y; }
    template<class T>        T          dot         (const vec<T,3> & a, const vec<T,3> & b)    { return a.x*b.x + a.y*b.y + a.z*b.z; }
    template<class T>        T          dot         (const vec<T,4> & a, const vec<T,4> & b)    { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
    template<class T, class S> T        lerp        (const T & a, const T & b, S t)             { return a*(1-t) + b*t; }
    template<class T, int M> T          magnitude   (const vec<T,M> & a)                        { return sqrt(mag2(a)); }
    template<class T, int M> T          mag2        (const vec<T,M> & a)                        { return dot(a,a); }
    template<class T, int M> vec<T,M>   normalize   (const vec<T, M> & a)                       { return a / magnitude(a); }
	template<class T, class S> T        nlerp       (const T & a, const T & b, S t)             { return normalize(lerp(a,b,t)); }

	template<class T, class S> T        qlerp       (const T & a, const T & b, S t)             { return nlerp(a,((dot(a,b)>=0)?b:-b),t); }
    template<class T> vec<T,4>          qconj       (const vec<T,4> & q)                        { return {-q.x,-q.y,-q.z,q.w}; }
    template<class T> vec<T,4>          qmul        (const vec<T,4> & a, const vec<T,4> & b)    { return {a.x*b.w+a.w*b.x+a.y*b.z-a.z*b.y, a.y*b.w+a.w*b.y+a.z*b.x-a.x*b.z, a.z*b.w+a.w*b.z+a.x*b.y-a.y*b.x, a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z}; }
    template<class T> vec<T,3>          qrot        (const vec<T,4> & q, const vec<T,3> & v)    { return qxdir(q)*v.x + qydir(q)*v.y + qzdir(q)*v.z; } // qvq*           
    template<class T> vec<T,3>          qxdir       (const vec<T,4> & q)                        { return {q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z, (q.x*q.y+q.z*q.w)*2, (q.z*q.x-q.y*q.w)*2}; } // qrot(q,{1,0,0})
    template<class T> vec<T,3>          qydir       (const vec<T,4> & q)                        { return {(q.x*q.y-q.z*q.w)*2, q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z, (q.y*q.z+q.x*q.w)*2}; } // qrot(q,{0,1,0})
    template<class T> vec<T,3>          qzdir       (const vec<T,4> & q)                        { return {(q.z*q.x+q.y*q.w)*2, (q.y*q.z-q.x*q.w)*2, q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z}; } // qrot(q,{0,0,1})
    template<class T> T	                qangle      (const vec<T,4> & q)                        { return acos(q.w) * 2; }
    template<class T> vec<T,3>          qaxis       (const vec<T,4> & q)                        { auto a = qangle(q); return a < 0.0000001 ? vec<T,3>(1,0,0) : q.xyz()*(1/sin(a/2)); }
    template<class T> mat<T,3,3>        qgetmatrix  (const vec<T,4> & q)                        { return{ qxdir(q), qydir(q), qzdir(q) }; }

    template<class T, int M> struct mat<T,M,2>
    {
        typedef vec<T,M>        U;
        U                       x, y;
                                mat()                               : x(), y() {}
                                mat(U x, U y)                       : x(x), y(y) {}
        U &                     operator [] (int j)                 { return (&x)[j]; } // M[j] retrieves the j'th column
        const U &               operator [] (int j) const           { return (&x)[j]; } // M[j] retrieves the j'th column
        vec<T,2>                row(int i) const                    { return {x[i],y[i]}; } // row(i) retrieves the i'th row
        template<class F> mat   apply(const mat & r, F f) const     { return {x.apply(r.x,f), y.apply(r.y,f)}; }
        template<class F> mat   apply(T r, F f) const               { return {x.apply(r,f), y.apply(r,f)}; }
    };

    template<class T, int M> struct mat<T,M,3>
    {
        typedef vec<T,M>        U;
        U                       x, y, z;
                                mat()                               : x(), y(), z() {}
                                mat(U x, U y, U z)                  : x(x), y(y), z(z) {}
        U &                     operator [] (int j)                 { return (&x)[j]; } // M[j] retrieves the j'th column
        const U &               operator [] (int j) const           { return (&x)[j]; } // M[j] retrieves the j'th column
        vec<T,3>                row(int i) const                    { return {x[i],y[i],z[i]}; } // row(i) retrieves the i'th row
        template<class F> mat   apply(const mat & r, F f) const     { return {x.apply(r.x,f), y.apply(r.y,f), z.apply(r.z,f)}; }
        template<class F> mat   apply(T r, F f) const               { return {x.apply(r,f), y.apply(r,f), z.apply(r,f)}; }
    };

    template<class T, int M> struct mat<T,M,4>
    {
        typedef vec<T,M>        U;
        U                       x, y, z, w;
                                mat()                               : x(), y(), z(), w() {}
                                mat(U x, U y, U z, U w)             : x(x), y(y), z(z), w(w) {}
        U &                     operator [] (int j)                 { return (&x)[j]; } // M[j] retrieves the j'th column
        const U &               operator [] (int j) const           { return (&x)[j]; } // M[j] retrieves the j'th column
        vec<T,4>                row(int i) const                    { return {x[i],y[i],z[i],w[i]}; } // row(i) retrieves the i'th row
        template<class F> mat   apply(const mat & r, F f) const     { return {x.apply(r.x,f), y.apply(r.y,f), z.apply(r.z,f), w.apply(r.w,f)}; }
        template<class F> mat   apply(T r, F f) const               { return {x.apply(r,f), y.apply(r,f), z.apply(r,f), w.apply(r,f)}; }
    };

    template<class T, int M, int N> mat<T,M,N>      operator -  (const mat<T,M,N> & a)                          { return a.apply(T(), [](T x, T) { return -x; }); }
    template<class T, int M, int N> mat<T,M,N>      operator +  (const mat<T,M,N> & a, const mat<T,M,N> & b)    { return a.apply(b, std::plus      <T>()); }
    template<class T, int M, int N> mat<T,M,N>      operator -  (const mat<T,M,N> & a, const mat<T,M,N> & b)    { return a.apply(b, std::minus     <T>()); }
    template<class T, int M, int N> mat<T,M,N>      operator *  (const mat<T,M,N> & a, T b)                     { return a.apply(b, std::multiplies<T>()); }
    template<class T, int M, int N> mat<T,M,N>      operator /  (const mat<T,M,N> & a, T b)                     { return a.apply(b, std::divides   <T>()); }
    template<class T, int M, int N> mat<T,M,N> &    operator += (mat<T,M,N> & a, const mat<T,M,N> & b)          { return a=a+b; }
    template<class T, int M, int N> mat<T,M,N> &    operator -= (mat<T,M,N> & a, const mat<T,M,N> & b)          { return a=a-b; }
    template<class T, int M, int N> mat<T,M,N> &    operator *= (mat<T,M,N> & a, T b)                           { return a=a*b; }
    template<class T, int M, int N> mat<T,M,N> &    operator /= (mat<T,M,N> & a, T b)                           { return a=a/b; }

    template<class T, int M>        vec<T,M>        mul         (const mat<T,M,2> & a, const vec<T,2> & b)      { return a.x*b.x + a.y*b.y; }
    template<class T, int M>        vec<T,M>        mul         (const mat<T,M,3> & a, const vec<T,3> & b)      { return a.x*b.x + a.y*b.y + a.z*b.z; }
    template<class T, int M>        vec<T,M>        mul         (const mat<T,M,4> & a, const vec<T,4> & b)      { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
    template<class T, int M, int N> mat<T,M,2>      mul         (const mat<T,M,N> & a, const mat<T,N,2> & b)    { return {mul(a,b.x), mul(a,b.y)}; }
    template<class T, int M, int N> mat<T,M,3>      mul         (const mat<T,M,N> & a, const mat<T,N,3> & b)    { return {mul(a,b.x), mul(a,b.y), mul(a,b.z)}; }
    template<class T, int M, int N> mat<T,M,4>      mul         (const mat<T,M,N> & a, const mat<T,N,4> & b)    { return {mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w)}; }
    template<class T, int M>        mat<T,M,2>      outerprod   (const vec<T,M> & a, const vec<T,2> & b)        { return {a*b.x, a*b.y}; }
    template<class T, int M>        mat<T,M,3>      outerprod   (const vec<T,M> & a, const vec<T,3> & b)        { return {a*b.x, a*b.y, a*b.z}; }
    template<class T, int M>        mat<T,M,4>      outerprod   (const vec<T,M> & a, const vec<T,4> & b)        { return {a*b.x, a*b.y, a*b.z, a*b.w}; }
    template<class T, int M>        mat<T,M,2>      transpose   (const mat<T,2,M> & a)                          { return {a.row(0), a.row(1)}; }
    template<class T, int M>        mat<T,M,3>      transpose   (const mat<T,3,M> & a)                          { return {a.row(0), a.row(1), a.row(2)}; }
    template<class T, int M>        mat<T,M,4>      transpose   (const mat<T,4,M> & a)                          { return {a.row(0), a.row(1), a.row(2), a.row(3)}; }
    template<class T, int N>        mat<T,N,N>      inverse     (const mat<T,N,N> & a)                          { return adjoint(a)/determinant(a); }
    template<class T>               T               determinant (const mat<T,2,2> & a)                          { return a.x.x*a.y.y - a.x.y*a.y.x; }
    template<class T>               T               determinant (const mat<T,3,3> & a)                          { return a.x.x*(a.y.y*a.z.z - a.z.y*a.y.z) + a.x.y*(a.y.z*a.z.x - a.z.z*a.y.x) + a.x.z*(a.y.x*a.z.y - a.z.x*a.y.y); }
    template<class T>               T               determinant (const mat<T,4,4> & a)                          { return a.x.x*(a.y.y*a.z.z*a.w.w + a.w.y*a.y.z*a.z.w + a.z.y*a.w.z*a.y.w - a.y.y*a.w.z*a.z.w - a.z.y*a.y.z*a.w.w - a.w.y*a.z.z*a.y.w) +
                                                                                                                         a.x.y*(a.y.z*a.w.w*a.z.x + a.z.z*a.y.w*a.w.x + a.w.z*a.z.w*a.y.x - a.y.z*a.z.w*a.w.x - a.w.z*a.y.w*a.z.x - a.z.z*a.w.w*a.y.x) +
                                                                                                                         a.x.z*(a.y.w*a.z.x*a.w.y + a.w.w*a.y.x*a.z.y + a.z.w*a.w.x*a.y.y - a.y.w*a.w.x*a.z.y - a.z.w*a.y.x*a.w.y - a.w.w*a.z.x*a.y.y) +
                                                                                                                         a.x.w*(a.y.x*a.w.y*a.z.z + a.z.x*a.y.y*a.w.z + a.w.x*a.z.y*a.y.z - a.y.x*a.z.y*a.w.z - a.w.x*a.y.y*a.z.z - a.z.x*a.w.y*a.y.z); }
    template<class T>               mat<T,2,2>      adjoint     (const mat<T,2,2> & a)                          { return {{a.y.y, -a.x.y}, {-a.y.x, a.x.x}}; }
    template<class T>               mat<T,3,3>      adjoint     (const mat<T,3,3> & a)                          { return {{a.y.y*a.z.z - a.z.y*a.y.z, a.z.y*a.x.z - a.x.y*a.z.z, a.x.y*a.y.z - a.y.y*a.x.z},
                                                                                                                          {a.y.z*a.z.x - a.z.z*a.y.x, a.z.z*a.x.x - a.x.z*a.z.x, a.x.z*a.y.x - a.y.z*a.x.x},
                                                                                                                          {a.y.x*a.z.y - a.z.x*a.y.y, a.z.x*a.x.y - a.x.x*a.z.y, a.x.x*a.y.y - a.y.x*a.x.y}}; }
    template<class T>               mat<T,4,4>      adjoint     (const mat<T,4,4> & a)                          { return {{a.y.y*a.z.z*a.w.w + a.w.y*a.y.z*a.z.w + a.z.y*a.w.z*a.y.w - a.y.y*a.w.z*a.z.w - a.z.y*a.y.z*a.w.w - a.w.y*a.z.z*a.y.w,
                                                                                                                           a.x.y*a.w.z*a.z.w + a.z.y*a.x.z*a.w.w + a.w.y*a.z.z*a.x.w - a.w.y*a.x.z*a.z.w - a.z.y*a.w.z*a.x.w - a.x.y*a.z.z*a.w.w,
                                                                                                                           a.x.y*a.y.z*a.w.w + a.w.y*a.x.z*a.y.w + a.y.y*a.w.z*a.x.w - a.x.y*a.w.z*a.y.w - a.y.y*a.x.z*a.w.w - a.w.y*a.y.z*a.x.w,
                                                                                                                           a.x.y*a.z.z*a.y.w + a.y.y*a.x.z*a.z.w + a.z.y*a.y.z*a.x.w - a.x.y*a.y.z*a.z.w - a.z.y*a.x.z*a.y.w - a.y.y*a.z.z*a.x.w},
                                                                                                                          {a.y.z*a.w.w*a.z.x + a.z.z*a.y.w*a.w.x + a.w.z*a.z.w*a.y.x - a.y.z*a.z.w*a.w.x - a.w.z*a.y.w*a.z.x - a.z.z*a.w.w*a.y.x,
                                                                                                                           a.x.z*a.z.w*a.w.x + a.w.z*a.x.w*a.z.x + a.z.z*a.w.w*a.x.x - a.x.z*a.w.w*a.z.x - a.z.z*a.x.w*a.w.x - a.w.z*a.z.w*a.x.x,
                                                                                                                           a.x.z*a.w.w*a.y.x + a.y.z*a.x.w*a.w.x + a.w.z*a.y.w*a.x.x - a.x.z*a.y.w*a.w.x - a.w.z*a.x.w*a.y.x - a.y.z*a.w.w*a.x.x,
                                                                                                                           a.x.z*a.y.w*a.z.x + a.z.z*a.x.w*a.y.x + a.y.z*a.z.w*a.x.x - a.x.z*a.z.w*a.y.x - a.y.z*a.x.w*a.z.x - a.z.z*a.y.w*a.x.x},
                                                                                                                          {a.y.w*a.z.x*a.w.y + a.w.w*a.y.x*a.z.y + a.z.w*a.w.x*a.y.y - a.y.w*a.w.x*a.z.y - a.z.w*a.y.x*a.w.y - a.w.w*a.z.x*a.y.y,
                                                                                                                           a.x.w*a.w.x*a.z.y + a.z.w*a.x.x*a.w.y + a.w.w*a.z.x*a.x.y - a.x.w*a.z.x*a.w.y - a.w.w*a.x.x*a.z.y - a.z.w*a.w.x*a.x.y,
                                                                                                                           a.x.w*a.y.x*a.w.y + a.w.w*a.x.x*a.y.y + a.y.w*a.w.x*a.x.y - a.x.w*a.w.x*a.y.y - a.y.w*a.x.x*a.w.y - a.w.w*a.y.x*a.x.y,
                                                                                                                           a.x.w*a.z.x*a.y.y + a.y.w*a.x.x*a.z.y + a.z.w*a.y.x*a.x.y - a.x.w*a.y.x*a.z.y - a.z.w*a.x.x*a.y.y - a.y.w*a.z.x*a.x.y},
                                                                                                                          {a.y.x*a.w.y*a.z.z + a.z.x*a.y.y*a.w.z + a.w.x*a.z.y*a.y.z - a.y.x*a.z.y*a.w.z - a.w.x*a.y.y*a.z.z - a.z.x*a.w.y*a.y.z,
                                                                                                                           a.x.x*a.z.y*a.w.z + a.w.x*a.x.y*a.z.z + a.z.x*a.w.y*a.x.z - a.x.x*a.w.y*a.z.z - a.z.x*a.x.y*a.w.z - a.w.x*a.z.y*a.x.z,
                                                                                                                           a.x.x*a.w.y*a.y.z + a.y.x*a.x.y*a.w.z + a.w.x*a.y.y*a.x.z - a.x.x*a.y.y*a.w.z - a.w.x*a.x.y*a.y.z - a.y.x*a.w.y*a.x.z,
                                                                                                                           a.x.x*a.y.y*a.z.z + a.z.x*a.x.y*a.y.z + a.y.x*a.z.y*a.x.z - a.x.x*a.z.y*a.y.z - a.y.x*a.x.y*a.z.z - a.z.x*a.y.y*a.x.z}}; }

    // Variadic multiply functions allow composition of many quaternions or matrices
    template<class T, class... R> vec<T,4> qmul(const vec<T,4> & a, R... r) { return qmul(a, qmul(r...)); }
    template<class T, int M, int N, class... R> auto mul(const mat<T,M,N> & a, R... r) -> decltype(mul(a, mul(r...))) { return mul(a, mul(r...)); }

    // Implement stream operators for vector types, and specifically interpret byte-sized integers as integers instead of characters
    template<class T, int M> std::ostream & operator << (std::ostream & out, const linalg::vec<      T,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << v[i]; return out; }
    template<         int M> std::ostream & operator << (std::ostream & out, const linalg::vec< int8_t,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << (int)v[i]; return out; }
    template<         int M> std::ostream & operator << (std::ostream & out, const linalg::vec<uint8_t,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << (int)v[i]; return out; }
    template<class T, int M> std::istream & operator >> (std::istream & in, linalg::vec<      T,M> & v) { for (int i = 0; i<M; ++i) in >> v[i]; return in; }
    template<         int M> std::istream & operator >> (std::istream & in, linalg::vec< int8_t,M> & v) { for (int i = 0, x; i<M; ++i) if (in >> x) v[i] = (int8_t)x; return in; }
    template<         int M> std::istream & operator >> (std::istream & in, linalg::vec<uint8_t,M> & v) { for (int i = 0, x; i<M; ++i) if (in >> x) v[i] = (uint8_t)x; return in; }
}

typedef linalg::vec<int8_t, 2> byte2;   typedef linalg::vec<int16_t, 2> short2;     typedef linalg::vec<int32_t, 2> int2;       typedef linalg::vec<int64_t, 2> long2;
typedef linalg::vec<int8_t, 3> byte3;   typedef linalg::vec<int16_t, 3> short3;     typedef linalg::vec<int32_t, 3> int3;       typedef linalg::vec<int64_t, 3> long3;
typedef linalg::vec<int8_t, 4> byte4;   typedef linalg::vec<int16_t, 4> short4;     typedef linalg::vec<int32_t, 4> int4;       typedef linalg::vec<int64_t, 4> long4;
typedef linalg::vec<uint8_t,2> ubyte2;  typedef linalg::vec<uint16_t,2> ushort2;    typedef linalg::vec<uint32_t,2> uint2;      typedef linalg::vec<uint64_t,2> ulong2;
typedef linalg::vec<uint8_t,3> ubyte3;  typedef linalg::vec<uint16_t,3> ushort3;    typedef linalg::vec<uint32_t,3> uint3;      typedef linalg::vec<uint64_t,3> ulong3;
typedef linalg::vec<uint8_t,4> ubyte4;  typedef linalg::vec<uint16_t,4> ushort4;    typedef linalg::vec<uint32_t,4> uint4;      typedef linalg::vec<uint64_t,4> ulong4;
typedef linalg::vec<float, 2> float2;   typedef linalg::mat<float, 2,2> float2x2;   typedef linalg::mat<float, 2,3> float2x3;   typedef linalg::mat<float, 2,4> float2x4; 
typedef linalg::vec<float, 3> float3;   typedef linalg::mat<float, 3,2> float3x2;   typedef linalg::mat<float, 3,3> float3x3;   typedef linalg::mat<float, 3,4> float3x4; 
typedef linalg::vec<float, 4> float4;   typedef linalg::mat<float, 4,2> float4x2;   typedef linalg::mat<float, 4,3> float4x3;   typedef linalg::mat<float, 4,4> float4x4;
typedef linalg::vec<double,2> double2;  typedef linalg::mat<double,2,2> double2x2;  typedef linalg::mat<double,2,3> double2x3;  typedef linalg::mat<double,2,4> double2x4; 
typedef linalg::vec<double,3> double3;  typedef linalg::mat<double,3,2> double3x2;  typedef linalg::mat<double,3,3> double3x3;  typedef linalg::mat<double,3,4> double3x4; 
typedef linalg::vec<double,4> double4;  typedef linalg::mat<double,4,2> double4x2;  typedef linalg::mat<double,4,3> double4x3;  typedef linalg::mat<double,4,4> double4x4;

#endif // VECMATQUAT_H