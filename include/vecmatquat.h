//
//  vecmatquat (complete version)
//
//  defines the same set of structs as the alternative file vecmatquat_minimal.h.   
//  The vecmatquat_minimal.h version is meant to enable copy-paste just the code snippets that are needed.
//
// This file is meant as a transitionary stub to assist in moving to linalg.h v2.0. It defines aliases for
// functions and types whose names have changed, and retains the iostream operators that are not currently
// part of linalg.h v2.0.

#ifdef VECMATQUAT_MINIMAL_H
#error
#endif

#pragma once
#ifndef VECMATQUAT_H
#define VECMATQUAT_H
#define VECMATQUAT_FULL_H

#include "linalg.h"
using namespace linalg::aliases; // vecmatquat.h defined aliases (float3, etc.) in the global namespace

// These vecmatquat.h names differ from their equivalents in linalg.h v2.0, route them to the new functions
template<class T, int M> linalg::vec<T,M>   vabs        (const linalg::vec<T,M> & a)                                { return abs(a); }
template<class T, int M> linalg::vec<T,M>   cmax        (const linalg::vec<T,M> & a, const linalg::vec<T,M> & b)    { return max(a,b); }
template<class T, int M> linalg::vec<T,M>   cmin        (const linalg::vec<T,M> & a, const linalg::vec<T,M> & b)    { return min(a,b); }
template<class T, int M> linalg::vec<T,M>   cmul        (const linalg::vec<T,M> & a, const linalg::vec<T,M> & b)    { return a*b; }
template<class T, int M> linalg::vec<T,M>   cdiv        (const linalg::vec<T,M> & a, const linalg::vec<T,M> & b)    { return a/b; }
template<class T, int M> T                  magnitude   (const linalg::vec<T,M> & a)                                { return length(a); }
template<class T, int M> T                  mag2        (const linalg::vec<T,M> & a)                                { return length2(a); }
template<class T, class S> T                qlerp       (const T & a, const T & b, S t)                             { return qnlerp(a,b,t); }
template<class T> linalg::mat<T,3,3>        qgetmatrix  (const linalg::vec<T,4> & q)                                { return qmat(q); }
typedef linalg::aliases::byte2 ubyte2;
typedef linalg::aliases::byte3 ubyte3;
typedef linalg::aliases::byte4 ubyte4;

// Implement stream operators for vector types, and specifically interpret byte-sized integers as integers instead of characters
#include <iosfwd>
template<class T, int M> std::ostream & operator << (std::ostream & out, const linalg::vec<      T,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << v[i]; return out; }
template<         int M> std::ostream & operator << (std::ostream & out, const linalg::vec< int8_t,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << (int)v[i]; return out; }
template<         int M> std::ostream & operator << (std::ostream & out, const linalg::vec<uint8_t,M> & v) { for (int i = 0; i<M; ++i) out << (i ? " " : "") << (int)v[i]; return out; }
template<class T, int M> std::istream & operator >> (std::istream & in, linalg::vec<      T,M> & v) { for (int i = 0; i<M; ++i) in >> v[i]; return in; }
template<         int M> std::istream & operator >> (std::istream & in, linalg::vec< int8_t,M> & v) { for (int i = 0, x; i<M; ++i) if (in >> x) v[i] = (int8_t)x; return in; }
template<         int M> std::istream & operator >> (std::istream & in, linalg::vec<uint8_t,M> & v) { for (int i = 0, x; i<M; ++i) if (in >> x) v[i] = (uint8_t)x; return in; }

#include <functional> // Not needed to implement linalg.h v2.0, but was used by vecmatquat.h, and needed by some #includers of this file

#endif // VECMATQUAT_H