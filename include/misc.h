//
//  Common convenience routines not yet sure about convention or where it goes.
//  Rather than copy-paste code between projects, better to move into this file.
//  Stuff in this file should generally only depend on standard library.  
//  So graphics stuff would go in misc_gl.h instead.
//

#pragma once
#ifndef SANDBOX_MISC_H
#define SANDBOX_MISC_H

//-----------------------


#include <string>
#include <iostream>
#include <fstream>
#include <strstream>
#include <sstream>
#include <vector>

// some misc convenience functions
#if _MSC_VER >= 1900 
template<typename F, typename S> auto Transform(std::vector<S> &src, F f) { std::vector<std::result_of_t<F(S)>> dst(src.size()); std::transform(src.begin(), src.end(), dst.begin(), f); return dst; }
template<typename F, typename S> auto Transform(const std::vector<S> &src, F f) { std::vector<std::result_of_t<F(S)>> dst(src.size()); std::transform(src.begin(), src.end(), dst.begin(), f); return dst; }
#else // for visual studio 2013 support
template <
    template <typename, typename> class Container,
    typename Value,
    typename Allocator = std::allocator<Value>, typename Func >
    auto Transform(const Container<Value, Allocator> & input, const Func &f)
    -> Container<decltype(f(std::declval<Value>())), std::allocator<decltype(f(std::declval<Value>()))>> {
    Container<decltype(f(std::declval<Value>())), std::allocator<decltype(f(std::declval<Value>()))>> ret;
    std::transform(std::begin(input), std::end(input), std::back_inserter(ret), f);
    return ret;
}
#endif

template<class T> std::vector<T> & Append(std::vector<T> &a, const T&  t) { a.push_back(t); return a; }
template<class T> std::vector<T> & Append(std::vector<T> &a,       T&& t) { a.push_back(std::move(t)); return a; }
template<class T> std::vector<T> & Append(std::vector<T> &a, const std::vector<T> &b) { a.insert(a.end(), b.begin(), b.end()); return a; }
template<class T> std::vector<T> & Append(std::vector<T> &a, std::vector<T> &&b)      { for(auto &e:b) a.push_back(std::move(e)); return a; } 
template<class T> std::vector<T*>  Addresses(std::vector<T> &a) { return Transform(a, [](T &t)->T* {return &t; }); }


// fixme: basepathname and fileprefix are just two attempts to implemement the same thing

inline std::string basepathname(std::string fname) { return std::string(fname.begin(), fname.begin() + fname.find_last_of('.')); } // FIXME  stl string newb  not sure if correct if no '.' exists
inline bool fileexists(std::string filename) { std::ifstream i(filename, std::ifstream::in);  return i.is_open(); }
inline std::string filesuffix(std::string fname) { std::string s(fname.begin() + fname.find_last_of('.'), fname.end()); return (s.find('/') == std::string::npos && s.find('\\') == std::string::npos) ? s : ""; }

inline const char* strstp(const char *s, char c) { const char* p = NULL; while (*s) { if (*s == c) p = s; s++; } return p?p:s; }  // pointer to last c in string s or end of string if not found
inline std::string fileprefix(const char *filename) { return std::string(filename, strstp(filename, '.')); }
inline std::string fileprefix(std::string filename) { return fileprefix(filename.c_str()); }

inline std::string freefilename(std::string prefix, std::string suffix)  // find next unused file in sequence
{
	int fid = 0;
	auto fnaming_convention = [prefix, suffix](int fid) { return prefix + std::to_string(fid) + suffix; };
	while (fileexists(fnaming_convention(fid).c_str()))
		fid++;
	return fnaming_convention(fid);
}




//  ToString() - convenience class/function for using << overloads to generate strings inline within a single expression
// Example Usage:   return ToString() << "test" << my_vec3 << " more " << my_float_var << " and so on";
//            or:   set_thermometer_gui_string( ToString() << (temperature*9/5+32) ); 
struct ToString
{
	std::ostringstream o;
	std::string str() { return o.str(); }
	operator std::string() { return o.str(); }
	template<class T>ToString &operator<<(const T &t) { o << t; return *this; }
};

// StringTo and FromString  - convenience class/function for using >> overloads to convert from strings inline within a single expression
//    f(StringTo<float3>("6 7 8"));  // if f() is overloaded specify type with StringTo<>()
//    g(FromString("9"));            // works if g isn't overloaded such as it takes a float
//
template<class T> inline  T StringTo(std::string s) { T v; std::istringstream i(std::move(s)); i >> v; return v; }

struct FromString
{
	const std::string &s;
	FromString(const std::string &s) :s(s) {}
	template<class T> operator T () { return StringTo<T>(s); }
	operator std::string() { return s; }
};




#endif // SANDBOX_MISC_H

