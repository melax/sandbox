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

// fixme: basepathname and fileprefix are just two attempts to implemement the same thing

std::string basepathname(std::string fname) { return std::string(fname.begin(), fname.begin() + fname.find_last_of('.')); } // FIXME  stl string newb  not sure if correct if no '.' exists
inline bool fileexists(std::string filename) { std::ifstream i(filename, std::ifstream::in);  return i.is_open(); }

inline const char* strstp(const char *s, char c) { while (*s && *s != c) s++; return s; }  // pointer to first c in string s or end of string if not found
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







#endif // SANDBOX_MISC_H

