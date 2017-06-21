//
//  misc files are temporary home for convenience routines not yet sure about convention or where it goes.
//  json support code for our linalg types
//


#pragma once
#ifndef SANDBOX_MISC_JSON_H
#define SANDBOX_MISC_JSON_H

//-----------------------

#include "json.h"
#include "geometric.h"



// Serialize vectors and matrices as JSON arrays
template<class T> json::value to_json(const linalg::vec<T, 2> & vec) { return json::array{to_json(vec.x), to_json(vec.y)}; }
template<class T> json::value to_json(const linalg::vec<T, 3> & vec) { return json::array{to_json(vec.x), to_json(vec.y), to_json(vec.z)}; }
template<class T> json::value to_json(const linalg::vec<T, 4> & vec) { return json::array{to_json(vec.x), to_json(vec.y), to_json(vec.z), to_json(vec.w)}; }
template<class T, int M> json::value to_json(const linalg::mat<T, M, 2> & mat) { return json::array{to_json(mat.x), to_json(mat.y)}; }
template<class T, int M> json::value to_json(const linalg::mat<T, M, 3> & mat) { return json::array{to_json(mat.x), to_json(mat.y), to_json(mat.z)}; }
template<class T, int M> json::value to_json(const linalg::mat<T, M, 4> & mat) { return json::array{to_json(mat.x), to_json(mat.y), to_json(mat.z), to_json(mat.w)}; }

template<class T> void from_json(linalg::vec<T, 2> & vec, const json::value & val) { from_json(vec.x, val[0]); from_json(vec.y, val[1]); }
template<class T> void from_json(linalg::vec<T, 3> & vec, const json::value & val) { from_json(vec.x, val[0]); from_json(vec.y, val[1]); from_json(vec.z, val[2]); }
template<class T> void from_json(linalg::vec<T, 4> & vec, const json::value & val) { from_json(vec.x, val[0]); from_json(vec.y, val[1]); from_json(vec.z, val[2]); from_json(vec.w, val[3]); }
template<class T, int M> void from_json(linalg::mat<T, M, 2> & mat, const json::value & val) { from_json(mat.x, val[0]); from_json(mat.y, val[1]); }
template<class T, int M> void from_json(linalg::mat<T, M, 3> & mat, const json::value & val) { from_json(mat.x, val[0]); from_json(mat.y, val[1]); from_json(mat.z, val[2]); }
template<class T, int M> void from_json(linalg::mat<T, M, 4> & mat, const json::value & val) { from_json(mat.x, val[0]); from_json(mat.y, val[1]); from_json(mat.z, val[2]); from_json(mat.w, val[3]); }

json::value to_json(const Pose &pose) { auto &p = pose.position; auto &o = pose.orientation; return json::array{to_json(p.x), to_json(p.y), to_json(p.z), to_json(o.x), to_json(o.y), to_json(o.z), to_json(o.w)}; }
void from_json(Pose &pose, const json::value & val) { for (auto i : { 0,1,2 }) from_json(pose.position[i], val[i]); for (auto i : { 0,1,2,3 }) from_json(pose.orientation[i], val[3 + i]); }



#endif // SANDBOX_MISC_JSON_H

