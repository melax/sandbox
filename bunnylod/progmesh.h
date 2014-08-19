/*
 *  Progressive Mesh type Polygon Reduction Algorithm
 *  by Stan Melax (c) 1998
 *
 *  The function ProgressiveMesh() takes a model in an "indexed face 
 *  set" sort of way.  i.e. Array of vertices and Array of triangles.
 *  The function then does the polygon reduction algorithm
 *  internally and reduces the model all the way down to 0
 *  vertices and then returns the order in which the
 *  vertices are collapsed and to which neighbor each vertex
 *  is collapsed to.  More specifically the returned "permutation"
 *  indicates how to reorder your vertices so you can render
 *  an object by using the first n vertices (for the n 
 *  vertex version).  After permuting your vertices, the
 *  map Array indicates to which vertex each vertex is collapsed to.
 */

#ifndef PROGRESSIVE_MESH_H
#define PROGRESSIVE_MESH_H

#include "../include/vecmatquat_minimal.h"
#include "array.h"

class tridata {
  public:
	int	v[3];  // indices to vertex Array
	// texture and vertex normal info removed for this demo
};

void ProgressiveMesh(Array<float3> &vert,  Array<tridata> &tri, 
                     Array<int> &map,  Array<int> &permutation );

#endif
