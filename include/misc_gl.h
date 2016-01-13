
#pragma once
#ifndef MISC_GL_H
#define MISC_GL_H

#include "vecmatquat.h"
#include "glwin.h"
#include "mesh.h"

//---------- mesh draw gl  --------------

void drawimage(const ubyte3* imagedata, int2 dim, float2 p, float2 s, int tid = 0)
{
	if (!imagedata)
		return;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, tid);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dim.x, dim.y, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);

	glBegin(GL_QUADS);
	glColor3f(1, 1, 1);
	float2 corners[] = { { 0.0f, 0.0f },{ 1.0f,0.0f },{ 1.0f,1.0f },{ 0.0f,1.0f } };
	for (float2 c : corners)
		glTexCoord2f(c.x, c.y), glVertex3fv(float3(p + cmul(c, s), 0.0f));
	glEnd();


	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
}

void progressbar(float t, bool highlight)
{
	float2 position(0.0f, 0.0f), size(1.0f, 0.05f), gap(0.01f, 0.01f);
	ubyte3 boost = highlight ? ubyte3(0, 50, 50) : ubyte3(0, 0, 0);
	std::vector<ubyte3> image(1000, ubyte3(0, 103, 103) + boost);  // init to something not red
	for (int i = 0; i < (int)(t * 1000) && i < 1000; i++)
		image[i] = ubyte3(255, 52, 52) + boost;  // make the portion left of time t more red
	drawimage(image.data(), { 1000,1 }, position + gap, size - gap*2.0f);
}


inline void drawpoints(std::vector<float3> &pts, float3 color = float3(1, 1, 1), float size = 1.0f)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	if (size != 1.0f) glPointSize(size);
	glBegin(GL_POINTS);
	glColor3fv(color);
	for (auto &p : pts)
		glVertex3fv(p);
	glEnd();
	glPopAttrib();
}

inline std::vector<std::pair<float3, float3>> wirefrustrum(float2 fov, float zmin, float zmax)  // generates the 12 lines of the cropped pyramid (eg view-volume frustrum) specified by field of view and range
{
	float3 cnr = { tanf(fov.y / 2.0f), tanf(fov.y / 2.0f), 1.0f };
	std::vector<std::pair<float3, float3>> lines;
	float3 generator[] = { { 1, 1, 1 },{ -1, 1, 1 },{ -1, -1, 1 },{ 1, -1, 1 } };  // ccw
	for (int i = 0; i < 4; i++)
	{
		lines.push_back(std::pair<float3, float3>(cmul(generator[i], cnr)*zmin, cmul(generator[i], cnr)*zmax));
		for (float z : {zmin, zmax})
			lines.push_back(std::pair<float3, float3>(cmul(generator[i], cnr)*z, cmul(generator[(i + 1) % 4], cnr)*z));
	}
	return lines;
}

inline void drawlines(const std::vector<std::pair<float3, float3>> &lines, float3 color = { 1, 1, 1 }, float linewidth = 1.0f)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(linewidth);
	glBegin(GL_LINES);
	glColor3fv(color);
	for (auto l : lines)
		glVertex3fv(l.first), glVertex3fv(l.second);
	glEnd();
	glLineWidth(1.0f);  // pop attrib should take care of this
	glPopAttrib();
}
void drawfrustrum(float2 fov, float zmin, float zmax, float linewidth = 1.0f) { drawlines(wirefrustrum(fov, zmin, zmax), { 1, 1, 1 }, linewidth); }


inline void MeshDraw(const Mesh &mesh)  
{
	glPushMatrix();
	glMultMatrixf(mesh.pose.Matrix());
	auto emitvert = [](const Vertex &v) ->void {glNormal3fv(qzdir(v.orientation)); glTexCoord2fv(v.texcoord); glVertex3fv(v.position); };
	glBegin(GL_TRIANGLES);
	for (auto t : mesh.tris) for (int i = 0; i < 3; i++)
		emitvert(mesh.verts[t[i]]);
	glEnd();
	glPopMatrix();
}



#endif // MISC_GL_H
