
#pragma once
#ifndef MISC_GL_H
#define MISC_GL_H

#include "geometric.h"
#include "glwin.h"
#include "mesh.h"

#include <assert.h>

void drawimage(const byte3* imagedata, int2 dim, float2 p, float2 s, int tid = 0)
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
		glTexCoord2f(c.x, c.y), glVertex3fv(float3(p + c*s, 0.0f));
	glEnd();


	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
}
inline void drawimage(const std::vector<byte3> image, int2 dim, float2 p, float2 s, int tid = 0) { drawimage(image.data(), dim, p, s, tid); }
inline void drawimage(const std::vector<unsigned char> image, int2 dim, float2 p, float2 s, int tid = 0) { drawimage(Transform(image, [](unsigned char c) {return byte3(c);}), dim, p, s, tid); }
void drawimage(const std::pair<const std::vector<byte3>&, int2> im, float2 p, float2 s, int tid = 0) { drawimage(im.first, im.second, p, s, tid); }
void drawimage(const std::pair<const std::vector<unsigned char>&, int2> im, float2 p, float2 s, int tid = 0) { drawimage(im.first, im.second, p, s, tid); }

void progressbar(float t, bool highlight)
{
	float2 position(0.0f, 0.0f), size(1.0f, 0.05f), gap(0.01f, 0.01f);
	byte3 boost = highlight ? byte3(0, 50, 50) : byte3(0, 0, 0);
	std::vector<byte3> image(1000, byte3(boost+ byte3( 0, 103, 103 ) ));  // init to something not red
	for (int i = 0; i < (int)(t * 1000) && i < 1000; i++)
		image[i] = byte3(byte3(255, 52, 52) + boost);  // make the portion left of time t more red
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
		lines.push_back(std::pair<float3, float3>((generator[i]* cnr)*zmin, (generator[i]* cnr)*zmax));
		for (float z : {zmin, zmax})
			lines.push_back(std::pair<float3, float3>((generator[i]* cnr)*z, (generator[(i + 1) % 4]* cnr)*z));
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

inline void glGridxy(float r, float3 c = { 0, 1, 0 }, const Pose &pose = { { 0,0,0 },{ 0,0,0,1 } })
{
	glPushMatrix();
	glMultMatrixf(pose.matrix());
	glColor3fv(c);
	glBegin(GL_LINES);
	glColor3fv({ 0.25f, 0.25f, 0.25f });
	for (float t = -4; t <= 4; t += 1.0f)
	{
		glVertex3fv(float3(t, -4.0f, 0)*r / 4.0f); glVertex3fv(float3(t, 4.0f, 0)*r / 4.0f);
		glVertex3fv(float3(-4.0f, t, 0)*r / 4.0f); glVertex3fv(float3(4.0f, t, 0)*r / 4.0f);
	}
	glEnd();
	glPopMatrix();
}

void gldraw(const std::vector<float3> &verts, const std::vector<int3> &tris)
{
	glBegin(GL_TRIANGLES);
	glColor4f(1, 1, 1, 0.25f);
	for (auto t : tris)
	{
		auto n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		glNormal3fv(n); auto vn = abs(n);
		int k = argmax(&vn.x, 3);
		for (int j = 0; j < 3; j++)
		{
			const auto &v = verts[t[j]];
			glTexCoord2f(v[(k + 1) % 3], v[(k + 2) % 3]);
			glVertex3fv(v);
		}
	}
	glEnd();
}

inline void MeshDraw(const Mesh &mesh)  
{
	glPushMatrix();
	glMultMatrixf(mesh.pose.matrix());
	auto emitvert = [](const Vertex &v) ->void {glNormal3fv(qzdir(v.orientation)); glTexCoord2fv(v.texcoord); glVertex3fv(v.position); };
	glBegin(GL_TRIANGLES);
	for (auto t : mesh.tris) for (int i = 0; i < 3; i++)
		emitvert(mesh.verts[t[i]]);
	glEnd();
	glPopMatrix();
}

inline void glcolorbox(const float3 &r, const Pose &p)   // p = { { 0, 0, 0 }, { 0, 0, 0, 1 } })
{
	glPushMatrix();
	glMultMatrixf(p.matrix());
	glBegin(GL_QUADS);
	for (int m : {0, 1}) for (int i : {0, 1, 2})
	{
		int i1 = (i + 1 + m) % 3;
		int i2 = (i + 2 - m) % 3;
		float3 u, v, w;
		u[i1] = r[i1];
		v[i2] = r[i2];
		w[i] = (m) ? -1.0f : 1.0f;
		float3 a((float)m, (float)m, (float)m);
		a[i] = 1 - a[i];
		glColor3fv(a);
		glNormal3fv(w);
		float2 corners[] = { { -1.0f, -1.0f },{ 1.0f, -1.0f },{ 1.0f, 1.0f },{ -1.0f, 1.0f } };  // ccw order
		for (float2 t : corners)
			glTexCoord2fv(t), glVertex3fv(w*r[i] + u*t.x + v*t.y);
	}
	glEnd();
	glPopMatrix();
}

inline void glwirebox(std::function<float3(int)> p) { glBegin(GL_LINES); for (auto e : boxedges()) glVertex3fv(p(e[0])), glVertex3fv(p(e[1])); glEnd(); }
inline void glwirebox(std::vector<float3> verts) { assert(verts.size() == 8); glwirebox([&verts](int i) {return verts[i]; }); }
inline void glwirefrustumz(const float2x2 &corners, const float2 &zrange) { glwirebox([&](int i)->float3 {return float3(corners[i & 1][0], corners[(i >> 1) & 1][1], 1.0f)*zrange[(i >> 2) & 1]; }); }
inline void glwirefrustumz(float2 cmin, float2 cmax, float2 zrange) { return glwirefrustumz(float2x2(cmin, cmax), zrange); }


#endif // MISC_GL_H
