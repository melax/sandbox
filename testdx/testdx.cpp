
#include "linalg.h"
#include "mesh.h"
#include "dxwin.h"

#include "wingmesh.h"


Mesh MeshFlatShadeTex(const WingMesh &m)  // procedurally generate normals and texture coords mesh
{
	return MeshFlatShadeTex(m.verts, WingMeshTris(m));
}

int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try  // int main(int argc, char *argv[])
{
	DXWin dxwin("simple test of d3d render window");

	WingMesh cube_wm = WingMeshCube(1.0f);
	WingMesh oct_wm = WingMeshDual(cube_wm, 1.4f);
	auto cube_mesh = MeshFlatShadeTex(cube_wm);
	auto oct_mesh = MeshFlatShadeTex(oct_wm);
	int frame = 0;
	bool stereo = 0;
	dxwin.keyboardfunc = [&](int key, int, int)
	{
		stereo =  key == 's' != stereo;
	};
	while (dxwin.WindowUp())
	{
		frame++;
		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		frame++;

		cube_mesh.hack = { 0.5f + 0.5f*sinf(frame*0.0002f), 1, 1, 1 };
		cube_mesh.pose.orientation = { 0, 0, sinf(frame*0.0001f), cosf(frame*0.0001f) };

		oct_mesh.pose = { { -2.5f, 6.3f, -0.25f }, { 0, 0, -sinf(frame*0.0001f), cosf(frame*0.0001f) } };
		oct_mesh.hack = { 1, 0, 1, 1 };

		(dxwin.*(stereo ? &DXWin::RenderStereo : &DXWin::RenderScene))({ { 0, -3, 2 }, normalize(float4(1, 0, 0, 2)) }, { &cube_mesh, &oct_mesh });
	}
}
catch (const char *c) 
{	
	MessageBox(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}

