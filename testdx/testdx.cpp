
#include "vecmatquat.h"
#include "dxwin.h"

#include "wingmesh.h"

std::pair< std::vector<Vertex>, std::vector<int3> > MeshFlatShadeTex(const std::vector<float3> &verts, const std::vector<int3> &tris)  // procedurally generate normals and texture coords mesh
{
	std::vector<Vertex> vout;
	std::vector<int3> tout;
	for (auto t : tris)
	{
		float3 n = TriNormal(verts[t[0]], verts[t[1]], verts[t[2]]);
		float4 q = RotationArc(float3(0, 0, 1), n);
		auto vn = vabs(n);
		int k = argmax(&vn.x, 3);
		int c = (int)vout.size();
		tout.push_back({ c, c + 1, c + 2 });
		for (int j = 0; j < 3; j++)
		{
			const float3 &v = verts[t[j]];
			vout.push_back({ v, q, { v[(k + 1) % 3], v[(k + 2) % 3] } });
		}
	}
	return std::pair< std::vector<Vertex>, std::vector<int3> >(vout, tout);
}
std::pair< std::vector<Vertex>, std::vector<int3> > MeshFlatShadeTex(const WingMesh &m)  // procedurally generate normals and texture coords mesh
{
	return MeshFlatShadeTex(m.verts, WingMeshTris(m));
}

int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try  // int main(int argc, char *argv[])
{
	DXWin dxwin("dx test (nothing to see here yet, work in progress)");

	WingMesh cube_wm = WingMeshBox({ -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f });
	WingMesh oct_wm = WingMeshDual(cube_wm, 0.7f);
	auto cube_mesh = MeshFlatShadeTex(cube_wm);
	auto oct_mesh  = MeshFlatShadeTex(oct_wm );
	int frame = 0;
	while (dxwin.WindowUp())
	{
		frame++;
		float c[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		dxwin.pImmediateContext->ClearRenderTargetView(dxwin.pRenderTargetView, c);
		dxwin.pImmediateContext->ClearDepthStencilView(dxwin.pDepthStencilView, D3D11_CLEAR_DEPTH, 1.0f, 0); // Clear depth buffer to 1.0 (max depth)

		DXWin::ConstantBuffer cb;
		cb.Projection = transpose(MatrixPerspectiveFov(dxwin.ViewAngle*3.14f/180.0f, (float)1 / 1, 0.05f, 50.0f));
		cb.hack = { 0.5f+0.5f*sinf(frame*0.0002f), 1, 1, 1 };
		cb.camerap = { 0, -3, 1 };
		cb.cameraq = normalize(float4(1, 0, 0, 1));  
		cb.meshq = { 0, 0, sinf(frame*0.0001f), cosf(frame*0.0001f) };
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, &cb, 0, 0);

		dxwin.DrawImmediate({{0,0.5f,-0.5f},{-0.5f,-0.f,-0.5f},{0.5f,0,-0.5f}} , {{0,1,2}} );
		cb.hack={1,0,0,1};
		cb.meshq = { 0, 0, 0, 1 };//DXWin::ConstantBuffer(dxwin.ViewAngle,1)
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, cb, 0, 0);
		//dxwin.pImmediateContext->PSSetConstantBuffers(0, 1, &dxwin.pConstantBuffer);
		dxwin.DrawImmediate({{0.5f,-0.0f,-0.45f},{-0.5f,-0.0f,-0.55f},{0.0f,-0.6f,-0.5f}} , {{0,1,2}} );

		cb.hack = { 1, 1, 0, 1 };
		cb.meshp = { 0.5f, 3.3f, -0.25f };
		cb.meshq = { 0, 0, sinf(frame*0.0001f), cosf(frame*0.0001f) }; // spin counter clockwize about z axis
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, cb, 0, 0);
		dxwin.DrawImmediate(cube_mesh.first, cube_mesh.second);

		cb.hack = { 1, 0, 1, 1 };
		cb.meshp = { -2.5f, 6.3f, -0.25f };
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, cb, 0, 0);
		dxwin.DrawImmediate(oct_mesh.first, oct_mesh.second);

		dxwin.pSwapChain->Present(0, 0);
	}
}
catch (const char *c) 
{	
	MessageBox(GetActiveWindow(), "FAIL", c, 0); //throw(std::exception(c));
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}

