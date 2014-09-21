
#include "vecmatquat.h"
#include "dxwin.h"



int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow) try  // int main(int argc, char *argv[])
{
	DXWin dxwin("dx test (nothing to see here yet, work in progress)");
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
		cb.meshq = { 0, 0, sinf(frame*0.0001f), cosf(frame*0.0001f) };
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, &cb, 0, 0);

		
		//dxwin.pImmediateContext->PSSetShader(dxwin.pPixelShader, nullptr, 0);
		//dxwin.pImmediateContext->VSSetShader(dxwin.pVertexShader, nullptr, 0);
		//dxwin.pImmediateContext->VSSetConstantBuffers(0, 1, &dxwin.pConstantBuffer);
		//dxwin.pImmediateContext->PSSetConstantBuffers(0, 1, &dxwin.pConstantBuffer);
		dxwin.DrawImmediate(
						{
							{ 0 , 0.5f, -2.5f },   //was 0,.5,.5
							{ 0.5f, -0.f, -2.5f },
							{ -0.5f, -0.f, -2.5f },
						}, 
						{ { 0, 1, 2 } }
			);
		cb.hack={1,1,1,1};
		dxwin.pImmediateContext->UpdateSubresource(dxwin.pConstantBuffer, 0, nullptr, DXWin::ConstantBuffer(dxwin.ViewAngle,1), 0, 0);
		//dxwin.pImmediateContext->PSSetConstantBuffers(0, 1, &dxwin.pConstantBuffer);
		dxwin.DrawImmediate(
						{
							{ 0.0f, 0.5f, -2.5f },  // unused, just to test
							{ 0.5f, -0.0f, -2.45f },
							{ -0.5f, -0.0f, -2.55f },
							{ 0.0f, -0.6f, -2.5f },
						}, 
						{ { 1, 3, 2 } }
			);
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

