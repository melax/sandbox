// 
//   dxwin.h
//
// trying for smallest possible d3d11 on windows framework for writing quick graphics test apps but need d3d11.
// All in one header file.  No extra downloads, installs, cmake, .DLLs, .LIBs. etc...
//
// This is preliminary work/experimentation in progress
// started this less than 24 hours ago, referenced the d3d11 version of the dx tutorials from the MSDN site.  
// I hadn't been recenly been focusing on anything graphics and usually had just used a dx9 based rendering sytem that is kinda out of date.
// want a d3d11 version going forward for compatability with future libs and usages.
// hoping to complete and cram this into a concise api/header.
//
// given the emphasis is on convenience, likely just have one vertex format and one constant uniform buffer that is a superset of everyones needs.
//
// avoid 16 bit wchar.
//


#include <assert.h>
#include <functional>
#include <vector>
#include <string>

#define NOMINMAX
#include <windows.h>
#include <d3d11_1.h>
#include <d3dcompiler.h>

#include <cstring>
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf

#ifdef WIN32
#pragma comment(lib,"d3d11.lib")
#pragma comment(lib,"d3dcompiler.lib")
#pragma comment(lib,"dxguid.lib")
#endif

#include "geometric.h"
// not sure if best way to do this:
#include "dxshaders.h"  // defines shaders in a big string (char*) dxshaders

#define VERIFY (assert(0),throw(std::exception((std::string(__FILE__) + ":" + std::to_string(__LINE__)).c_str())),1)


inline float4x4 MatrixPerspectiveFov(float fovy, float aspect, float zn, float zf) { auto h = 1 / tan(fovy / 2), zr = zf / (zn - zf); return{ { h / aspect, 0, 0, 0 }, { 0, h, 0, 0 }, { 0, 0, zr, -1 }, { 0, 0, zn*zr, 0 } }; }
#define OFFSET(Class,Member)  (((char*) (&(((Class*)NULL)-> Member )))- ((char*)NULL))

struct Vertex  // pretty much just need one vertex layout to do anything.  
{
	float3 position;    // : POSITION;
	float4 orientation; // : TEXCOORD1;  quaternion for tangent,binormal,normal
	float2 texcoord;    // : TEXCOORD0;
};

class DXWin
{
public:

	struct ConstantBuffer  // struct matches global var struct used by shaders
	{
		float4 hack = { 1, 1, 0, 1 };
		float4x4 Projection;
		float3 camerap; float unusedc;  // seems to need like float3  so either float4 or have extra 32bit float with float3
		float4 cameraq = { 0, 0, 0, 1 };
		float3 meshp; float unusedm;
		float4 meshq = { 0, 0, 0, 1 };
		ConstantBuffer(){}
		ConstantBuffer(float fov, float aspect) :Projection(transpose(MatrixPerspectiveFov(fov*3.14f / 180.0f, aspect, 0.05f, 50.0f))){}
		operator const ConstantBuffer*() { return this; } // useful for passing to d3d api
	};


	ID3D11Device*           pd3dDevice = nullptr;
	ID3D11Device1*          pd3dDevice1 = nullptr;
	ID3D11DeviceContext*    pImmediateContext = nullptr;
	ID3D11DeviceContext1*   pImmediateContext1 = nullptr;
	IDXGISwapChain*         pSwapChain = nullptr;
	IDXGISwapChain1*        pSwapChain1 = nullptr;
	ID3D11RenderTargetView* pRenderTargetView = nullptr;
	ID3D11Texture2D*        pDepthStencil = nullptr;
	ID3D11DepthStencilView* pDepthStencilView = nullptr;

	ID3D11ShaderResourceView *MakeTexCheckerboard()
	{
		int w = 64, h = 64;
		std::vector<int> image(64 * 64, 0);
		for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
			image[i*w + j] = ((i & 8) == (j & 8)) ? 0x00000000 : -1;
		return MakeTex((unsigned char*)image.data(), w, h);
	}
	ID3D11ShaderResourceView *MakeTex(const unsigned char *buf, int w, int h)
	{
		ID3D11Texture2D *tex;
		ID3D11ShaderResourceView* textureView;
		D3D11_TEXTURE2D_DESC tdesc;
		D3D11_SUBRESOURCE_DATA tbsd;

		tbsd.pSysMem = (void *)buf;
		tbsd.SysMemPitch = w * 4;
		tbsd.SysMemSlicePitch = w*h * 4; // Not needed since this is a 2d texture
		tdesc.Width = w;
		tdesc.Height = h;
		tdesc.MipLevels = 1;
		tdesc.ArraySize = 1;
		tdesc.SampleDesc.Count = 1;
		tdesc.SampleDesc.Quality = 0;
		tdesc.Usage = D3D11_USAGE_DEFAULT;
		tdesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		tdesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		tdesc.CPUAccessFlags = 0;
		tdesc.MiscFlags = 0;

		pd3dDevice->CreateTexture2D(&tdesc, &tbsd, &tex) && VERIFY;

		D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc;
		memset(&SRVDesc, 0, sizeof(SRVDesc));
		SRVDesc.Format = tdesc.Format;
		SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		SRVDesc.Texture2D.MipLevels = (!tdesc.MipLevels) ? -1 : tdesc.MipLevels;
		pd3dDevice->CreateShaderResourceView(tex,&SRVDesc,&textureView)&&VERIFY;

		return(textureView);
	}
	HWND CreateDXWindow(const char* title)  // make a d3d11 window
	{
		WNDCLASSA   wc;   // force non-unicode16 version using 'A' suffix 
		wc.style = CS_OWNDC;
		wc.lpfnWndProc   = (WNDPROC)MsgProcG;
		wc.cbClsExtra    = 0;
		wc.cbWndExtra    = 0;
		wc.hInstance     = GetModuleHandleA(NULL); // hInstance;
		wc.hIcon         = LoadIcon(NULL, IDI_WINLOGO);
		wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = NULL;
		wc.lpszMenuName  = NULL;
		wc.lpszClassName = "D3D11";

		if (!RegisterClassA(&wc)) 
			throw("RegisterClassA() failed:  Cannot register window class.");  // supposedly should only register the window class once

		HWND hWnd = CreateWindowA("D3D11", title, WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
			0, 0, Width, Height, NULL, NULL, wc.hInstance, this);  // force non-unicode16 non-wchar version of Windows's CreateWindow

		if (hWnd == NULL)
			throw("CreateWindow() failed:  Cannot create a window.");
		ShowWindow(hWnd, 1);
		//hDC = GetDC(hWnd);

		HRESULT hr = S_OK;

		RECT rc;
		GetClientRect(hWnd, &rc);
		UINT width = rc.right - rc.left;
		UINT height = rc.bottom - rc.top;

		UINT createDeviceFlags = 0;   // if(debug) createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;

		D3D_FEATURE_LEVEL featureLevels[] = {/*D3D_FEATURE_LEVEL_11_1,*/D3D_FEATURE_LEVEL_11_0,	D3D_FEATURE_LEVEL_10_1,	D3D_FEATURE_LEVEL_10_0 };
		D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;

		D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevels, sizeof(featureLevels)/sizeof(featureLevel),
			D3D11_SDK_VERSION, &pd3dDevice, &featureLevel, &pImmediateContext) && VERIFY ;  // ("create d3d11 device");

		// Obtain DXGI factory from device (since we used nullptr for pAdapter above)
		IDXGIFactory1* dxgiFactory = nullptr;

		IDXGIDevice* dxgiDevice = nullptr;
		pd3dDevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice)) && VERIFY;  // ("dxgidevice");
		IDXGIAdapter* adapter = nullptr;
		dxgiDevice->GetAdapter(&adapter) && VERIFY; // "GetAdapter");
		adapter->GetParent(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory)) && VERIFY ;  // ("GetParent dxgiFactory");
		dxgiDevice->Release();
		
		IDXGIFactory2* dxgiFactory2 = nullptr;
		dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));
		if (dxgiFactory2) // DirectX 11.1 
		{
			if( pd3dDevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&pd3dDevice1)) >=0)
				(void)pImmediateContext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&pImmediateContext1));

			DXGI_SWAP_CHAIN_DESC1 sd = 
			{                                           
				width, height,                          // UINT Width; UINT Height;
				DXGI_FORMAT_R8G8B8A8_UNORM, 			// DXGI_FORMAT Format;
				0, 										// BOOL Stereo;
				{ 1, 0 }, 								// DXGI_SAMPLE_DESC {Count,Quality} SampleDesc; 
				DXGI_USAGE_RENDER_TARGET_OUTPUT, 		// DXGI_USAGE BufferUsage;
				1, 										// UINT BufferCount;
				DXGI_SCALING_STRETCH, 					// DXGI_SCALING Scaling;
				DXGI_SWAP_EFFECT_DISCARD,				// DXGI_SWAP_EFFECT SwapEffect;
				DXGI_ALPHA_MODE_UNSPECIFIED, 			// DXGI_ALPHA_MODE AlphaMode;
				0 										// UINT Flags;
			};
			dxgiFactory2->CreateSwapChainForHwnd(pd3dDevice, hWnd, &sd, nullptr, nullptr, &pSwapChain1) && VERIFY;  // ("dxgifactory2createswapchain");
			pSwapChain1->QueryInterface(__uuidof(IDXGISwapChain), reinterpret_cast<void**>(&pSwapChain)) && VERIFY;  // ("swapchain QueryInterface");
			dxgiFactory2->Release();
		}
		else // DirectX 11.0 
		{
			DXGI_SWAP_CHAIN_DESC sd;
			ZeroMemory(&sd, sizeof(sd));
			sd.BufferCount = 1;
			sd.BufferDesc.Width = width;
			sd.BufferDesc.Height = height;
			sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			sd.BufferDesc.RefreshRate.Numerator = 60;
			sd.BufferDesc.RefreshRate.Denominator = 1;
			sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
			sd.OutputWindow = hWnd;
			sd.SampleDesc.Count = 1;
			sd.SampleDesc.Quality = 0;
			sd.Windowed = TRUE;

			dxgiFactory->CreateSwapChain(pd3dDevice, &sd, &pSwapChain) && VERIFY;  // ("dxgiFactory->CreateSwapChain");
		}

		dxgiFactory->Release();

		ID3D11Texture2D* pBackBuffer = nullptr;
		hr = pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer)) && VERIFY;  //("pSwapChain->GetBuffer");

		hr = pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &pRenderTargetView) && VERIFY;  // ("pd3dDevice->CreateRenderTargetView")
		pBackBuffer->Release();
		pImmediateContext->OMSetRenderTargets(1, &pRenderTargetView, nullptr);

		// so the next many dozen lines of code are about the same as glEnable(GL_DEPTH_TEST) sigh
		D3D11_TEXTURE2D_DESC descDepth;
		ZeroMemory(&descDepth, sizeof(descDepth));
		descDepth.Width = width;
		descDepth.Height = height;
		descDepth.MipLevels = 1;
		descDepth.ArraySize = 1;
		descDepth.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
		descDepth.SampleDesc.Count = 1;
		descDepth.SampleDesc.Quality = 0;
		descDepth.Usage = D3D11_USAGE_DEFAULT;
		descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
		descDepth.CPUAccessFlags = 0;
		descDepth.MiscFlags = 0;
		pd3dDevice->CreateTexture2D(&descDepth, nullptr, &pDepthStencil) &&VERIFY;
		D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
		ZeroMemory(&descDSV, sizeof(descDSV));
		descDSV.Format = descDepth.Format;
		descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
		descDSV.Texture2D.MipSlice = 0;
		pd3dDevice->CreateDepthStencilView(pDepthStencil, &descDSV, &pDepthStencilView)  &&VERIFY;
		pImmediateContext->OMSetRenderTargets(1, &pRenderTargetView, pDepthStencilView);

		// Setup the viewport
		D3D11_VIEWPORT vp;
		vp.Width = (FLOAT)width;
		vp.Height = (FLOAT)height;
		vp.MinDepth = 0.0f;
		vp.MaxDepth = 1.0f;
		vp.TopLeftX = 0;
		vp.TopLeftY = 0;
		pImmediateContext->RSSetViewports(1, &vp);

		return hWnd;
	}    

	ID3D11VertexShader*                 pVertexShader   = nullptr;
	ID3D11PixelShader*                  pPixelShader    = nullptr;
	ID3D11InputLayout*                  pVertexLayout   = nullptr;
	ID3D11Buffer*                       pConstantBuffer = nullptr;

	ID3D11ShaderResourceView*           pTextureRV      = nullptr;
	ID3D11SamplerState*                 pSamplerLinear  = nullptr;

	void CreateShaders()
	{
		ID3DBlob* pVSBlob = nullptr;
		ID3DBlob* pErrorBlob = nullptr;
		//hr = CompileShaderFromFile( L"Tutorial02.fx", "VS", "vs_4_0", &pVSBlob );
		D3DCompile(dxshaders, strlen(dxshaders), "vs", NULL, NULL, "VS", "vs_4_0", D3DCOMPILE_DEBUG, 0, &pVSBlob, &pErrorBlob) && VERIFY;
		pd3dDevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &pVertexShader) && VERIFY;

		D3D11_INPUT_ELEMENT_DESC layout[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT   , 0, OFFSET(Vertex, position   ), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, OFFSET(Vertex, orientation), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT      , 0, OFFSET(Vertex, texcoord   ), D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};
		UINT numElements = ARRAYSIZE(layout);

		// Create the input layout
		pd3dDevice->CreateInputLayout(layout,sizeof(layout)/sizeof(*layout), pVSBlob->GetBufferPointer(),pVSBlob->GetBufferSize(), &pVertexLayout) &&VERIFY;
		pVSBlob->Release();
		pImmediateContext->IASetInputLayout(pVertexLayout);

		// Compile the pixel shader
		ID3DBlob* pPSBlob = nullptr;

		D3DCompile(dxshaders, strlen(dxshaders), "ps", NULL, NULL, "PS", "ps_4_0", D3DCOMPILE_DEBUG, 0, &pPSBlob, &pErrorBlob) && VERIFY;

		// Create the pixel shader
		pd3dDevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &pPixelShader) && VERIFY;
		pPSBlob->Release();

		ConstantBuffer shaderglobals;
		shaderglobals.Projection = MatrixPerspectiveFov(this->ViewAngle, (float)Width / Height, 0.05f, 50.0f);
		shaderglobals.hack = { 1, 0, 0, 1 };
		D3D11_BUFFER_DESC bdc =
		{
			sizeof(ConstantBuffer) ,         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_CONSTANT_BUFFER, // UINT        BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA shaderglobals_rc = { &shaderglobals, 0, 0 };
		//pd3dDevice->CreateBuffer(&bdc, &shaderglobals_rc, &pConstantBuffer) && VERIFY;
		pd3dDevice->CreateBuffer(&bdc, nullptr, &pConstantBuffer) && VERIFY;

		D3D11_SAMPLER_DESC sampDesc;
		ZeroMemory(&sampDesc, sizeof(sampDesc));
		sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
		sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
		sampDesc.MinLOD = 0;
		sampDesc.MaxLOD = D3D11_FLOAT32_MAX;
		pd3dDevice->CreateSamplerState(&sampDesc, &pSamplerLinear) &&VERIFY;

		pTextureRV = MakeTexCheckerboard();

		pImmediateContext->PSSetShader(pPixelShader, nullptr, 0);
		pImmediateContext->VSSetShader(pVertexShader, nullptr, 0);
		pImmediateContext->VSSetConstantBuffers(0, 1, &pConstantBuffer);
		pImmediateContext->PSSetConstantBuffers(0, 1, &pConstantBuffer);
		pImmediateContext->PSSetSamplers(0, 1, &pSamplerLinear);
		pImmediateContext->PSSetShaderResources(0, 1, &pTextureRV);

	}

	void ComputeMouseVector(){
		OldMouseVector=MouseVector;
		float spread = (float)tan(ViewAngle/2*3.14/180);  
		float y = spread * ((Height-MouseY)-Height/2.0f) /(Height/2.0f);
		float x = spread * (MouseX-Width/2.0f)  /(Height/2.0f);
		MouseVector = normalize(float3(x,y,-1));
	}
	void Reshape(int width, int height){
		// called initially and when the window changes size
	}
	void DestroyDXWindow()
	{
		if(!hWnd)
			return; // already destroyed or otherwise non existent
//
		hWnd=NULL;
	}

	HWND    hWnd;
	//HDC     hDC;              // device context 
    //HGLRC   hRC;              // opengl context 
	int 	Width  ;
	int 	Height ;
	int     mousewheel;   // if and how much its been rolled up/down this frame
	int     MouseX ;
	int     MouseY ;
	float3  MouseVector;      // 3D direction mouse points
	float3  OldMouseVector;
	int     MouseState;     // true iff left button down
	float 	ViewAngle;
	std::function<void(int, int, int)> keyboardfunc;

	DXWin(const char *title) : Width(512), Height(512), MouseX(0), MouseY(0), MouseState(0), mousewheel(0), ViewAngle(60.0f) //, keyboardfunc([](int, int, int){})
	{
		hWnd = CreateDXWindow(title);
		if (hWnd == NULL) throw("failed to create d3d11 window");
		CreateShaders();
	}
	~DXWin()
	{
		DestroyDXWindow();
	}
	void DrawImmediate(const std::vector<float3> &points, const std::vector<int3> &tris)   
	{
		std::vector<Vertex> verts;
		for (auto p : points)
			verts.push_back({ p, { 0, 0, 0, 0 }, p.xy() });
		for (auto t : tris)
		{
			float3 n = cross(points[t[1]] - points[t[0]], points[t[2]] - points[t[0]]);
			auto q = RotationArc(float3(0, 0, 1), n);
			for (int i = 0; i < 3;i++)
			{
				verts[t[i]].orientation += q;
			}
		}
		for (auto &v : verts)
		{
			v.orientation = normalize(v.orientation);
			float3 n = qrot(v.orientation, float3(0, 0, 1));
			//v.texcoord = { atan2(n.y, n.x), 0.5f + n.y / 2.0f };  // quick sphereical hack
		}
		return DrawImmediate(verts, tris);
	}
	void DrawImmediate(const std::vector<Vertex> &verts, const std::vector<int3> &tris)   // worst reimplementation of dx9 style drawprimitiveup() ever
	{
		ID3D11Buffer* pVertexBuffer = nullptr;
		ID3D11Buffer* pIndexBuffer = nullptr;
		D3D11_BUFFER_DESC bdv =
		{
			verts.size() * sizeof(Vertex),         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_VERTEX_BUFFER, // UINT        BindFlags;
			0,0,0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA vertices_rc = { verts.data(), 0, 0 };
		pd3dDevice->CreateBuffer(&bdv, &vertices_rc, &pVertexBuffer) && VERIFY;

		// Set vertex buffer
		UINT strides[] = { sizeof(Vertex) };
		UINT offsets[] = { 0 };
		pImmediateContext->IASetVertexBuffers(0, 1, &pVertexBuffer, strides, offsets);

		D3D11_BUFFER_DESC bdt =
		{
			tris.size() * sizeof(int3),          // UINT         ByteWidth;   // full memory byte size of the list eg sizeof(struct)*count
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE  Usage;
			D3D11_BIND_INDEX_BUFFER,  // UINT         BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA indices_rc = { tris.data(), 0, 0 };
		pd3dDevice->CreateBuffer(&bdt, &indices_rc, &pIndexBuffer) && VERIFY;
		pImmediateContext->IASetIndexBuffer(pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);  // Set index buffer

		pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);  // Set primitive topology
		pImmediateContext->DrawIndexed(3, 0, 0);  // triangle count * 3 
		pIndexBuffer->Release();
		pVertexBuffer->Release();
		pIndexBuffer = NULL;
		pVertexBuffer = NULL;
	}

	bool SwapBuffers() {    
		return true;
	}

	LONG WINAPI MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{

		switch (uMsg) {
		case WM_SIZE:
			Reshape(LOWORD(lParam), HIWORD(lParam));
			PostMessage(hWnd, WM_PAINT, 0, 0);
			return 0;
		case WM_CHAR:
			switch (wParam) {
			case 27: /* ESC key */
				PostQuitMessage(0);
				break;
			}
			if (keyboardfunc)
				keyboardfunc(wParam, MouseX, MouseY); // to match glut's api, add the x and y.
			return 0;

		case WM_LBUTTONDOWN:
			SetCapture(hWnd);  // set the capture to get mouse moves outside window
			MouseX = LOWORD(lParam);
			MouseY = HIWORD(lParam);
			ComputeMouseVector();
			MouseState = 1;
			return 0;

		case WM_LBUTTONUP:
			MouseX = LOWORD(lParam);
			MouseY = HIWORD(lParam);
			if (MouseX & 1 << 15) MouseX -= (1 << 16);
			if (MouseY & 1 << 15) MouseY -= (1 << 16);
			ComputeMouseVector();
			MouseState = 0;
			ReleaseCapture();
			return 0;

		case WM_MOUSEMOVE:
			MouseX = LOWORD(lParam);
			MouseY = HIWORD(lParam);
			if (MouseX & 1 << 15) MouseX -= (1 << 16); // when negative probably needed because 16bit vs 32
			if (MouseY & 1 << 15) MouseY -= (1 << 16);
			ComputeMouseVector();
			return 0;
		case  WM_MOUSEWHEEL:
			//shiftdown = (wParam&MK_SHIFT) ? 1 : 0;  
			//ctrldown = (wParam&MK_CONTROL) ? 1 : 0;
			mousewheel += GET_WHEEL_DELTA_WPARAM(wParam) / WHEEL_DELTA;
			return 0;
		case WM_CLOSE:
			PostQuitMessage(0);
			return 0;
		}
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
	bool WindowUp()
	{
		mousewheel = 0;    // reset to 0 each frame
		MSG   msg;		   // Windows message		
		while (PeekMessage(&msg, hWnd, 0, 0, PM_NOREMOVE)) {
			if (GetMessage(&msg, hWnd, 0, 0)) {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			else
			{
				return false;
			}
		}
		return true;
	}

	static LRESULT WINAPI MsgProcG(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
	{
		if(msg==WM_NCCREATE)
			SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG)(reinterpret_cast<CREATESTRUCT *>(lParam)->lpCreateParams));  // grab my pointer passed into createwindow
		auto w = reinterpret_cast<DXWin *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		return (w)? w->MsgProc(hWnd, msg, wParam, lParam ) : DefWindowProc( hWnd, msg, wParam, lParam );
	}
};


