// 
//   dxwin.h
//
// trying for smallest possible d3d11 on windows framework for writing quick graphics test apps but need d3d11.
// All in one header file.  No extra downloads, installs, cmake, .DLLs, .LIBs. etc...
//
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
#include "mesh.h"

// not sure if best way to do this:
#include "dxshaders.h"  // defines shaders in a big string (char*) dxshaders

#define VERIFY (assert(0),throw(std::exception((std::string(__FILE__) + ":" + std::to_string(__LINE__)).c_str())),1)


inline float4x4 MatrixPerspectiveFovAspect(float fovy, float aspect, float zn, float zf) { auto h = 1 / tan(fovy / 2), zr = zf / (zn - zf); return{ { h / aspect, 0, 0, 0 }, { 0, h, 0, 0 }, { 0, 0, zr, -1 }, { 0, 0, zn*zr, 0 } }; }
#define OFFSET(Class,Member)  (((char*) (&(((Class*)NULL)-> Member )))- ((char*)NULL))

//from mesh.h:
//      struct Vertex  // pretty much just need one vertex layout to do anything.  
//      {
//      	float3 position;    // : POSITION;
//      	float4 orientation; // : TEXCOORD1;  quaternion for tangent,binormal,normal
//      	float2 texcoord;    // : TEXCOORD0;
//      };

class DXWin
{
public:

	HWND                    hWnd;
	int 	                Width;
	int 	                Height;
	int                     mousewheel;   // if and how much its been rolled up/down this frame
	int                     MouseX;
	int                     MouseY;
	float3                  MouseVector;      // 3D direction mouse points
	float3                  OldMouseVector;
	int                     MouseState;     // true iff left button down
	float 	                ViewAngle;
	int                     vsync = 1;
	std::function<void(int, int, int)> keyboardfunc;


	ID3D11Device*           dxdevice           = nullptr;
	ID3D11Device1*          dxdevice1          = nullptr;
	ID3D11DeviceContext*    dxcontext          = nullptr;
	IDXGISwapChain*         dxswapchain        = nullptr;
	ID3D11RenderTargetView* dxrendertargetview = nullptr;
	ID3D11Texture2D*        dxdepthstenciltex  = nullptr;
	ID3D11DepthStencilView* dxdepthstencilview = nullptr;


	struct ConstantBuffer  // struct matches global var struct used by shaders
	{
		float4 hack = { 1, 1, 0, 1 };
		float4x4 Projection;
		float3 camerap; float unusedc;  // seems to need like float3  so either float4 or have extra 32bit float with float3
		float4 cameraq = { 0, 0, 0, 1 };
		float3 meshp; float unusedm;
		float4 meshq = { 0, 0, 0, 1 };
		float4 lightposn = { -2, -4, 3, 1 };
		ConstantBuffer(){}
		ConstantBuffer(float fov, float aspect) :Projection(transpose(MatrixPerspectiveFovAspect(fov*3.14f / 180.0f, aspect, 0.05f, 50.0f))){}
	};

	ID3D11ShaderResourceView *MakeTexCheckerboard()
	{
		int w = 64, h = 64;
		std::vector<int> image(64 * 64, 0);
		for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
			image[i*w + j] = ((i & 8) == (j & 8)) ? 0x00000000 : -1;
		return MakeTex((unsigned char*)image.data(), w, h);
	}
	ID3D11ShaderResourceView *MakeTexWav()
	{
		int w = 128, h = 128;
		std::vector<float> hmap(w*h, 0);
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			float fx = (float)x / float(w) -0.5f ;
			float fy = (float)y / float(h) - 0.5f;
			hmap[y*w + x] = 0.5f + std::max(0.0f,cosf( sqrtf(fx*fx+fy*fy)*1.1f*3.14f  )) / 2.0f / 1.0f;  // 1/10 max slopes
//			hmap[y*w + x] = 0.5f + (cosf(((float)x / (float)(w))*3.14f*2.0f * 3) + cosf( ((float)y / (float)h)*3.14f*2.0f * 3)) / 4.0f / 10.0f;  // 1/10 max slopes
		}
		std::vector<unsigned char> image(w * h * 4, 0);
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			int xp = (x + w - 1) % w, xn = (x + 1) % w, yp = (y + h - 1) % h, yn = (y + 1) % h;
			auto n = normalize(float3( (hmap[y*w + xp] - hmap[y*w + xn]), (hmap[yp*w + x] - hmap[yn*w + x]), 2.0f/w ))*127.0f + float3(128,128,128);
			image[(y*w + x) * 4 + 0] = (unsigned char)n.x;
			image[(y*w + x) * 4 + 1] = (unsigned char)n.y;
			image[(y*w + x) * 4 + 2] = (unsigned char)n.z;
			image[(y*w + x) * 4 + 3] = (unsigned char) (hmap[y*w+x]*255.0f );
		}
		return MakeTex((unsigned char*)image.data(), w, h);
	}
	ID3D11ShaderResourceView *MakeTex(const unsigned char *buf, int w, int h) // input image is rgba
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

		dxdevice->CreateTexture2D(&tdesc, &tbsd, &tex) && VERIFY;

		D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc;
		memset(&SRVDesc, 0, sizeof(SRVDesc));
		SRVDesc.Format = tdesc.Format;
		SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		SRVDesc.Texture2D.MipLevels = (!tdesc.MipLevels) ? -1 : tdesc.MipLevels;
		dxdevice->CreateShaderResourceView(tex,&SRVDesc,&textureView)&&VERIFY;

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

		RECT rc = { 100, 100, 100 + Width, 100 + Height };
		auto bb = AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, 0);
		HWND hWnd = CreateWindowA("D3D11", title, WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
			rc.left , rc.top , rc.right-rc.left, rc.bottom-rc.top, NULL, NULL, wc.hInstance, this);  // force non-unicode16 non-wchar version of Windows's CreateWindow

		if (hWnd == NULL)
			throw("CreateWindow() failed:  Cannot create a window.");
		ShowWindow(hWnd, 1);
		//hDC = GetDC(hWnd);

		HRESULT hr = S_OK;

		GetClientRect(hWnd, &rc);
		UINT width = rc.right - rc.left;
		UINT height = rc.bottom - rc.top;

		UINT createDeviceFlags = 0;   // if(debug) createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;

		D3D_FEATURE_LEVEL featureLevels[] = {/*D3D_FEATURE_LEVEL_11_1,*/D3D_FEATURE_LEVEL_11_0,	D3D_FEATURE_LEVEL_10_1,	D3D_FEATURE_LEVEL_10_0 };
		D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;

		D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevels, sizeof(featureLevels)/sizeof(featureLevel),
			D3D11_SDK_VERSION, &dxdevice, &featureLevel, &dxcontext) && VERIFY ;  // ("create d3d11 device");

		// Obtain DXGI factory from device (since we used nullptr for pAdapter above)
		IDXGIFactory1* dxgiFactory = nullptr;

		IDXGIDevice* dxgiDevice = nullptr;
		dxdevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice)) && VERIFY;  // ("dxgidevice");
		IDXGIAdapter* adapter = nullptr;
		dxgiDevice->GetAdapter(&adapter) && VERIFY; // "GetAdapter");
		adapter->GetParent(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory)) && VERIFY ;  // ("GetParent dxgiFactory");
		dxgiDevice->Release();
		
		IDXGIFactory2* dxgiFactory2 = nullptr;
		dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));
		if (dxgiFactory2) // DirectX 11.1 
		{
			ID3D11DeviceContext1*   dxcontext1 = nullptr;  // note probably should be 'released' at shutdown, but oh well
			IDXGISwapChain1*        dxswapchain1 = nullptr;
			if (dxdevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&dxdevice1)) >= 0)
				(void)dxcontext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&dxcontext1));

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
			dxgiFactory2->CreateSwapChainForHwnd(dxdevice, hWnd, &sd, nullptr, nullptr, &dxswapchain1) && VERIFY;  // ("dxgifactory2createswapchain");
			dxswapchain1->QueryInterface(__uuidof(IDXGISwapChain), reinterpret_cast<void**>(&dxswapchain)) && VERIFY;  // ("swapchain QueryInterface");
			dxgiFactory2->Release();
		}
		else // DirectX 11.0     sigh, so much api to using dx 
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

			dxgiFactory->CreateSwapChain(dxdevice, &sd, &dxswapchain) && VERIFY;  // ("dxgiFactory->CreateSwapChain");
		}

		dxgiFactory->Release();

		ID3D11Texture2D* pBackBuffer = nullptr;
		hr = dxswapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer)) && VERIFY;  //("dxswapchain->GetBuffer");

		hr = dxdevice->CreateRenderTargetView(pBackBuffer, nullptr, &dxrendertargetview) && VERIFY;  // ("dxdevice->CreateRenderTargetView")
		pBackBuffer->Release();
		dxcontext->OMSetRenderTargets(1, &dxrendertargetview, nullptr);

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
		dxdevice->CreateTexture2D(&descDepth, nullptr, &dxdepthstenciltex) &&VERIFY;
		D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
		ZeroMemory(&descDSV, sizeof(descDSV));
		descDSV.Format = descDepth.Format;
		descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
		descDSV.Texture2D.MipSlice = 0;
		dxdevice->CreateDepthStencilView(dxdepthstenciltex, &descDSV, &dxdepthstencilview)  &&VERIFY;
		dxcontext->OMSetRenderTargets(1, &dxrendertargetview, dxdepthstencilview);

		// Setup the viewport
		D3D11_VIEWPORT vp = { 0, 0, (FLOAT)width, (FLOAT)height, 0.0f, 1.0f };
		dxcontext->RSSetViewports(1, &vp);

		return hWnd;
	}    

	ID3D11VertexShader*                 pVertexShader   = nullptr;
	ID3D11PixelShader*                  pPixelShader    = nullptr;
	ID3D11InputLayout*                  pVertexLayout   = nullptr;
	ID3D11Buffer*                       pConstantBuffer = nullptr;

	ID3D11ShaderResourceView*           pTextureRV      = nullptr;
	ID3D11ShaderResourceView*           pNMapRV         = nullptr;
	ID3D11SamplerState*                 pSamplerLinear  = nullptr;

	void CreateShaders()
	{
		ID3DBlob* pVSBlob = nullptr;
		ID3DBlob* pErrorBlob = nullptr;
		//hr = CompileShaderFromFile( L"Tutorial02.fx", "VS", "vs_4_0", &pVSBlob );
		D3DCompile(dxshaders, strlen(dxshaders), "vs", NULL, NULL, "VS", "vs_4_0", D3DCOMPILE_DEBUG, 0, &pVSBlob, &pErrorBlob) && VERIFY;
		dxdevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &pVertexShader) && VERIFY;

		D3D11_INPUT_ELEMENT_DESC layout[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT   , 0, OFFSET(Vertex, position   ), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, OFFSET(Vertex, orientation), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT      , 0, OFFSET(Vertex, texcoord   ), D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};
		UINT numElements = ARRAYSIZE(layout);

		// Create the input layout
		dxdevice->CreateInputLayout(layout,sizeof(layout)/sizeof(*layout), pVSBlob->GetBufferPointer(),pVSBlob->GetBufferSize(), &pVertexLayout) &&VERIFY;
		pVSBlob->Release();
		dxcontext->IASetInputLayout(pVertexLayout);

		// Compile the pixel shader
		ID3DBlob* pPSBlob = nullptr;

		D3DCompile(dxshaders, strlen(dxshaders), "ps", NULL, NULL, "PS", "ps_4_0", D3DCOMPILE_DEBUG, 0, &pPSBlob, &pErrorBlob) && VERIFY;

		// Create the pixel shader
		dxdevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &pPixelShader) && VERIFY;
		pPSBlob->Release();

		ConstantBuffer shaderglobals;
		shaderglobals.Projection = MatrixPerspectiveFovAspect(this->ViewAngle, (float)Width / Height, 0.05f, 50.0f);
		shaderglobals.hack = { 1, 0, 0, 1 };
		D3D11_BUFFER_DESC bdc =
		{
			sizeof(ConstantBuffer) ,         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_CONSTANT_BUFFER, // UINT        BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA shaderglobals_rc = { &shaderglobals, 0, 0 };
		//dxdevice->CreateBuffer(&bdc, &shaderglobals_rc, &pConstantBuffer) && VERIFY;
		dxdevice->CreateBuffer(&bdc, nullptr, &pConstantBuffer) && VERIFY;

		D3D11_SAMPLER_DESC sampDesc;
		ZeroMemory(&sampDesc, sizeof(sampDesc));
		sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
		sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
		sampDesc.MinLOD = 0;
		sampDesc.MaxLOD = D3D11_FLOAT32_MAX;
		dxdevice->CreateSamplerState(&sampDesc, &pSamplerLinear) &&VERIFY;

		pTextureRV = MakeTexCheckerboard();
		pNMapRV = MakeTexWav();

		// yup, do all the following to specify that "front" faces are ccw.
		D3D11_RASTERIZER_DESC rastdesc = {
			 D3D11_FILL_SOLID, // D3D11_FILL_MODE FillMode;
			 D3D11_CULL_BACK , // D3D11_CULL_MODE CullMode;
			 true            , // BOOL   FrontCounterClockwise;
			 0, 0.0, 0.0,      // INT DepthBias; FLOAT DepthBiasClamp; FLOAT SlopeScaledDepthBias;
			 true,false,false,false,  // BOOL DepthClipEnable, ScissorEnable,  MultisampleEnable,  AntialiasedLineEnable;
		};
		ID3D11RasterizerState *raststate;
		dxdevice->CreateRasterizerState(&rastdesc, &raststate)&&VERIFY;
		dxcontext->RSSetState(raststate);

		dxcontext->PSSetShader(pPixelShader, nullptr, 0);
		dxcontext->VSSetShader(pVertexShader, nullptr, 0);
		dxcontext->VSSetConstantBuffers(0, 1, &pConstantBuffer);
		dxcontext->PSSetConstantBuffers(0, 1, &pConstantBuffer);
		dxcontext->PSSetSamplers(0, 1, &pSamplerLinear);
		dxcontext->PSSetShaderResources(0, 1, &pTextureRV);
		dxcontext->PSSetShaderResources(1, 1, &pNMapRV);

	}

	void ComputeMouseVector(){
		OldMouseVector=MouseVector;
		float spread = (float)tan(ViewAngle/2*3.14/180);  
		float y = spread * ((Height-MouseY)-Height/2.0f) /(Height/2.0f);
		float x = spread * (MouseX-Width/2.0f)  /(Height/2.0f);
		MouseVector = normalize(float3(x,y,-1));
	}
	void DestroyDXWindow()
	{
		if(!hWnd)
			return; // already destroyed or otherwise non existent
		hWnd=NULL;
	}

	DXWin(const char *title, int2 dim = { 1920,1080 }) : Width(dim.x), Height(dim.y), MouseX(0), MouseY(0), MouseState(0), mousewheel(0), ViewAngle(60.0f) //, keyboardfunc([](int, int, int){})
	{
		hWnd = CreateDXWindow(title);
		if (hWnd == NULL) throw("failed to create d3d11 window");
		CreateShaders();
	}
	~DXWin()
	{
		DestroyDXWindow();
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
		dxdevice->CreateBuffer(&bdv, &vertices_rc, &pVertexBuffer) && VERIFY;

		// Set vertex buffer
		UINT strides[] = { sizeof(Vertex) };
		UINT offsets[] = { 0 };
		dxcontext->IASetVertexBuffers(0, 1, &pVertexBuffer, strides, offsets);

		D3D11_BUFFER_DESC bdt =
		{
			tris.size() * sizeof(int3),          // UINT         ByteWidth;   // full memory byte size of the list eg sizeof(struct)*count
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE  Usage;
			D3D11_BIND_INDEX_BUFFER,  // UINT         BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA indices_rc = { tris.data(), 0, 0 };
		dxdevice->CreateBuffer(&bdt, &indices_rc, &pIndexBuffer) && VERIFY;
		dxcontext->IASetIndexBuffer(pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);  // Set index buffer

		dxcontext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);  // Set primitive topology
		dxcontext->DrawIndexed(tris.size()*3, 0, 0);  // triangle count * 3 
		pIndexBuffer->Release();
		pVertexBuffer->Release();
		pIndexBuffer = NULL;
		pVertexBuffer = NULL;
	}

	void DrawMeshes(ConstantBuffer cb, const std::vector<Mesh*> &meshes)
	{
		for (auto m : meshes)
		{
			cb.hack  = m->hack;
			cb.meshp = m->pose.position;
			cb.meshq = m->pose.orientation;
			dxcontext->UpdateSubresource(pConstantBuffer, 0, nullptr, &cb, 0, 0);
			DrawImmediate(m->verts, m->tris);
		}
	}
	void RenderScene(const Pose &camera, const std::vector<Mesh*> &meshes)
	{
		float clearcolor[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		dxcontext->ClearRenderTargetView(dxrendertargetview, clearcolor);
		dxcontext->ClearDepthStencilView(dxdepthstencilview, D3D11_CLEAR_DEPTH, 1.0f, 0); // Clear depth buffer to 1.0 (max depth)
		DXWin::ConstantBuffer cb;
		cb.Projection = transpose(MatrixPerspectiveFovAspect(ViewAngle*3.14f / 180.0f, (float)Width / Height, 0.05f, 50.0f));
		cb.camerap = camera.position;
		cb.cameraq = camera.orientation;
		DrawMeshes(cb, meshes);
		dxswapchain->Present(vsync, 0);
	}
	void RenderStereo(const Pose &camera, const std::vector<Mesh*> &meshes)
	{
		float clearcolor[] = { 0.5f, 0.6f, 1.0f, 1.0f };
		D3D11_VIEWPORT fullviewport = { 0.0f, 0.0f, (float)Width, (float)Height, 0.0f, 1.0f };
		dxcontext->RSSetViewports(1, &fullviewport);
		dxcontext->ClearRenderTargetView(dxrendertargetview, clearcolor);
		dxcontext->ClearDepthStencilView(dxdepthstencilview, D3D11_CLEAR_DEPTH, 1.0f, 0); // Clear depth buffer to 1.0 (max depth)
		for (int e = 0; e < 2; e++)
		{
			D3D11_VIEWPORT eviewport = { (float)Width / 2 * e, 0.0f, (float)Width / 2, (float)Height, 0.0f, 1.0f };
			dxcontext->RSSetViewports(1, &eviewport);
			DXWin::ConstantBuffer cb;
			cb.Projection = transpose(MatrixPerspectiveFovAspect(ViewAngle*3.14f / 180.0f, (float)1 / 1, 0.05f, 50.0f));
			cb.camerap = camera.position;
			cb.cameraq = camera.orientation;
			DrawMeshes(cb, meshes);
		}
		dxcontext->RSSetViewports(1, &fullviewport);
		dxswapchain->Present(0, 0);
	}

	LONG WINAPI MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{

		switch (uMsg) {
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
			SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)(reinterpret_cast<CREATESTRUCT *>(lParam)->lpCreateParams));  // grab my pointer passed into createwindow
		auto w = reinterpret_cast<DXWin *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		return (w)? w->MsgProc(hWnd, msg, wParam, lParam ) : DefWindowProc( hWnd, msg, wParam, lParam );
	}
};


