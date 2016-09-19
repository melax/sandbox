// 
//   mswin.h
//
//  Create and setup of a Window for a typical MS Windows application intended for user interaction.
//  This file contains the OS specific common window setup and callback functionality. 
//  This file does not include any of the opengl or directx graphics library setup, just the win32 parts.
//  see glwin.h or dxwin.h for subclasses that do this.
//
//

#ifndef  SANDBOX_MSWIN_H
#define  SANDBOX_MSWIN_H

#include <assert.h>
#include <functional>
#include <vector>
#include <string>

#define NOMINMAX
#include <windows.h>

#include <cstring>
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf

#include "geometric.h"

#define VERIFY (assert(0),throw(std::exception((std::string(__FILE__) + ":" + std::to_string(__LINE__)).c_str())),1)


class MSWin  // parent class for DXWin and GLWin, abstract class for the common 3D usages and win32 gui calls support(separated from any dx or opengl)
{
public:
	HWND                    hWnd;
	int2                    res;
	int                     mousewheel;   // if and how much its been rolled up/down this frame
	int2                    mousepos;
	int2                    mousepos_previous;
	float2                  dmouse = { 0,0 };
	float3                  MouseVector;      // 3D direction mouse points
	float3                  OldMouseVector;
	int                     MouseState;     // true iff left button down
	float 	                ViewAngle;
	bool                    downevent;
	bool                    centermouse = false;
	bool                    centered_last_frame = false;
	bool                    focus = true;

	RECT                    window_inset;     // the extra pixels windows needs for its borders on windowed windows.
	std::function<void(int, int, int)> keyboardfunc;
	std::function<void()>              preshutdown = []() {};
	std::function<void(int,int)>       reshape     = [](int,int) {};

	float aspect_ratio() { return (float)res.x / (float)res.y; }
	void ComputeMouseVector() 
	{
		OldMouseVector = MouseVector;
		float spread = (float)tan(ViewAngle / 2 * 3.14 / 180);
		float y = spread * ((res.y - mousepos.y) - res.y / 2.0f) / (res.y / 2.0f);
		float x = spread * (mousepos.x - res.x / 2.0f) / (res.y / 2.0f);
		MouseVector = normalize(float3(x, y, -1));
	}
	MSWin(const char *title, int2 res) :res(res), mousepos(0, 0), MouseState(0), mousewheel(0), ViewAngle(60.0f), reshape([this](int x, int y) {this->res = { x,y }; })//, keyboardfunc([](int, int, int){})
	{
	}
	// virtual LONG WINAPI MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) = 0;  // since we override

	void CreateMSWindow(const char* title, int2 res, int2 win_position = { 100,100 })
	{
		WNDCLASSA   wc;   // force non-unicode16 version using 'A' suffix 
		wc.style = CS_OWNDC;
		wc.lpfnWndProc = (WNDPROC)MsgProcG;  // the global winproc
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = GetModuleHandleA(NULL); // hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = NULL;
		wc.lpszMenuName = NULL;
		wc.lpszClassName = "SANDBOX";

		if (!RegisterClassA(&wc))
			throw("RegisterClassA() failed:  Cannot register window class.");  // supposedly should only register the window class once

		SetRect(&window_inset, 0, 0, 0, 0);
		AdjustWindowRect(&window_inset, WS_OVERLAPPEDWINDOW, 0);
		this->hWnd = CreateWindowA("SANDBOX", title, WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
			//rc.left, rc.top, rc.right - rc.left, rc.bottom - rc.top,
			win_position.x + window_inset.left, win_position.y + window_inset.top, res.x + window_inset.right - window_inset.left, res.y + window_inset.bottom - window_inset.top, 
			NULL, NULL, wc.hInstance, this);  // force non-unicode16 non-wchar version of Windows's CreateWindow

		if (hWnd == NULL)
			throw("CreateWindow() failed:  Cannot create a window.");
	}

	static LRESULT WINAPI MsgProcG(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
	{
		if (msg == WM_NCCREATE)
			SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)(reinterpret_cast<CREATESTRUCT *>(lParam)->lpCreateParams));  // grab my pointer passed into createwindow
		auto w = reinterpret_cast<MSWin *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		return (w) ? w->MsgProc(hWnd, msg, wParam, lParam) : DefWindowProc(hWnd, msg, wParam, lParam);
	}

	LONG WINAPI MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		auto fixnbits = [](int d) { return (d & 1 << 15) ? d - (1 << 16) : d; };
		auto take_mouse_position_from_lparam = [lParam, fixnbits]() ->int2 { return int2(fixnbits(LOWORD(lParam)), fixnbits(HIWORD(lParam))); };
		switch (uMsg) {
		case WM_CHAR:
			switch (wParam) {
			case 27: /* ESC key */
				if (preshutdown) 
					preshutdown();
				PostQuitMessage(0);
				break;
			}
			if (keyboardfunc)
				keyboardfunc(wParam, mousepos.x, mousepos.y); // to match glut's api, add the x and y.
			return 0;

		case WM_LBUTTONDOWN:
			SetCapture(hWnd);  // set the capture to get mouse moves outside window
			mousepos_previous = mousepos = take_mouse_position_from_lparam();
			ComputeMouseVector();
			OldMouseVector = MouseVector; // for touch devices to avoid unwanted snappings
			downevent = 1;
			MouseState = 1;
			return 0;

		case WM_LBUTTONUP:
			mousepos = take_mouse_position_from_lparam();
			ComputeMouseVector();
			MouseState = 0;
			ReleaseCapture();
			return 0;

		case WM_MOUSEMOVE:
			mousepos = take_mouse_position_from_lparam();
			ComputeMouseVector();
			return 0;
		case  WM_MOUSEWHEEL:
			//shiftdown = (wParam&MK_SHIFT) ? 1 : 0;  ctrldown = (wParam&MK_CONTROL) ? 1 : 0;
			mousewheel += GET_WHEEL_DELTA_WPARAM(wParam) / WHEEL_DELTA;
			return 0;
		case WM_SIZE:
			if(reshape)
				reshape(LOWORD(lParam), HIWORD(lParam));
			PostMessage(hWnd, WM_PAINT, 0, 0);
			return 0;
		case WM_SETFOCUS:
			focus = 1;
			break;
		case WM_KILLFOCUS:
			focus = 0;
			break;
		case WM_CLOSE:
			if (preshutdown)
				preshutdown();
			PostQuitMessage(0);
			return 0;
		}
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}


	bool WindowUp()
	{
		dmouse = { 0,0 };  // only used if mouse is being centered each frame
		downevent = 0;
		mousewheel = 0;    // reset to 0 each frame
		mousepos_previous = mousepos;
		OldMouseVector = MouseVector;
		MSG   msg;		   // Windows message		
		while (PeekMessage(&msg, hWnd, 0, 0, PM_NOREMOVE)) {
			if (GetMessage(&msg, hWnd, 0, 0)) {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			else
			{
				if (preshutdown)
					preshutdown();
				return false;
			}
		}

		if (centermouse && focus) {
			RECT rect;
			RECT crect;
			POINT pt;
			GetWindowRect(hWnd, &rect);
			GetClientRect(hWnd, &crect);
			GetCursorPos(&pt);
			if (1) // (windowed)
			{
				rect.left -= window_inset.left; // todo: look into the function: ScreenToClient(hwnd,point)
				rect.top -= window_inset.top;
			}
			if (centered_last_frame)
			{
				dmouse.x = (float)(pt.x - rect.left - (res.x / 2)) / (res.x / 2.0f);
				dmouse.y = -(float)(pt.y - rect.top - (res.y / 2)) / (res.y / 2.0f);
			}
			else
				dmouse = { 0,0 };
			SetCursorPos(res.x / 2 + rect.left, res.y / 2 + rect.top);
			MouseVector = float3(0, 0, -1);
			centered_last_frame = 1;
		}
		else
		{
			centered_last_frame = 0;
		}

		return true;
	}

};

// see dxwin.h for usage such as:
//    class DXWin : public MSWin


#endif //SANDBOX_MSWIN_H
