/*
 *
 * Copyright (c) 2013, Yujin Robot Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the Yujin Robot Co., Ltd.  The name of the
 * Yujin Robot Co., Ltd. may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include "../include/zeroconf_bonjour/stdafx.h"

#include "../include/zeroconf_bonjour/basetype.h"
#include "../include/zeroconf_bonjour/bonjour.h"
#include "../include/zeroconf_bonjour/messenger.h"

#include <tchar.h>

HWND g_hWnd;
Bonjour* g_pBonjour;
Messenger* g_pMessenger;

BOOL CreateWindowHandle(HINSTANCE hInstance);
LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
void CALLBACK UpdateTimerProc(HWND hWnd, UINT uMsg, UINT uEvent, DWORD dwTime);

int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
	HANDLE hMutex = ::CreateMutex(NULL, FALSE, "ZeroconfBonjourMutex");

	if (GetLastError() == ERROR_ALREADY_EXISTS)
		return 0; 

	if (!CreateWindowHandle(hInstance))
		return 0;

	g_pBonjour = new Bonjour(g_hWnd);
	g_pBonjour->Initialize();

	g_pMessenger = new Messenger(g_hWnd, g_pBonjour);
	g_pMessenger->Initialize();

	std::cout << "[zeroconf] Ready" << std::endl;

	UINT uUpdateTimer = ::SetTimer(NULL, 0, 10000, UpdateTimerProc);

	::PostMessage(g_hWnd, WM_SCAN_BEGIN, 0, 0);

	MSG	msg;

	while (::GetMessage(&msg, NULL, 0, 0)) 
	{
		::TranslateMessage(&msg);
		::DispatchMessage(&msg);
	}

	::KillTimer(NULL, uUpdateTimer);

	g_pMessenger->Uninitialize();
	delete g_pMessenger;

	g_pBonjour->Uninitialize();
	delete g_pBonjour;

	return 0;
}

BOOL CreateWindowHandle(HINSTANCE hInstance) 
{
	WNDCLASS wc;

	wc.lpszClassName = _T("ZeroconfBonjourClass");
	wc.lpfnWndProc = (WNDPROC)WindowProc;
	wc.style = CS_VREDRAW | CS_HREDRAW;
	wc.hInstance = hInstance;
	wc.hIcon = NULL;
	wc.hCursor = NULL;
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wc.lpszMenuName = NULL;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;

	if (!::RegisterClass(&wc))
		return FALSE;

	g_hWnd = ::CreateWindow(_T("ZeroconfBonjourClass"), 
		_T("bonjour"),
		CW_USEDEFAULT, 
		CW_USEDEFAULT,
		CW_USEDEFAULT, 
		CW_USEDEFAULT, 
		0, 
		NULL, 
		NULL, 
		hInstance,
		NULL);
 
	return g_hWnd != NULL;
}

LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_CREATE:
		{
		}
		break;
	case WM_SCAN_COMPLETE:
		{
			g_pBonjour->Completed();
		}
		break;
	case WM_SCAN_BEGIN:
		{
			g_pBonjour->Begin();
		}
		break;
	case WM_SCAN_ENUMERATE:
		{
			g_pBonjour->Scan();
		}
		break;
	case WM_PUBLISH_ADD_SERVICE:
		{
			ServiceItem* psi = (ServiceItem*)wParam;

			if (!psi)
				break;

			g_pBonjour->AddServiceToPublish(psi->name.data(),
				psi->type.data(),
				psi->domain.data(),
				psi->hostport,
				psi->description.data());
		}
		break;
	case WM_PUBLISH_REMOVE_SERVICE:
		{
			ServiceItem* psi = (ServiceItem*)wParam;

			if (!psi)
				break;

			g_pBonjour->RemoveServiceToPublish(psi->name.data(),
				psi->type.data());
		}
		break;
	case WM_DISCOVER_ADD_SERVICE:
		{
			g_pBonjour->AddServiceToDiscover((const char*)wParam);
		}
		break;
	case WM_DISCOVER_REMOVE_SERVICE:
		{
			g_pBonjour->RemoveServiceToDiscover((const char*)wParam);
		}
		break;
	default:
		break;
	}

	return ::DefWindowProc(hWnd, message, wParam, lParam);
}

void CALLBACK UpdateTimerProc(HWND hWnd, UINT uMsg, UINT uEvent, DWORD dwTime)
{
	if (g_hWnd)
	{
		::PostMessage(g_hWnd, WM_SCAN_BEGIN, 0, 0);
	}
}
