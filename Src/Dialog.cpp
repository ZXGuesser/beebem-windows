/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2023 Chris Needham

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public
License along with this program; if not, write to the Free
Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
Boston, MA  02110-1301, USA.
****************************************************************/

#include <windows.h>

#include <vector>

#include "Dialog.h"
#include "Main.h"
#include "WindowUtils.h"

/****************************************************************************/

Dialog::Dialog(HINSTANCE hInstance,
               HWND hwndParent,
               int DialogID) :
	m_hInstance(hInstance),
	m_hwndParent(hwndParent),
	m_DialogID(DialogID)
{
}

/****************************************************************************/

// Show modal dialog box.

bool Dialog::DoModal()
{
	INT_PTR Result = DialogBoxParam(m_hInstance,
	                                MAKEINTRESOURCE(m_DialogID),
	                                m_hwndParent,
	                                DlgProcCallback,
	                                reinterpret_cast<LPARAM>(this));

	return Result == IDOK;
}

/****************************************************************************/

// Open modeless dialog box.

bool Dialog::Open()
{
	if (m_hwnd == nullptr)
	{
		m_hwnd = CreateDialogParam(m_hInstance,
		                           MAKEINTRESOURCE(m_DialogID),
		                           m_hwndParent,
		                           DlgProcCallback,
		                           reinterpret_cast<LPARAM>(this));

		ShowWindow(m_hwnd, SW_SHOW);
	}

	return true;
}

/****************************************************************************/

// Close modeless dialog box.

void Dialog::Close()
{
	DestroyWindow(m_hwnd);
	m_hwnd = nullptr;
}

/****************************************************************************/

INT_PTR CALLBACK Dialog::DlgProcCallback(HWND hwnd,
                                         UINT nMessage,
                                         WPARAM wParam,
                                         LPARAM lParam)
{
	Dialog* pDialog;

	if (nMessage == WM_INITDIALOG)
	{
		SetWindowLongPtr(hwnd, DWLP_USER, lParam);
		pDialog = reinterpret_cast<Dialog*>(lParam);
		pDialog->m_hwnd = hwnd;

		DisableRoundedCorners(hwnd);

		CentreWindow(pDialog->m_hwndParent, hwnd);
	}
	else
	{
		pDialog = reinterpret_cast<Dialog*>(
			GetWindowLongPtr(hwnd, DWLP_USER)
		);
	}

	if (pDialog != nullptr)
	{
		return pDialog->DlgProc(nMessage, wParam, lParam);
	}
	else
	{
		return FALSE;
	}
}

/****************************************************************************/
