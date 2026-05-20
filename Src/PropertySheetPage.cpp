/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2026  Chris Needham

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

#include "PropertySheetPage.h"

/****************************************************************************/

PropertySheetPage::PropertySheetPage(HINSTANCE hInstance, int DialogID)
{
	ZeroMemory(&m_Page, sizeof(m_Page));

	m_Page.dwSize      = sizeof(m_Page);
	m_Page.hInstance   = hInstance;
	m_Page.pszTemplate = MAKEINTRESOURCE(DialogID);
	m_Page.pfnDlgProc  = DlgProcCallback;
	m_Page.lParam      = reinterpret_cast<LPARAM>(this);
}

/****************************************************************************/

const PROPSHEETPAGE* PropertySheetPage::GetPropSheetPage() const
{
	return &m_Page;
}

/****************************************************************************/

INT_PTR CALLBACK PropertySheetPage::DlgProcCallback(HWND hwnd,
                                                    UINT nMessage,
                                                    WPARAM wParam,
                                                    LPARAM lParam)
{
	PropertySheetPage* pDialog;

	if (nMessage == WM_INITDIALOG)
	{
		PROPSHEETPAGE* pPage = reinterpret_cast<PROPSHEETPAGE*>(lParam);

		pDialog = reinterpret_cast<PropertySheetPage*>(pPage->lParam);
		pDialog->m_hwnd = hwnd;

		SetWindowLongPtr(hwnd, GWLP_USERDATA, pPage->lParam);
	}
	else
	{
		pDialog = reinterpret_cast<PropertySheetPage*>(
			GetWindowLongPtr(hwnd, GWLP_USERDATA)
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

INT_PTR PropertySheetPage::DlgProc(UINT nMessage,
                                   WPARAM wParam,
                                   LPARAM lParam)
{
	switch (nMessage)
	{
		case WM_INITDIALOG:
			OnInitDialog();
			return TRUE;

		case WM_NOTIFY: {
			NMHDR* pnmhdr = (NMHDR*)lParam;

			switch (pnmhdr->code)
			{
				case PSN_SETACTIVE: {
					bool Accept = OnSetActive();
					SetWindowLongPtr(m_hwnd, DWLP_MSGRESULT, Accept ? 0 : -1);
					return TRUE;
				}

				case PSN_KILLACTIVE: {
					bool Ok = OnKillActive();
					SetWindowLongPtr(m_hwnd, DWLP_MSGRESULT, Ok ? FALSE : TRUE);
					return TRUE;
				}

				case PSN_APPLY: {
					bool Ok = OnApply();
					SetWindowLongPtr(m_hwnd, DWLP_MSGRESULT, Ok ? PSNRET_NOERROR : PSNRET_INVALID);
					return TRUE;
				}

				case PSN_RESET:
					return TRUE;

				case PSN_QUERYCANCEL:
					SetWindowLongPtr(m_hwnd, DWLP_MSGRESULT, FALSE);
					return TRUE;
			}
			break;
		}

		default:
			break;
	}

	return HandleMessage(nMessage, wParam, lParam);
}

/****************************************************************************/

INT_PTR PropertySheetPage::HandleMessage(UINT /* nMessage */,
                                         WPARAM /* wParam */,
                                         LPARAM /* lParam */)
{
	return 0;
}

/****************************************************************************/

void PropertySheetPage::OnInitDialog()
{
}

/****************************************************************************/

bool PropertySheetPage::OnSetActive()
{
	return true; // Allow activation
}

/****************************************************************************/

bool PropertySheetPage::OnKillActive()
{
	return true; // Allow deactivation
}

/****************************************************************************/

bool PropertySheetPage::OnApply()
{
	return true;
}

/****************************************************************************/
