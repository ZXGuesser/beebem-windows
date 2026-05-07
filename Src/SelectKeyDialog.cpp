/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2020  Chris Needham

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

// User defined keyboard functionality.

#include <windows.h>

#include <string>

#include "SelectKeyDialog.h"
#include "KeyMap.h"
#include "Main.h"
#include "Messages.h"
#include "Resource.h"
#include "WindowUtils.h"

/****************************************************************************/

SelectKeyDialog* g_pSelectKeyDialog = nullptr;

/****************************************************************************/

SelectKeyDialog::SelectKeyDialog(HINSTANCE hInstance,
                                 HWND hwndParent,
                                 const std::string& Title,
                                 const std::string& SelectedKey,
                                 bool BeebKey,
                                 int Row,
                                 int Column,
                                 bool DoingShifted) :
	Dialog(hInstance, hwndParent, IDD_SELECT_KEY),
	m_Title(Title),
	m_SelectedKey(SelectedKey),
	m_BeebKey(BeebKey),
	m_Key(-1),
	m_Row(Row),
	m_Column(Column),
	m_DoingShifted(DoingShifted),
	m_Shift(false),
	m_Result(IDCANCEL)
{
}

/****************************************************************************/

INT_PTR SelectKeyDialog::DlgProc(UINT nMessage,
                                 WPARAM wParam,
                                 LPARAM /* lParam */)
{
	switch (nMessage)
	{
	case WM_INITDIALOG:
		SetWindowText(m_hwnd, m_Title.c_str());

		SetDlgItemText(IDC_ASSIGNED_KEYS, m_SelectedKey.c_str());

		if (!m_BeebKey)
		{
			ShowWindow(GetDlgItem(IDC_SHIFT), SW_HIDE);
		}
		return TRUE;

	case WM_ACTIVATE:
		if (LOWORD(wParam) == WA_INACTIVE)
		{
			hCurrentDialog = nullptr;
		}
		else
		{
			hCurrentDialog = m_hwnd;
			hCurrentAccelTable = nullptr;
		}
		break;

	case WM_SYSCOMMAND:
		if (wParam == SC_CLOSE)
		{
			m_Result = IDCANCEL;
			Close();
			return TRUE;
		}
		break;

	case WM_COMMAND:
		switch (wParam)
		{
		case IDC_CLEAR:
			SendMessage(m_hwndParent, WM_CLEAR_KEY_MAPPING, 0, 0);

			if (m_BeebKey)
			{
				m_SelectedKey = GetKeysUsed(m_Row, m_Column, m_DoingShifted);
			}
			else
			{
				m_SelectedKey = "";
			}

			SetDlgItemText(IDC_ASSIGNED_KEYS, m_SelectedKey.c_str());
			return TRUE;

		case IDOK:
			m_Result = IDCONTINUE;
			Close();
			return TRUE;
		}
		break;

	case WM_DESTROY:
		PostMessage(m_hwndParent, WM_SELECT_KEY_DIALOG_CLOSED, m_Result, 0);
		break;
	}

	return FALSE;
}

/****************************************************************************/

bool SelectKeyDialog::HandleMessage(const MSG& msg)
{
	if (msg.message == WM_KEYDOWN || msg.message == WM_SYSKEYDOWN)
	{
		m_Key = (int)msg.wParam;
		m_Shift = IsDlgItemChecked(IDC_SHIFT);
		m_Result = IDOK;

		Close();

		return true;
	}

	return false;
}

/****************************************************************************/

int SelectKeyDialog::Key() const
{
	return m_Key;
}

/****************************************************************************/

bool SelectKeyDialog::Shift() const
{
	return m_Shift;
}

/****************************************************************************/
