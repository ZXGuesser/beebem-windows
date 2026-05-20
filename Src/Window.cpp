/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2026 Chris Needham

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

#include "Window.h"

/****************************************************************************/

Window::Window() :
	m_hwnd(nullptr)
{
}

/****************************************************************************/

HWND Window::GetDlgItem(int nID)
{
	return ::GetDlgItem(m_hwnd, nID);
}

/****************************************************************************/

std::string Window::GetDlgItemText(int nID)
{
	int Length = GetWindowTextLength(GetDlgItem(nID));

	std::vector<char> Text;
	Text.resize(Length + 1);

	::GetDlgItemText(m_hwnd, nID, &Text[0], (int)Text.size());

	return std::string(&Text[0]);
}

/****************************************************************************/

LRESULT Window::SendDlgItemMessage(int nID, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	return ::SendDlgItemMessage(m_hwnd, nID, Msg, wParam, lParam);
}

/****************************************************************************/

void Window::SetDlgItemText(int nID, const std::string& str)
{
	SetWindowText(GetDlgItem(nID), str.c_str());
}

/****************************************************************************/

bool Window::IsDlgItemChecked(int nID)
{
	return SendDlgItemMessage(nID, BM_GETCHECK, 0, 0) == BST_CHECKED;
}

/****************************************************************************/

void Window::SetDlgItemChecked(int nID, bool bChecked)
{
	SendDlgItemMessage(nID, BM_SETCHECK, bChecked ? BST_CHECKED : BST_UNCHECKED, 0);
}

/****************************************************************************/

void Window::SetDlgItemFocus(int nID)
{
	SetFocus(GetDlgItem(nID));
}

/****************************************************************************/

void Window::EnableDlgItem(int nID, bool bEnable)
{
	EnableWindow(GetDlgItem(nID), bEnable);
}

/****************************************************************************/

void Window::EndDialog(INT_PTR nResult)
{
	::EndDialog(m_hwnd, nResult);
}

/****************************************************************************/
