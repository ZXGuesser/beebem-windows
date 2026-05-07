/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 1997  Laurie Whiffen

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

#ifndef USERKEYBOARDDIALOG_HEADER
#define USERKEYBOARDDIALOG_HEADER

#include "Dialog.h"

class UserKeyboardDialog : public Dialog
{
	public:
		UserKeyboardDialog(HINSTANCE hInstance,
		                   HWND hwndParent);

	private:
		virtual INT_PTR DlgProc(UINT nMessage,
		                        WPARAM wParam,
		                        LPARAM lParam);

		void SetKeyColour(COLORREF aColour);
		void SelectKeyMapping(UINT ctrlID, HWND hwndCtrl);
		void SetRowCol(UINT ctrlID);
		void OnDrawItem(UINT CtrlID, LPDRAWITEMSTRUCT lpDrawItemStruct);
		void DrawSides(HDC hDC, RECT rect, COLORREF TopLeft, COLORREF BottomRight);
		void DrawBorder(HDC hDC, RECT rect, BOOL Depressed);
		void DrawText(HDC hDC, RECT rect, HWND hwndCtrl, COLORREF colour, bool Depressed);
		COLORREF GetKeyColour(UINT ctrlID);

	private:
		HWND m_hwndBBCKey; // Holds the BBCKey control handle which is now selected.
		UINT m_SelectedCtrlID; // Holds ctrlId of selected key (or 0 if none selected).
		COLORREF m_OldKeyColour;
		int m_BBCRow; // Used to store the Row and Col values while we wait
		int m_BBCCol; // for a key press from the User.
		bool m_DoingShifted; // Selecting shifted or unshifted key press.
};

extern UserKeyboardDialog* g_pUserKeyboardDialog;

#endif
