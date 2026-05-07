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

// User defined keyboard functionality.

#include <windows.h>

#include <stdio.h>
#include <string.h>

#include <string>

#include "UserKeyboardDialog.h"
#include "KeyMap.h"
#include "Main.h"
#include "Messages.h"
#include "Resource.h"
#include "SelectKeyDialog.h"
#include "WindowUtils.h"

/****************************************************************************/

UserKeyboardDialog* g_pUserKeyboardDialog = nullptr;

// Colour used to highlight the selected key.
static const COLORREF HighlightColour   = 0x00FF0080; // Purple
static const COLORREF FunctionKeyColour = 0x000000FF; // Red
static const COLORREF NormalKeyColour   = 0x00000000; // Black

static const char* szSelectKeyDialogTitle[2] = {
	"Press key for unshifted press...",
	"Press key for shifted press..."
};

/****************************************************************************/

UserKeyboardDialog::UserKeyboardDialog(HINSTANCE hInstance,
                                       HWND hwndParent) :
	Dialog(hInstance, hwndParent, IDD_USERKYBRD),
	m_hwndBBCKey(nullptr),
	m_SelectedCtrlID(0),
	m_OldKeyColour(RGB(0, 0, 0)),
	m_BBCRow(0),
	m_BBCCol(0),
	m_DoingShifted(false)
{
}

/****************************************************************************/

INT_PTR UserKeyboardDialog::DlgProc(UINT nMessage,
                                    WPARAM wParam,
                                    LPARAM lParam)
{
	switch (nMessage)
	{
	case WM_INITDIALOG:
		EnableWindow(m_hwndParent, FALSE);
		break;

	case WM_COMMAND:
		switch (wParam)
		{
		case IDOK:
		case IDCANCEL:
			EnableWindow(m_hwndParent, TRUE);

			Close();

			PostMessage(m_hwndParent, WM_USER_KEYBOARD_DIALOG_CLOSED, 0, 0);
			break;

		default:
			SelectKeyMapping((UINT)wParam, (HWND)lParam);
			break;
		}
		return TRUE;

	case WM_DRAWITEM:
		// Draw the key.
		OnDrawItem((UINT)wParam, (LPDRAWITEMSTRUCT)lParam);
		return TRUE;

	case WM_CLEAR_KEY_MAPPING:
		ClearUserKeyMapping(m_BBCRow, m_BBCCol, m_DoingShifted);
		break;

	case WM_SELECT_KEY_DIALOG_CLOSED:
		if (wParam == IDOK)
		{
			// Assign the BBC key to the PC key.
			SetUserKeyMapping(
				m_BBCRow,
				m_BBCCol,
				m_DoingShifted,
				g_pSelectKeyDialog->Key(),
				g_pSelectKeyDialog->Shift()
			);
		}

		delete g_pSelectKeyDialog;
		g_pSelectKeyDialog = nullptr;

		if ((wParam == IDOK || wParam == IDCONTINUE) && !m_DoingShifted)
		{
			m_DoingShifted = true;

			std::string UsedKeys = GetKeysUsed(m_BBCRow, m_BBCCol, m_DoingShifted);

			g_pSelectKeyDialog = new SelectKeyDialog(
				hInst,
				m_hwnd,
				szSelectKeyDialogTitle[m_DoingShifted ? 1 : 0],
				UsedKeys,
				true,
				m_BBCRow,
				m_BBCCol,
				m_DoingShifted
			);

			g_pSelectKeyDialog->Open();
		}
		else
		{
			m_SelectedCtrlID = 0;

			// Show the key as not depressed, i.e., normal.
			SetKeyColour(m_OldKeyColour);
		}
		return TRUE;

	default:
		break;
	}

	return FALSE;
}

/****************************************************************************/

void UserKeyboardDialog::SetKeyColour(COLORREF aColour)
{
	HDC hdc = GetDC(m_hwndBBCKey);
	SetBkColor(hdc, aColour);
	ReleaseDC(m_hwndBBCKey, hdc);
	InvalidateRect(m_hwndBBCKey, nullptr, TRUE);
	UpdateWindow(m_hwndBBCKey);
}

/****************************************************************************/

void UserKeyboardDialog::SelectKeyMapping(UINT ctrlID, HWND hwndCtrl)
{
	// Set the placeholders.
	SetRowCol(ctrlID);

	m_OldKeyColour = GetKeyColour(ctrlID);

	m_hwndBBCKey = hwndCtrl;
	m_SelectedCtrlID = ctrlID;

	m_DoingShifted = false;

	std::string UsedKeys = GetKeysUsed(m_BBCRow, m_BBCCol, m_DoingShifted);

	// Now ask the user to input the PC key to assign to the BBC key.
	g_pSelectKeyDialog = new SelectKeyDialog(
		hInst,
		m_hwnd,
		szSelectKeyDialogTitle[m_DoingShifted ? 1 : 0],
		UsedKeys,
		true,
		m_BBCRow,
		m_BBCCol,
		m_DoingShifted
	);

	g_pSelectKeyDialog->Open();
}

/****************************************************************************/

void UserKeyboardDialog::SetRowCol(UINT ctrlID)
{
	switch (ctrlID)
	{
	// Character keys.
	case IDK_A: m_BBCRow = 4; m_BBCCol = 1; break;
	case IDK_B: m_BBCRow = 6; m_BBCCol = 4; break;
	case IDK_C: m_BBCRow = 5; m_BBCCol = 2; break;
	case IDK_D: m_BBCRow = 3; m_BBCCol = 2; break;
	case IDK_E: m_BBCRow = 2; m_BBCCol = 2; break;
	case IDK_F: m_BBCRow = 4; m_BBCCol = 3; break;
	case IDK_G: m_BBCRow = 5; m_BBCCol = 3; break;
	case IDK_H: m_BBCRow = 5; m_BBCCol = 4; break;
	case IDK_I: m_BBCRow = 2; m_BBCCol = 5; break;
	case IDK_J: m_BBCRow = 4; m_BBCCol = 5; break;
	case IDK_K: m_BBCRow = 4; m_BBCCol = 6; break;
	case IDK_L: m_BBCRow = 5; m_BBCCol = 6; break;
	case IDK_M: m_BBCRow = 6; m_BBCCol = 5; break;
	case IDK_N: m_BBCRow = 5; m_BBCCol = 5; break;
	case IDK_O: m_BBCRow = 3; m_BBCCol = 6; break;
	case IDK_P: m_BBCRow = 3; m_BBCCol = 7; break;
	case IDK_Q: m_BBCRow = 1; m_BBCCol = 0; break;
	case IDK_R: m_BBCRow = 3; m_BBCCol = 3; break;
	case IDK_S: m_BBCRow = 5; m_BBCCol = 1; break;
	case IDK_T: m_BBCRow = 2; m_BBCCol = 3; break;
	case IDK_U: m_BBCRow = 3; m_BBCCol = 5; break;
	case IDK_V: m_BBCRow = 6; m_BBCCol = 3; break;
	case IDK_W: m_BBCRow = 2; m_BBCCol = 1; break;
	case IDK_X: m_BBCRow = 4; m_BBCCol = 2; break;
	case IDK_Y: m_BBCRow = 4; m_BBCCol = 4; break;
	case IDK_Z: m_BBCRow = 6; m_BBCCol = 1; break;

	// Number keys.
	case IDK_0: m_BBCRow = 2; m_BBCCol = 7; break;
	case IDK_1: m_BBCRow = 3; m_BBCCol = 0; break;
	case IDK_2: m_BBCRow = 3; m_BBCCol = 1; break;
	case IDK_3: m_BBCRow = 1; m_BBCCol = 1; break;
	case IDK_4: m_BBCRow = 1; m_BBCCol = 2; break;
	case IDK_5: m_BBCRow = 1; m_BBCCol = 3; break;
	case IDK_6: m_BBCRow = 3; m_BBCCol = 4; break;
	case IDK_7: m_BBCRow = 2; m_BBCCol = 4; break;
	case IDK_8: m_BBCRow = 1; m_BBCCol = 5; break;
	case IDK_9: m_BBCRow = 2; m_BBCCol = 6; break;

	// Function keys.
	case IDK_F0: m_BBCRow = 2; m_BBCCol = 0; break;
	case IDK_F1: m_BBCRow = 7; m_BBCCol = 1; break;
	case IDK_F2: m_BBCRow = 7; m_BBCCol = 2; break;
	case IDK_F3: m_BBCRow = 7; m_BBCCol = 3; break;
	case IDK_F4: m_BBCRow = 1; m_BBCCol = 4; break;
	case IDK_F5: m_BBCRow = 7; m_BBCCol = 4; break;
	case IDK_F6: m_BBCRow = 7; m_BBCCol = 5; break;
	case IDK_F7: m_BBCRow = 1; m_BBCCol = 6; break;
	case IDK_F8: m_BBCRow = 7; m_BBCCol = 6; break;
	case IDK_F9: m_BBCRow = 7; m_BBCCol = 7; break;

	// Special keys.
	case IDK_LEFT:       m_BBCRow = 1;  m_BBCCol = 9; break;
	case IDK_RIGHT:      m_BBCRow = 7;  m_BBCCol = 9; break;
	case IDK_UP:         m_BBCRow = 3;  m_BBCCol = 9; break;
	case IDK_DOWN:       m_BBCRow = 2;  m_BBCCol = 9; break;
	case IDK_BREAK:      m_BBCRow = -2; m_BBCCol = -2; break;
	case IDK_COPY:       m_BBCRow = 6;  m_BBCCol = 9; break;
	case IDK_DEL:        m_BBCRow = 5;  m_BBCCol = 9; break;
	case IDK_CAPS:       m_BBCRow = 4;  m_BBCCol = 0; break;
	case IDK_TAB:        m_BBCRow = 6;  m_BBCCol = 0; break;
	case IDK_CTRL:       m_BBCRow = 0;  m_BBCCol = 1; break;
	case IDK_SPACE:      m_BBCRow = 6;  m_BBCCol = 2; break;
	case IDK_RETURN:     m_BBCRow = 4;  m_BBCCol = 9; break;
	case IDK_ESC:        m_BBCRow = 7;  m_BBCCol = 0; break;
	case IDK_SHIFT_L:    m_BBCRow = 0;  m_BBCCol = 0; break;
	case IDK_SHIFT_R:    m_BBCRow = 0;  m_BBCCol = 0; break;
	case IDK_SHIFT_LOCK: m_BBCRow = 5;  m_BBCCol = 0; break;

	// Special Character keys.
	case IDK_SEMI_COLON:   m_BBCRow = 5; m_BBCCol = 7; break;
	case IDK_EQUALS:       m_BBCRow = 1; m_BBCCol = 7; break;
	case IDK_COMMA:        m_BBCRow = 6; m_BBCCol = 6; break;
	case IDK_CARET:        m_BBCRow = 1; m_BBCCol = 8; break;
	case IDK_DOT:          m_BBCRow = 6; m_BBCCol = 7; break;
	case IDK_FWDSLASH:     m_BBCRow = 6; m_BBCCol = 8; break;
	case IDK_STAR:         m_BBCRow = 4; m_BBCCol = 8; break;
	case IDK_OPEN_SQUARE:  m_BBCRow = 3; m_BBCCol = 8; break;
	case IDK_BACKSLASH:    m_BBCRow = 7; m_BBCCol = 8; break;
	case IDK_CLOSE_SQUARE: m_BBCRow = 5; m_BBCCol = 8; break;
	case IDK_AT:           m_BBCRow = 4; m_BBCCol = 7; break;
	case IDK_UNDERSCORE:   m_BBCRow = 2; m_BBCCol = 8; break;

	default:
		m_BBCRow = 0; m_BBCCol = 0;
		break;
	}
}

/****************************************************************************/

void UserKeyboardDialog::OnDrawItem(UINT CtrlID, LPDRAWITEMSTRUCT lpDrawItemStruct)
{
	// Set the Pen and Background Brush.
	HBRUSH aBrush = CreateSolidBrush(GetKeyColour(CtrlID));
	HPEN aPen = CreatePen(PS_NULL, 1, RGB(0, 0, 0));

	// Copy into the Device Context.
	aBrush = (HBRUSH)SelectObject(lpDrawItemStruct->hDC, aBrush);
	aPen = (HPEN)SelectObject(lpDrawItemStruct->hDC, aPen);

	// Draw the rectangle.
	SetBkColor(lpDrawItemStruct->hDC, GetKeyColour(CtrlID));
	Rectangle(lpDrawItemStruct->hDC,
	          lpDrawItemStruct->rcItem.left,
	          lpDrawItemStruct->rcItem.top,
	          lpDrawItemStruct->rcItem.right,
	          lpDrawItemStruct->rcItem.bottom);

	// Draw border.
	DrawBorder(lpDrawItemStruct->hDC,
	           lpDrawItemStruct->rcItem,
	           lpDrawItemStruct->itemState == (ODS_FOCUS | ODS_SELECTED));

	// Draw the text.
	DrawText(lpDrawItemStruct->hDC,
	         lpDrawItemStruct->rcItem,
	         lpDrawItemStruct->hwndItem,
	         0x00FFFFFF,
	         lpDrawItemStruct->itemState == (ODS_FOCUS | ODS_SELECTED));

	// Clear up.
	DeleteObject(SelectObject(lpDrawItemStruct->hDC, aBrush));
	DeleteObject(SelectObject(lpDrawItemStruct->hDC, aPen));
}

/****************************************************************************/

void UserKeyboardDialog::DrawSides(HDC hDC, RECT rect, COLORREF TopLeft, COLORREF BottomRight)
{
	HPEN hTopLeftPen = CreatePen(PS_SOLID, 1, TopLeft);
	HPEN hBottomRightPen = CreatePen(PS_SOLID, 1, BottomRight);

	HPEN hOldPen = (HPEN)SelectObject(hDC, hTopLeftPen);

	POINT point;
	MoveToEx(hDC, rect.left, rect.bottom - 1, &point);
	LineTo(hDC, rect.left, rect.top);
	LineTo(hDC, rect.right - 1, rect.top);

	SelectObject(hDC, hBottomRightPen);
	LineTo(hDC, rect.right - 1, rect.bottom - 1);
	LineTo(hDC, rect.left, rect.bottom - 1);

	// Clean up.
	SelectObject(hDC, hOldPen);
	DeleteObject(hTopLeftPen);
	DeleteObject(hBottomRightPen);
}

/****************************************************************************/

void UserKeyboardDialog::DrawBorder(HDC hDC, RECT rect, BOOL Depressed)
{
	// Draw outer border.
	if (Depressed)
		DrawSides(hDC, rect, 0x00000000, 0x00FFFFFF);
	else
		DrawSides(hDC, rect, 0x00FFFFFF, 0x00000000);

	// Draw inner border.
	rect.top++;
	rect.left++;
	rect.right--;
	rect.bottom--;

	if (Depressed)
		DrawSides(hDC, rect, 0x00777777, 0x00FFFFFF);
	else
		DrawSides(hDC, rect, 0x00FFFFFF, 0x00777777);
}

/****************************************************************************/

void UserKeyboardDialog::DrawText(HDC hDC, RECT rect, HWND hwndCtrl, COLORREF colour, bool Depressed)
{
	SIZE Size;
	CHAR text[10];

	GetWindowText(hwndCtrl, text, 9);

	if (GetTextExtentPoint32(hDC, text, (int)strlen(text), &Size))
	{
		// Set text colour.
		SetTextColor(hDC, colour);

		// Output text.
		const int Offset = Depressed ? 1 : 0;

		TextOut(hDC,
		        ((rect.right - rect.left) - Size.cx) / 2 + Offset,
		        ((rect.bottom - rect.top) - Size.cy) / 2 + Offset,
		        text,
		        (int)strlen(text));
	}
}

/****************************************************************************/

COLORREF UserKeyboardDialog::GetKeyColour(UINT ctrlID)
{
	if (m_SelectedCtrlID == ctrlID)
	{
		return HighlightColour;
	}

	switch (ctrlID)
	{
	case IDK_F0:
	case IDK_F1:
	case IDK_F2:
	case IDK_F3:
	case IDK_F4:
	case IDK_F5:
	case IDK_F6:
	case IDK_F7:
	case IDK_F8:
	case IDK_F9:
		return FunctionKeyColour;

	default:
		return NormalKeyColour;
	}
}

/****************************************************************************/
