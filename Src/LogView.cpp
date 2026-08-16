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
#include <windowsx.h>

#include <algorithm>
#include <utility>

#include "LogView.h"
#include "Messages.h"

/****************************************************************************/

LogView::LogView(std::deque<EconetLogMessage>* pLogBuffer) :
	m_hwnd(nullptr),
	m_hwndParent(nullptr),
	m_pLogBuffer(pLogBuffer),
	m_hFont(nullptr),
	m_LineHeight(0),
	m_ScrollPos(0),
	m_Dragging(false),
	m_SelectionStart(-1),
	m_SelectionEnd(-1)
{
}

/****************************************************************************/

bool LogView::InitClass(HINSTANCE hInstance)
{
	WNDCLASS wc;
	ZeroMemory(&wc, sizeof(wc));

	wc.lpfnWndProc   = WndProcCallback;
	wc.hInstance     = hInstance;
	wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = nullptr;
	wc.lpszClassName = "LogView";

	return RegisterClass(&wc) != 0;
}

/****************************************************************************/

bool LogView::Create(HINSTANCE hInstance, HWND hwndParent, int id, const RECT& Rect)
{
	m_hwndParent = hwndParent;

	m_hwnd = CreateWindow("LogView",
	                      nullptr,
	                      WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_BORDER,
	                      Rect.left,
	                      Rect.top,
	                      Rect.right - Rect.left,
	                      Rect.bottom - Rect.top,
	                      hwndParent,
	                      (HMENU)(INT_PTR)id,
	                      hInstance,
	                      this);

	return m_hwnd != nullptr;
}

/****************************************************************************/

void LogView::AppendLog(bool BufferFull)
{
	if (m_hwnd != nullptr)
	{
		RECT rc;
		GetClientRect(m_hwnd, &rc);

		int LinesVisible = rc.bottom / m_LineHeight;

		// The producer can add several messages before this posted UI update is
		// handled.  Use the scrollbar's existing range, rather than the current
		// buffer size, to determine whether the viewport was at the old end.
		SCROLLINFO ScrollInfo;
		ZeroMemory(&ScrollInfo, sizeof(ScrollInfo));

		ScrollInfo.cbSize = sizeof(ScrollInfo);
		ScrollInfo.fMask = SIF_RANGE | SIF_PAGE;

		GetScrollInfo(m_hwnd, SB_VERT, &ScrollInfo);

		int PreviousMaxScrollPos = std::max(ScrollInfo.nMin,
		                                    ScrollInfo.nMax - (int)ScrollInfo.nPage + 1);

		// Follow a live log only when it was already being viewed at its end.
		// Otherwise keep the current viewport stable for users reviewing history.
		if (m_ScrollPos >= PreviousMaxScrollPos)
		{
			m_ScrollPos = std::max(0, (int)m_pLogBuffer->size() - LinesVisible);
		}

		if (BufferFull)
		{
			// The buffer is full and the oldest entry has been removed.
			// Adjust the selection to that the same range of lines
			// is selected.

			if (m_SelectionStart > m_SelectionEnd)
			{
				std::swap(m_SelectionStart, m_SelectionEnd);
			}

			if (m_SelectionStart != -1)
			{
				m_SelectionStart--;
			}

			if (m_SelectionEnd != -1)
			{
				m_SelectionEnd--;
			}

			if (m_SelectionEnd >= 0 && m_SelectionStart == -1)
			{
				m_SelectionStart = 0;
			}

			if (m_SelectionStart == -1)
			{
				SendMessage(m_hwndParent, WM_ECONET_LOG_SELECT_MESSAGE, 0, (LPARAM)nullptr);
			}
		}

		UpdateScrollBar();

		InvalidateRect(m_hwnd, nullptr, FALSE);
	}
}

/****************************************************************************/

void LogView::Clear()
{
	m_pLogBuffer->clear();

	m_ScrollPos = 0;
	m_SelectionStart = -1;
	m_SelectionEnd = -1;

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, TRUE);

	SendMessage(m_hwndParent, WM_ECONET_LOG_SELECT_MESSAGE, 0, (LPARAM)nullptr);
}

/****************************************************************************/

LRESULT CALLBACK LogView::WndProcCallback(HWND hwnd,
                                          UINT nMessage,
                                          WPARAM wParam,
                                          LPARAM lParam)
{
	LogView* pLogView = (LogView*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

	switch (nMessage)
	{
		case WM_NCCREATE: {
			CREATESTRUCT* pCreateStruct = (CREATESTRUCT*)lParam;

			pLogView = (LogView*)pCreateStruct->lpCreateParams;

			pLogView->m_hwnd = hwnd;

			SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)pLogView);
			break;
		}
	}

	if (pLogView != nullptr)
	{
		return pLogView->WndProc(nMessage, wParam, lParam);
	}
	else
	{
		return DefWindowProc(hwnd, nMessage, wParam, lParam);
	}
}

/****************************************************************************/

LRESULT LogView::WndProc(UINT nMessage, WPARAM wParam, LPARAM lParam)
{
	switch (nMessage)
	{
		case WM_NCCREATE:
			OnNcCreate();
			return TRUE;

		case WM_PAINT:
			OnPaint();
			return 0;

		case WM_ERASEBKGND:
			return 1; // Prevent default behaviour.

		case WM_SIZE:
			UpdateScrollBar();
			return 0;

		case WM_LBUTTONDOWN:
			OnLButtonDown(GET_Y_LPARAM(lParam));
			return 0;

		case WM_MOUSEMOVE:
			OnMouseMove(GET_Y_LPARAM(lParam));
			return 0;

		case WM_LBUTTONUP:
			OnLButtonUp(GET_Y_LPARAM(lParam));
			return 0;

		case WM_VSCROLL:
			OnVScroll(LOWORD(wParam));
			return 0;

		case WM_MOUSEWHEEL:
			OnMouseWheel(GET_WHEEL_DELTA_WPARAM(wParam));
			return 0;
	}

	return DefWindowProc(m_hwnd, nMessage, wParam, lParam);
}

/****************************************************************************/

void LogView::OnNcCreate()
{
	m_hFont = (HFONT)SendMessage(m_hwndParent, WM_GETFONT, 0, 0);

	HDC hdc = GetDC(m_hwnd);

	HFONT hOldFont = (HFONT)SelectObject(hdc, m_hFont);

	TEXTMETRIC tm;
	GetTextMetrics(hdc, &tm);

	m_LineHeight = tm.tmHeight + tm.tmExternalLeading;

	SelectObject(hdc, hOldFont);
	ReleaseDC(m_hwnd, hdc);
}

/****************************************************************************/

void LogView::OnPaint()
{
	PAINTSTRUCT ps;

	HDC hdc = BeginPaint(m_hwnd, &ps);

	RECT rc;
	GetClientRect(m_hwnd, &rc);

	int Width = rc.right - rc.left;
	int Height = rc.bottom - rc.top;

	HDC hdcBuffer = CreateCompatibleDC(hdc);
	HBITMAP hBitmap = CreateCompatibleBitmap(hdc, Width, Height);
	HBITMAP hOldBitmap = (HBITMAP)SelectObject(hdcBuffer, hBitmap);

	FillRect(hdcBuffer, &rc, (HBRUSH)(COLOR_WINDOW + 1));

	HFONT hOldFont = (HFONT)SelectObject(hdcBuffer, m_hFont);

	SetBkMode(hdcBuffer, TRANSPARENT);

	COLORREF OldTextColour = SetTextColor(hdcBuffer, GetSysColor(COLOR_WINDOWTEXT));

	int LinesVisible = rc.bottom / m_LineHeight + 1;

	int Last = std::min((int)m_pLogBuffer->size(), m_ScrollPos + LinesVisible);

	int y = 0;

	for (int i = m_ScrollPos; i < Last; i++)
	{
		const EconetLogMessage& Message = (*m_pLogBuffer)[i];

		bool Selected = m_SelectionStart >= 0 &&
		                i >= std::min(m_SelectionStart, m_SelectionEnd) &&
		                i <= std::max(m_SelectionStart, m_SelectionEnd);

		if (Selected)
		{
			RECT LineRect = { 0, y, rc.right, y + m_LineHeight };
			FillRect(hdcBuffer, &LineRect, (HBRUSH)(COLOR_HIGHLIGHT + 1));
			SetTextColor(hdcBuffer, GetSysColor(COLOR_HIGHLIGHTTEXT));
		}

		TextOut(hdcBuffer, 4, y, Message.c_str(), (int)Message.length());

		if (Selected)
		{
			SetTextColor(hdcBuffer, GetSysColor(COLOR_WINDOWTEXT));
		}

        y += m_LineHeight;
    }

	SetTextColor(hdcBuffer, OldTextColour);
	SelectObject(hdcBuffer, hOldFont);

	BitBlt(hdc, 0, 0, Width, Height, hdcBuffer, 0, 0, SRCCOPY);

	SelectObject(hdcBuffer, hOldBitmap);
	DeleteObject(hBitmap);
	DeleteDC(hdcBuffer);

	EndPaint(m_hwnd, &ps);
}

/****************************************************************************/

void LogView::OnVScroll(int Event)
{
	SCROLLINFO ScrollInfo;
	ZeroMemory(&ScrollInfo, sizeof(ScrollInfo));

	ScrollInfo.cbSize = sizeof(ScrollInfo);
	ScrollInfo.fMask = SIF_ALL;

	GetScrollInfo(m_hwnd, SB_VERT, &ScrollInfo);

	switch (Event)
	{
		case SB_LINEUP:
			m_ScrollPos--;
			break;

		case SB_LINEDOWN:
			m_ScrollPos++;
			break;

		case SB_PAGEUP:
			m_ScrollPos -= ScrollInfo.nPage;
			break;

		case SB_PAGEDOWN:
			m_ScrollPos += ScrollInfo.nPage;
			break;

		case SB_THUMBTRACK:
			m_ScrollPos = ScrollInfo.nTrackPos;
			break;
	}

	int MaxScrollPos = std::max(0, ScrollInfo.nMax - (int)ScrollInfo.nPage + 1);

	m_ScrollPos = std::max(0, std::min(m_ScrollPos, MaxScrollPos));

	SetScrollPos(m_hwnd, SB_VERT, m_ScrollPos, TRUE);

	InvalidateRect(m_hwnd, nullptr, FALSE);
}

/****************************************************************************/

void LogView::OnMouseWheel(int Delta)
{
	m_ScrollPos -= Delta / WHEEL_DELTA * 3;

	RECT rc;
	GetClientRect(m_hwnd, &rc);

	int LinesVisible = rc.bottom / m_LineHeight;
	int MaxScrollPos = std::max(0, (int)m_pLogBuffer->size() - LinesVisible);

	m_ScrollPos = std::max(0, std::min(m_ScrollPos, MaxScrollPos));

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, FALSE);
}

/****************************************************************************/

void LogView::OnLButtonDown(int YPos)
{
	int Line = GetLineAtY(YPos);

	m_SelectionStart = Line;
	m_SelectionEnd = Line;
	m_Dragging = true;

	SetCapture(m_hwnd);

	InvalidateRect(m_hwnd, nullptr, FALSE);
}

/****************************************************************************/

void LogView::OnMouseMove(int YPos)
{
	if (m_Dragging)
	{
		int Line = GetLineAtY(YPos);

		if (Line != m_SelectionEnd)
		{
			m_SelectionEnd = Line;

			InvalidateRect(m_hwnd, nullptr, FALSE);
		}
	}
}

/****************************************************************************/

void LogView::OnLButtonUp(int YPos)
{
	if (m_Dragging)
	{
		m_SelectionEnd = GetLineAtY(YPos);

		m_Dragging = false;

		ReleaseCapture();

		InvalidateRect(m_hwnd, nullptr, FALSE);

		// If a single message is selected, show that message's
		// data bytes.
		if (m_SelectionStart == m_SelectionEnd)
		{
			const EconetLogMessage* pMessage = nullptr;

			if (m_SelectionStart >= 0 && m_SelectionStart < m_pLogBuffer->size())
			{
				pMessage = &(*m_pLogBuffer)[m_SelectionStart];
			}

			SendMessage(m_hwndParent, WM_ECONET_LOG_SELECT_MESSAGE, 0, (LPARAM)pMessage);
		}
	}
}

/****************************************************************************/

void LogView::UpdateScrollBar()
{
	RECT rc;
	GetClientRect(m_hwnd, &rc);

	int LinesVisible = std::max(1, (int)(rc.bottom / m_LineHeight));
	bool CanScroll = m_pLogBuffer->size() > (size_t)LinesVisible;

	SCROLLINFO ScrollInfo;
	ZeroMemory(&ScrollInfo, sizeof(ScrollInfo));

	ScrollInfo.cbSize = sizeof(ScrollInfo);
	ScrollInfo.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;

	ScrollInfo.nMin = 0;
	ScrollInfo.nMax = std::max(0, (int)m_pLogBuffer->size() - 1);
	ScrollInfo.nPage = LinesVisible;
	ScrollInfo.nPos = m_ScrollPos;

	SetScrollInfo(m_hwnd, SB_VERT, &ScrollInfo, TRUE);

	EnableScrollBar(m_hwnd, SB_VERT, CanScroll ? ESB_ENABLE_BOTH : ESB_DISABLE_BOTH);
}

/****************************************************************************/

int LogView::GetLineAtY(int y) const
{
	return m_ScrollPos + (y / m_LineHeight);
}

/****************************************************************************/
