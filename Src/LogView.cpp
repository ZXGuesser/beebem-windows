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
#include "Clipboard.h"
#include "EconetLog.h"
#include "Messages.h"

/****************************************************************************/

LogView::LogView(EconetLogBuffer& LogBuffer, bool ShowBroadcasts) :
	m_LogBuffer(LogBuffer),
	m_hwnd(nullptr),
	m_hwndParent(nullptr),
	m_ShowBroadcasts(ShowBroadcasts),
	m_hFont(nullptr),
	m_LineHeight(0),
	m_ScrollPos(0),
	m_SelectedIndex(-1),
	m_SelectAll(false)
{
	m_LogBuffer.SetFilter(!m_ShowBroadcasts);
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
	if (m_hwnd == nullptr)
	{
		// The log window hasn't been initialised yet.
		return;
	}

	// If the end of the log is being viewed, ensure the latest entry
	// is visible, otherwise keep the existing scroll position.

	SCROLLINFO ScrollInfo;
	ZeroMemory(&ScrollInfo, sizeof(ScrollInfo));

	ScrollInfo.cbSize = sizeof(ScrollInfo);
	ScrollInfo.fMask = SIF_RANGE | SIF_PAGE;

	GetScrollInfo(m_hwnd, SB_VERT, &ScrollInfo);

	int MaxScrollPos = std::max(ScrollInfo.nMin,
	                            ScrollInfo.nMax - (int)ScrollInfo.nPage + 1);

	if (m_ScrollPos >= MaxScrollPos)
	{
		int LinesVisible = GetLinesVisible();

		m_ScrollPos = std::max(0, m_LogBuffer.GetSize() - LinesVisible);
	}

	if (BufferFull)
	{
		// The buffer is full and the oldest entry has been removed.
		// Adjust the selection so that the same line is selected.

		if (m_SelectedIndex != -1)
		{
			m_SelectedIndex--;
		}

		if (m_SelectedIndex == -1)
		{
			SelectMessage(nullptr);
		}
	}

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, FALSE);
}

/****************************************************************************/

void LogView::Clear()
{
	m_LogBuffer.Clear();

	m_ScrollPos = 0;
	m_SelectedIndex = -1;
	m_SelectAll = false;

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, TRUE);

	SelectMessage(nullptr);
}

/****************************************************************************/

void LogView::CopyToClipboard()
{
	if (m_LogBuffer.GetSize() == 0)
	{
		return;
	}

	if (!m_SelectAll && m_SelectedIndex == -1)
	{
		return;
	}

	size_t Size = 0;

	const int StartIndex = m_SelectAll ? 0 : m_SelectedIndex;
	const int EndIndex = m_SelectAll ? m_LogBuffer.GetSize() - 1 : m_SelectedIndex;

	for (int i = StartIndex; i <= EndIndex; i++)
	{
		Size += m_LogBuffer.GetMessage(i)->GetMessageLength() + 2;
	}

	Size += 1; // For the NUL terminator.

	auto CopyData = [=](unsigned char* pBuffer)
	{
		size_t Offset = 0;

		for (int i = StartIndex; i <= EndIndex; i++)
		{
			strcpy((char*)pBuffer + Offset, m_LogBuffer.GetMessage(i)->GetMessageStr());
			Offset += m_LogBuffer.GetMessage(i)->GetMessageLength();

			pBuffer[Offset++] = '\r';
			pBuffer[Offset++] = '\n';
		}

		pBuffer[Offset] = '\0';
	};

	::CopyToClipboard(m_hwnd, Size, CopyData);
}

/****************************************************************************/

void LogView::SelectUp()
{
	if (m_SelectedIndex > 0)
	{
		m_SelectedIndex--;

		SelectMessage(m_LogBuffer.GetMessage(m_SelectedIndex));

		InvalidateRect(m_hwnd, nullptr, TRUE);
	}

	if (m_SelectedIndex >= 0 && m_SelectedIndex < m_ScrollPos)
	{
		SetScrollPosition(m_SelectedIndex);
	}
}

/****************************************************************************/

void LogView::SelectDown()
{
	const int BufferSize = m_LogBuffer.GetSize();

	if (m_SelectedIndex < BufferSize - 1)
	{
		m_SelectedIndex++;

		SelectMessage(m_LogBuffer.GetMessage(m_SelectedIndex));

		InvalidateRect(m_hwnd, nullptr, TRUE);
	}

	const int LinesVisible = GetLinesVisible();

	if (m_SelectedIndex >= m_ScrollPos + LinesVisible)
	{
		SetScrollPosition(m_SelectedIndex - LinesVisible + 1);
	}
}

/****************************************************************************/

void LogView::SelectAll()
{
	m_SelectedIndex = -1;

	const int BufferSize = m_LogBuffer.GetSize();

	m_SelectAll = true;

	if (m_SelectedIndex != -1)
	{
		const EconetLogMessage* pMessage = m_LogBuffer.GetMessage(m_SelectedIndex);

		SelectMessage(pMessage);
	}

	InvalidateRect(m_hwnd, nullptr, TRUE);
}

/****************************************************************************/

void LogView::ShowBroadcasts(bool Show)
{
	m_ShowBroadcasts = Show;
	m_LogBuffer.SetFilter(!m_ShowBroadcasts);

	m_SelectedIndex = -1;
	SelectMessage(nullptr);

	// Show the most recent log message.

	int LinesVisible = GetLinesVisible();

	m_ScrollPos = std::max(0, m_LogBuffer.GetSize() - LinesVisible);

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, TRUE);
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

	int Last = std::min(m_LogBuffer.GetSize(), m_ScrollPos + LinesVisible);

	int y = 0;

	for (int i = m_ScrollPos; i < Last; i++)
	{
		const EconetLogMessage* pMessage = m_LogBuffer.GetMessage(i);

		bool Selected = m_SelectAll || i == m_SelectedIndex;

		if (Selected)
		{
			RECT LineRect = { 0, y, rc.right, y + m_LineHeight };
			FillRect(hdcBuffer, &LineRect, (HBRUSH)(COLOR_HIGHLIGHT + 1));
			SetTextColor(hdcBuffer, GetSysColor(COLOR_HIGHLIGHTTEXT));
		}

		TextOut(hdcBuffer, 4, y, pMessage->GetMessageStr(), (int)pMessage->GetMessageLength());

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

	int LinesVisible = GetLinesVisible();
	int MaxScrollPos = std::max(0, m_LogBuffer.GetSize() - LinesVisible);

	m_ScrollPos = std::max(0, std::min(m_ScrollPos, MaxScrollPos));

	UpdateScrollBar();

	InvalidateRect(m_hwnd, nullptr, FALSE);
}

/****************************************************************************/

void LogView::OnLButtonUp(int YPos)
{
	m_SelectedIndex = GetLineAtY(YPos);
	m_SelectAll = false;

	InvalidateRect(m_hwnd, nullptr, FALSE);

	// Show the selected message's data bytes.

	const EconetLogMessage* pMessage = nullptr;

	if (m_SelectedIndex >= 0 && m_SelectedIndex < m_LogBuffer.GetSize())
	{
		pMessage = m_LogBuffer.GetMessage(m_SelectedIndex);
	}

	SelectMessage(pMessage);
}

/****************************************************************************/

void LogView::UpdateScrollBar()
{
	RECT Rect;
	GetClientRect(m_hwnd, &Rect);

	int Size = m_LogBuffer.GetSize();
	int LinesVisible = GetLinesVisible();

	SCROLLINFO ScrollInfo;
	ZeroMemory(&ScrollInfo, sizeof(ScrollInfo));

	ScrollInfo.cbSize = sizeof(ScrollInfo);
	ScrollInfo.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;

	ScrollInfo.nMin = 0;
	ScrollInfo.nMax = std::max(0, Size - 1);
	ScrollInfo.nPage = std::max(1, LinesVisible);
	ScrollInfo.nPos = m_ScrollPos;

	SetScrollInfo(m_hwnd, SB_VERT, &ScrollInfo, TRUE);

	EnableScrollBar(m_hwnd, SB_VERT, Size > LinesVisible ? ESB_ENABLE_BOTH : ESB_DISABLE_BOTH);
}

/****************************************************************************/

void LogView::SetScrollPosition(int Index)
{
	m_ScrollPos = Index;

	UpdateScrollBar();
}

/****************************************************************************/

int LogView::GetLinesVisible()
{
	RECT Rect;
	GetClientRect(m_hwnd, &Rect);

	return Rect.bottom / m_LineHeight;
}

/****************************************************************************/

int LogView::GetLineAtY(int y) const
{
	return m_ScrollPos + (y / m_LineHeight);
}

/****************************************************************************/

void LogView::SelectMessage(const EconetLogMessage* pMessage)
{
	SendMessage(m_hwndParent, WM_ECONET_LOG_SELECT_MESSAGE, 0, (LPARAM)pMessage);
}

/****************************************************************************/
