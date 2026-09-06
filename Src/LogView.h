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

#ifndef LOG_VIEW_HEADER
#define LOG_VIEW_HEADER

#include <deque>

class EconetLogMessage;
class EconetLogBuffer;

class LogView
{
	public:
		LogView(EconetLogBuffer& LogBuffer,
		        bool ShowBroadcasts);

		static bool InitClass(HINSTANCE hInstance);

		bool Create(HINSTANCE hInstance,
		            HWND hwndParent,
		            int id,
		            const RECT& rc);

		void AppendLog(bool BufferFull);
		void Clear();
		void CopyToClipboard();
		void SelectUp();
		void SelectDown();
		void SelectAll();
		void ShowBroadcasts(bool bShow);

	private:
		static LRESULT CALLBACK WndProcCallback(HWND hwnd,
		                                        UINT nMessage,
		                                        WPARAM wParam,
		                                        LPARAM lParam);

		LRESULT WndProc(UINT nMessage,
		                WPARAM wParam,
		                LPARAM lParam);

		void OnNcCreate();
		void OnPaint();
		void OnLButtonUp(int YPos);
		void OnVScroll(int Event);
		void OnMouseWheel(int Delta);

		void UpdateScrollBar();
		void SetScrollPosition(int Index);
		int GetLinesVisible();
		int GetLineAtY(int y) const;
		void SelectMessage(const EconetLogMessage* pMessage);

	private:
		EconetLogBuffer& m_LogBuffer;
		HWND m_hwnd;
		HWND m_hwndParent;
		bool m_ShowBroadcasts;
		HFONT m_hFont;
		int m_LineHeight;
		int m_ScrollPos;
		int m_SelectedIndex;
		bool m_SelectAll;
};

#endif
