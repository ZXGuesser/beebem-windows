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

#ifndef PROPERTY_SHEET_PAGE_HEADER
#define PROPERTY_SHEET_PAGE_HEADER

#include "Window.h"

class PropertySheetPage : public Window
{
	public:
		PropertySheetPage(HINSTANCE hInstance,
		                  int DialogID);

	public:
		const PROPSHEETPAGE* GetPropSheetPage() const;
		bool Apply() const;

	private:
		static INT_PTR CALLBACK DlgProcCallback(HWND hwnd,
		                                        UINT nMessage,
		                                        WPARAM wParam,
		                                        LPARAM lParam);

		INT_PTR DlgProc(UINT nMessage,
		                WPARAM wParam,
		                LPARAM lParam);

		virtual INT_PTR HandleMessage(UINT nMessage,
		                              WPARAM wParam,
		                              LPARAM lParam);

	private:
		virtual void OnInitDialog();
		virtual bool OnSetActive();
		virtual bool OnKillActive();
		virtual bool OnApply();

	private:
		PROPSHEETPAGE m_Page;
		bool m_Apply;
};

#endif
