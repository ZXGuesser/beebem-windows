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

#ifndef ECONET_DIALOG_HEADER
#define ECONET_DIALOG_HEADER

#include "Econet.h"
#include "ListView.h"
#include "LogView.h"
#include "PropertySheetPage.h"

class EconetNetworkPage : public PropertySheetPage
{
	public:
		EconetNetworkPage(HINSTANCE hInstance,
		                  int DialogID);

	private:
		virtual INT_PTR HandleMessage(UINT nMessage,
		                              WPARAM wParam,
		                              LPARAM lParam);

	private:
		virtual void OnInitDialog();
		virtual bool OnSetActive();
		virtual bool OnKillActive();

		void InitStationsList();
		void InitNetworksList();
		void UpdateNetworkState();
		void UpdateStationsList();
		void UpdateNetworksList();

		void OnTimer();

	private:
		ListView m_StationsListView;
		ListView m_NetworksListView;
};

class EconetSettingsPage : public PropertySheetPage
{
	public:
		EconetSettingsPage(HINSTANCE hInstance,
		                   int DialogID);

	private:
		virtual void OnInitDialog();

		virtual INT_PTR HandleMessage(UINT nMessage,
		                              WPARAM wParam,
		                              LPARAM lParam);

	private:
		ListView m_StationsListView;
		ListView m_NetworksListView;
};

class EconetLogMessage;

class EconetLogPage : public PropertySheetPage
{
	public:
		EconetLogPage(HINSTANCE hInstance,
		              int DialogID,
		              std::deque<EconetLogMessage>* pLogBuffer);

	public:
		void AppendLog(bool BufferFull);

	private:
		virtual void OnInitDialog();

		virtual INT_PTR HandleMessage(UINT nMessage,
		                              WPARAM wParam,
		                              LPARAM lParam);

		BOOL OnCommand(UINT MenuID);
		void OnSelectMessage(const EconetLogMessage* pMessage);

	private:
		LogView m_LogView;
		HFONT m_hFont;
};

class EconetDialog
{
	public:
		EconetDialog(HINSTANCE hInstance,
		             HWND hwndParent,
		             std::deque<EconetLogMessage>* pLogBuffer);

	public:
		bool Open();
		void Close();

		bool IsOpen() const;

		void AppendLog(bool BufferFull);

		bool HandleMessage(MSG* pMsg);

	private:
		static int CALLBACK PropSheetCallback(HWND hwnd,
		                                      UINT nMessage,
		                                      LPARAM lParam);

	private:
		HINSTANCE m_hInstance;
		HWND m_hwndParent;
		HWND m_hwnd;
		HACCEL m_hAccelerators;
		EconetNetworkPage m_EconetNetworkPage;
		EconetSettingsPage m_EconetSettingsPage;
		EconetLogPage m_EconetLogPage;
};

extern EconetDialog* g_pEconetDialog;

#endif
