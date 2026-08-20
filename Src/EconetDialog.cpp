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

#pragma warning(push)
#pragma warning(disable: 4091) // ignored on left of 'tagGPFIDL_FLAGS' when no variable is declared
#include <shlobj.h>
#pragma warning(pop)

#include "EconetDialog.h"
#include "BeebWin.h"
#include "Econet.h"
#include "Main.h"
#include "Messages.h"
#include "Resource.h"
#include "Socket.h"
#include "WindowUtils.h"

/****************************************************************************/

EconetDialog* g_pEconetDialog = nullptr;

/****************************************************************************/

EconetDialog::EconetDialog(HINSTANCE hInstance, HWND hwndParent,
                           std::deque<EconetLogMessage>* pLogBuffer) :
	m_hInstance(hInstance),
	m_hwndParent(hwndParent),
	m_hwnd(nullptr),
	m_hAccelerators(nullptr),
	m_EconetNetworkPage(hInstance, IDD_ECONET_NETWORK),
	m_EconetSettingsPage(hInstance, IDD_ECONET_SETTINGS),
	m_EconetLogPage(hInstance, IDD_ECONET_LOG, pLogBuffer)
{
}

/****************************************************************************/

bool EconetDialog::Open()
{
	m_hAccelerators = LoadAccelerators(m_hInstance, MAKEINTRESOURCE(IDR_ECONET));

	PROPSHEETPAGE Pages[3];
	ZeroMemory(Pages, sizeof(Pages));

	Pages[0] = *m_EconetNetworkPage.GetPropSheetPage();
	Pages[1] = *m_EconetSettingsPage.GetPropSheetPage();
	Pages[2] = *m_EconetLogPage.GetPropSheetPage();

	PROPSHEETHEADER Header;
	ZeroMemory(&Header, sizeof(Header));

	Header.dwSize      = sizeof(PROPSHEETHEADER);
	Header.dwFlags     = PSH_MODELESS | PSH_PROPSHEETPAGE | PSH_NOAPPLYNOW |
	                     PSH_NOCONTEXTHELP | PSH_USECALLBACK;
	Header.hwndParent  = m_hwndParent;
	Header.hInstance   = m_hInstance;
	Header.pszCaption  = "Econet";
	Header.nPages      = 3;
	Header.nStartPage  = 0;
	Header.ppsp        = Pages;
	Header.pfnCallback = PropSheetCallback;

	m_hwnd = reinterpret_cast<HWND>(PropertySheet(&Header));

	if (m_hwnd == nullptr)
	{
		return false;
	}

	CentreWindow(m_hwndParent, m_hwnd);

	return true;
}

/****************************************************************************/

void EconetDialog::Close()
{
	if (m_hwnd != nullptr)
	{
		DestroyWindow(m_hwnd);
		m_hwnd = nullptr;
	}
}

/****************************************************************************/

bool EconetDialog::IsOpen() const
{
	return m_hwnd != nullptr;
}

/****************************************************************************/

void EconetDialog::AppendLog(bool BufferFull)
{
	m_EconetLogPage.AppendLog(BufferFull);
}

/****************************************************************************/

bool EconetDialog::HandleMessage(MSG* pMsg)
{
	BOOL bHandled = FALSE;

	HWND hwndFocus = GetFocus();

	if (hwndFocus == m_hwnd || IsChild(m_hwnd, hwndFocus))
	{
		bHandled = TranslateAccelerator(m_hwnd, m_hAccelerators, pMsg);
	}

	if (!bHandled)
	{
		bHandled = PropSheet_IsDialogMessage(m_hwnd, pMsg) != 0;
	}

	// PropSheet_GetCurrentPageHwnd() returns NULL after OK or Cancel has
	// notified all pages.

	if (PropSheet_GetCurrentPageHwnd(m_hwnd) == nullptr)
	{
		Close();
	}

	return !!bHandled;
}

/****************************************************************************/

int CALLBACK EconetDialog::PropSheetCallback(HWND hwnd,
                                             UINT nMessage,
                                             LPARAM /* lParam */)
{
	switch (nMessage)
	{
		case PSCB_INITIALIZED:
			DisableRoundedCorners(hwnd);
			break;

		default:
			break;
	}

	return 0;
}

/****************************************************************************/

EconetNetworkPage::EconetNetworkPage(HINSTANCE hInstance,
                                     int DialogID) :
	PropertySheetPage(hInstance, DialogID)
{
}

/****************************************************************************/

INT_PTR EconetNetworkPage::HandleMessage(UINT nMessage,
                                         WPARAM /* wParam */,
                                         LPARAM /* lParam */)
{
	switch (nMessage)
	{
		case WM_TIMER:
			OnTimer();
			return 0;

		default:
			break;
	}

	return 0;
}

/****************************************************************************/

static const char* const StationColumns[] =
{
	"Station",
	"IP Address",
	"Port"
};

static const char* const NetworkColumns[] =
{
	"Network",
	"IP Address",
	"Port"
};

/****************************************************************************/

void EconetNetworkPage::OnInitDialog()
{
	InitStationsList();
	InitNetworksList();
}

/****************************************************************************/

bool EconetNetworkPage::OnSetActive()
{
	UpdateNetworkState();

	// Refresh every 5 seconds while this page is active.
	SetTimer(m_hwnd, 1, 5000, nullptr);

	return true; // Allow activation.
}

/****************************************************************************/

bool EconetNetworkPage::OnKillActive()
{
	KillTimer(m_hwnd, 1);

	return true; // Allow deactivation.
}

/****************************************************************************/

void EconetNetworkPage::InitStationsList()
{
	m_StationsListView.Init(m_hwnd, IDC_STATIONS_LIST);

	for (int i = 0; i < (int)_countof(StationColumns); ++i)
	{
		m_StationsListView.InsertColumn(i, StationColumns[i], LVCFMT_LEFT, 50);
	}

	m_StationsListView.SetExtendedStyle(LVS_EX_FULLROWSELECT);

	for (int i = 0; i < (int)_countof(StationColumns); ++i)
	{
		m_StationsListView.SetColumnWidth(i, LVSCW_AUTOSIZE_USEHEADER);
	}
}

/****************************************************************************/

void EconetNetworkPage::InitNetworksList()
{
	m_NetworksListView.Init(m_hwnd, IDC_NETWORKS_LIST);

	for (int i = 0; i < (int)_countof(NetworkColumns); ++i)
	{
		m_NetworksListView.InsertColumn(i, NetworkColumns[i], LVCFMT_LEFT, 50);
	}

	m_NetworksListView.SetExtendedStyle(LVS_EX_FULLROWSELECT);

	for (int i = 0; i < (int)_countof(NetworkColumns); ++i)
	{
		m_NetworksListView.SetColumnWidth(i, LVSCW_AUTOSIZE_USEHEADER);
	}
}

/****************************************************************************/

void EconetNetworkPage::UpdateNetworkState()
{
	m_StationsListView.DeleteAllItems();
	m_NetworksListView.DeleteAllItems();

	UpdateStationsList();
	UpdateNetworksList();

	const EconetGateway* pGateway = GetEconetGateway();

	if (pGateway->IPAddress != 0 && pGateway->Port != 0)
	{
		char sz[100];

		std::string str = IPAddressStr(pGateway->IPAddress);

		sprintf(sz, "%s:%u", str.c_str(), pGateway->Port);

		SetDlgItemText(IDC_GATEWAY, sz);
	}
	else
	{
		SetDlgItemText(IDC_GATEWAY, "Not found");
	}
}

/****************************************************************************/

void EconetNetworkPage::UpdateStationsList()
{
	int Count = GetEconetHostCount();

	for (int i = 0; i < Count; i++)
	{
		const EconetHost* pEconetHost = GetEconetHost(i);

		char szHost[100];
		sprintf(szHost, "%d.%d", pEconetHost->Network, pEconetHost->Station);

		if (pEconetHost->Network == EconetNetworkID &&
		    pEconetHost->Station == EconetStationID)
		{
			strcat(szHost, " *");
		}

		std::string str;
		IPAddressToString(AF_INET, &pEconetHost->IPAddress, str);

		char szPort[100];
		sprintf(szPort, "%u", pEconetHost->Port);

		m_StationsListView.InsertItem(i, 0, szHost, 0);
		m_StationsListView.SetItemText(i, 1, str.c_str());
		m_StationsListView.SetItemText(i, 2, szPort);
	}

	m_StationsListView.SetColumnWidth(0, LVSCW_AUTOSIZE_USEHEADER);
	m_StationsListView.SetColumnWidth(1, LVSCW_AUTOSIZE_USEHEADER);
	m_StationsListView.SetColumnWidth(2, LVSCW_AUTOSIZE_USEHEADER);
}

/****************************************************************************/

void EconetNetworkPage::UpdateNetworksList()
{
	int Count = GetEconetNetworkCount();

	for (int i = 0; i < Count; i++)
	{
		const EconetNet* pEconetNet = GetEconetNetwork(i);

		char szNetwork[100];
		sprintf(szNetwork, "%d", pEconetNet->Network);

		std::string str;
		IPAddressToString(AF_INET, &pEconetNet->IPAddress, str);

		char szPort[100];
		sprintf(szPort, "%u", pEconetNet->Port);

		m_NetworksListView.InsertItem(i, 0, szNetwork, 0);
		m_NetworksListView.SetItemText(i, 1, str.c_str());
		m_NetworksListView.SetItemText(i, 2, szPort);
	}

	m_NetworksListView.SetColumnWidth(0, LVSCW_AUTOSIZE_USEHEADER);
	m_NetworksListView.SetColumnWidth(1, LVSCW_AUTOSIZE_USEHEADER);
	m_NetworksListView.SetColumnWidth(2, LVSCW_AUTOSIZE_USEHEADER);
}

/****************************************************************************/

void EconetNetworkPage::OnTimer()
{
	UpdateNetworkState();
}

/****************************************************************************/

EconetSettingsPage::EconetSettingsPage(HINSTANCE hInstance,
                                       int DialogID) :
	PropertySheetPage(hInstance, DialogID)
{
}

/****************************************************************************/

void EconetSettingsPage::OnInitDialog()
{
	SetDlgItemChecked(IDC_AUTO_CONFIGURE, EconetConfig.AutoConfigure);
	SetDlgItemChecked(IDC_MASSAGE_NETWORKS, EconetConfig.MassageNetworks);
	SetDlgItemChecked(IDC_FIND_GATEWAYS, EconetConfig.FindGateways);

	char sz[100];

	sprintf(sz, "%d", EconetConfig.FlagFillTimeout);
	SetDlgItemText(IDC_FLAG_FILL_TIMEOUT_EDIT, sz);

	sprintf(sz, "%d", EconetConfig.ScoutAckTimeout);
	SetDlgItemText(IDC_SCOUT_ACK_TIMEOUT_EDIT, sz);

	sprintf(sz, "%d", EconetConfig.TimeBetweenBytes);
	SetDlgItemText(IDC_TIME_BETWEEN_BYTES_EDIT, sz);

	sprintf(sz, "%d", EconetConfig.FourWayStageTimeout);
	SetDlgItemText(IDC_FOUR_WAY_TIMEOUT_EDIT, sz);

	std::string str = IPAddressStr(EconetConfig.GatewayIPAddress);
	SetDlgItemText(IDC_GATEWAY_IP_ADDRESS, str.c_str());

	sprintf(sz, "%u", EconetConfig.GatewayPort);
	SetDlgItemText(IDC_GATEWAY_PORT, sz);

	sprintf(sz, "%u", PreferredNetworkID);
	SetDlgItemText(IDC_DEFAULT_NETWORK_ID, sz);
}

/****************************************************************************/

INT_PTR EconetSettingsPage::HandleMessage(UINT nMessage,
                                          WPARAM /* wParam */,
                                          LPARAM /* lParam */)
{
	switch (nMessage)
	{
		case WM_NOTIFY:
			break;
	}

	return 0;
}

/****************************************************************************/

EconetLogPage::EconetLogPage(HINSTANCE hInstance,
                             int DialogID,
                             std::deque<EconetLogMessage>* pLogBuffer) :
	PropertySheetPage(hInstance, DialogID),
	m_LogView(pLogBuffer)
{
}

/****************************************************************************/

void EconetLogPage::OnInitDialog()
{
	HWND hwndPlaceholder = GetDlgItem(IDC_LOG);

	RECT rc;
	GetWindowRect(hwndPlaceholder, &rc);

	MapWindowPoints(nullptr,
	                m_hwnd,
	                reinterpret_cast<POINT*>(&rc),
	                2);

	DestroyWindow(hwndPlaceholder);

	m_LogView.Create(hInst, m_hwnd, IDC_LOG, rc);

	SendDlgItemMessage(IDC_DETAIL,
	                   WM_SETFONT,
	                   (WPARAM)GetStockObject(ANSI_FIXED_FONT),
	                   MAKELPARAM(FALSE, 0));
}

/****************************************************************************/

void EconetLogPage::AppendLog(bool BufferFull)
{
	m_LogView.AppendLog(BufferFull);
}

/****************************************************************************/

INT_PTR EconetLogPage::HandleMessage(UINT nMessage,
                                     WPARAM wParam,
                                     LPARAM lParam)
{
	switch (nMessage)
	{
		case WM_COMMAND:
			return OnCommand(LOWORD(wParam));

		case WM_ECONET_LOG_SELECT_MESSAGE:
			OnSelectMessage((const EconetLogMessage*)lParam);
			break;
	}

	return 0;
}

/****************************************************************************/

BOOL EconetLogPage::OnCommand(UINT MenuID)
{
	switch (MenuID)
	{
		case IDC_CLEAR:
			m_LogView.Clear();
			return TRUE;

		case IDM_COPY:
			m_LogView.CopyToClipboard();
			return TRUE;

		case IDM_SELECT_ALL:
			m_LogView.SelectAll();
			return TRUE;

		default:
			break;
	}

	return FALSE;
}

/****************************************************************************/

void EconetLogPage::OnSelectMessage(const EconetLogMessage* pMessage)
{
	SendDlgItemMessage(IDC_DETAIL, LB_RESETCONTENT, 0, 0);

	if (pMessage == nullptr)
	{
		return;
	}

	const unsigned char* pData = pMessage->GetData();

	if (pData == nullptr)
	{
		return;
	}

	int Length = pMessage->GetDataLength();

	const int BytesPerLine = 16;

	int Offset = 0;

	std::string str;

	while (Length > 0)
	{
		int i;

		for (i = 0; i < BytesPerLine && i < Length; i++)
		{
			char sz[5];
			sprintf(sz, "%02X ", pData[Offset + i]);

			str += sz;
		}

		for (; i < BytesPerLine; i++)
		{
			str += "   ";
		}

		str += "| ";

		for (i = 0; i < BytesPerLine && i < Length; i++)
		{
			str += isprint(pData[Offset + i]) ? pData[Offset + i] : '.';
		}

		SendDlgItemMessage(IDC_DETAIL, LB_ADDSTRING, 0, (LPARAM)str.c_str());

		str.clear();

		Length -= BytesPerLine;
		Offset += BytesPerLine;
	}
}

/****************************************************************************/
