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

#pragma warning(push)
#pragma warning(disable: 4091) // ignored on left of 'tagGPFIDL_FLAGS' when no variable is declared
#include <shlobj.h>
#pragma warning(pop)

#include "JoystickDialog.h"
#include "JoystickController.h"
#include "ListView.h"
#include "Resource.h"

/****************************************************************************/

JoystickDialog::JoystickDialog(HINSTANCE hInstance,
                               HWND hwndParent,
                               JoystickController& Controller,
                               JoystickOption Option) :
	Dialog(hInstance, hwndParent, IDD_JOYSTICK),
	m_JoystickController(Controller),
	m_JoystickOption(Option),
	m_DeviceIndex(0)
{
}

/****************************************************************************/

INT_PTR JoystickDialog::DlgProc(UINT   nMessage,
                                WPARAM wParam,
                                LPARAM /* lParam */)
{
	switch (nMessage)
	{
		case WM_INITDIALOG: {
			m_JoystickListView.Init(m_hwnd, IDC_JOYSTICK_LIST);

			m_JoystickListView.SetExtendedStyle(LVS_EX_FULLROWSELECT);

			m_JoystickListView.InsertColumn(0, "Option", LVCFMT_LEFT, 50);

			UpdateJoystickList();

			m_DeviceIndex = m_JoystickController.GetActiveDevice();

			LPARAM ItemToSelect = MAKELPARAM(m_DeviceIndex, (int)m_JoystickOption);

			int Index = m_JoystickListView.FindItemData(ItemToSelect);

			if (Index == -1)
			{
				Index = 0;
			}

			m_JoystickListView.SelectItem(Index);
			m_JoystickListView.SetFocus();

			return TRUE;
		}

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case IDOK:
					UpdateSelected();
					EndDialog(m_hwnd, wParam);
					return TRUE;

				case IDCANCEL:
					EndDialog(m_hwnd, wParam);
					return TRUE;

				case IDC_REFRESH:
					UpdateJoystickList();
					return TRUE;
			}
			break;
	}

	return FALSE;
}

/****************************************************************************/

void JoystickDialog::UpdateJoystickList()
{
	m_JoystickController.EnumerateDevices();

	m_JoystickListView.DeleteAllItems();

	int Row = 0;

	// List is sorted so store catalogue index in list's item data
	m_JoystickListView.InsertItem(Row++,
	                              0,
	                              "None",
	                              MAKELPARAM(0, (int)JoystickOption::Disabled));

	size_t Count = m_JoystickController.GetDeviceCount();

	for (size_t i = 0; i < Count; i++)
	{
		const JoystickDeviceInfo& Info = m_JoystickController.GetDeviceInfo(i);

		m_JoystickListView.InsertItem(Row++,
		                              0,
		                              Info.Name.c_str(),
		                              MAKELPARAM(i, (int)JoystickOption::Joystick));
	}

	m_JoystickListView.InsertItem(Row++,
	                              0,
	                              "Analogue Mouststick",
	                              MAKELPARAM(0, (int)JoystickOption::AnalogueMouseStick));

	m_JoystickListView.InsertItem(Row++,
	                              0,
	                              "Digital Mouststick",
	                              MAKELPARAM(0, (int)JoystickOption::DigitalMouseStick));

	m_JoystickListView.SetColumnWidth(0, LVSCW_AUTOSIZE_USEHEADER);
}

/****************************************************************************/

void JoystickDialog::UpdateSelected()
{
	int Index = m_JoystickListView.GetNextItem(-1, LVNI_SELECTED);

	if (Index != -1)
	{
		LPARAM ItemData = m_JoystickListView.GetItemData(Index);

		m_JoystickOption = static_cast<JoystickOption>(HIWORD(ItemData));

		if (m_JoystickOption == JoystickOption::Joystick)
		{
			m_DeviceIndex = LOWORD(ItemData);
		}
	}
}

/****************************************************************************/

JoystickOption JoystickDialog::GetJoystickOption() const
{
	return m_JoystickOption;
}

/****************************************************************************/

size_t JoystickDialog::GetDeviceIndex() const
{
	return m_DeviceIndex;
}

/****************************************************************************/
