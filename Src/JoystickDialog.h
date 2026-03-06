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

#ifndef JOYSTICKDIALOG_HEADER
#define JOYSTICKDIALOG_HEADER

#include "BeebWin.h"
#include "Dialog.h"
#include "ListView.h"

class JoystickController;

class JoystickDialog : public Dialog
{
	public:
		JoystickDialog(HINSTANCE hInstance,
		               HWND hwndParent,
		               JoystickController& Controller,
		               JoystickOption Option);

	public:
		JoystickOption GetJoystickOption() const;
		size_t GetDeviceIndex() const;

	private:
		virtual INT_PTR DlgProc(UINT   nMessage,
		                        WPARAM wParam,
		                        LPARAM lParam);

		void UpdateJoystickList();
		void UpdateSelected();

	private:
		JoystickController& m_JoystickController;
		JoystickOption m_JoystickOption;
		size_t m_DeviceIndex;
		ListView m_JoystickListView;
};

#endif
