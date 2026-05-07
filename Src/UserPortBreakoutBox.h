/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 1994  David Alan Gilbert
Copyright (C) 1997  Mike Wyatt
Copyright (C) 2004  Ken Lowe

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

#ifndef USER_PORT_BREAKOUT_BOX_HEADER
#define USER_PORT_BREAKOUT_BOX_HEADER

/* User Port Breakout Box */

#include "Dialog.h"

class UserPortBreakoutDialog : public Dialog
{
	public:
		UserPortBreakoutDialog(HINSTANCE hInstance,
		                       HWND hwndParent);

	public:
		bool KeyDown(int Key);
		bool KeyUp(int Key);

		void ShowBitKey(int key, int ctrlID);
		void ShowInputs(unsigned char data);
		void ShowOutputs(unsigned char data);

	private:
		virtual INT_PTR DlgProc(UINT nMessage,
		                        WPARAM wParam,
		                        LPARAM lParam);

		void PromptForBitKeyInput(int bitKey);
		bool GetValue(int ctrlID);
		void SetValue(int ctrlID, bool State);

	private:
		int m_BitKey;
		unsigned char m_LastInputData;
		unsigned char m_LastOutputData;
};

extern int BitKeys[8];

extern UserPortBreakoutDialog* g_pUserPortBreakoutDialog;

#endif
