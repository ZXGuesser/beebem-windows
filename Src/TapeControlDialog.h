/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2001  Richard Gellman
Copyright (C) 2004  Mike Wyatt

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

#ifndef TAPE_CONTROL_DIALOG_HEADER
#define TAPE_CONTROL_DIALOG_HEADER

#include <vector>

#include "Dialog.h"
#include "TapeMap.h"

class TapeControlDialog : public Dialog
{
	public:
		TapeControlDialog(HINSTANCE hInstance,
		                  HWND hwndParent);

	public:
		void UpdateCounter(int Time);
		void SetUnlock(bool Unlock);
		void SetFileName(const char *FileName);
		void CloseTape();
		void AddMapLines();

	private:
		virtual INT_PTR DlgProc(UINT nMessage,
		                        WPARAM wParam,
		                        LPARAM lParam);

		void NewTape();
		void EjectTape();
		void UpdateState();

	private:
		HWND m_hwndMap;
};

void TapeControlUpdateCounter(int Time);

extern std::vector<TapeMapEntry> TapeMap;
extern TapeControlDialog* g_pTapeControlDialog;

#endif
