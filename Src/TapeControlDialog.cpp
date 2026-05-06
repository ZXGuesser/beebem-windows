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

#include <windows.h>

#include <string>

#include "TapeControlDialog.h"
#include "Main.h"
#include "Resource.h"
#include "Serial.h"
#include "WindowUtils.h"

// Tape control dialog box variables
std::vector<TapeMapEntry> TapeMap;
TapeControlDialog* g_pTapeControlDialog = nullptr;

/****************************************************************************/

TapeControlDialog::TapeControlDialog(HINSTANCE hinst, HWND hwndMain) :
	Dialog(hinst, hwndMain, IDD_TAPECONTROL)
{
}

/****************************************************************************/

void TapeControlDialog::AddMapLines()
{
	SendMessage(m_hwndMap, LB_RESETCONTENT, 0, 0);

	for (const TapeMapEntry& line : TapeMap)
	{
		SendMessage(m_hwndMap, LB_ADDSTRING, 0, (LPARAM)line.desc.c_str());
	}

	UpdateState();
}

/****************************************************************************/

void TapeControlDialog::UpdateCounter(int tape_time)
{
	size_t i = 0;

	while (i < TapeMap.size() && TapeMap[i].time <= tape_time)
	{
		i++;
	}

	if (i > 0)
	{
		i--;
	}

	SendMessage(m_hwndMap, LB_SETCURSEL, (WPARAM)i, 0);
}

/****************************************************************************/

void TapeControlDialog::UpdateState()
{
	SetFocus(m_hwnd);

	SerialTapeState State = SerialGetTapeState();

	UINT nIDCheckButton;

	switch (State)
	{
		case SerialTapeState::Playing:
			nIDCheckButton = IDC_PLAYING;

			EnableDlgItem(IDC_TAPE_CONTROL_PLAY, false);
			EnableDlgItem(IDC_TAPE_CONTROL_STOP, true);
			EnableDlgItem(IDC_TAPE_CONTROL_EJECT, false);
			EnableDlgItem(IDC_TAPE_CONTROL_REWIND, true);
			EnableDlgItem(IDC_TAPE_CONTROL_LOAD_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_NEW_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_RECORD, false);
			break;

		case SerialTapeState::Recording:
			nIDCheckButton = IDC_RECORDING;

			EnableDlgItem(IDC_TAPE_CONTROL_PLAY, false);
			EnableDlgItem(IDC_TAPE_CONTROL_STOP, true);
			EnableDlgItem(IDC_TAPE_CONTROL_EJECT, false);
			EnableDlgItem(IDC_TAPE_CONTROL_REWIND, true);
			EnableDlgItem(IDC_TAPE_CONTROL_LOAD_TAPE, false);
			EnableDlgItem(IDC_TAPE_CONTROL_NEW_TAPE, false);
			EnableDlgItem(IDC_TAPE_CONTROL_RECORD, false);
			break;

		case SerialTapeState::Stopped:
			nIDCheckButton = IDC_STOPPED;

			EnableDlgItem(IDC_TAPE_CONTROL_PLAY, true);
			EnableDlgItem(IDC_TAPE_CONTROL_STOP, false);
			EnableDlgItem(IDC_TAPE_CONTROL_EJECT, true);
			EnableDlgItem(IDC_TAPE_CONTROL_REWIND, true);
			EnableDlgItem(IDC_TAPE_CONTROL_LOAD_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_NEW_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_RECORD, SerialTapeIsUef());
			break;

		case SerialTapeState::NoTape:
		default:
			nIDCheckButton = IDC_STOPPED;

			EnableDlgItem(IDC_TAPE_CONTROL_PLAY, false);
			EnableDlgItem(IDC_TAPE_CONTROL_STOP, false);
			EnableDlgItem(IDC_TAPE_CONTROL_EJECT, false);
			EnableDlgItem(IDC_TAPE_CONTROL_REWIND, false);
			EnableDlgItem(IDC_TAPE_CONTROL_LOAD_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_NEW_TAPE, true);
			EnableDlgItem(IDC_TAPE_CONTROL_RECORD, false);
			break;
	}

	mainWin->EnableSaveState(State != SerialTapeState::Recording);

	CheckRadioButton(m_hwnd,
	                 IDC_PLAYING,
	                 IDC_STOPPED,
	                 nIDCheckButton);
}

/****************************************************************************/

INT_PTR TapeControlDialog::DlgProc(UINT message, WPARAM wParam, LPARAM /* lParam */)
{
	switch (message)
	{
		case WM_INITDIALOG: {
			m_hwndMap = GetDlgItem(m_hwnd, IDC_TAPE_CONTROL_MAP);

			SendMessage(m_hwndMap,
			            WM_SETFONT,
			            (WPARAM)GetStockObject(ANSI_FIXED_FONT),
			            (LPARAM)MAKELPARAM(FALSE, 0));

			SetDlgItemText(IDC_TAPE_FILENAME, TapeFileName);
			UpdateState();

			int Time = SerialGetTapeClock();
			AddMapLines();
			UpdateCounter(Time);

			return TRUE;
		}

		case WM_ACTIVATE:
			if (LOWORD(wParam) == WA_INACTIVE)
			{
				hCurrentDialog = nullptr;
			}
			else
			{
				hCurrentDialog = m_hwnd;
			}
			return FALSE;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case IDC_TAPE_CONTROL_MAP:
					if (HIWORD(wParam) == LBN_SELCHANGE)
					{
						LRESULT s = SendMessage(m_hwndMap, LB_GETCURSEL, 0, 0);

						if (s != LB_ERR && s >= 0 && s < (int)TapeMap.size())
						{
							SetTapePosition(TapeMap[s].time);
						}
					}
					return FALSE;

				case IDC_TAPE_CONTROL_PLAY:
					SerialStopTapeRecording();
					SerialPlayTape();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_STOP:
					if (TapeState.Recording)
					{
						SerialStopTapeRecording();
						SerialUpdateTapeClock();

						if (SerialTapeIsModified())
						{
							UEFResult Result = UEFFile.Save(TapeFileName);

							if (Result != UEFResult::Success)
							{
								mainWin->Report(MessageType::Error,
								                "Failed to write to tape file:\n %s", TapeFileName);
							}

							UEFFile.CreateTapeMap(TapeMap);
							AddMapLines();
						}
					}

					SerialStopTape();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_EJECT:
					EjectTape();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_REWIND:
					RewindTape();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_LOAD_TAPE:
					mainWin->LoadTape();
					return TRUE;

				case IDC_TAPE_CONTROL_NEW_TAPE:
					NewTape();
					SetFileName("(Untitled)");
					UEFFile.CreateTapeMap(TapeMap);
					AddMapLines();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_RECORD:
					SerialRecordTape();
					UpdateState();
					return TRUE;

				case IDC_TAPE_CONTROL_UNLOCK: {
					bool Unlock = IsDlgItemChecked(IDC_TAPE_CONTROL_UNLOCK);
					mainWin->SetUnlockTape(Unlock);
					return TRUE;
				}

				case IDCANCEL:
					Close();
					g_pTapeControlDialog = nullptr;
					return TRUE;
			}
	}

	return FALSE;
}

/****************************************************************************/

void TapeControlDialog::EjectTape()
{
	SerialEjectTape();
	SetFileName("");
}

/****************************************************************************/

void TapeControlDialog::NewTape()
{
	mainWin->NewTape(TapeFileName, sizeof(TapeFileName));
}

/****************************************************************************/

void TapeControlDialog::CloseTape()
{
	SendMessage(m_hwndMap, LB_RESETCONTENT, 0, 0);
	UpdateState();
}

/****************************************************************************/

void TapeControlDialog::SetFileName(const char *FileName)
{
	SetDlgItemText(IDC_TAPE_FILENAME, FileName);
}

/****************************************************************************/

void TapeControlDialog::SetUnlock(bool Unlock)
{
	SetDlgItemChecked(IDC_TAPE_CONTROL_UNLOCK, Unlock);
}

/****************************************************************************/

void TapeControlUpdateCounter(int Time)
{
	if (g_pTapeControlDialog != nullptr)
	{
		g_pTapeControlDialog->UpdateCounter(Time);
	}
}

/****************************************************************************/
