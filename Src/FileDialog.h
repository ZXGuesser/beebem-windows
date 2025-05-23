/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 1994  Nigel Magnay
Copyright (C) 1997  Mike Wyatt
Copyright (C) 1998  Robert Schmidt
Copyright (C) 2001  Richard Gellman
Copyright (C) 2004  Ken Lowe
Copyright (C) 2004  Rob O'Donnell
Copyright (C) 2005  Jon Welch

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

#ifndef FILEDIALOG_HEADER
#define FILEDIALOG_HEADER

class FileDialog
{
	public:
		FileDialog(HWND hwndOwner, LPTSTR Result, DWORD ResultLength,
		           LPCTSTR InitialFolder, LPCTSTR Filter);
		FileDialog(const FileDialog&) = delete;
		FileDialog& operator=(FileDialog&) = delete;

	public:
		// Prepare dialog
		void SetFilterIndex(DWORD Index);
		void AllowMultiSelect();
		void NoOverwritePrompt();
		void SetTitle(LPCTSTR Title);

		// Show dialog
		bool Open();
		bool Save();

		// Get results
		DWORD GetFilterIndex() const;

	private:
		OPENFILENAME m_ofn;

		bool ShowDialog(bool Open);
};

#endif
