/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2021  Chris Needham

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

#include <stdarg.h>
#include <stdio.h>

#include "DebugTrace.h"

#if !defined(NDEBUG)

void DebugTrace(const char *format, ...)
{
	va_list args;
	va_start(args, format);

	// Calculate required length, +1 is for NUL terminator
	const int length = _vscprintf(format, args) + 1;

	char *buffer = (char*)malloc(length);

	if (buffer != nullptr)
	{
		vsprintf(buffer, format, args);
		OutputDebugString(buffer);
		free(buffer);
	}

	va_end(args);
}

void DebugDumpBytes(const char* pszMessage, const unsigned char* pData, int Length)
{
	const int BytesPerLine = 16;

	bool Pad  = Length > BytesPerLine;

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

		if (Pad)
		{
			for (; i < BytesPerLine; i++)
			{
				str += "   ";
			}
		}

		str += "| ";

		for (i = 0; i < BytesPerLine && i < Length; i++)
		{
			str += isprint(pData[Offset + i]) ? pData[Offset + i] : '.';
		}

		DebugTrace("%s %s\n", pszMessage, str.c_str());

		str.clear();

		Length -= BytesPerLine;
		Offset += BytesPerLine;
	}
}

#endif
