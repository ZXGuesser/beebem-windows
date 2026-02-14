/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2004  Mike Wyatt
Copyright (C) 2004  Rob O'Donnell
Copyright (C) 2009  Steve Pick

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

//
// BeebEm debugger
//

#ifndef DEBUG_HEADER
#define DEBUG_HEADER

#include <windows.h>
#include <stdarg.h>
#include <string>

#include "Via.h"

enum class DebugType
{
	None,
	Video,
	SysVIA,
	UserVIA,
	Tube,
	Serial,
	Econet,
	RemoteServer,
	Teletext,
	CMOS,
	BRK,
	Manual,
	Breakpoint,
	Last
};

//*******************************************************************
// Data structs

struct Label
{
	std::string name;
	int addr;

	Label() : addr(0)
	{
	}

	Label(const std::string& n, int a) : name(n), addr(a)
	{
	}
};

struct Breakpoint
{
	int start;
	int end;
	std::string name;
};

struct Watch
{
	int start;
	char type;
	int value;
	bool host;
	std::string name;
};

struct InstInfo
{
	const char* opcode;
	int bytes;
	int flag;
};

struct AddrInfo
{
	int start;
	int end;
	std::string desc;
};

struct DebugCmd
{
	const char *name;
	bool (*handler)(const char* arguments);
	const char *argdesc;
	const char *help;
};

bool DebugDisassembler(int Addr,
                       int PrevAddr,
                       int Accumulator,
                       int XReg,
                       int YReg,
                       unsigned char PSR,
                       unsigned char StackReg,
                       bool Host);
int DebugDisassembleInstruction(int Addr, bool Host, char *pszOutput);
int DebugDisassembleInstructionWithCPUStatus(int Addr,
                                             bool Host,
                                             int Accumulator,
                                             int XReg,
                                             int YReg,
                                             unsigned char StackReg,
                                             unsigned char PSR,
                                             char *pszOutput);

void DebugOpenDialog(HINSTANCE hinst, HWND hwndMain);
void DebugCloseDialog();

void DebugDisplayTrace(DebugType type, bool host, const char *info);
void DebugDisplayTraceF(DebugType type, bool host, const char *format, ...);
void DebugDisplayTraceV(DebugType type, bool host, const char *format, va_list args);

void DebugDisplayInfo(const char *info);
void DebugDisplayInfoF(const char *format, ...);

void DebugRunScript(const char *filename);
bool DebugLoadLabels(const char *filename, bool host);

unsigned char DebugReadMem(int addr, bool host);
void DebugBreakExecution(DebugType type);

void DebugInitMemoryMaps();
bool DebugLoadMemoryMap(const char* filename, int bank);

extern bool DebugEnabled;
extern HWND hwndDebug;

#endif
