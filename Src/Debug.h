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

#include <stdarg.h>

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
	StepOver,
	RunTo,
	Last
};

bool DebugDisassembler(int Addr,
                       int PrevAddr,
                       int Accumulator,
                       int XReg,
                       int YReg,
                       unsigned char PSR,
                       unsigned char StackReg,
                       bool Host);
int DebugDisassembleInstruction(int Addr, bool Host, char* pszOutput);
int DebugDisassembleInstructionWithCPUStatus(int Addr,
                                             bool Host,
                                             int Accumulator,
                                             int XReg,
                                             int YReg,
                                             unsigned char StackReg,
                                             unsigned char PSR,
                                             char* pszOutput);

void DebugOpenDialog(HINSTANCE hInstance, HWND hwndParent);
void DebugCloseDialog();

void DebugDisplayTrace(DebugType Type, bool Host, const char* Info);
void DebugDisplayTraceF(DebugType Type, bool Host, const char* Format, ...);
void DebugDisplayTraceV(DebugType Type, bool Host, const char* Format, va_list Args);

void DebugDisplayInfo(const char* Info);
void DebugDisplayInfoF(const char* Format, ...);

void DebugRunScript(const char* Filename);
bool DebugLoadLabels(const char* Filename, bool Host);

unsigned char DebugReadMem(int Addr, bool Host);
void DebugBreakExecution(DebugType Type);

bool DebugIsHalted();
bool DebugIsStepping();
bool DebugParasiteIsStepping();

void DebugInitMemoryMaps();
bool DebugLoadMemoryMap(const char* Filename, int Bank);

extern bool DebugEnabled;
extern bool DebugHost;

extern HWND hwndDebug;

#endif
