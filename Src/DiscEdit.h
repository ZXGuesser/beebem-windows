/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2009  Mike Wyatt

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
// BeebEm Disc Edit - file import/export
//
// Mike Wyatt - Mar 2009
//

#ifndef DISCEDIT_HEADER
#define DISCEDIT_HEADER

#include <string>

constexpr int DFS_MAX_CAT_SIZE       = 62; // Allow for Watford DFS
constexpr int DFS_MAX_TITLE_LEN      = 12;
constexpr int DFS_MAX_NAME_LEN       = 7;
constexpr int DFS_SECTORS_PER_TRACK  = 10;
constexpr int DFS_SECTOR_SIZE        = 256;

struct DfsFileAttributes
{
	char FileName[DFS_MAX_NAME_LEN + 1];
	char Directory;
	bool Locked;
	int LoadAddr;
	int ExecAddr;
	int Length;
	int StartSector;
};

struct DfsDiscCatalogue
{
	char Title[DFS_MAX_TITLE_LEN + 1];
	int NumWrites;
	int NumFiles;
	int NumSectors;
	int BootOpts;
	bool Watford62;
	DfsFileAttributes FileAttrs[DFS_MAX_CAT_SIZE];
};

std::string DfsToLocalFileName(const std::string& DfsFileName);

bool DfsGetCatalogue(const char* pszDiscFile,
                     int NumSides,
                     int Side,
                     DfsDiscCatalogue* pCatalogue);

bool DfsExportFile(const char* pszDiscFile,
                   int NumSides,
                   int Side,
                   const DfsFileAttributes* pFileAttr,
                   const char* pszExportFolder,
                   char* pszErrStr);

bool DfsImportFile(const char* pszDiscFile,
                   int NumSides,
                   int Side,
                   DfsDiscCatalogue* pCatalog,
                   const char* pszFile,
                   const char* pszImportFolder,
                   char* pszErrStr);

#endif
