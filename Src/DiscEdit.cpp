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

#include <windows.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "DiscEdit.h"
#include "DiscInfo.h"
#include "FileUtils.h"

/****************************************************************************/

static int DFS_LENGTH_TO_SECTORS(int Length)
{
	return ((int)Length + DFS_SECTOR_SIZE - 1) / DFS_SECTOR_SIZE;
}

/****************************************************************************/

static void StripTrailingSpaces(char* str)
{
	size_t Index = strlen(str);

	while (Index > 0 && str[Index - 1] == ' ')
	{
		str[Index - 1] = 0;
		Index--;
	}
}

/****************************************************************************/

std::string DfsToLocalFileName(const std::string& DfsFileName)
{
	std::string FileName;

	for (std::string::size_type i = 0; i != DfsFileName.size(); i++)
	{
		char c = DfsFileName[i];

		if (!isprint(c) ||
		    c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' ||
		    c == '"' || c == '<'  || c == '>' || c == '|')
		{
			c = '_';
		}

		FileName += c;
	}

	return FileName;
}

/****************************************************************************/

static bool MakeDfsFileName(char* DFSName, const char* FileName)
{
	int Index = 0;

	if (strlen(FileName) >= 3 && FileName[1] == '.')
	{
		DFSName[0] = FileName[0];
		DFSName[1] = '.';
		Index = 2;
	}
	else
	{
		strcpy(DFSName, "$.");
		Index = 0;
	}

	int Count = 0;

	while (Count < DFS_MAX_NAME_LEN && FileName[Index] != 0)
	{
		char c = static_cast<char>(toupper(FileName[Index]));

		if ((c >= 'A' && c <= 'Z') ||
		    (c >= '0' && c <= '9') ||
		    c == '!' || c == '$' || c == '%' || c == '^' || c == '&' || c == '(' || c == ')' ||
		    c == '_' || c == '-' || c == '=' || c == '+' || c == '[' || c == ']' || c == '{' ||
		    c == '}' || c == '@' || c == '#' || c == '~' || c == ',')
		{
			DFSName[Count + 2] = c;
			Count++;
		}

		Index++;
	}

	return Count > 0;
}

/****************************************************************************/

static void DfsGetFilesFromCatalog(const unsigned char* pSector0,
                                   const unsigned char* pSector1,
                                   int NumFiles,
                                   DfsFileAttributes* pFileAttr)
{
	for (int i = 0; i < NumFiles; ++i)
	{
		// Reverse order so files are in start sector order.
		int Offset = (NumFiles - i) * 8;

		memcpy(pFileAttr[i].FileName, pSector0 + Offset, 7);
		StripTrailingSpaces(pFileAttr[i].FileName);

		pFileAttr[i].Directory   = pSector0[Offset + 7] & 0x7f;
		pFileAttr[i].Locked      = (pSector0[Offset + 7] & 0x80) ? true : false;

		unsigned char c = pSector1[Offset + 6];

		pFileAttr[i].LoadAddr    = pSector1[Offset]     + (pSector1[Offset + 1] << 8) + ((c & 0x0c) ? 0xffff0000 : 0);
		pFileAttr[i].ExecAddr    = pSector1[Offset + 2] + (pSector1[Offset + 3] << 8) + ((c & 0xc0) ? 0xffff0000 : 0);
		pFileAttr[i].Length      = pSector1[Offset + 4] + (pSector1[Offset + 5] << 8) + ((c & 0x30) << 12);
		pFileAttr[i].StartSector = pSector1[Offset + 7] + ((c & 0x03) << 8);
	}
}

/****************************************************************************/

bool DfsGetCatalogue(const char* pszDiscFile,
                     int NumSides,
                     int Side,
                     DfsDiscCatalogue* pCatalog)
{
	bool Success = true;

	FILE* pFile = fopen(pszDiscFile, "rb");

	if (pFile == nullptr)
	{
		Success = false;
	}
	else
	{
		bool DoubleSidedSSD = IsDoubleSidedSSD(pszDiscFile, pFile);

		int CatalogOffset = 0;

		if (NumSides == 2 && Side == 1)
		{
			if (DoubleSidedSSD)
			{
				// Tracks are not interleaved, so side 2 appears after side 1.
				CatalogOffset = 80 * DFS_SECTORS_PER_TRACK * DFS_SECTOR_SIZE;
			}
			else
			{
				// Tracks are interleaved in a double sided disc image.
				CatalogOffset = DFS_SECTORS_PER_TRACK * DFS_SECTOR_SIZE;
			}
		}

		if (fseek(pFile, CatalogOffset, SEEK_SET) != 0)
		{
			Success = false;
		}
		else
		{
			unsigned char Buffer[DFS_SECTOR_SIZE * 4];
			ZeroMemory(Buffer, DFS_SECTOR_SIZE * 4);

			if (fread(Buffer, 1, DFS_SECTOR_SIZE * 4, pFile) < DFS_SECTOR_SIZE * 2)
			{
				Success = false;
			}
			else
			{
				ZeroMemory(pCatalog, sizeof(DfsDiscCatalogue));
				memcpy(pCatalog->Title, Buffer, 8);
				memcpy(pCatalog->Title + 8, Buffer + DFS_SECTOR_SIZE, 4);
				StripTrailingSpaces(pCatalog->Title);

				pCatalog->NumWrites  = Buffer[DFS_SECTOR_SIZE + 4];
				pCatalog->NumFiles   = Buffer[DFS_SECTOR_SIZE + 5] / 8;
				pCatalog->NumSectors = ((Buffer[DFS_SECTOR_SIZE + 6] & 3) << 8) + Buffer[DFS_SECTOR_SIZE + 7];
				pCatalog->BootOpts   = (Buffer[DFS_SECTOR_SIZE + 6] >> 4) & 3;

				// Read first 31 files
				DfsGetFilesFromCatalog(Buffer, Buffer + DFS_SECTOR_SIZE, pCatalog->NumFiles, pCatalog->FileAttrs);

				// Check for a Watford DFS 62 file catalogue
				int i;

				for (i = DFS_SECTOR_SIZE * 2; i < DFS_SECTOR_SIZE * 2 + 8; ++i)
				{
					if (Buffer[i] != 0xAA)
						break;
				}

				if (i == DFS_SECTOR_SIZE * 2 + 8)
				{
					pCatalog->Watford62 = true;

					// Total number of sectors is in second catalogue
					pCatalog->NumSectors = ((Buffer[DFS_SECTOR_SIZE * 3 + 6] & 3) << 8) +
					                       Buffer[DFS_SECTOR_SIZE * 3 + 7];

					i = Buffer[DFS_SECTOR_SIZE * 3 + 5] / 8;

					DfsGetFilesFromCatalog(Buffer + DFS_SECTOR_SIZE * 2,
					                       Buffer + DFS_SECTOR_SIZE * 3,
					                       i,
					                       &pCatalog->FileAttrs[pCatalog->NumFiles]);

					pCatalog->NumFiles += i;
				}
			}
		}

		fclose(pFile);
	}

	return Success;
}

/****************************************************************************/

bool DfsExportFile(const char* pszDiscFile,
                   int NumSides,
                   int Side,
                   const DfsFileAttributes* pFileAttr,
                   const char* pszLocalFileName,
                   char* pszErrStr)
{
	bool Success = true;
	unsigned char buffer[DFS_SECTOR_SIZE];

	FILE* pDiscFile = fopen(pszDiscFile, "rb");

	if (pDiscFile == nullptr)
	{
		sprintf(pszErrStr, "Failed to open disc file:\n  %s", pszDiscFile);
		return false;
	}

	FILE* pFile = fopen(pszLocalFileName, "wb");

	if (pFile == nullptr)
	{
		sprintf(pszErrStr, "Failed to open file for writing:\n  %s", pszLocalFileName);
		Success = false;
	}
	else
	{
		bool DoubleSidedSSD = IsDoubleSidedSSD(pszDiscFile, pDiscFile);

		// Export the data
		int Sector = pFileAttr->StartSector;
		size_t Length = (size_t)pFileAttr->Length;
		int Offset;
		int Track;

		while (Success && Length > 0)
		{
			// Calc file offset for next sector
			if (NumSides == 1)
			{
				Offset = Sector * DFS_SECTOR_SIZE;
			}
			else
			{
				if (DoubleSidedSSD)
				{
					Offset = 80 * DFS_SECTORS_PER_TRACK * DFS_SECTOR_SIZE;
					Offset += Sector * DFS_SECTOR_SIZE;
				}
				else
				{
					Track = Sector / DFS_SECTORS_PER_TRACK;
					Offset = (Track * 2 + Side) * DFS_SECTORS_PER_TRACK;
					Offset += (Sector % DFS_SECTORS_PER_TRACK);
					Offset *= DFS_SECTOR_SIZE;
				}
			}

			// Read next sector
			size_t n = Length < DFS_SECTOR_SIZE ? Length : DFS_SECTOR_SIZE;

			if (fseek(pDiscFile, Offset, SEEK_SET) != 0 ||
			    fread(buffer, 1, n, pDiscFile) != n)
			{
				sprintf(pszErrStr, "Failed to read data from:\n  %s", pszDiscFile);
				Success = false;
			}
			else
			{
				// Export to file
				if (fwrite(buffer, 1, n, pFile) != n)
				{
					sprintf(pszErrStr, "Failed to write data to:\n  %s", pszLocalFileName);
					Success = false;
				}
			}

			Sector++;
			Length -= n;
		}

		fclose(pFile);

		if (Success)
		{
			// Create .inf file.
			std::string InfFileName = pszLocalFileName + std::string(".inf");

			pFile = fopen(InfFileName.c_str(), "wb");

			if (pFile == nullptr)
			{
				sprintf(pszErrStr, "Failed to open file for writing:\n  %s", InfFileName.c_str());
				Success = false;
			}
			else
			{
				fprintf(pFile, "%c.%-7s %06X %06X %06X\n",
				        pFileAttr->Directory,
				        pFileAttr->FileName,
				        pFileAttr->LoadAddr & 0xffffff,
				        pFileAttr->ExecAddr & 0xffffff,
				        pFileAttr->Length);

				fclose(pFile);
			}
		}
	}

	fclose(pDiscFile);

	return Success;
}

/****************************************************************************/

static void DfsWriteFilesToCatalog(unsigned char* pSector0,
                                   unsigned char* pSector1,
                                   int NumFiles,
                                   DfsFileAttributes* pFileAttrs)
{
	for (int i = 0; i < NumFiles; ++i)
	{
		// Reverse order so files are in reverse start sector order.
		int Offset = (NumFiles - i) * 8;

		memcpy(pSector0 + Offset, pFileAttrs[i].FileName, 7);

		for (int j = (int)strlen(pFileAttrs[i].FileName); j < 7; ++j)
		{
			pSector0[Offset + j] = ' ';
		}

		pSector0[Offset + 7] = pFileAttrs[i].Directory | (pFileAttrs[i].Locked ? 0x80 : 0);

		unsigned char c = 0;

		pSector1[Offset + 0] = pFileAttrs[i].LoadAddr & 0xff;
		pSector1[Offset + 1] = (pFileAttrs[i].LoadAddr >> 8) & 0xff;
		c |= ((pFileAttrs[i].LoadAddr >> 16) & 0x3) << 2;

		pSector1[Offset + 2] = pFileAttrs[i].ExecAddr & 0xff;
		pSector1[Offset + 3] = (pFileAttrs[i].ExecAddr >> 8) & 0xff;
		c |= ((pFileAttrs[i].ExecAddr >> 16) & 0x3) << 6;

		pSector1[Offset + 4] = pFileAttrs[i].Length & 0xff;
		pSector1[Offset + 5] = (pFileAttrs[i].Length >> 8) & 0xff;
		c |= ((pFileAttrs[i].Length >> 16) & 0x3) << 4;

		pSector1[Offset + 7] = pFileAttrs[i].StartSector & 0xff;
		c |= ((pFileAttrs[i].StartSector >> 8) & 0x3);

		pSector1[Offset + 6] = c;
	}
}

/****************************************************************************/

bool DfsImportFile(const char* pszDiscFile,
                   int NumSides,
                   int Side,
                   DfsDiscCatalogue* pCatalogue,
                   const char* pszFileName,
                   const char* pszImportFolder,
                   char* pszErrStr)
{
	// Open the DFS disc image.
	FILE* pDiscImageFile = fopen(pszDiscFile, "rb+");

	if (pDiscImageFile == nullptr)
	{
		sprintf(pszErrStr, "Failed to open disc file for writing:\n  %s", pszDiscFile);
		return false;
	}

	bool Success = true;

	int StartSector = 0;
	int CatalogIndex = 0;
	int i;
	unsigned char Buffer[DFS_SECTOR_SIZE * 4];

	char DfsFileName[DFS_MAX_NAME_LEN + 3];

	// Build file names
	char FileName[MAX_PATH];
	strcpy(FileName, pszImportFolder);
	AppendPath(FileName, pszFileName);

	char InfFileName[MAX_PATH];
	strcpy(InfFileName, FileName);
	strcat(InfFileName, ".inf");

	// Read .inf file
	int LoadAddr = 0;
	int ExecAddr = 0;
	int FileLength = -1;

	FILE* pInfFile = fopen(InfFileName, "rb");

	if (pInfFile != NULL)
	{
		if (fscanf(pInfFile, "%9s %X %X", DfsFileName, (unsigned int*)&LoadAddr, (unsigned int*)&ExecAddr) < 1)
		{
			sprintf(pszErrStr, "Failed to read file attributes from:\n  %s", InfFileName);
			Success = false;
		}

		fclose(pInfFile);
	}
	else
	{
		// No .inf file, construct DFS filename from the host filename.
		ZeroMemory(DfsFileName, sizeof(DfsFileName));

		Success = MakeDfsFileName(DfsFileName, pszFileName);

		if (!Success)
		{
			sprintf(pszErrStr, "Failed to create DFS file name for:\n  %s", pszFileName);
		}
	}

	if (Success)
	{
		// Determine file length.
		if (GetFileLength(FileName, &FileLength))
		{
			if (FileLength < 0)
			{
				sprintf(pszErrStr, "Failed to determine file length:\n  %s", FileName);
				Success = false;
			}
			else if (FileLength >= 0x80000)
			{
				sprintf(pszErrStr, "File too large to import:\n  %s", FileName);
				Success = false;
			}
		}
		else
		{
			sprintf(pszErrStr, "Failed to open file:\n  %s", FileName);
			Success = false;
		}
	}

	if (Success)
	{
		// Search catalogue for existing file with same name.
		for (i = 0; i < pCatalogue->NumFiles; ++i)
		{
			if (pCatalogue->FileAttrs[i].Directory == DfsFileName[0] &&
			    strcmp(pCatalogue->FileAttrs[i].FileName, DfsFileName + 2) == 0)
			{
				// Delete the file
				memmove(&pCatalogue->FileAttrs[i], &pCatalogue->FileAttrs[i + 1],
				        sizeof(DfsFileAttributes) * pCatalogue->NumFiles - i - 1);
				pCatalogue->NumFiles--;
				break;
			}
		}
	}

	if (Success)
	{
		// Check for space in the catalogue
		if ((pCatalogue->Watford62 && pCatalogue->NumFiles >= 62) ||
		    (!pCatalogue->Watford62 && pCatalogue->NumFiles >= 31))
		{
			sprintf(pszErrStr, "Catalogue full, cannot import %s", pszFileName);
			Success = false;
		}
	}

	if (Success)
	{
		// Search catalogue for suitable gap (catalogue is in StartSector order).
		StartSector = pCatalogue->Watford62 ? 4 : 2;

		int NumSectors = DFS_LENGTH_TO_SECTORS(FileLength);

		for (i = 0; i < pCatalogue->NumFiles; ++i)
		{
			if ((pCatalogue->FileAttrs[i].StartSector - StartSector) >= NumSectors)
			{
				// Found a gap large enough.
				break;
			}
			else
			{
				StartSector = pCatalogue->FileAttrs[i].StartSector +
				              DFS_LENGTH_TO_SECTORS(pCatalogue->FileAttrs[i].Length);
			}
		}

		if (i == pCatalogue->NumFiles)
		{
			// Space at end of disc?
			if ((pCatalogue->NumSectors - StartSector) < NumSectors)
			{
				sprintf(pszErrStr, "Insufficient space to import %s", pszFileName);
				Success = false;
			}
		}

		CatalogIndex = i;
	}

	if (Success)
	{
		// Import the data.
		FILE* pFile = fopen(FileName, "rb");

		if (pFile == nullptr)
		{
			sprintf(pszErrStr, "Failed to open file:\n  %s", FileName);
			Success = false;
		}
		else
		{
			int Sector = StartSector;
			size_t len = (size_t)FileLength;

			while (Success && len > 0)
			{
				int Offset;

				// Calculate file offset for next sector.
				if (NumSides == 1)
				{
					Offset = Sector * DFS_SECTOR_SIZE;
				}
				else
				{
					int Track = Sector / DFS_SECTORS_PER_TRACK;
					Offset = (Track * 2 + Side) * DFS_SECTORS_PER_TRACK;
					Offset += (Sector % DFS_SECTORS_PER_TRACK);
					Offset *= DFS_SECTOR_SIZE;
				}

				// Read next sector
				ZeroMemory(Buffer, DFS_SECTOR_SIZE);

				size_t n = len < DFS_SECTOR_SIZE ? len : DFS_SECTOR_SIZE;

				if (fread(Buffer, 1, n, pFile) != n)
				{
					sprintf(pszErrStr, "Failed to read data from:\n  %s", FileName);
					Success = false;
				}
				else
				{
					// Import to file
					if (fseek(pDiscImageFile, Offset, SEEK_SET) != 0 ||
					    fwrite(Buffer, 1, DFS_SECTOR_SIZE, pDiscImageFile) != DFS_SECTOR_SIZE)
					{
						sprintf(pszErrStr, "Failed to write data to:\n  %s", pszDiscFile);
						Success = false;
					}
				}

				Sector++;
				len -= n;
			}

			fclose(pFile);
		}
	}

	if (Success)
	{
		// Update the catalogue
		int Offset = 0;

		if (NumSides == 2 && Side == 1)
		{
			// Tracks are interleaved in a double sided disc image.
			Offset = DFS_SECTORS_PER_TRACK * DFS_SECTOR_SIZE;
		}

		if (fseek(pDiscImageFile, Offset, SEEK_SET) != 0)
		{
			sprintf(pszErrStr, "Failed to seek in file:\n  %s", pszDiscFile);
			Success = false;
		}
		else
		{
			ZeroMemory(Buffer, DFS_SECTOR_SIZE * 4);

			size_t NumSectors = pCatalogue->Watford62 ? 4 : 2;

			if (fread(Buffer, 1, DFS_SECTOR_SIZE * NumSectors, pDiscImageFile) !=
			    (DFS_SECTOR_SIZE * NumSectors))
			{
				sprintf(pszErrStr, "Failed to read catalogue from:\n  %s", pszDiscFile);
				Success = false;
			}
			else
			{
				// Add file to catalogue
				memmove(&pCatalogue->FileAttrs[CatalogIndex + 1],
				        &pCatalogue->FileAttrs[CatalogIndex],
				        sizeof(DfsFileAttributes) * pCatalogue->NumFiles - CatalogIndex);

				strcpy(pCatalogue->FileAttrs[CatalogIndex].FileName, DfsFileName + 2);

				pCatalogue->FileAttrs[CatalogIndex].Directory = DfsFileName[0];
				pCatalogue->FileAttrs[CatalogIndex].Locked = false;
				pCatalogue->FileAttrs[CatalogIndex].LoadAddr = LoadAddr;
				pCatalogue->FileAttrs[CatalogIndex].ExecAddr = ExecAddr;
				pCatalogue->FileAttrs[CatalogIndex].Length = FileLength;
				pCatalogue->FileAttrs[CatalogIndex].StartSector = StartSector;

				pCatalogue->NumFiles++;

				// Update catalogue sectors.
				int n = (pCatalogue->NumFiles > 31) ? 31 : pCatalogue->NumFiles;
				Buffer[DFS_SECTOR_SIZE + 5] = (unsigned char)(n * 8);

				DfsWriteFilesToCatalog(Buffer, Buffer + DFS_SECTOR_SIZE, n, pCatalogue->FileAttrs);

				if (pCatalogue->Watford62)
				{
					n = (pCatalogue->NumFiles > 31) ? pCatalogue->NumFiles - 31 : 0;
					Buffer[DFS_SECTOR_SIZE * 3 + 5] = (unsigned char)(n * 8);

					DfsWriteFilesToCatalog(Buffer + DFS_SECTOR_SIZE * 2,
					                       Buffer + DFS_SECTOR_SIZE * 3,
					                       n,
					                       &pCatalogue->FileAttrs[31]);
				}

				// Write catalogue sectors back to file.
				if (fseek(pDiscImageFile, Offset, SEEK_SET) != 0)
				{
					sprintf(pszErrStr, "Failed to seek in file:\n  %s", pszDiscFile);
					Success = false;
				}
				else if (fwrite(Buffer, 1, DFS_SECTOR_SIZE * NumSectors, pDiscImageFile) !=
				         (DFS_SECTOR_SIZE * NumSectors))
				{
					sprintf(pszErrStr, "Failed to write catalogue to:\n  %s", pszDiscFile);
					Success = false;
				}
			}
		}
	}

	fclose(pDiscImageFile);

	if (!Success)
	{
		// Re-read the catalogue from the disc image.
		DfsGetCatalogue(pszDiscFile, NumSides, Side, pCatalogue);
	}

	return Success;
}

/****************************************************************************/
