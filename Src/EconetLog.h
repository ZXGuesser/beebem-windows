/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2026  Chris Needham

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

#ifndef ECONETLOG_HEADER
#define ECONETLOG_HEADER

#include <deque>
#include <string>

class EconetLogMessage
{
	public:
		explicit EconetLogMessage(const std::string& Message);

		EconetLogMessage(const std::string& Message,
		                 const unsigned char* pData,
		                 int Length);

		EconetLogMessage(const EconetLogMessage&) = delete;
		EconetLogMessage& operator=(const EconetLogMessage&) = delete;

		~EconetLogMessage();

	public:
		const char* c_str() const { return m_Message.c_str(); }
		size_t length() const { return m_Message.length(); }
		const unsigned char* GetData() const { return m_pData; }
		int GetDataLength() const { return m_DataLength; }
	private:
		std::string m_Message;
		unsigned char* m_pData;
		int m_DataLength;
};

void EconetLog(const char *Format, ...);
void EconetLogData(const unsigned char* pData, int Length, const char *Format, ...);

std::deque<EconetLogMessage>* GetEconetLogBuffer();

#endif
