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

enum class EconetLogMessageType
{
	Status,
	Data,
	Broadcast
};

class EconetLogMessage
{
	public:
		EconetLogMessage(EconetLogMessageType Type,
		                 const std::string& Message);

		EconetLogMessage(EconetLogMessageType Type,
		                 const std::string& Message,
		                 const unsigned char* pData,
		                 int Length);

		EconetLogMessage(const EconetLogMessage&) = delete;
		EconetLogMessage& operator=(const EconetLogMessage&) = delete;

		~EconetLogMessage();

	public:
		EconetLogMessageType GetType() const { return m_Type; }

		const char* GetMessageStr() const { return m_Message.c_str(); }
		size_t GetMessageLength() const { return m_Message.length(); }

		const unsigned char* GetData() const { return m_pData; }
		int GetDataLength() const { return m_DataLength; }

	private:
		EconetLogMessageType m_Type;
		std::string m_Message;
		unsigned char* m_pData;
		int m_DataLength;
};

class EconetLogBuffer
{
	public:
		EconetLogBuffer();
		~EconetLogBuffer();

	public:
		int GetSize() const;
		const EconetLogMessage* GetMessage(int Index) const;

		bool AddMessage(EconetLogMessage* pMessage);
		void SetFilter(bool Filter);
		void Clear();

	private:
		std::deque<EconetLogMessage*> m_Messages;
		std::deque<EconetLogMessage*> m_FilteredMessages;
		bool m_Filter;
};

EconetLogBuffer& GetEconetLogBuffer();

void EconetLog(EconetLogMessageType Type,
               const char *Format, ...);

void EconetLogData(EconetLogMessageType Type,
                   const unsigned char* pData,
                   int Length,
                   const char *Format, ...);


#endif
