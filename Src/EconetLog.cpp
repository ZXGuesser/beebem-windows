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

#include <windows.h>

#include "EconetLog.h"

#include <assert.h>

#include "BeebWin.h"
#include "Main.h"
#include "Messages.h"

/****************************************************************************/

static const size_t MAX_LOG_MESSAGES = 2000;

static EconetLogBuffer LogBuffer;

/****************************************************************************/

EconetLogMessage::EconetLogMessage(EconetLogMessageType Type,
                                   const std::string& Message) :
	m_Type(Type),
	m_Message(Message),
	m_pData(nullptr),
	m_DataLength(0)
{
}

/****************************************************************************/

EconetLogMessage::EconetLogMessage(EconetLogMessageType Type,
                                   const std::string& Message,
                                   const unsigned char* pData,
                                   int Length) :
	m_Type(Type),
	m_Message(Message),
	m_DataLength(Length)
{
	m_pData = new(std::nothrow) unsigned char[Length];

	if (m_pData != nullptr)
	{
		memcpy(m_pData, pData, Length);
	}
}

/****************************************************************************/

EconetLogMessage::~EconetLogMessage()
{
	if (m_pData != nullptr)
	{
		delete [] m_pData;
		m_pData = nullptr;
	}
}

/****************************************************************************/

EconetLogBuffer::EconetLogBuffer() :
	m_Filter(false)
{
}

/****************************************************************************/

EconetLogBuffer::~EconetLogBuffer()
{
	Clear();
}

/****************************************************************************/

int EconetLogBuffer::GetSize() const
{
	return m_Filter ? (int)m_FilteredMessages.size() : (int)m_Messages.size();
}

/****************************************************************************/

const EconetLogMessage* EconetLogBuffer::GetMessage(int Index) const
{
	return m_Filter ? m_FilteredMessages[Index] : m_Messages[Index];
}

/****************************************************************************/

bool EconetLogBuffer::AddMessage(EconetLogMessage* pMessage)
{
	bool BufferFull = m_Messages.size() == MAX_LOG_MESSAGES;

	if (BufferFull)
	{
		// Discard oldest message.
		EconetLogMessage* pLastMessage = m_Messages.front();

		m_Messages.pop_front();

		if (pLastMessage->GetType() != EconetLogMessageType::Broadcast)
		{
			m_FilteredMessages.pop_front();
		}

		delete pLastMessage;
	}

	m_Messages.push_back(pMessage);

	if (pMessage->GetType() != EconetLogMessageType::Broadcast)
	{
		m_FilteredMessages.push_back(pMessage);
	}

	return BufferFull;
}

/****************************************************************************/

void EconetLogBuffer::SetFilter(bool Filter)
{
	m_Filter = Filter;
}

/****************************************************************************/

void EconetLogBuffer::Clear()
{
	for (size_t i = 0; i < m_Messages.size(); i++)
	{
		delete m_Messages[i];
	}

	m_Messages.clear();
	m_FilteredMessages.clear();
}

/****************************************************************************/

EconetLogBuffer& GetEconetLogBuffer()
{
	return LogBuffer;
}

/****************************************************************************/

void EconetLog(EconetLogMessageType Type, const char *Format, ...)
{
	va_list Args;
	va_start(Args, Format);

	// 2026-07-11 11:45:00.123
	// _vscprintf doesn't count terminating '\0'
	#ifndef NDEBUG
	int Length = _vscprintf(Format, Args) + 24 + 1;
	assert(Length < 512 - 24 - 1);
	#endif

	SYSTEMTIME Time;
	GetLocalTime(&Time);

	char Buffer[512];

	sprintf(Buffer, "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
	        Time.wYear, Time.wMonth, Time.wDay,
	        Time.wHour, Time.wMinute, Time.wSecond, Time.wMilliseconds);

	vsprintf_s(Buffer + 24, (512 - 24) * sizeof(char), Format, Args);

	EconetLogMessage* pMessage = new(std::nothrow) EconetLogMessage(Type, Buffer);

	if (pMessage != nullptr)
	{
		bool BufferFull = LogBuffer.AddMessage(pMessage);

		PostMessage(mainWin->GethWnd(), WM_ECONET_APPEND_LOG, BufferFull, 0);
	}

	va_end(Args);
}

/****************************************************************************/

void EconetLogData(EconetLogMessageType Type,
                   const unsigned char* pData, int Length,
                   const char *Format, ...)
{
	va_list Args;
	va_start(Args, Format);

	// 2026-07-11 11:45:00.123
	// _vscprintf doesn't count terminating '\0'
	#ifndef NDEBUG
	int MessageLength = _vscprintf(Format, Args) + 24 + 1;
	assert(MessageLength < 512 - 24 - 1);
	#endif

	SYSTEMTIME Time;
	GetLocalTime(&Time);

	char Buffer[512];

	sprintf(Buffer, "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
	        Time.wYear, Time.wMonth, Time.wDay,
	        Time.wHour, Time.wMinute, Time.wSecond, Time.wMilliseconds);

	vsprintf_s(Buffer + 24, (512 - 24) * sizeof(char), Format, Args);

	EconetLogMessage* pMessage = new(std::nothrow) EconetLogMessage(Type, Buffer, pData, Length);

	if (pMessage != nullptr)
	{
		bool BufferFull = LogBuffer.AddMessage(pMessage);

		PostMessage(mainWin->GethWnd(), WM_ECONET_APPEND_LOG, BufferFull, 0);
	}

	va_end(Args);
}

/****************************************************************************/
