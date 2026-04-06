/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2024  Chris Needham

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

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <ws2tcpip.h>

#include "Socket.h"
#include "StringUtils.h"

#ifndef WIN32
#include <errno.h>
#include <unistd.h>
#endif

#include <stdexcept>

/****************************************************************************/

int CloseSocket(SOCKET Socket)
{
	#ifdef WIN32

	return closesocket(Socket);

	#else

	return close(Socket);

	#endif
}

/****************************************************************************/

int GetLastSocketError()
{
	#ifdef WIN32

	return WSAGetLastError();

	#else

	return errno;

	#endif
}

/****************************************************************************/

bool SetSocketBlocking(SOCKET Socket, bool Blocking)
{
	#ifdef WIN32

	unsigned long Mode = Blocking ? 0 : 1;
	return ioctlsocket(Socket, FIONBIO, &Mode) == 0;

	#else

	int Flags = fcntl(Socket, F_GETFL, 0);
	if (Flags == -1) return false;

	if (Blocking)
	{
		Flags &= ~O_NONBLOCK;
	}
	else
	{
		Flags |= O_NONBLOCK;
	}

	return fcntl(Socket, F_SETFL, Flags) == 0;

	#endif
}

/****************************************************************************/

bool WouldBlock(int Error)
{
	#ifdef WIN32

	return Error == WSAEWOULDBLOCK;

	#else

	return Error == EWOULDBLOCK; // TODO: EAGAIN?

	#endif
}

/****************************************************************************/

bool EnableBroadcast(SOCKET Socket)
{
	const BOOL Broadcast = 1;

	return setsockopt(Socket, SOL_SOCKET, SO_BROADCAST,
	                  (const char*)&Broadcast, sizeof(Broadcast)) == 0;
}

/****************************************************************************/

bool SetReuseAddr(SOCKET Socket)
{
	const BOOL Reuse = 1;

	return setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR,
	                  (const char*)&Reuse, sizeof(Reuse)) == 0;
}

/****************************************************************************/

bool SetExclusiveAddrUse(SOCKET Socket)
{
	const BOOL Exclusive = 1;

	return setsockopt(Socket, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
	                  (const char*)&Exclusive, sizeof(Exclusive)) == 0;
}

/****************************************************************************/

unsigned long ParseIPAddress(const char* Name, const std::string& Value)
{
	wchar_t AddressStr[INET6_ADDRSTRLEN + 1];

	std::wstring wstrValue = Str2WStr(Value);
	wcsncpy(AddressStr, wstrValue.c_str(), _countof(AddressStr));
	AddressStr[_countof(AddressStr) - 1] = '\0';

	sockaddr_in Address;
	int AddressLength = sizeof(Address);

	int Result = WSAStringToAddressW(AddressStr,
	                                 AF_INET,
	                                 NULL,
	                                 (SOCKADDR*)&Address,
	                                 &AddressLength);

	if (Result != 0)
	{
		char Message[100];
		sprintf(Message, "%s: %s", Name, Value.c_str());

		throw std::invalid_argument(Message);
	}

	return Address.sin_addr.s_addr;
}

/****************************************************************************/

// Similar to inet_pton, which isn't available on Windows XP.

int ParseIPAddress(int Family, const char* pszName, void* pAddr)
{
	SOCKADDR_STORAGE_XP Addr;
	ZeroMemory(&Addr, sizeof(Addr));
	int AddressLength = sizeof(Addr);

	wchar_t Name[INET6_ADDRSTRLEN + 1];

	std::wstring wstrName = Str2WStr(pszName);
	wcsncpy(Name, wstrName.c_str(), _countof(Name));
	Name[_countof(Name) - 1] = '\0';

	int Result = WSAStringToAddressW(Name,
	                                 Family,
	                                 NULL,
	                                 (struct sockaddr*)&Addr,
	                                 &AddressLength);

	if (Result == 0)
	{
		if (Family == AF_INET)
		{
			struct in_addr* pInetAddr = (struct in_addr*)pAddr;

			*pInetAddr = ((struct sockaddr_in*)&Addr)->sin_addr;
			return 1;
		}
		else if (Family == AF_INET6)
		{
			struct in6_addr* pInetAddr = (struct in6_addr*)pAddr;

			*pInetAddr = ((struct sockaddr_in6*)&Addr)->sin6_addr;
			return 1;
		}
		else
		{
			return 0;
		}
	}

	return 0;
}

/****************************************************************************/

// Similar to inet_ntop, which isn't available on Windows XP.

bool IpAddressToString(int Family, const void* pAddress, std::string& Dest)
{
	SOCKADDR_STORAGE_XP Addr;
	ZeroMemory(&Addr, sizeof(Addr));

	if (Family == AF_INET)
	{
		struct sockaddr_in* pInetAddr = (struct sockaddr_in*)&Addr;
		pInetAddr->sin_family = AF_INET;
		memcpy(&pInetAddr->sin_addr, pAddress, sizeof(struct in_addr));
	}
	else if (Family == AF_INET6)
	{
		struct sockaddr_in6* pInetAddr = (struct sockaddr_in6*)&Addr;
		pInetAddr->sin6_family = AF_INET6;
		memcpy(&pInetAddr->sin6_addr, pAddress, sizeof(struct in6_addr));
	}
	else
	{
		return false;
	}

	wchar_t Name[INET6_ADDRSTRLEN + 1];
	DWORD AddressLength = INET6_ADDRSTRLEN + 1;

	int Result = WSAAddressToStringW((struct sockaddr*)&Addr,
	                                 sizeof(Addr),
	                                 nullptr,
	                                 Name,
	                                 &AddressLength);

	if (Result == 0)
	{
		Dest = WStr2Str(Name);
	}

	return nullptr;
}

/****************************************************************************/
