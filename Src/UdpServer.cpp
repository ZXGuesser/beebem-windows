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

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <process.h>

#include <assert.h>

#include "UdpServer.h"
#include "DebugTrace.h"
#include "Socket.h"

// #define DEBUG_IOCP
// #define DEBUG_UDP

/****************************************************************************/

ReceiveQueue::ReceiveQueue() :
	m_Head(0),
	m_Tail(0),
	m_Count(0)
{
	InitializeCriticalSection(&m_Lock);
}

/****************************************************************************/

ReceiveQueue::~ReceiveQueue()
{
	DeleteCriticalSection(&m_Lock);
}

/****************************************************************************/

bool ReceiveQueue::Push(const char* pData,
                        int Length,
                        const sockaddr_in* pSrc)
{
	bool Success = false;

	EnterCriticalSection(&m_Lock);

	if (m_Count < RECEIVE_QUEUE_DEPTH)
	{
		ReceivedPacket* pPacket = &m_Buffer[m_Tail];

		pPacket->Length = Length;
		pPacket->Src = *pSrc;

		memcpy(pPacket->Data, pData, Length);

		if (++m_Tail == RECEIVE_QUEUE_DEPTH)
		{
			m_Tail = 0;
		}

		m_Count++;

		Success = true;
	}

	LeaveCriticalSection(&m_Lock);

	return Success;
}

/****************************************************************************/

bool ReceiveQueue::Pop(ReceivedPacket* pPacket)
{
	bool Success = false;

	EnterCriticalSection(&m_Lock);

	if (m_Count > 0)
	{
		*pPacket = m_Buffer[m_Head];

		if (++m_Head == RECEIVE_QUEUE_DEPTH)
		{
			m_Head = 0;
		}

		m_Count--;

		Success = true;
	}

	LeaveCriticalSection(&m_Lock);

	return Success;
}

/****************************************************************************/

enum class IoOperation
{
	Receive,
	Send
};

struct PerIoContext
{
	OVERLAPPED Overlapped;
	WSABUF Buffer;
	char Data[UDPSERVER_MAX_PACKET_SIZE];
	sockaddr_in Addr;
	int AddrLen;
	IoOperation Operation;
};

/****************************************************************************/

UdpSocket::UdpSocket(UdpServer* pServer) :
	m_pServer(pServer),
	m_iocp(nullptr),
	m_Socket(INVALID_SOCKET),
	m_ListenPort(0)
{
}

/****************************************************************************/

UdpSocket::~UdpSocket()
{
}

/****************************************************************************/

bool UdpSocket::Create(HANDLE iocp)
{
	assert(m_Socket == INVALID_SOCKET);

	m_iocp = iocp;

	m_Socket = WSASocket(AF_INET,
	                     SOCK_DGRAM,
	                     IPPROTO_UDP,
	                     nullptr,
	                     0,
	                     WSA_FLAG_OVERLAPPED);

	if (m_Socket == INVALID_SOCKET)
	{
		#ifdef DEBUG_UDP
		DebugTrace("UdpSocket: Failed to create socket (error %d)\n",
		           GetLastSocketError());
		#endif

		return false;
	}

	#ifdef DEBUG_UDP
	DebugTrace("UdpSocket: Created socket\n");
	#endif

	return true;
}

/****************************************************************************/

bool UdpSocket::Bind(unsigned long IPAddress, unsigned short Port)
{
	assert(m_Socket != INVALID_SOCKET);

	sockaddr_in local;
	ZeroMemory(&local, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = IPAddress;
	local.sin_port = htons(Port);

	if (bind(m_Socket, (sockaddr*)&local, sizeof(local)) == SOCKET_ERROR)
	{
		#ifdef DEBUG_UDP
		DebugTrace("UdpSocket: Failed to bind socket (error %d)\n",
		           GetLastSocketError());
		#endif

		return false;
	}

	HANDLE Result = CreateIoCompletionPort((HANDLE)m_Socket,
	                                       m_iocp,
	                                       (ULONG_PTR)this,
	                                       0);

	if (Result == nullptr)
	{
		return false;
	}

	// Pre-post receive buffers
	for (int i = 0; i < ReceiveQueue::RECEIVE_QUEUE_DEPTH; i++)
	{
		PerIoContext* pContext = new(std::nothrow) PerIoContext();

		if (pContext == nullptr)
		{
			return false;
		}

		PostReceive(pContext);
	}

	#ifdef DEBUG_UDP
	DebugTrace("UdpSocket: Bind succeeded %s:%u\n", IpAddressStr(IPAddress).c_str(), Port);
	#endif

	return true;
}

/****************************************************************************/

void UdpSocket::Close()
{
	if (m_Socket != INVALID_SOCKET)
	{
		CancelIoEx((HANDLE)m_Socket, nullptr);

		closesocket(m_Socket);
		m_Socket = INVALID_SOCKET;
	}
}

/****************************************************************************/

bool UdpSocket::IsOpen() const
{
	return m_Socket != INVALID_SOCKET;
}

/****************************************************************************/

bool UdpSocket::EnableBroadcast()
{
	return ::EnableBroadcast(m_Socket);
}

/****************************************************************************/

bool UdpSocket::SetExclusiveAddrUse()
{
	return ::SetExclusiveAddrUse(m_Socket);
}

/****************************************************************************/

bool UdpSocket::SetReuseAddr()
{
	return ::SetReuseAddr(m_Socket);
}

/****************************************************************************/

unsigned short UdpSocket::GetListenPort() const
{
	return m_ListenPort;
}

/****************************************************************************/

bool UdpSocket::Send(unsigned long IPAddress,
                     unsigned short Port,
                     const unsigned char* pData,
                     int Length)
{
	return m_pServer->Send(m_Socket, IPAddress, Port, pData, Length);
}

/****************************************************************************/

bool UdpSocket::Received(const char* pData,
                         int Length,
                         const sockaddr_in* pSrc)
{
	return m_ReceiveQueue.Push(pData, Length, pSrc);
}

/****************************************************************************/

bool UdpSocket::PostReceive(PerIoContext* pIoContext)
{
	ZeroMemory(&pIoContext->Overlapped, sizeof(OVERLAPPED));

	pIoContext->Buffer.buf = pIoContext->Data;
	pIoContext->Buffer.len = UDPSERVER_MAX_PACKET_SIZE;
	pIoContext->AddrLen = sizeof(sockaddr_in);
	pIoContext->Operation = IoOperation::Receive;

	DWORD Flags = 0;

	int Result = WSARecvFrom(m_Socket,
	                         &pIoContext->Buffer,
	                         1,
	                         nullptr,
	                         &Flags,
	                         (sockaddr*)&pIoContext->Addr,
	                         &pIoContext->AddrLen,
	                         &pIoContext->Overlapped,
	                         nullptr);

	if (Result == SOCKET_ERROR)
	{
		if (WSAGetLastError() != WSA_IO_PENDING)
		{
			return false;
		}
	}

	return true;
}

/****************************************************************************/

bool UdpSocket::GetReceivedPacket(ReceivedPacket* pPacket)
{
	return m_ReceiveQueue.Pop(pPacket);
}

/****************************************************************************/

UdpServer::UdpServer() :
	m_iocp(nullptr),
	m_hThread(nullptr)
{
}

/****************************************************************************/

UdpServer::~UdpServer()
{
	Stop();
}

/****************************************************************************/

bool UdpServer::Start()
{
	m_iocp = CreateIoCompletionPort(INVALID_HANDLE_VALUE,
	                                nullptr,
	                                0,
	                                0);

	m_hThread = (HANDLE)_beginthreadex(nullptr,    // security
	                                   0,          // stack_size
	                                   ThreadFunc, // start_address
	                                   this,       // arglist
	                                   0,          // initflag
	                                   nullptr);   // thrdaddr

	return true;
}

/****************************************************************************/

void UdpServer::Stop()
{
	if (m_hThread == nullptr)
	{
		return;
	}

	assert(m_iocp != nullptr);

	#ifdef DEBUG_UDP
	DebugTrace("UdpServer::Stop\n");
	#endif

	// Signal the worker thread to exit.
	PostQueuedCompletionStatus(m_iocp,   // CompletionPort
	                           0,        // dwNumberOfBytesTransferred
	                           0,        // dwCompletionKey
	                           nullptr); // lpOverlapped

	WaitForSingleObject(m_hThread, INFINITE);

	CloseHandle(m_hThread);
	m_hThread = nullptr;

	CloseHandle(m_iocp);
	m_iocp = nullptr;
}

/****************************************************************************/

UdpSocket* UdpServer::CreateSocket()
{
	UdpSocket* pSocket  = new(std::nothrow) UdpSocket(this);

	if (pSocket != nullptr && !pSocket->Create(m_iocp))
	{
		delete pSocket;
		return nullptr;
	}

	return pSocket;
}

/****************************************************************************/

void UdpServer::CloseSocket(SOCKET* pSocket)
{
	CancelIoEx((HANDLE)*pSocket, nullptr);

	closesocket(*pSocket);
	*pSocket = INVALID_SOCKET;
}

/****************************************************************************/

bool UdpServer::Send(SOCKET Socket,
                     unsigned long IPAddress,
                     unsigned short Port,
                     const unsigned char* pData,
                     int Length)
{
	assert(m_hThread != nullptr);
	assert(m_iocp != nullptr);

	#ifdef DEBUG_UDP
	DebugTrace("UdpServer: Send packet to %s:%u (%d bytes)\n",
	        IpAddressStr(IPAddress).c_str(),
	        Port,
	        Length);

	DebugDumpBytes("UdpServer: Packet data:", pData, Length);
	#endif

	assert(Length < UDPSERVER_MAX_PACKET_SIZE);

	if (Length >= UDPSERVER_MAX_PACKET_SIZE)
	{
		return false;
	}

	PerIoContext* pContext = new(std::nothrow) PerIoContext();

	if (pContext == nullptr)
	{
		return false;
	}

	ZeroMemory(pContext, sizeof(PerIoContext));

	memcpy(pContext->Data, pData, Length);

	pContext->Buffer.buf = pContext->Data;
	pContext->Buffer.len = Length;

	pContext->Addr.sin_family = AF_INET;
	pContext->Addr.sin_addr.s_addr = IPAddress;
	pContext->Addr.sin_port = htons(Port);
	pContext->AddrLen = sizeof(pContext->Addr);

	pContext->Operation = IoOperation::Send;

	int Result = WSASendTo(Socket,
	                       &pContext->Buffer,
	                       1,
	                       nullptr,
	                       0,
	                       (sockaddr*)&pContext->Addr,
	                       pContext->AddrLen,
	                       &pContext->Overlapped,
	                       nullptr);

	if (Result == SOCKET_ERROR)
	{
		if (WSAGetLastError() != WSA_IO_PENDING)
		{
			delete pContext;
			return false;
		}
	}

	return true;
}

/****************************************************************************/

unsigned int __stdcall UdpServer::ThreadFunc(void* pParameter)
{
	UdpServer* pServer = reinterpret_cast<UdpServer*>(pParameter);

	pServer->ThreadFunc();

	return 0;
}

/****************************************************************************/

void UdpServer::ThreadFunc()
{
	while (true)
	{
		DWORD BytesTransferred = 0;
		ULONG_PTR CompletionKey = 0;
		OVERLAPPED* pOverlapped = nullptr;

		BOOL Success = GetQueuedCompletionStatus(m_iocp,
		                                         &BytesTransferred,
		                                         &CompletionKey,
		                                         &pOverlapped,
		                                         INFINITE);

		#ifdef DEBUG_IOCP
		DebugTrace("GetQueuedCompletionStatus returned %d, pOverlapped=%p, CompletionKey=%p, BytesTransferred=%lu\n",
		           Success, pOverlapped, CompletionKey, BytesTransferred);
		#endif

		// Check for shutdown signal.
		if (pOverlapped == nullptr && CompletionKey == 0 && BytesTransferred == 0)
		{
			break;
		}

		if (!Success)
		{
			continue;
		}

		UdpSocket* pSocket = reinterpret_cast<UdpSocket*>(CompletionKey);

		PerIoContext* pIoContext = (PerIoContext*)pOverlapped;

		if (pIoContext->Operation == IoOperation::Receive)
		{
			if (BytesTransferred > 0)
			{
				#ifdef DEBUG_UDP
				DebugTrace("UdpServer: Received packet from %s:%u (%d bytes)\n",
				        IpAddressStr(pIoContext->Addr.sin_addr.S_un.S_addr).c_str(),
				        ntohs(pIoContext->Addr.sin_port),
				        BytesTransferred);

				DebugDumpBytes("UdpServer: Packet data:", (const unsigned char*)pIoContext->Data, BytesTransferred);
				#endif

				pSocket->Received(pIoContext->Data, BytesTransferred, &pIoContext->Addr);
			}

			pSocket->PostReceive(pIoContext);
		}
		else if (pIoContext->Operation == IoOperation::Send)
		{
			// Send completed
			#ifdef DEBUG_UDP
			DebugTrace("Send completed\n");
			#endif

			delete pIoContext;
		}
	}
}

/****************************************************************************/
