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

#ifndef UDPSERVER_HEADER
#define UDPSERVER_HEADER

constexpr int UDPSERVER_MAX_PACKET_SIZE = 2048;

struct ReceivedPacket
{
    int Length;
    sockaddr_in Src;
    unsigned char Data[UDPSERVER_MAX_PACKET_SIZE];
};

class ReceiveQueue
{
	public:
		ReceiveQueue();
		~ReceiveQueue();

		bool Push(const unsigned char* pData,
		          int Length,
		          const sockaddr_in* pSrc);

		bool Pop(ReceivedPacket* pPacket);

	public:
		static constexpr int RECEIVE_QUEUE_DEPTH = 16;

	private:
		ReceivedPacket m_Buffer[RECEIVE_QUEUE_DEPTH];
		int m_Head;
		int m_Tail;
		int m_Count;
		CRITICAL_SECTION m_Lock;
};

struct PerIoContext;

class UdpServer;

class UdpSocket
{
	public:
		explicit UdpSocket(UdpServer* pServer);
		UdpSocket(const UdpSocket&) = delete;
		UdpSocket& operator=(const UdpSocket&) = delete;
		~UdpSocket();

	public:
		bool Create(HANDLE iocp);

		bool Bind(unsigned long IPAddress,
		          unsigned short Port);

		void Close();

		bool IsOpen() const;

		bool SetExclusiveAddrUse();
		bool SetReuseAddr();
		bool EnableBroadcast();

		unsigned short GetListenPort() const;

		bool Send(unsigned long IPAddress,
		          unsigned short Port,
		          const unsigned char* pData,
		          int Length);

		bool Received(const unsigned char* pData,
		              int Length,
		              const sockaddr_in* pSrc);

		bool PostReceive(PerIoContext* pIoContext);

		bool GetReceivedPacket(ReceivedPacket* pPacket);

	private:
		UdpServer* m_pServer;
		HANDLE m_iocp;
		SOCKET m_Socket;
		unsigned short m_ListenPort;
		ReceiveQueue m_ReceiveQueue;
};

class UdpServer
{
	public:
		UdpServer();
		~UdpServer();

		bool Start();
		void Stop();

		UdpSocket* CreateSocket();

		bool Send(SOCKET Socket,
		          unsigned long IPAddress,
		          unsigned short Port,
		          const unsigned char* pData,
		          int Length);

	private:
		void CloseSocket(SOCKET* pSocket);

		static unsigned int __stdcall ThreadFunc(void* pParameter);
		void ThreadFunc();

	private:
		HANDLE m_iocp;
		HANDLE m_hThread;
};

#endif
