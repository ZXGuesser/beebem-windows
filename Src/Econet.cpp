/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2004  Rob O'Donnell
Copyright (C) 2005  Mike Wyatt

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

// Econet support for BeebEm
// Rob O'Donnell. robert@irrelevant.com, December 28th 2004.
// Mike Wyatt - further development, Dec 2005
// AUN by Rob Jun/Jul 2009
//
// Search TODO for some issues that need addressing.
//
// Resources:
// * http://www.riscos.com/support/developers/prm/aun.html
// * https://mdfs.net/Docs/Comp/Econet/Specs/ISOLayer.txt
// * https://mdfs.net/Docs/Comp/Econet/Specs/Ports
// * https://mdfs.net/Docs/Comp/Econet/Specs/Packets

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdio.h>

#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <algorithm>

#include "Econet.h"
#include "6502core.h"
#include "BeebWin.h"
#include "Debug.h"
#include "DebugTrace.h"
#include "Main.h"
#include "Rtc.h"
#include "Socket.h"
#include "StringUtils.h"

// Emulated 6854 ADLC control registers.
// control1_b0 is AC
// this splits register address 0x01 as control2 and control3
// and register address 0x03 as tx-data-last-data and control4
struct MC6854
{
	unsigned char Control1;
	unsigned char Control2;
	unsigned char Control3;
	unsigned char Control4;

	unsigned char TxFifo[3];
	unsigned char RxFifo[3];
	unsigned char TxFifoPtr; // first empty byte in fifo
	unsigned char RxFifoPtr; // first empty byte in fifo
	unsigned char TxFifoTxLast; // tx fifo tx lst flags. (bits relate to subscripts)
	unsigned char RxFifoFCFlags; // rx fifo fc flags bits
	unsigned char RxFifoAPFlags; // rx fifo ap flags (bits relate to subscripts)

	unsigned char Status1;
	unsigned char Status2;

	int PriorityStatus; // PSE level for SR2 rx bits
	// 0 = inactive
	// 1 = ERR, FV, DCD, OVRN, ABT
	// 2 = Idle
	// 3 = AP
	// 4 = RDA

	bool CTS; // signal up
	bool Idle;
};

const unsigned char CONTROL_REG1_ADDRESS_CONTROL               = 0x01;
const unsigned char CONTROL_REG1_RX_INT_ENABLE                 = 0x02;
const unsigned char CONTROL_REG1_TX_INT_ENABLE                 = 0x04;
const unsigned char CONTROL_REG1_RDSR_MODE                     = 0x08;
const unsigned char CONTROL_REG1_TDSR_MODE                     = 0x10;
const unsigned char CONTROL_REG1_RX_FRAME_DISCONTINUE          = 0x20;
const unsigned char CONTROL_REG1_RX_RESET                      = 0x40;
const unsigned char CONTROL_REG1_TX_RESET                      = 0x80;

const unsigned char CONTROL_REG2_PRIORITIZED_STATUS_ENABLE     = 0x01;
const unsigned char CONTROL_REG2_2_BYTE_TRANSFER               = 0x02;
const unsigned char CONTROL_REG2_FLAG_MARK_IDLE                = 0x04;
const unsigned char CONTROL_REG2_TDRA_SELECT                   = 0x08;
const unsigned char CONTROL_REG2_FRAME_COMPLETE                = 0x08;
const unsigned char CONTROL_REG2_TX_LAST_DATA                  = 0x10;
const unsigned char CONTROL_REG2_CLEAR_RX_STATUS               = 0x20;
const unsigned char CONTROL_REG2_CLEAR_TX_STATUS               = 0x40;
const unsigned char CONTROL_REG2_RTS_CONTROL                   = 0x80;

const unsigned char CONTROL_REG3_LOGICAL_CONTROL_FIELD_SELECT  = 0x01;
const unsigned char CONTROL_REG3_EXTENDED_CONTROL_FIELD_SELECT = 0x02;
const unsigned char CONTROL_REG3_AUTO_ADDRESS_EXTENSION_MODE   = 0x04;
const unsigned char CONTROL_REG3_01_11_IDLE                    = 0x08;
const unsigned char CONTROL_REG3_FLAG_DETECTED_STATUS_ENABLE   = 0x10;
const unsigned char CONTROL_REG3_LOOP_MODE                     = 0x20;
const unsigned char CONTROL_REG3_GO_ACTIVE_ON_POLL             = 0x40;
const unsigned char CONTROL_REG3_TEST                          = 0x40;
const unsigned char CONTROL_REG3_LOOP_ONLINE_CONTROL           = 0x80;
const unsigned char CONTROL_REG3_LOOP_DTR                      = 0x80;

const unsigned char CONTROL_REG4_DOUBLE_FLAG                   = 0x01;
const unsigned char CONTROL_REG4_TX_WORD_LENGTH                = 0x06;
const unsigned char CONTROL_REG4_RX_WORD_LENGTH                = 0x18;
const unsigned char CONTROL_REG4_TX_ABORT                      = 0x20;
const unsigned char CONTROL_REG4_ABORT_EXTEND                  = 0x40;
const unsigned char CONTROL_REG4_NRZI                          = 0x80;

const unsigned char STATUS_REG1_RX_DATA_AVAILABLE              = 0x01;
const unsigned char STATUS_REG1_STATUS2_READ_REQUEST           = 0x02;
const unsigned char STATUS_REG1_LOOP                           = 0x04;
const unsigned char STATUS_REG1_FLAG_DETECTED                  = 0x08;
const unsigned char STATUS_REG1_CTS                            = 0x10;
const unsigned char STATUS_REG1_TX_UNDERRUN                    = 0x20;
const unsigned char STATUS_REG1_TDRA                           = 0x40;
const unsigned char STATUS_REG1_FRAME_COMPLETE                 = 0x40;
const unsigned char STATUS_REG1_IRQ                            = 0x80;

const unsigned char STATUS_REG2_ADDRESS_PRESENT                = 0x01;
const unsigned char STATUS_REG2_FRAME_VALID                    = 0x02;
const unsigned char STATUS_REG2_INACTIVE_IDLE_RECEIVED         = 0x04;
const unsigned char STATUS_REG2_ABORT_RECEIVED                 = 0x08;
const unsigned char STATUS_REG2_FCS_ERROR                      = 0x10;
const unsigned char STATUS_REG2_DCD                            = 0x20;
const unsigned char STATUS_REG2_RX_OVERRUN                     = 0x40;
const unsigned char STATUS_REG2_RX_DATA_AVAILABLE              = 0x80;

// Configuration Options.
// These, among others, are overridden in Econet.cfg, see ReadNetwork()
const int DEFAULT_FLAG_FILL_TIMEOUT = 500000;
const int DEFAULT_SCOUT_ACK_TIMEOUT = 5000;
const unsigned int DEFAULT_TIME_BETWEEN_BYTES = 128;
const unsigned int DEFAULT_FOUR_WAY_STAGE_TIMEOUT = 500000;
const unsigned char DEFAULT_PREFERRED_NET = 1;
const bool DEFAULT_MASSAGE_NETWORKS = false;
const bool DEFAULT_AUTOCONFIGURE = false;
const bool DEFAULT_FINDGATEWAYS = false;

static unsigned int FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
static bool MassageNetworks = DEFAULT_MASSAGE_NETWORKS; // Massage network numbers on send/receive (add/sub 128)
static bool AutoConfigure = DEFAULT_AUTOCONFIGURE; // Enable station autoconfiguration and discovery features
static bool FindGateways = DEFAULT_FINDGATEWAYS; // Enable gateway discovery

bool EconetStateChanged = false;
bool EconetEnabled;    // Enable hardware
bool EconetNMIEnabled; // 68B54 -> NMI enabled. (IC97)
int EconetTrigger;     // Poll timer

const unsigned int DEFAULT_GATEWAY_TIMEOUT = 300; // 5 minutes
time_t GatewayTimeout; // Gateway timer - system time not emulation trigger.

const unsigned int ANNOUNCE_TIMEOUT = 15;
time_t AnnounceTimeout = 0; // BeebEm host announcement timer - system time not emulation trigger.
uint32_t AnnounceHandle; // Sequence number for host announcements.
const unsigned int HOST_TIMEOUT = 60; // How long since last ping before hosts can be replaced

static const unsigned char powers[4] = { 1, 2, 4, 8 };

// Frequency between network actions.
// max 250Khz network clock. 2MHz system clock. one click every 8 cycles.
// say one byte takes about 8 clocks, receive a byte every 64 cpu cycles. ?
// (The reason for "about" 8 clocks is that as this a continuous synchronous tx,
// there are no start/stop bits, however to avoid detecting a dead line as ffffff
// zeros are added and removed transparently if you get more than five "1"s
// during data transmission - more than 5 are flags or errors)
// 6854 datasheet has max clock frequency of 1.5MHz for the B version.
// 64 cycles seems to be a bit fast for 'netmon' prog to keep up - set to 128.
static unsigned int TimeBetweenBytes = DEFAULT_TIME_BETWEEN_BYTES;

// Station Configuration settings:
// You specify station number on command line.
// This allows multiple different instances of the emulator to be run and
// to communicate with each other. Note that you STILL need to have them
// all listed in Econet.cfg so each one knows where the others are.
unsigned char EconetStationID = 0; // Default Station ID
unsigned char EconetNetworkID = 0; // Default Network ID

unsigned char PreferredStationID = 0;
unsigned char PreferredNetworkID = 0;

static unsigned short EconetListenPort = 0; // Default listen port
static unsigned long EconetListenIP = 0; // IP address

static SOCKET Socket = INVALID_SOCKET; // Also used to flag line up and clock running
static SOCKET BroadcastListenSocket = INVALID_SOCKET;

const unsigned short DEFAULT_AUN_PORT = 32768;
const unsigned char BEEBEM_ECONET_PORT = 0x9b; // where gateway replies and BeebEm Ping/Pong will be sent

// Written in 2004:
// We will be using Econet over Ethernet as per AUN,
// however I've not got a real Acorn ethernet machine to see how
// it actually works! The only details I can find is it is:
// "Standard econet encapsulated in UDP to port 32768" and that
// addressing defaults to "1.0.net.stn" where net >= 128 for Ethernet.
// but can be overridden, so we won't worry about that.

// 2009: Now I come back to this, I know the format ... :-)
// and sure enough, it's pretty simple.
// It's translating the different protocols that was harder.

enum class AUNType : unsigned char
{
	Broadcast = 1,
	Unicast = 2,
	Ack = 3,
	NAck = 4,
	Immediate = 5,
	ImmReply = 6,
	BeebEm = 0xff  // proprietary BeebEm messages carried over an AUN port
};

struct AUNHeaderType
{
	AUNType Type;           // AUN magic protocol byte
	unsigned char Port;     // Dest port
	unsigned char CtrlByte; // Control byte
	unsigned char Pad;      // retrans
	uint32_t Handle;        // 4 byte sequence little-endian.
};

static unsigned long ec_sequence = 0;

enum class FourWayStage
{
	Idle,
	ScoutSent,
	ScoutAckReceived,
	DataSent,
	WaitForIdle,
	ScoutReceived,
	ScoutAckSent,
	DataReceived,
	ImmediateSent,
	ImmediateReceived
};

static FourWayStage AUNState;

enum class BroadcastSource
{
	Unknown,
	Local,
	Gateway
};

struct EconetHeaderType
{
	unsigned char DestStn;
	unsigned char DestNet;
	unsigned char SrcStn;
	unsigned char SrcNet;
};

struct LongEconetPacket
{
	unsigned char DestStn;
	unsigned char DestNet;
	unsigned char SrcStn;
	unsigned char SrcNet;
	unsigned char CtrlByte;
	unsigned char Port;
};

// MC6854 has 3 byte FIFOs. There is no wait for an end of data
// before transmission starts. Data is sent immediately it's put into
// the first slot.

// Does Econet send multiple packets for big transfers, or just one huge
// packet?
// What's MTU on econet? Depends on clock speed but it's big (e.g. 100K).
// As we are using UDP, we will construct a 2048 byte buffer, accept data
// into this, and send it periodically.  We will accept incoming data
// similarly, and dribble it back into the emulated 68B54.
// We should thus never suffer underrun errors....
// --we do actually flag an underrun, if data exceeds the size of the buffer.
// -- sniffed AUN between live Arcs seems to max out at 1288 bytes (1280+header)
// --- bigger packets ARE possible - UDP fragments & reassembles transparently.. doh..

// 64K max.. can't see any transfers being needed larger than this too often!
// (and that's certainly larger than Acorn bridges can cope with.)
const int ETHERNET_BUFFER_SIZE = 65536;

// Buffers
// =======
//
// Non-AUN Mode
// ------------
//
// Transmit: Beeb -> ADLC.txfifo -> BeebTx -> sendto()
// Receive:  recvfrom() -> BeebRx -> ADLC.rxfifo -> Beeb
//
// AUN Mode
// --------
//
// Transmit: Beeb -> ADLC.txfifo -> BeebTx -> EconetTX -> sendto()
// Receive:  recvfrom() -> EconetRX -> BeebRx -> ADLC.rxfifo -> Beeb

// Buffers used to construct packets sent to/received from BBC micro

struct EconetPacket
{
	union {
		LongEconetPacket EconetHeader;
		unsigned char Buffer[ETHERNET_BUFFER_SIZE + 12];
	};

	unsigned int Pointer;
	unsigned int BytesInBuffer;
};

static EconetPacket BeebTx;
static EconetPacket BeebRx;

static unsigned char BeebTxCopy[sizeof(LongEconetPacket)];

struct EthernetPacket
{
	union {
		unsigned char raw[sizeof(AUNHeaderType)];
		AUNHeaderType AUNHeader;
	};

	union {
		unsigned char Buffer[ETHERNET_BUFFER_SIZE];
		EconetHeaderType EconetHeader;
	};

	unsigned int Pointer;
	unsigned int BytesInBuffer;

	unsigned char DestStn;
	unsigned char DestNet;
};

struct ExtendedAUNPacket
{
	EconetHeaderType EconetHeader;
	AUNHeaderType AUNHeader;
	unsigned char Buffer[ETHERNET_BUFFER_SIZE];
};

// Buffers used to construct packets for sending out via UDP
static EthernetPacket EconetRx;
static EthernetPacket EconetTx;

// Temporary packet for discovery and gateway messages.
static EthernetPacket EconetTemp;

// Holds data from Econet.cfg file or a host we have discovered
struct EconetHost
{
	unsigned char station;
	unsigned char network;
	unsigned long inet_addr;
	unsigned short port;
	BroadcastSource broadcasts; // where to accept broadcasts from
	time_t timeout;
};

struct EconetNet
{
	unsigned long inet_addr;
	unsigned char network;
	unsigned short port; // AUN port or base port from which sequential ports are calculated
	BroadcastSource broadcasts; // where to accept broadcasts from
};

struct EconetGateway
{
	unsigned long inet_addr;
	unsigned short port;
};

struct NetStn
{
	unsigned char network;
	unsigned char station;
};

static std::vector<EconetHost> Stations; // Individual stations we know about
static std::vector<EconetNet> Networks; // AUN networks we know about

static EconetGateway Gateway = { 0, 0 }; // No gateway address

static unsigned char IRQCause;  // Flag to indicate cause of IRQ (SR1 bit 7)
static unsigned char S2RQCause; // Flag to indicate cause of S2RQ (SR1 bit 2)

char EconetCfgPath[MAX_PATH];
char AUNMapPath[MAX_PATH];

// A receiving station goes into flag fill mode while it is processing
// a message. This stops other stations sending messages that may interfere
// with the four-way handshake. Attempting to notify every station using
// IP messages when flag fill goes active/inactive would be complicated and
// would suffer from timing issues due to network latency, so a pseudo
// flag fill algorithm is emulated. We assume that the receiving station
// will go into flag fill when we send a message or when we see a message
// destined for another station. We cancel flag fill when we receive a
// message as the other station must have cancelled flag fill. In order to
// cancel flag fill after the last message of a four-way handshake we time it
// out - which is not ideal as we do not want to delay new messages any
// longer that we have to - but it will have to do for now!

static bool FlagFillActive; // Flag fill state
int EconetFlagFillTimeoutTrigger; // Trigger point for flag fill
int EconetFlagFillTimeout = DEFAULT_FLAG_FILL_TIMEOUT; // Cycles for flag fill timeout // added cfg file to override this
static int EconetScoutAckTrigger; // Trigger point for scout ack
static int EconetScoutAckTimeout = DEFAULT_SCOUT_ACK_TIMEOUT; // Cycles to delay before sending ack to scout (AUN mode only)
static int EconetFourWayTrigger;

static MC6854 ADLC;

// Bridges/Gateways doing dynamic AUN address translation might respond to
// WhatNet queries with nets that differ from the net we have been configured
// with. This causes all sorts of issues so we try to intercept the WhatNet
// response and fudge the value. For this we need to track which port the
// query requested the response be sent to.
int WhatNetPort = -1; // invalid value when not expecting a WhatNet response

//---------------------------------------------------------------------------

static bool ReadNetwork();
static bool EconetPollReal();
static void EconetSendPacket();
static bool EconetReceivePacket();
static void EconetError(const char *Format, ...);

//---------------------------------------------------------------------------

#ifdef DEBUG_ECONET

static void DebugDumpBytes(const char* pszMessage, const unsigned char* pData, int Length)
{
	const int BytesPerLine = 16;

	bool Pad  = Length > BytesPerLine;

	int Offset = 0;

	std::string str;

	while (Length > 0)
	{
		int i;

		for (i = 0; i < BytesPerLine && i < Length; i++)
		{
			char sz[5];
			sprintf(sz, "%02X ", pData[Offset + i]);

			str += sz;
		}

		if (Pad)
		{
			for (; i < BytesPerLine; i++)
			{
				str += "   ";
			}
		}

		str += "| ";

		for (i = 0; i < BytesPerLine && i < Length; i++)
		{
			str += isprint(pData[Offset + i]) ? pData[Offset + i] : '.';
		}

		DebugTrace("%s %s\n", pszMessage, str.c_str());

		str.clear();

		Length -= BytesPerLine;
		Offset += BytesPerLine;
	}
}

#endif

//---------------------------------------------------------------------------

static const char* AUNStateStr(FourWayStage State)
{
	switch (State)
	{
		case FourWayStage::Idle:
			return "Idle";

		case FourWayStage::ScoutSent:
			return "ScoutSent";

		case FourWayStage::ScoutAckReceived:
			return "ScoutAckReceived";

		case FourWayStage::DataSent:
			return "DataSent";

		case FourWayStage::WaitForIdle:
			return "WaitForIdle";

		case FourWayStage::ScoutReceived:
			return "ScoutReceived";

		case FourWayStage::ScoutAckSent:
			return "ScoutAckSent";

		case FourWayStage::DataReceived:
			return "DataReceived";

		case FourWayStage::ImmediateSent:
			return "ImmediateSent";

		case FourWayStage::ImmediateReceived:
			return "ImmediateReceived";

		default:
			return "?";
	}
}

//---------------------------------------------------------------------------

static bool IsBroadcastStation(unsigned char Station)
{
	return Station == 0 || Station == 255;
}

//---------------------------------------------------------------------------

static std::string IpAddressStr(unsigned long IpAddress)
{
	std::string str;
	IpAddressToString(AF_INET, &IpAddress, str);

	return str;
}

//---------------------------------------------------------------------------

static EconetHost* FindNetworkConfig(unsigned char Station, unsigned char Network)
{
	for (size_t i = 0; i < Stations.size(); ++i)
	{
		if (Stations[i].station == Station && Stations[i].network == Network)
		{
			return &Stations[i];
		}
	}

	return nullptr;
}

//---------------------------------------------------------------------------

// Add or replace a station in stations list.

static void AddStation(unsigned char Station,
                       unsigned char Network,
                       unsigned long IPAddress,
                       unsigned short Port,
                       BroadcastSource Broadcasts = BroadcastSource::Unknown)
{
	EconetHost* pHost = FindNetworkConfig(Station, Network);

	if (pHost != nullptr)
	{
		// Station already defined, replace it.
		pHost->station = Station;
		pHost->network = Network;
		pHost->inet_addr = IPAddress;
		pHost->port = Port;
		pHost->timeout = time(NULL) + HOST_TIMEOUT;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Replaced station %d.%d in host list (now at %s:%u)\n",
		           (int)Network,
		           (int)Station,
		           IpAddressStr(IPAddress).c_str(),
		           Port);
		#endif
	}
	else
	{
		// Station unknown, so add it.
		EconetHost Host;
		Host.station = Station;
		Host.network = Network;
		Host.inet_addr = IPAddress;
		Host.port = Port;
		Host.broadcasts = Broadcasts;
		Host.timeout = time(NULL) + HOST_TIMEOUT;

		Stations.emplace_back(Host);

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Added station %d.%d on %s:%u to host list\n",
		           (int)Network,
		           (int)Station,
		           IpAddressStr(IPAddress).c_str(),
		           Port);
		#endif
	}
}

//---------------------------------------------------------------------------

static void EconetCloseSockets()
{
	if (Socket != INVALID_SOCKET)
	{
		CloseSocket(Socket);
		Socket = INVALID_SOCKET;
	}

	if (BroadcastListenSocket != INVALID_SOCKET)
	{
		CloseSocket(BroadcastListenSocket);
		BroadcastListenSocket = INVALID_SOCKET;
	}
}

//---------------------------------------------------------------------------

static bool GetLocalIpAddresses(std::vector<unsigned long>& IpAddresses)
{
	IpAddresses.clear();

	// Get localhost IP address.

	char LocalHostName[256];

	if (gethostname(LocalHostName, sizeof(LocalHostName)) == SOCKET_ERROR)
	{
		EconetError("Econet: Failed to get local host name");
		return false;
	}

	struct addrinfo Hints;
	ZeroMemory(&Hints, sizeof(Hints));
	Hints.ai_family = AF_INET;
	Hints.ai_socktype = SOCK_DGRAM;
	Hints.ai_protocol = IPPROTO_UDP;

	struct addrinfo* pAddrInfo = nullptr;

	char ServiceName[20];
	sprintf(ServiceName, "%d", DEFAULT_AUN_PORT);

	int Result = getaddrinfo(LocalHostName,
	                         ServiceName,
	                         &Hints,
	                         &pAddrInfo);

	if (Result != 0)
	{
		EconetError("Econet: Failed to resolve local IP address");
		return false;
	}

	// Check address for each network interface/card.
	for (struct addrinfo* p = pAddrInfo; p != nullptr; p = p->ai_next)
	{
		struct sockaddr_in* pAddr = (struct sockaddr_in *)p->ai_addr;

		#ifdef DEBUG_ECONET
		DebugTrace("Local IP address: %s\n", IpAddressStr(pAddr->sin_addr.s_addr).c_str());
		#endif

		IpAddresses.emplace_back(pAddr->sin_addr.s_addr);
	}

	freeaddrinfo(pAddrInfo);

	return true;
}

//---------------------------------------------------------------------------

// Find an available station number.

static void AllocateNewAddress()
{
	std::vector<unsigned long> LocalIpAddresses;
	GetLocalIpAddresses(LocalIpAddresses);

	unsigned long LocalHostAddr;
	ParseIPAddress(AF_INET, "127.0.0.1", &LocalHostAddr);

	// See if configured addresses match local IPs.
	for (size_t i = 0; i < Stations.size(); ++i)
	{
		const EconetHost& Station = Stations[i];

		// Check address for each network interface/card.
		for (size_t j = 0; j < LocalIpAddresses.size(); ++j)
		{
			unsigned long IpAddress = LocalIpAddresses[j];

			if (Station.inet_addr == LocalHostAddr ||
			    Station.inet_addr == IpAddress)
			{
				if (PreferredStationID == 0 ||
				    (Station.station == PreferredStationID &&
				     Station.network == PreferredNetworkID))
				{
					sockaddr_in service;
					service.sin_family = AF_INET;
					service.sin_addr.s_addr = Station.inet_addr;
					service.sin_port = htons(Station.port);

					if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == 0)
					{
						EconetListenPort = Station.port;
						EconetListenIP = Station.inet_addr;
						EconetStationID = Station.station;
						EconetNetworkID = Station.network;
					}
					else
					{
						// Reset station announcement sequence number.
						AnnounceHandle = 0;
					}
				}
			}
		}
	}

	if (EconetStationID == 0)
	{
		// Still can't find one - try to find our AUNNet.
		#ifdef DEBUG_ECONET
		DebugTrace("Econet: couldn't get host from configured addresses - trying automatic\n");
		#endif

		if (PreferredStationID == 0)
		{
			for (size_t i = 0; i < Networks.size(); ++i)
			{
				const EconetNet& Network = Networks[i];

				for (size_t j = 0; j < LocalIpAddresses.size(); ++j)
				{
					unsigned long IpAddress = LocalIpAddresses[j];

					if (Network.inet_addr == (IpAddress & 0x00FFFFFF))
					{
						sockaddr_in service;
						service.sin_family = AF_INET;
						service.sin_addr.s_addr = IpAddress;
						service.sin_port = htons(DEFAULT_AUN_PORT);

						if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == 0)
						{
							EconetListenIP = IpAddress;
							EconetListenPort = DEFAULT_AUN_PORT;
							EconetStationID = IpAddress >> 24;
							EconetNetworkID = Network.network;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Automatically assigned station %d.%d using AUNMap\n",
							           EconetNetworkID,
							           EconetStationID);
							#endif
						}
						else
						{
							// Reset station announcement sequence number.
							AnnounceHandle = 0;
						}
					}
				}
			}
		}

		if (EconetStationID == 0 && AutoConfigure)
		{
			// Look for a free port and assign a suitable station number.
			// We assign station numbers at random to reduce the chances
			// of collisions between instances on different PCs. We have
			// no other way to prevent them so hope for the best!

			// Create randomly shuffled pool of all free (unconfigured)
			// station numbers in our net.

			std::vector<unsigned char> Numbers;
			for (unsigned char i = 0; i < 255; i++) Numbers.push_back(i); // vector of numbers 0-254
			Numbers[254] = 0; // Mark out station 254 so it can never be automatically assigned.

			for (size_t i = 0; i < Stations.size(); ++i)
			{
				// Mark out any configured station numbers in net.
				if (Stations[i].network == PreferredNetworkID)
				{
					Numbers[Stations[i].station] = 0;
				}
			}

			// Mark out preferred station number.
			Numbers[PreferredStationID] = 0;

			// Remove all the marked out numbers.
			Numbers.erase(std::remove(Numbers.begin(), Numbers.end(), 0),
			              Numbers.end());

			// Shuffle remaining station numbers.
			std::srand((unsigned int)std::time(0));
			std::random_shuffle(Numbers.begin(), Numbers.end());

			unsigned char StationID;

			// Try to bind the station ID asked for before picking randomly.
			if (PreferredStationID != 0)
			{
				StationID = PreferredStationID;
			}
			else
			{
				StationID = Numbers[0];

				// Reset station announcement sequence number.
				AnnounceHandle = 0;
			}

			for (size_t j = 0; j < Numbers.size(); )
			{
				const unsigned short Port = 10000 + (PreferredNetworkID << 8) + StationID;

				// TODO: This will use the first network address of this PC.
				// This might not be useful if there are multiple network
				// adapters but we have no good way to determine which to use
				// in the absence of any user configuration.

				sockaddr_in service;
				service.sin_family = AF_INET;
				service.sin_addr.s_addr = LocalIpAddresses[0];
				service.sin_port = htons(Port);

				if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == 0)
				{
					EconetListenIP = LocalIpAddresses[0];
					EconetListenPort = Port;
					EconetStationID = StationID;
					EconetNetworkID = PreferredNetworkID;

					#ifdef DEBUG_ECONET
					DebugTrace("Econet: automatically assigned random station %d.%d on %s:%d\n",
					           EconetNetworkID,
					           EconetStationID,
					           IpAddressStr(EconetListenIP).c_str(),
					           EconetListenPort);
					#endif

					break;
				}

				// Reset station announcement sequence number.
				AnnounceHandle = 0;

				StationID = Numbers[++j]; // The next number in the shuffled vector.
			}
		}

		if (EconetStationID == 0)
		{
			// Couldn't even bind a random port.
			EconetError("Econet: Failed to find free station/port to bind to");
		}
	}
}

//---------------------------------------------------------------------------

bool EconetReset()
{
	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet,
		                   true,
		                   "Econet: Reset (hardware %s)",
		                   EconetEnabled ? "enabled" : "disabled");
	}

	// Clear WhatNet reply port state.
	WhatNetPort = -1;

	// hardware operations:
	// set RxReset and TxReset
	ADLC.Control1 = CONTROL_REG1_RX_RESET | CONTROL_REG1_TX_RESET;
	// reset TxAbort, RTS, LoopMode, DTR
	ADLC.Control4 = 0; //ADLC.control4 & 223;
	ADLC.Control2 = 0; //ADLC.control2 & 127;
	ADLC.Control3 = 0; //ADLC.control3 & 95;

	// Clear all status conditions.
	ADLC.Status1 = 0; // CTS - clear to send line input (no collisions talking UDP)
	ADLC.Status2 = 0; // DCD - no clock (until sockets initialised and open)
	ADLC.PriorityStatus = 0;

	EconetRx.Pointer = 0;
	EconetRx.BytesInBuffer = 0;

	EconetTx.Pointer = 0;
	EconetTx.BytesInBuffer = 0;

	BeebRx.Pointer = 0;
	BeebRx.BytesInBuffer = 0;

	BeebTx.Pointer = 0;
	BeebTx.BytesInBuffer = 0;

	AUNState = FourWayStage::Idle; // Used for AUN mode translation stage.

	#ifdef DEBUG_ECONET
	DebugTrace("Econet: Set FourWayStage::Idle (Reset)\n");
	#endif

	ADLC.RxFifoPtr = 0;
	ADLC.RxFifoAPFlags = 0;
	ADLC.RxFifoFCFlags = 0;
	ADLC.TxFifoPtr = 0;
	ADLC.TxFifoTxLast = 0;

	ADLC.Idle = true;
	ADLC.CTS = false;

	IRQCause = 0;
	S2RQCause = 0;

	FlagFillActive = false;

	ClearTrigger(EconetTrigger);
	ClearTrigger(EconetFlagFillTimeoutTrigger);
	ClearTrigger(EconetScoutAckTrigger);
	ClearTrigger(EconetFourWayTrigger)

	EconetCloseSockets();

	// Stop here if not enabled.
	if (!EconetEnabled)
	{
		AnnounceHandle = 0; // Clear announce packet sequence number.
		return true;
	}

	// Read in Econet.cfg and AUNMap. Done here so can refresh it on Break.
	if (!ReadNetwork())
	{
		goto Fail;
	}

	if (PreferredNetworkID == 0)
	{
		// Config didn't include a DEFAULTNET.
		PreferredNetworkID = DEFAULT_PREFERRED_NET;
	}

	// Create a SOCKET for sending messages and listening for incoming
	// connection requests.
	Socket = socket(AF_INET, SOCK_DGRAM, 0);

	if (Socket == INVALID_SOCKET)
	{
		EconetError("Econet: Failed to open listening socket (error %d)", GetLastSocketError());
		goto Fail;
	}

	// Create a SOCKET for listening for incoming AUN broadcasts
	BroadcastListenSocket = socket(AF_INET, SOCK_DGRAM, 0);

	if (BroadcastListenSocket == INVALID_SOCKET)
	{
		EconetError("Econet: Failed to open broadcast listener socket (error %d)", GetLastSocketError());
		goto Fail;
	}

	/* needed if we bind our socket to 0.0.0.0, but means bringing in winsock2
	   which causes VS2022 to get very upset about lots of deprecated functions
	   that we can't replace if we want to retain XP compatibility

	// Stops multiple instances binding to the same port (which shouldn't be
	// possible but happens anyway where we bind to a wildcard address)
	if (!SetExclusiveAddrUse(Socket))
	{
		EconetError("Econet: Failed to set exclusive socket lock: %d", GetLastSocketError());
		goto Fail;
	}
	*/

	unsigned char LastEconetStationID = EconetStationID;

	if (AutoConfigure &&
	    PreferredStationID == 0 &&
	    (MachineType == Model::Master128 || MachineType == Model::MasterET))
	{
		// No PreferredStationID has been set so try to use the station number in CMOS.
		RTCWriteAddress(0xE);
		PreferredStationID = RTCReadData();
	}

	if (PreferredStationID != 0)
	{
		EconetHost* pNetworkConfig = FindNetworkConfig(PreferredStationID, PreferredNetworkID);

		if (pNetworkConfig != nullptr)
		{
			EconetListenPort = pNetworkConfig->port;
			EconetListenIP = pNetworkConfig->inet_addr;
			EconetStationID = pNetworkConfig->station;
			EconetNetworkID = pNetworkConfig->network;

			// The sockaddr_in structure specifies the address family,
			// IP address, and port for the socket that is being bound.
			sockaddr_in service;
			service.sin_family = AF_INET;
			service.sin_addr.s_addr = EconetListenIP;
			service.sin_port = htons(EconetListenPort);

			if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) != 0)
			{
				EconetError("Econet: Failed to bind to address %s:%d",
				            IpAddressStr(EconetListenIP).c_str(),
				            EconetListenPort);

				// Clear this so we don't try to allocate it again.
				PreferredStationID = 0;

				EconetStationID = 0;
				EconetNetworkID = 0;

				// Try to allocate a different station number instead.
				AllocateNewAddress();
			}
		}
		else
		{
			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Failed to find station %d.%d in Econet.cfg\n", PreferredNetworkID, PreferredStationID);
			#endif

			EconetStationID = 0;
			EconetNetworkID = 0;

			// Try to allocate a station number instead.
			AllocateNewAddress();
		}
	}
	else
	{
		// Try to allocate a station number instead.
		AllocateNewAddress();
	}

	if (EconetStationID != LastEconetStationID)
	{
		// Station number has changed
		// Reset announce packet sequence number.
		AnnounceHandle = 0;
	}

	if (EconetStationID == 0)
	{
		goto Fail;
	}

	#ifdef DEBUG_ECONET
	DebugTrace("Econet: Station number set to %d, port %d\n",
	           EconetStationID, EconetListenPort);
	#endif

	// On Master the station number is read from CMOS so update it
	if (MachineType == Model::Master128 || MachineType == Model::MasterET)
	{
		RTCWriteAddress(0xE);
		RTCWriteData(EconetStationID);
	}

	// This call is what allows broadcast packets to be sent.
	if (!EnableBroadcast(Socket))
	{
		EconetError("Econet: Failed to set socket for broadcasts (error %d)", GetLastSocketError());
		goto Fail;
	}

	if (EconetListenPort != DEFAULT_AUN_PORT)
	{
		// Bind additional BroadcastListenSocket for reception of AUN broadcasts.

		if (!EnableBroadcast(BroadcastListenSocket))
		{
			EconetError("Econet: Failed to set socket for shared reception of broadcasts (error %d)", GetLastSocketError());
			goto Fail;
		}

		sockaddr_in BroadcastAddr;
		BroadcastAddr.sin_family = AF_INET;
		BroadcastAddr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces.
		BroadcastAddr.sin_port = htons(DEFAULT_AUN_PORT);

		if (bind(BroadcastListenSocket, (SOCKADDR*)&BroadcastAddr, sizeof(BroadcastAddr)) == 0)
		{
			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Started broadcast listener\n");
			#endif
		}
	}

	// How long before we bother with poll routine?
	SetTrigger(TimeBetweenBytes, EconetTrigger);

	EconetStateChanged = true;

	if (FindGateways && Gateway.port == 0)
	{
		// Send a bridge discovery broadcast to learn of any
		// Pi Econet Bridge gateways on the network.
		sockaddr_in RecvAddr;
		RecvAddr.sin_family = AF_INET;
		RecvAddr.sin_addr.s_addr = INADDR_BROADCAST;
		RecvAddr.sin_port = htons(DEFAULT_AUN_PORT);

		EconetTemp.AUNHeader.Type = AUNType::Broadcast; // the gateway is listening for an AUN broadcast
		EconetTemp.AUNHeader.CtrlByte = 0x10; // control &90 locate gateway
		EconetTemp.AUNHeader.Port = 0x9c; // Pi Econet Bridge port
		EconetTemp.AUNHeader.Pad = 0;
		EconetTemp.AUNHeader.Handle = 0;
		ZeroMemory(EconetTemp.Buffer, 8);
		EconetTemp.Buffer[0] = BEEBEM_ECONET_PORT; // where response is sent

		// This is crude, but sometimes the gateway request broadcast
		// can get lost so spam them out.
		bool Error = false;

		for (int i = 0; i < 3; i++)
		{
			if (sendto(Socket, (char *)&EconetTemp, sizeof(EconetTemp.AUNHeader) + 8, 0,
			           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
			{
				Error = true;
			}
		}

		if (Error)
		{
			EconetError("Econet: Failed to send bridge discovery broadcast");
		}
	}

	if (AutoConfigure && AnnounceHandle == 0)
	{
		// Start up regular announce pings.
		time(&AnnounceTimeout);
	}

	return true;

Fail:
	AnnounceHandle = 0; // Clear announce packet sequence number.
	EconetCloseSockets();

	EconetEnabled = false;
	return false;
}

//---------------------------------------------------------------------------

// Read Econet.cfg file into network table

static bool ReadEconetConfigFile()
{
	std::ifstream Input(EconetCfgPath);

	if (!Input)
	{
		EconetError("Econet: Failed to open configuration file:\n  %s", EconetCfgPath);
		return false;
	}

	bool Success = true;

	Stations.clear();

	std::string Line;
	int LineCounter = 0;

	while (std::getline(Input, Line))
	{
		LineCounter++;

		Trim(Line);

		// Skip blank lines and comments
		if (Line.empty() || Line[0] == '#')
		{
			continue;
		}

		// Remove comments
		std::string::size_type Pos = Line.find('#');

		if (Pos != std::string::npos)
		{
			Line.erase(Pos);
		}

		// In BeebEm 4.19 and earlier, the default Econet.cfg file included
		// '//' comments on some lines
		Pos = Line.find("//");

		if (Pos != std::string::npos)
		{
			Line.erase(Pos);
		}

		std::vector<std::string> Tokens;

		ParseLine(Line, Tokens);

		try
		{
			if (((Tokens.size() == 5 && StrCaseCmp(Tokens[0].c_str(), "STATION") == 0)) ||
			    ((Tokens.size() == 4 && IsNumber(Tokens[0].c_str()))))
			{
				size_t Index = Tokens.size() - 4;

				unsigned char Network = (unsigned char)ParseNumber("Network", Tokens[Index], 1, 127);
				unsigned char Station = (unsigned char)ParseNumber("Station", Tokens[Index + 1], 1, 254);
				unsigned long IPAddress = ParseIPAddress("IP address", Tokens[Index + 2]);
				unsigned short Port = (unsigned short)ParseNumber("Port", Tokens[Index + 3], 0, 65535);

				AddStation(Station, Network, IPAddress, Port);
			}
			else if (Tokens.size() == 3 && StrCaseCmp(Tokens[0].c_str(), "GATEWAY") == 0)
			{
				if (Gateway.port == 0)
				{
					// No gateway configured.
					Gateway.inet_addr = ParseIPAddress("IP address", Tokens[1]);
					Gateway.port = (unsigned short)ParseNumber("Port", Tokens[2], 0, 65535);

					#ifdef DEBUG_ECONET
					DebugTrace("Econet: ConfigFile Gateway IP %s Port %d",
					           IpAddressStr(Gateway.inet_addr).c_str(),
					           Gateway.port);
					#endif

					// Wake up gateway as soon as Econet is initialised.
					time(&GatewayTimeout);
				}
				else
				{
					EconetError("Multiple gateway entries found in Econet config file:\n  %s (Line %d)", EconetCfgPath, LineCounter);
					Success = false;
					break;
				}
			}
			else if (Tokens.size() == 4 && StrCaseCmp(Tokens[0].c_str(), "NETWORK") == 0)
			{
				// This line defines an entire network. It is added before any networks
				// from AUNMap so takes precedence.

				EconetNet Network;

				Network.network    = (unsigned char)ParseNumber("Network", Tokens[1], 1, 127);
				Network.inet_addr  = ParseIPAddress("IP address", Tokens[2]);
				Network.port       = (unsigned short)ParseNumber("Port", Tokens[3], 1, 65535);
				Network.broadcasts = BroadcastSource::Unknown;

				#ifdef DEBUG_ECONET
				DebugTrace("Econet: ConfigFile Net %d IP %s Port %d\n",
				           Network.network,
				           IpAddressStr(Network.inet_addr).c_str(),
				           Network.port);
				#endif

				Networks.emplace_back(Network);
			}
			else if (Tokens.size() == 2)
			{
				// Key/value options.
				const std::string& Key   = Tokens[0];
				const std::string& Value = Tokens[1];

				if (StrCaseCmp(Key.c_str(), "AUNMODE") == 0)
				{
					// AUNMode removed jun 2025
				}
				else if (StrCaseCmp(Key.c_str(), "LEARN") == 0)
				{
					// LearnMode removed jun 2025
				}
				else if (StrCaseCmp(Key.c_str(), "AUNSTRICT") == 0)
				{
					// StrictAUNMode removed jun 2025
				}
				else if (StrCaseCmp(Key.c_str(), "SINGLESOCKET") == 0)
				{
					// Ignored.
				}
				else if (StrCaseCmp(Key.c_str(), "FLAGFILLTIMEOUT") == 0)
				{
					EconetFlagFillTimeout = ParseNumber(Key.c_str(), Value, 0, INT_MAX);
				}
				else if (StrCaseCmp(Key.c_str(), "SCACKTIMEOUT") == 0 ||
				         StrCaseCmp(Key.c_str(), "SCOUTACKTIMEOUT") == 0)
				{
					EconetScoutAckTimeout = ParseNumber(Key.c_str(), Value, 0, INT_MAX);
				}
				else if (StrCaseCmp(Key.c_str(), "TIMEBETWEENBYTES") == 0)
				{
					TimeBetweenBytes = ParseNumber(Key.c_str(), Value, 0, INT_MAX);
				}
				else if (StrCaseCmp(Key.c_str(), "FOURWAYTIMEOUT") == 0)
				{
					FourWayStageTimeout = ParseNumber(Key.c_str(), Value, 0, INT_MAX);
				}
				else if (StrCaseCmp(Key.c_str(), "MASSAGENETS") == 0)
				{
					MassageNetworks = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "AUTOCONFIGURE") == 0)
				{
					AutoConfigure = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "FINDGATEWAYS") == 0)
				{
					FindGateways = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "DEFAULTNET") == 0)
				{
					int NetworkID = ParseNumber(Key.c_str(), Value, 1, 127);

					if (PreferredNetworkID == 0) // Only set if uninitialized.
					{
						PreferredNetworkID = (unsigned char)NetworkID;
					}
				}
				else
				{
					EconetError("Unknown entry in Econet config file: %s\n  %s (Line %d)", Key.c_str(), EconetCfgPath, LineCounter);
				}
			}
		}
		catch (const std::exception& e)
		{
			EconetError("Invalid value in Econet config file\n%s\n%s (Line %d)", e.what(), EconetCfgPath, LineCounter);
			Success = false;
			break;
		}
	}

	return Success;
}

//---------------------------------------------------------------------------

static bool ReadAUNConfigFile()
{
	std::ifstream Input(AUNMapPath);

	if (!Input)
	{
		EconetError("Econet: Failed to open configuration file:\n  %s", AUNMapPath);
		return false;
	}

	bool Success = true;

	std::string Line;
	int LineCounter = 0;

	while (std::getline(Input, Line))
	{
		LineCounter++;

		Trim(Line);

		// Skip blank lines and comments
		if (Line.empty() || Line[0] == '#' || Line[0] == '|')
		{
			continue;
		}

		// Remove comments
		std::string::size_type Pos = Line.find('#');

		if (Pos != std::string::npos)
		{
			Line.erase(Pos);
		}

		std::vector<std::string> Tokens;

		ParseLine(Line, Tokens);

		if (Tokens.size() == 3 && StrCaseCmp("ADDMAP", Tokens[0].c_str()) == 0)
		{
			try
			{
				EconetNet Network;

				Network.inet_addr  = ParseIPAddress("IP address", Tokens[1]) & 0x00FFFFFF;
				Network.network    = (unsigned char)ParseNumber("Network", Tokens[2], 0, 255);
				Network.port       = DEFAULT_AUN_PORT; // always use the default port for proper AUN networks
				Network.broadcasts = BroadcastSource::Unknown;

				#ifdef DEBUG_ECONET
				DebugTrace("Econet: AUNMap Net %d IP %s:%u\n",
				           Network.network, IpAddressStr(Network.inet_addr).c_str(), Network.port);
				#endif

				Networks.emplace_back(Network);
			}
			catch (const std::exception& e)
			{
				EconetError("Invalid value in AUNMap file\n%s\n%s (Line %d)", e.what(), AUNMapPath, LineCounter);
				Success = false;
				break;
			}
		}
	}

	return Success;
}

//---------------------------------------------------------------------------

static bool ReadNetwork()
{
	EconetFlagFillTimeout = DEFAULT_FLAG_FILL_TIMEOUT;
	EconetScoutAckTimeout = DEFAULT_SCOUT_ACK_TIMEOUT;
	TimeBetweenBytes = DEFAULT_TIME_BETWEEN_BYTES;
	FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
	MassageNetworks = DEFAULT_MASSAGE_NETWORKS;
	AutoConfigure = DEFAULT_AUTOCONFIGURE;
	FindGateways = DEFAULT_FINDGATEWAYS;

	// Clear tables.
	Stations.clear();
	Networks.clear();

	// Clear the gateway address.
	Gateway.inet_addr = 0;
	Gateway.port = 0;

	if (!ReadEconetConfigFile())
	{
		return false;
	}

	return ReadAUNConfigFile();
}

//---------------------------------------------------------------------------

// Read from address FE18.

unsigned char EconetReadStationID()
{
	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet,
		                   true,
		                   "Econet: Read Station: %d",
		                   (int)EconetStationID);
	}

	return EconetStationID;
}

//---------------------------------------------------------------------------

// Read from address FEA0-3.

unsigned char EconetRead(unsigned char Register)
{
	unsigned char Value;

	if (Register == 0)
	{
		Value = ADLC.Status1;
	}
	else if (Register == 1)
	{
		Value = ADLC.Status2;
	}
	else
	{
		// RxReset not set and something in FIFO.
		if (((ADLC.Control1 & CONTROL_REG1_RX_RESET) == 0) && ADLC.RxFifoPtr > 0)
		{
			Value = ADLC.RxFifo[--ADLC.RxFifoPtr]; // Read RX buffer.

			EconetStateChanged = true;
		}
		else
		{
			Value = 0;
		}
	}

	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet,
		                   true,
		                   "Econet: Read ADLC register %02X, value %02X",
		                   (int)Register, (int)Value);
	}

	return Value;
}

//---------------------------------------------------------------------------

// Write to address FEA0-3.

void EconetWrite(unsigned char Register, unsigned char Value)
{
	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet,
		                   true,
		                   "Econet: Write ADLC %02X = %02X",
		                   (int)Register, (int)Value);
	}

	// Command registers are really just a set of flags that affect
	// operation of the rest of the device.

	if (Register == 0)
	{
		ADLC.Control1 = Value;
	}
	else if (Register == 1 && !(ADLC.Control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.Control2 = Value;
	}
	else if (Register == 1 && (ADLC.Control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.Control3 = Value;
	}
	else if (Register == 3 && (ADLC.Control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.Control4 = Value;
	}
	else if (Register == 2 || Register == 3) // adr 02 or adr 03 & AC=0
	{
		// Cannot write an output byte if TxReset is set.
		// register 2 is an output byte
		// register 3 with c1b0=0 is output byte & finalise tx.
		// can also finalise tx by setting a control bit.so do that automatically for reg 3
		// worry about actually sending stuff in the poll routines, not here.
		if ((ADLC.Control1 & CONTROL_REG1_TX_RESET) == 0)
		{
			ADLC.TxFifo[2] = ADLC.TxFifo[1];
			ADLC.TxFifo[1] = ADLC.TxFifo[0];
			ADLC.TxFifo[0] = Value;
			ADLC.TxFifoPtr++;
			ADLC.TxFifoTxLast <<= 1; // Shift TxLast bits up.

			if (Register == 3)
			{
				ADLC.Control2 |= CONTROL_REG2_TX_LAST_DATA; // Set TxLast control flag ourself.
			}
		}
	}

	EconetStateChanged = true;
}

//--------------------------------------------------------------------------------------------

bool EconetInterruptRequest()
{
	return (ADLC.Status1 & STATUS_REG1_IRQ) != 0;
}

//--------------------------------------------------------------------------------------------

// Returns NMI status.

bool EconetPoll()
{
	if (EconetStateChanged || EconetTrigger <= TotalCycles)
	{
		EconetStateChanged = false;

		// Don't poll if failed to init sockets
		if (Socket != INVALID_SOCKET)
		{
			// Optimisation - only call real poll routine when something has changed.
			return EconetPollReal();
		}
	}

	return false;
}

//--------------------------------------------------------------------------------------------

// Run when state changed or time to check comms.
// The majority of this code is to handle the status registers.
// These are just flags that depend on the TX and RX status, and the control flags.
// These change immediately anything happens, so need refreshing all the time,
// as RX and TX operations can depend on them too. It /might/ be possible to
// only re-calculate them when needed (e.g., on a memory read or in the receive
// routines before they are checked) but for the moment I just want to get this
// code actually working!

bool EconetPollReal()
{
	bool Interrupt = false;

	// Save flags.
	unsigned char PrevStatus1 = ADLC.Status1;
	unsigned char PrevStatus2 = ADLC.Status2;

	// okie dokie. This is where the brunt of the ADLC emulation & network handling will happen.

	// Look for control bit changes and take appropriate action.

	// CR1b0 - Address Control - only used to select between register 2/3/4
	//         No action needed here.
	// CR1b1 - RIE - Receiver Interrupt Enable - Flag to allow receiver section to create interrupt.
	//         No action needed here.
	// CR1b2 - TIE - Transmitter Interrupt Enable - ditto
	//         No action needed here.
	// CR1b3 - RDSR mode. When set, interrupts on received data are inhibited.
	//         Unsupported - no action needed here
	// CR1b4 - TDSR mode. When set, interrupts on transmit data are inhibited.
	//         Unsupported - no action needed here
	// CR1b5 - Discontinue - when set, discontinue reception of incoming data.
	//         Automatically reset this when reach the end of current frame in progress.
	//         Automatically reset when frame aborted by receiving an abort flag, or DCD fails.
	if (ADLC.Control1 & CONTROL_REG1_RX_FRAME_DISCONTINUE)
	{
		BeebRx.Pointer = 0;
		BeebRx.BytesInBuffer = 0;

		ADLC.RxFifoPtr = 0;
		ADLC.RxFifoAPFlags = 0;
		ADLC.RxFifoFCFlags = 0;

		ADLC.Control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE;

		AUNState = FourWayStage::Idle;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Set FourWayStage::Idle (RxABORT is set)\n");
		#endif
	}

	// CR1b6 - RxRs - Receiver reset. set by cpu or when reset line goes low.
	//         all receive operations blocked (bar DCD monitoring) when this is set.
	//         see CR2b5
	// CR1b7 - TxRS - Transmitter reset. set by CPU or when reset line goes low.
	//         all transmit operations blocked (bar CTS monitoring) when this is set.
	//         no action needed here; watch this bit elsewhere to inhibit actions

	// CR2b0 - PSE - prioritised status enable - adjusts how status bits show up.
	//         See PriorityStatus and code in status section
	// CR2b1 - 2byte/1byte mode. Set to indicate 2 byte mode. See TDRA status bit.
	// CR2b2 - Flag/Mark idle select. What is transmitted when TX idle.
	//         Ignored here as not needed.
	// CR2b3 - FC/TDRA mode - does status bit SR1b6 indicate 1=Frame Complete,
	//         0=TX Data Reg Available. See TDRA status bit.
	// CR2b4 - TxLast - byte just put into FIFO was the last byte of a packet.
	if (ADLC.Control2 & CONTROL_REG2_TX_LAST_DATA)
	{
		ADLC.TxFifoTxLast |= 1; // set b0 - flag for fifo[0]
		ADLC.Control2 &= ~CONTROL_REG2_TX_LAST_DATA;
	}

	// CR2b5 - CLR RxST - Clear Receiver Status - reset status bits
	if ((ADLC.Control2 & CONTROL_REG2_CLEAR_RX_STATUS) || (ADLC.Control1 & CONTROL_REG1_RX_RESET)) // or RxReset
	{
		ADLC.Control2 &= ~CONTROL_REG2_CLEAR_RX_STATUS;

		ADLC.Status1 &= ~(STATUS_REG1_STATUS2_READ_REQUEST | STATUS_REG1_FLAG_DETECTED);

		// Clear FV, RxIdle, RxAbt, Err, OVRN, DCD.
		ADLC.Status2 &= ~(STATUS_REG2_FRAME_VALID |
		                  STATUS_REG2_INACTIVE_IDLE_RECEIVED |
		                  STATUS_REG2_ABORT_RECEIVED |
		                  STATUS_REG2_FCS_ERROR |
		                  STATUS_REG2_DCD |
		                  STATUS_REG2_RX_OVERRUN);

		// If PSE is active, advance to the next priority.
		if ((ADLC.Control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE) && ADLC.PriorityStatus > 0)
		{
			ADLC.PriorityStatus++;

			if (ADLC.PriorityStatus > 4)
			{
				ADLC.PriorityStatus = 0;
			}
		}
		else
		{
			ADLC.PriorityStatus = 0;
		}

		S2RQCause = 0; // Clear cause of SR2 bit 1 (S2RQ) going up.

		// Clear buffers on RX reset.
		if (ADLC.Control1 & CONTROL_REG1_RX_RESET)
		{
			BeebRx.Pointer = 0;
			BeebRx.BytesInBuffer = 0;

			ADLC.RxFifoPtr = 0;
			ADLC.RxFifoAPFlags = 0;
			ADLC.RxFifoFCFlags = 0;
			ADLC.PriorityStatus = 0;
		}

		// AUNState = FourWayStage::Idle; // this really doesn't like being here.
	}

	// CR2b6 - CLT TxST - Clear Transmitter Status - reset status bits
	if ((ADLC.Control2 & CONTROL_REG2_CLEAR_TX_STATUS) || (ADLC.Control1 & CONTROL_REG1_TX_RESET)) // or TxReset
	{
		ADLC.Control2 &= ~CONTROL_REG2_CLEAR_TX_STATUS;

		ADLC.Status1 &= ~(STATUS_REG1_CTS |
		                  STATUS_REG1_TX_UNDERRUN |
		                  STATUS_REG1_TDRA);

		if (ADLC.CTS)
		{
			ADLC.Status1 |= STATUS_REG1_CTS; // CTS follows signal, reset high again.
			PrevStatus1 |= STATUS_REG1_CTS; // Don't trigger another interrupt instantly.
		}

		// Clear buffers on TX reset.
		if (ADLC.Control1 & CONTROL_REG1_TX_RESET)
		{
			BeebTx.Pointer = 0;
			BeebTx.BytesInBuffer = 0;

			ADLC.TxFifoPtr = 0;
			ADLC.TxFifoTxLast = 0;
		}
	}

	// CR2b7 - RTS control - looks after RTS output line. ignored here.
	// but used in CTS logic
	// RTS gates TXD onto the econet bus. if not zero, no tx reaches it,
	// in the B+, RTS substitutes for the collision detection circuit.

	// CR3 seems always to be all zero while debugging emulation.
	// CR3b0 - LCF - Logical Control Field Select. if zero, no control fields in frame, ignored.
	// CR3b1 - CEX - Extend Control Field Select - when set, control field is 16 bits. ignored.
	// CR3b2 - AEX - When set, address will be two bytes (unless first byte is zero). ignored here.
	// CR3b3 - 01/11 idle - idle transmission mode - ignored here.
	// CR3b4 - FDSE - Flag Detect Status Enable. When set, then FD (SR1b3) + interrupt indicated a flag
	// has been received. I don't think we use this mode, so ignoring it.
	// CR3b5 - Loop - Loop mode. Not used.
	// CR3b6 - GAP/TST - Sets test loopback mode (when not in Loop operation mode). ignored.
	// CR3b7 - LOC/DTR - (when not in loop mode) controls DTR pin directly. Pin not used in a BBC B

	// CR4b0 - FF/F - When clear, re-used the Flag at end of one packet as start of next packet. ignored.
	// CR4b1,2 - TX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b3,4 - RX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b5 - TransmitABT - Abort Transmission.  Once abort starts, bit is cleared.
	if (ADLC.Control4 & CONTROL_REG4_TX_ABORT)
	{
		ADLC.TxFifoPtr = 0; // reset FIFO
		ADLC.TxFifoTxLast = 0; // reset FIFO flags
		ADLC.Control4 &= ~CONTROL_REG4_TX_ABORT; // reset flag.

		BeebTx.Pointer = 0;
		BeebTx.BytesInBuffer = 0;

		AUNState = FourWayStage::Idle;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Set FourWayStage::Idle (TxABORT is set)");
		#endif
	}

	// CR4b6 - ABTex - extend abort - adjust way the abort flag is sent.  ignore.
	// Can affect timing of RTS output line (and thus CTS input) still ignored.
	// CR4b7 - NRZI/NRZ - invert data encoding on wire. ignore.

	if (TotalCycles >= EconetTrigger)
	{
		// Only do this bit occasionally as data only comes in from
		// line occasionally.
		// Trickle data between FIFO registers and IP packets.

		// Transmit data.
		if (!(ADLC.Control1 & CONTROL_REG1_TX_RESET))
		{
			if (ADLC.TxFifoPtr > 0) // There is data in the transmit FIFO.
			{
				#ifdef DEBUG_ECONET_ADLC_FIFO
				DebugTrace("EconetPoll: Write to FIFO: %02X %c\n",
				           ADLC.TxFifo[ADLC.TxFifoPtr - 1],
				           isprint(ADLC.TxFifo[ADLC.TxFifoPtr - 1]) ? ADLC.TxFifo[ADLC.TxFifoPtr - 1] : '.');
				#endif

				bool TxLast = false;

				if (ADLC.TxFifoTxLast & powers[ADLC.TxFifoPtr - 1]) // TxLast set
				{
					TxLast = true;
				}

				if (BeebTx.Pointer >= sizeof(BeebTx.Buffer) || // Overflow in IP buffer
				    (ADLC.TxFifoPtr > 4)) // Overflowed FIFO
				{
					ADLC.Status1 |= STATUS_REG1_TX_UNDERRUN;

					// Wipe buffer
					BeebTx.Pointer = 0;
					BeebTx.BytesInBuffer = 0;

					ADLC.TxFifoPtr = 0;
					ADLC.TxFifoTxLast = 0;

					#ifdef DEBUG_ECONET
					DebugTrace("EconetPoll: TxUnderrun!\n");
					#endif
				}
				else
				{
					BeebTx.Buffer[BeebTx.Pointer] = ADLC.TxFifo[--ADLC.TxFifoPtr];
					BeebTx.Pointer++;
				}

				if (TxLast)
				{
					EconetSendPacket();
				}
			}
		}

		// Receive data.
		if (!(ADLC.Control1 & CONTROL_REG1_RX_RESET)) // RX reset off
		{
			if (BeebRx.Pointer < BeebRx.BytesInBuffer)
			{
				// There's something waiting to be given to the processor.
				if (ADLC.RxFifoPtr < 3) // space in FIFO
				{
					#ifdef DEBUG_ECONET_ADLC_FIFO
					DebugTrace("EconetPoll: Send received byte to the Beeb: %02X %c\n",
					           BeebRx.Buffer[BeebRx.Pointer],
					           isprint(BeebRx.Buffer[BeebRx.Pointer]) ? BeebRx.Buffer[BeebRx.Pointer] : '.');
					#endif

					ADLC.RxFifo[2] = ADLC.RxFifo[1];
					ADLC.RxFifo[1] = ADLC.RxFifo[0];
					ADLC.RxFifo[0] = BeebRx.Buffer[BeebRx.Pointer];
					ADLC.RxFifoPtr++;
					ADLC.RxFifoFCFlags = (ADLC.RxFifoFCFlags << 1) & 7;
					ADLC.RxFifoAPFlags = (ADLC.RxFifoAPFlags << 1) & 7;

					if (BeebRx.Pointer == 0)
					{
						ADLC.RxFifoAPFlags |= 1; // 2 bytes? adr extension mode
					}

					if (++BeebRx.Pointer >= BeebRx.BytesInBuffer) // that was last byte!
					{
						// Set Frame Valid flag (this was last byte of frame).
						ADLC.RxFifoFCFlags |= 1;

						// Reset read for next packet.
						BeebRx.Pointer = 0;
						BeebRx.BytesInBuffer = 0;
					}
				}
			}

			if (ADLC.RxFifoPtr == 0)
			{
				// Still nothing in buffers (and thus nothing in EconetRx buffer).
				ADLC.Control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE;

				// Wait for CPU to clear Frame Valid flag from last frame received.
				if (!(ADLC.Status2 & STATUS_REG2_FRAME_VALID))
				{
					if (!EconetReceivePacket())
					{
						return false;
					}
				}
			}
		}

		// Update idle status
		if (!(ADLC.Control1 & CONTROL_REG1_RX_RESET) && // Not RxReset
		    ADLC.RxFifoPtr == 0 && // Nothing in FIFO
		    !(ADLC.Status2 & STATUS_REG2_FRAME_VALID) && // No FV
		    BeebRx.BytesInBuffer == 0) // Nothing in IP buffer
		{
			ADLC.Idle = true;
		}
		else
		{
			ADLC.Idle = false;
		}

		// How long before we come back in here?
		SetTrigger(TimeBetweenBytes, EconetTrigger);
	}

	// Reset pseudo flag fill?
	if (FlagFillActive && TotalCycles > EconetFlagFillTimeoutTrigger)
	{
		FlagFillActive = false;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: FlagFill timeout reset\n");
		#endif
	}

	// Waiting for AUN to become idle?
	if (AUNState == FourWayStage::WaitForIdle &&
	    BeebRx.BytesInBuffer == 0 &&
	    ADLC.RxFifoPtr == 0 &&
	    ADLC.TxFifoPtr == 0 // ??
	    // && EconetScoutAckTrigger > TotalCycles
	    )
	{
		AUNState = FourWayStage::Idle;
		EconetFourWayTrigger = 0;
		EconetScoutAckTrigger = 0;
		FlagFillActive = false;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Set FourWayStage::Idle (Rx FIFO empty)\n");
		#endif
	}

	// Timeout four way handshake - for when we get lost.
	if (EconetFourWayTrigger == 0)
	{
		if (AUNState != FourWayStage::Idle)
		{
			SetTrigger(FourWayStageTimeout, EconetFourWayTrigger);
		}
	}
	else if (TotalCycles >= EconetFourWayTrigger)
	{
		EconetScoutAckTrigger = 0;
		EconetFourWayTrigger = 0;
		AUNState = FourWayStage::Idle;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Set FourWayStage::Idle (FourWayStage timeout)\n");
		#endif
	}

	// Send Gateway keepalive if timeout value has been passed.
	if (Gateway.port != 0 && time(NULL) >= GatewayTimeout)
	{
		sockaddr_in RecvAddr;
		RecvAddr.sin_family = AF_INET;

		// Construct an extended AUN packet on EconetTemp.
		ExtendedAUNPacket *KeepalivePacket = (ExtendedAUNPacket*)&EconetTemp;
		KeepalivePacket->EconetHeader.DestStn = 255;
		KeepalivePacket->EconetHeader.DestNet = 255;
		KeepalivePacket->EconetHeader.SrcStn = 0; // source (left blank)
		KeepalivePacket->EconetHeader.SrcNet = 0;
		KeepalivePacket->AUNHeader.Type = AUNType::Broadcast;
		KeepalivePacket->AUNHeader.Port = 0x9c; // bridge port
		KeepalivePacket->AUNHeader.CtrlByte = (0xd0 & 0x7f); // reuse trunk keepalive
		KeepalivePacket->AUNHeader.Pad = 0;
		KeepalivePacket->AUNHeader.Handle = 0;
		ZeroMemory(KeepalivePacket->Buffer, 8); // a broadcast has 8 data bytes
		int KeepaliveLen = 20; // total size to send is 4 + 8 + 8

		#ifdef DEBUG_TRACE
		DebugTrace("Econet: Sending gateway keepalive\n");
		#endif

		RecvAddr.sin_addr.s_addr = Gateway.inet_addr;
		RecvAddr.sin_port = htons(Gateway.port);

		if (sendto(Socket, (const char*)KeepalivePacket, KeepaliveLen, 0,
		           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
		{
			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Failed to send Gateway keepalive (%s port %u)\n",
			           IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
			           (unsigned int)htons(RecvAddr.sin_port));
			#endif
		}

		// Set timeout again.
		GatewayTimeout = time(NULL) + DEFAULT_GATEWAY_TIMEOUT;
	}

	// Send host announce pings if timeout value has been passed.
	if (AutoConfigure && time(NULL) > AnnounceTimeout)
	{
		// Announce our address other BeebEm instances by pinging the network
		// this uses packets conforming to the structure of AUN, but with a
		// proprietary type which will hopefully be ignored by any existing
		// AUN code.
		sockaddr_in RecvAddr;
		RecvAddr.sin_family = AF_INET;
		RecvAddr.sin_addr.s_addr = INADDR_BROADCAST;
		RecvAddr.sin_port = htons(DEFAULT_AUN_PORT);

		EconetTemp.AUNHeader.Type = AUNType::BeebEm;
		EconetTemp.AUNHeader.CtrlByte = 0x1f; // control &9f a discovery Ping
		EconetTemp.AUNHeader.Port = BEEBEM_ECONET_PORT; // BeebEm reply port
		EconetTemp.AUNHeader.Pad = 0;
		EconetTemp.AUNHeader.Handle = AnnounceHandle++; // sequence number
		ZeroMemory(EconetTemp.Buffer, 8);
		EconetTemp.Buffer[0] = EconetStationID;
		EconetTemp.Buffer[1] = EconetNetworkID;

		if (sendto(Socket, (char *)&EconetTemp, sizeof(EconetTemp.AUNHeader) + 8, 0,
		           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
		{
			EconetError("Econet: Failed to send BeebEm ping");
		}

		// Set timeout again.
		AnnounceTimeout = time(NULL) + ANNOUNCE_TIMEOUT;
	}

	// Status bits need changing?

	// SR1b0 - RDA - received data available.
	if (!(ADLC.Control1 & CONTROL_REG1_RX_RESET)) // rx reset off
	{
		if ((ADLC.RxFifoPtr > 0 && !(ADLC.Control2 & CONTROL_REG2_2_BYTE_TRANSFER)) || // 1 byte mode
		    (ADLC.RxFifoPtr > 1 &&  (ADLC.Control2 & CONTROL_REG2_2_BYTE_TRANSFER))) // 2 byte mode
		{
			ADLC.Status1 |= STATUS_REG1_RX_DATA_AVAILABLE; // set RDA copy
			ADLC.Status2 |= STATUS_REG2_RX_DATA_AVAILABLE;
		}
		else
		{
			ADLC.Status1 &= ~STATUS_REG1_RX_DATA_AVAILABLE;
			ADLC.Status2 &= ~STATUS_REG2_RX_DATA_AVAILABLE;
		}
	}

	// SR1b1 - S2RQ - set after SR2, see below
	// SR1b2 - LOOP - set if in loop mode. not supported in this emulation
	// SR1b3 - FD - Flag detected. Hmm.
	if (FlagFillActive)
	{
		ADLC.Status1 |= STATUS_REG1_FLAG_DETECTED;
	}
	else
	{
		ADLC.Status1 &= ~STATUS_REG1_FLAG_DETECTED;
	}

	// SR1b4 - CTS - Set by ~CTS line going up, and causes IRQ if enabled.
	//               Only cleared by CPU.
	//               ~CTS is a NAND of DCD(clock present)(high if valid)
	//               & collision detection!
	//               i.e. it's low (thus clear to send) when we have both DCD(clock)
	//               present AND no collision on line and no collision.
	//               CTS will ALSO be high if there is no cable!
	// We will only bother checking against DCD here as can't have collisions.
	// but NFS then loops waiting for CTS high!
	// On the B+ there is (by default) no collision detection circuitry. Instead S29
	// links RTS in its place, thus CTS is a NAND of not RTS & not DCD
	// i.e. CTS = !(!RTS && !DCD) All signals are active low.
	// There is a delay on RTS going high after cr2b7=0 - ignore this for now.
	// cr2b7 = 1 means RTS low means not RTS high means CTS low
	// sockets true means DCD low means not DCD high means CTS low
	// doing it this way finally works !!  great :-) :-)

	if (Socket != INVALID_SOCKET && (ADLC.Control2 & CONTROL_REG2_RTS_CONTROL)) // clock + RTS
	{
		ADLC.CTS = false;
		ADLC.Status1 &= ~STATUS_REG1_CTS;
	}
	else
	{
		ADLC.CTS = true;
	}

	// And then set the status bit if the line is high! (status bit stays
	// up until the CPU tries to clear it) (and still stays up if the CTS
	// line is still high)

	if (!(ADLC.Control1 & CONTROL_REG1_RX_RESET) && ADLC.CTS)
	{
		ADLC.Status1 |= STATUS_REG1_CTS; // set CTS now
	}

	// SR1b5 - TXU - Tx Underrun.
	if (ADLC.TxFifoPtr > 4) // probably not needed
	{
		#ifdef DEBUG_ECONET
		DebugTrace("Econet: TX Underrun - TXfptr %02x\n", (unsigned int)ADLC.TxFifoPtr);
		#endif

		ADLC.Status1 |= STATUS_REG1_TX_UNDERRUN;
		ADLC.TxFifoPtr = 4;
	}

	// SR1b6 TDRA flag - another complicated derivation
	if (!(ADLC.Control1 & CONTROL_REG1_TX_RESET)) // not TxReset
	{
		if (!(ADLC.Control2 & CONTROL_REG2_TDRA_SELECT)) // TDRA mode
		{
			if (   (   ((ADLC.TxFifoPtr < 3) && !(ADLC.Control2 & CONTROL_REG2_2_BYTE_TRANSFER)) // space in FIFO?
			        || ((ADLC.TxFifoPtr < 2) && (ADLC.Control2 & CONTROL_REG2_2_BYTE_TRANSFER))) // space in FIFO?
			    && (!(ADLC.Status1 & STATUS_REG1_CTS)) // Clear to send is ok
			    && (!(ADLC.Status2 & STATUS_REG2_DCD)) ) // DTR not high
			{
				#ifdef DEBUG_ECONET_ADLC
				if (!(ADLC.Status1 & STATUS_REG1_TDRA))
				{
					DebugTrace("ADLC: Set TDRA\n");
				}
				#endif

				ADLC.Status1 |= STATUS_REG1_TDRA;
			}
			else
			{
				#ifdef DEBUG_ECONET_ADLC
				if (ADLC.Status1 & STATUS_REG1_TDRA)
				{
					DebugTrace("ADLC: Clear TDRA\n");
				}
				#endif

				ADLC.Status1 &= ~STATUS_REG1_TDRA;
			}
		}
		else // FC mode
		{
			if (ADLC.TxFifoPtr == 0) // Nothing in FIFO.
			{
				#ifdef DEBUG_ECONET_ADLC
				if (!(ADLC.Status1 & STATUS_REG1_TDRA))
				{
					DebugTrace("ADLC: Set FC\n");
				}
				#endif

				ADLC.Status1 |= STATUS_REG1_TDRA;
			}
			else
			{
				#ifdef DEBUG_ECONET_ADLC
				if (ADLC.Status1 & STATUS_REG1_TDRA)
				{
					DebugTrace("ADLC: Clear FC\n");
				}
				#endif

				ADLC.Status1 &= ~STATUS_REG1_TDRA;
			}
		}
	}
	// SR1b7 IRQ flag - see below

	// SR2b0 - AP - Address Present
	if (!(ADLC.Control1 & CONTROL_REG1_RX_RESET))
	{
		if (ADLC.RxFifoPtr > 0 &&
		    (ADLC.RxFifoAPFlags & (powers[ADLC.RxFifoPtr - 1]))) // AP bits set on FIFO
		{
			ADLC.Status2 |= STATUS_REG2_ADDRESS_PRESENT;
		}
		else
		{
			ADLC.Status2 &= ~STATUS_REG2_ADDRESS_PRESENT;
		}

		// SR2b1 - FV - Frame Valid - set in RX - only reset by ClearRx or RxReset
		if (ADLC.RxFifoPtr > 0 &&
		    (ADLC.RxFifoFCFlags & (powers[ADLC.RxFifoPtr - 1])))
		{
			ADLC.Status2 |= STATUS_REG2_FRAME_VALID;
		}

		// SR2b2 - Inactive Idle Received - sets IRQ!
		if (ADLC.Idle && !FlagFillActive)
		{
			ADLC.Status2 |= STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
		else
		{
			ADLC.Status2 &= ~STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
	}

	// SR2b3 - RxAbort - Abort received - set in RX routines above
	// SR2b4 - Error during reception - set if error flagged in RX routine.
	// SR2b5 - DCD
	if (Socket == INVALID_SOCKET) // is line down?
	{
		ADLC.Status2 |= STATUS_REG2_DCD; // Flag error
	}
	else
	{
		ADLC.Status2 &= ~STATUS_REG2_DCD;
	}

	// SR2b6 - OVRN - Receipt Overrun. Probably not needed.
	if (ADLC.RxFifoPtr > 4)
	{
		ADLC.Status2 |= STATUS_REG2_RX_OVERRUN;
		ADLC.RxFifoPtr = 4;
	}

	// SR2b7 - RDA. As per SR1b0 - set above.

	// Handle PSE - only for SR2 Rx bits at the moment.

	#ifdef DEBUG_ECONET_ADLC
	int PrevPriorityStatus = ADLC.PriorityStatus;
	#endif

	if (ADLC.Control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE)
	{
		if (ADLC.PriorityStatus <= 1 && (ADLC.Status2 & (STATUS_REG2_FRAME_VALID |
		                                                 STATUS_REG2_ABORT_RECEIVED |
		                                                 STATUS_REG2_FCS_ERROR |
		                                                 STATUS_REG2_DCD |
		                                                 STATUS_REG2_RX_OVERRUN)))
		{
			ADLC.PriorityStatus = 1;
			ADLC.Status2 &= ~(STATUS_REG2_ADDRESS_PRESENT |
			                  STATUS_REG2_INACTIVE_IDLE_RECEIVED |
			                  STATUS_REG2_RX_DATA_AVAILABLE);
		}
		else if (ADLC.PriorityStatus <= 2 && (ADLC.Status2 & STATUS_REG2_INACTIVE_IDLE_RECEIVED)) // Idle
		{
			ADLC.PriorityStatus = 2;
			ADLC.Status2 &= ~(STATUS_REG2_ADDRESS_PRESENT |
			                  STATUS_REG2_RX_DATA_AVAILABLE);
		}
		else if (ADLC.PriorityStatus <= 3 && (ADLC.Status2 & STATUS_REG2_ADDRESS_PRESENT))
		{
			ADLC.PriorityStatus = 3;
			ADLC.Status2 &= ~STATUS_REG2_RX_DATA_AVAILABLE;
		}
		else if (ADLC.Status2 & STATUS_REG2_RX_DATA_AVAILABLE)
		{
			ADLC.PriorityStatus = 4;
			ADLC.Status2 &= ~STATUS_REG2_FRAME_VALID;
		}
		else
		{
			ADLC.PriorityStatus = 0; // No relevant bits set.
		}

		// Set SR1 RDA copy.
		if (ADLC.Status2 & STATUS_REG2_RX_DATA_AVAILABLE)
		{
			ADLC.Status1 |= STATUS_REG1_RX_DATA_AVAILABLE;
		}
		else
		{
			ADLC.Status1 &= ~STATUS_REG1_RX_DATA_AVAILABLE;
		}
	}
	else
	{
		// PSE inactive.
		ADLC.PriorityStatus = 0;
	}

	#ifdef DEBUG_ECONET_ADLC
	if (ADLC.PriorityStatus != PrevPriorityStatus)
	{
		DebugTrace("ADLC: PSE SR2Rx priority changed to %d\n", ADLC.PriorityStatus);
	}
	#endif

	// Do we need to flag an interrupt?
	if (ADLC.Status1 != PrevStatus1 || ADLC.Status2 != PrevStatus2) // Something changed.
	{
		// SR1b1 - S2RQ - Status2 request. New bit set in S2?
		unsigned char TempCause = ((ADLC.Status2 ^ PrevStatus2) & ADLC.Status2) & ~STATUS_REG2_RX_DATA_AVAILABLE;

		if (!(ADLC.Control1 & CONTROL_REG1_RX_INT_ENABLE))
		{
			TempCause = 0;
		}

		if (TempCause) // Something got set.
		{
			ADLC.Status1 |= STATUS_REG1_STATUS2_READ_REQUEST;
			S2RQCause |= TempCause;
		}
		else if (!(ADLC.Status2 & S2RQCause)) // Cause has gone.
		{
			ADLC.Status1 &= ~STATUS_REG1_STATUS2_READ_REQUEST;
			S2RQCause = 0;
		}

		// New bit set in S1?
		TempCause = ((ADLC.Status1 ^ PrevStatus1) & ADLC.Status1) & ~STATUS_REG1_IRQ;

		if (!(ADLC.Control1 & CONTROL_REG1_RX_INT_ENABLE))
		{
			TempCause &= ~(STATUS_REG1_RX_DATA_AVAILABLE |
			               STATUS_REG1_STATUS2_READ_REQUEST |
			               STATUS_REG1_FLAG_DETECTED);
		}

		if (!(ADLC.Control1 & CONTROL_REG1_TX_INT_ENABLE))
		{
			TempCause &= ~(STATUS_REG1_CTS |
			               STATUS_REG1_TX_UNDERRUN |
			               STATUS_REG1_TDRA);
		}

		if (TempCause != 0) // Something got set.
		{
			Interrupt = true;
			IRQCause |= TempCause; // Remember which bit went high to flag IRQ.

			ADLC.Status1 |= STATUS_REG1_IRQ;

			#ifdef DEBUG_ECONET_ADLC
			DebugTrace("ADLC: Status1 bit got set %02x, interrupt\n", (int)TempCause);
			#endif
		}

		// Bit cleared in S1?
		TempCause = ((ADLC.Status1 ^ PrevStatus1) & PrevStatus1) & ~STATUS_REG1_IRQ;

		if (TempCause != 0) // Something went off.
		{
			IRQCause &= ~TempCause; // Clear flags that went off.

			if (IRQCause == 0) // All flags gone off now.
			{
				// Clear IRQ status bit when cause has gone.
				ADLC.Status1 &= ~STATUS_REG1_IRQ;
			}
			else
			{
				// Interrupt again because we still have flags set.
				if (ADLC.Control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE)
				{
					Interrupt = true;

					#ifdef DEBUG_ECONET_ADLC
					DebugTrace("ADLC: S1 flags still set, interrupt\n");
					#endif
				}
			}

			#ifdef DEBUG_ECONET_ADLC
			DebugTrace("ADLC: IRQ cause reset, irqcause %02x\n", (int)IRQCause);
			#endif
		}
	}

	// Flag NMI if necessary. See also INTON flag as
	// this can cause a delayed interrupt (BeebMem.cpp).
	return Interrupt;
}

//--------------------------------------------------------------------------------------------

// BeebTx.Buffer contains the data to send, length BeebTx.Pointer

static void EconetSendPacket()
{
	// Translate destination network for local packets.
	if (BeebTx.EconetHeader.DestNet == 0)
	{
		BeebTx.EconetHeader.DestNet = EconetNetworkID;
	}

	sockaddr_in RecvAddr;
	ZeroMemory(&RecvAddr, sizeof(RecvAddr));
	RecvAddr.sin_family = AF_INET;

	bool SendMe = false;
	bool ExtendedAUN = false;
	int SendLen = 0;

	// First two bytes of BeebTx.Buffer contain the destination address
	// (or one zero byte for broadcast).

	if (IsBroadcastStation(BeebTx.EconetHeader.DestStn))
	{
		// Set address to the local broadcast address and port to the default AUN port
		// to do an AUN broadcast. Some BeebEm instances might not see this so we will
		// send unicast copies to those that need them later.
		RecvAddr.sin_addr.s_addr = INADDR_BROADCAST; // ((EconetListenIP & 0x00FFFFFF) | 0xFF000000);
		RecvAddr.sin_port = htons(DEFAULT_AUN_PORT);
		SendMe = true;
	}

	// Match AUN nets for Econet network numbers if MassageNetworks is enabled.
	const unsigned int mask = MassageNetworks ? 0x7F : 0xFF;

	if (!SendMe)
	{
		// Search for the destination host in the Stations table.

		for (size_t i = 0; i < Stations.size(); i++)
		{
			const EconetHost& Station = Stations[i];

			if ((Station.network & mask) == (BeebTx.EconetHeader.DestNet & mask) &&
			    Station.station == BeebTx.EconetHeader.DestStn)
			{
				RecvAddr.sin_addr.s_addr = Station.inet_addr;
				RecvAddr.sin_port = htons(Station.port);
				SendMe = true;
				break;
			}
		}
	}

	if (!SendMe)
	{
		// Station not found. Search to see if the destination network
		// is defined in the Networks table.

		for (size_t i = 0; i < Networks.size(); i++)
		{
			const EconetNet& Network = Networks[i];

			if ((Network.network & mask) == (BeebTx.EconetHeader.DestNet & mask))
			{
				// Located the network.
				if ((Network.inet_addr & 0xFF000000) == 0)
				{
					// Last octet is zero so this is true AUN.
					RecvAddr.sin_addr.s_addr = (Network.inet_addr & 0x00FFFFFF) | (BeebTx.EconetHeader.DestStn << 24);
					RecvAddr.sin_port = htons(Network.port); // TODO this should always be DEFAULT_AUN_PORT - should we override this?
					SendMe = true;
					break;
				}
				else
				{
					// Whole network defined with a single address.
					// Treat port as the base port number for a PiEconetBridge exposed network.
					RecvAddr.sin_addr.s_addr = Network.inet_addr;
					RecvAddr.sin_port = htons(Network.port + BeebTx.EconetHeader.DestStn);
					SendMe = true;
					break;
				}
			}
		}
	}

	if (!SendMe)
	{
		// Network not found in the Networks table. If the packet is not for
		// net 0 or 255, use the gateway to get packets to this network.

		if (BeebTx.EconetHeader.DestNet != 0 && BeebTx.EconetHeader.DestNet != 255)
		{
			if (Gateway.port != 0)
			{
				RecvAddr.sin_addr.s_addr = Gateway.inet_addr;
				RecvAddr.sin_port = htons(Gateway.port);

				// We need to send Extended AUN to this port.
				ExtendedAUN = true;
				SendMe = true;
			}
		}
	}

	if (!SendMe)
	{
		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Unable to resolve station %d.%d\n",
		           (int)BeebTx.EconetHeader.DestNet,
		           (int)BeebTx.EconetHeader.DestStn);
		#endif

		return;
	}

	// Send a datagram to the receiver.

	#ifdef DEBUG_ECONET
	DebugTrace("Econet: TXLast set: Send %d byte packet to station %d.%d (%s port %u)\n",
	           BeebTx.Pointer,
	           (int)BeebTx.EconetHeader.DestNet,
	           (int)BeebTx.EconetHeader.DestStn,
	           IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
	           (unsigned int)htons(RecvAddr.sin_port));

	DebugDumpBytes("Econet: Packet data:", BeebTx.Buffer, BeebTx.Pointer);
	#endif

	unsigned int j = 0;
	// OK. Lets do AUN ...
	// The beeb has given us a packet .. what is it?
	SendMe = false;

	EconetTx.DestNet = BeebTx.EconetHeader.DestNet;
	EconetTx.DestStn = BeebTx.EconetHeader.DestStn;

	switch (AUNState)
	{
		case FourWayStage::ScoutAckReceived:
			// It came in response to our ack of a scout.
			// What we have /should/ be the data block.
			// CLUDGE WARNING is this a scout sent again immediately?? TODO fix this?!?!
			if (EconetTx.AUNHeader.Port == 0x00)
			{
				if (EconetTx.AUNHeader.CtrlByte == (0x82 & 0x7f))
				{
					j = 8;
				}
				else if (EconetTx.AUNHeader.CtrlByte >= (0x83 & 0x7f) &&
				         EconetTx.AUNHeader.CtrlByte <= (0x85 & 0x7f))
				{
					j = 4;
				}
			}

			if (BeebTx.Pointer != sizeof(EconetHeaderType) + j || memcmp(BeebTx.Buffer, BeebTxCopy, sizeof(EconetHeaderType) + j) != 0) // nope
			{
				for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
					EconetTx.Buffer[j] = BeebTx.Buffer[k];
				}
				EconetTx.Pointer = j;

				SendMe = true;
				SendLen = sizeof(AUNHeaderType) + EconetTx.Pointer;

				AUNState = FourWayStage::DataSent;

				#ifdef DEBUG_ECONET
				DebugTrace("Econet: Set FourWayStage::DataSent\n");
				#endif
				break;
			} // else fall through...

		case FourWayStage::Idle:
			// Not currently doing anything, so this will be a scout,
			// maybe a long scout or a broadcast.
			memcpy(BeebTxCopy, BeebTx.Buffer, sizeof(EconetHeaderType));
			EconetTx.AUNHeader.CtrlByte = BeebTx.EconetHeader.CtrlByte & 127; // | 128;
			EconetTx.AUNHeader.Port = BeebTx.EconetHeader.Port;
			EconetTx.AUNHeader.Pad = 0;
			EconetTx.AUNHeader.Handle = (ec_sequence += 4);

			for (unsigned int k = 6; k < BeebTx.Pointer; k++, j++) {
				EconetTx.Buffer[j] = BeebTx.Buffer[k];
			}

			EconetTx.Pointer = j;

			if (IsBroadcastStation(EconetTx.DestStn))
			{
				EconetTx.AUNHeader.Type = AUNType::Broadcast;

				AUNState = FourWayStage::WaitForIdle; // no response to broadcasts...

				SendMe = true; // send packet ...
				SendLen = sizeof(AUNHeaderType) + 8;

				if (EconetTx.AUNHeader.Port == 0x9c &&
				    EconetTx.AUNHeader.CtrlByte == (0x82 & 0x7f))
				{
					WhatNetPort = EconetTx.Buffer[6]; // where the reply will be sent

					#ifdef DEBUG_ECONET
					DebugTrace("Econet: Sent WhatNet query with port &%x\n", WhatNetPort);
					#endif
				}
			}
			else if (EconetTx.AUNHeader.Port == 0 &&
			         (EconetTx.AUNHeader.CtrlByte < (0x82 & 0x7f) ||
			          EconetTx.AUNHeader.CtrlByte > (0x85 & 0x7f)))
			{
				EconetTx.AUNHeader.Type = AUNType::Immediate;

				AUNState = FourWayStage::ImmediateSent;

				SendMe = true; // send packet ...
				SendLen = sizeof(AUNHeaderType) + EconetTx.Pointer;

				#ifdef DEBUG_ECONET
				DebugTrace("Econet: Set FourWayStage::ImmediateSent\n");
				#endif
			}
			else
			{
				EconetTx.AUNHeader.Type = AUNType::Unicast;

				AUNState = FourWayStage::ScoutSent;

				// Don't send anything but set wait anyway.
				SetTrigger(EconetScoutAckTimeout, EconetScoutAckTrigger);

				#ifdef DEBUG_ECONET
				DebugTrace("Econet: Set FourWayStage::ScoutSent\n");
				DebugTrace("Econet: Scout Ack Timeout set\n");
				#endif
			} // else BROADCAST !!!!
			break;

		case FourWayStage::ScoutReceived:
			// It's an ack for a scout which we sent the Beeb.
			// Just drop it, but move on.
			AUNState = FourWayStage::ScoutAckSent;

			SetTrigger(EconetScoutAckTimeout, EconetScoutAckTrigger);

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::ScoutAckSent\n");
			DebugTrace("Econet: Scout Ack Timeout set\n");
			#endif
			break;

		case FourWayStage::DataReceived:
			// This must be ack for data just received.
			// Now we really need to send an ack to the far AUN host...
			// Send header of last block received straight back.
			// This ought to work, but only because the Beeb can only
			// talk to one machine at any time.
			EconetTx.AUNHeader = EconetRx.AUNHeader;
			EconetTx.AUNHeader.Type = AUNType::Ack;

			SendLen = sizeof(AUNHeaderType);
			SendMe = true;

			AUNState = FourWayStage::WaitForIdle;

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::WaitForIdle (final ack sent)\n");
			#endif
			break;

		case FourWayStage::ImmediateReceived:
			// It's a reply to an immediate command we just had.
			for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
				EconetTx.Buffer[j] = BeebTx.Buffer[k];
			}

			EconetTx.Pointer = j;

			EconetTx.AUNHeader = EconetRx.AUNHeader;
			EconetTx.AUNHeader.Type = AUNType::ImmReply;

			SendMe = true;
			SendLen = sizeof(AUNHeaderType) + EconetTx.Pointer;

			AUNState = FourWayStage::WaitForIdle;

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::WaitForIdle (immediate received)\n");
			#endif
			break;

		default:
			// Shouldn't be here. Ignore packet and abort fourway.
			AUNState = FourWayStage::WaitForIdle;

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::WaitForIdle (unexpected mode, packet ignored)\n");
			#endif
			break;
	}

	if (SendMe)
	{
		char *p = (char *)&EconetTx;

		ExtendedAUNPacket *tmp = (ExtendedAUNPacket*)&EconetTemp;

		if (ExtendedAUN || (IsBroadcastStation(EconetTx.DestStn) && Gateway.port != 0))
		{
			// We need to make a copy of the packet with additional addressing on the front.
			tmp->EconetHeader.DestStn = BeebTx.EconetHeader.DestStn;
			tmp->EconetHeader.DestNet = BeebTx.EconetHeader.DestNet;
			tmp->EconetHeader.SrcStn = 0; // source (left blank)
			tmp->EconetHeader.SrcNet = 0;
			memcpy(&tmp->AUNHeader, &EconetTx.raw, SendLen); // Copy original AUN data.

			if (ExtendedAUN)
			{
				SendLen += 4;
				p = (char *)tmp; // Transmit this buffer instead of EconetTx.
			}
			// else a broadcast - we will send this extended AUN packet to the gateway after sending the original packet as a UDP broadcast
		}

		if (!(RecvAddr.sin_addr.s_addr == EconetListenIP && htons(RecvAddr.sin_port) == EconetListenPort)) // never send to ourself
		{
			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Send packet to station %d.%d (%s port %u)\n",
			           (int)EconetTx.DestNet,
			           (int)EconetTx.DestStn,
			           IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
			           (unsigned int)htons(RecvAddr.sin_port));

			DebugDumpBytes("Econet: Ethernet data:", (const unsigned char*)p, SendLen);
			#endif

			if (sendto(Socket, p, SendLen, 0,
			           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
			{
				EconetError("Econet: Failed to send packet to station %d (%s port %u)",
				            (int)EconetTx.DestStn,
				            IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
				            (unsigned int)htons(RecvAddr.sin_port));
			}
		}

		if (IsBroadcastStation(EconetTx.DestStn) && Gateway.port != 0)
		{
			// we want to send a copy of the broadcast to the gateway
			RecvAddr.sin_addr.s_addr = Gateway.inet_addr;
			RecvAddr.sin_port = htons(Gateway.port);

			#ifdef DEBUG_ECONET
			DebugDumpBytes("Econet: Gateway broadcast ethernet data:", (unsigned char *)p, SendLen + 4);
			#endif

			if (sendto(Socket, (char *)tmp, SendLen + 4, 0,
			           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
			{
				EconetError("Econet: Failed to send broadcast to gateway (%s port %u)",
				            IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
				            (unsigned int)htons(RecvAddr.sin_port));
			}
		}
	}

	// Sending packet will mean peer goes into flag fill while
	// it deals with it.
	FlagFillActive = true;
	SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);

	#ifdef DEBUG_ECONET
	DebugTrace("Econet: FlagFill set (packet sent)\n");
	#endif

	// Wipe buffer.
	BeebTx.Pointer = 0;
	BeebTx.BytesInBuffer = 0;
}

//--------------------------------------------------------------------------------------------

static bool EconetReceivePacket()
{
	if (AUNState == FourWayStage::Idle ||
	    AUNState == FourWayStage::ImmediateSent ||
	    AUNState == FourWayStage::DataSent)
	{
GetNewPacket:
		// Try to get another packet from the network.
		// Check if packet is waiting without blocking.
		fd_set ReadFds;
		timeval TimeOut = {0, 0};
		SOCKET SocketToRead = Socket;

		// Check the main listen socket.
		FD_ZERO(&ReadFds);
		FD_SET(Socket, &ReadFds);
		int NumReady = select((int)Socket + 1, &ReadFds, NULL, NULL, &TimeOut);

		if (NumReady == 0 && BroadcastListenSocket != INVALID_SOCKET)
		{
			// Nothing on main listen socket, check if BroadcastListenSocket has a packet.
			FD_ZERO(&ReadFds);
			FD_SET(BroadcastListenSocket, &ReadFds);

			NumReady = select((int)BroadcastListenSocket + 1, &ReadFds, NULL, NULL, &TimeOut);

			if (NumReady > 0)
			{
				// Switch to use this socket.
				SocketToRead = BroadcastListenSocket;
			}
		}

		if (NumReady > 0)
		{
			// Read the packet.
			sockaddr_in RecvAddr;
			int RecvAddrSize = sizeof(RecvAddr);

			int BytesReceived = recvfrom(SocketToRead,
			                             (char *)&EconetRx,
			                             sizeof(AUNHeaderType) + sizeof(EconetRx.Buffer),
			                             0,
			                             (SOCKADDR *)&RecvAddr,
			                             &RecvAddrSize);

			if (SocketToRead == BroadcastListenSocket &&
			    EconetRx.AUNHeader.Type != AUNType::Broadcast &&
			    EconetRx.AUNHeader.Type != AUNType::BeebEm)
			{
				BytesReceived = 0; // Non-broadcast/beebem packet seen on broadcast socket - ignore
			}

			if (RecvAddr.sin_addr.s_addr == EconetListenIP &&
			    htons(RecvAddr.sin_port) == EconetListenPort)
			{
				BytesReceived = 0; // We sent this broadcast packet - ignore
			}

			EconetRx.BytesInBuffer = BytesReceived;

			if (BytesReceived > 0)
			{
				#ifdef DEBUG_ECONET
				DebugTrace("EconetPoll: Packet received: %d bytes from %s port %u\n",
				           BytesReceived,
				           IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
				           htons(RecvAddr.sin_port));

				DebugDumpBytes("EconetPoll: Packet data:",
				               EconetRx.raw,
				               BytesReceived);
				#endif

				// Convert from AUN format.
				// Find network and station number of sender.
				bool Found = false;

				// Search for source in known stations.
				for (size_t i = 0; i < Stations.size(); ++i)
				{
					EconetHost& Station = Stations[i];

					if (htons(RecvAddr.sin_port) == Station.port &&
					    RecvAddr.sin_addr.s_addr == Station.inet_addr)
					{
						Found = true;

						BeebRx.EconetHeader.SrcNet = Station.network;
						BeebRx.EconetHeader.SrcStn = Station.station;

						if (EconetRx.AUNHeader.Type == AUNType::Broadcast)
						{
							// See if a gateway has already sent broadcasts from this station.
							if (Station.broadcasts == BroadcastSource::Gateway)
							{
								// Don't resolve this station.
								Found = false;
							}
							else
							{
								Station.broadcasts = BroadcastSource::Local;
							}
						}
						break;
					}
				}

				if (!Found)
				{
					// Search source in networks.
					for (size_t i = 0; i < Networks.size(); ++i)
					{
						EconetNet& Network = Networks[i];

						if (RecvAddr.sin_addr.s_addr == Network.inet_addr)
						{
							// A single address using sequential ports.
							int Station = htons(RecvAddr.sin_port) - Network.port;

							// Check whether result is in range.
							if (Station > 0 && Station < 255)
							{
								BeebRx.EconetHeader.SrcNet = Network.network;
								BeebRx.EconetHeader.SrcStn = (unsigned char)Station;
								Found = true;
							}
							// else must be a different net on the same host
						}
						else if ((RecvAddr.sin_addr.s_addr & 0x00FFFFFF) == Network.inet_addr &&
						         htons(RecvAddr.sin_port) == DEFAULT_AUN_PORT)
						{
							// True AUN addressing.
							BeebRx.EconetHeader.SrcNet = Network.network;
							BeebRx.EconetHeader.SrcStn = (RecvAddr.sin_addr.s_addr & 0xFF000000) >> 24;
							Found = true;
						}

						if (Found)
						{
							if (EconetRx.AUNHeader.Type == AUNType::Broadcast)
							{
								// See if a gateway has already sent broadcasts from this network.
								if (Network.broadcasts == BroadcastSource::Gateway)
								{
									// Don't resolve this network.
									Found = false;
								}
								else
								{
									Network.broadcasts = BroadcastSource::Local;
								}
							}

							break;
						}
					}
				}

				if (!Found && BytesReceived > 4)
				{
					// Search to see if source is extended AUN gateway.
					if (RecvAddr.sin_addr.s_addr == Gateway.inet_addr &&
					    htons(RecvAddr.sin_port) == Gateway.port)
					{
						// PiEconetBridge gateways use an extended AUN which contains
						// the Econet addresses at the start of the packet.
						// This means the AUN data we want starts four bytes later
						// than usual. This seems terribly inefficient, but lets
						// remove those bytes from the buffer rather than trying
						// to keep track of an offset through all the rest of the code.

						memcpy(&BeebRx.EconetHeader, &EconetRx, sizeof(BeebRx.EconetHeader));
						memmove(EconetRx.raw, EconetRx.raw + 4, BytesReceived - 4);
						BytesReceived -= 4; // adjust the length
						EconetRx.BytesInBuffer = BytesReceived; // must update this too!

						Found = true;

						if (IsBroadcastStation(BeebRx.EconetHeader.DestStn))
						{
							// We want to ignore any broadcasts via the gateway
							// if we will also receive them directly.
							for (size_t i = 0; i < Networks.size(); i++)
							{
								EconetNet& Network = Networks[i];

								if (Network.network == BeebRx.EconetHeader.SrcNet)
								{
									if (Network.broadcasts == BroadcastSource::Local)
									{
										// Don't resolve broadcasts from this net.
										Found = false;
									}
									else
									{
										Network.broadcasts = BroadcastSource::Gateway;
									}

									break;
								}
							}

							for (size_t i = 0; i < Stations.size() && Found; ++i)
							{
								EconetHost& Station = Stations[i];

								if (Station.network == BeebRx.EconetHeader.SrcNet && Station.station == BeebRx.EconetHeader.SrcStn)
								{
									if (Station.broadcasts == BroadcastSource::Local)
									{
										// Don't resolve broadcasts from this station.
										Found = false;
									}
									else
									{
										Station.broadcasts = BroadcastSource::Gateway;
									}

									break;
								}
							}
						}
					}
				}
				else
				{
					if (MassageNetworks)
					{
						// Make AUN nets > 127 appear to be Econet.
						BeebRx.EconetHeader.SrcNet &= 0x7F;
					}
				}

				if (!Found)
				{
					// Couldn't resolve Econet source address

					// It might be a bridge gateway response.
					if (BytesReceived == 12 && FindGateways)
					{
						// If it is then it will be Extended AUN so all the headers are moved.
						ExtendedAUNPacket *rx = (ExtendedAUNPacket*)&EconetRx;

						// Check it looks like a gateway reply should do.
						if (rx->EconetHeader.SrcStn == 0 &&
						    rx->EconetHeader.SrcNet != 0 &&
						    rx->AUNHeader.Type == AUNType::Unicast &&
						    rx->AUNHeader.Port == BEEBEM_ECONET_PORT &&
						    (rx->AUNHeader.CtrlByte | 0x80) == 0x91 &&
						    rx->AUNHeader.Pad == 0)
						{
							// It does!
							// rx->addr.deststn is our station number on the bridge
							// rx->addr.destnet is our network number on the bridge

							// Check we haven't got a gateway defined already.
							if (Gateway.port == 0)
							{
								Gateway.inet_addr = RecvAddr.sin_addr.s_addr;
								Gateway.port = htons(RecvAddr.sin_port);

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Learned about gateway at %s:%d. Bridge sees us as station %d.%d\n",
								           IpAddressStr(Gateway.inet_addr).c_str(),
								           Gateway.port,
								           (unsigned int)rx->EconetHeader.DestNet,
								           (unsigned int)rx->EconetHeader.DestStn);
								#endif

								// Start/reset keepalives.
								time(&GatewayTimeout);
							}
							else if (Gateway.inet_addr != RecvAddr.sin_addr.s_addr ||
							         Gateway.port != htons(RecvAddr.sin_port))
							{
								// This response was from a different gateway
								// to the one we already have configured!
								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Ignored gateway response from %s:%d",
								           IpAddressStr(RecvAddr.sin_addr.s_addr).c_str(),
								           htons(RecvAddr.sin_port));
								#endif
							}
						}
					}

					// It might be an address announce packet from an unknown station.
					if (BytesReceived == 16 &&
					    EconetRx.AUNHeader.Port == BEEBEM_ECONET_PORT &&
					    EconetRx.AUNHeader.Type == AUNType::BeebEm &&
					    AutoConfigure)
					{
						if ((EconetRx.AUNHeader.CtrlByte | 128) == 0x9f) // BeebEm Ping
						{
							// This is a BeebEm Ping used for host discovery.
							BeebRx.EconetHeader.SrcStn = EconetRx.Buffer[0];
							BeebRx.EconetHeader.SrcNet = EconetRx.Buffer[1];

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Received BeebEm Ping from %d.%d\n",
							           (int)BeebRx.EconetHeader.SrcNet,
							           (int)BeebRx.EconetHeader.SrcStn);
							#endif

							if (BeebRx.EconetHeader.SrcStn == EconetStationID &&
							    BeebRx.EconetHeader.SrcNet == EconetNetworkID)
							{
								// Address collision!
								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Address collision!\n");
								#endif

								if (EconetRx.AUNHeader.Handle >= AnnounceHandle)
								{
									// They have had the address longer than us.
									// Relinquish the address.
									PreferredStationID = (rand() % 253) + 1;
									EconetStationID = 0;

									EconetError("Econet: Address collision detected.");
									mainWin->ToggleEconet(); // Turn Econet off entirely.
									return false;
								}
							}
							else
							{
								EconetHost* ptr = FindNetworkConfig(BeebRx.EconetHeader.SrcStn, BeebRx.EconetHeader.SrcNet);

								if (ptr == nullptr || time(NULL) >= ptr->timeout)
								{
									// Station is new or stale.
									AddStation(BeebRx.EconetHeader.SrcStn,
									           BeebRx.EconetHeader.SrcNet,
									           RecvAddr.sin_addr.s_addr,
									           htons(RecvAddr.sin_port),
									           BroadcastSource::Local); // Must be in the broadcast domain to have received this ping.
								}
							}
						}
					}

					#ifdef DEBUG_ECONET
					DebugTrace("Econet: Packet ignored\n");
					#endif

					// Go back and look for another packet.
					goto GetNewPacket;
				}
				else
				{
					#ifdef DEBUG_ECONET
					DebugTrace("Econet: Packet was from station %d.%d\n",
					           (int)BeebRx.EconetHeader.SrcNet,
					           (int)BeebRx.EconetHeader.SrcStn);
					#endif

					BeebRx.EconetHeader.CtrlByte = EconetRx.AUNHeader.CtrlByte | 128;
					BeebRx.EconetHeader.Port = EconetRx.AUNHeader.Port;

					if (EconetRx.AUNHeader.Type == AUNType::BeebEm)
					{
						// Catch proprietary packets used for network discovery.
						// This has no effect on the FourWayStage state, so can
						// be handled at any time.
						if (EconetRx.AUNHeader.Port == BEEBEM_ECONET_PORT &&
						    EconetRx.AUNHeader.CtrlByte == 0x9f && AutoConfigure)
						{
							// This is a BeebEm Ping used for host discovery
							// from an address we think we know already.

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Received BeebEm Ping from %d.%d\n",
							           (int)EconetRx.Buffer[1],
							           (int)EconetRx.Buffer[0]);
							#endif

							if (EconetRx.Buffer[0] == EconetStationID && EconetRx.Buffer[1] == EconetNetworkID)
							{
								// Address collision!
								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Address collision!\n");
								#endif

								if (EconetRx.AUNHeader.Handle >= AnnounceHandle)
								{
									// They have had the address longer than us.
									// Relinquish the address.
									PreferredStationID = (rand() % 253) + 1;
									EconetStationID = 0;

									EconetError("Econet: Address collision detected.");
									mainWin->ToggleEconet(); // Turn Econet off entirely.
									return false;
								}
							}
							else if (EconetRx.Buffer[0] != BeebRx.EconetHeader.SrcStn ||
							         EconetRx.Buffer[1] != BeebRx.EconetHeader.SrcNet)
							{
								// Station number of this host has changed for some reason.
								EconetHost* pStation = FindNetworkConfig(BeebRx.EconetHeader.SrcStn, BeebRx.EconetHeader.SrcNet);

								if (pStation != nullptr)
								{
									if (time(NULL) >= pStation->timeout)
									{
										// Host is stale - replace it.
										pStation->station = EconetRx.Buffer[0];
										pStation->network = EconetRx.Buffer[1];
										pStation->timeout = time(NULL) + HOST_TIMEOUT;

										#ifdef DEBUG_ECONET
										DebugTrace("Econet: updated station number\n");
										#endif
									}
								}
							}
							else
							{
								EconetHost* pStation = FindNetworkConfig(BeebRx.EconetHeader.SrcStn, BeebRx.EconetHeader.SrcNet);

								if (pStation != nullptr)
								{
									// update timeout
									pStation->timeout = time(NULL) + HOST_TIMEOUT;
								}
							}
						}

						// Go back and look for a real Econet packet.
						goto GetNewPacket;
					}
					else if (EconetRx.AUNHeader.Type == AUNType::Unicast &&
					         BeebRx.BytesInBuffer == 0 &&
					         BeebRx.EconetHeader.Port == BEEBEM_ECONET_PORT &&
					         BeebRx.EconetHeader.CtrlByte == 0x91 &&
					         EconetRx.AUNHeader.Handle == 0)
					{
						// This is a bridge gateway response for a gateway
						// we already know about.
						#ifdef DEBUG_ECONET
						DebugTrace("Econet: Gateway response received. Bridge sees us as station %d.%d\n",
						           (int)BeebRx.EconetHeader.DestNet,
						           (int)BeebRx.EconetHeader.DestStn);
						#endif

						// Go back and look for a real Econet packet.
						goto GetNewPacket;
					}

					switch (AUNState)
					{
					case FourWayStage::Idle:
						// We weren't doing anything when this packet came in.
						switch (EconetRx.AUNHeader.Type)
						{
							case AUNType::Broadcast:
								if (BeebRx.EconetHeader.Port == 0x9c &&
								    EconetRx.AUNHeader.Handle == 0)
								{
									// This is a gateway discovery broadcast
									// from another BeebEm instance so ignore it.
									AUNState = FourWayStage::WaitForIdle;
									break;
								}
								else
								{
									// It is a real Econet broadcast.
									BeebRx.EconetHeader.DestStn = 255; // Not just for us.
									BeebRx.EconetHeader.DestNet = 255;

									const int Offset = sizeof(LongEconetPacket);
									const int Length = BytesReceived - sizeof(EconetRx.AUNHeader);
									memcpy(BeebRx.Buffer + Offset, EconetRx.Buffer, Length);
									BeebRx.BytesInBuffer = Offset + Length;

									AUNState = FourWayStage::WaitForIdle;

									#ifdef DEBUG_ECONET
									DebugTrace("Econet: Set FourWayStage::WaitForIdle (broadcast received)\n");
									#endif
								}
								break;

							case AUNType::Immediate: {
								// Must be for us.
								BeebRx.EconetHeader.DestStn = EconetStationID;
								BeebRx.EconetHeader.DestNet = 0;

								const int Offset = sizeof(LongEconetPacket);
								const int Length = BytesReceived - sizeof(AUNHeaderType);
								memcpy(BeebRx.Buffer + Offset, EconetRx.Buffer, Length);
								BeebRx.BytesInBuffer = Offset + Length;

								AUNState = FourWayStage::ImmediateReceived;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::ImmediateReceived\n");
								#endif
								break;
							}

							case AUNType::Unicast:
								BeebRx.EconetHeader.DestStn = EconetStationID; // must be for us.
								BeebRx.EconetHeader.DestNet = 0;

								if (EconetRx.AUNHeader.Port == 0 && EconetRx.AUNHeader.CtrlByte == (0x82 & 0x7f))
								{
									const int Offset = sizeof(LongEconetPacket);
									const int Length = 8;
									memcpy(BeebRx.Buffer + Offset, EconetRx.Buffer, Length);
									BeebRx.BytesInBuffer = Offset + Length;
								}
								else if (EconetRx.AUNHeader.Port == 0 &&
								         EconetRx.AUNHeader.CtrlByte >= (0x83 & 0x7f) &&
								         EconetRx.AUNHeader.CtrlByte <= (0x85 & 0x7f))
								{
									const int Offset = sizeof(LongEconetPacket);
									const int Length = 4;
									memcpy(BeebRx.Buffer + Offset, EconetRx.Buffer, Length);
									BeebRx.BytesInBuffer = Offset + Length;
								}
								else
								{
									if (EconetRx.AUNHeader.Port == WhatNetPort && EconetRx.AUNHeader.CtrlByte == 0x80)
									{
										#ifdef DEBUG_ECONET
										DebugTrace("Econet: Got WhatNet reply\n");
										#endif

										WhatNetPort = -1;
										EconetRx.Buffer[0] = EconetNetworkID; // fudge whatnet reply
									}

									BeebRx.BytesInBuffer = sizeof(LongEconetPacket);
								}

								AUNState = FourWayStage::ScoutReceived;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::ScoutReceived\n");
								#endif
								break;

							default:
								// Ignore anything else.
								AUNState = FourWayStage::WaitForIdle;
								BeebRx.BytesInBuffer = 0;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::WaitForIdle\n");
								#endif
								break;
						}

						BeebRx.Pointer = 0;
						break;

					case FourWayStage::ImmediateSent:
						// It should be reply to an immediate instruction
						// (or an Ack for immediates &86 and &87?).
						// I'm pretty sure that real Econet can't send to itself.

						// Must be for us.
						BeebRx.EconetHeader.DestStn = EconetStationID;
						BeebRx.EconetHeader.DestNet = 0;

						if (EconetRx.AUNHeader.Type == AUNType::ImmReply)
						{
							const int Offset = 4;
							const int Length = BytesReceived - sizeof(AUNHeaderType);
							memcpy(BeebRx.Buffer + Offset, EconetRx.Buffer, Length);
							BeebRx.BytesInBuffer = Offset + Length;
							BeebRx.Pointer = 0;

							AUNState = FourWayStage::WaitForIdle;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Set FourWayStage::WaitForIdle (ack received from remote AUN server)\n");
							#endif
						}
						else
						{
							// Packet wasn't an immediate reply so throw it away.
							BeebRx.BytesInBuffer = 0;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Unexpected packet dropped\n");
							#endif
						}

						BeebRx.Pointer = 0;
						break;

					case FourWayStage::DataSent:
						// Must be for us.
						BeebRx.EconetHeader.DestStn = EconetStationID;
						BeebRx.EconetHeader.DestNet = 0;

						// We sent block of data, awaiting final ack.
						if (EconetRx.AUNHeader.Type == AUNType::Ack ||
						    EconetRx.AUNHeader.Type == AUNType::NAck)
						{
							// Are we expecting a (N)ACK?
							// TODO check it is a (n)ack for the packet we just sent. Deal with nacks!
							// Construct a final ack for the Beeb.
							BeebRx.BytesInBuffer = 4;

							AUNState = FourWayStage::WaitForIdle;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Set FourWayStage::WaitForIdle (AUN ack received)\n");
							#endif
						}
						else
						{
							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Unexpected packet dropped\n");
							#endif

							BeebRx.BytesInBuffer = 0;
						}

						BeebRx.Pointer = 0;
						break;

					default:
						// Erm, what are we doing here? Ignore packet.
						AUNState = FourWayStage::WaitForIdle;

						#ifdef DEBUG_ECONET
						DebugTrace("Econet: Set FWS_WAIT4IDLE (invalid state)\n");
						#endif
						break;
					}

					if ((BeebRx.EconetHeader.DestStn == EconetStationID || IsBroadcastStation(BeebRx.EconetHeader.DestStn)) &&
					    BeebRx.BytesInBuffer > 0)
					{
						// Peer sent us packet - no longer in flag fill.
						FlagFillActive = false;

						#ifdef DEBUG_ECONET
						DebugTrace("Econet: FlagFill reset\n");
						#endif
					}
					else
					{
						// Two other stations communicating - assume one of them will flag fill
						FlagFillActive = true;
						SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);

						#ifdef DEBUG_ECONET
						DebugTrace("Econet: FlagFill set - other station comms\n");
						#endif
					}
				}
			}
			else if (BytesReceived == SOCKET_ERROR)
			{
				#ifdef DEBUG_ECONET
				DebugTrace("Econet: Failed to receive packet (error %d)\n", GetLastSocketError());
				#endif
			}
		}
		else if (NumReady == SOCKET_ERROR)
		{
			EconetError("Econet: Failed to check for new packet");
		}
	}

	// This bit fakes the bits of the 4-way handshake that AUN doesn't do.

	if (EconetScoutAckTrigger > TotalCycles)
	{
		switch (AUNState)
		{
		case FourWayStage::ScoutSent:
			// Just got a scout from the Beeb, fake an acknowledgement.
			BeebRx.EconetHeader.DestStn = EconetStationID;
			BeebRx.EconetHeader.DestNet = 0;

			BeebRx.EconetHeader.SrcStn = EconetTx.DestStn; // Use scout's dest as source of ack.
			BeebRx.EconetHeader.SrcNet = EconetTx.DestNet;

			BeebRx.BytesInBuffer = 4;
			BeebRx.Pointer = 0;

			AUNState = FourWayStage::ScoutAckReceived;

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::ScoutAckReceived\n");
			#endif
			break;

		case FourWayStage::ScoutAckSent: {
			// Beeb acked the scout we gave it, so give it the data AUN sent us earlier.
			BeebRx.EconetHeader.DestStn = EconetStationID; // As it is data it must be for us.
			BeebRx.EconetHeader.DestNet = 0;

			BeebRx.EconetHeader.SrcStn = EconetTx.DestStn; //30jun don't think this is right..
			BeebRx.EconetHeader.SrcNet = EconetTx.DestNet;

			const int DestOffset = sizeof(EconetHeaderType);

			if (EconetRx.AUNHeader.Port == 0 &&
			    EconetRx.AUNHeader.CtrlByte == (0x82 & 0x7f))
			{
				const int SrcOffset = 8;
				const int Length = EconetRx.BytesInBuffer - sizeof(AUNHeaderType) - SrcOffset;
				memcpy(BeebRx.Buffer + DestOffset, EconetRx.Buffer + SrcOffset, Length);
				BeebRx.BytesInBuffer = DestOffset + Length;
			}
			else if (EconetRx.AUNHeader.Port == 0 &&
			         EconetRx.AUNHeader.CtrlByte >= (0x83 & 0x7f) &&
			         EconetRx.AUNHeader.CtrlByte <= (0x85 & 0x7f))
			{
				const int SrcOffset = 4;
				const int Length = EconetRx.BytesInBuffer - sizeof(AUNHeaderType) - SrcOffset;
				memcpy(BeebRx.Buffer + DestOffset, EconetRx.Buffer + SrcOffset, Length);
				BeebRx.BytesInBuffer = DestOffset + Length;
			}
			else
			{
				const int Length = EconetRx.BytesInBuffer - sizeof(AUNHeaderType);
				memcpy(BeebRx.Buffer + DestOffset, EconetRx.Buffer, Length);
				BeebRx.BytesInBuffer = DestOffset + Length;
			}

			BeebRx.Pointer = 0;

			AUNState = FourWayStage::DataReceived;

			#ifdef DEBUG_ECONET
			DebugTrace("Econet: Set FourWayStage::DataReceived\n");
			#endif
			break;
		}

		default:
			break;
		}
	}

	// Translate packets from the same net number to look local.
	if (BeebRx.EconetHeader.SrcNet == EconetNetworkID)
	{
		BeebRx.EconetHeader.SrcNet = 0;
	}

	return true;
}

//--------------------------------------------------------------------------------------------

void DebugEconetState()
{
	DebugDisplayInfoF("ADLC: Ctl:%02X %02X %02X %02X St:%02X %02X TXptr:%01x rx:%01x FF:%d IRQc:%02x SR2Qc:%02x PC:%04x AUN:%s",
	                  (int)ADLC.Control1, (int)ADLC.Control2, (int)ADLC.Control3, (int)ADLC.Control4,
	                  (int)ADLC.Status1, (int)ADLC.Status2,
	                  (int)ADLC.TxFifoPtr, (int)ADLC.RxFifoPtr, FlagFillActive ? 1 : 0,
	                  (int)IRQCause, (int)S2RQCause, (int)ProgramCounter, AUNStateStr(AUNState));
}

//--------------------------------------------------------------------------------------------

// Display an error message box.

static void EconetError(const char *Format, ...)
{
	va_list Args;
	va_start(Args, Format);

	if (DebugEnabled)
	{
		DebugDisplayTraceV(DebugType::Econet, true, Format, Args);
	}

	mainWin->ReportV(MessageType::Error, Format, Args);

	va_end(Args);
}

//--------------------------------------------------------------------------------------------
