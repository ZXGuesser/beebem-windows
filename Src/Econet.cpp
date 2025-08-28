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

#include <windows.h>

#include <stdio.h>

#include <fstream>
#include <string>
#include <vector>

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
struct MC6854 {
	unsigned char control1;
	unsigned char control2;
	unsigned char control3;
	unsigned char control4;
	unsigned char txfifo[3];
	unsigned char rxfifo[3];
	unsigned char txfptr; // first empty byte in fifo
	unsigned char rxfptr; // first empty byte in fifo
	unsigned char txftl; // tx fifo tx lst flags. (bits relate to subscripts)
	unsigned char rxffc; // rx fifo fc flags bits
	unsigned char rxap; // rx fifo ap flags (bits relate to subscripts)

	unsigned char status1;
	unsigned char status2;

	int sr2pse; // PSE level for SR2 rx bits
	// 0 = inactive
	// 1 = ERR, FV, DCD, OVRN, ABT
	// 2 = Idle
	// 3 = AP
	// 4 = RDA

	bool cts; // signal up
	bool idle;
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
const bool DEFAULT_AUN_MODE = false;
const bool DEFAULT_STRICT_AUN_MODE = false;
const bool DEFAULT_SINGLE_SOCKET = true;
const int DEFAULT_FLAG_FILL_TIMEOUT = 500000;
const int DEFAULT_SCOUT_ACK_TIMEOUT = 5000;
const unsigned int DEFAULT_TIME_BETWEEN_BYTES = 128;
const unsigned int DEFAULT_FOUR_WAY_STAGE_TIMEOUT = 500000;
const bool DEFAULT_MASSAGE_NETWORKS = false;

static bool AUNMode = DEFAULT_AUN_MODE; // Use Acorn Universal Networking (AUN) style networking
static bool StrictAUNMode = DEFAULT_STRICT_AUN_MODE; // Assume network ip=stn number when sending to unknown hosts
static bool SingleSocket = DEFAULT_SINGLE_SOCKET; // Use same socket for Send and receive
static unsigned int FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
static bool MassageNetworks = DEFAULT_MASSAGE_NETWORKS; // Massage network numbers on send/receive (add/sub 128)

bool EconetStateChanged = false;
bool EconetEnabled;    // Enable hardware
bool EconetNMIEnabled; // 68B54 -> NMI enabled. (IC97)
int EconetTrigger;     // Poll timer

static const unsigned char powers[4] = { 1, 2, 4, 8 };

// Frequency between network actions.
// max 250Khz network clock. 2MHz system clock. one click every 8 cycles.
// say one byte takes about 8 clocks, receive a byte every 64 cpu cycles. ?
// (The reason for "about" 8 clocks is that as this a continuous syncronous tx,
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
unsigned char EconetStationID = 0; // default Station ID
unsigned char myaunnet = 0; // what is our local net number
static u_short EconetListenPort = 0; // default Listen port
static unsigned long EconetListenIP = 0x0100007f;
// IP settings:
static SOCKET ListenSocket = INVALID_SOCKET; // Listen socket
static SOCKET SendSocket = INVALID_SOCKET;
static bool ReceiverSocketsOpen = false; // Used to flag line up and clock running

const u_short DEFAULT_AUN_PORT = 32768;

// Written in 2004:
// we will be using Econet over Ethernet as per AUN,
// however I've not got a real Acorn ethernet machine to see how
// it actually works! The only details I can find is it is:
// "standard econet encpsulated in UDP to port 32768" and that
// Addressing defaults to "1.0.net.stn" where net >= 128 for ethernet.
// but can be overridden, so we won't worry about that.

// 2009: Now I come back to this, I know the format ... :-)
// and sure enough, it's pretty simple.
// It's translating the different protocols that was harder.

enum class AUNType : unsigned char {
	Broadcast = 1,
	Unicast = 2,
	Ack = 3,
	NAck = 4,
	Immediate = 5,
	ImmReply = 6
};

struct AUNHeader
{
	AUNType type;         // AUN magic protocol byte
	unsigned char port;   // dest port
	unsigned char cb;     // flag
	unsigned char pad;    // retrans
	uint32_t handle;      // 4 byte sequence little-endian.
};

// #define EC_PORT_FS 0x99
// #define EC_PORT_PS_STATUS_ENQ 0x9f
// #define EC_PORT_PS_STATUS_REPLY 0x9e
// #define EC_PORT_PS_JOB 0xd1

static unsigned long ec_sequence = 0;

enum class FourWayStage {
	Idle = 0,
	ScoutSent = 1,
	ScoutAckReceived = 2,
	DataSent = 3,
	WaitForIdle = 4,
	ScoutReceived = 11,
	ScoutAckSent = 12,
	DataReceived = 13,
	ImmediateSent = 7,
	ImmediateReceived = 8
};

static FourWayStage fourwaystage;

struct ShorEconetHeader
{
	unsigned char deststn;
	unsigned char destnet;
	unsigned char srcstn;
	unsigned char srcnet;
};

struct LongEconetPacket
{
	unsigned char deststn;
	unsigned char destnet;
	unsigned char srcstn;
	unsigned char srcnet;
	unsigned char cb;
	unsigned char port;
	// unsigned char buff[2];
};

// MC6854 has 3 byte FIFOs. There is no wait for an end of data
// before transmission starts. Data is sent immediately it's put into
// the first slot.

// Does Econet send multiple packets for big transfers, or just one huge
// packet?
// What's MTU on econet? Depends on clock speed but its big (e.g. 100K).
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

struct EthernetPacket
{
	union {
		unsigned char raw[8];
		AUNHeader ah;
	};

	union {
		unsigned char buff[ETHERNET_BUFFER_SIZE];
		ShorEconetHeader eh;
	};

	unsigned int Pointer;
	unsigned int BytesInBuffer;
	unsigned long inet_addr;
	unsigned int port;
	unsigned int deststn;
	unsigned int destnet;
};

// Buffers used to construct packets for sending out via UDP
static EthernetPacket EconetRx;
static EthernetPacket EconetTx;

// Buffers used to construct packets sent to/received from BBC micro

struct EconetPacket
{
	union {
		LongEconetPacket eh;
		unsigned char buff[ETHERNET_BUFFER_SIZE + 12];
	};

	unsigned int Pointer;
	unsigned int BytesInBuffer;
};

static EconetPacket BeebTx;
static EconetPacket BeebRx;

static unsigned char BeebTxCopy[sizeof(LongEconetPacket)];

// Holds data from Econet.cfg file
struct EconetHost {
	unsigned char station;
	unsigned char network;
	unsigned long inet_addr;
	u_short port;
};

struct EconetNet {
	unsigned long inet_addr;
	unsigned char network;
	u_short port; // AUN port or base port from which sequential ports are calculated
};

struct EconetGateway {
	unsigned long inet_addr;
	unsigned char network;
	u_short port;
};

struct NetStn {
	unsigned char network;
	unsigned char station;
};

static NetStn LastError;

const int NETWORK_TABLE_LENGTH = 512; // Total number of hosts we can know about
const int AUN_TABLE_LENGTH = 128; // number of disparate networks in AUNMap
static EconetHost stations[NETWORK_TABLE_LENGTH]; // individual stations we know about
static EconetNet networks[AUN_TABLE_LENGTH]; // AUN networks we know about
static EconetGateway gateways[AUN_TABLE_LENGTH]; // Extended AUN Gateways we know about

static int stationsp = 0; // How many individual stations do I know about?
static int networksp = 0;  // How many networks do I know about?
static int gatewaysp = 0; // How many gateways do I know about?


static unsigned char irqcause;   // flag to indicate cause of irq sr1b7
static unsigned char sr1b2cause; // flag to indicate cause of irq sr1b2

char EconetCfgPath[MAX_PATH];
char AUNMapPath[MAX_PATH];

// A receiving station goes into flag fill mode while it is processing
// a message.  This stops other stations sending messages that may interfere
// with the four-way handshake.  Attempting to notify evey station using
// IP messages when flag fill goes active/inactive would be complicated and
// would suffer from timing issues due to network latency, so a pseudo
// flag fill algorithm is emulated.  We assume that the receiving station
// will go into flag fill when we send a message or when we see a message
// destined for another station.  We cancel flag fill when we receive a
// message as the other station must have cancelled flag fill.  In order to
// cancel flag fill after the last message of a four-way handshake we time it
// out - which is not ideal as we do not want to delay new messages any
// longer that we have to - but it will have to do for now!

static bool FlagFillActive; // Flag fill state
int EconetFlagFillTimeoutTrigger; // Trigger point for flag fill
int EconetFlagFillTimeout = DEFAULT_FLAG_FILL_TIMEOUT; // Cycles for flag fill timeout // added cfg file to override this
static int EconetScoutAckTrigger; // Trigger point for scout ack
static int EconetScoutAckTimeout = DEFAULT_SCOUT_ACK_TIMEOUT; // Cycles to delay before sending ack to scout (AUN mode only)
static int EconetFourWayTrigger;

// Device and temp copy!
static MC6854 ADLC;
static MC6854 ADLCtemp;

//---------------------------------------------------------------------------

static bool ReadNetwork();
static bool EconetPoll_real();
static void DebugDumpADLC();
static void EconetError(const char *Format, ...);

//---------------------------------------------------------------------------

static bool IsBroadcastStation(unsigned int Station)
{
	return Station == 0 || Station == 255;
}

//---------------------------------------------------------------------------

static const char* IpAddressStr(unsigned long inet_addr)
{
	in_addr in;
	IN_ADDR(in) = inet_addr;

	return inet_ntoa(in);
}

//---------------------------------------------------------------------------

static std::string BytesToString(const unsigned char* pData, int Length)
{
	std::string str;

	for (int i = 0; i < Length; i++)
	{
		char sz[10];
		sprintf(sz, " %02X", pData[i]);

		str += sz;
	}

	return str;
}

//---------------------------------------------------------------------------

static EconetHost* FindNetworkConfig(unsigned char Station)
{
	for (int i = 0; i < stationsp; ++i)
	{
		if (stations[i].station == Station)
		{
			return &stations[i];
		}
	}

	return nullptr;
}

//---------------------------------------------------------------------------

static void EconetCloseSockets()
{
	// In single socket mode, SendSocket == ListenSocket
	if (SendSocket != INVALID_SOCKET && SendSocket != ListenSocket)
	{
		CloseSocket(SendSocket);
	}

	if (ListenSocket != INVALID_SOCKET)
	{
		CloseSocket(ListenSocket);
	}

	SendSocket = INVALID_SOCKET;
	ListenSocket = INVALID_SOCKET;

	ReceiverSocketsOpen = false;
}

//---------------------------------------------------------------------------

bool EconetReset()
{
	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet, true, "Econet: Reset (hardware %s)",
		                   EconetEnabled ? "enabled" : "disabled");
	}

	// hardware operations:
	// set RxReset and TxReset
	ADLC.control1 = CONTROL_REG1_RX_RESET | CONTROL_REG1_TX_RESET;
	// reset TxAbort, RTS, LoopMode, DTR
	ADLC.control4 = 0; //ADLC.control4 & 223;
	ADLC.control2 = 0; //ADLC.control2 & 127;
	ADLC.control3 = 0; //ADLC.control3 & 95;

	// clear all status conditions
	ADLC.status1 = 0; // cts - clear to send line input (no collissions talking udp)
	ADLC.status2 = 0; // dcd - no clock (until sockets initialised and open)
	ADLC.sr2pse = 0;

	//software stuff:
	EconetRx.Pointer = 0;
	EconetRx.BytesInBuffer = 0;
	EconetTx.Pointer = 0;
	EconetTx.BytesInBuffer = 0;

	BeebRx.Pointer = 0;
	BeebRx.BytesInBuffer = 0;
	BeebTx.Pointer = 0;
	BeebTx.BytesInBuffer = 0;

	fourwaystage = FourWayStage::Idle; // used for AUN mode translation stage.

	ADLC.rxfptr = 0;
	ADLC.rxap = 0;
	ADLC.rxffc = 0;
	ADLC.txfptr = 0;
	ADLC.txftl = 0;

	ADLC.idle = true;
	ADLC.cts = false;

	irqcause = 0;
	sr1b2cause = 0;

	FlagFillActive = false;
	EconetFlagFillTimeoutTrigger = 0;

	// Kill anything that was in use
	EconetCloseSockets();

	// Stop here if not enabled
	if (!EconetEnabled)
	{
		return true;
	}

	// Read in Econet.cfg and AUNMap. Done here so can refresh it on Break.
	if (!ReadNetwork())
	{
		goto Fail;
	}

	// Create a SOCKET for listening for incoming connection requests.
	ListenSocket = socket(AF_INET, SOCK_DGRAM, 0);

	if (ListenSocket == INVALID_SOCKET)
	{
		EconetError("Econet: Failed to open listening socket (error %ld)", GetLastSocketError());
		goto Fail;
	}

	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	sockaddr_in service;
	service.sin_family = AF_INET;
	service.sin_addr.s_addr = INADDR_ANY; //inet_addr("127.0.0.1");

	// Already have a station num? Either from command line or a free one
	// we found on previous reset.
	if (EconetStationID != 0)
	{
		// Look up our port number in network config
		EconetHost* pNetworkConfig = FindNetworkConfig(EconetStationID);

		if (pNetworkConfig != nullptr)
		{
			EconetListenPort = pNetworkConfig->port;
			EconetListenIP = pNetworkConfig->inet_addr;
			myaunnet = pNetworkConfig->network;
		}
		else
		{
			EconetError("Econet: Failed to find station %d in Econet.cfg", EconetStationID);
			goto Fail;
		}

		service.sin_port = htons(EconetListenPort);
		S_ADDR(service) = EconetListenIP;

		if (bind(ListenSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR)
		{
			EconetError("Econet: Failed to bind to port %d (error %ld)", EconetListenPort, GetLastSocketError());
			goto Fail;
		}
	}
	else
	{
		// Station number not specified, find first one not already in use.
		char localhost[256];
		hostent *host;

		// Get localhost IP address
		if (gethostname(localhost, 256) != SOCKET_ERROR &&
		    (host = gethostbyname(localhost)) != NULL)
		{
			// See if configured addresses match local IPs
			for (int i = 0; i < stationsp && EconetStationID == 0; ++i)
			{
				// Check address for each network interface/card
				for (int a = 0; host->h_addr_list[a] != nullptr && EconetStationID == 0; ++a)
				{
					struct in_addr localaddr;
					memcpy(&localaddr, host->h_addr_list[a], sizeof(struct in_addr));

					if (stations[i].inet_addr == inet_addr("127.0.0.1") ||
					    stations[i].inet_addr == IN_ADDR(localaddr))
					{
						service.sin_port = htons(stations[i].port);
						S_ADDR(service) = stations[i].inet_addr;

						if (bind(ListenSocket, (SOCKADDR*)&service, sizeof(service)) == 0)
						{
							EconetListenPort = stations[i].port;
							EconetListenIP = stations[i].inet_addr;
							EconetStationID = stations[i].station;
							myaunnet = stations[i].network;
						}
					}
				}
			}

			if (EconetListenPort == 0)
			{
				// Still can't find one ... strict mode?

				if (AUNMode && StrictAUNMode && stationsp < NETWORK_TABLE_LENGTH)
				{
					if (DebugEnabled)
						DebugDisplayTrace(DebugType::Econet, true, "Econet: No free hosts in table; trying automatic mode");

					for (int j = 0; j < networksp && EconetStationID == 0; j++)
					{
						for (int a = 0; host->h_addr_list[a] != NULL && EconetStationID == 0; ++a)
						{
							struct in_addr localaddr;
							memcpy(&localaddr, host->h_addr_list[a], sizeof(struct in_addr));

							if (networks[j].inet_addr == (IN_ADDR(localaddr) & 0x00FFFFFF))
							{
								service.sin_port = htons(DEFAULT_AUN_PORT);
								S_ADDR(service) = IN_ADDR(localaddr);

								if (bind(ListenSocket, (SOCKADDR*)&service, sizeof(service)) == 0)
								{

									EconetListenIP = IN_ADDR(localaddr);
									EconetListenPort = DEFAULT_AUN_PORT;
									EconetStationID = IN_ADDR(localaddr) >> 24;
									myaunnet = networks[j].network;

									// add ourself to the list of known stations
									// TODO: is this a good idea?
									stations[stationsp].inet_addr = EconetListenIP;
									stations[stationsp].port = EconetListenPort;
									stations[stationsp].station = EconetStationID;
									stations[stationsp].network = networks[j].network;
									stationsp++;
								}
							}
						}
					}
				}

				if (EconetStationID == 0)
				{
					EconetError("Econet: Failed to find free station/port to bind to");
					goto Fail;
				}
			}
		}
		else
		{
			EconetError("Econet: Failed to resolve local IP address");
			goto Fail;
		}
	}

	if (DebugEnabled) {
		DebugDisplayTraceF(DebugType::Econet, true,
		                   "Econet: Station number set to %d, port %d",
		                   EconetStationID, EconetListenPort);
	}

	// On Master the station number is read from CMOS so update it
	if (MachineType == Model::Master128 || MachineType == Model::MasterET)
	{
		RTCWriteAddress(0xE);
		RTCWriteData(EconetStationID);
	}

	// Socket used to send messages.
	if (SingleSocket)
	{
		SendSocket = ListenSocket;
	}
	else
	{
		SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

		if (SendSocket == INVALID_SOCKET)
		{
			EconetError("Econet: Failed to open sending socket (error %ld)", GetLastSocketError());
			goto Fail;
		}
	}

	// This call is what allows broadcast packets to be sent:
	const char broadcast = '1';

	if (setsockopt(SendSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1)
	{
		EconetError("Econet: Failed to set socket for broadcasts (error %ld)", GetLastSocketError());
		goto Fail;
	}

	ReceiverSocketsOpen = true;

	// how long before we bother with poll routine?
	SetTrigger(TimeBetweenBytes, EconetTrigger);

	EconetStateChanged = true;

	return true;

Fail:
	EconetCloseSockets();

	EconetEnabled = false;
	return false;
}

//---------------------------------------------------------------------------

static void ParseConfigLine(const std::string& Line, std::vector<std::string>& Tokens)
{
	int i = 0;

	while (Line[i] != '\0')
	{
		std::string Token;

		while (Line[i] != '\0' && isspace(Line[i]))
		{
			i++;
		}

		while (Line[i] != '\0' && !isspace(Line[i]))
		{
			Token += Line[i++];
		}

		if (!Token.empty())
		{
			Tokens.push_back(Token);
		}
	}
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

	stationsp = 0;

	std::string Line;
	int LineCounter = 0;

	while (std::getline(Input, Line))
	{
		LineCounter++;

		trim(Line);

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

		ParseConfigLine(Line, Tokens);

		if (Tokens.size() == 4)
		{
			if (stationsp < NETWORK_TABLE_LENGTH)
			{
				try
				{
					unsigned char network = (unsigned char)std::stoi(Tokens[0]);
					unsigned long address = inet_addr(Tokens[2].c_str());
					u_short port          = (u_short)std::stoi(Tokens[3]);
					
					if (Tokens[1] == "*")
					{
						// this line defines an entire network
						if (network >= 253)
							throw std::out_of_range ("invalid network");
							
						// it is added before any networks from AUNMap so takes precedence
						networks[networksp].network = network;
						networks[networksp].inet_addr = address;
						networks[networksp].port = port;
						
						DebugDisplayTraceF(DebugType::Econet, true,
						                   "Econet: ConfigFile Net %i IP %s Port %i",
						                   networks[networksp].network,
						                   IpAddressStr(networks[networksp].inet_addr),
						                   networks[networksp].port);
						
						networks[++networksp].network = 255; // terminate list with invalid net
					}
					else if (Tokens[1] == "G")
					{
						// this line defines an Extended AUN gateway
						if (network >= 253)
							throw std::out_of_range ("invalid network");
						// a gateway is allowed to specify network zero, in which case it will match all otherwise unknown nets
						
						gateways[gatewaysp].network = network;
						gateways[gatewaysp].inet_addr = address;
						gateways[gatewaysp].port = port;
						
						DebugDisplayTraceF(DebugType::Econet, true,
						                   "Econet: ConfigFile Gateway %i IP %s Port %i",
						                   gateways[gatewaysp].network,
						                   IpAddressStr(gateways[gatewaysp].inet_addr),
						                   gateways[gatewaysp].port);
						
						gateways[++gatewaysp].network = 255; // terminate list with invalid net
					}
					else
					{
						unsigned char station = (unsigned char)std::stoi(Tokens[1]);
						if (station == 0 || station == 255)
							throw std::out_of_range ("invalid station"); // not a valid station number
						
						stations[stationsp].network = network;
						stations[stationsp].station = station;
						stations[stationsp].inet_addr = address;
						stations[stationsp].port = port;
						
						DebugDisplayTraceF(DebugType::Econet, true,
						                   "Econet: ConfigFile Net %i Stn %i IP %s Port %i",
						                   stations[stationsp].network, stations[stationsp].station,
						                   IpAddressStr(stations[stationsp].inet_addr), stations[stationsp].port);
						
						stations[++stationsp].station = 0; // terminate list with invalid station
					}
				}
				catch (const std::exception&)
				{
					EconetError("Invalid value in Econet config file:\n  %s (Line %d)", EconetCfgPath, LineCounter);
					Success = false;
					break;
				}
			}
			else
			{
				EconetError("Too many network entries in Econet config file:\n  %s (Line %d)", EconetCfgPath, LineCounter);
				Success = false;
				break;
			}
		}
		else if (Tokens.size() == 2)
		{
			const std::string& Key   = Tokens[0];
			const std::string& Value = Tokens[1];

			try
			{
				if (StrCaseCmp(Key.c_str(), "AUNMODE") == 0)
				{
					AUNMode = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "LEARN") == 0)
				{
					// LearnMode removed 2025-06-21
				}
				else if (StrCaseCmp(Key.c_str(), "AUNSTRICT") == 0)
				{
					StrictAUNMode = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "SINGLESOCKET") == 0)
				{
					SingleSocket = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "FLAGFILLTIMEOUT") == 0)
				{
					EconetFlagFillTimeout = std::stoi(Value);
				}
				else if (StrCaseCmp(Key.c_str(), "SCACKTIMEOUT") == 0 ||
				         StrCaseCmp(Key.c_str(), "SCOUTACKTIMEOUT") == 0)
				{
					EconetScoutAckTimeout = std::stoi(Value);
				}
				else if (StrCaseCmp(Key.c_str(), "TIMEBETWEENBYTES") == 0)
				{
					TimeBetweenBytes = std::stoi(Value);
				}
				else if (StrCaseCmp(Key.c_str(), "FOURWAYTIMEOUT") == 0)
				{
					FourWayStageTimeout = std::stoi(Value);
				}
				else if (StrCaseCmp(Key.c_str(), "MASSAGENETS") == 0)
				{
					MassageNetworks = std::stoi(Value) != 0;
				}
				else
				{
					EconetError("Unknown entry in Econet config file: %s\n  %s (Line %d)", Key.c_str(), EconetCfgPath, LineCounter);
				}
			}
			catch (const std::exception&)
			{
				EconetError("Invalid value in Econet config file: %s\n  %s (Line %d)", Value.c_str(), EconetCfgPath, LineCounter);
				Success = false;
				break;
			}
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

		trim(Line);

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

		ParseConfigLine(Line, Tokens);

		if (Tokens.size() == 3 && StrCaseCmp("ADDMAP", Tokens[0].c_str()) == 0)
		{
			if (networksp < AUN_TABLE_LENGTH)
			{
				try
				{
					networks[networksp].inet_addr = inet_addr(Tokens[1].c_str()) & 0x00FFFFFF; // stored as lsb..msb ?!?!
					networks[networksp].network   = (unsigned char)(std::stoi(Tokens[2]));
					networks[networksp].port      = DEFAULT_AUN_PORT; // always use the default port for proper AUN networks

					if (DebugEnabled)
					{
						DebugDisplayTraceF(DebugType::Econet, true,
						                   "Econet: AUNMap Net %i IP %08x",
						                   networks[networksp].network, networks[networksp].inet_addr);
					}

					// Note which network we are a part of. This won't work on first run as EconetListenIP not set!
					if (networks[networksp].inet_addr == (EconetListenIP & 0x00FFFFFF))
					{
						myaunnet = networks[networksp].network;

						if (DebugEnabled)
						{
							DebugDisplayTrace(DebugType::Econet, true, "Econet: ..and that's the one we're in");
						}
					}

					networksp++;
				}
				catch (const std::exception&)
				{
					EconetError("Invalid value in Econet config file:\n  %s (Line %d)", EconetCfgPath, LineCounter);
					Success = false;
					break;
				}
			}
			else
			{
				EconetError("Too many entries in Econet config file:\n  %s (Line %d)", EconetCfgPath, LineCounter);
				Success = false;
				break;
			}
		}
	}

	networks[networksp].network = 255; // terminate table with invalid net.

	return Success;
}

//---------------------------------------------------------------------------

static bool ReadNetwork()
{
	AUNMode = DEFAULT_AUN_MODE;
	StrictAUNMode = DEFAULT_STRICT_AUN_MODE;
	SingleSocket = DEFAULT_SINGLE_SOCKET;
	EconetFlagFillTimeout = DEFAULT_FLAG_FILL_TIMEOUT;
	EconetScoutAckTimeout = DEFAULT_SCOUT_ACK_TIMEOUT;
	TimeBetweenBytes = DEFAULT_TIME_BETWEEN_BYTES;
	FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
	MassageNetworks = DEFAULT_MASSAGE_NETWORKS;

	// clear tables
	stationsp = 0;
	stations[0].station = 0;
	networksp = 0;
	networks[0].network = 255;
	gatewaysp = 0;
	gateways[0].network = 255;

	if (!ReadEconetConfigFile())
	{
		return false;
	}

	// Don't bother reading file if not using AUN.
	if (AUNMode)
	{
		return ReadAUNConfigFile();
	}

	return true;
}

//---------------------------------------------------------------------------
// read to FE18..

unsigned char EconetReadStationID()
{
	//if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet, true,
		                   "Econet: Read Station: %d",
		                   (int)EconetStationID);
	}

	return EconetStationID;
}

//---------------------------------------------------------------------------
// read to FEA0-3

unsigned char EconetRead(unsigned char Register)
{
	//if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet, true,
		                   "Econet: Read ADLC %02X",
		                   (int)Register);
		DebugDumpADLC();
	}

	if (Register == 0)
	{
		return ADLC.status1;
	}
	else if (Register == 1)
	{
		return ADLC.status2;
	}
	else
	{
		// rxreset not set and someting in fifo
		if (((ADLC.control1 & CONTROL_REG1_RX_RESET) == 0) && ADLC.rxfptr > 0)
		{
			//if (DebugEnabled)
			{
				DebugDisplayTraceF(DebugType::Econet, true,
				                   "Econet: Returned fifo: %02X",
				                   (int)ADLC.rxfifo[ADLC.rxfptr - 1]);
				DebugDumpADLC();
			}

			if (ADLC.rxfptr > 0)
			{
				EconetStateChanged = true;
				return ADLC.rxfifo[--ADLC.rxfptr]; // read rx buffer
			}
			else
			{
				return 0;
			}
		}
	}

	return 0;
}

//---------------------------------------------------------------------------
// write to FEA0-3

void EconetWrite(unsigned char Register, unsigned char Value)
{
	//if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet, true,
		                   "Econet: Write ADLC %02X = %02X",
		                   (int)Register, (int)Value);
	}

	// Command registers are really just a set of flags that affect
	// operation of the rest of the device.

	if (Register == 0)
	{
		ADLC.control1 = Value;
	}
	else if (Register == 1 && !(ADLC.control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.control2 = Value;
	}
	else if (Register == 1 && (ADLC.control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.control3 = Value;
	}
	else if (Register == 3 && (ADLC.control1 & CONTROL_REG1_ADDRESS_CONTROL))
	{
		ADLC.control4 = Value;
	}
	else if (Register == 2 || Register == 3) // adr 02 or adr 03 & AC=0
	{
		// cannot write an output byte if txreset is set
		// register 2 is an output byte
		// register 3 with c1b0=0 is output byte & finalise tx.
		// can also finalise tx by setting a control bit.so do that automatically for reg 3
		// worry about actually sending stuff in the poll routines, not here.
		if ((ADLC.control1 & CONTROL_REG1_TX_RESET) == 0)
		{
			ADLC.txfifo[2] = ADLC.txfifo[1];
			ADLC.txfifo[1] = ADLC.txfifo[0];
			ADLC.txfifo[0] = Value;
			ADLC.txfptr++;
			ADLC.txftl = ADLC.txftl << 1; // shift txlast bits up.

			if (Register == 3)
			{
				ADLC.control2 |= CONTROL_REG2_TX_LAST_DATA; // set txlast control flag ourself
			}
		}
	}

	if (DebugEnabled) DebugDumpADLC();

	EconetStateChanged = true;
}

//--------------------------------------------------------------------------------------------

bool EconetInterruptRequest()
{
	return (ADLC.status1 & STATUS_REG1_IRQ) != 0;
}

//--------------------------------------------------------------------------------------------
// Optimisation - only call real poll routine when something has changed

bool EconetPoll() // return NMI status
{
	if (EconetStateChanged || EconetTrigger <= TotalCycles)
	{
		EconetStateChanged = false;

		// Don't poll if failed to init sockets
		if (ReceiverSocketsOpen)
		{
			return EconetPoll_real();
		}
	}

	return false;
}

//--------------------------------------------------------------------------------------------
// Run when state changed or time to check comms.
// The majority of this code is to handle the status registers.
// These are just flags that depend on the tx & rx status, and the control flags.
// These change immediately anything happens, so need refreshing all the time,
// as rx and tx operations can depend on them too.  It /might/ be possible to
// only re-calculate them when needed (e.g. on a memory-read or in the receive
// routines before they are checked) but for the moment I just want to get this
// code actually working!

bool EconetPoll_real() // return NMI status
{
	bool interruptnow = false;

	// save flags
	ADLCtemp.status1 = ADLC.status1;
	ADLCtemp.status2 = ADLC.status2;

	// okie dokie.  This is where the brunt of the ADLC emulation & network handling will happen.

	// look for control bit changes and take appropriate action

	// CR1b0 - Address Control - only used to select between register 2/3/4
	//         no action needed here
	// CR1b1 - RIE - Receiver Interrupt Enable - Flag to allow receiver section to create interrupt.
	//         no action needed here
	// CR1b2 - TIE - Transmitter Interrupt Enable - ditto
	//         no action needed here
	// CR1b3 - RDSR mode. When set, interrupts on received data are inhibited.
	//         unsupported - no action needed here
	// CR1b4 - TDSR mode. When set, interrupts on trasmit data are inhibited.
	//         unsupported - no action needed here
	// CR1b5 - Discontinue - when set, discontinue reception of incoming data.
	//         automatically reset this when reach the end of current frame in progress
	//         automatically reset when frame aborted bvy receiving an abort flag, or DCD fails
	if (ADLC.control1 & CONTROL_REG1_RX_FRAME_DISCONTINUE)
	{
		if (DebugEnabled) DebugDisplayTrace(DebugType::Econet, true, "EconetPoll: RxABORT is set");
		BeebRx.Pointer = 0;
		BeebRx.BytesInBuffer = 0;
		ADLC.rxfptr = 0;
		ADLC.rxap = 0;
		ADLC.rxffc = 0;
		ADLC.control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE; // reset flag
		fourwaystage = FourWayStage::Idle;
	}
	// CR1b6 - RxRs - Receiver reset. set by cpu or when reset line goes low.
	//         all receive operations blocked (bar dcd monitoring) when this is set.
	//         see CR2b5
	// CR1b7 - TxRS - Transmitter reset. set by cpu or when reset line goes low.
	//         all transmit operations blocked (bar cts monitoring) when this is set.
	//         no action needed here; watch this bit elsewhere to inhibit actions

	// CR2b0 - PSE - priotitised status enable - adjusts how status bits show up.
	//         See sr2pse and code in status section
	// CR2b1 - 2byte/1byte mode.  set to indicate 2 byte mode. see trda status bit.
	// CR2b2 - Flag/Mark idle select. What is transmitted when tx idle. ignored here as not needed
	// CR2b3 - FC/TDRA mode - does status bit SR1b6 indicate 1=frame complete,
	//         0=tx data reg available. 1=frame tx complete.  see tdra status bit
	// CR2b4 - TxLast - byte just put into fifo was the last byte of a packet.
	if (ADLC.control2 & CONTROL_REG2_TX_LAST_DATA)
	{
		ADLC.txftl |= 1; // set b0 - flag for fifo[0]
		ADLC.control2 &= ~CONTROL_REG2_TX_LAST_DATA; // clear flag.
	}

	// CR2b5 - CLR RxST - Clear Receiver Status - reset status bits
	if ((ADLC.control2 & CONTROL_REG2_CLEAR_RX_STATUS) || (ADLC.control1 & CONTROL_REG1_RX_RESET)) // or rxreset
	{
		ADLC.control2 &= ~CONTROL_REG2_CLEAR_RX_STATUS; // clear this bit

		ADLC.status1 &= ~(STATUS_REG1_STATUS2_READ_REQUEST | STATUS_REG1_FLAG_DETECTED); // clear sr2rq, FD

		// clear FV, RxIdle, RxAbt, Err, OVRN, DCD
		ADLC.status2 &= ~(STATUS_REG2_FRAME_VALID |
		                  STATUS_REG2_INACTIVE_IDLE_RECEIVED |
		                  STATUS_REG2_ABORT_RECEIVED |
		                  STATUS_REG2_FCS_ERROR |
		                  STATUS_REG2_DCD |
		                  STATUS_REG2_RX_OVERRUN);

		if ((ADLC.control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE) && ADLC.sr2pse > 0) // PSE active?
		{
			ADLC.sr2pse++; // Advance PSE to next priority
			if (ADLC.sr2pse > 4)
				ADLC.sr2pse = 0;
		}
		else
		{
			ADLC.sr2pse = 0;
		}

		sr1b2cause = 0; // clear cause of sr2b1 going up

		if (ADLC.control1 & CONTROL_REG1_RX_RESET) // rx reset,clear buffers.
		{
			BeebRx.Pointer = 0;
			BeebRx.BytesInBuffer = 0;
			ADLC.rxfptr = 0;
			ADLC.rxap = 0;
			ADLC.rxffc = 0;
			ADLC.sr2pse = 0;
		}
		// fourwaystage = FourWayStage::Idle; // this really doesn't like being here.
	}

	// CR2b6 - CLT TxST - Clear Transmitter Status - reset status bits
	if ((ADLC.control2 & CONTROL_REG2_CLEAR_TX_STATUS) || (ADLC.control1 & CONTROL_REG1_TX_RESET)) // or txreset
	{
		ADLC.control2 &= ~CONTROL_REG2_CLEAR_TX_STATUS; // clear this bit
		ADLC.status1 &= ~(STATUS_REG1_CTS |
		                  STATUS_REG1_TX_UNDERRUN |
		                  STATUS_REG1_TDRA); // clear TXU , cts, TDRA/FC

		if (ADLC.cts)
		{
			ADLC.status1 |= STATUS_REG1_CTS; // cts follows signal, reset high again
			ADLCtemp.status1 |= STATUS_REG1_CTS; // don't trigger another interrupt instantly
		}

		if (ADLC.control1 & CONTROL_REG1_TX_RESET) // tx reset,clear buffers.
		{
			BeebTx.Pointer = 0;
			BeebTx.BytesInBuffer = 0;
			ADLC.txfptr = 0;
			ADLC.txftl = 0;
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
	// CR3b4 - FDSE - flag detect status enable.  when set, then FD (SR1b3) + interrupr indicated a flag
	// has been received. I don't think we use this mode, so ignoring it.
	// CR3b5 - Loop - Loop mode. Not used.
	// CR3b6 - GAP/TST - sets test loopback mode (when not in Loop operation mode.) ignored.
	// CR3b7 - LOC/DTR - (when not in loop mode) controls DTR pin directly. pin not used in a BBC B

	// CR4b0 - FF/F - when clear, re-used the Flag at end of one packet as start of next packet. ignored.
	// CR4b1,2 - TX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b3,4 - RX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b5 - TransmitABT - Abort Transmission.  Once abort starts, bit is cleared.
	if (ADLC.control4 & CONTROL_REG4_TX_ABORT)
	{
		//if (DebugEnabled)
			DebugDisplayTrace(DebugType::Econet, true, "EconetPoll: TxABORT is set");

		ADLC.txfptr = 0; // reset fifo
		ADLC.txftl = 0; // reset fifo flags
		BeebTx.Pointer = 0;
		BeebTx.BytesInBuffer = 0;
		ADLC.control4 &= ~CONTROL_REG4_TX_ABORT; // reset flag.
		fourwaystage = FourWayStage::Idle;

		//if (DebugEnabled)
			DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_IDLE (abort)");
	}

	// CR4b6 - ABTex - extend abort - adjust way the abort flag is sent.  ignore.
	// can affect timing of RTS output line (and thus CTS input) still ignored.
	// CR4b7 - NRZI/NRZ - invert data encoding on wire. ignore.

	if (EconetTrigger <= TotalCycles)
	{
		// Only do this bit occasionally as data only comes in from
		// line occasionally.
		// Trickle data between fifo registers and ip packets.

		// Transmit data
		if (!(ADLC.control1 & CONTROL_REG1_TX_RESET)) // tx reset off
		{
			if (ADLC.txfptr > 0) // there is data in tx fifo
			{
				//if (DebugEnabled)
					DebugDisplayTrace(DebugType::Econet, true, "EconetPoll: Write to FIFO noticed");

				bool TXlast = false;

				if (ADLC.txftl & powers[ADLC.txfptr - 1]) // TxLast set
				{
					TXlast = true;
				}

				if (BeebTx.Pointer + 1 > sizeof(BeebTx.buff) || // overflow IP buffer
				    (ADLC.txfptr > 4)) // overflowed fifo
				{
					ADLC.status1 |= STATUS_REG1_TX_UNDERRUN; // set tx underrun flag
					BeebTx.Pointer = 0; // wipe buffer
					BeebTx.BytesInBuffer = 0;
					ADLC.txfptr = 0;
					ADLC.txftl = 0;

					if (DebugEnabled)
						DebugDisplayTrace(DebugType::Econet, true, "EconetPoll: TxUnderun!!");
				}
				else
				{
					BeebTx.buff[BeebTx.Pointer] = ADLC.txfifo[--ADLC.txfptr];
					BeebTx.Pointer++;
				}

				if (TXlast) // TxLast set
				{
					if (DebugEnabled)
					{
						DebugDisplayTraceF(DebugType::Econet, true,
						                   "Econet: TXLast set - Send packet to network %d station %d",
						                   (int)BeebTx.eh.destnet,
						                   (int)BeebTx.eh.deststn);
					}

					// first two bytes of BeebTx.buff contain the destination address
					// (or one zero byte for broadcast)

					sockaddr_in RecvAddr;
					RecvAddr.sin_family = AF_INET;
					
					bool SendMe = false;
					bool ExtendedAUN = false;
					int SendLen = 0;

					if (IsBroadcastStation(BeebTx.eh.deststn))
					{
						// Set address to the local broadcast address and port to the default AUN port
						// to do an AUN broadcast. Some BeebEm instances might not see this so we will
						// send unicast copies to those that need them later
						S_ADDR(RecvAddr) = INADDR_BROADCAST; // ((EconetListenIP & 0x00FFFFFF) | 0xFF000000);
						RecvAddr.sin_port = htons(DEFAULT_AUN_PORT);
						SendMe = true;
					}
					else
					{
						// Work out where we need to send a packet for this econet address
						
						unsigned int mask = MassageNetworks ? 0x7F : 0xFF; // match AUN nets for econet network numbers if MassageNetworks is enabled.
						
						// search for a single specifically defined host in stations table
						for (int i = 0; i < stationsp; i++)
						{
							if ((stations[i].network & mask) == (BeebTx.eh.destnet & mask) &&
							    stations[i].station == BeebTx.eh.deststn)
							{
								S_ADDR(RecvAddr) = stations[i].inet_addr;
								RecvAddr.sin_port = htons(stations[i].port);
								SendMe = true;
								break;
							}
						}
						
						if (!SendMe)
						{
							// didn't find the station
							// search to see if the destination network is defined in networks table
							for (int i = 0; i < networksp; i++)
							{
								if ((networks[i].network & mask) == (BeebTx.eh.destnet & mask))
								{
									// located the network
									if ((networks[i].inet_addr & 0xFF000000) == 0)
									{
										// last octet is zero so this is true AUN
										S_ADDR(RecvAddr) = (networks[i].inet_addr & 0x00FFFFFF) | (BeebTx.eh.deststn << 24);
										RecvAddr.sin_port = htons(networks[i].port); // TODO this should always be DEFAULT_AUN_PORT - should we override this?
										SendMe = true;
										break;
									}
									else
									{
										// whole network defined with a single address
										// treat port as the base port number for a PiEB exposed network
										S_ADDR(RecvAddr) = networks[i].inet_addr;
										RecvAddr.sin_port = htons(networks[i].port + (BeebTx.eh.destnet << 8) + BeebTx.eh.deststn);
										SendMe = true;
										break;
									}
								}
							}
						}
						
						if (!SendMe && BeebTx.eh.destnet != 0 && BeebTx.eh.destnet != 255)
						{
							// didn't find the network and it is not for net 0 or 255
							// search for a gateway which can get packets to this network
							
							for (int i = 0; i < gatewaysp; i++)
							{
								if (gateways[i].network == BeebTx.eh.destnet || gateways[i].network == 0)
								{
									// A gateway defined with network 0 matches all networks so will send anything we weren't able to find
									
									S_ADDR(RecvAddr) = gateways[i].inet_addr;
									RecvAddr.sin_port = htons(gateways[i].port);
									
									ExtendedAUN = true; // we need to send Extended AUN to this port
									SendMe = true;
									break;
								}
							}
						}
					}

					// Send a datagram to the receiver
					if (SendMe)
					{
						DebugDisplayTraceF(DebugType::Econet, true,
										   "Econet: TXLast set: Send %d byte packet to network %d station %d (%s port %u)",
										   BeebTx.Pointer,
										   (unsigned int)BeebTx.eh.destnet,
										   (unsigned int)BeebTx.eh.deststn,
										   IpAddressStr(S_ADDR(RecvAddr)),
										   (unsigned int)htons(RecvAddr.sin_port));

						std::string str = "Econet: Packet data:" + BytesToString(BeebTx.buff, BeebTx.Pointer);

						DebugDisplayTrace(DebugType::Econet, true, str.c_str());
						
						LastError.network = 0; // reset the network & station where the last send error occurred
						LastError.station = 0;

						//if (DebugEnabled)
							DebugDisplayTrace(DebugType::Econet, true, "Econet: Sending a packet..");

						if (AUNMode)
						{
							unsigned int j = 0;
							// OK. Lets do AUN ...
							// The beeb has given us a packet .. what is it?
							SendMe = false;

							switch (fourwaystage)
							{
							case FourWayStage::ScoutAckReceived:
								// it came in response to our ack of a scout
								// what we have /should/ be the data block ..
								// CLUDGE WARNING is this a scout sent again immediately?? TODO fix this?!?!
								if (EconetTx.ah.port == 0x00) {
									if (EconetTx.ah.cb == (0x82 & 0x7f))
										j = 8;
									else if (EconetTx.ah.cb >= (0x83 & 0x7f) && EconetTx.ah.cb <= (0x85 & 0x7f))
										j = 4;
								}

								if (BeebTx.Pointer != sizeof(BeebTx.eh) + j || memcmp(BeebTx.buff, BeebTxCopy, sizeof(BeebTx.eh) + j) != 0) { // nope
									// j = 0;
									for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
										EconetTx.buff[j] = BeebTx.buff[k];
									}
									EconetTx.Pointer = j;
									fourwaystage = FourWayStage::DataSent;
									if (DebugEnabled) DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_DATASENT");
									SendMe = true;
									SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;
									break;
								} // else fall through...

							case FourWayStage::Idle:
								// not currently doing anything, so this will be a scout,
								memcpy(BeebTxCopy, BeebTx.buff, sizeof(BeebTx.eh));
								// maybe a long scout or a broadcast
								EconetTx.ah.cb = (unsigned int)(BeebTx.eh.cb) & 127; // | 128;
								EconetTx.ah.port = (unsigned int)BeebTx.eh.port;
								EconetTx.ah.pad = 0;
								EconetTx.ah.handle = (ec_sequence += 4);

								EconetTx.destnet = BeebTx.eh.destnet; //30JUN
								EconetTx.deststn = BeebTx.eh.deststn;
								// j = 0;
								for (unsigned int k = 6; k < BeebTx.Pointer; k++, j++) {
									EconetTx.buff[j] = BeebTx.buff[k];
								}

								EconetTx.Pointer = j;

								if (IsBroadcastStation(EconetTx.deststn))
								{
									EconetTx.ah.type = AUNType::Broadcast;
									fourwaystage = FourWayStage::WaitForIdle; // no response to broadcasts...
									//if (DebugEnabled)
									DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (broadcast snt)");
									SendMe = true; // send packet ...
									SendLen = sizeof(EconetTx.ah) + 8;
								}
								else if (EconetTx.ah.port == 0 && (EconetTx.ah.cb < (0x82 & 0x7f) || EconetTx.ah.cb >(0x85 & 0x7f)))
								{
									EconetTx.ah.type = AUNType::Immediate;
									fourwaystage = FourWayStage::ImmediateSent;
									//if (DebugEnabled)
									DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_IMMSENT");
									SendMe = true; // send packet ...
									SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;
								}
								else
								{
									EconetTx.ah.type = AUNType::Unicast;
									fourwaystage = FourWayStage::ScoutSent;
									//if (DebugEnabled)
									DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_SCOUTSENT");
									// dont send anything but set wait anyway
									SetTrigger(EconetScoutAckTimeout, EconetScoutAckTrigger);
									//if (DebugEnabled)
									DebugDisplayTrace(DebugType::Econet, true, "Econet: SCACKtimer set");
								} // else BROADCAST !!!!
								break;

							case FourWayStage::ScoutReceived:
								// it's an ack for a scout which we sent the beeb. just drop it, but move on.
								fourwaystage = FourWayStage::ScoutAckSent;
								//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_SCACKSENT");
								SetTrigger(EconetScoutAckTimeout, EconetScoutAckTrigger);
								//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: SCACKtimer set");
								break;

							case FourWayStage::DataReceived:
								// this must be ack for data just receved
								// now we really need to send an ack to the far AUN host...
								// send header of last block received straight back.
								// this ought to work, but only because the beeb can only talk to one machine at any time..
								SendLen = sizeof(EconetRx.ah);
								EconetTx.ah = EconetRx.ah;
								EconetTx.ah.type = AUNType::Ack;
								SendMe = true;

								fourwaystage = FourWayStage::WaitForIdle;
								//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (final ack sent)");
								break;

							case FourWayStage::ImmediateReceived:
								// it's a reply to an immediate command we just had
								fourwaystage = FourWayStage::WaitForIdle;
								//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (imm rcvd)");
								// j = 0;
								for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
									EconetTx.buff[j] = BeebTx.buff[k];
								}
								EconetTx.Pointer = j;
								EconetTx.ah = EconetRx.ah;
								EconetTx.ah.type = AUNType::ImmReply;
								SendMe = true;
								SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;
								break;

							default:
								// shouldn't be here.. ignore packet and abort fourway
								fourwaystage = FourWayStage::WaitForIdle;
								//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (unexpected mode, packet ignored)");
								break;
							}

							if (SendMe)
							{
								char *p = (char *)&EconetTx;
								if (ExtendedAUN)
								{
									// we need to make a copy of the packet with  additional addressing on the front
									unsigned char tmp[ETHERNET_BUFFER_SIZE+16];
									int i = 0;
									tmp[i++] = BeebTx.eh.deststn;
									tmp[i++] = BeebTx.eh.destnet;
									tmp[i++] = 0; // srcstn will be determined by the bridge
									tmp[i++] = 0; // srcnet will be determined by the bridge
									memcpy(tmp+i, &EconetTx.raw, SendLen);
									SendLen += 4;
									p = (char *)&tmp; // transmit this buffer instead of EconetTx
								}
								
								if (!(S_ADDR(RecvAddr) == EconetListenIP && htons(RecvAddr.sin_port) == EconetListenPort)) // never send to ourself
								{
									if (sendto(SendSocket, p, SendLen, 0,
											   (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
									{
										EconetError("Econet: Failed to send packet to station %d (%s port %u)",
													(unsigned int)EconetTx.deststn,
													IpAddressStr(S_ADDR(RecvAddr)), (unsigned int)htons(RecvAddr.sin_port));
									}
									
									std::string str2 = "Econet: Ethernet data:" + BytesToString((unsigned char *)p, SendLen);

									DebugDisplayTrace(DebugType::Econet, true, str2.c_str());
								}
								
								if (EconetTx.ah.type == AUNType::Broadcast)
								{
									// need to send unicast copies to all hosts which won't see the broadcast
									for (int i=0; i < stationsp; i++)
									{
										// don't send to ourself or hosts listening on the AUN port
										if (stations[i].port != DEFAULT_AUN_PORT && !(stations[i].inet_addr == EconetListenIP && stations[i].port == EconetListenPort))
										{
											S_ADDR(RecvAddr) = stations[i].inet_addr;
											RecvAddr.sin_port = htons(stations[i].port);
											
											if (sendto(SendSocket, (char *)&EconetTx, SendLen, 0,
											           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
											{
												EconetError("Econet: Failed to send packet to station %d (%s port %u)",
												            (unsigned int)EconetTx.deststn,
												            IpAddressStr(S_ADDR(RecvAddr)), (unsigned int)htons(RecvAddr.sin_port));
											}
										}
									}
								}
							}
						}
						else
						{
							if (sendto(SendSocket, (char *)BeebTx.buff, BeebTx.Pointer, 0,
							           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
							{
								EconetError("Econet: Failed to send packet to network %d station %d (%s port %u)",
								            (unsigned int)BeebTx.eh.destnet, (unsigned int)BeebTx.eh.deststn,
								            IpAddressStr(S_ADDR(RecvAddr)), (unsigned int)htons(RecvAddr.sin_port));
							}
						}

						// Sending packet will mean peer goes into flag fill while
						// it deals with it
						FlagFillActive = true;
						SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);
						//if (DebugEnabled)
							DebugDisplayTrace(DebugType::Econet, true, "Econet: FlagFill set (packet sent)");

						BeebTx.Pointer = 0; // wipe buffer
						BeebTx.BytesInBuffer = 0;
						//if (DebugEnabled)
							DebugDumpADLC();
					}
					/* disabled because puking error boxes is annoying
					else
					{
						if (LastError.network != BeebTx.eh.destnet && LastError.station != BeebTx.eh.deststn)
						{
							if (AUNMode)
							{
								EconetError("Econet: Station %d.%d not found in AUNMap or Econet.cfg",
								            (unsigned int)BeebTx.eh.destnet,
								            (unsigned int)BeebTx.eh.deststn);
							}
							else
							{
								EconetError("Econet: Station %d.%d not found in Econet.cfg",
								            (unsigned int)BeebTx.eh.destnet,
								            (unsigned int)BeebTx.eh.deststn);
							}

							LastError.network = BeebTx.eh.destnet; // if there is a send error, remember the network and station
							LastError.station = BeebTx.eh.deststn; // to prevent the user being notified on each retry
						}
					}*/
				}
			}
		}

		// Receive data
		if (!(ADLC.control1 & CONTROL_REG1_RX_RESET)) // rx reset off
		{
			if (BeebRx.Pointer < BeebRx.BytesInBuffer)
			{
				// something waiting to be given to the processor
				if (ADLC.rxfptr < 3) // space in fifo
				{
					//if (DebugEnabled)
						DebugDisplayTrace(DebugType::Econet, true,
							"EconetPoll: Time to give another byte to the beeb");

					ADLC.rxfifo[2] = ADLC.rxfifo[1];
					ADLC.rxfifo[1] = ADLC.rxfifo[0];
					ADLC.rxfifo[0] = BeebRx.buff[BeebRx.Pointer];
					ADLC.rxfptr++;
					ADLC.rxffc = (ADLC.rxffc << 1) & 7;
					ADLC.rxap = (ADLC.rxap << 1) & 7;

					if (BeebRx.Pointer == 0)
					{
						ADLC.rxap |= 1; // 2 bytes? adr extention mode
					}

					if (++BeebRx.Pointer >= BeebRx.BytesInBuffer) // that was last byte!
					{
						ADLC.rxffc |= 1; // set FV flag (this was last byte of frame)
						BeebRx.Pointer = 0; // Reset read for next packet
						BeebRx.BytesInBuffer = 0;
					}
				}
			}

			if (ADLC.rxfptr == 0)
			{
				int j = 0;

				// still nothing in buffers (and thus nothing in EconetRx buffer)
				ADLC.control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE; // reset discontinue flag

				// wait for cpu to clear FV flag from last frame received
				if (!(ADLC.status2 & STATUS_REG2_FRAME_VALID))
				{
					if (!AUNMode ||
					    fourwaystage == FourWayStage::Idle ||
					    fourwaystage == FourWayStage::ImmediateSent ||
					    fourwaystage == FourWayStage::DataSent)
					{
						// Try and get another packet from network
						// Check if packet is waiting without blocking
						fd_set ReadFds;
						FD_ZERO(&ReadFds);
						FD_SET(ListenSocket, &ReadFds);

						timeval TimeOut = {0, 0};

						int RetVal = select((int)ListenSocket + 1, &ReadFds, NULL, NULL, &TimeOut);

						if (RetVal > 0)
						{
							sockaddr_in RecvAddr;
							// Read the packet
							int sizRcvAdr = sizeof(RecvAddr);

							if (AUNMode)
							{
								RetVal = recvfrom(ListenSocket, (char *)EconetRx.raw, sizeof(EconetRx.raw) + sizeof(EconetRx.buff), 0, (SOCKADDR *)&RecvAddr, &sizRcvAdr);
								EconetRx.BytesInBuffer = RetVal;
							}
							else
							{
								RetVal = recvfrom(ListenSocket, (char *)BeebRx.buff, sizeof(BeebRx.buff), 0, (SOCKADDR *)&RecvAddr, &sizRcvAdr);
							}

							if (RetVal > 0)
							{
								//if (DebugEnabled)
								{
									DebugDisplayTraceF(DebugType::Econet, true,
									                   "EconetPoll: Packet received: %u bytes from %s port %u",
									                   (int)RetVal,
									                   IpAddressStr(S_ADDR(RecvAddr)),
									                   htons(RecvAddr.sin_port));

									std::string str = "EconetPoll: Packet data:" + BytesToString(AUNMode ? EconetRx.raw : BeebRx.buff, RetVal);

									DebugDisplayTrace(DebugType::Econet, true, str.c_str());
								}

								if (AUNMode)
								{
									// convert from AUN format
									// find network and station number of sender
									bool found = false;
									
									// search for source in known stations
									for (int i = 0; i < stationsp; i++)
									{
										if (htons(RecvAddr.sin_port) == stations[i].port &&
											S_ADDR(RecvAddr) == stations[i].inet_addr)
										{
											BeebRx.eh.srcnet = stations[i].network;
											BeebRx.eh.srcstn = stations[i].station;
											found = true;
											break;
										}
									}
									
									if (!found)
									{
										// search source in networks
										for (int i = 0; i < networksp; i++)
										{
											if (S_ADDR(RecvAddr) == networks[i].inet_addr)
											{
												// a single address using sequential ports
												int s = (htons(RecvAddr.sin_port) - networks[i].port) - (networks[i].network << 8);
												// check whether result is in range
												if (s > 0 && s < 255)
												{
													BeebRx.eh.srcnet = networks[i].network;
													BeebRx.eh.srcstn = (unsigned char) s;
													found = true;
													break;
												}
												// else must be a different net on the same host
											}
											else if ((S_ADDR(RecvAddr) & 0x00FFFFFF) == networks[i].inet_addr && htons(RecvAddr.sin_port) == DEFAULT_AUN_PORT)
											{
												// true AUN addressing
												BeebRx.eh.srcnet = networks[i].network;
												BeebRx.eh.srcstn = (S_ADDR(RecvAddr) & 0xFF000000) >> 24;
												found = true;
												break;
											}
										}
									}
									
									if (!found && RetVal > 4)
									{
										// search source in extended AUN gateways
										for (int i = 0; i < gatewaysp; i++)
										{
											if (S_ADDR(RecvAddr) == gateways[i].inet_addr && htons(RecvAddr.sin_port) == gateways[i].port)
											{
												// PiEB gateways use an extended AUN which contains the econet addresses at the start of the packet
												memcpy(&BeebRx.eh, &EconetRx, sizeof(ShorEconetHeader));
												
												// this means the AUN data we want starts four bytes later than usual. This seems terribly inefficient, but lets remove those bytes from the buffer rather than trying to keep track of an offset through all the rest of the code.
												
												memmove(EconetRx.raw, EconetRx.raw + 4, RetVal - 4);
												RetVal -= 4; // adjust the length
												EconetRx.BytesInBuffer = RetVal; // must update this too!
												
												std::string str = "EconetPoll: Packet data:" + BytesToString( EconetRx.raw, RetVal);
												DebugDisplayTrace(DebugType::Econet, true, str.c_str());
												
												found = true;
												break;
											}
										}
									}
									else
									{
										if (MassageNetworks)
										{
											BeebRx.eh.srcnet &= 0x7F; // make AUN nets > 127 appear to be econet
										}
									}

									if (!found) // couldn't resolve econet source address
									{
										//if (DebugEnabled)
											DebugDisplayTrace(DebugType::Econet, true, "Econet: Packet ignored");

										BeebRx.BytesInBuffer = 0; // ignore the packet
									}
									else
									{
										// must be for us.
										BeebRx.eh.deststn = EconetStationID;
										BeebRx.eh.destnet = 0;
										
										//if (DebugEnabled)
										{
											DebugDisplayTraceF(DebugType::Econet, true,
											                   "Econet: Packet was from %02x %02x ",
											                   (unsigned int)BeebRx.eh.srcnet,
											                   (unsigned int)BeebRx.eh.srcstn);
										}

										// TODO - many of these copies can use memcpy()
										switch (fourwaystage)
										{
										case FourWayStage::Idle:
											// we weren't doing anything when this packet came in.
											BeebRx.eh.cb = EconetRx.ah.cb | 128;
											BeebRx.eh.port = EconetRx.ah.port;

											switch (EconetRx.ah.type)
											{
												case AUNType::Broadcast:
													BeebRx.eh.deststn = 255; // wasn't just for us..
													BeebRx.eh.destnet = 255;
													j = 6;
													for (unsigned int i = 0; i < RetVal - sizeof(EconetRx.ah); i++, j++) {
														BeebRx.buff[j] = EconetRx.buff[i];
													}
													BeebRx.BytesInBuffer = j;
													fourwaystage = FourWayStage::WaitForIdle;
													//if (DebugEnabled)
														DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (broadcast received)");
													break;

												case AUNType::Immediate:
													j = 6;
													for (unsigned int i = 0; i < RetVal - sizeof(EconetRx.ah); i++, j++) {
														BeebRx.buff[j] = EconetRx.buff[i];
													}
													BeebRx.BytesInBuffer = j;
													fourwaystage = FourWayStage::ImmediateReceived;
													//if (DebugEnabled)
														DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_IMMRCVD");
													break;

												case AUNType::Unicast:
													// we're assuming things here..
													if (EconetRx.ah.port == 0 && EconetRx.ah.cb == (0x82 & 0x7f)) {
														j = 6;
														for (unsigned int i = 0; i < 8; i++, j++) {
															BeebRx.buff[j] = EconetRx.buff[i];
														}
														BeebRx.BytesInBuffer = j;
													}

													else if (EconetRx.ah.port == 0 && EconetRx.ah.cb >= (0x83 & 0x7f) && EconetRx.ah.cb <= (0x85 & 0x7f)) {
														j = 6;
														for (unsigned int i = 0; i < 4; i++, j++) {
															BeebRx.buff[j] = EconetRx.buff[i];
														}
														BeebRx.BytesInBuffer = j;
													}

													else BeebRx.BytesInBuffer = sizeof(BeebRx.eh);
													fourwaystage = FourWayStage::ScoutReceived;
													//if (DebugEnabled)
														DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_SCOUTRCVD");
													break;

												default:
													//ignore anything else
													BeebRx.BytesInBuffer = 0;
													break;
											}

											BeebRx.Pointer = 0;
											break;

										case FourWayStage::ImmediateSent:  // it should be reply to an immediate instruction
											// TODO  check that it is!!!   Example scenario where it will not
											// be - *STATIONs poll sends packet to itself... packet we get
											// here is the one we just sent out..!!!
											// I'm pretty sure that real econet can't send to itself..

											j = 4;
											for (unsigned int i = 0; i < RetVal - sizeof(EconetRx.ah); i++, j++) {
												BeebRx.buff[j] = EconetRx.buff[i];
											}
											BeebRx.BytesInBuffer = j;
											BeebRx.Pointer = 0;
											fourwaystage = FourWayStage::WaitForIdle;
											//if (DebugEnabled)
												DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (ack received from remote AUN server)");
											break;

										case FourWayStage::DataSent:
											// we sent block of data, awaiting final ack..
											if (EconetRx.ah.type == AUNType::Ack || EconetRx.ah.type == AUNType::NAck)
											{
												// are we expecting a (N)ACK ?
												// TODO check it is a (n)ack for packet we just sent!!, deal with naks!
												// construct a final ack for the beeb

												BeebRx.BytesInBuffer = 4;
												BeebRx.Pointer = 0;
												//if (DebugEnabled)
													DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (aun ack rxd)");
												fourwaystage = FourWayStage::WaitForIdle;
												break;
											} // else unexpected packet - ignore it.TODO: queue it?

										default: // erm, what are we doing here?
											// ignore packet
											fourwaystage = FourWayStage::WaitForIdle;
											//if (DebugEnabled)
												DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_WAIT4IDLE (ack received from remote AUN server)");
											break;
										}
									}
								}
								else
								{
									BeebRx.BytesInBuffer = RetVal;
									BeebRx.Pointer = 0;
								}



								if ((BeebRx.eh.deststn == EconetStationID || IsBroadcastStation(BeebRx.eh.deststn)) &&
								    BeebRx.BytesInBuffer > 0)
								{
									// Peer sent us packet - no longer in flag fill
									FlagFillActive = false;

									//if (DebugEnabled)
									{
										DebugDisplayTrace(DebugType::Econet, true, "Econet: FlagFill reset");
									}
								}
								else
								{
									// Two other stations communicating - assume one of them will flag fill
									FlagFillActive = true;
									SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);

									//if (DebugEnabled)
									{
										DebugDisplayTrace(DebugType::Econet, true, "Econet: FlagFill set - other station comms");
									}
								}
							}
							else if (RetVal == SOCKET_ERROR && !SingleSocket)
							{
								EconetError("Econet: Failed to receive packet (error %ld)", GetLastSocketError());
							}
						}
						else if (RetVal == SOCKET_ERROR)
						{
							EconetError("Econet: Failed to check for new packet");
						}
					}

					// this bit fakes the bits of the 4-way handshake that AUN doesn't do.

					if (AUNMode && EconetScoutAckTrigger > TotalCycles)
					{
						switch (fourwaystage) {
						case FourWayStage::ScoutSent:
							// just got a scout from the beeb, fake an acknowledgement.
							BeebRx.eh.deststn = EconetStationID;
							BeebRx.eh.destnet = 0;
							BeebRx.eh.srcstn = (unsigned char)EconetTx.deststn; // use scout's dest as source of ack.
							BeebRx.eh.srcnet = (unsigned char)EconetTx.destnet;

							BeebRx.BytesInBuffer = 4;
							BeebRx.Pointer = 0;
							fourwaystage = FourWayStage::ScoutAckReceived;
							//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_SCACKRCVD");
							break;

						case FourWayStage::ScoutAckSent:
							// beeb acked the scout we gave it, so give it the data AUN sent us earlier.
							BeebRx.eh.deststn = EconetStationID; // as it is data it must be for us
							BeebRx.eh.destnet = 0;

							BeebRx.eh.srcstn = (unsigned char)EconetTx.deststn;  //30jun dont think this is right..
							BeebRx.eh.srcnet = (unsigned char)(EconetTx.destnet);

							j = 4;

							if (EconetRx.ah.port == 0 && EconetRx.ah.cb == (0x82 & 0x7f))
							{
								for (unsigned int i = 8; i < EconetRx.BytesInBuffer - sizeof(EconetRx.ah); i++, j++)
								{
									BeebRx.buff[j] = EconetRx.buff[i];
								}
							}
							else if (EconetRx.ah.port == 0 && EconetRx.ah.cb >= (0x83 & 0x7f) && EconetRx.ah.cb <= (0x85 & 0x7f))
							{
								for (unsigned int i = 4; i < EconetRx.BytesInBuffer - sizeof(EconetRx.ah); i++, j++)
								{
									BeebRx.buff[j] = EconetRx.buff[i];
								}
							}
							else
							{
								for (unsigned int i = 0; i < EconetRx.BytesInBuffer - sizeof(EconetRx.ah); i++, j++)
								{
									BeebRx.buff[j] = EconetRx.buff[i];
								}
							}

							BeebRx.BytesInBuffer = j;
							BeebRx.Pointer =0;
							fourwaystage = FourWayStage::DataReceived;
							//if (DebugEnabled)
								DebugDisplayTrace(DebugType::Econet, true, "Econet: Set FWS_DATARCVD");
							break;

						default:
							break;
						}
					}
				}
			}
		}

		// Update idle status
		if (!(ADLC.control1 & CONTROL_REG1_RX_RESET) // not rxreset
		    && ADLC.rxfptr == 0 // nothing in fifo
		    && !(ADLC.status2 & STATUS_REG2_FRAME_VALID) // no FV
		    && BeebRx.BytesInBuffer == 0) // nothing in ip buffer
		{
			ADLC.idle = true;
		}
		else
		{
			ADLC.idle = false;
		}

		// how long before we come back in here?
		SetTrigger(TimeBetweenBytes,EconetTrigger);
	}

	// Reset pseudo flag fill?
	if (EconetFlagFillTimeoutTrigger <= TotalCycles && FlagFillActive)
	{
		FlagFillActive = false;

		//if (DebugEnabled)
			DebugDisplayTrace(DebugType::Econet, true, "Econet: FlagFill timeout reset");
	}

	// waiting for AUN to become idle?
	if (AUNMode && fourwaystage == FourWayStage::WaitForIdle
		&& BeebRx.BytesInBuffer == 0
		&& ADLC.rxfptr == 0
		&& ADLC.txfptr == 0 // ??
		// && EconetScoutAckTrigger > TotalCycles
		)
	{
		fourwaystage = FourWayStage::Idle;
		EconetFourWayTrigger = 0;
		EconetScoutAckTrigger = 0;
		FlagFillActive = false;
	}

	// timeout four way handshake - for when we get lost..
	if (EconetFourWayTrigger == 0)
	{
		if (fourwaystage != FourWayStage::Idle)
		{
			SetTrigger(FourWayStageTimeout, EconetFourWayTrigger);
		}
	}
	else if (EconetFourWayTrigger <= TotalCycles)
	{
		EconetScoutAckTrigger = 0;
		EconetFourWayTrigger = 0;
		fourwaystage = FourWayStage::Idle;

		//if (DebugEnabled)
		{
			DebugDisplayTrace(DebugType::Econet, true, "Econet: 4waystage timeout; Set FWS_IDLE");
			DebugDumpADLC();
		}
	}

	// Status bits need changing?

	// SR1b0 - RDA - received data available.
	if (!(ADLC.control1 & CONTROL_REG1_RX_RESET)) // rx reset off
	{
		if ((ADLC.rxfptr > 0 && !(ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER)) || // 1 byte mode
		    (ADLC.rxfptr > 1 &&  (ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER))) // 2 byte mode
		{
			ADLC.status1 |= STATUS_REG1_RX_DATA_AVAILABLE; // set RDA copy
			ADLC.status2 |= STATUS_REG2_RX_DATA_AVAILABLE;
		}
		else
		{
			ADLC.status1 &= ~STATUS_REG1_RX_DATA_AVAILABLE;
			ADLC.status2 &= ~STATUS_REG2_RX_DATA_AVAILABLE;
		}
	}
	// SR1b1 - S2RQ - set after SR2, see below
	// SR1b2 - LOOP - set if in loop mode. not supported in this emulation
	// SR1b3 - FD - Flag detected. Hmm.
	if (FlagFillActive)
	{
		ADLC.status1 |= STATUS_REG1_FLAG_DETECTED;
	}
	else
	{
		ADLC.status1 &= ~STATUS_REG1_FLAG_DETECTED;
	}

	// SR1b4 - CTS - set by ~CTS line going up, and causes IRQ if enabled.
	//               only cleared by cpu.
	//               ~CTS is a NAND of DCD(clock present)(high if valid)
	//               & collission detection!
	//               i.e. it's low (thus clear to send) when we have both DCD(clock)
	//               present AND no collision on line and no collision.
	//               cts will ALSO be high if there is no cable!
	// we will only bother checking against DCD here as can't have collisions.
	// but nfs then loops waiting for CTS high!
	// on the B+ there is (by default) no collission detection circuitary. instead S29
	// links RTS in its place, thus CTS is a NAND of not RTS & not DCD
	// i.e. cts = ! ( !rts && !dcd ) all signals are active low.
	// there is a delay on rts going high after cr2b7=0 - ignore this for now.
	// cr2b7 = 1 means RTS low means not rts high means cts low
	// sockets true means dcd low means not dcd high means cts low
	// doing it this way finally works !!  great :-) :-)

	if (ReceiverSocketsOpen && (ADLC.control2 & CONTROL_REG2_RTS_CONTROL)) // clock + RTS
	{
		ADLC.cts = false;
		ADLC.status1 &= ~STATUS_REG1_CTS;
	}
	else
	{
		ADLC.cts = true;
	}

	// and then set the status bit if the line is high! (status bit stays
	// up until cpu tries to clear it) (& still stays up if cts line still high)

	if (!(ADLC.control1 & CONTROL_REG1_RX_RESET) && ADLC.cts)
	{
		ADLC.status1 |= STATUS_REG1_CTS; // set CTS now
	}

	// SR1b5 - TXU - Tx Underrun.
	if (ADLC.txfptr > 4) // probably not needed
	{
		//if (DebugEnabled)
		{
			DebugDisplayTraceF(DebugType::Econet, true,
			                   "Econet: TX Underrun - TXfptr %02x",
			                   (unsigned int)ADLC.txfptr);
		}

		ADLC.status1 |= STATUS_REG1_TX_UNDERRUN;
		ADLC.txfptr = 4;
	}

	// SR1b6 TDRA flag - another complicated derivation
	if (!(ADLC.control1 & CONTROL_REG1_TX_RESET)) // not txreset
	{
		if (!(ADLC.control2 & CONTROL_REG2_TDRA_SELECT)) // tdra mode
		{
			if (   (   ((ADLC.txfptr < 3) && !(ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER)) // space in fifo?
			        || ((ADLC.txfptr < 2) && (ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER))) // space in fifo?
			    && (!(ADLC.status1 & STATUS_REG1_CTS)) // clear to send is ok
			    && (!(ADLC.status2 & STATUS_REG2_DCD)) ) // DTR not high
			{
				if (/* DebugEnabled && */!(ADLC.status1 & STATUS_REG1_TDRA))
					DebugDisplayTrace(DebugType::Econet, true, "set tdra");

				ADLC.status1 |= STATUS_REG1_TDRA; // set Tx Reg Data Available flag
			}
			else
			{
				if (/*DebugEnabled && */(ADLC.status1 & STATUS_REG1_TDRA))
					DebugDisplayTrace(DebugType::Econet, true, "clear tdra");

				ADLC.status1 &= ~STATUS_REG1_TDRA; // clear Tx Reg Data Available flag
			}
		}
		else // FC mode
		{
			if (ADLC.txfptr == 0) // nothing in fifo
			{
				if (/*DebugEnabled && */!(ADLC.status1 & STATUS_REG1_TDRA))
					DebugDisplayTrace(DebugType::Econet, true, "set fc");

				ADLC.status1 |= STATUS_REG1_TDRA; // set Tx Reg Data Available flag.
			}
			else
			{
				if (/*DebugEnabled && */(ADLC.status1 & STATUS_REG1_TDRA))
					DebugDisplayTrace(DebugType::Econet, true, "clear fc");

				ADLC.status1 &= ~STATUS_REG1_TDRA; // clear Tx Reg Data Available flag.
			}
		}
	}
	// SR1b7 IRQ flag - see below

	// SR2b0 - AP - Address present
	if (!(ADLC.control1 & CONTROL_REG1_RX_RESET))
	{
		if (ADLC.rxfptr > 0 &&
		    (ADLC.rxap & (powers[ADLC.rxfptr - 1]))) // ap bits set on fifo
		{
			ADLC.status2 |= STATUS_REG2_ADDRESS_PRESENT;
		}
		else
		{
			ADLC.status2 &= ~STATUS_REG2_ADDRESS_PRESENT;
		}

		// SR2b1 - FV -Frame Valid - set in rx - only reset by ClearRx or RxReset
		if (ADLC.rxfptr > 0 &&
		    (ADLC.rxffc & (powers[ADLC.rxfptr - 1])))
		{
			ADLC.status2 |= STATUS_REG2_FRAME_VALID;
		}

		// SR2b2 - Inactive Idle Received - sets irq!
		if (ADLC.idle && !FlagFillActive)
		{
			ADLC.status2 |= STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
		else
		{
			ADLC.status2 &= ~STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
	}

	// SR2b3 - RxAbort - Abort received - set in rx routines above
	// SR2b4 - Error during reception - set if error flaged in rx routine.
	// SR2b5 - DCD
	if (!ReceiverSocketsOpen) // is line down?
	{
		ADLC.status2 |= STATUS_REG2_DCD; // flag error
	}
	else
	{
		ADLC.status2 &= ~STATUS_REG2_DCD;
	}

	// SR2b6 - OVRN -receipt overrun. probably not needed
	if (ADLC.rxfptr > 4)
	{
		ADLC.status2 |= STATUS_REG2_RX_OVERRUN;
		ADLC.rxfptr = 4;
	}

	// SR2b7 - RDA. As per SR1b0 - set above.

	// Handle PSE - only for SR2 Rx bits at the moment
	int PrevSr2pse = ADLC.sr2pse;

	if (ADLC.control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE)
	{
		if (ADLC.sr2pse <= 1 && (ADLC.status2 & (STATUS_REG2_FRAME_VALID |
		                                         STATUS_REG2_ABORT_RECEIVED |
		                                         STATUS_REG2_FCS_ERROR |
		                                         STATUS_REG2_DCD |
		                                         STATUS_REG2_RX_OVERRUN)))
		{
			ADLC.sr2pse = 1;
			ADLC.status2 &= ~(STATUS_REG2_ADDRESS_PRESENT |
			                  STATUS_REG2_INACTIVE_IDLE_RECEIVED |
			                  STATUS_REG2_RX_DATA_AVAILABLE);
		}
		else if (ADLC.sr2pse <= 2 && (ADLC.status2 & STATUS_REG2_INACTIVE_IDLE_RECEIVED)) // Idle
		{
			ADLC.sr2pse = 2;
			ADLC.status2 &= ~(STATUS_REG2_ADDRESS_PRESENT |
			                  STATUS_REG2_RX_DATA_AVAILABLE);
		}
		else if (ADLC.sr2pse <= 3 && (ADLC.status2 & STATUS_REG2_ADDRESS_PRESENT))
		{
			ADLC.sr2pse = 3;
			ADLC.status2 &= ~STATUS_REG2_RX_DATA_AVAILABLE;
		}
		else if (ADLC.status2 & STATUS_REG2_RX_DATA_AVAILABLE)
		{
			ADLC.sr2pse = 4;
			ADLC.status2 &= ~STATUS_REG2_FRAME_VALID;
		}
		else
		{
			ADLC.sr2pse = 0; // No relevant bits set
		}

		// Set SR1 RDA copy
		if (ADLC.status2 & STATUS_REG2_RX_DATA_AVAILABLE)
		{
			ADLC.status1 |= STATUS_REG1_RX_DATA_AVAILABLE;
		}
		else
		{
			ADLC.status1 &= ~STATUS_REG1_RX_DATA_AVAILABLE;
		}
	}
	else // PSE inactive
	{
		ADLC.sr2pse = 0;
	}

	if (/*DebugEnabled && */ADLC.sr2pse != PrevSr2pse)
	{
		DebugDisplayTraceF(DebugType::Econet, true,
		                   "ADLC: PSE SR2Rx priority changed to %d",
		                   ADLC.sr2pse);
	}

	// Do we need to flag an interrupt?
	if (ADLC.status1 != ADLCtemp.status1 || ADLC.status2 != ADLCtemp.status2) // something changed
	{
		// SR1b1 - S2RQ - Status2 request. New bit set in S2?
		unsigned char tempcause = ((ADLC.status2 ^ ADLCtemp.status2) & ADLC.status2) & ~STATUS_REG2_RX_DATA_AVAILABLE;

		if (!(ADLC.control1 & CONTROL_REG1_RX_INT_ENABLE)) // RIE not set
		{
			tempcause = 0;
		}

		if (tempcause) // something got set
		{
			ADLC.status1 |= STATUS_REG1_STATUS2_READ_REQUEST;
			sr1b2cause = sr1b2cause | tempcause;
		}
		else if (!(ADLC.status2 & sr1b2cause)) // cause has gone
		{
			ADLC.status1 &= ~STATUS_REG1_STATUS2_READ_REQUEST;
			sr1b2cause = 0;
		}

		// New bit set in S1?
		tempcause = ((ADLC.status1 ^ ADLCtemp.status1) & ADLC.status1) & ~STATUS_REG1_IRQ;

		if (!(ADLC.control1 & CONTROL_REG1_RX_INT_ENABLE)) // RIE not set
		{
			tempcause &= ~(STATUS_REG1_RX_DATA_AVAILABLE |
			               STATUS_REG1_STATUS2_READ_REQUEST |
			               STATUS_REG1_FLAG_DETECTED);
		}

		if (!(ADLC.control1 & CONTROL_REG1_TX_INT_ENABLE)) // TIE not set
		{
			tempcause &= ~(STATUS_REG1_CTS |
			               STATUS_REG1_TX_UNDERRUN |
			               STATUS_REG1_TDRA);
		}

		if (tempcause != 0) // something got set
		{
			interruptnow = true;
			irqcause = irqcause | tempcause; // remember which bit went high to flag irq

			ADLC.status1 |= STATUS_REG1_IRQ;

			//if (DebugEnabled)
			{
				DebugDisplayTraceF(DebugType::Econet, true,
				                  "ADLC: Status1 bit got set %02x, interrupt",
				                  (int)tempcause);
			}
		}

		// Bit cleared in S1?
		unsigned char temp2 = ((ADLC.status1 ^ ADLCtemp.status1) & ADLCtemp.status1) & ~STATUS_REG1_IRQ;

		if (temp2 != 0) // something went off
		{
			irqcause = irqcause & ~temp2; // clear flags that went off

			if (irqcause == 0) // all flag gone off now
			{
				// clear irq status bit when cause has gone.
				ADLC.status1 &= ~STATUS_REG1_IRQ;
			}
			else
			{
				// interrupt again because still have flags set
				if (ADLC.control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE)
				{
					interruptnow = true;

					//if (DebugEnabled)
						DebugDisplayTrace(DebugType::Econet, true, "ADLC: S1 flags still set, interrupt");
				}
			}

			//if (DebugEnabled)
			{
				DebugDisplayTraceF(DebugType::Econet, true,
				                   "ADLC: IRQ cause reset, irqcause %02x",
				                   (int)irqcause);
			}
		}

		//if (DebugEnabled)
			DebugDumpADLC();
	}

	return interruptnow; // flag NMI if necessary. see also INTON flag as
	                     // this can cause a delayed interrupt (beebmem.cpp).
}

//--------------------------------------------------------------------------------------------
// display some information

void DebugDumpADLC()
{
	DebugDisplayTraceF(DebugType::Econet, true,
	                   "ADLC: Ctl:%02X %02X %02X %02X St:%02X %02X TXptr:%01x rx:%01x FF:%d IRQc:%02x SR2c:%02x PC:%04x 4W:%i ",
	                   (int)ADLC.control1, (int)ADLC.control2, (int)ADLC.control3, (int)ADLC.control4,
	                   (int)ADLC.status1, (int)ADLC.status2,
	                   (int)ADLC.txfptr, (int)ADLC.rxfptr, FlagFillActive ? 1 : 0,
	                   (int)irqcause, (int)sr1b2cause, (int)ProgramCounter, (int)fourwaystage);
}

//--------------------------------------------------------------------------------------------
// Display an error message box

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
