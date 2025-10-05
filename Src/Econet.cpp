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
const bool DEFAULT_LEARN_MODE = false;
const bool DEFAULT_STRICT_AUN_MODE = false;
const bool DEFAULT_SINGLE_SOCKET = true;
const int DEFAULT_FLAG_FILL_TIMEOUT = 500000;
const int DEFAULT_SCOUT_ACK_TIMEOUT = 5000;
const unsigned int DEFAULT_TIME_BETWEEN_BYTES = 128;
const unsigned int DEFAULT_FOUR_WAY_STAGE_TIMEOUT = 500000;
const bool DEFAULT_MASSAGE_NETWORKS = false;

static bool AUNMode = DEFAULT_AUN_MODE; // Use Acorn Universal Networking (AUN) style networking
static bool LearnMode = DEFAULT_LEARN_MODE; // Add receipts from unknown hosts to network table
static bool StrictAUNMode = DEFAULT_STRICT_AUN_MODE; // Assume network ip=stn number when sending to unknown hosts
static unsigned int FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
static bool MassageNetworks = DEFAULT_MASSAGE_NETWORKS; // Massage network numbers on send/receive (add/sub 128)

static int inmask, outmask;

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
unsigned char EconetStationID = 0; // Default Station ID
unsigned char EconetNetworkID = 0; // Default Network ID

static u_short EconetListenPort = 0; // default Listen port
static unsigned long EconetListenIP = inet_addr("127.0.0.1");
// IP settings:
static SOCKET Socket = INVALID_SOCKET; // Also used to flag line up and clock running

const u_short DEFAULT_AUN_PORT = 32768;

// Written in 2004:
// We will be using Econet over Ethernet as per AUN,
// however I've not got a real Acorn ethernet machine to see how
// it actually works! The only details I can find is it is:
// "Standard econet encpsulated in UDP to port 32768" and that
// addressing defaults to "1.0.net.stn" where net >= 128 for Ethernet.
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

struct EconetHeader
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

struct EthernetPacket
{
	AUNHeader ah;

	union {
		unsigned char buff[ETHERNET_BUFFER_SIZE];
		EconetHeader eh;
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
struct EconetHost
{
	unsigned char station;
	unsigned char network;
	unsigned long inet_addr;
	u_short port;
};

struct EconetNet
{
	unsigned long inet_addr;
	unsigned char network;
};

struct NetStn
{
	unsigned char network;
	unsigned char station;
};

static NetStn LastError;

const int STATIONS_TABLE_LENGTH = 512; // Total number of hosts we can know about
const int NETWORKS_TABLE_LENGTH = 128; // Number of disparate networks in AUNMap

static EconetHost stations[STATIONS_TABLE_LENGTH]; // Individual stations we know about
static EconetNet networks[NETWORKS_TABLE_LENGTH]; // AUN networks we know about

static int stationsp = 0; // How many individual stations do I know about?
static int networksp = 0;  // How many networks do I know about?
static int myaunnet = 0; // aunnet table entry that I match. should be -1 as 0 is valid

static unsigned char irqcause;   // flag to indicate cause of irq sr1b7
static unsigned char sr1b2cause; // flag to indicate cause of irq sr1b2

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

// Device and temp copy!
static MC6854 ADLC;
static MC6854 ADLCtemp;

//---------------------------------------------------------------------------

static bool ReadNetwork();
static bool EconetPollReal();
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

static EconetHost* FindHost(sockaddr_in* pAddress)
{
	for (int i = 0; i < stationsp; i++)
	{
		if (pAddress->sin_port == htons(stations[i].port) &&
		    S_ADDR(*pAddress) == stations[i].inet_addr)
		{
			return &stations[i];
		}
	}

	return nullptr;
}

//---------------------------------------------------------------------------

static EconetHost* AddHost(sockaddr_in* pAddress)
{
	if (stationsp < STATIONS_TABLE_LENGTH)
	{
		if (DebugEnabled)
		{
			DebugDisplayTrace(DebugType::Econet,
			                  true,
			                  "Econet: Previously unknown host; add entry!");
		}

		EconetHost* pHost = &stations[stationsp];

		pHost->port = ntohs(pAddress->sin_port);
		pHost->inet_addr = pAddress->sin_addr.s_addr;
		// TODO sort this out!! potential for clashes!! look for dupes
		pHost->station = (pHost->inet_addr & 0xFF000000) >> 24;
		// TODO and we need to use the map file ..
		pHost->network = 0;

		stationsp++;

		return pHost;
	}
	else
	{
		if (DebugEnabled)
		{
			DebugDisplayTrace(DebugType::Econet,
			                  true,
			                  "Econet: Previously unknown host. Host table full");
		}
	}

	return nullptr;
}

//---------------------------------------------------------------------------

static void EconetCloseSockets()
{
	if (Socket != INVALID_SOCKET)
	{
		CloseSocket(Socket);
		Socket = INVALID_SOCKET;
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

	// hardware operations:
	// set RxReset and TxReset
	ADLC.control1 = CONTROL_REG1_RX_RESET | CONTROL_REG1_TX_RESET;
	// reset TxAbort, RTS, LoopMode, DTR
	ADLC.control4 = 0; //ADLC.control4 & 223;
	ADLC.control2 = 0; //ADLC.control2 & 127;
	ADLC.control3 = 0; //ADLC.control3 & 95;

	// clear all status conditions
	ADLC.status1 = 0; // cts - clear to send line input (no collisions talking udp)
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

	// Create a SOCKET for sending messages and listening for incoming
	// connection requests.
	Socket = socket(AF_INET, SOCK_DGRAM, 0);

	if (Socket == INVALID_SOCKET)
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
		}
		else
		{
			EconetError("Econet: Failed to find station %d in Econet.cfg", EconetStationID);
			goto Fail;
		}

		service.sin_port = htons(EconetListenPort);
		S_ADDR(service) = EconetListenIP;

		if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR)
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

						if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == 0)
						{
							EconetListenPort = stations[i].port;
							EconetListenIP = stations[i].inet_addr;
							EconetStationID = stations[i].station;
						}
					}
				}
			}

			if (EconetListenPort == 0)
			{
				// Still can't find one ... strict mode?

				if (AUNMode && StrictAUNMode && stationsp < STATIONS_TABLE_LENGTH)
				{
					if (DebugEnabled)
					{
						DebugDisplayTrace(DebugType::Econet,
						                  true,
						                  "Econet: No free hosts in table. Trying automatic mode");
					}

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

								if (bind(Socket, (SOCKADDR*)&service, sizeof(service)) == 0)
								{
									myaunnet = j;
									EconetNetworkID = networks[j].network;

									EconetListenIP = IN_ADDR(localaddr);
									EconetListenPort = DEFAULT_AUN_PORT;
									EconetStationID = IN_ADDR(localaddr) >> 24;

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

	if (DebugEnabled)
	{
		DebugDisplayTraceF(DebugType::Econet,
		                   true,
		                   "Econet: Station number set to %d, port %d",
		                   EconetStationID, EconetListenPort);
	}

	// On Master the station number is read from CMOS so update it
	if (MachineType == Model::Master128 || MachineType == Model::MasterET)
	{
		RTCWriteAddress(0xE);
		RTCWriteData(EconetStationID);
	}

	// This call is what allows broadcast packets to be sent:
	const char broadcast = '1';

	if (setsockopt(Socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1)
	{
		EconetError("Econet: Failed to set socket for broadcasts (error %ld)", GetLastSocketError());
		goto Fail;
	}

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

		ParseConfigLine(Line, Tokens);

		if (Tokens.size() == 4)
		{
			if (stationsp < STATIONS_TABLE_LENGTH)
			{
				try
				{
					stations[stationsp].network   = (unsigned char)std::stoi(Tokens[0]);
					stations[stationsp].station   = (unsigned char)std::stoi(Tokens[1]);
					stations[stationsp].inet_addr = inet_addr(Tokens[2].c_str());
					stations[stationsp].port      = (u_short)std::stoi(Tokens[3]);

					DebugDisplayTraceF(DebugType::Econet, true,
					                   "Econet: ConfigFile Net %d Stn %d IP %s Port %d",
					                   stations[stationsp].network, stations[stationsp].station,
					                   IpAddressStr(stations[stationsp].inet_addr), stations[stationsp].port);

					stationsp++;
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
					LearnMode = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "AUNSTRICT") == 0)
				{
					StrictAUNMode = std::stoi(Value) != 0;
				}
				else if (StrCaseCmp(Key.c_str(), "SINGLESOCKET") == 0)
				{
					// Ignored.
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

	stations[stationsp].station = 0;

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

	networksp = 0;

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

		ParseConfigLine(Line, Tokens);

		if (Tokens.size() == 3 && StrCaseCmp("ADDMAP", Tokens[0].c_str()) == 0)
		{
			if (networksp < NETWORKS_TABLE_LENGTH)
			{
				try
				{
					networks[networksp].inet_addr = inet_addr(Tokens[1].c_str()) & 0x00FFFFFF; // stored as lsb..msb ?!?!
					networks[networksp].network   = (unsigned char)(std::stoi(Tokens[2]) & inmask); // 30jun strip b7

					if (DebugEnabled)
					{
						DebugDisplayTraceF(DebugType::Econet,
						                   true,
						                   "Econet: AUNMap Net %i IP %s",
						                   networks[networksp].network,
						                   IpAddressStr(networks[networksp].inet_addr));
					}

					// Note which network we are a part of. This won't work on first run as EconetListenIP not set!
					if (networks[networksp].inet_addr == (EconetListenIP & 0x00FFFFFF))
					{
						myaunnet = networksp;
						EconetNetworkID = networks[networksp].network;

						if (DebugEnabled)
						{
							DebugDisplayTrace(DebugType::Econet,
							                  true,
							                  "Econet: ..and that's the one we're in");
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

	networks[networksp].network = 0; // terminate table. 0 is always local so should not be in file.

	return Success;
}

//---------------------------------------------------------------------------

static bool ReadNetwork()
{
	AUNMode = DEFAULT_AUN_MODE;
	LearnMode = DEFAULT_LEARN_MODE;
	StrictAUNMode = DEFAULT_STRICT_AUN_MODE;
	EconetFlagFillTimeout = DEFAULT_FLAG_FILL_TIMEOUT;
	EconetScoutAckTimeout = DEFAULT_SCOUT_ACK_TIMEOUT;
	TimeBetweenBytes = DEFAULT_TIME_BETWEEN_BYTES;
	FourWayStageTimeout = DEFAULT_FOUR_WAY_STAGE_TIMEOUT;
	MassageNetworks = DEFAULT_MASSAGE_NETWORKS;

	stationsp = 0;
	stations[0].station = 0;

	if (!ReadEconetConfigFile())
	{
		return false;
	}

	if (MassageNetworks)
	{
		inmask  = 255;
		outmask = 0;
	}
	else
	{
		inmask  = 127;
		outmask = 128;
	}

	networksp = 0;
	networks[0].network = 0; // terminate table

	// Don't bother reading file if not using AUN.
	if (AUNMode)
	{
		return ReadAUNConfigFile();
	}

	return true;
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
		Value = ADLC.status1;
	}
	else if (Register == 1)
	{
		Value = ADLC.status2;
	}
	else
	{
		// RxReset not set and something in FIFO.
		if (((ADLC.control1 & CONTROL_REG1_RX_RESET) == 0) && ADLC.rxfptr > 0)
		{
			Value = ADLC.rxfifo[--ADLC.rxfptr]; // Read RX buffer.

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
		// Cannot write an output byte if TxReset is set.
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
		if (Socket != INVALID_SOCKET)
		{
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
// only re-calculate them when needed (e.g. on a memory read or in the receive
// routines before they are checked) but for the moment I just want to get this
// code actually working!

bool EconetPollReal() // return NMI status
{
	bool Interrupt = false;

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
	//         automatically reset when frame aborted by receiving an abort flag, or DCD fails
	if (ADLC.control1 & CONTROL_REG1_RX_FRAME_DISCONTINUE)
	{
		#ifdef DEBUG_ECONET
		DebugTrace("EconetPoll: RxABORT is set\n");
		#endif

		BeebRx.Pointer = 0;
		BeebRx.BytesInBuffer = 0;

		ADLC.rxfptr = 0;
		ADLC.rxap = 0;
		ADLC.rxffc = 0;
		ADLC.control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE; // reset flag

		fourwaystage = FourWayStage::Idle;
	}
	// CR1b6 - RxRs - Receiver reset. set by cpu or when reset line goes low.
	//         all receive operations blocked (bar DCD monitoring) when this is set.
	//         see CR2b5
	// CR1b7 - TxRS - Transmitter reset. set by CPU or when reset line goes low.
	//         all transmit operations blocked (bar CTS monitoring) when this is set.
	//         no action needed here; watch this bit elsewhere to inhibit actions

	// CR2b0 - PSE - priotitised status enable - adjusts how status bits show up.
	//         See sr2pse and code in status section
	// CR2b1 - 2byte/1byte mode.  set to indicate 2 byte mode. see trda status bit.
	// CR2b2 - Flag/Mark idle select. What is transmitted when TX idle.
	//         Ignored here as not needed.
	// CR2b3 - FC/TDRA mode - does status bit SR1b6 indicate 1=frame complete,
	//         0=tx data reg available. 1=frame tx complete.  see tdra status bit
	// CR2b4 - TxLast - byte just put into FIFO was the last byte of a packet.
	if (ADLC.control2 & CONTROL_REG2_TX_LAST_DATA)
	{
		ADLC.txftl |= 1; // set b0 - flag for fifo[0]
		ADLC.control2 &= ~CONTROL_REG2_TX_LAST_DATA; // clear flag.
	}

	// CR2b5 - CLR RxST - Clear Receiver Status - reset status bits
	if ((ADLC.control2 & CONTROL_REG2_CLEAR_RX_STATUS) || (ADLC.control1 & CONTROL_REG1_RX_RESET)) // or RxReset
	{
		ADLC.control2 &= ~CONTROL_REG2_CLEAR_RX_STATUS; // clear this bit

		ADLC.status1 &= ~(STATUS_REG1_STATUS2_READ_REQUEST | STATUS_REG1_FLAG_DETECTED); // clear sr2rq, FD

		// Clear FV, RxIdle, RxAbt, Err, OVRN, DCD.
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
	if ((ADLC.control2 & CONTROL_REG2_CLEAR_TX_STATUS) || (ADLC.control1 & CONTROL_REG1_TX_RESET)) // or TxReset
	{
		ADLC.control2 &= ~CONTROL_REG2_CLEAR_TX_STATUS; // clear this bit
		ADLC.status1 &= ~(STATUS_REG1_CTS |
		                  STATUS_REG1_TX_UNDERRUN |
		                  STATUS_REG1_TDRA); // clear TXU , cts, TDRA/FC

		if (ADLC.cts)
		{
			ADLC.status1 |= STATUS_REG1_CTS; // CTS follows signal, reset high again
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
	// CR3b4 - FDSE - flag detect status enable.  when set, then FD (SR1b3) + interrupt indicated a flag
	// has been received. I don't think we use this mode, so ignoring it.
	// CR3b5 - Loop - Loop mode. Not used.
	// CR3b6 - GAP/TST - sets test loopback mode (when not in Loop operation mode). ignored.
	// CR3b7 - LOC/DTR - (when not in loop mode) controls DTR pin directly. Pin not used in a BBC B

	// CR4b0 - FF/F - When clear, re-used the Flag at end of one packet as start of next packet. ignored.
	// CR4b1,2 - TX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b3,4 - RX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b5 - TransmitABT - Abort Transmission.  Once abort starts, bit is cleared.
	if (ADLC.control4 & CONTROL_REG4_TX_ABORT)
	{
		#ifdef DEBUG_ECONET
		DebugTrace("EconetPoll: TxABORT is set\n");
		#endif

		ADLC.txfptr = 0; // reset FIFO
		ADLC.txftl = 0; // reset FIFO flags
		BeebTx.Pointer = 0;
		BeebTx.BytesInBuffer = 0;
		ADLC.control4 &= ~CONTROL_REG4_TX_ABORT; // reset flag.
		fourwaystage = FourWayStage::Idle;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: Set FourWayStage::Idle (abort)");
		#endif
	}

	// CR4b6 - ABTex - extend abort - adjust way the abort flag is sent.  ignore.
	// Can affect timing of RTS output line (and thus CTS input) still ignored.
	// CR4b7 - NRZI/NRZ - invert data encoding on wire. ignore.

	if (EconetTrigger <= TotalCycles)
	{
		// Only do this bit occasionally as data only comes in from
		// line occasionally.
		// Trickle data between FIFO registers and IP packets.

		// Transmit data.
		if (!(ADLC.control1 & CONTROL_REG1_TX_RESET)) // TX reset off
		{
			if (ADLC.txfptr > 0) // There is data in the transmit FIFO.
			{
				#ifdef DEBUG_ECONET
				DebugTrace("EconetPoll: Write to FIFO noticed\n");
				#endif

				bool TxLast = false;

				if (ADLC.txftl & powers[ADLC.txfptr - 1]) // TxLast set
				{
					TxLast = true;
				}

				if (BeebTx.Pointer + 1 > sizeof(BeebTx.buff) || // overflow IP buffer
				    (ADLC.txfptr > 4)) // overflowed FIFO
				{
					ADLC.status1 |= STATUS_REG1_TX_UNDERRUN; // set tx underrun flag
					BeebTx.Pointer = 0; // wipe buffer
					BeebTx.BytesInBuffer = 0;
					ADLC.txfptr = 0;
					ADLC.txftl = 0;

					#ifdef DEBUG_ECONET
					DebugTrace("EconetPoll: TxUnderun!\n");
					#endif
				}
				else
				{
					BeebTx.buff[BeebTx.Pointer] = ADLC.txfifo[--ADLC.txfptr];
					BeebTx.Pointer++;
				}

				if (TxLast) // TxLast set
				{
					if (DebugEnabled)
					{
						DebugDisplayTraceF(DebugType::Econet,
						                   true,
						                   "Econet: TXLast set - Send packet to network %d station %d",
						                   (int)BeebTx.eh.destnet,
						                   (int)BeebTx.eh.deststn);
					}

					// first two bytes of BeebTx.buff contain the destination address
					// (or one zero byte for broadcast)

					sockaddr_in RecvAddr;
					bool SendMe = false;
					int SendLen = 0;
					int i = 0;

					if (AUNMode && IsBroadcastStation(BeebTx.eh.deststn))
					{
						// TODO something
						// Somewhere that I cannot now find suggested that
						// AUN buffers broadcast packet, and broadcasts a simple flag. Stations
						// poll us to get the actual broadcast data ..
						// Hmmm...
						//
						// ok, just send it to the local broadcast address.
						// TODO: lookup destnet in AUNNet and use proper IP address!
						RecvAddr.sin_family = AF_INET;
						RecvAddr.sin_port = htons(DEFAULT_AUN_PORT);
						S_ADDR(RecvAddr) = INADDR_BROADCAST; // ((EconetListenIP & 0x00FFFFFF) | 0xFF000000);
						SendMe = true;
					}
					else
					{
						do {
							// Does the packet match this network table entry?
							// // check for 0.stn and mynet.stn.
							// AUNNet won't be populated if not in AUN mode, but we don't need to not check
							// it because it won't matter.
							if ((stations[i].network == BeebTx.eh.destnet ||
							    (stations[i].network == networks[myaunnet].network && stations[i].network != 0)) &&
							    stations[i].station == BeebTx.eh.deststn)
							{
								SendMe = true;
								break;
							}
							i++;
						} while (i < stationsp);

						// Guess address if not found in table.
						if (!SendMe && StrictAUNMode) // Didn't find it and allowed to guess.
						{
							if (DebugEnabled)
							{
								DebugDisplayTrace(DebugType::Econet,
								                  true,
								                  "Econet: Send to unknown host; make assumptions & add entry!");
							}

							if (BeebTx.eh.destnet == 0 || BeebTx.eh.destnet == networks[myaunnet].network)
							{
								stations[i].inet_addr = networks[myaunnet].inet_addr | (BeebTx.eh.deststn << 24);
								stations[i].port = DEFAULT_AUN_PORT;
								stations[i].network = BeebTx.eh.destnet;
								stations[i].station = BeebTx.eh.deststn;
								SendMe = true;
								stations[++stationsp].station = 0;
							}
							else
							{
								int j = 0;

								do {
									if (networks[j].network == BeebTx.eh.destnet)
									{
										stations[i].inet_addr = networks[j].inet_addr | (BeebTx.eh.deststn << 24);
										stations[i].port = DEFAULT_AUN_PORT;
										stations[i].network = BeebTx.eh.destnet;
										stations[i].station = BeebTx.eh.deststn;
										SendMe = true;
										stations[++stationsp].station = 0;
										break;
									}
									j++;
								} while (j < networksp);
							}
						}

						RecvAddr.sin_family = AF_INET;
						RecvAddr.sin_port = htons(stations[i].port);
						S_ADDR(RecvAddr) = stations[i].inet_addr;
					}

					if (DebugEnabled)
					{
						DebugDisplayTraceF(DebugType::Econet,
						                   true,
						                   "Econet: TXLast set: Send %d byte packet to network %d station %d (%s port %u)",
						                   BeebTx.Pointer,
						                   (int)BeebTx.eh.destnet,
						                   (int)BeebTx.eh.deststn,
						                   IpAddressStr(S_ADDR(RecvAddr)),
						                   (unsigned int)htons(RecvAddr.sin_port));

						std::string str = "Econet: Packet data:" + BytesToString(BeebTx.buff, BeebTx.Pointer);

						DebugDisplayTrace(DebugType::Econet, true, str.c_str());
					}

					/*
					if (AUNMode && fourwaystage != FWS_IDLE) {
						if (RecvAddr.sin_port != EconetTx.inet_addr ||
							RecvAddr.sin_port != htons(EconetTx.port) ) {
								EconetError("Erm.. trying to send somewhere while in the middle of talking to somewhere else.");
						}
					}
					*/

					// Send a datagram to the receiver
					if (SendMe)
					{
						LastError.network = 0; // reset the network & station where the last send error occurred
						LastError.station = 0;

						#ifdef DEBUG_ECONET
						DebugTrace("Econet: Sending a packet\n");
						#endif

						if (AUNMode)
						{
							unsigned int j = 0;
							// OK. Lets do AUN ...
							// The Beeb has given us a packet .. what is it?
							SendMe = false;

							switch (fourwaystage)
							{
							case FourWayStage::ScoutAckReceived:
								// It came in response to our ack of a scout.
								// What we have /should/ be the data block.
								// CLUDGE WARNING is this a scout sent again immediately?? TODO fix this?!?!
								if (EconetTx.ah.port == 0x00)
								{
									if (EconetTx.ah.cb == (0x82 & 0x7f))
									{
										j = 8;
									}
									else if (EconetTx.ah.cb >= (0x83 & 0x7f) &&
									         EconetTx.ah.cb <= (0x85 & 0x7f))
									{
										j = 4;
									}
								}

								if (BeebTx.Pointer != sizeof(BeebTx.eh) + j || memcmp(BeebTx.buff, BeebTxCopy, sizeof(BeebTx.eh) + j) != 0) // nope
								{
									for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
										EconetTx.buff[j] = BeebTx.buff[k];
									}
									EconetTx.Pointer = j;

									SendMe = true;
									SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;

									fourwaystage = FourWayStage::DataSent;

									#ifdef DEBUG_ECONST
									DebugTrace("Econet: Set FourWayStage::DataSent\n");
									#endif
									break;
								} // else fall through...

							case FourWayStage::Idle:
								// Not currently doing anything, so this will be a scout,
								// maybe a long scout or a broadcast.
								memcpy(BeebTxCopy, BeebTx.buff, sizeof(BeebTx.eh));
								EconetTx.ah.cb = (unsigned int)(BeebTx.eh.cb) & 127; // | 128;
								EconetTx.ah.port = (unsigned int)BeebTx.eh.port;
								EconetTx.ah.pad = 0;
								EconetTx.ah.handle = (ec_sequence += 4);

								EconetTx.destnet = BeebTx.eh.destnet | outmask; //30JUN
								EconetTx.deststn = BeebTx.eh.deststn;

								for (unsigned int k = 6; k < BeebTx.Pointer; k++, j++) {
									EconetTx.buff[j] = BeebTx.buff[k];
								}

								EconetTx.Pointer = j;

								if (IsBroadcastStation(EconetTx.deststn))
								{
									EconetTx.ah.type = AUNType::Broadcast;

									fourwaystage = FourWayStage::WaitForIdle; // no response to broadcasts...

									SendMe = true; // Send packet.
									SendLen = sizeof(EconetTx.ah) + 8;

									#ifdef DEBUG_ECONET
									DebugTrace("Econet: Set FourWayStage::WaitForIdle (broadcast sent)\n");
									#endif
								}
								else if (EconetTx.ah.port == 0 &&
								         (EconetTx.ah.cb < (0x82 & 0x7f) || EconetTx.ah.cb >(0x85 & 0x7f)))
								{
									EconetTx.ah.type = AUNType::Immediate;

									fourwaystage = FourWayStage::ImmediateSent;

									SendMe = true; // Send packet.
									SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;

									#ifdef DEBUG_ECONET
									DebugTrace("Econet: Set FourWayStage::ImmediateSent\n");
									#endif
								}
								else
								{
									EconetTx.ah.type = AUNType::Unicast;

									fourwaystage = FourWayStage::ScoutSent;

									// Don't send anything but set wait anyway.
									SetTrigger(EconetScoutAckTimeout, EconetScoutAckTrigger);

									#ifdef DEBUG_ECONET
									DebugTrace("Econet: Set FourWayStage::ScoutSent\n");
									DebugTrace("Econet: Scout Ack Timeout set\n");
									#endif
								} // else BROADCAST !!!!
								break;

							case FourWayStage::ScoutReceived:
								// It's an ack for a scout which we sent the Beeb. Just drop it, but move on.
								fourwaystage = FourWayStage::ScoutAckSent;

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
								// This ought to work, but only because the Beeb can only talk to one machine at any time.
								EconetTx.ah = EconetRx.ah;
								EconetTx.ah.type = AUNType::Ack;

								SendLen = sizeof(EconetRx.ah);
								SendMe = true;
								/*
								if (sendto(SendSocket, (char *) &EconetTx.ah, SendLen, 0,
									(SOCKADDR *) &RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR) {
										EconetError("Econet: Failed to send packet to %02x %02x (%08X :%u)",
											(unsigned int)(stations[i].inet_addr), (unsigned int)stations[i].station,
											(unsigned int)stations[i].inet_addr, (unsigned int)stations[i].port);
								}
								*/

								fourwaystage = FourWayStage::WaitForIdle;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::WaitForIdle (final ack sent)\n");
								#endif
								break;

							case FourWayStage::ImmediateReceived:
								// It's a reply to an immediate command we just had.
								for (unsigned int k = 4; k < BeebTx.Pointer; k++, j++) {
									EconetTx.buff[j] = BeebTx.buff[k];
								}

								EconetTx.Pointer = j;

								EconetTx.ah = EconetRx.ah;
								EconetTx.ah.type = AUNType::ImmReply;

								SendMe = true;
								SendLen = sizeof(EconetTx.ah) + EconetTx.Pointer;

								fourwaystage = FourWayStage::WaitForIdle;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::WaitForIdle (immediate received)\n");
								#endif
								break;

							default:
								// Shouldn't be here. Ignore packet and abort fourway.
								fourwaystage = FourWayStage::WaitForIdle;

								#ifdef DEBUG_ECONET
								DebugTrace("Econet: Set FourWayStage::WaitForIdle (unexpected mode, packet ignored)\n");
								#endif
								break;
							}

							if (SendMe)
							{
								if (sendto(Socket, (char *)&EconetTx, SendLen, 0,
								           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
								{
									EconetError("Econet: Failed to send packet to station %d (%s port %u)",
									            (unsigned int)stations[i].station,
									            IpAddressStr(stations[i].inet_addr), (unsigned int)stations[i].port);
								}
							}
						}
						else
						{
							if (sendto(Socket, (char *)BeebTx.buff, BeebTx.Pointer, 0,
							           (SOCKADDR *)&RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR)
							{
								EconetError("Econet: Failed to send packet to network %d station %d (%s port %u)",
								            (unsigned int)BeebTx.eh.destnet, (unsigned int)BeebTx.eh.deststn,
								            IpAddressStr(stations[i].inet_addr), (unsigned int)stations[i].port);
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

							// If there is a send error, remember the network and station
							// to prevent the user being notified on each retry.
							LastError.network = BeebTx.eh.destnet;
							LastError.station = BeebTx.eh.deststn;
						}
					}
				}
			}
		}

		// Receive data.
		if (!(ADLC.control1 & CONTROL_REG1_RX_RESET)) // RX reset off
		{
			if (BeebRx.Pointer < BeebRx.BytesInBuffer)
			{
				// There's something waiting to be given to the processor.
				if (ADLC.rxfptr < 3) // space in FIFO
				{
					#ifdef DEBUG_ECONET
					DebugTrace("EconetPoll: Time to give another byte to the Beeb\n");
					#endif

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
						// Set Frame Valid flag (this was last byte of frame).
						ADLC.rxffc |= 1;

						// Reset read for next packet.
						BeebRx.Pointer = 0;
						BeebRx.BytesInBuffer = 0;
					}
				}
			}

			if (ADLC.rxfptr == 0)
			{
				// Still nothing in buffers (and thus nothing in EconetRx buffer).
				ADLC.control1 &= ~CONTROL_REG1_RX_FRAME_DISCONTINUE;

				// Wait for CPU to clear Frame Valid flag from last frame received.
				if (!(ADLC.status2 & STATUS_REG2_FRAME_VALID))
				{
					if (!AUNMode ||
					    fourwaystage == FourWayStage::Idle ||
					    fourwaystage == FourWayStage::ImmediateSent ||
					    fourwaystage == FourWayStage::DataSent)
					{
						// Try to get another packet from the network.
						// Check if packet is waiting without blocking.
						fd_set ReadFds;
						FD_ZERO(&ReadFds);
						FD_SET(Socket, &ReadFds);

						timeval TimeOut = {0, 0};

						int NumReady = select((int)Socket + 1, &ReadFds, NULL, NULL, &TimeOut);

						if (NumReady > 0)
						{
							// Read the packet.
							sockaddr_in RecvAddr;
							int RecvAddrSize = sizeof(RecvAddr);
							int BytesReceived;

							if (AUNMode)
							{
								BytesReceived = recvfrom(Socket,
								                         (char *)&EconetRx,
								                         sizeof(EconetRx.ah) + sizeof(EconetRx.buff),
								                         0,
								                         (SOCKADDR *)&RecvAddr,
								                         &RecvAddrSize);

								EconetRx.BytesInBuffer = BytesReceived;
							}
							else
							{
								BytesReceived = recvfrom(Socket,
								                         (char *)BeebRx.buff,
								                         sizeof(BeebRx.buff),
								                         0,
								                         (SOCKADDR *)&RecvAddr,
								                         &RecvAddrSize);
							}

							if (BytesReceived > 0)
							{
								if (DebugEnabled)
								{
									DebugDisplayTraceF(DebugType::Econet,
									                   true,
									                   "EconetPoll: Packet received: %d bytes from %s port %u",
									                   BytesReceived,
									                   IpAddressStr(S_ADDR(RecvAddr)),
									                   htons(RecvAddr.sin_port));

									std::string str = "EconetPoll: Packet data:" + BytesToString(AUNMode ? (const unsigned char*)&EconetRx : BeebRx.buff, BytesReceived);

									DebugDisplayTrace(DebugType::Econet, true, str.c_str());
								}

								if (AUNMode)
								{
									// Convert from AUN format.
									// Find station number of sender.
									EconetHost* pHost = FindHost(&RecvAddr);

									if (pHost == nullptr)
									{
										// Packet from unknown host.
										if (LearnMode)
										{
											pHost = AddHost(&RecvAddr);
										}
									}

									if (pHost == nullptr)
									{
										// Didn't find it in the table. Ignore the packet.
										BeebRx.BytesInBuffer = 0;

										#ifdef DEBUG_ECONET
										DebugTrace("Econet: Packet ignored\n");
										#endif
									}
									else
									{
										if (DebugEnabled)
										{
											DebugDisplayTraceF(DebugType::Econet,
											                   true,
											                   "Econet: Packet was from %d.%d",
											                   (int)pHost->network,
											                   (int)pHost->station);
										}

										switch (fourwaystage)
										{
										case FourWayStage::Idle:
											// We weren't doing anything when this packet came in.
											BeebRx.eh.deststn = EconetStationID; // Must be for us.
											BeebRx.eh.destnet = 0;

											BeebRx.eh.srcstn = pHost->station;
											BeebRx.eh.srcnet = pHost->network;

											BeebRx.eh.cb = EconetRx.ah.cb | 128;
											BeebRx.eh.port = EconetRx.ah.port;

											switch (EconetRx.ah.type)
											{
												case AUNType::Broadcast: {
													BeebRx.eh.deststn = 255; // Wasn't just for us..
													BeebRx.eh.destnet = 255;

													const int Offset = sizeof(LongEconetPacket);
													const int Length = BytesReceived - sizeof(EconetRx.ah);
													memcpy(BeebRx.buff + Offset, EconetRx.buff, Length);
													BeebRx.BytesInBuffer = Offset + Length;

													fourwaystage = FourWayStage::WaitForIdle;

													#ifdef DEBUG_ECONET
													DebugTrace("Econet: Set FourWayStage::WaitForIdle (broadcast received)\n");
													#endif
													break;
												}

												case AUNType::Immediate: {
													const int Offset = sizeof(LongEconetPacket);
													const int Length = BytesReceived - sizeof(EconetRx.ah);
													memcpy(BeebRx.buff + Offset, EconetRx.buff, Length);
													BeebRx.BytesInBuffer = Offset + Length;

													fourwaystage = FourWayStage::ImmediateReceived;

													#ifdef DEBUG_ECONET
													DebugTrace("Econet: Set FourWayStage::ImmediateReceived\n");
													#endif
													break;
												}

												case AUNType::Unicast:
													// We're assuming things here.
													if (EconetRx.ah.port == 0 && EconetRx.ah.cb == (0x82 & 0x7f))
													{
														const int Offset = sizeof(LongEconetPacket);
														const int Length = 8;
														memcpy(BeebRx.buff + Offset, EconetRx.buff, Length);
														BeebRx.BytesInBuffer = Offset + Length;
													}
													else if (EconetRx.ah.port == 0 &&
													         EconetRx.ah.cb >= (0x83 & 0x7f) &&
													         EconetRx.ah.cb <= (0x85 & 0x7f))
													{
														const int Offset = sizeof(LongEconetPacket);
														const int Length = 4;
														memcpy(BeebRx.buff + Offset, EconetRx.buff, Length);
														BeebRx.BytesInBuffer = Offset + Length;
													}
													else
													{
														BeebRx.BytesInBuffer = sizeof(LongEconetPacket);
													}

													fourwaystage = FourWayStage::ScoutReceived;

													#ifdef DEBUG_ECONET
													DebugTrace("Econet: Set FourWayStage::ScoutReceived\n");
													#endif
													break;

												default:
													// Ignore anything else,
													BeebRx.BytesInBuffer = 0;
													break;
											}

											BeebRx.Pointer = 0;
											break;

										case FourWayStage::ImmediateSent: {
											// It should be reply to an immediate instruction.
											// TODO  check that it is!!! Example scenario where it will not
											// be - *STATIONs poll sends packet to itself... packet we get
											// here is the one we just sent out..!!!
											// I'm pretty sure that real Econet can't send to itself.
											BeebRx.eh.deststn = EconetStationID; // must be for us.
											BeebRx.eh.destnet = 0;

											BeebRx.eh.srcstn = pHost->station;
											BeebRx.eh.srcnet = pHost->network;

											const int Offset = 4;
											const int Length = BytesReceived - sizeof(EconetRx.ah);
											memcpy(BeebRx.buff + Offset, EconetRx.buff, Length);
											BeebRx.BytesInBuffer = Offset + Length;
											BeebRx.Pointer = 0;

											fourwaystage = FourWayStage::WaitForIdle;

											#ifdef DEBUG_ECONET
											DebugTrace("Econet: Set FourWayStage::WaitForIdle (ack received from remote AUN server)\n");
											#endif
											break;
										}

										case FourWayStage::DataSent:
											// We sent block of data, awaiting final ack.
											if (EconetRx.ah.type == AUNType::Ack || EconetRx.ah.type == AUNType::NAck)
											{
												// Are we expecting a (N)ACK?
												// TODO check it is a (n)ack for the packet we just sent. Deal with nacks!
												// Construct a final ack for the Beeb.
												BeebRx.eh.deststn = EconetStationID; // must be for us.
												BeebRx.eh.destnet = 0;

												BeebRx.eh.srcstn = pHost->station;
												BeebRx.eh.srcnet = pHost->network;

												BeebRx.BytesInBuffer = 4;
												BeebRx.Pointer = 0;

												fourwaystage = FourWayStage::WaitForIdle;

												#ifdef DEBUG_ECONET
												DebugTrace("Econet: Set FourWayStage::WaitForIdle (AUN ack received)\n");
												#endif
												break;
											} // else unexpected packet - ignore it. TODO: queue it?

										default:
											// Erm, what are we doing here? Ignore packet.
											fourwaystage = FourWayStage::WaitForIdle;

											#ifdef DEBUG_ECONET
											DebugTrace("Econet: Set FourWayStage::WaitForIdle (ack received from remote AUN server)\n");
											#endif
											break;
										}
									}
								}
								else
								{
									BeebRx.BytesInBuffer = BytesReceived;
									BeebRx.Pointer = 0;
								}

								if ((BeebRx.eh.deststn == EconetStationID || IsBroadcastStation(BeebRx.eh.deststn)) &&
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
									// Two other stations communicating - assume one of them will flag fill.
									FlagFillActive = true;
									SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);

									#ifdef DEBUG_ECONET
									DebugTrace("Econet: FlagFill set - other station comms\n");
									#endif
								}
							}
							/* else if (RetVal == SOCKET_ERROR)
							{
								EconetError("Econet: Failed to receive packet (error %ld)", GetLastSocketError());
							} */
						}
						else if (NumReady == SOCKET_ERROR)
						{
							EconetError("Econet: Failed to check for new packet");
						}
					}

					// This bit fakes the bits of the 4-way handshake that AUN doesn't do.

					if (AUNMode && EconetScoutAckTrigger > TotalCycles)
					{
						switch (fourwaystage) {
						case FourWayStage::ScoutSent:
							// Just got a scout from the Beeb, fake an acknowledgement.
							BeebRx.eh.deststn = EconetStationID;
							BeebRx.eh.destnet = 0;

							BeebRx.eh.srcstn = (unsigned char)EconetTx.deststn; // Use scout's dest as source of ack.
							BeebRx.eh.srcnet = (unsigned char)EconetTx.destnet;

							BeebRx.BytesInBuffer = 4;
							BeebRx.Pointer = 0;

							fourwaystage = FourWayStage::ScoutAckReceived;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Set FourWayStage::ScoutAckReceived\n");
							#endif
							break;

						case FourWayStage::ScoutAckSent: {
							// Beeb acked the scout we gave it, so give it the data AUN sent us earlier.
							BeebRx.eh.deststn = EconetStationID; // As it is data it must be for us.
							BeebRx.eh.destnet = 0;

							BeebRx.eh.srcstn  = (unsigned char)EconetTx.deststn;  //30jun dont think this is right..
							BeebRx.eh.srcnet  = (unsigned char)(EconetTx.destnet & inmask);

							const int DestOffset = sizeof(EconetHeader);

							if (EconetRx.ah.port == 0 && EconetRx.ah.cb == (0x82 & 0x7f))
							{
								const int SrcOffset = 8;
								const int Length = EconetRx.BytesInBuffer - sizeof(EconetRx.ah) - SrcOffset;
								memcpy(BeebRx.buff + DestOffset, EconetRx.buff + SrcOffset, Length);
								BeebRx.BytesInBuffer = DestOffset + Length;
							}
							else if (EconetRx.ah.port == 0 &&
							         EconetRx.ah.cb >= (0x83 & 0x7f) &&
							         EconetRx.ah.cb <= (0x85 & 0x7f))
							{
								const int SrcOffset = 4;
								const int Length = EconetRx.BytesInBuffer - sizeof(EconetRx.ah) - SrcOffset;
								memcpy(BeebRx.buff + DestOffset, EconetRx.buff + SrcOffset, Length);
								BeebRx.BytesInBuffer = DestOffset + Length;
							}
							else
							{
								const int Length = EconetRx.BytesInBuffer - sizeof(EconetRx.ah);
								memcpy(BeebRx.buff + DestOffset, EconetRx.buff, Length);
								BeebRx.BytesInBuffer = DestOffset + Length;
							}

							BeebRx.Pointer = 0;

							fourwaystage = FourWayStage::DataReceived;

							#ifdef DEBUG_ECONET
							DebugTrace("Econet: Set FourWayStage::DataReceived\n");
							#endif
							break;
						}

						default:
							break;
						}
					}
				}
			}
		}

		// Update idle status
		if (!(ADLC.control1 & CONTROL_REG1_RX_RESET) && // Not RxReset
		    ADLC.rxfptr == 0 && // Nothing in FIFO
		    !(ADLC.status2 & STATUS_REG2_FRAME_VALID) && // No FV
		    BeebRx.BytesInBuffer == 0) // Nothing in IP buffer
		{
			ADLC.idle = true;
		}
		else
		{
			ADLC.idle = false;
		}

		// How long before we come back in here?
		SetTrigger(TimeBetweenBytes, EconetTrigger);
	}

	// Reset pseudo flag fill?
	if (EconetFlagFillTimeoutTrigger <= TotalCycles && FlagFillActive)
	{
		FlagFillActive = false;

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: FlagFill timeout reset\n");
		#endif
	}

	// waiting for AUN to become idle?
	if (AUNMode &&
	    fourwaystage == FourWayStage::WaitForIdle &&
	    BeebRx.BytesInBuffer == 0 &&
	    ADLC.rxfptr == 0 &&
	    ADLC.txfptr == 0 // ??
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

		#ifdef DEBUG_ECONET
		DebugTrace("Econet: FourWayStage timeout. Set FourWayStage::Idle\n");
		#endif
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

	if (Socket != INVALID_SOCKET && (ADLC.control2 & CONTROL_REG2_RTS_CONTROL)) // clock + RTS
	{
		ADLC.cts = false;
		ADLC.status1 &= ~STATUS_REG1_CTS;
	}
	else
	{
		ADLC.cts = true;
	}

	// And then set the status bit if the line is high! (status bit stays
	// up until the CPU tries to clear it) (and still stays up if the CTS
	// line is still high)

	if (!(ADLC.control1 & CONTROL_REG1_RX_RESET) && ADLC.cts)
	{
		ADLC.status1 |= STATUS_REG1_CTS; // set CTS now
	}

	// SR1b5 - TXU - Tx Underrun.
	if (ADLC.txfptr > 4) // probably not needed
	{
		#ifdef DEBUG_ECONET
		DebugTrace("Econet: TX Underrun - TXfptr %02x\n", (unsigned int)ADLC.txfptr);
		#endif

		ADLC.status1 |= STATUS_REG1_TX_UNDERRUN;
		ADLC.txfptr = 4;
	}

	// SR1b6 TDRA flag - another complicated derivation
	if (!(ADLC.control1 & CONTROL_REG1_TX_RESET)) // not TxReset
	{
		if (!(ADLC.control2 & CONTROL_REG2_TDRA_SELECT)) // TDRA mode
		{
			if (   (   ((ADLC.txfptr < 3) && !(ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER)) // space in FIFO?
			        || ((ADLC.txfptr < 2) && (ADLC.control2 & CONTROL_REG2_2_BYTE_TRANSFER))) // space in FIFO?
			    && (!(ADLC.status1 & STATUS_REG1_CTS)) // Clear to send is ok
			    && (!(ADLC.status2 & STATUS_REG2_DCD)) ) // DTR not high
			{
				#ifdef DEBUG_ECONET
				if (!(ADLC.status1 & STATUS_REG1_TDRA))
				{
					DebugTrace("ADLC: Set TDRA\n");
				}
				#endif

				ADLC.status1 |= STATUS_REG1_TDRA;
			}
			else
			{
				#ifdef DEBUG_ECONET
				if ((ADLC.status1 & STATUS_REG1_TDRA))
				{
					DebugTrace("ADLC: Clear TDRA\n");
				}
				#endif

				ADLC.status1 &= ~STATUS_REG1_TDRA;
			}
		}
		else // FC mode
		{
			if (ADLC.txfptr == 0) // Nothing in FIFO.
			{
				#ifdef DEBUG_ECONET
				if (!(ADLC.status1 & STATUS_REG1_TDRA))
				{
					DebugTrace("ADLC: Set FC\n");
				}
				#endif

				ADLC.status1 |= STATUS_REG1_TDRA;
			}
			else
			{
				#ifdef DEBUG_ECONET
				if (ADLC.status1 & STATUS_REG1_TDRA)
				{
					DebugTrace("ADLC: Clear FC\n");
				}
				#endif

				ADLC.status1 &= ~STATUS_REG1_TDRA;
			}
		}
	}
	// SR1b7 IRQ flag - see below

	// SR2b0 - AP - Address Present
	if (!(ADLC.control1 & CONTROL_REG1_RX_RESET))
	{
		if (ADLC.rxfptr > 0 &&
		    (ADLC.rxap & (powers[ADLC.rxfptr - 1]))) // AP bits set on FIFO
		{
			ADLC.status2 |= STATUS_REG2_ADDRESS_PRESENT;
		}
		else
		{
			ADLC.status2 &= ~STATUS_REG2_ADDRESS_PRESENT;
		}

		// SR2b1 - FV - Frame Valid - set in RX - only reset by ClearRx or RxReset
		if (ADLC.rxfptr > 0 &&
		    (ADLC.rxffc & (powers[ADLC.rxfptr - 1])))
		{
			ADLC.status2 |= STATUS_REG2_FRAME_VALID;
		}

		// SR2b2 - Inactive Idle Received - sets IRQ!
		if (ADLC.idle && !FlagFillActive)
		{
			ADLC.status2 |= STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
		else
		{
			ADLC.status2 &= ~STATUS_REG2_INACTIVE_IDLE_RECEIVED;
		}
	}

	// SR2b3 - RxAbort - Abort received - set in RX routines above
	// SR2b4 - Error during reception - set if error flagged in RX routine.
	// SR2b5 - DCD
	if (Socket == INVALID_SOCKET) // is line down?
	{
		ADLC.status2 |= STATUS_REG2_DCD; // Flag error
	}
	else
	{
		ADLC.status2 &= ~STATUS_REG2_DCD;
	}

	// SR2b6 - OVRN - Receipt Overrun. Probably not needed.
	if (ADLC.rxfptr > 4)
	{
		ADLC.status2 |= STATUS_REG2_RX_OVERRUN;
		ADLC.rxfptr = 4;
	}

	// SR2b7 - RDA. As per SR1b0 - set above.

	// Handle PSE - only for SR2 Rx bits at the moment.

	#ifdef DEBUG_ECONET
	int PrevSr2pse = ADLC.sr2pse;
	#endif

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
			ADLC.sr2pse = 0; // No relevant bits set.
		}

		// Set SR1 RDA copy.
		if (ADLC.status2 & STATUS_REG2_RX_DATA_AVAILABLE)
		{
			ADLC.status1 |= STATUS_REG1_RX_DATA_AVAILABLE;
		}
		else
		{
			ADLC.status1 &= ~STATUS_REG1_RX_DATA_AVAILABLE;
		}
	}
	else
	{
		// PSE inactive.
		ADLC.sr2pse = 0;
	}

	#ifdef DEBUG_ECONET
	if (ADLC.sr2pse != PrevSr2pse)
	{
		DebugTrace("ADLC: PSE SR2Rx priority changed to %d\n", ADLC.sr2pse);
	}
	#endif

	// Do we need to flag an interrupt?
	if (ADLC.status1 != ADLCtemp.status1 || ADLC.status2 != ADLCtemp.status2) // Something changed.
	{
		// SR1b1 - S2RQ - Status2 request. New bit set in S2?
		unsigned char tempcause = ((ADLC.status2 ^ ADLCtemp.status2) & ADLC.status2) & ~STATUS_REG2_RX_DATA_AVAILABLE;

		if (!(ADLC.control1 & CONTROL_REG1_RX_INT_ENABLE))
		{
			tempcause = 0;
		}

		if (tempcause) // Something got set.
		{
			ADLC.status1 |= STATUS_REG1_STATUS2_READ_REQUEST;
			sr1b2cause = sr1b2cause | tempcause;
		}
		else if (!(ADLC.status2 & sr1b2cause)) // Cause has gone.
		{
			ADLC.status1 &= ~STATUS_REG1_STATUS2_READ_REQUEST;
			sr1b2cause = 0;
		}

		// New bit set in S1?
		tempcause = ((ADLC.status1 ^ ADLCtemp.status1) & ADLC.status1) & ~STATUS_REG1_IRQ;

		if (!(ADLC.control1 & CONTROL_REG1_RX_INT_ENABLE))
		{
			tempcause &= ~(STATUS_REG1_RX_DATA_AVAILABLE |
			               STATUS_REG1_STATUS2_READ_REQUEST |
			               STATUS_REG1_FLAG_DETECTED);
		}

		if (!(ADLC.control1 & CONTROL_REG1_TX_INT_ENABLE))
		{
			tempcause &= ~(STATUS_REG1_CTS |
			               STATUS_REG1_TX_UNDERRUN |
			               STATUS_REG1_TDRA);
		}

		if (tempcause != 0) // Something got set.
		{
			Interrupt = true;
			irqcause |= tempcause; // Remember which bit went high to flag IRQ.

			ADLC.status1 |= STATUS_REG1_IRQ;

			#ifdef DEBUG_ECONET
			DebugTrace("ADLC: Status1 bit got set %02x, interrupt\n", (int)tempcause);
			#endif
		}

		// Bit cleared in S1?
		unsigned char temp2 = ((ADLC.status1 ^ ADLCtemp.status1) & ADLCtemp.status1) & ~STATUS_REG1_IRQ;

		if (temp2 != 0) // Something went off.
		{
			irqcause &= ~temp2; // Clear flags that went off.

			if (irqcause == 0) // All flags gone off now.
			{
				// Clear IRQ status bit when cause has gone.
				ADLC.status1 &= ~STATUS_REG1_IRQ;
			}
			else
			{
				// Interrupt again because we still have flags set.
				if (ADLC.control2 & CONTROL_REG2_PRIORITIZED_STATUS_ENABLE)
				{
					Interrupt = true;

					#ifdef DEBUG_ECONET
					DebugTrace("ADLC: S1 flags still set, interrupt\n");
					#endif
				}
			}

			#ifdef DEBUG_ECONET
			DebugTrace("ADLC: IRQ cause reset, irqcause %02x\n",(int)irqcause);
			#endif
		}
	}

	// Flag NMI if necessary. See also INTON flag as
	// this can cause a delayed interrupt (BeebMem.cpp).
	return Interrupt;
}

//--------------------------------------------------------------------------------------------

void DebugEconetState()
{
	DebugDisplayTraceF(DebugType::Econet, true,
	                   "ADLC: Ctl:%02X %02X %02X %02X St:%02X %02X TXptr:%01x rx:%01x FF:%d IRQc:%02x SR2c:%02x PC:%04x 4W:%i",
	                   (int)ADLC.control1, (int)ADLC.control2, (int)ADLC.control3, (int)ADLC.control4,
	                   (int)ADLC.status1, (int)ADLC.status2,
	                   (int)ADLC.txfptr, (int)ADLC.rxfptr, FlagFillActive ? 1 : 0,
	                   (int)irqcause, (int)sr1b2cause, (int)ProgramCounter, (int)fourwaystage);
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
