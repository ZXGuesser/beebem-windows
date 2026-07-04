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

// Econet for BeebEm
// Written by Rob O'Donnell. robert@irrelevant.com
// Mike Wyatt - further development, Dec 2005

#ifndef ECONET_HEADER
#define ECONET_HEADER

bool EconetReset();
unsigned char EconetRead(unsigned char Register);
void EconetWrite(unsigned char Register, unsigned char Value);
unsigned char EconetReadStationID();
bool EconetInterruptRequest();
bool EconetPoll();
void DebugEconetState();

// Config settings
struct EconetConfigType
{
	// Massage network numbers on send/receive (add/sub 128)
	bool MassageNetworks;
	// Enable station autoconfiguration and discovery features
	bool AutoConfigure;
	// Enable gateway discovery
	bool FindGateways;
	// Fixed gateway IP address and port (if FindGateways is false)
	unsigned long GatewayIPAddress;
	unsigned short GatewayPort;
	// Cycles for flag fill timeout
	int FlagFillTimeout;
	// Cycles to delay before sending ack to scout (AUN mode only)
	int ScoutAckTimeout;
	// Frequency between network actions.
	// max 250Khz network clock. 2MHz system clock. one click every 8 cycles.
	// say one byte takes about 8 clocks, receive a byte every 64 cpu cycles. ?
	// (The reason for "about" 8 clocks is that as this a continuous synchronous tx,
	// there are no start/stop bits, however to avoid detecting a dead line as ffffff
	// zeros are added and removed transparently if you get more than five "1"s
	// during data transmission - more than 5 are flags or errors)
	// 6854 datasheet has max clock frequency of 1.5MHz for the B version.
	// 64 cycles seems to be a bit fast for 'netmon' prog to keep up - set to 128.
	unsigned int TimeBetweenBytes;
	unsigned int FourWayStageTimeout;
};

extern EconetConfigType EconetConfig;

extern bool EconetEnabled;
extern bool EconetNMIEnabled;
extern bool EconetStateChanged;
extern int EconetTrigger;
extern int EconetFlagFillTimeoutTrigger;

extern unsigned char EconetStationID;
extern unsigned char EconetNetworkID;
extern unsigned char PreferredStationID;
extern unsigned char PreferredNetworkID;

extern char EconetCfgPath[MAX_PATH];
extern char AUNMapPath[MAX_PATH];

// #define DEBUG_ECONET
// #define DEBUG_ECONET_ADLC
// #define DEBUG_ECONET_ADLC_FIFO
// #define DEBUG_ECONET_INTERRUPTS

#endif
