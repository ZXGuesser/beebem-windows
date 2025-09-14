/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 1994  David Alan Gilbert

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

/* Support file for 6522 via - 30/10/94 - David Alan Gilbert */

#ifndef VIA_HEADER
#define VIA_HEADER

#include <stdio.h>

struct VIAState {
  unsigned char ora,orb;
  unsigned char ira,irb;
  unsigned char ddra,ddrb;
  unsigned char acr,pcr;
  unsigned char ifr,ier;
  int timer1c,timer2c; /* NOTE: Timers descrement at 2MHz and values are */
  int timer1l,timer2l; /*   fixed up on read/write - latches hold 1MHz values*/
  bool timer1hasshot; // True if we have already caused an interrupt for one shot mode
  bool timer2hasshot; // True if we have already caused an interrupt for one shot mode
  int timer1adjust; // Adjustment for 1.5 cycle counts, every other interrupt, it becomes 2 cycles instead of one
  int timer2adjust; // Adjustment for 1.5 cycle counts, every other interrupt, it becomes 2 cycles instead of one
  unsigned char sr;
  bool ca2;
  bool cb2;
};

// 6522 Interrupt Flags Register
constexpr unsigned char IFR_CA2      = 0x01;
constexpr unsigned char IFR_CA1      = 0x02;
constexpr unsigned char IFR_SHIFTREG = 0x04;
constexpr unsigned char IFR_CB2      = 0x08;
constexpr unsigned char IFR_CB1      = 0x10;
constexpr unsigned char IFR_TIMER2   = 0x20;
constexpr unsigned char IFR_TIMER1   = 0x40;
constexpr unsigned char IFR_IRQ      = 0x80;

// 6522 Interrupt Enable Register
constexpr unsigned char IER_CA2       = 0x01;
constexpr unsigned char IER_CA1       = 0x02;
constexpr unsigned char IER_SHIFTREG  = 0x04;
constexpr unsigned char IER_CB2       = 0x08;
constexpr unsigned char IER_CB1       = 0x10;
constexpr unsigned char IER_TIMER2    = 0x20;
constexpr unsigned char IER_TIMER1    = 0x40;
constexpr unsigned char IER_SET_CLEAR = 0x80;

// 6522 Auxiliary Control Register
constexpr unsigned char ACR_PA_LATCH_ENABLE      = 0x01;
constexpr unsigned char ACR_PB_LATCH_ENABLE      = 0x02;
constexpr unsigned char ACR_PB_SHIFTREG_CONTROL  = 0x1c;
constexpr unsigned char ACR_TIMER2_CONTROL       = 0x20;
constexpr unsigned char ACR_TIMER1_CONTINUOUS    = 0x40;
constexpr unsigned char ACR_TIMER1_OUTPUT_ENABLE = 0x80;

// 6522 Peripheral Control Register
constexpr unsigned char PCR_CB2_CONTROL           = 0xe0;
constexpr unsigned char PCR_CB1_INTERRUPT_CONTROL = 0x10;
constexpr unsigned char PCR_CA2_CONTROL           = 0x0e;
constexpr unsigned char PCR_CA1_INTERRUPT_CONTROL = 0x01;

// PCR CB2 control bits
constexpr unsigned char PCR_CB2_OUTPUT_PULSE = 0xa0;
constexpr unsigned char PCR_CB2_OUTPUT_LOW   = 0xc0;
constexpr unsigned char PCR_CB2_OUTPUT_HIGH  = 0xe0;

// PCR CB1 interrupt control bit
constexpr unsigned char PCB_CB1_POSITIVE_INT = 0x10;

// PCR CA2 control bits
constexpr unsigned char PCR_CA2_OUTPUT_PULSE = 0x0a;
constexpr unsigned char PCR_CA2_OUTPUT_LOW   = 0x0c;
constexpr unsigned char PCR_CA2_OUTPUT_HIGH  = 0x0e;

// PCR CA1 interrupt control bit
constexpr unsigned char PCB_CA1_POSITIVE_INT = 0x01;

void VIAReset(VIAState *ToReset);
void SaveVIAUEF(FILE *SUEF);
void LoadViaUEF(FILE *SUEF);

#endif
