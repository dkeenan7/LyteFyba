/*
 * Tritium MSP430 2xx USCI SPI interface
 * Copyright (c) 2009, Tritium Pty Ltd.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * - Implements the following SPI interface functions
 *	- init
 *	- transmit
 *	- exchange
 *
 *	Also now UART functions
 *	- chgr_transmit
 *
 */

// Include files
#include <msp430x24x.h>
#include "tri86.h"
#include "usci.h"
#include <signal.h>			// For interrupt() macro
#include <string.h>			// For memcpy()

/*
 * Initialise SPI port
 * 	- Master, 8 bits, mode 0:0, max speed, 3 wire
 * 	- Clock = 0: SMCLK /2 (works with no external clock input, DCO only)
 *	- Clock = 1: ACLK  /2 (fastest possible, from external clock input)
 */
void usci_init( unsigned char clock )
{
	P3SEL |= CAN_MOSI | CAN_MISO | CAN_SCLK;			// Set pins to peripheral function, not GPIO
	UCB0CTL0 |= UCMST | UCSYNC | UCCKPL | UCMSB;		// 3-pin, 8-bit SPI master
	if( clock == 0 ) UCB0CTL1 = UCSSEL_2 | UCSWRST;		// BRCLK = SMCLK
	else UCB0CTL1 = UCSSEL_1 | UCSWRST;					// BRCLK = ACLK
	UCB0BR0 = 0x02;										// /2
	UCB0BR1 = 0;										//
	UCB0CTL1 &= ~UCSWRST;								// Initialize USCI state machine
}

/*
 * Transmits data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- On devices with hardware SPI support, this function is identical to spi_exchange,
 *	  with the execption of not returning a value
 *	- On devices with software (bit-bashed) SPI support, this function can run faster
 *	  because it does not require data reception code
 */
void usci_transmit( unsigned char data )
{
	UCB0TXBUF = data;
	while(( IFG2 & UCB0RXIFG ) == 0x00 );			// Wait for Rx completion (implies Tx is also complete)
	UCB0RXBUF;
}

/*
 * Exchanges data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- This function is safe to use to control hardware lines that rely on shifting being finalised
 */
unsigned char usci_exchange( unsigned char data )
{
	UCB0TXBUF = data;
	while(( IFG2 & UCB0RXIFG ) == 0x00 );			// Wait for Rx completion (implies Tx is also complete)
	return( UCB0RXBUF );
}

// USCI0 A0/B0 Transmit ISR.
// #pragma vector=USCIAB0TX_VECTOR
interrupt(USCIAB0TX_VECTOR) usciab0tx(void)
{
	if (IFG2 & UCA0TXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		unsigned char ch = chgr_txbuf[chgr_txidx++];
		events |= EVENT_ACTIVITY;				// Turn on activity light

		if (chgr_txidx == 12) {					// TX over?
			IE2 &= ~UCA0TXIE;					// Disable USCI_A0 TX interrupt
			chgr_txidx = 0;
		}
		UCA0TXBUF = ch;							// TX next character
	}
}

// For use with A1/B1, need separate ISR, using UC1IFG instead of IFG2, and UC1IE for IE2.
// #pragma vector=USCIAB1TX_VECTOR
interrupt(USCIAB1TX_VECTOR) usciab1tx(void)
{
	if (UC1IFG & UCA1TXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		unsigned char ch = bmu_txbuf[bmu_txidx++];// Next character
		events |= EVENT_ACTIVITY;				// Turn on activity light

		if (ch == '\r') {						// TX over? All commands terminated with return
			UC1IE &= ~UCA1TXIE;					// Disable USCI_A1 TX interrupt
			bmu_txidx = 0;
		}
		UCA1TXBUF = ch;							// Transmit this byte
	}
}

// USCI0 A0/B0 Receive ISR
//  #pragma vector=USCIAB0RX_VECTOR
interrupt(USCIAB0RX_VECTOR) usciab0rx(void)
{
	if (IFG2 & UCA0RXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		chgr_rxbuf[chgr_rxidx++] = UCA0RXBUF;
		events |= EVENT_ACTIVITY;				// Turn on activity light
		if (chgr_rxidx >= 12)
		{
			chgr_rxidx = 0;
			chgr_events |= CHGR_REC;			// Tell main line we've received a charger packet
			chgr_events &= ~CHGR_SENT;			// No longer unacknowledged
		}
	}
}

// USCI1 A0/B0 Receive ISR
//  #pragma vector=USCIAB1RX_VECTOR
interrupt(USCIAB1RX_VECTOR) usciab1rx(void)
{
	if (UC1IFG & UCA1RXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		unsigned char ch = UCA1RXBUF;
		events |= EVENT_ACTIVITY;				// Turn on activity light
		if (ch >= 0x80) {
			bmu_events |= BMU_BADNESS;
			bmu_badness = ch;
		} else {
			bmu_rxbuf[bmu_rxidx++] = ch;
			if (ch == '\r')						// All BMU responses terminate with a return
			{
				bmu_rxidx = 0;
				bmu_events |= BMU_REC;			// Tell main line we've received a BMU response
				bmu_events &= ~BMU_SENT;		// No longer unacknowledged (not used yet)
			if (bmu_rxidx == 64)	bmu_rxidx = 0; // DCK: Avoid buffer overrun
			}
		}
	}
}

void chgr_transmit(const unsigned char* ptr)
{
	memcpy(chgr_txbuf, ptr, 12);					// Copy the data to the transmit buffer
	chgr_transmit_buf();							// Tail call the main transmit function
}

// chgr_transmit_buf sends the transmit buffer. Used for resending after a timeout.
void chgr_transmit_buf(void)
{
	chgr_txidx = 0;
    UCA0TXBUF = chgr_txbuf[chgr_txidx++];			// Send the first char to kick things off
	chgr_events |= CHGR_SENT;						// Flag that packet is sent but not yet ack'd
	chgr_sent_timeout = CHGR_TIMEOUT;				// Initialise timeout counter
    IE2 |= UCA0TXIE;                        		// Enable USCI_A0 TX interrupt
	events |= EVENT_ACTIVITY;						// Turn on activity light
}


void bmu_transmit(const unsigned char* ptr)
{
	unsigned char ch, i = 0;
	do {
		ch = *ptr++;
		bmu_txbuf[i++] = ch;						// Copy the data to the transmit buffer
	} while (ch != '\r');
	bmu_transmit_buf();								// Tail call the main transmit function
}

// bmu_transmit_buf sends the transmit buffer. Used for resending after a timeout.
void bmu_transmit_buf(void)
{
	bmu_txidx = 0;
    UCA1TXBUF = bmu_txbuf[bmu_txidx++];				// Send the first char to kick things off
	bmu_events |= BMU_SENT;							// Flag that packet is sent but not yet ack'd
    UC1IE |= UCA1TXIE;                        		// Enable USCI_A1 TX interrupt
	events |= EVENT_ACTIVITY;						// Turn on activity light
}
