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

#ifdef __ICC430__								// MVE: attempt to make the source code more IAR friendly
#define __inline__								//	in case press F7, and so definitions might work
#define __volatile__
#define interrupt(x) void
void eint();
void dint();
#endif

/* Enqueue and Dequeue general queueing functions */
// Enqueue a byte. Returns true on success (queue not full)
// Declared static so the compiler can optimise out the body if it inlines all calls
static bool enqueue(
  volatile unsigned char* buf,			// The buffer
		   unsigned char rd,			// The read index
  volatile unsigned char* wr,			// *Pointer to* the write index
		   unsigned int bufSize,		// The buffer size, pass a constant
		   unsigned char ch)			// The byte to enqueue
{
	unsigned char wr_copy = *wr;		// Make a copy of the write index
	buf[wr_copy++] = ch;				// Tentatively write the byte to the queue; there is always
										//	one free space, but don't update write index yet
										// Also increments the index copy
	wr_copy &= (bufSize-1);				//	modulo the buffer size
	if (wr_copy == rd)					// Does the incremented write pointer equal the read pointer?
		return false;					// Yes, error return
	*wr = wr_copy;						// Update write pointer; byte is officially in the queue now
	return true;						// Normal return
}

// Dequeue a byte. Returns true on success (queue not empty). 
static bool dequeue(
	volatile unsigned char* buf,		// The buffer
	volatile unsigned char* rd,			// *Pointer to* the read index
			 unsigned char wr,			// The write index
			 unsigned int bufSize,		// Buffer size (pass a constant)
			 unsigned char* ch)			// Pointer to the byte to be read to
{
	unsigned char rd_copy = *rd;
	if (wr == rd_copy)					// Indexes equal?
		return false;					// If so, buffer is empty
	*ch = buf[rd_copy++];				// Read the byte, increment read index
	rd_copy &= (bufSize-1);				//	modulo the buffer size
	*rd = rd_copy;						// Atomic update
	return true;
}

// Returns the number of bytes in the queue.
static unsigned int queue_length(
				unsigned char rd,		// Read index
				unsigned char wr,		// Write index
				unsigned int bufSize)	// Buffer size
{
	return (wr - rd) & (bufSize-1);
}

// Amouunt of space in the queue. This is the capacity of the queue minus the number already in the queue.
// The capacity is actually bufSize-1, so space = (bufSize-1 - (wr - rd)) & bufSize-1, which is the same
// as (wr - rd - 1) & (bufSize-1)
static unsigned int queue_space(
				unsigned char rd,		// Read index
				unsigned char wr,		// Write index
				unsigned int bufSize)	// Buffer size
{
	return (wr - rd - 1) & (bufSize-1);
}

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
		unsigned char ch;						// Get byte from the transmit queue
		if (!dequeue(chgr_txbuf, &chgr_txrd, chgr_txwr, CHGR_TX_BUFSZ, &ch))
			fault();							// Fault if queue is empty
		else {
			events |= EVENT_ACTIVITY;			// Turn on activity light
	
			if (queue_length(chgr_txrd, chgr_txwr, CHGR_TX_BUFSZ) == 0)	// TX complete?
				IE2 &= ~UCA0TXIE;				// Disable USCI_A0 TX interrupt
			UCA0TXBUF = ch;						// TX this byte
		}
	}
}

// For use with A1/B1, need separate ISR, using UC1IFG instead of IFG2, and UC1IE for IE2.
// #pragma vector=USCIAB1TX_VECTOR
interrupt(USCIAB1TX_VECTOR) usciab1tx(void)
{
	if (UC1IFG & UCA1TXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		unsigned char ch;						// Get byte from the transmit queue
		if (!dequeue(bmu_txbuf, &bmu_txrd, bmu_txwr, BMU_TX_BUFSZ, &ch))
			fault();							// Fault if queue is empty
		else {
			events |= EVENT_ACTIVITY;			// Turn on activity light

			if (queue_length(bmu_txrd, bmu_txwr, BMU_TX_BUFSZ) == 0)	// TX complete?
				UC1IE &= ~UCA1TXIE;				// Disable USCI_A1 TX interrupt
			UCA1TXBUF = ch;						// Transmit this byte
		}
	}
}

// USCI0 A0/B0 Receive ISR
//  #pragma vector=USCIAB0RX_VECTOR
interrupt(USCIAB0RX_VECTOR) usciab0rx(void)
{
	if (IFG2 & UCA0RXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		if (!enqueue(chgr_rxbuf, chgr_rxrd, &chgr_rxwr, CHGR_RX_BUFSZ, UCA0RXBUF))
			fault();							// Fault if queue is full
		else {
			events |= EVENT_ACTIVITY;				// Turn on activity light
			if (++chgr_rxcnt >= 12) {
				chgr_rxcnt = 0;
				chgr_events |= CHGR_REC;			// Tell main line we've received a charger packet
				chgr_events &= ~CHGR_SENT;			// No longer unacknowledged
			}
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
		if (ch >= 0x80) {
			bmu_events |= BMU_BADNESS;
			bmu_badness = ch;
		} else {
			if (!enqueue(bmu_rxbuf, bmu_rxrd, &bmu_rxwr, BMU_RX_BUFSZ, ch))
				fault();							// Fault if queue is full
			else {
				events |= EVENT_ACTIVITY;			// Turn on activity light
				if (ch == '\r')					// All BMU responses terminate with a return
				{
					bmu_events |= BMU_REC;		// Tell main line we've received a BMU response
					// Note: don't reset BMU_SENT till we have a completely valid response
				}
			}
		}
	}
}

unsigned char chgr_lastxmit[12];					// Last CAN message for charger

bool chgr_transmit(const unsigned char* ptr)
{
	memcpy(chgr_lastxmit, ptr, 12);					// Copy the data to the last message buffer
	return chgr_transmit_buf();						// Call the main transmit function
}

// chgr_transmit_buf sends the transmit buffer. Used for resending after a timeout.
// Returns true on success
bool chgr_transmit_buf(void)
{
	int i;
	if (queue_space(chgr_txrd, chgr_txwr, CHGR_RX_BUFSZ) < 11) {
		// Need 11 bytes in the queue (first byte is sent immediately)
		// If not, best to abort the whole command, rather than sending a partial message
		fault();
		return false;
	}
	chgr_txcnt = 0;									// No bytes transmitted yet
    UCA0TXBUF = chgr_lastxmit[0];					// Send the first char to kick things off
	for (i=1; i < 12; ++i)
		enqueue(chgr_txbuf, chgr_txrd, &chgr_txwr, CHGR_TX_BUFSZ, chgr_lastxmit[i]);
	chgr_events |= CHGR_SENT;						// Flag that packet is sent but not yet ack'd
	chgr_sent_timeout = CHGR_TIMEOUT;				// Initialise timeout counter
    IE2 |= UCA0TXIE;                        		// Enable USCI_A0 TX interrupt
	events |= EVENT_ACTIVITY;						// Turn on activity light
	return true;
}

unsigned char bmu_lastxmit[BMU_TX_BUFSZ];			// Copy of the last command sent to the BMUs

bool bmu_transmit(const unsigned char* ptr)
{
#if USE_CKSUM
	unsigned char ch, i = 0, sum = 0;
	do {
		ch = *ptr++;
		sum ^= ch;									// Calculate XOR checksum
		bmu_txbuf[i++] = ch;						// Copy the data to the transmit buffer
	} while (ch != '\r');
	sum ^= '\r';									// CR is not part of the checksum
	if (sum < ' ') {
		// If the checksum would be a control character that could be confused with a CR, BS,
		//	etc, then send a space, which will change the checksum to a non-control character
		bmu_txbuf[i++-1] = ' ';						// Replace CR with space
		sum ^= ' ';									// Update checksum
	}
	bmu_txbuf[i++-1] = sum;							// Insert the checksum
	bmu_txbuf[i-1] = '\r';							// Add CR
#endif
	return bmu_transmit_buf();						// Call the main transmit function
}

// bmu_transmit_buf sends the transmit buffer. Used for resending after a timeout.
// Returns true on success
bool bmu_transmit_buf(void)
{
	if (queue_space(bmu_txrd, bmu_txwr, BMU_TX_BUFSZ) < strlen((char*)bmu_lastxmit)-1) {
		fault();
		return false;
	}
    UCA1TXBUF = bmu_lastxmit[0];					// Send the first char to kick things off
	bmu_events |= BMU_SENT;							// Flag that packet is sent but not yet ack'd
	bmu_sent_timeout = BMU_TIMEOUT;					// Initialise timeout counter
    UC1IE |= UCA1TXIE;                        		// Enable USCI_A1 TX interrupt
	events |= EVENT_ACTIVITY;						// Turn on activity light
	return true;
}

// For tri86.c to call:
unsigned char bmu_getByte() {
	unsigned char ch = 0;
	dequeue(bmu_rxbuf, &bmu_rxrd, bmu_rxwr, BMU_RX_BUFSZ, &ch);
	return ch;
}
unsigned char chgr_getByte() {
	unsigned char ch = 0;
	dequeue(chgr_rxbuf, &chgr_rxrd, chgr_rxwr, CHGR_RX_BUFSZ, &ch);
	return ch;
}

unsigned int bmu_queueLength() {
	return queue_length(bmu_rxrd, bmu_txwr, BMU_RX_BUFSZ);
}

unsigned int chgr_queueLength() {
	return queue_length(chgr_rxrd, chgr_txwr, CHGR_RX_BUFSZ);
}

