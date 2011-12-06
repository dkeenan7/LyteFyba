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
			volatile queue* q,			// Pointer to the queue structure
			unsigned char ch )			// The byte to enqueue
{
	unsigned char wr_copy = q->wr;		// Make a copy of the write index
	q->buf[wr_copy++] = ch;				// Tentatively write the byte to the queue; there is always
										//	one free space, but don't update write index yet
										// Also increments the index copy
	wr_copy &= (q->bufSize-1);			//	modulo the buffer size
	if (wr_copy == q->rd)				// Does the incremented write pointer equal the read pointer?
		return false;					// Yes means queue is full, error return
	q->wr = wr_copy;					// Update write pointer; byte is officially in the queue now
	return true;						// Normal return
}

// Dequeue a byte. Returns true on success (queue was not empty).
static bool dequeue(
			volatile queue* q,			// Pointer to the queue structure
			unsigned char* ch )			// Pointer to the char to be read to
{
	unsigned char rd_copy = q->rd;		// Make a copy of the read index
	if (q->wr == rd_copy)				// Indexes equal?
		return false;					// If so, buffer is empty
	*ch = q->buf[rd_copy++];			// Read the byte, increment read index
	rd_copy &= (q->bufSize-1);			//	modulo the buffer size
	q->rd = rd_copy;					// Atomic update
	return true;
}

// Amouunt of space in the queue. This is the capacity of the queue minus the number already in the queue.
// The capacity is actually bufSize-1, so space = (bufSize-1 - (wr - rd)) & bufSize-1, which is the same
// as (rd - wr - 1) & (bufSize-1)
static unsigned int queue_space( queue* q )	// Pointer to queue structure
{
	return (q->rd - q->wr - 1) & (q->bufSize-1);
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
		unsigned char ch = 0;					// Get byte from the transmit queue
		dequeue(&chgr_tx_q, &ch);
		if (chgr_tx_q.rd == chgr_tx_q.wr)		// Queue empty and therefore TX complete?
			IE2 &= ~UCA0TXIE;					// Disable USCI_A0 TX interrupt
		UCA0TXBUF = ch;							// TX this byte
		events |= EVENT_ACTIVITY;				// Turn on activity light
	}
}

// For use with A1/B1, need separate ISR, using UC1IFG instead of IFG2, and UC1IE for IE2.
// #pragma vector=USCIAB1TX_VECTOR
interrupt(USCIAB1TX_VECTOR) usciab1tx(void)
{
	if (UC1IFG & UCA1TXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		unsigned char ch = 0;					// Get byte from the transmit queue
		dequeue(&bmu_tx_q, &ch);
		if (bmu_tx_q.rd == bmu_tx_q.wr)			// Queue empty and therefore TX complete?
			UC1IE &= ~UCA1TXIE;					// Disable USCI_A1 TX interrupt
		UCA1TXBUF = ch;							// Transmit this byte
		events |= EVENT_ACTIVITY;				// Turn on activity light
	}
}

// USCI0 A0/B0 Receive ISR
//  #pragma vector=USCIAB0RX_VECTOR
interrupt(USCIAB0RX_VECTOR) usciab0rx(void)
{
	if (IFG2 & UCA0RXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		if (!enqueue(&chgr_rx_q, UCA0RXBUF))
			fault();							// Fault if queue is full
		else
			events |= EVENT_ACTIVITY;			// Turn on activity light
	}
}

// USCI1 A0/B0 Receive ISR
//  #pragma vector=USCIAB1RX_VECTOR
interrupt(USCIAB1RX_VECTOR) usciab1rx(void)
{
	if (UC1IFG & UCA1RXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		if (!enqueue(&bmu_rx_q, UCA1RXBUF))
			fault();							// Fault if queue is full
		else
			events |= EVENT_ACTIVITY;			// Turn on activity light
	}
}

unsigned char chgr_lastSentPacket[12];					// Copy of the last CAN packet sent to the charger

bool chgr_sendPacket(const unsigned char* ptr)
{
	memcpy(chgr_lastSentPacket, ptr, 12);				// Copy the data to the last message buffer
	return chgr_resendLastPacket();						// Call the main transmit function
}

// chgr_resendLastPacket is used for resending after a timeout.
// Returns true on success
bool chgr_resendLastPacket(void)
{
	int i;
	if (queue_space(&chgr_tx_q) < 12) {
		// Need 12 bytes of space in the queue
		// If not, best to abort the whole command, rather than sending a partial message
		fault();
		return false;
	}
	for (i=0; i < 12; ++i)
		chgr_sendByte(chgr_lastSentPacket[i]);
	chgr_events |= CHGR_SENT;						// Flag that packet is sent but not yet ack'd
	chgr_sent_timeout = CHGR_TIMEOUT;				// Initialise timeout counter
	return true;
}

unsigned char bmu_lastSentPacket[BMU_TX_BUFSZ];		// Copy of the last packet sent to the BMUs

bool bmu_sendPacket(const unsigned char* ptr)
{
#if USE_CKSUM
	unsigned char ch, i = 0, sum = 0;
	do {
		ch = *ptr++;
		sum ^= ch;									// Calculate XOR checksum
		bmu_lastSentPacket[i++] = ch;				// Copy the data to the transmit buffer
	} while (ch != '\r');
	sum ^= '\r';									// CR is not part of the checksum
	if (sum < ' ') {
		// If the checksum would be a control character that could be confused with a CR, BS,
		//	etc, then send a space, which will change the checksum to a non-control character
		bmu_lastSentPacket[i++-1] = ' ';			// Replace CR with space
		sum ^= ' ';									// Update checksum
	}
	bmu_lastSentPacket[i++-1] = sum;				// Insert the checksum
	bmu_lastSentPacket[i-1] = '\r';					// Add CR
	bmu_lastSentPacket[i] = '\0';					// Null terminate; bmu_resendLastPacket expects this
#else
	strcpy(bmu_lastSentPacket, ptr);				// Copy the buffer in case we have to resend
#endif
	return bmu_resendLastPacket();					// Call the main transmit function
}

// bmu_resendLastPacket is used for resending after a timeout.
// Returns true on success
bool bmu_resendLastPacket(void)
{
	int i, len = strlen((char*)bmu_lastSentPacket);
	if ((int)queue_space(&bmu_tx_q) < len) {
		fault();
		return false;
	}
	for (i=0; i < len; ++i)							// Send the bytes of the packet
		bmu_sendByte(bmu_lastSentPacket[i]);
	bmu_events |= BMU_SENT;							// Flag that packet is sent but not yet ack'd
	bmu_sent_timeout = BMU_TIMEOUT;					// Initialise timeout counter
	return true;
}

// For tri86.c to call:
bool bmu_getByte(unsigned char* chp) {
	return dequeue(&bmu_rx_q, chp);
}

bool bmu_sendByte(unsigned char ch) {
	if (enqueue(&bmu_tx_q, ch)) {
    	UC1IE |= UCA1TXIE;                        		// Enable USCI_A1 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}

bool chgr_getByte(unsigned char* chp) {
	return dequeue(&chgr_rx_q, chp);
}

bool chgr_sendByte(unsigned char ch) {
	if (enqueue(&chgr_tx_q, ch)) {
    	IE2 |= UCA0TXIE;                        		// Enable USCI_A0 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}
