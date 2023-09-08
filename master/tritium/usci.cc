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
 * - Implements the following SPI interface functions (for talking to/from CAN chip and hence motor controller)
 *	- init
 *	- transmit
 *	- exchange
 *
 *  - 4 interrupt routines (for talking to/from BMS and chargers)
 *
 *  Higher level UART functions are in charger.c and bms.c now
 */

// Include files
#include <msp430x24x.h>
//#include <intrinsics.h>                               // No longer needed for dint() and eint()
#include <string.h>					// For memcpy()

#include "tri86.h"					// For CAN_MOSI etc
#include "usci.h"
#include "bms.h"					// For the BMS and
#include "charger.h"				//	charger queues
#include "queue.h"


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
	UCB0CTL1 &= (uchar)~UCSWRST;						// Initialize USCI state machine
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
//#pragma vector=USCIAB0TX_VECTOR
__interrupt void __attribute__((interrupt(USCIAB0TX_VECTOR))) usciab0tx(void)
{
	if (IFG2 & UCA0TXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		unsigned char ch = 0;					// Get byte from the
		chgr_tx_q.dequeue(ch);					//	transmit queue
		if (chgr_tx_q.empty())					// Queue empty and therefore TX complete?
			IE2 &= (uchar)~UCA0TXIE;			// Yes, disable USCI_A0 TX interrupt
		UCA0TXBUF = ch;							// TX this byte
		events |= EVENT_ACTIVITY;				// Turn on activity light
	}
}

// For use with A1/B1, need separate ISR, using UC1IFG instead of IFG2, and UC1IE for IE2.
//#pragma vector=USCIAB1TX_VECTOR
__interrupt void __attribute__((interrupt(USCIAB1TX_VECTOR))) usciab1tx(void)
{
	if (UC1IFG & UCA1TXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		unsigned char ch = 0;					// Get byte from the
		bms_tx_q.dequeue(ch);					//	transmit queue
		if (bms_tx_q.empty())					// Queue empty and therefore TX complete?
			UC1IE &= (uchar)~UCA1TXIE;			// Yes, disable USCI_A1 TX interrupt
		UCA1TXBUF = ch;							// Transmit this byte
		events |= EVENT_ACTIVITY;				// Turn on activity light
	}
}

// USCI0 A0/B0 Receive ISR
//#pragma vector=USCIAB0RX_VECTOR
__interrupt void __attribute__((interrupt(USCIAB0RX_VECTOR))) usciab0rx(void)
{
	if (IFG2 & UCA0RXIFG)						// Make sure it's UCA0 causing the interrupt
	{
		if (!chgr_rx_q.enqueue(UCA0RXBUF))
			fault();							// Fault if queue is full
		else
			events |= EVENT_ACTIVITY;			// Turn on activity light
	}
}

// USCI1 A0/B0 Receive ISR
//#pragma vector=USCIAB1RX_VECTOR
__interrupt void __attribute__((interrupt(USCIAB1RX_VECTOR))) usciab1rx(void)
{
	if (UC1IFG & UCA1RXIFG)						// Make sure it's UCA1 causing the interrupt
	{
		if (!bms_rx_q.enqueue(UCA1RXBUF))
			fault();							// Fault if queue is full
		else
			events |= EVENT_ACTIVITY;			// Turn on activity light
	}
}

