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
 */

// Include files
#include <msp430x24x.h>
#include "tri86.h"
#include "usci.h"

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
	if( clock == 0 ) UCB0CTL1 = UCSSEL_2 | UCSWRST;	// BRCLK = SMCLK
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
