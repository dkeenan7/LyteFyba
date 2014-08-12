#include <string.h>			// For strlen() etc
#include <msp430x24x.h>		// For TACCTL1 and CCIE

#include "tri86.h"
#include "voice.h"
#include "queue.h"

// Voice synth buffers
queue voice_tx_q(VOICE_TX_BUFSZ);
queue voice_rx_q(VOICE_RX_BUFSZ);

// Voice synth private variables

// Static globals


void voice_init()
{
}

bool voice_sendByte(unsigned char ch) {
	int TARplusSetup;
#define MckPerTAck 8		// Main clock cycles per Timer A clock cycle
	if (voice_tx_q.enqueue(ch)) {
		// If transmit interrupts are disabled, set up the timer for a new start bit
		// and enable transmit compare interrupts.
		if (!(TACCTL1 & CCIE)) {		// If transmit interrupts disabled
			// Ensure a fixed delay between reading timer and setting
			// compare time and output mode for a start bit.
			__dint();						// Disable all interrupts (includes a nop)
			// We want the max number of timer clock cycles till both TAR and TACCTLt are set up below.
			// First number is sum of execution cycles in parentheses below for statements.
			// The +MckPerTAck-1 is for rounding up.
			// The +1 is in case the TAR increments just after we read it.
			// Add setup time to timer value.
			TARplusSetup = (signed)TAR + ((28+MckPerTAck-1)/MckPerTAck) + 1; // (3+2)
			// If previous stop bit has not completed AND existing TACCCR1 is AFTER TAR+setup then
			//  leave TACCR1 unchanged so the start bit begins immediately the stop bit ends.
			// If previous stop bit has completed OR existing TACCR1 is ON_OR_BEFORE TAR+setup
			if ((TACCTL1 & CCIFG) || (TARplusSetup - (signed)TACCR1 >= 0)) // (5+2+4+2)
				TACCR1 = (unsigned)TARplusSetup;// (4) Set the new compare value for the start-bit
			TACCTL1 = OUTMOD_5+CCIE;		// (5) Set output mode for start bit and enable compare ints
			__eint(); 						// (1) Enable interrupts generally
		} // End if transmit interrupts disabled
		return true;
	}
	return false;
} // End voice_sendByte()


// Read incoming bytes from voice synth
void voice_readBytes()
{	unsigned char ch;
	while (	voice_rx_q.dequeue(ch)) {	// Get a byte from the voice synth receive queue
		// FIXME! Do whatever needs doing with it
	} // End while
}


void voice_sendString(const char* messagePtr)
{
	unsigned int i, len;
	len = strlen(messagePtr);
	for (i=0; i < len; ++i)							// Send the bytes of the message
		voice_sendByte((uchar)messagePtr[i]);
}
