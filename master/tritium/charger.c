#include <string.h>			// For memcpy() etc
#include <msp430x24x.h>		// For IE2 etc

#include "charger.h"
#include "bms.h"
#include "tri86.h"			// For fault() etc

// Private function prototypes
bool chgr_sendByte(unsigned char ch);
bool chgr_sendPacket(const unsigned char* ptr);
void chgr_processPacket();

// Public variables
volatile unsigned int chgr_events = 0;
		 unsigned int chgr_state = 0;
volatile unsigned int chgr_sent_timeout;
unsigned int charger_volt = 0;			// MVE: charger voltage in tenths of a volt
unsigned int charger_curr = 0;			// MVE: charger current in tenths of an ampere
unsigned char charger_status = 0;		// MVE: charger status (e.g. bit 1 on = overtemp)
unsigned int chgr_current = BMU_BYPASS_CAP - CHGR_CURR_DELTA;	// Charger present current; initially equal
							// to bypass capability (incremented before first use)
unsigned int chgr_report_volt = 0;		// Charger reported voltage in tenths of a volt
unsigned int chgr_soakCnt = 0;			// Counter for soak phase

// Charger buffers
queue chgr_tx_q = {						// Initialise structure members and size of
	.rd = 0,
	.wr = 0,
	.bufSize =      CHGR_TX_BUFSZ,
	.buf = { [0 ... CHGR_TX_BUFSZ-1] = 0 }
};

queue chgr_rx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      CHGR_RX_BUFSZ,
	.buf = { [0 ... CHGR_RX_BUFSZ-1] = 0 }
};

// Charger private variables
unsigned char	chgr_lastrx[12];		// Buffer for the last received charger message
unsigned char	chgr_lastrxidx;			// Index into the above
unsigned char chgr_txbuf[12];			// A buffer for a charger packet

unsigned char chgr_lastSentPacket[12];					// Copy of the last CAN packet sent to the charger

bool chgr_sendPacket(const unsigned char* ptr)
{
	memcpy(chgr_lastSentPacket, ptr, 12);				// Copy the data to the last message buffer
	return chgr_resendLastPacket();						// Call the main transmit function
}

void chgr_init() {
	chgr_lastrxidx = 0;
}

// chgr_resendLastPacket is used for resending after a timeout, but also used for sending the first time.
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
	chgr_state |= CHGR_SENT;						// Flag that packet is sent but not yet ack'd
	chgr_sent_timeout = CHGR_TIMEOUT;				// Initialise timeout counter
	return true;
}


bool chgr_sendByte(unsigned char ch) {
	if (enqueue(&chgr_tx_q, ch)) {
    	IE2 |= UCA0TXIE;                        		// Enable USCI_A0 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}

// Read incoming bytes from charger
void readChargerBytes()
{	unsigned char ch;
	while (	dequeue(&chgr_rx_q, &ch)) {
		chgr_lastrx[chgr_lastrxidx++] = ch;
		if (chgr_lastrxidx == 12)	{	// All charger messages are 12 bytes long
			chgr_events |= CHGR_REC;	// We've received a charger response
			chgr_events &= ~CHGR_SENT;	// No longer unacknowledged
			break;
		}
	}
}

void chgr_sendRequest(int voltage, int current, bool chargerOff) {
	// Charger is on the UART in UCI0
	chgr_txbuf[0] = 0x18;					// Send 18 06 E5 F4 0V VV 00 WW 0X 00 00 00
	chgr_txbuf[1] = 0x06;					//	where VVV is the voltage in tenths of a volt,
	chgr_txbuf[2] = 0xE5;					//	WW is current limit in tenths of an amp, and
	chgr_txbuf[3] = 0xF4;					//	X is 0 to turn charger on
	chgr_txbuf[4] = voltage >> 8;
	chgr_txbuf[5] = voltage & 0xFF;
	chgr_txbuf[6] = 0;
	chgr_txbuf[7] = current;		// Soak at the soak current level (< bypass capacity)
	chgr_txbuf[8] = chargerOff;
	chgr_txbuf[9] = 0; chgr_txbuf[10] = 0; chgr_txbuf[11] = 0;
	chgr_sendPacket(chgr_txbuf);
	chgr_lastrxidx = 0;						// Expect receive packet in response
}

void chgr_processPacket() {

	chgr_lastrxidx = 0;						// Ready for next charger response to overwrite this one
													//	(starting next timer interrupt)
	bmu_sendVAComment((chgr_lastrx[4] << 8) + chgr_lastrx[5], chgr_lastrx[7]); // For debugging
}


void chgr_timer() {							// Called every timer tick, for charger related processing
	if (chgr_events & CHGR_SENT) {
		if (--chgr_sent_timeout == 0) {
			fault();				// Turn on fault LED (eventually)
			chgr_events |= CHGR_RESEND;	// Tell the main loop to resend
		}
	}
}

void handleChargerEvent() {
	int current, chargerOff;
	events |= EVENT_ACTIVITY;			// Not strictly CAN bus, but we'll call this activity

	if (chgr_state & CHGR_END_CHARGE) {
		current = 0;					// Request no current now
		chargerOff = 1;					// Turn off charger
	} else if (chgr_state & CHGR_SOAKING) {
		current = CHGR_SOAK_CURR;		// Soak at the soak current level (< bypass capacity)
		chargerOff = 0;					// Keep charger on
	} else {
		chgr_current += CHGR_CURR_DELTA;	// Increase charger current by the fixed amount
		if (chgr_current > CHGR_CURR_LIMIT)
			chgr_current = CHGR_CURR_LIMIT;
		current = chgr_current;
		chargerOff = 0;
	}
	chgr_sendRequest(CHGR_VOLT_LIMIT, current, chargerOff);
}

