#include <string.h>			// For memcpy() etc
#include <msp430x24x.h>		// For IE2 etc

#include "charger.h"
#include "bms.h"
#include "tri86.h"			// For fault() etc
#include "assert2.h"		// An assert-like function

// Private function prototypes
bool chgr_sendByte(unsigned char ch);
bool chgr_sendPacket(const unsigned char* ptr);
void chgr_processPacket();

// Public variables
volatile unsigned int chgr_events = 0;
		 unsigned int chgr_state = 0;
		 int chgr_rx_timeout = CHGR_TIMEOUT;// MVE: counts to zero; reset when see any output from the charger
unsigned int charger_volt = 0;			// MVE: charger voltage in tenths of a volt
unsigned int charger_curr = 0;			// MVE: charger current in tenths of an ampere
unsigned char charger_status = 0;		// MVE: charger status (e.g. bit 1 on = overtemp)
//unsigned int chgr_soakCnt = 0;		// Counter for soak phase
unsigned int chgr_bypCount = 0;			// Count of BMU ticks where all in bypass and current low
pid pidCharge(							// State for the control algorithm for charge current
//		(int)((3.5/8.0) * 4096),	// Set point will be 3.5 out of 8.0, left shifted by 12 bits
		(int)(1.0*256),				// Kp as 8.8
		(int)(0.5*256),				// Ki
		(int)(0.1*256),				// Kd
		0);							// Initial "measure"

// Charger buffers
chgr_queue chgr_tx_q(CHGR_TX_BUFSZ);
chgr_queue chgr_rx_q(CHGR_RX_BUFSZ);

// Charger private variables
unsigned char	chgr_lastrx[12];		// Buffer for the last received charger message
unsigned char	chgr_lastrxidx;			// Index into the above
unsigned char chgr_txbuf[12];			// A buffer for a charger packet
unsigned int chgr_lastCurrent = 9999;	// Last commanded current sent to the charger

unsigned char chgr_lastSentPacket[12];					// Copy of the last CAN packet sent to the charger

bool chgr_sendPacket(const unsigned char* ptr)
{
	memcpy(chgr_lastSentPacket, ptr, 12);				// Copy the data to the last message buffer
	return chgr_resendLastPacket();						// Call the main transmit function
}

chgr_queue::chgr_queue(unsigned char sz) : queue(sz) {
	assert2(sz <= CHGR_RX_BUFSZ, "chgr_queue buffer size");
};

void chgr_init() {
	chgr_lastrxidx = 0;
}

void chgr_start() {
	charger_count = CHARGER_SPEED;
	chgr_lastCurrent = 9999;		// So first call to chgr_setCurrent will actually send
	chgr_bypCount = 0;
}


// chgr_resendLastPacket could be used for resending, but also used for sending the first time.
// Returns true on success
bool chgr_resendLastPacket(void)
{
	int i;
	if (chgr_tx_q.queue_space() < 12) {
		// Need 12 bytes of space in the queue
		// If not, best to abort the whole command, rather than sending a partial message
		fault();
		return false;
	}
	for (i=0; i < 12; ++i)
		chgr_sendByte(chgr_lastSentPacket[i]);
	return true;
}


bool chgr_sendByte(unsigned char ch) {
	if (chgr_tx_q.enqueue(ch)) {
    	IE2 |= UCA0TXIE;                        		// Enable USCI_A0 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}

// Read incoming bytes from charger
void readChargerBytes()
{	unsigned char ch;
	while (	chgr_rx_q.dequeue(ch)) {
		chgr_lastrx[chgr_lastrxidx++] = ch;
		if (chgr_lastrxidx == 12)	{		// All charger messages are 12 bytes long
			chgr_processPacket();			// We've received a charger response
			chgr_rx_timeout = CHGR_TIMEOUT;	// Reset received anything counter
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
			
	if (--chgr_rx_timeout <= 0) {
		fault();						// Turn on fault LED (eventually)
	}
}

// Set the current, but don't send again if it's the same as last time. These chargers don't like too
// much traffic; more than one per second has been said to crash them.
void chgr_setCurrent(unsigned int iCurr) {
	if (iCurr == chgr_lastCurrent)
		return;							// Do nothing; we have a timeout in case nothing is sent
										// for ~ 5 seconds
	chgr_sendCurrent(iCurr);			// If it's changed, send it now
}

// Here when we haven't sent anything to the charger for ~ 5 seconds
void handleChargerEvent() {
	chgr_sendCurrent(chgr_lastCurrent);	// Resend the last charger current, so the
										//	charger doesn't stop due to a comms error
}

// Send the current command now
void chgr_sendCurrent(unsigned int iCurr) {
	chgr_sendRequest(CHGR_VOLT_LIMIT, iCurr, 0);
	charger_count = CHARGER_SPEED;			// Reset the charger timer
}

void chgr_off() {
	chgr_sendRequest(0, 0, true);			// Zero volts, zero amps, and turn charger off
	chgr_state = CHGR_END_CHARGE;			// Reset all other flags, set end charge flag
}
