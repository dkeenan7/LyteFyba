#include <string.h>			// For memcpy() etc
#include <msp430x24x.h>		// For IE2 etc

#include "charger.h"
#include "bms.h"
#include "tri86.h"			// For fault() etc
#include "can.h"			// For can_push_ptr
#include "gauge.h"			// For gauge_tach_update()

// Private function prototypes
bool chgr_sendByte(unsigned char ch);
bool chgr_sendPacket(const unsigned char* ptr);
void chgr_processPacket();
void SendChgrCurr(unsigned int uChgrCurr);

// Public variables
volatile unsigned int chgr_events = 0;
		 unsigned int chgr_state = CHGR_IDLE;
		 int chgr_rx_timer = 0;			// MVE: counts to zero; restart when receive anything from charger
		 int chgr_tx_timer = 0;			// DCK: counts to zero; restart when transmit anything to charger
unsigned int charger_volt = 0;			// MVE: charger voltage in tenths of a volt
unsigned int charger_curr = 0;			// MVE: charger current in tenths of an ampere
unsigned char charger_status = 0;		// MVE: charger status (e.g. bit 1 on = overtemp)
//unsigned int chgr_soakCnt = 0;		// Counter for soak phase
unsigned int chgr_bypCount = 0;			// Count of BMS ticks where all in bypass and current low
unsigned int uChgrCurrA=0, uChgrCurrB=0;// Charger A or B actual current

// Charger buffers
queue chgr_tx_q(CHGR_TX_BUFSZ);
queue chgr_rx_q(CHGR_RX_BUFSZ);

// Charger private variables
unsigned char	chgr_lastrx[12];		// Buffer for the last received charger message
unsigned char	chgr_lastrxidx;			// Index into the above
unsigned char	chgr_txbuf[12];			// A buffer for a charger packet
unsigned int chgr_dsp_ctr = 0;			// Charger current display on tacho - count of timer ticks


// Program global
extern unsigned int uCharging;

unsigned char chgr_lastSentPacket[12];	// Copy of the last serial packet sent to charger A

void chgr_init() {
	chgr_lastrxidx = 0;
}

void chgr_start() {
	chgr_state = CHGR_CHARGING;
	P1OUT |= CHG_CONT_OUT;		// Turn on the charger contactor.
								// External relays and diodes should ensure that we also
								// turn on the battery contactors.
	chgr_bypCount = 0;
}


void chgr_idle() {
	chgr_sendRequest(0, 0, false); // Zero volts, zero amps, but let it keep sending data
	chgr_state = CHGR_IDLE;
	P1OUT &= (uchar)~CHG_CONT_OUT;		// Turn off the charger contactor.
								// External relays and diodes should ensure that we also
								// turn off the battery contactors.
}


void chgr_stop() {
	chgr_sendRequest(0, 0, false); // Zero volts, zero amps, but let it keep sending data
	chgr_state = CHGR_IDLE;
	P1OUT &= (uchar)~CHG_CONT_OUT;		// Turn off the charger contactor.
								// External relays and diodes should ensure that we also
								// turn off the battery contactors.
}


void chgr_timer(unsigned int switches) {				// Called every 10 ms timer tick, for charger related processing
	if (chgr_tx_timer > 0) --chgr_tx_timer;	// Decrement without letting it wrap around
	if (chgr_rx_timer > 0) --chgr_rx_timer;	// Decrement without letting it wrap around
	if ((chgr_state != CHGR_IDLE) && (chgr_rx_timer == 0)) {
		fault();						// Turn on fault LED (eventually)
	}
	// If we're DCU-A and either we or DCU-B are in charge mode,
	// display the charger currents on the tacho alternately.
	// Mnemonic is alphabetical order A, B, ... corresponds to numbers 1, 2, ...
	if (!bDCUb && ((command.state == MODE_CHARGE) || (switches & SW_INH_TRACTION))) {
		if (++chgr_dsp_ctr == 300) {				// 3 second display cycle
			chgr_dsp_ctr = 0;
			gauge_tach_update(uChgrCurrB * 100);	// 2 seconds displaying charger B current
		}
		if (chgr_dsp_ctr == 200)
			gauge_tach_update(uChgrCurrA * 100);	// 1 second displaying charger A current
	}
}


// Send a command to set the current
bool chgr_sendCurrent(unsigned int iCurr) {
	return chgr_sendRequest(CHGR_VOLT_LIMIT, iCurr, false);
}


bool chgr_sendRequest(unsigned int voltage, unsigned int current, bool chargerOff) {
	bool ret;				// can_push does not return a success value
	if (bDCUb)				// For now, DCUB has the CAN-bus charger, DCU-A has serial
	{
		can_push_ptr->identifier = CHGR_ID_B_CTRL;
		can_push_ptr->status = 8;	// Packet size in bytes
		can_push_ptr->data.data_u8[0] = (uchar)(voltage >> 8);
		can_push_ptr->data.data_u8[1] = voltage & 0xFF;
		can_push_ptr->data.data_u8[2] = (uchar)(current >> 8);
		can_push_ptr->data.data_u8[3] = current & 0xFF;
		can_push_ptr->data.data_u8[4] = chargerOff;
		can_push_ptr->data.data_u8[5] = 0;
		can_push_ptr->data.data_u8[6] = 0;
		can_push_ptr->data.data_u8[7] = 0;
		can_push();
		ret = true;
	}
	else
	{
		// Charger is on the UART in UCI0 for DCU A
		chgr_txbuf[0] = 0x18;					// Send 18 06 E5 F4 0V VV 00 WW 0X 00 00 00
		chgr_txbuf[1] = 0x06;					//	where VVV is the voltage in tenths of a volt,
		chgr_txbuf[2] = 0xE5;					//	WW is current limit in tenths of an amp, and
		chgr_txbuf[3] = 0xF4;					//	X is 0 to turn charger on
		chgr_txbuf[4] = (uchar)(voltage >> 8);
		chgr_txbuf[5] = voltage & 0xFF;
		chgr_txbuf[6] = (uchar)(current >> 8);
		chgr_txbuf[7] = current & 0xFF;
		chgr_txbuf[8] = chargerOff;
		chgr_txbuf[9] = 0; chgr_txbuf[10] = 0; chgr_txbuf[11] = 0;
		ret = chgr_sendPacket(chgr_txbuf);
		chgr_lastrxidx = 0;						// Expect receive packet in response
	}
	return ret;						// FIXME: can we do better?
}


bool chgr_sendPacket(const unsigned char* ptr)
{
	memcpy(chgr_lastSentPacket, ptr, 12);				// Copy the data to the last message buffer
	return chgr_resendLastPacket();						// Call the main transmit function
}


// chgr_resendLastPacket could be used for resending, but also used for sending the first time.
// Returns true on success
bool chgr_resendLastPacket(void)
{
	if (bDCUb)
		return false;

	// Send serial data via fibre
	int i;
	if (chgr_tx_q.queue_space() < 12) {
		// Need 12 bytes of space in the queue
		// If not, best to abort the whole command, rather than sending a partial message
		fault();
		return false;
	}
	else {
		for (i=0; i < 12; ++i)
			chgr_sendByte(chgr_lastSentPacket[i]);
		return true;
	}
}


bool chgr_sendByte(unsigned char ch) {
	if (chgr_tx_q.enqueue(ch)) {
		IE2 |= UCA0TXIE;						// Enable USCI_A0 TX interrupt
		events |= EVENT_ACTIVITY;				// Turn on activity light
		chgr_tx_timer = CHGR_TX_TIMEOUT;		// Restart transmitted-anything timer
		return true;
	}
	else
		return false;
}


// Read incoming bytes from charger
void readChargerBytes()
{	unsigned char ch;
	while (	chgr_rx_q.dequeue(ch)) {
		chgr_rx_timer = CHGR_RX_TIMEOUT;	// Restart received-anything timer
		chgr_lastrx[chgr_lastrxidx++] = ch;
		if (chgr_lastrxidx == 12)	{		// All charger messages are 12 bytes long
			chgr_processSerPacket();		// We've received a charger response
			break;
		}
	}
}


void chgr_processSerPacket() {

	chgr_lastrxidx = 0;						// Ready for next charger response to overwrite this one
											//	(starting next timer interrupt)
	// bms_sendVAComment((chgr_lastrx[4] << 8) + chgr_lastrx[5], chgr_lastrx[7]); // For debugging
	if (chgr_lastrx[3] == 0xE5 &&
		chgr_lastrx[2] == 0x50 &&
		chgr_lastrx[1] == 0xFF &&
		chgr_lastrx[0] == 0x18)		// Might be other packet IDs
	{
		if (bDCUb)
			SendChgrCurr(chgr_lastrx[7]);
		else
			uChgrCurrA = chgr_lastrx[7];
	}
}

void chgr_processCanPacket() {
	chgr_rx_timer = CHGR_RX_TIMEOUT;		// Restart the received-anything timer
	if (bDCUb) {
		unsigned int current = (unsigned int)((can.data.data_u8[2]) << 8) + can.data.data_u8[3];
		SendChgrCurr(current);		// Tell DCU A about our charge current, for tacho
	}
}

void SendChgrCurr(unsigned int uChgrCurr) {
	can_push_ptr->identifier = DC_CAN_BASE + DC_CHGR_CURR;
	can_push_ptr->status = 2;	// Packet size in bytes
	can_push_ptr->data.data_u16[0] = uChgrCurr;		// Send charger actual current to DCU-A
	can_push();
}


