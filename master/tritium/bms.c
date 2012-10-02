#include <string.h>			// For strlen() etc
#include <msp430x24x.h>		// For UC1IE etc

#include "tri86.h"
#include "bms.h"
#include "charger.h"
#include "queue.h"
#include "can.h"			// For can_transmit(), but also general definitions

// Private function prototypes
bool bmu_sendByte(unsigned char ch);
bool bmu_sendPacket(const unsigned char* ptr);

// Public variables
volatile unsigned int bmu_events = 0;
		 unsigned int bmu_state = 0;
volatile unsigned int bmu_sent_timeout;

// BMU buffers
queue bmu_tx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      BMU_TX_BUFSZ,
	.buf = { [0 ... BMU_TX_BUFSZ-1] = 0 }
};

queue bmu_rx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      BMU_RX_BUFSZ,
	.buf = { [0 ... BMU_RX_BUFSZ-1] = 0 }
};

// BMU private variables
volatile unsigned int  bmu_min_mV = 9999;	// The minimum cell voltage in mV
volatile unsigned int  bmu_max_mV = 0;	// The maximum cell voltage in mV
volatile unsigned int  bmu_min_id = 0;	// Id of the cell with minimum voltage
volatile unsigned int  bmu_max_id = 0;	// Id of the cell with maximum voltage
// Current cell in the current end-of-charge test (send voltage request to this cell next)
unsigned int bmu_curr_cell = 1;			// ID of BMU to send to next
//signed	 int	first_bmu_in_bypass = -1; // Charger end-of-charge test

// Stress table with check bits
static int stressTable[8] = {
			0x80 + (2<<3) + 0,		// $90 is lowest stress level, level 0
			0x80 + (3<<3) + 1,		// Stress 1
			0x80 + (3<<3) + 2,
			0x80 + (2<<3) + 3,
			0x80 + (1<<3) + 4,
			0x80 + (0<<3) + 5,
			0x80 + (0<<3) + 6,
			0x80 + (1<<3) + 7		// Stress 7
};


void bms_init()
{
	bmu_lastrxidx = 0;
#if USE_CKSUM	
	// Turn on checksumming in BMUs.
	// The "k" packet is ignored if BMU checksumming is on (bad checksum), but toggles BMU
	// checksumming on if it was off.
	bmu_sendByte('k'); bmu_sendByte('\r');		// NOTE: can't use bmu_SendPacket as it would insert a
												// checksum giving "kk\r" and having opposite effect
#else
	// Turn off checksumming in BMUs.
	// The "kk" packet toggles BMU checksumming off if it was on (single k command with good checksum),
	// but toggles BMU checksumming twice if it was off, thereby leaving it off.
	bmu_sendPacket((unsigned char*)"kk\r");		// DCU checksumming is off, so it won't change the pkt
#endif
	bmu_sendPacket((unsigned char*)"0K\r");	// Turn on (turn off Killing of) BMU badness sending
	bmu_sendVoltReq();						// Send the first voltage request packet;driving or charging
}

bool bmu_sendByte(unsigned char ch) {
	if (enqueue(&bmu_tx_q, ch)) {
    	UC1IE |= UCA1TXIE;                        		// Enable USCI_A1 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}

// Make a voltage request command at *cmd for cell number cellNo
// Called by bmu_sendVoltReq below
void makeVoltCmd(unsigned char* cmd, int cellNo)
{
	unsigned char* p; p = cmd;
	int n = cellNo;
	int h = n / 100;
	if (h) {						// If any hundreds
		*p++ = h + '0';					// then emit hundreds digit
		n -= h * 100;
	}
	int t = n / 10;
	if (h || t) {					// If hundreds or tens
		*p++ = t + '0';					// then emit tens digit
		n -= t * 10;
	}
	*p++ = n + '0';					// Emit units digit
	*p++ = 's'; *p++ = 'v'; *p++ = '\r'; *p++ = '\0';	// Emit s (select) and v (voltage) cmnds
}														// and terminate with return and null

// Send a request for a voltage reading, to a specific BMU
// Returns true on success
bool bmu_sendVoltReq()
{
	unsigned char cmd[8];
	makeVoltCmd(cmd, bmu_curr_cell);	// cmd := "XXsv\r"
	return bmu_sendPacket(cmd);
}

// Send min and max cell voltage and ids on CAN bus so telemetry software on PC can display them
void can_sendCellMaxMin(unsigned int bmu_min_mV, unsigned int bmu_max_mV,
								unsigned int bmu_min_id, unsigned int bmu_max_id)
{
	// We have the min and max cell voltage information. Send a CAN packet so the telemetry
	//	software can display them. Use CAN id 0x266, as the IQcell BMS would
	can.identifier = 0x266;
	can.data.data_u16[0] = bmu_min_mV;
	can.data.data_u16[1] = bmu_max_mV;
	can.data.data_u16[2] = bmu_min_id;
	can.data.data_u16[3] = bmu_max_id;
	can_transmit();
}

// Only used for debugging.
// Send total battery voltage as a comment packet on BMU channel
bool bmu_sendVAComment(int nVolt, int nAmp)
{
	// Packet to announce the charger's meas of total voltage and current
	// \ C H G _ n n n V _ n . n A \r
	// 0 1 2 3 4 5 6 7 8 9 a b c d e
	static unsigned char szChgrVolt[16] = "\\CHG nnnV n.nA\r";
	szChgrVolt[5] = nVolt / 1000 + '0';				// Voltage hundreds
	szChgrVolt[6] = (nVolt % 1000) / 100 + '0';		//	tens
	szChgrVolt[7] = (nVolt % 100) / 10 + '0';		//	units
	szChgrVolt[10] = (nAmp / 10) + '0';	// Current units
	szChgrVolt[12] = (nAmp % 10) + '0';	//	tenths
	return bmu_sendPacket(szChgrVolt); // Send as comment packet on BMU channel for debugging
}

void handleBMUstatusByte(unsigned char status, bool bCharging)
{
	int output;
	int stress = status & 0x07;			// Isolate stress bits
	int encoded = status & 0x9F;		// All but bypass and comms error bits
	bool bValid;
	
	
	// Check for validity
	bValid = stressTable[stress] == encoded;

	if (bCharging) {
		if (chgr_state & CHGR_END_CHARGE)
			return;
		if (bValid) {
			output = ctl_tick(&hCtlCharge, stress);
			if (status & 0x20 && (chgr_lastCurrent < CHGR_CUT_CURR)) {	// Bit 5 is all in bypass
				if (++chgr_bypCount >= CHGR_EOC_SOAKT) {
					// Terminate charging
					chgr_off();
					// FIXME: What status?
				}
				else if (chgr_bypCount != 0)			// Care! chgr_bypCount is unsigned
					--chgr_bypCount;					// Saturate at zero
			}
		}
		else
			// We have to insert a dummy measurement tick so the integral term still integrates
			// Use the last known good measurement, = set_point + prev_error
			output = ctl_tick(&hCtlCharge, hCtlCharge.set_point + hCtlCharge.prev_error);
		// Apply the output. We will have to tune the "gain" later. For now, let's try one whole
		// stress level = almost full current, so an output of 16 should represent a current of 55 (5.5 A)
		// 55/16 ~= 4
		chgr_sendRequest(CHGR_VOLT_LIMIT, output << 2, false);
	} else {
		// Not charging, assume driving.
		// TO BE COMPLETED
	}
}

// Read incoming bytes from BMUs
void readBMUbytes(bool bCharging)
{	unsigned char ch;
	while (	dequeue(&bmu_rx_q, &ch)) {		// Get a byte from the BMU receive queue
		if (ch >= 0x80) {
			handleBMUstatusByte(ch, bCharging);
		} else {
			if (bmu_lastrxidx >= BMU_RX_BUFSZ) {
				fault();
				break;
			}
			bmu_lastrx[bmu_lastrxidx++] = ch; // !!! Need to check for buffer overflow
			if (ch == '\r')	{				// All BMU responses end with a carriage return
				bmu_processPacket(bCharging);
				break;
			}
		}
	}
}
	
unsigned char bmu_lastSentPacket[BMU_TX_BUFSZ];		// Copy of the last packet sent to the BMUs

// Returns true on success
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
	strcpy((char*)bmu_lastSentPacket, (char*)ptr);	// Copy the buffer in case we have to resend
#endif
	return bmu_resendLastPacket();					// Call the main transmit function
}

// bmu_resendLastPacket is used for resending after a timeout, but also used for sending the first time.
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
	bmu_state |= BMU_SENT;							// Flag that packet is sent but not yet ack'd
	bmu_sent_timeout = BMU_TIMEOUT;					// Initialise timeout counter
	return true;
}


void bmu_processPacket(bool bCharging) {
	bmu_lastrxidx = 0;								// Ready for next BMU response to overwrite this one
													//	(starting next timer interrupt)
	if (chgr_state & CHGR_END_CHARGE)
		return;										// Ignore when charging completed

#if USE_CKSUM
	{	unsigned char sum = 0, j = 0;
		while (bmu_lastrx[j] != '\r')
			sum ^= bmu_lastrx[j++];
		if (sum != 0) {
			// Checksum error; set the error LED and resend the last command
			fault();
	//		bmu_resendLastPacket();					// Resend
			return;									// Don't process this packet
		}
	}
#endif

	// Check for a voltage response
	// Expecting \123:1234 V  ret
	//           0   45    10 11  (note space before the 'V'
	if (bmu_lastrx[0] == '\\' && bmu_lastrx[4] == ':' && bmu_lastrx[10] == 'V') {
		int bmu_id = 100 * (bmu_lastrx[1] - '0') + (bmu_lastrx[2] - '0') * 10 +
			bmu_lastrx[3] - '0';
		if (bmu_id == bmu_curr_cell) {
			bmu_state &= ~BMU_SENT;				// Call this valid and no longer unacknowledged
			unsigned int rxvolts =
#if 0
				(bmu_lastrx[5] - '0') * 100 +
#else
				// The *50 and << 1 below are to work around a mspgcc bug! See
				// http://sourceforge.net/tracker/index.php?func=detail&aid=2082985&group_id=42303&atid=432701
				(((bmu_lastrx[5] - '0') * 50) << 1) +
#endif
				(bmu_lastrx[6] - '0') * 10 +
				(bmu_lastrx[7] - '0');
			// We expect voltage responses during driving only now
			// We use the voltage measurements to find the min and max cell voltages
			// Get the whole 4-digit number
			rxvolts *= 10; rxvolts += bmu_lastrx[8] - '0';
			if (rxvolts < bmu_min_mV) {
				bmu_min_mV = rxvolts;
				bmu_min_id = bmu_id;
			}
			if (rxvolts > bmu_max_mV) {
				bmu_max_mV = rxvolts;
				bmu_max_id = bmu_id;
			}
			if (bmu_id >= NUMBER_OF_BMUS) {
				// We have the min and max information. Send a CAN packet so the telemetry
				//	software can display them.
				can_sendCellMaxMin(bmu_min_mV, bmu_max_mV, bmu_min_id, bmu_max_id);

				// Reset the min/max data
				bmu_min_mV = 9999;	bmu_max_mV = 0;
				bmu_min_id = 0;		bmu_max_id = 0;
			}
			// Move to the next BMU, only if packet valid
			if (++bmu_curr_cell > NUMBER_OF_BMUS)
				bmu_curr_cell = 1;
			bmu_sendVoltReq();								// Send another voltage request
		} // End if (bCharging)
	} // End if valid voltage response		
}


void bmu_timer() {							// Called every timer tick, for BMU related processing
	if (bmu_state & BMU_SENT) {
		if (--bmu_sent_timeout == 0) {
			fault();
			bmu_resendLastPacket();			// Resend; will loop until a complete packet is recvd
		}
	}
}

