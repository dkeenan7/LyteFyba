#include <string.h>			// For strlen() etc
#include <msp430x24x.h>		// For UC1IE etc

#include "tri86.h"
//#include "usci.h"
//#include "can.h"
#include "bms.h"
#include "queue.h"
#include "can.h"			// For can_transmit(), but also general definitions

// Private function prototypes
bool bmu_sendByte(unsigned char ch);
bool bmu_sendPacket(const unsigned char* ptr);

bool chgr_sendByte(unsigned char ch);
bool chgr_sendPacket(const unsigned char* ptr);
void chgr_processPacket();

// Public variables
volatile unsigned int chgr_events = 0;
volatile unsigned int bmu_events = 0;
volatile unsigned char bmu_badness = 0;		// Zero says we have received no badness so far
volatile unsigned int chgr_sent_timeout;
volatile unsigned int bmu_sent_timeout;

unsigned int charger_volt = 0;			// MVE: charger voltage in tenths of a volt
unsigned int charger_curr = 0;			// MVE: charger current in tenths of an ampere
unsigned char charger_status = 0;		// MVE: charger status (e.g. bit 1 on = overtemp)
unsigned int chgr_current = BMU_BYPASS_CAP - CHGR_CURR_DELTA;	// Charger present current; initially equal
							// to bypass capability (incremented before first use)
unsigned int chgr_report_volt = 0;		// Charger reported voltage in tenths of a volt
unsigned int chgr_soaking = 0;			// Counter for soak phase

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

// BMU private variables
volatile unsigned int  bmu_min_mV = 9999;	// The minimum cell voltage in mV
volatile unsigned int  bmu_max_mV = 0;	// The maximum cell voltage in mV
volatile unsigned int  bmu_min_id = 0;	// Id of the cell with minimum voltage
volatile unsigned int  bmu_max_id = 0;	// Id of the cell with maximum voltage
// Current cell in the current end-of-charge test (send voltage request to this cell next)
unsigned int bmu_curr_cell = 1;			// ID of BMU to send to next


// Charger private variables
unsigned char	chgr_lastrx[12];		// Buffer for the last received charger message
unsigned char	chgr_lastrxidx;			// Index into the above
signed	 int	first_bmu_in_bypass = -1; // Charger end-of-charge test
unsigned char chgr_txbuf[12];			// A buffer for a charger packet

void bms_init()
{
	bmu_lastrxidx = 0;
	chgr_lastrxidx = 0;
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
	bmu_sendPacket("kk\r");						// DCU checksumming is off, so it won't change the pkt
#endif
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
	unsigned char szChgrVolt[16] = "\\CHG nnnV n.nA\r";
	szChgrVolt[5] = nVolt / 1000 + '0';				// Voltage hundreds
	szChgrVolt[6] = (nVolt % 1000) / 100 + '0';		//	tens
	szChgrVolt[7] = (nVolt % 100) / 10 + '0';		//	units
	szChgrVolt[10] = (nAmp / 10) + '0';	// Current units
	szChgrVolt[12] = (nAmp % 10) + '0';	//	tenths
	return bmu_sendPacket(szChgrVolt); // Send as comment packet on BMU channel for debugging
}

void handleBMUbadnessEvent()
{
	if (bmu_badness > 0x80)					// Simple algorithm:
		chgr_current = BMU_BYPASS_CAP - CHGR_CURR_DELTA;	// On any badness, cut back to
									                        // what the BMUs can bypass
#if 0   // ? Likely was when driven by badness bytes; now driven by events
	if ((chgr_events & (CHGR_SOAKING | CHGR_END_CHARGE)) == 0) {
		// Send a voltage check for the current cell
		bmu_sendVoltReq();
		if (++bmu_curr_cell > NUMBER_OF_BMUS)
			bmu_curr_cell = 1;
	}
#endif
}

// Read incoming bytes from BMUs
void readBMUbytes()
{	unsigned char ch;
	while (	dequeue(&bmu_rx_q, &ch)) {		// Get a byte from the BMU receive queue
		if (ch >= 0x80) {
			bmu_events |= BMU_BADNESS;
			bmu_badness = ch;
		} else {
			if (bmu_lastrxidx >= BMU_RX_BUFSZ) {
				fault();
				break;
			}
			bmu_lastrx[bmu_lastrxidx++] = ch; // !!! Need to check for buffer overflow
			if (ch == '\r')	{				// All BMU responses end with a carriage return
				bmu_events |= BMU_REC;		// We've received a BMU response
				break;
			}
		}
	}
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
	bmu_events |= BMU_SENT;							// Flag that packet is sent but not yet ack'd
	bmu_sent_timeout = BMU_TIMEOUT;					// Initialise timeout counter
	return true;
}


void bmu_processPacket(bool bCharging) {
	bmu_lastrxidx = 0;								// Ready for next BMU response to overwrite this one
													//	(starting next timer interrupt)

#if USE_CKSUM
	{	unsigned char sum = 0, j = 0;
		while (bmu_lastrx[j] != '\r')
			sum ^= bmu_lastrx[j++];
		if (sum != 0) {
			// Checksum error; set the error LED and resend the last command
			fault();
			bmu_resendLastPacket();					// Resend
			return;									// Don't process this packet
		}
	}
#endif

	if ((chgr_events & CHGR_SOAKING) == 0) {
		// Check for a voltage response
		// Expecting \123:1234 V  ret
		//           0   45    10 11  (note space before the 'V'
		if (bmu_lastrx[0] == '\\' && bmu_lastrx[4] == ':' && bmu_lastrx[10] == 'V') {
			int bmu_id = 100 * (bmu_lastrx[1] - '0') + (bmu_lastrx[2] - '0') * 10 +
				bmu_lastrx[3] - '0';
			if (bmu_id == bmu_curr_cell) {
				bmu_events &= ~BMU_SENT;		// Call this valid and no longer unacknowledged
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
				// We expect voltage responses during charging and driving; split the logic here
				if (bCharging) {
					if (rxvolts < 359)
						// This cell is not bypassing. So the string of cells known to be in bypass
						// is zero length. Flag this
						first_bmu_in_bypass = -1;
					else {
						// This cell is in bypass. Check if the first bmu in bypass is the next one
						int next_bmu_id = bmu_id+1;
						if (next_bmu_id > NUMBER_OF_BMUS)
							next_bmu_id = 1;
						if (next_bmu_id == first_bmu_in_bypass) {
							// We have detected all cells in bypass. Now we enter the soak phase
							// The idea is to allow the last cell to have gone into bypass some
							// time to stay at that level and balance with the others
							chgr_events |= CHGR_SOAKING;
						}
						else {
							if (first_bmu_in_bypass == -1)
							// This cell is in bypass; we must be starting a new string of bypassed BMUs
							first_bmu_in_bypass = bmu_id;
						}
					} // End if (rxvolts < 359)
				} // End if (command.state == MODE_CHARGE)
				
				// Charging or driving. We use the voltage measurements to find the min and
				//	max cell voltages
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
				// Move to the next BMU (driving or charging, but only if packet valid)
				if (++bmu_curr_cell > NUMBER_OF_BMUS)
					bmu_curr_cell = 1;
			} // End if (bmu_id == curr_cell)
		} // End if valid voltage response

		bmu_events |= BMU_VOLTREQ;			// Schedule another voltage request
			
	} // End if ((chgr_events & CHGR_SOAKING) == 0)
}

//	//	//	//	//	//	//	//	//
//	Charger related functions	//
//	//	//	//	//	//	//	//	//

unsigned char chgr_lastSentPacket[12];					// Copy of the last CAN packet sent to the charger

bool chgr_sendPacket(const unsigned char* ptr)
{
	memcpy(chgr_lastSentPacket, ptr, 12);				// Copy the data to the last message buffer
	return chgr_resendLastPacket();						// Call the main transmit function
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
	chgr_events |= CHGR_SENT;						// Flag that packet is sent but not yet ack'd
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
