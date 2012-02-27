#include "tri86.h"
#include "usci.h"
#include "can.h"
#include "bms.h"


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

// Called by bmu_sendVoltReq below
void makeVoltCmd(unsigned char* cmd, int cellNo)
{
	unsigned char* p; p = cmd;
	int n = cellNo;
	int h = n / 100;
	if (h) {
		*p++ = h + '0';
		n -= h * 100;
	}
	int t = n / 10;
	if (t) {
		*p++ = t + '0';
		n -= t * 10;
	}
	*p++ = n + '0';
	*p++ = 's'; *p++ = 'v'; *p++ = '\r'; *p++ = '\0';
}

// Send a request for a voltage reading, to a specific BMU
bool bmu_sendVoltReq(unsigned int cellNo)
{
	unsigned char cmd[8];
	makeVoltCmd(cmd, cellNo);			// cmd := "XXsv\r"
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

void handleBMUbadnessEvent(unsigned int* bmu_curr_cell)
{
	if (bmu_badness > 0x80)					// Simple algorithm:
		chgr_current = BMU_BYPASS_CAP - CHGR_CURR_DELTA;	// On any badness, cut back to
															// what the BMUs can bypass
	if ((chgr_events & (CHGR_SOAKING | CHGR_END_CHARGE)) == 0) {
		// Send a voltage check for the current cell
		bmu_sendVoltReq(*bmu_curr_cell);
		++*bmu_curr_cell;
		if (*bmu_curr_cell > NUMBER_OF_BMUS)
			*bmu_curr_cell = 1;
	}
}

// Read incoming bytes from BMUs
void readBMUbytes()
{	unsigned char ch;
	while (bmu_getByte(&ch)) {			// Get a byte from the BMU receive queue
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
	
// Read incoming bytes from charger
void readChargerBytes()
{	unsigned char ch;
	while (chgr_getByte(&ch)) {
		chgr_lastrx[chgr_lastrxidx++] = ch;
		if (chgr_lastrxidx == 12)	{	// All charger messages are 12 bytes long
			chgr_events |= CHGR_REC;	// We've received a charger response
			chgr_events &= ~CHGR_SENT;	// No longer unacknowledged
			break;
		}
	}
}
