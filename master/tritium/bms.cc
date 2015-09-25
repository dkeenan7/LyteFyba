#define USE_VOLT_REQ 0		// 1 to send voltage requests every 45 sec to get min and max cell voltage

#include <string.h>			// For strlen() etc
#include <msp430x24x.h>		// For UC1IE etc

#include "tri86.h"
#include "bms.h"
#include "charger.h"
#include "queue.h"
#include "can.h"			// For can_push(), but also general definitions
#include "gauge.h"			// For gauge_stress_update()
#include "assert2.h"		// assert-like function

// Private function prototypes
bool bms_sendPacket(const unsigned char* ptr);
void SendChgrLim(unsigned int uChgrLim);
void SendBMScurr(int uBMScurr);
void SendBMSinsul(int uBMSinsul);
void SendBMSfuel(int uBMSfuel);
void SendBMSvolt(int uBMSvolt);

// Public variables
volatile unsigned int bms_events = 0;
		 unsigned int bms_state = 0;
volatile unsigned int bms_sent_timeout;
volatile unsigned int bms_vr_count = BMS_VR_SPEED;	// Counts BMS_VR_SPEED to 1 for voltage requests
		 unsigned int chgr_lastCurrent = 0;			// Last commanded charger current
int uBMScurrA=0, uBMScurrB=0;						// Half-pack A or B current (neg means charging)
int uBMSinsulA=0, uBMSinsulB=0;						// Half-pack A or B insulation test touch current
int uBMSfuelA=0, uBMSfuelB=0;						// Half-pack A or B fuel gauge state of charge
int uBMSvoltA=0, uBMSvoltB=0;						// Half-pack A or B voltage (from IMU)
unsigned int bmsStatusBtimeout = 0;			// Timeout for BMS status via CAN from DCU-B to DCU-A

// BMS buffers
queue bms_tx_q(BMS_TX_BUFSZ);
queue bms_rx_q(BMS_RX_BUFSZ);
unsigned char bms_lastrx[BMS_RX_BUFSZ];	// Buffer for the last received BMS packet

// BMS private variables
volatile unsigned int  bms_min_mV = 9999;	// The minimum cell voltage in mV
volatile unsigned int  bms_max_mV = 0;	// The maximum cell voltage in mV
volatile unsigned char  bms_min_id = 0;	// Id of the cell with minimum voltage
volatile unsigned char  bms_max_id = 0;	// Id of the cell with maximum voltage
unsigned char bms_curr_cell = 1;		// ID of CMU to send to next
bool bCharging = FALSE;					// Whether we are in charge mode
// Static globals
static unsigned int bmsFakeStatusCtr = 0;	// Counts timer ticks between sending fake status during comms timeout
static unsigned int bmsStatusTimeout = 0;	// Counts timer ticks with no status byte received from BMS

// Stress table with check bits
static unsigned char stressTable[16] = {
			(1<<4) + 0,		// Stress 0   $10
			(1<<4) + 1,		// Stress 1   $11
			(1<<4) + 2,		// Stress 2   $12
			(1<<4) + 3,		// Stress 3   $13
			(1<<4) + 4,		// Stress 4   $14
			(1<<4) + 5,		// Stress 5   $15
			(1<<4) + 6,		// Stress 6   $16
			(1<<4) + 7,		// Stress 7   $17
			(0<<4) + 8,		// Stress 8   $08
			(0<<4) + 9,		// Stress 9   $09
			(0<<4) + 10,	// Stress 10  $0A
			(0<<4) + 11,	// Stress 11  $0B
			(0<<4) + 12,	// Stress 12  $0C
			(0<<4) + 13,	// Stress 13  $0D
			(0<<4) + 14,	// Stress 14  $0E
			(0<<4) + 15		// Stress 15  $0F
};

pid pidCharge(						// State for the PID control algorithm for charge current
//		(int)((7.0/16.0) * 4096),	// Set point will be 7.0 out of 16.0, left shifted by 12 bits
		(int)(1.164*256),			// Kp as s7.8 fixed-point
		// Proportional gain of (0.4/5.5)/(1/16) = 4 * 16/55 = 1.164 means that a change of
		// 1 stress level will cause a change of 0.4 A. With this Kp, 0.4 A is the lowest non-zero
		// current that can be stable (except that we can achieve 0.3 A occasionally thanks to Kd).
		// If Kp was any larger we would not be able to achieve a stable 0.4 A (except occasionally
		// thanks to Kd). 0.4 A is the CMUs' typical bypass current.
		(int)(0.291*256),		// Ki as s7.8 fixed-point
		// Integral gain of (0.1/5.5)/(1/16) = 16/55 = 0.291 means that when the stress is 1 level
		// away from the setpoint, the requested current will change by 0.1 A per tick.
		// 0.1 A is the minimum change that the TC charger can make.
		(int)(-0.582*256),			// Kd as s7.8 fixed-point
		// Setting Kd to negative half Kp spreads the current steps due to Kp over two ticks,
		// and allows current to occasionally reach 0.3 A via a single tick spent at stress 8.
		0);							// Initial "measure"

pid pidDrive(						// State for the PID control algorithm for charge current
//		(int)((7.0/16.0) * 4096),	// Set point will be 7.0 out of 16.0, left shifted by 12 bits
		(int)(1.164*256/4),			// Kp as s7.8 fixed-point
		// Proportional gain of (0.4/5.5)/(1/16) = 4 * 16/55 = 1.164 means that a change of
		// 1 stress level will cause a change of 0.4 A. With this Kp, 0.4 A is the lowest non-zero
		// current that can be stable (except that we can achieve 0.3 A occasionally thanks to Kd).
		// If Kp was any larger we would not be able to achieve a stable 0.4 A (except occasionally
		// thanks to Kd). 0.4 A is the CMUs' typical bypass current.
		(int)(0.291*256/4),		// Ki as s7.8 fixed-point
		// Integral gain of (0.1/5.5)/(1/16) = 16/55 = 0.291 means that when the stress is 1 level
		// away from the setpoint, the requested current will change by 0.1 A per tick.
		// 0.1 A is the minimum change that the TC charger can make.
		(int)(-0.582*256/4),			// Kd as s7.8 fixed-point
		// Setting Kd to negative half Kp spreads the current steps due to Kp over two ticks,
		// and allows current to occasionally reach 0.3 A via a single tick spent at stress 8.
		0);							// Initial "measure"

void bms_init()
{
#if USE_CKSUM
	// Turn on checksumming in CMUs.
	// The "k" packet is ignored if CMU checksumming is on (bad checksum), but toggles CMU
	// checksumming on if it was off.
	bms_sendByte('k'); bms_sendByte('\r');		// NOTE: can't use bms_SendPacket as it would insert a
												// checksum giving "kk\r" and having opposite effect
#else
	// Turn off checksumming in CMUs.
	// The "kk" packet toggles CMU checksumming off if it was on (single k command with good checksum),
	// but toggles CMU checksumming twice if it was off, thereby leaving it off.
	bms_sendPacket((unsigned char*)"kk\r");		// DCU checksumming is off, so it won't change the pkt
#endif
	bms_sendPacket((unsigned char*)"0K\r");	// Turn on (turn off Killing of) BMS badness sending
	bms_state &= (uchar)~BMS_SENT;			// Don't expect these packets to be acknowledged or resent if not
//	bms_sendCurrentReq();					// Send the first current request packet;driving or charging
#if USE_VOLT_REQ
	bms_sendVoltReq();						// Send the first voltage request packet;driving or charging
#endif
}

bool bms_sendByte(unsigned char ch) {
	if (bms_tx_q.enqueue(ch)) {
    	UC1IE |= UCA1TXIE;                        		// Enable USCI_A1 TX interrupt
		events |= EVENT_ACTIVITY;						// Turn on activity light
		return true;
	}
	return false;
}

// Make a command at *cmd for IMU or CMU number cellNo. If cellNo is negative, send to all.
// Called by bms_sendVoltReq, bms_sendCurrentReq and bms_sendInsulReq below
void makeBMScmd(unsigned char* cmd, int cellNo, unsigned char cmdchar)
{
	unsigned char* p; p = cmd;
	if (cellNo >= 0) {
		unsigned char n = (unsigned char)cellNo;
		unsigned char h = n / 100;
		if (h) {						// If any hundreds
			*p++ = (uchar)(h + '0');	// then emit hundreds digit
			n = (uchar)(n - h * 100);
		}
		int t = n / 10;
		if (h || t) {					// If hundreds or tens
			*p++ = (uchar)(t + '0');	// then emit tens digit
			n = (uchar)(n - t * 10);
		}
		*p++ = (uchar)(n + '0');		// Emit units digit
		*p++ = 's';
	}
	*p++ = cmdchar; *p++ = '\r'; *p++ = '\0';	// Emit s (select) and v (voltage) or
}									// l (link or IMU current) commnd and terminate with return and null

// Send a request for a voltage reading, to a specific CMU
// Returns true on success
bool bms_sendVoltReq()
{
	unsigned char cmd[8];
	makeBMScmd(cmd, bms_curr_cell, 'v');	// cmd := "XXXsv\r"
	return bms_sendPacket(cmd);
}

// Send a request for a current reading, to "CMU" 0 which is the IMU.
// Returns true on success
bool bms_sendCurrentReq()
{
	unsigned char cmd[8];
	makeBMScmd(cmd, 0, 'l');	// cmd := "0sl\r"
	return bms_sendPacket(cmd);
}

// Send a request for an insulation test (touch current reading), a no-op to all but the IMU.
// Returns true on success
bool bms_sendInsulReq()
{
	unsigned char cmd[8];
	makeBMScmd(cmd, -1, 'I');	// cmd := "I\r"
	return bms_sendPacket(cmd);
}

// Send a request for an fuel gauge reading (depth of discharge), a no-op to all but the IMU.
// Returns true on success
bool bms_sendFuelGaugeReq()
{
	unsigned char cmd[8];
	makeBMScmd(cmd, -1, 'g');	// cmd := "g\r"
	return bms_sendPacket(cmd);
}

// Send a fuel gauge reset (to 100% SoC), a no-op to all but the IMU. Sent at charge completion.
// Returns true on success
bool bms_sendFuelGaugeReset()
{
	unsigned char cmd[8];
	makeBMScmd(cmd, -1, '%');	// cmd := "%\r"
	return bms_sendPacket(cmd);
}

// Send bus current setpoint on CAN bus to wavesculptor immediately.
// For cell protection.
void can_transmitBusCurrent(float bus_current)
{
	can_push_ptr->identifier = DC_CAN_BASE + DC_POWER;
	can_push_ptr->status = 8;
	can_push_ptr->data.data_fp[1] = bus_current + BUS_CURRENT_OFFSET;
	can_push_ptr->data.data_fp[0] = ADC12MEM0; // Pedal position. Shows in log as Misc SP
	can_push();
}

// Send BMS character on CAN bus so we can sniff it on a netbook via the CAN-Ethernet Bridge
void can_transmitBMSchar(unsigned char ch)
{
	can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_A_SNIFF + (unsigned)bDCUb; // DC_BMS_B_SNIFF if so
	can_push_ptr->status = 1; // Data length 1 byte
	can_push_ptr->data.data_u8[0] = ch;
	can_push();
}

// Send min and max cell voltage and ids on CAN bus so telemetry software on PC can display them
void can_queueCellMaxMin(unsigned int bms_min_mV, unsigned int bms_max_mV,
								unsigned int bms_min_id, unsigned int bms_max_id)
{
	// We have the min and max cell voltage information. Send a CAN packet so the telemetry
	//	software can display them. Use CAN id 0x266, as the IQcell BMS would
	can_push_ptr->identifier = 0x266;
	can_push_ptr->status = 8;
	can_push_ptr->data.data_u16[0] = bms_min_mV;
	can_push_ptr->data.data_u16[1] = bms_max_mV;
	can_push_ptr->data.data_u16[2] = bms_min_id;
	can_push_ptr->data.data_u16[3] = bms_max_id;
	can_push();
}

// Only used for debugging.
// Send total battery voltage as a comment packet on BMS channel
bool bms_sendVAComment(int nVolt, int nAmp)
{
	// Packet to announce the charger's meas of total voltage and current
	// \ C H G _ _ n n S _ n . n A \r
	// 0 1 2 3 4 5 6 7 8 9 a b c d e
	static unsigned char szChgrVolt[16] = "\\CHG nnnS n.nA\r";
	szChgrVolt[5] = (uchar)(nVolt / 1000 + '0');			// Voltage hundreds
	szChgrVolt[6] = (uchar)((nVolt % 1000) / 100 + '0');	//	tens
	szChgrVolt[7] = (uchar)((nVolt % 100) / 10 + '0');		//	units
	szChgrVolt[10] = (uchar)((nAmp / 10) + '0');			// Current units
	szChgrVolt[12] = (uchar)((nAmp % 10) + '0');			//	tenths
	bool result = bms_sendPacket(szChgrVolt); // Send as comment packet on BMS channel for debugging
	bms_state &= (unsigned)~BMS_SENT;		// Don't expect these packets to be acknowledged or resent if not
	return result;
}

#define max(x, y) (((x)>(y))?(x):(y))
#define min(x, y) (((x)<(y))?(x):(y))

void bms_processStatusByte(unsigned char status)
{
#define SET_POINT 7 // stress level
	unsigned int current;
	int output;

	// Check for validity of local status byte
	unsigned char stress = status & STRESS;			// Isolate stress bits
	unsigned char encoded = status & ENC_STRESS;	// Stress bits and check bits
	bool bValid = stressTable[stress] == encoded;
	if (!bValid)
		stress = 8;				// Treat invalid status byte as most minor dis-stress
	if (status & COM_ERR)		// If communications error
		stress = (uchar)max(stress, 8);	// Treat as at least stress 8 (charging or driving)

	if (bDCUb) { // If we are DCU-B
		// Send status in CAN packet to DCU A
		can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_B_STATUS;
		can_push_ptr->status = 1;	// Packet size in bytes
		can_push_ptr->data.data_u8[0] = status; // Note that this has not been checked for validity
		can_push();
	} else { // else we are DCU-A
		// Check for validity of status byte received in CAN packet from DCU B
		unsigned char stressB = statusB & STRESS;			// Isolate stress bits for B
		unsigned char encodedB = statusB & ENC_STRESS;
		bool bValidB = stressTable[stressB] == encodedB;
		if (!bValidB)
			stressB = 8;				// Treat invalid status byte as most minor dis-stress
		if (statusB & COM_ERR)			// If communications error
			stressB = (uchar)max(stressB, 8);	// Treat as at least stress 8
		// Calculate max of stresses from A and B half-packs
		unsigned char maxStress = max(stress, stressB);
		// Turn on the cell stress alarm if stress is 11 or more
		if (maxStress < 11)
			P5OUT |= LED_FAULT_2;			// Turn off cell stress alarm
		else
			P5OUT &= (uchar)~LED_FAULT_2;	// Turn on cell stress alarm
		// Send max stress info to the Oil Pressure gauge
		gauge_stress_update( maxStress );
		// Send the stress information in the min and max cell voltage fields for WSconfig
		// Also comms error bits in min and max cell IDs
		//can_queueCellMaxMin(bms_min_mV, bms_max_mV, bms_min_id, bms_max_id);
		can_queueCellMaxMin((unsigned)(stress*1000),
							(unsigned)(stressB*1000),
							(unsigned)((status & (COM_ERR | ALL_NBYP))>>5),
							(unsigned)((statusB & (COM_ERR | ALL_NBYP))>>5));
		if (!bCharging) { // If not charging, assume driving. (DCU-A only)
			// We need to scale the measurement (stress 0-15) to make good use of the s0.15
			// fixedpoint range (-0x8000 to 0x7FFF) while being biased so that the set-point
			// (stress 7) maps to 0x0000 and taking care to avoid overflow or underflow.
			output = pidDrive.tick(sat_minus(((int)maxStress-8) << 12, (SET_POINT-8) << 12));

			// Map fract -1.0 .. almost +1.0 to float almost 0.0 .. 1.0
			float fCurLim = ((float)output + 32769.0F) / 65536.0F;
			// When the DC current limit needs to be cut back
			// to keep the stress at or below the setpoint,
			// continue to allow a minimum current for getting out of danger,
			// but progressively reduce this current to zero as the stress increases to 15.
			float fMinCurr = LIMP_CURR/8.0 * (15 - max(maxStress, SET_POINT));
			fCurLim = fCurLim * (1 - fMinCurr) + fMinCurr;	// Map 0.0 .. 1.0 to fMinCurr .. 1.0
			can_transmitBusCurrent(fCurLim);
		} // End if not charging
	} // End else DCU-A

	if (bCharging) {
		if (chgr_state == CHGR_IDLE)
			return;
#if 0
		// Protect against errant charger or PI loop by turning off charge contactor
		if (stress >= 11) {
			chgr_idle();	// Terminate charging
			fault();
			return;
		}
#endif
		// Check for normal charge termination -- all near bypass for sufficient time at low enough current
		if (status & ALL_NBYP && (chgr_lastCurrent <= CHGR_CUT_CURR)) {
			if (++chgr_bypCount >= CHGR_EOC_SOAKT) {
				chgr_idle();				// Terminate charging
				bms_sendFuelGaugeReset();	// Reset fuel gauge counter in our IMU for 100% SoC
				return;
			}
		}
		else if (chgr_bypCount != 0)			// Care! chgr_bypCount is unsigned
			--chgr_bypCount;					// Saturate at zero

		// We need to scale the measurement (stress 0-15) to make good use of the s0.15
		// fixedpoint range (-0x8000 to 0x7FFF) while being biased so that the set-point
		// (stress 7) maps to 0x0000 and taking care to avoid overflow or underflow.
		output = pidCharge.tick(sat_minus(((int)stress-8) << 12, (SET_POINT-8) << 12));
		// Scale the output. +1.0 has to correspond to maximum charger current,
		// and -1 to minimum current. This is a range of 2^16 (-$8000 .. $7FFF),
		// which we want to map to CHGR_CURR_MIN .. uChgrCurrLim (a global that defaults to
		// CHGR_CURR_LIMIT, but	could be set lower via a CAN packet with ID CHGR_LIM).
		// We have a hardware multiplier, so the most efficient way to do this is with a
		// 16x16 bit multiply giving a 32-bit result, and taking the upper half of the result.
		// But we also want to offset the output by 1.0 ($8000).
		// To avoid overflow, we do this with 32-bit arithmetic, i.e.
		//  current = ((out + $8000L) / $10000L) * max
        //			= ((out + $8000L) * max) / $10000L	// Do division last so no fractional intermediate results
        //         =  ((out + $8000L) * max) >> 16		// Do division as shifts, for speed
        // But no actual shifts are required -- just take high word of a long
		// Also add $8000 before the >> 16 for rounding.
		current = (unsigned int)(((output + 0x8000L) * (uChgrCurrLim - CHGR_CURR_MIN) + 0x8000) >> 16) + CHGR_CURR_MIN;
		// Only send a packet to the charger if the current has changed, or on a timeout
		if ((current != chgr_lastCurrent) || (chgr_tx_timer == 0)) {
#if 1
			if (chgr_sendCurrent(current))
				chgr_lastCurrent = current;
#else
			if (chgr_sendCurrent(current)) {
				chgr_lastCurrent = current;
				if (bValid)
					bms_sendVAComment(stress*10, current); // for debugging
				else
					bms_sendVAComment(99, current); // for debugging
			}
			else
				// Charger failed to send. Indicate this
				bms_sendPacket((const unsigned char*)"\\F\r\n");
#endif
		}
	} // End of if charging
} // End of bms_processStatusByte()


// Read incoming bytes from BMS
void readBMSbytes()
{	unsigned char ch;
	static unsigned char wr = 0;		// Write index into the received packet buffer
	static unsigned char sum = 0;		// For checking packet checksum

	while (bms_rx_q.dequeue(ch)) {		// While we can get a byte from the BMS receive queue
		can_transmitBMSchar(ch);		// Send on CAN bus so it can be sniffed via ethernet bridge
		if (ch >= 0x80) {				// If it's a status byte
			bms_processStatusByte(ch);	// Process it
			bmsStatusTimeout = 0;		// Reset timeout counter
			bmsFakeStatusCtr = 0;		// Ensure fake status is sent immediately on timeout
		}
		else {							// Else it's part of a command or response packet
			if (wr < BMS_RX_BUFSZ)		// If the packet buffer is not full
				bms_lastrx[wr++] = ch;	// Append the character to the packet buffer
			if (ch != '\r')				// If it's not the end of a packet (a carriage return)
				sum ^= ch;				// Accumulate the XOR checksum
			else if ((sum != 0) || (wr >= BMS_RX_BUFSZ)) { // Else it is the end of a packet
											// and if the checksum is bad or the buffer is full
				wr = 0;					// Reset the write index
				sum = 0;				// Reset the checksum accumulator
				fault();				// Flash the red LED and beep
			}
			else {						// Else it's the end of a good packet
				bms_processPacket();	// Process it
				wr = 0;					// Reset the write index
				sum = 0;				// Reset the checksum accumulator
			} // End of Else it's the end of a good packet
		} // End of Else it's part of a packet
	} // End of While we can get a byte from the BMS receive queue
} // End of readBMSbytes()


// Act on any change in the direction of current flow.
// Temperatures below zero constitute stress when charging or regen-braking but not when accelerating.
// And vice versa for undervoltage.
// So send appropriate 'c' (Charging) command to CMUs.
void bms_changeDirection(bool chargeOrRegen)
{
	bCharging = chargeOrRegen;
	if (chargeOrRegen) bms_sendPacket((unsigned char*)"1c\r");
	else bms_sendPacket((unsigned char*)"0c\r");
	bms_state &= (unsigned)~BMS_SENT;		// Don't expect these packets to be acknowledged or resent if not
}


unsigned char bms_lastSentPacket[BMS_TX_BUFSZ];		// Copy of the last packet sent to the BMS

// Returns true on success
bool bms_sendPacket(const unsigned char* ptr)
{
#if USE_CKSUM
	unsigned char ch, i = 0, sum = 0;
	do {
		ch = *ptr++;
		sum ^= ch;									// Calculate XOR checksum
		bms_lastSentPacket[i++] = ch;				// Copy the data to the transmit buffer
	} while (ch != '\r');
	sum ^= '\r';									// CR is not part of the checksum
	if (sum < ' ') {
		// If the checksum would be a control character that could be confused with a CR, BS,
		//	etc, then send a space, which will change the checksum to a non-control character
		bms_lastSentPacket[i++-1] = ' ';			// Replace CR with space
		sum ^= ' ';									// Update checksum
	}
	bms_lastSentPacket[i++-1] = sum;				// Insert the checksum
	bms_lastSentPacket[i-1] = '\r';					// Add CR
	bms_lastSentPacket[i] = '\0';					// Null terminate; bms_resendLastPacket expects this
#else
	strcpy((char*)bms_lastSentPacket, (char*)ptr);	// Copy the buffer in case we have to resend
#endif
	return bms_resendLastPacket();					// Call the main transmit function
}

// bms_resendLastPacket is used for resending after a timeout, but also used for sending the first time.
// Returns true on success
bool bms_resendLastPacket(void)
{
	unsigned int i, len = strlen((char*)bms_lastSentPacket);
	if (bms_tx_q.queue_space() < len) {
		fault();
		return false;
	}
	for (i=0; i < len; ++i)							// Send the bytes of the packet
		bms_sendByte(bms_lastSentPacket[i]);
	bms_state |= BMS_SENT;							// Flag that packet is sent but not yet ack'd
	bms_sent_timeout = BMS_TIMEOUT;					// Initialise timeout counter
	return true;
}


void bms_processPacket()
{
	// Check for a valid response from the BMS
	// Expecting \000:l 1234  ret
	//           0   45 7  10 11  (note the space after the 'l' or 'v', and a possible minus sign)
	if (bms_lastrx[0] == '\\' &&
		bms_lastrx[4] == ':') {
		// Extract the CMU ID
		unsigned char rx_id = (uchar)
			( (bms_lastrx[1] - '0') * 100
			+ (bms_lastrx[2] - '0') * 10
			+ (bms_lastrx[3] - '0')    );
		// Extract the command character that this is a response to
		unsigned char rx_cmd = bms_lastrx[5];	// Only expecting 'v' or 'l'
		// Extract any minus sign
		int rx_sign = (bms_lastrx[7] == '-');
		unsigned int i1 = 7, i2 = 8, i3 = 9, i4 = 10;
		if (rx_sign)							// If there was a minus sign, skip it
			i1 = 8, i2 = 9, i3 = 10, i4 = 11;
		// Extract the absolute value
		int rx_value =
			( (bms_lastrx[i1] - '0') * 1000
			+ (bms_lastrx[i2] - '0') * 100
			+ (bms_lastrx[i3] - '0') * 10
			+ (bms_lastrx[i4] - '0')    );
		// Calculate the signed value
		if (rx_sign) rx_value = -rx_value;

		// Check for a current (amps) response from the IMU (ID = 0)
		if (rx_cmd == 'l' && rx_id == 0) {
			bms_state &= (unsigned)~BMS_SENT;	// Call this valid and no longer unacknowledged
			// Set global or CAN-send value to be displayed on the tacho by DCU-A
			if (bDCUb)
				SendBMScurr(2 * rx_value); 		// Tenths of an amp
			else
				uBMScurrA = 2 * rx_value;		// Tenths of an amp
//			bms_sendCurrentReq();				// Send another current request
		}
		// Check for a fuel gauge (depth of discharge) response from the IMU (ID = 0)
		else if (rx_cmd == 'g' && rx_id == 0) {
			bms_state &= (unsigned)~BMS_SENT;	// Call this valid and no longer unacknowledged
			// Set global or CAN-send value to be displayed on the tacho by DCU-A
			if (bDCUb)
				SendBMSfuel(1000-rx_value); 	// Tenths of a percent (state of charge)
			else
				uBMSfuelA = 1000-rx_value;		// Tenths of a percent (state of charge)
		}
		// Check for an insulation-test (tenths of a milliamp) response from the IMU (ID = 0)
		else if (rx_cmd == 'I' && rx_id == 0) {
			bms_state &= (unsigned)~BMS_SENT;	// Call this valid and no longer unacknowledged
			// Set global or CAN-send value to be used for insulation alarm by DCU-A
			if (bDCUb)
				SendBMSinsul(rx_value); 	// Tenths of a milliamp (prospective touch current)
			else
				uBMSinsulA = rx_value;		// Tenths of a milliamp (prospective touch current)
		}
		// Check for a voltage response from the IMU (ID = 0)
		else if (rx_cmd == 'v' && rx_id == 0) {
			bms_state &= (unsigned)~BMS_SENT;	// Call this valid and no longer unacknowledged
			// Set global or CAN-send value to be using in precharge check by DCU-A
			if (bDCUb)
				SendBMSvolt(rx_value); 		// Half-pack voltage/109 = average cell voltage (mV)
			else
				uBMSvoltA = rx_value;		// Half-pack voltage/109 = average cell voltage (mV)
		}
		// Check for a voltage response from a particular CMU
		else if (rx_cmd == 'v' && rx_id == bms_curr_cell) {
			bms_state &= (unsigned)~BMS_SENT;	// Call this valid and no longer unacknowledged
			// We use the voltage measurements to find the min and max cell voltages
			if ((unsigned)rx_value < bms_min_mV) {
				bms_min_mV = (unsigned)rx_value;
				bms_min_id = rx_id;
			}
			if ((unsigned)rx_value > bms_max_mV) {
				bms_max_mV = (unsigned)rx_value;
				bms_max_id = rx_id;
			}
			if (rx_id >= NUMBER_OF_CMUS) {
				// We have the min and max information. Send a CAN packet so the telemetry
				//	software can display them.
				can_queueCellMaxMin(bms_min_mV, bms_max_mV, bms_min_id, bms_max_id);
			}
			// Move to the next CMU, only if packet valid
			if (++bms_curr_cell > NUMBER_OF_CMUS)
				bms_curr_cell = 1;
			else {
//				bms_sendVoltReq();				// Send another voltage request for the next cell
			}
		} // End if (rx_cmd == 'v')
	} // End if valid response
} // End bms_processPacket()


// Called every timer tick from the mainline, for BMS related processing
void bms_timer() {
	if (bms_state & BMS_SENT) {
		if (--bms_sent_timeout == 0) {
			fault();
			bms_resendLastPacket();			// Resend; will loop until a complete packet is recvd
		}
	}
	if (++bmsStatusTimeout >= BMS_STATUS_TIMEOUT) {
		bmsStatusTimeout = BMS_STATUS_TIMEOUT;	// Don't allow overflow
		if (bmsFakeStatusCtr == 0)
			// Fake status with comms error and a stress of 8 (lowest distress)
			bms_processStatusByte(0x80 | COM_ERR | stressTable[8]);
		if (++bmsFakeStatusCtr == BMS_FAKESTATUS_RATE)
			bmsFakeStatusCtr = 0;
	}
	if (!bDCUb && (++bmsStatusBtimeout >= BMS_STATUS_TIMEOUT)) {
		bmsStatusBtimeout = BMS_STATUS_TIMEOUT;	// Don't allow overflow
		// Fake statusB with comms error and a stress of 8 (lowest distress)
		statusB = 0x80 | COM_ERR | stressTable[8];
	}
}


void SendBMScurr(int uBMScurr) {
	can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_CURR;
	can_push_ptr->status = 2;	// Packet size in bytes
	can_push_ptr->data.data_16[0] = uBMScurr; 	// Send B half-pack current to DCU-A
	can_push();
}

void SendBMSinsul(int uBMSinsul) {
	can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_INSUL;
	can_push_ptr->status = 2;	// Packet size in bytes
	can_push_ptr->data.data_16[0] = uBMSinsul; 	// Send B half-pack insulation-test touch-current to DCU-A
	can_push();
}

void SendBMSfuel(int uBMSfuel) {
	can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_FUEL;
	can_push_ptr->status = 2;	// Packet size in bytes
	can_push_ptr->data.data_16[0] = uBMSfuel; 	// Send B half-pack fuel gauge state of charge to DCU-A
	can_push();
}

void SendBMSvolt(int uBMSvolt) {
	can_push_ptr->identifier = DC_CAN_BASE + DC_BMS_VOLT;
	can_push_ptr->status = 2;	// Packet size in bytes
	can_push_ptr->data.data_16[0] = uBMSvolt; 	// Send B half-pack voltage to DCU-A
	can_push();
}
