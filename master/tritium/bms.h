/*
 * Battery Management System interface header file
 *
 * Created 25-Feb-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"
#include "pid.h"
#include "pedal.h"						// For command_variables and command

// BMS constants
#define USE_CKSUM 1						// Set non-zero to send and expect checksums to/from IMU and CMUs
#define BMS_STATUS_RATE		15			// Number of BMS status bytes per second
#define BMS_STATUS_PER		(1000/BMS_STATUS_RATE)	// Period in ms of BMS status bytes
#define BMS_TX_BUFSZ		64
#define BMS_RX_BUFSZ		64
#define BMS_TIMEOUT			100			// BMS timeout in timer ticks; absolute minimum is
										//   about 250 ms = 25 ticks
#define BMS_VR_SPEED		4500.F		// Number of 10 ms ticks per voltage request
										// Note: it takes > 30s to send a voltage request to 109 CMUs
#define BMS_STATUS_TIMEOUT	50			// Number of 10 ms ticks we tolerate with no BMS status byte
#define BMS_FAKESTATUS_RATE ((BMS_STATUS_PER+5)/10) // Number of 10 ms ticks between fake status
										//	when detect BMS comms error
#define LIMP_CURR			0.125F		// DC bus current limit (fraction of max) when limping
#define BUS_CURRENT_OFFSET	0.0F		// DC bus current measurement error (fraction of max)

// Status-byte bit-masks
#define COM_ERR				(1<<6)		// Communications error
#define ALL_NBYP			(1<<5)		// All near bypass -- charge termination condition
#define ENC_STRESS			0x1F		// Encoded stress. Bits 0-4. Bit 4 is a check bit
#define STRESS				0x0F		// Raw stress. Bits 0-3

// BMS events
#define BMS_SENT			0x0001				// We have sent the BMS a command, no response yet
//#define	BMS_REC			0x0002				// We have received a response from the BMS
//#define BMS_BADNESS		0x0004				// We have received a badness value from the BMS
//#define BMS_VOLTREQ		0x0008				// Time to send a voltage request to BMS
//#define BMS_RESEND		0x0010				// Resend the last BMS packet from main loop

// Public Function prototypes
void bms_init();
bool bms_sendByte(unsigned char ch);
bool bms_sendVoltReq();
bool bms_sendCurrentReq();
bool bms_sendInsulReq();
bool bms_sendFuelGaugeReq();
bool bms_sendVAComment(int nVolt, int nAmp);
void can_queueCellMaxMin(unsigned int bms_min_mV, unsigned int bms_max_mV,
							unsigned int bms_min_id, unsigned int bms_max_id);
void bms_processStatusByte(unsigned char status);
void readBMSbytes();
void bms_processPacket();
void bms_changeDirection(bool charging);
bool bms_resendLastPacket(void);
void bms_timer();


// Public variables
extern volatile unsigned int bms_events;
extern volatile unsigned int bms_sent_timeout;
extern unsigned int uChgrCurrLim;			// Charger current limit, in tenths of a volt
extern unsigned int bForceChgrCurrLim;		// When true, ignore PI controller and force max chgr curr
extern unsigned int bmsStatusBtimeout;		// Timeout for status via CAN from DCU-B to DCU-A

// BMS buffers
extern queue bms_tx_q;
extern queue bms_rx_q;

