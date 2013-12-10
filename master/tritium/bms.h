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

// BMU constants
#define USE_CKSUM 1						// Set non-zero to send and expect checksums to BMUs
#define BMU_STATUS_RATE		15			// Number of BMU status bytes per second
#define BMU_STATUS_PER		(1000/BMU_STATUS_RATE)	// Period in ms of BMU status bytes
#define BMU_TX_BUFSZ		64
#define BMU_RX_BUFSZ		64
#define BMU_TIMEOUT			100			// BMU timeout in timer ticks; absolute minimum is
										//   about 250 ms = 25 ticks
#define BMU_VR_SPEED		4500		// Number of 10 ms ticks per voltage request
										// Note: it takes > 30s to send a voltage request to 114 BMUs
#define BMU_STATUS_TIMEOUT	50			// Number of 10 ms ticks we tolerate with no BMU status byte
#define BMU_FAKESTATUS_RATE ((BMU_STATUS_PER+5)/10) // Number of 10 ms ticks between fake status
										//	when detect BMU comms error
#define LIMP_CURR			0.12		// DC bus current limit (fraction of max) when limping
#define BUS_CURRENT_OFFSET	(7./180.)	// Overcome a constant ~ 7 A of bus current measurement error

// Status-byte bit-masks
#define COM_ERR				(1<<6)		// Communications error
#define ALL_NBYP			(1<<5)		// All near bypass -- charge termination condition
#define ENC_STRESS			0x1F		// Encoded stress. Bits 0-4. Bit 4 is a check bit
#define STRESS				0x0F		// Raw stress. Bits 0-3

// BMU events
#define BMU_SENT			0x0001				// We have sent the BMU a command, no response yet
//#define	BMU_REC			0x0002				// We have received a response from the BMU string
//#define BMU_BADNESS		0x0004				// We have received a badness value from the BMU string
//#define BMU_VOLTREQ		0x0008				// Time to send a voltage request to BMUs
//#define BMU_RESEND		0x0010				// Resend the last BMU packet from main loop

// Public Function prototypes
void bms_init();
bool bmu_sendByte(unsigned char ch);
bool bmu_sendVoltReq();
bool bmu_sendVAComment(int nVolt, int nAmp);
void can_queueCellMaxMin(unsigned int bmu_min_mV, unsigned int bmu_max_mV,
							unsigned int bmu_min_id, unsigned int bmu_max_id);
void handleBMUstatusByte(unsigned char status);
void readBMUbytes();
void bmu_processPacket();
void bmu_changeDirection(bool charging);
bool bmu_resendLastPacket(void);
void bmu_timer();


// Public variables
extern volatile unsigned int bmu_events;
extern volatile unsigned int bmu_sent_timeout;
extern float fRemoteCurLim;
extern unsigned int uChgrCurrLim;				// Charger current limit, in tenths of a volt

// BMU buffers
extern queue bmu_tx_q;
extern queue bmu_rx_q;

