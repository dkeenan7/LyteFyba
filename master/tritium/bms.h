/*
 * Battery Management System (BMUs and charger) interface header file
 *
 * Created 25-Feb-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"

// Charger constants
#define NUMBER_OF_CELLS		73
#define NUMBER_OF_BMUS		18		// FIXME: testing with 18 BMUs
#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 34.0))	// Charger voltage limit in tenths of a volt
#define CHGR_CURR_LIMIT		60					// Charger current limit in tenths of an amp
#define CHGR_CURR_DELTA		1					// Amount to increase the current by every second
#define CHGR_EOC_SOAKT		(5 * 60 * TICK_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger
#define CHGR_SOAK_CURR		5					// Soak mode current in tenths of an ampere
												//	Should be about 1/2 of BMU bypass capacity

// BMU constants
#define BMU_BYPASS_CAP		9					// BMU bypass capability in tenths of an amp
#define USE_CKSUM 1								// Set non-zero to send and expect checksums to BMUs


// Public Function prototypes
void bms_init();
bool bmu_sendVoltReq();
bool bmu_sendVAComment(int nVolt, int nAmp);
void can_sendCellMaxMin(unsigned int bmu_min_mV, unsigned int bmu_max_mV,
							unsigned int bmu_min_id, unsigned int bmu_max_id);
void handleBMUbadnessEvent();
void readBMUbytes();
void readChargerBytes();
void chgr_sendRequest(int voltage, int current, bool chargerOff);
void bmu_processPacket(bool bCharging);
void chgr_processPacket();
bool bmu_resendLastPacket(void);
bool chgr_resendLastPacket(void);



// Public variables
extern volatile unsigned int chgr_events;
extern volatile unsigned int bmu_events;
extern volatile unsigned char bmu_badness;		// Zero says we have received no badness so far
extern volatile unsigned int chgr_sent_timeout;
extern volatile unsigned int bmu_sent_timeout;

extern unsigned int charger_volt;			// MVE: charger voltage in tenths of a volt
extern unsigned int charger_curr;			// MVE: charger current in tenths of an ampere
extern unsigned char charger_status;		// MVE: charger status (e.g. bit 1 on = overtemp)
extern unsigned int chgr_current;			// Charger present current; initially 0.9 A
											// (incremented before first use)
extern unsigned int chgr_report_volt;		// Charger reported voltage in tenths of a volt
extern unsigned int chgr_soaking;			// Counter for soak phase

// BMU buffers and variables
// BMU buffers
#define BMU_TX_BUFSZ	64
#define BMU_RX_BUFSZ	64
extern queue bmu_tx_q;
extern queue bmu_rx_q;

// Charger buffers
#define CHGR_TX_BUFSZ	16
#define CHGR_RX_BUFSZ 	16
extern queue chgr_tx_q;
extern queue chgr_rx_q;

// FIXME: What are these?
//extern volatile unsigned char chgr_txcnt = 0;		// Count of bytes transmitted
//extern volatile unsigned char chgr_rxcnt = 0;		// Count of bytes received

unsigned char bmu_lastrx[BMU_RX_BUFSZ];	// Buffer for the last received BMU response
unsigned char bmu_lastrxidx;			// Index into the above

