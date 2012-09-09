/*
 * Charger interface header file
 *
 * Created 11-Apr-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"

// Charger constants
#define NUMBER_OF_CELLS		73
#define NUMBER_OF_BMUS		19		// FIXME: testing with 19 BMUs
#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 34.0))	// Charger voltage limit in tenths of a volt
#define CHGR_CURR_LIMIT		60					// Charger current limit in tenths of an amp
#define CHGR_CURR_DELTA		1					// Amount to increase the current by every second
#define CHGR_EOC_SOAKT		(5 * 60 * TICK_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger
#define CHGR_SOAK_CURR		5					// Soak mode current in tenths of an ampere
												//	Should be about 1/2 of BMU bypass capacity
// Public function prototypes
void chgr_init();
void readChargerBytes();
void chgr_sendRequest(int voltage, int current, bool chargerOff);
void chgr_processPacket();
bool chgr_resendLastPacket(void);
void chgr_timer();
void handleChargerEvent();


// Public variables
extern volatile unsigned int chgr_events;
extern 			unsigned int chgr_state;
extern unsigned int charger_volt;			// MVE: charger voltage in tenths of a volt
extern unsigned int charger_curr;			// MVE: charger current in tenths of an ampere
extern unsigned char charger_status;		// MVE: charger status (e.g. bit 1 on = overtemp)
extern unsigned int chgr_current;			// Charger present current; initially 0.9 A
											// (incremented before first use)
extern unsigned int chgr_report_volt;		// Charger reported voltage in tenths of a volt
extern unsigned int chgr_soakCnt;			// Counter for soak phase

// Charger buffers
#define CHGR_TX_BUFSZ	16
#define CHGR_RX_BUFSZ 	16
extern queue chgr_tx_q;
extern queue chgr_rx_q;



void chgr_timer();						// Called every timer tick, for charger related processing


