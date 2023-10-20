/*
 * Charger interface header file
 *
 * Created 11-Apr-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"

// Charger constants
#define NUMBER_OF_CELLS		109					// Number of cells in each charger's half-pack
#define NUMBER_OF_CMUS		109					// Number of cell monitoring units in half-pack's BMS
#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 35.70 * 1.009))// Charger voltage limit in tenths of a volt
							// Charger-B reads the battery voltage as 0.9% higher than it really is.
#define CHGR_CURR_LIMIT		55					// Charger current limit in tenths of an amp
												// Note that the CAN chargers could go to about 10 amps
#define CHGR_CURR_MIN		 2					// Minimum charge current in tenths of an amp (< bypass)
#define CHGR_EOC_SOAKT		(1 * 60 * BMS_STATUS_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger
#define CHGR_CUT_CURR		10					// Charge termination max current in tenths of an amp,
												// must be more than bypass plus max DC-DC loads
#define CHGR_TX_BUFSZ		64                  // Observed overflow with 32, due to rapid stress changes
#define CHGR_RX_BUFSZ 		16
#define CHGR_RX_TIMEOUT		500					// Charger recieve timeout in 10 ms timer ticks.
												//  Should receive something every second
#define CHGR_TX_TIMEOUT		100					// Charger transmit timeout in 10 ms timer ticks.
												//  Should transmit something every second
// Charger state
#define CHGR_IDLE			0x0000				// Not charging -- bat is full or not in charge mode
#define CHGR_CHARGING		0x0001				// We are charging
// #define CHGR_SOAKING		0x0002				// We are soaking with all CMUs in bypass -- not used

// Charger events
// 	None at present

bool chgr_sendByte(unsigned char ch);			// FIXME! Deleteme!

// Public function prototypes
void chgr_init();								// Once off, "cold" initialising
void chgr_start();								// Called when entering charge mode
void chgr_idle();								// Called in charge mode when the battery finishes charging
void chgr_stop();								// Called when leaving charge mode
void chgr_timer(unsigned int switches);			// Called every 10 ms timer tick, for charger related processing
bool chgr_sendCurrent(unsigned int iCurr);		// Send the current command now
bool chgr_sendRequest(unsigned int voltage, unsigned int current, bool chargerOff);
bool chgr_resendLastPacket(void);
void readChargerBytes();
void chgr_processSerPacket();
void chgr_processCanPacket(unsigned long canId, bool bSwapped, unsigned int current);


// Public variables
extern volatile unsigned int chgr_events;
extern 			unsigned int chgr_state;
extern int chgr_rx_timer;					// MVE: counts to zero; reset when receive anything from charger
extern int chgr_tx_timer;					// MVE: counts to zero; reset when transmit anything to charger
extern unsigned int charger_volt;			// MVE: charger voltage in tenths of a volt
extern unsigned int charger_curr;			// MVE: charger current in tenths of an ampere
extern unsigned char charger_status;		// MVE: charger status (e.g. bit 1 on = overtemp)
extern unsigned int chgr_soakCnt;			// Counter for soak phase
extern unsigned int chgr_bypCount;			// Balance count in BMS ticks when all in bypass and under
											//	cutoff current
// Charger buffers
extern queue chgr_tx_q;
extern queue chgr_rx_q;

