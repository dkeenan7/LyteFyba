/*
 * Charger interface header file
 *
 * Created 11-Apr-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"

// Charger constants
#define NUMBER_OF_CELLS		109
#define NUMBER_OF_BMUS		109
#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 36.00 * 1.021))// Charger voltage limit in tenths of a volt
							// Charger-B reads the battery voltage as 2.1% higher than it really is.
#define CHGR_CURR_LIMIT		55					// Charger current limit in tenths of an amp
#define CHGR_EOC_SOAKT		(1 * 60 * BMU_STATUS_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger
#define CHGR_CUT_CURR		8					// Charge termination max current in tenths of an amp,
												// must be more than bypass plus max DC-DC loads
#define CHGR_TX_BUFSZ		32                  // Have observed overflow with size 16
#define CHGR_RX_BUFSZ 		16
#define CHGR_RX_TIMEOUT		500					// Charger recieve timeout in 10 ms timer ticks.
												//  Should receive something every second
#define CHGR_TX_TIMEOUT		100					// Charger transmit timeout in 10 ms timer ticks.
												//  Should transmit something every second
// Charger state
#define CHGR_IDLE			0x0000				// Not charging -- bat is full or not in charge mode
#define CHGR_CHARGING		0x0001				// We are charging
// #define CHGR_SOAKING		0x0002				// We are soaking with all BMUs in bypass -- not used

// Charger events
// 	None at present


// Public function prototypes
void chgr_init();								// Once off, "cold" initialising
void chgr_start();								// Called when entering charge mode
void chgr_idle();								// Called in charge mode when the battery finishes charging
void chgr_stop();								// Called when leaving charge mode
void chgr_timer();								// Called every 10 ms timer tick, for charger related processing
bool chgr_sendCurrent(unsigned int iCurr);		// Send the current command now
bool chgr_sendRequest(int voltage, int current, bool chargerOff);
bool chgr_resendLastPacket(void);
void readChargerBytes();
void chgr_processPacket();


// Public variables
extern volatile unsigned int chgr_events;
extern 			unsigned int chgr_state;
extern int chgr_rx_timer;					// MVE: counts to zero; reset when receive anything from charger
extern int chgr_tx_timer;					// MVE: counts to zero; reset when transmit anything to charger
extern unsigned int charger_volt;			// MVE: charger voltage in tenths of a volt
extern unsigned int charger_curr;			// MVE: charger current in tenths of an ampere
extern unsigned char charger_status;		// MVE: charger status (e.g. bit 1 on = overtemp)
extern unsigned int chgr_soakCnt;			// Counter for soak phase
extern unsigned int chgr_bypCount;			// Balance count in BMU ticks when all in bypass and under
											//	cutoff current
// Charger buffers
extern queue chgr_tx_q;
extern queue chgr_rx_q;
