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
#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 36.00))// Charger voltage limit in tenths of a volt
#define CHGR_CURR_LIMIT		55					// Charger current limit in tenths of an amp
#define CHGR_EOC_SOAKT		(1 * 60 * BMU_STATUS_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger
#define CHGR_CUT_CURR		8					// Charge termination max current in tenths of an amp,
												// must be more than bypass plus max DC-DC loads
#define CHGR_TX_BUFSZ		32                  // Have observed overflow with size 16
#define CHGR_RX_BUFSZ 		16

// Public function prototypes
void chgr_init();								// Once off, "cold" initialising
void chgr_start();								// Called whenever begin charging
void readChargerBytes();
bool chgr_sendRequest(int voltage, int current, bool chargerOff);
void chgr_processPacket();
bool chgr_resendLastPacket(void);
void chgr_timer();							// Called every timer tick, for charger related processing
void chgr_off();
void chgr_sendCurrent(unsigned int iCurr);		// Send the current command now


// Public variables
extern volatile unsigned int chgr_events;
extern 			unsigned int chgr_state;
extern unsigned int charger_volt;			// MVE: charger voltage in tenths of a volt
extern unsigned int charger_curr;			// MVE: charger current in tenths of an ampere
extern unsigned char charger_status;		// MVE: charger status (e.g. bit 1 on = overtemp)
extern unsigned int chgr_soakCnt;			// Counter for soak phase
extern unsigned int chgr_bypCount;			// Balance count in BMU ticks when all in bypass and under
											//	cutoff current

class chgr_tx_queue : public queue {
	// Allocate space for the real buffer. Note that the base code will use member buf.
	char real_buf[CHGR_TX_BUFSZ];
public:
	chgr_tx_queue();
};
class chgr_rx_queue : public queue {
	// Allocate space for the real buffer. Note that the base code will use member buf.
	char real_buf[CHGR_RX_BUFSZ];
public:
	chgr_rx_queue();
};

// Charger buffers
extern chgr_tx_queue chgr_tx_q;
extern chgr_rx_queue chgr_rx_q;

