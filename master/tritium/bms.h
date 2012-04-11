/*
 * Battery Management System interface header file
 *
 * Created 25-Feb-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"

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
void readBMUbytes(bool bCharging);
void bmu_processPacket(bool bCharging);
bool bmu_resendLastPacket(void);
void bmu_timer();



// Public variables
extern volatile unsigned int bmu_events;
extern volatile unsigned char bmu_badness;		// Zero says we have received no badness so far
extern volatile unsigned int bmu_sent_timeout;

// BMU buffers
#define BMU_TX_BUFSZ	64
#define BMU_RX_BUFSZ	64
extern queue bmu_tx_q;
extern queue bmu_rx_q;

unsigned char bmu_lastrx[BMU_RX_BUFSZ];	// Buffer for the last received BMU response
unsigned char bmu_lastrxidx;			// Index into the above

