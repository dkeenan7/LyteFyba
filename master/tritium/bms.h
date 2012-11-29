/*
 * Battery Management System interface header file
 *
 * Created 25-Feb-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

#include "queue.h"
#include "pid.h"

// BMU constants
#define USE_CKSUM 1						// Set non-zero to send and expect checksums to BMUs
#define BMU_TICK_RATE		4			// Number of BMU status bytes per second
#define BMU_TX_BUFSZ		64
#define BMU_RX_BUFSZ		64
#define BMU_VR_SPEED		4500		// Number of 10 ms ticks per voltage request
										// Note: it takes > 30s to send a voltage request to 114 BMUs


// Public Function prototypes
void bms_init();
bool bmu_sendVoltReq();
bool bmu_sendVAComment(int nVolt, int nAmp);
void can_sendCellMaxMin(unsigned int bmu_min_mV, unsigned int bmu_max_mV,
							unsigned int bmu_min_id, unsigned int bmu_max_id);
void handleBMUstatusByte(unsigned char status);
void readBMUbytes();
void bmu_processPacket();
void bmu_changeDirection(bool charging);
bool bmu_resendLastPacket(void);
void bmu_timer();


class bmu_queue : public queue {
	// Allocate space for the real buffer. Note that the base code will use member buf.
	char real_buf[BMU_RX_BUFSZ];		// Assume that the rx buffer is no smaller than the tx buffer
public:
	bmu_queue(unsigned char sz);
};

// Public variables
extern volatile unsigned int bmu_events;
extern volatile unsigned int bmu_sent_timeout;

// BMU buffers
extern bmu_queue bmu_tx_q;
extern bmu_queue bmu_rx_q;

