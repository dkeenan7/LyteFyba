/*
 * Battery Management System (BMUs and charger) interface header file
 *
 * Created 25-Feb-2012
 * by Mike Van Emmerik and Dave Keenan
 *
 */

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
#define USE_CKSUM 1								// Set non-zero to send and expect checksums


// Public Function prototypes
extern	void bms_init();
extern	bool bmu_sendVoltReq(unsigned int cellNo);
extern  bool bmu_sendVAComment(int nVolt, int nAmp);
extern  void can_sendCellMaxMin(unsigned int bmu_min_mV, unsigned int bmu_max_mV,
								unsigned int bmu_min_id, unsigned int bmu_max_id);
extern	void handleBMUbadnessEvent(unsigned int* bmu_curr_cell);
extern	void readBMUbytes();
extern	void readChargerBytes();

// Public variables
volatile unsigned int chgr_events = 0;
volatile unsigned int bmu_events = 0;
volatile unsigned char bmu_badness = 0;		// Zero says we have received no badness so far

unsigned int charger_volt = 0;			// MVE: charger voltage in tenths of a volt
unsigned int charger_curr = 0;			// MVE: charger current in tenths of an ampere
unsigned char charger_status = 0;		// MVE: charger status (e.g. bit 1 on = overtemp)
unsigned int chgr_current = 9 - CHGR_CURR_DELTA;	// Charger present current; initially 0.9 A
										// (incremented before first use)
unsigned int chgr_report_volt = 0;		// Charger reported voltage in tenths of a volt
unsigned int chgr_soaking = 0;			// Counter for soak phase

// Charger buffers
queue chgr_tx_q = {						// Initialise structure members and size of
	.rd = 0,
	.wr = 0,
	.bufSize =      CHGR_TX_BUFSZ,
	.buf = { [0 ... CHGR_TX_BUFSZ-1] = 0 }
};

queue chgr_rx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      CHGR_RX_BUFSZ,
	.buf = { [0 ... CHGR_RX_BUFSZ-1] = 0 }
};

volatile unsigned char chgr_txcnt = 0;		// Count of bytes transmitted
volatile unsigned char chgr_rxcnt = 0;		// Count of bytes received
static	 unsigned char chgr_txbuf[12];		// A buffer for a charger packet

// BMU buffers and variables
queue bmu_tx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      BMU_TX_BUFSZ,
	.buf = { [0 ... BMU_TX_BUFSZ-1] = 0 }
};

queue bmu_rx_q = {
	.rd = 0,
	.wr = 0,
	.bufSize =      BMU_RX_BUFSZ,
	.buf = { [0 ... BMU_RX_BUFSZ-1] = 0 }
};

volatile unsigned int  bmu_min_mV = 9999;	// The minimum cell voltage in mV
volatile unsigned int  bmu_max_mV = 0;	// The maximum cell voltage in mV
volatile unsigned int  bmu_min_id = 0;	// Id of the cell with minimum voltage
volatile unsigned int  bmu_max_id = 0;	// Id of the cell with maximum voltage

unsigned char bmu_lastrx[BMU_RX_BUFSZ];	// Buffer for the last received BMU response
unsigned char bmu_lastrxidx;			// Index into the above
unsigned char chgr_lastrx[12];			// Buffer for the last received charger message
unsigned char chgr_lastrxidx;			// Index into the above
