/*
 * Tritium TRI86 EV Driver Controls, version 2 hardware
 * Copyright (c) 2010, Tritium Pty Ltd.  All rights reserved.
 *
 */

// Pin Definitions
// Port 1
#define IN_FUEL				0x01
#define IN_BRAKEn			0x02
#define IN_IGN_STARTn		0x04
#define IN_IGN_ONn			0x08
#define IN_IGN_ACCn			0x10
#define BRAKE_OUT			0x20
#define REVERSE_OUT			0x40
#define CAN_PWR_OUT			0x80
#define P1_UNUSED			0x00

// Port 2
#define IN_GEAR_1			0x01
#define IN_GEAR_2			0x02
#define IN_GEAR_3			0x04
#define IN_GEAR_4			0x08
#define IN_GEAR_5			0x10
#define IN_GEAR_6			0x20
#define EXPANSION_IRQ		0x40
#define CAN_INTn			0x80
#define P2_UNUSED			0x00

// Port 3
#define CAN_CSn				0x01
#define CAN_MOSI			0x02
#define CAN_MISO			0x04
#define CAN_SCLK			0x08
#define CHARGER_TXD			0x10
#define CHARGER_RXD			0x20
#define BMS_TXD				0x40
#define BMS_RXD				0x80
#define P3_UNUSED			0x00

// Port 4
#define EXPANSION_GPIO		0x01
#define GAUGE_4_OUT			0x02
#define GAUGE_3_OUT			0x04
#define GAUGE_2_OUT			0x08
#define GAUGE_1_OUT			0x10
#define LED_PWM				0x20
#define LED_REDn			0x40
#define LED_GREENn			0x80
//#define P4_UNUSED			0x40 | 0x80		// MVE: no longer unused

// Port 5
#define LED_FAULT_3			0x01
#define LED_FAULT_2			0x02
#define LED_FAULT_1			0x04
#define LED_GEAR_BL			0x08
#define LED_GEAR_4			0x10
#define LED_GEAR_3			0x20
#define LED_GEAR_2			0x40
#define LED_GEAR_1			0x80
#define P5_UNUSED			0x00

#define LED_GEAR_ALL		(LED_GEAR_4 | LED_GEAR_3 | LED_GEAR_2 | LED_GEAR_1)

// Port 6
#define ANLG_V_ENABLE		0x01
#define ANLG_SENSE_C		0x02
#define ANLG_SENSE_B		0x04
#define ANLG_SENSE_A		0x08
#define ANLG_SENSE_V		0x10
#define ANLG_BRAKE_I		0x20
#define ANLG_REVERSE_I		0x40
#define ANLG_CAN_PWR_I		0x80
#define P6_UNUSED			0x00

// Constant Definitions
#define	TRUE				1
#define FALSE				0

// Pushbutton switch states
#define PUSHED				1
#define RELEASED			0

// Drive states
#define MODE_OFF			0
#define MODE_ON				1					// There currently is no MODE_ON: if get ignition from
												//	MODE_OFF, go to MODE_N
#define MODE_START			2
#define MODE_R				3
#define MODE_N				4
#define MODE_B				5
#define MODE_D				6
#define MODE_CHARGE			7

// Event timing
#define INPUT_CLOCK			16000000			// Hz
#define TICK_RATE			100					// Hz
#define COMMS_SPEED			10					// Number of ticks per event: 10 ticks = 100ms = 10 Hz
#define CHARGE_FLASH_SPEED	20					// LED flash rate in charge mode: 20 ticks = 200ms = 5 Hz
#define ACTIVITY_SPEED		2					// LED flash period for activity: 2 ticks = 20ms
#define FAULT_SPEED			200					// LED sustain period for FAULT led: 200 ticks = 2 seconds
#define CHARGER_SPEED		100					// Charger update speed: 1 per second
#define CHGR_TIMEOUT		50					// Charger timeout in timer ticks; absolute minumum is
													// about 13*4*2 = 104 ms
#define BMU_TIMEOUT			100					// BMU timeout in timer ticks; absolute minumum is
													// about 250 ms

// Event definitions
#define EVENT_TIMER			0x0001				// Timer went off
#define EVENT_COMMS			0x0002				// Time to transmit telemetry
#define EVENT_REGEN			0x0004				// Motor controller is regenning
#define EVENT_PRECHARGE		0x0008				// Precharge has completed
#define EVENT_SLOW			0x0010				// Vehicle is within ENGAGE_VEL_R and ENGAGE_VEL_F speeds
#define EVENT_FORWARD		0x0020				// Vehicle is driving above ENGAGE_VEL_F speed
#define EVENT_REVERSE		0x0040				// Vehicle is reversing above ENGAGE_VEL_R speed
#define EVENT_CONNECTED		0x0080				// CAN bus is present and communicating
#define EVENT_ACTIVITY		0x0100				// CAN controller or UART just transmitted a packet
#define EVENT_REQ_SLEEP		0x0200				// Request sleep mode
#define EVENT_FAULT			0x0400				// MVE: turn on fault light
#define EVENT_CHARGER		0x0800				// MVE: time to send voltage and current command to CAN charger
#define EVENT_GAUGE1		0x1000				// Signal that gauge 1 has been recalculated and requires update
#define EVENT_GAUGE2		0x2000				// Signal that gauge 2 has been recalculated and requires update
#define EVENT_GAUGE3		0x4000				// Signal that gauge 3 has been recalculated and requires update
#define EVENT_GAUGE4		0x8000				// Signal that gauge 4 has been recalculated and requires update

// Charger events
#define CHGR_SENT			0x0001				// We have sent the charger a packet, not replied to yet
#define	CHGR_REC			0x0002				// We have received a packet from the charger
#define CHGR_SOAKING		0x0004				// We are soaking with all BMUs in bypass
#define CHGR_END_CHARGE		0x0008				// The charger is turned off; end of charge

// BMU events
#define BMU_SENT			0x0001				// We have sent the BMU a command, no response yet
#define	BMU_REC				0x0002				// We have received a response from the BMU string
#define BMU_BADNESS			0x0004				// We have received a badness value from the BMU string
#define BMU_MINMAX			0x0008				// Time to send a voltage reading to get cell min and max

// Charger constants
//#define NUMBER_OF_CELLS	60
#define NUMBER_OF_CELLS		19		// FIXME: testing with 19 BMUs
//#define CHGR_VOLT_LIMIT		((int)(NUMBER_OF_CELLS * 36.5))	// Charger voltage limit in tenths of a volt
//#define   CHGR_VOLT_LIMIT		((int)(  60            * 36.5))	// Charger voltage limit in tenths of a volt
#define   CHGR_VOLT_LIMIT		1995	// Charger voltage limit in tenths of a volt
#define CHGR_CURR_LIMIT		60					// Charger current limit in tenths of an amp
#define CHGR_CURR_DELTA		1					// Amount to increase the current by every second
#define CHGR_EOC_SOAKT		(5 * 60 * TICK_RATE)// Number of ticks from first detect of all bypass to
												//	turning off the charger

// BMU constants
#define BMU_BYPASS_CAP		9					// BMU bypass capability in tenths of an amp
#define USE_CKSUM 1								// Set non-zero to send and expect checksums

// Control parameters
#define ENGAGE_VEL_F		50					// Don't allow drive direction change above this speed, rpm
#define ENGAGE_VEL_R		-50					// Don't allow drive direction change above this speed, rpm
#define REGEN_THRESHOLD		-5					// Brake lights come on above this motor current, A

// Device serial number
#define DEVICE_SERIAL		5197

// Public variables
extern volatile unsigned int events;
extern volatile unsigned int chgr_events;		// Charger events
extern volatile unsigned int bmu_events;		// BMU events
extern volatile unsigned char bmu_badness;		// BMU badness
extern unsigned int chgr_current;				// Charger present current
extern unsigned int chgr_report_volt;			// Charger reported voltage

// Charger buffers
#define CHGR_TX_BUFSZ	16
// The following is not actually volatile, but we declare it such so we can avoid warnings about lost
//	qualifiers
extern volatile unsigned char chgr_txbuf[CHGR_TX_BUFSZ];	// Buffer for a transmitted charger "CAN" packet
#define CHGR_RX_BUFSZ 16
extern volatile unsigned char chgr_rxbuf[CHGR_RX_BUFSZ];	// Buffer for a received charger "CAN" packet
extern unsigned char chgr_txwr;						// Write index into the charger transmit buffer
extern volatile unsigned char chgr_txrd;			// Read index into the charger transmit buffer
extern volatile unsigned char chgr_rxwr;			// Write index into the charger receive buffer
extern			unsigned char chgr_rxrd;			// Read index into the charger receive buffer

// BMU buffers and variables
#define BMU_TX_BUFSZ	64
extern volatile unsigned char bmu_txbuf[BMU_TX_BUFSZ];	// Buffer for a transmitted BMU command
#define BMU_RX_BUFSZ	64
extern volatile unsigned char bmu_rxbuf[BMU_RX_BUFSZ];	// Buffer for a received BMU response
extern volatile unsigned char bmu_txwr;				// Write index into the BMU transmit buffer
extern volatile unsigned char bmu_txrd;				// Read index into the BMU transmit buffer
extern volatile unsigned char bmu_rxwr;				// Write index into the BMU  receive buffer
extern volatile unsigned char bmu_rxrd;				// Read index into the BMU  receive buffer

void fault() __attribute__ ((noinline));			// Single flash the error LED


// Typedefs for quickly joining multiple bytes/ints/etc into larger values
// These rely on byte ordering in CPU & memory - i.e. they're not portable across architectures
typedef union _group_64 {
	float data_fp[2];
	unsigned char data_u8[8];
	char data_8[8];
	unsigned int data_u16[4];
	int data_16[4];
	unsigned long data_u32[2];
	long data_32[2];
} group_64;

typedef union _group_32 {
	float data_fp;
	unsigned char data_u8[4];
	char data_8[4];
	unsigned int data_u16[2];
	int data_16[2];
	unsigned long data_u32;
	long data_32;
} group_32;

typedef union _group_16 {
	unsigned char data_u8[2];
	char data_8[2];
	unsigned int data_u16;
	int data_16;
} group_16;

typedef unsigned char bool;			// C does not define type bool
#define true 1
#define false 0