/*
 * Tritium TRI86 EV Driver Controls, version 2 hardware
 * Copyright (c) 2010, Tritium Pty Ltd.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

// Include files
#include <msp430x24x.h>
#include <signal.h>
#include "tri86.h"
#include "can.h"
#include "usci.h"
#include "pedal.h"
#include "gauge.h"

#ifdef __ICC430__								// MVE: attempt to make the source code more IAR friendly, in case
#define __inline__								//	press F7, and so that "go to definition of X" works better
#define __volatile__
#define interrupt(x) void
void eint();
void dint();
#endif

#define LED_PORT			P4OUT				// MVE: modified DCU; uses port 3 for UART, port 4 for
												//	GREENn and REDn LEDs

// Macro needed to swap from little to big endian for 16-bit charger quantities:
#define SWAP16(x) ((x >> 8) | ((x & 0xFF) << 8))

// Function prototypes
void clock_init( void );
void io_init( void );
void timerA_init( void );
void timerB_init( void );
void adc_init( void );
static void __inline__ brief_pause(register unsigned int n);
void update_switches( unsigned int *state, unsigned int *difference);

// Global variables
// Status and event flags
volatile unsigned int events = 0x0000;
volatile unsigned int chgr_events = 0;
volatile unsigned int bmu_events = 0;
volatile unsigned char bmu_badness = 0;		// Zero says we have received no badness so far
// \ C H G _ n n n V _ n . n A \r
// 0 1 2 3 4 5 6 7 8 9 a b c d e
unsigned char szChgrVolt[16] = "\\CHG nnnV n.nA\r";// Packet to announce the charger's meas of total voltage

// Data from controller
float motor_rpm = 0;
float motor_temp = 0;
float controller_temp = 0;
float battery_voltage = 0;
float battery_current = 0;
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
		 unsigned char chgr_lastrx[12];		// Buffer for the last received charger message
		 unsigned char chgr_lastrxidx = 0;	// Index into the above
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
		 unsigned char bmu_lastrxidx = 0;	// Index into the above





void fault() {
	events |= EVENT_FAULT;				// Breakpoint this instruction and use stack backtrace
										//	(but beware the compiler may well inline it)
}

void makeVoltCmd(unsigned char* cmd, int bmu_curr_cell)
{
	unsigned char* p; p = cmd;
	int n = bmu_curr_cell;
	int h = n / 100;
	if (h) {
		*p++ = h + '0';
		n -= h * 100;
	}
	int t = n / 10;
	if (t) {
		*p++ = t + '0';
		n -= t * 10;
	}
	*p++ = n + '0';
	*p++ = 's'; *p++ = 'v'; *p++ = '\r'; *p++ = '\0';
}

// Main routine
int main( void )
{
	// Local variables
	// Switch inputs - same bitfield positions as CAN packet spec
	unsigned int switches = 0x0000;
	unsigned int switches_diff = 0x0000;
	unsigned char next_state = MODE_OFF;
	// Comms
	unsigned int comms_event_count = 0;
	// LED flashing
	unsigned char charge_flash_count = CHARGE_FLASH_SPEED;
	// Debug
	unsigned int i;
	// Charger end-of-charge test
	signed	 int first_bmu_in_bypass = -1;
	// Current cell in the current end-of-charge test (send voltage request to this cell next)
	unsigned int bmu_curr_cell = 1;			// ID of BMU to send to next
	
	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;

	// Initialise I/O ports
	io_init();

	// Wait a bit for clocks etc to stabilise, and power to come up for external devices
	// MSP430 starts at 1.8V, CAN controller need 3.3V
	for(i = 0; i < 10000; i++) brief_pause(10);

	// Initialise clock module - internal osciallator
	clock_init();

	// Initialise SPI port for CAN controller (running with SMCLK)
	usci_init(0);
	
	// Reset CAN controller and initialise
	// This also changes the clock output from the MCP2515, but we're not using it in this software
	can_init();
	events |= EVENT_CONNECTED;

	// Initialise Timer A (10ms timing ticks)
	timerA_init();

	// Initialise Timer B (gauge outputs PWM / pulses)
	timerB_init();

	// Initialise A/D converter for potentiometer and current sense inputs
	adc_init();

	// Initialise switch & encoder positions
	update_switches(&switches, &switches_diff);
	
	// Initialise command state
	command.rpm = 0.0;
	command.current = 0.0;
	command.bus_current = 1.0;
	command.flags = 0x00;
//	command.state = MODE_OFF;
	command.state = MODE_D;			// For now, we're like "drive, baby, drive!" (FIXME)
	bmu_events |= BMU_VOLTREQ;		// Kick off the first voltage request packet while driving or charging
	
	// Init gauges
	gauge_init();

	// Enable interrupts
	eint();

#if USE_CKSUM	
	// Turn on checksumming in BMUs.
	// The "k" packet is ignored if BMU checksumming is on (bad checksum), but toggles BMU
	// checksumming on if it was off.
	bmu_sendByte('k'); bmu_sendByte('\r');		// NOTE: can't use bmu_SendPacket as it would insert a
												// checksum giving "kk\r" and having opposite effect
#else
	// Turn off checksumming in BMUs.
	// The "kk" packet toggles BMU checksumming off if it was on (single k command with good checksum),
	// but toggles BMU checksumming twice if it was off, thereby leaving it off.
	bmu_sendPacket("kk\r");						// DCU checksumming is off, so it won't change the pkt
#endif

	// Check switch inputs and generate command packets to motor controller
	while(TRUE){
		// Monitor switch positions & analog inputs
		if( events & EVENT_TIMER ) {
			events &= ~EVENT_TIMER;

			// Convert potentiometer and current monitoring inputs
			ADC12CTL0 |= ADC12SC;               	// Start A/D conversions. Reset automatically by hardware
			while ( ADC12CTL1 & ADC12BUSY );		// DCK: Busy wait for all conversions to complete TODO: replace with ADC ISR

			// TODO: Check for 5V pedal supply errors
			// TODO: Check for overcurrent errors on 12V outputs
			// Update motor commands based on pedal and slider positions
			process_pedal( ADC12MEM0, ADC12MEM1, ADC_MAX, motor_rpm );	// MVE: For now, pass constant regen as 3rd arg
			
			// Update current state of the switch inputs
			update_switches(&switches, &switches_diff);
			
			// Track current operating state
			switch(command.state){
				case MODE_OFF:
					if(switches & SW_IGN_ON) next_state = MODE_N;
					else next_state = MODE_OFF;
					P5OUT &= ~(LED_GEAR_ALL);
					break;
				case MODE_N:	// MVE: we get here briefly when the fuel door closes, and a few other cases
								// With the
#if 0
					if((switches & SW_MODE_R) && ((events & EVENT_SLOW) || (events & EVENT_REVERSE))) next_state = MODE_R;
					else if((switches & SW_MODE_B) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_B;
					else if((switches & SW_MODE_D) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_D;
					else
#endif
						 if (!(switches & SW_IGN_ON)) next_state = MODE_OFF;
					else if (switches & SW_CHARGE_CABLE) next_state = MODE_CHARGE;
//					else next_state = MODE_N;
					else next_state = MODE_D;			// Always proceed to MODE_D unless ignition is off or fuel door is open
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_3;
					break;
				case MODE_R:							// MVE: we never get here now
					if(switches & SW_MODE_N) next_state = MODE_N;
					else if((switches & SW_MODE_B) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_B;
					else if((switches & SW_MODE_D) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_D;
					else if (!(switches & SW_IGN_ON)) next_state = MODE_OFF;
					else if (switches & SW_CHARGE_CABLE) next_state = MODE_CHARGE;
					else next_state = MODE_R;
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_4;
					break;
				case MODE_B:
					if(switches & SW_MODE_N) next_state = MODE_N;
					else if((switches & SW_MODE_R) && ((events & EVENT_SLOW) || (events & EVENT_REVERSE))) next_state = MODE_R;
					else if((switches & SW_MODE_D) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_D;
					else if (!(switches & SW_IGN_ON)) next_state = MODE_OFF;
					else if (switches & SW_CHARGE_CABLE) next_state = MODE_CHARGE;
					else next_state = MODE_B;
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_2;
					break;
				case MODE_D:
#if 0
					if(switches & SW_MODE_N) next_state = MODE_N;
					else if((switches & SW_MODE_B) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_B;
					else if((switches & SW_MODE_R) && ((events & EVENT_SLOW) || (events & EVENT_REVERSE))) next_state = MODE_R;
					else
#endif
						if (!(switches & SW_IGN_ON)) next_state = MODE_OFF;
					else if (switches & SW_CHARGE_CABLE) next_state = MODE_CHARGE;
					else next_state = MODE_D;
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_1;
					break;
				case MODE_CHARGE:
					if(!(switches & SW_CHARGE_CABLE)) next_state = MODE_N;
					else if (!(switches & SW_IGN_ON)) next_state = MODE_OFF;
					else next_state = MODE_CHARGE;
					// Flash N LED in charge mode
					charge_flash_count--;
					P5OUT &= ~(LED_GEAR_4);
					if(charge_flash_count == 0){
						charge_flash_count = (CHARGE_FLASH_SPEED * 2);
						P5OUT |= LED_GEAR_3;
					}
					else if(charge_flash_count == CHARGE_FLASH_SPEED){
						P5OUT &= ~LED_GEAR_3;
					}
					break;
				default:
					next_state = MODE_OFF;
					break;
			}
			command.state = next_state;
				
			// Control brake lights
			if((switches & SW_BRAKE) || (events & EVENT_REGEN)) P1OUT |= BRAKE_OUT;
			else P1OUT &= ~BRAKE_OUT;
			
			// Control reversing lights
			if(command.state == MODE_R) P1OUT |= REVERSE_OUT;
			else P1OUT &= ~REVERSE_OUT;
			
			// Control CAN bus and pedal sense power
			if((switches & SW_IGN_ACC) || (switches & SW_IGN_ON)) {
			  	P1OUT |= CAN_PWR_OUT;
				P6OUT |= ANLG_V_ENABLE;
			}
			else{
				P1OUT &= ~CAN_PWR_OUT;
				P6OUT &= ~ANLG_V_ENABLE;
				events &= ~EVENT_CONNECTED;
				events |= EVENT_REQ_SLEEP;
			}

			// Control gear switch backlighting
//			if((switches & SW_IGN_ACC) || (switches & SW_IGN_ON))
			P5OUT |= LED_GEAR_BL;	// DCK: Supplies power to charger and BMS isolated TX
//			else P5OUT &= ~LED_GEAR_BL;
			
			// Control front panel fault indicator
			// MVE: this is handled with EVENT_FAULT now
//			if(switches & (SW_ACCEL_FAULT | SW_CAN_FAULT | SW_BRAKE_FAULT | SW_REV_FAULT))
//				LED_PORT &= ~LED_REDn;
//			else LED_PORT |= LED_REDn;
			

            if (chgr_events & CHGR_SOAKING) {
                if (++ chgr_soaking >= CHGR_EOC_SOAKT) {
                    chgr_events &= ~CHGR_SOAKING;
                    chgr_soaking = 0;
                    chgr_events |= CHGR_END_CHARGE;
                }
            }
			
			{	unsigned char ch;

				// Read incoming bytes from BMUs
				while (bmu_getByte(&ch)) {			// Get a byte from the BMU receive queue
					if (ch >= 0x80) {
						bmu_events |= BMU_BADNESS;
						bmu_badness = ch;
					} else {
						if (bmu_lastrxidx >= BMU_RX_BUFSZ) {
							fault();
							break;
						}
						bmu_lastrx[bmu_lastrxidx++] = ch; // !!! Need to check for buffer overflow
						if (ch == '\r')	{				// All BMU responses end with a carriage return
							bmu_events |= BMU_REC;		// We've received a BMU response
							break;
						}
					}
				}

				// Read incoming bytes from charger
				while (chgr_getByte(&ch)) {
					chgr_lastrx[chgr_lastrxidx++] = ch;
					if (chgr_lastrxidx == 12)	{	// All charger messages are 12 bytes long
						chgr_events |= CHGR_REC;	// We've received a charger response
						chgr_events &= ~CHGR_SENT;	// No longer unacknowledged
						break;
					}
				}
			}
			
		}		// End of processing EVENT_TIMER
		
		// Handle outgoing communications events (to motor controller)
		if(events & EVENT_COMMS){
			events &= ~EVENT_COMMS;

			// Update command state and override pedal commands if necessary
			if(switches & SW_IGN_ON){
				switch(command.state){
					case MODE_R:
					case MODE_D:
					case MODE_B:
						if(switches & SW_BRAKE){
							command.current = 0.0;	
							command.rpm = 0.0;
						}
						break;
					case MODE_CHARGE:
					case MODE_N:
					case MODE_START:
					case MODE_OFF:
					case MODE_ON:
					default:
						command.current = 0.0;
						command.rpm = 0.0;
						break;
				}
			}
			else{
				command.current = 0.0;
				command.rpm = 0.0;
			}

			// Transmit commands and telemetry
			if(events & EVENT_CONNECTED){
				// Blink activity LED
				// events |= EVENT_ACTIVITY;

				// Transmit drive command frame
				can.identifier = DC_CAN_BASE + DC_DRIVE;
				can.data.data_fp[1] = command.current;
				can.data.data_fp[0] = command.rpm;
				can_transmit();	
	
				// Transmit bus command frame
				can.identifier = DC_CAN_BASE + DC_POWER;
				can.data.data_fp[1] = command.bus_current;
				can.data.data_fp[0] = 0.0;
				can_transmit();
				
				// Transmit switch position/activity frame and clear switch differences variables
				can.identifier = DC_CAN_BASE + DC_SWITCH;
				can.data.data_u8[7] = command.state;
				can.data.data_u8[6] = command.flags;
				can.data.data_u16[2] = 0;
				can.data.data_u16[1] = 0;
				can.data.data_u16[0] = switches;
				can_transmit();
				
				// Transmit our ID frame at a slower rate (every 10 events = 1/second)
				comms_event_count++;
				if(comms_event_count == 10){
					comms_event_count = 0;
					can.identifier = DC_CAN_BASE;
					can.data.data_u8[7] = 'T';
					can.data.data_u8[6] = '0';
					can.data.data_u8[5] = '8';
					can.data.data_u8[4] = '6';
					can.data.data_u32[0] = DEVICE_SERIAL;
					can_transmit();		
				}
			}
		}
			
		// Process badness events before sending charger packets
		if (bmu_events & BMU_BADNESS) {
			bmu_events &= ~BMU_BADNESS;
			if (bmu_badness > 0x80)					// Simple algorithm:
				chgr_current = BMU_BYPASS_CAP - CHGR_CURR_DELTA;	// On any badness, cut back to
																	// what the BMUs can bypass
			if ((chgr_events & (CHGR_SOAKING | CHGR_END_CHARGE)) == 0) {
				// Send a voltage check for the current cell
//				sprintf(cmd, "%dsv\r", bmu_curr_cell);		// FIXME: send checksum
				// Note that sprintf uses a lot of stack, and seems to prepend a leading zero
				unsigned char cmd[8];
				makeVoltCmd(cmd, bmu_curr_cell);			// cmd := "XXsv\r"
				bmu_sendPacket(cmd);
				++bmu_curr_cell;
				if (bmu_curr_cell > NUMBER_OF_BMUS)
					bmu_curr_cell = 1;
			}
		}
		
		// Send voltage request if required for min/max while driving or max V when charging
		if (bmu_events & BMU_VOLTREQ) {
			bmu_events &= ~BMU_VOLTREQ;
			unsigned char cmd[8];
			makeVoltCmd(cmd, bmu_curr_cell);			// cmd := "XXsv\r"
			bmu_sendPacket(cmd);
		}

		// MVE: send packets to charger
		// Using switches gives a small amount of debouncing
		if (events & EVENT_CHARGER) {
			events &= ~EVENT_CHARGER;
			events |= EVENT_ACTIVITY;

			// Charger is on the UART in UCI0
			chgr_txbuf[0] = 0x18;					// Send 18 06 E5 F4 0V VV 00 WW 0X 00 00 00
			chgr_txbuf[1] = 0x06;					//	where VVV is the voltage in tenths of a volt,
			chgr_txbuf[2] = 0xE5;					//	WW is current limit in tenths of an amp, and
			chgr_txbuf[3] = 0xF4;					//	X is 0 to turn charger on
			chgr_txbuf[4] = CHGR_VOLT_LIMIT >> 8;
			chgr_txbuf[5] = CHGR_VOLT_LIMIT & 0xFF;
			chgr_txbuf[6] = 0;
			if (chgr_events & CHGR_END_CHARGE) {
				chgr_txbuf[7] = 0;					// Request no current now
				chgr_txbuf[8] = 1;					// Turn off charger
			} else if (chgr_events & CHGR_SOAKING) {
				chgr_txbuf[7] = CHGR_SOAK_CURR;		// Soak at the soak current level (< bypass capacity)
				chgr_txbuf[8] = 0;					// Keep charger on
			} else {
				chgr_current += CHGR_CURR_DELTA;	// Increase charger current by the fixed amount
				if (chgr_current > CHGR_CURR_LIMIT)
					chgr_current = CHGR_CURR_LIMIT;
				chgr_txbuf[7] = chgr_current;
				chgr_txbuf[8] = 0;
			}
			chgr_txbuf[9] = 0; chgr_txbuf[10] = 0; chgr_txbuf[11] = 0;
			chgr_sendPacket(chgr_txbuf);
			chgr_lastrxidx = 0;						// Expect receive packet in response
		}

		if (chgr_events & CHGR_REC) {
			chgr_events &= ~CHGR_REC;
			chgr_lastrxidx = 0;						// Ready for next charger response to overwrite this one
													//	(starting next timer interrupt)
			// Do something with the packet
#if 1
			{
				int nVolt = (chgr_lastrx[4] << 8) + chgr_lastrx[5];
				szChgrVolt[5] = nVolt / 1000 + '0';				// Voltage hundreds
				szChgrVolt[6] = (nVolt % 1000) / 100 + '0';		//	tens
				szChgrVolt[7] = (nVolt % 100) / 10 + '0';		//	units
				szChgrVolt[10] = (chgr_lastrx[7] / 10) + '0';	// Current units
				szChgrVolt[12] = (chgr_lastrx[7] % 10) + '0';	//	tenths
				bmu_sendPacket(szChgrVolt); // Send as comment packet on BMU channel for debugging
			}
#endif
		}
		
		if (bmu_events & BMU_REC) {
			bmu_events &= ~BMU_REC;
			bmu_lastrxidx = 0;						// Ready for next BMU response to overwrite this one
													//	(starting next timer interrupt)

#if USE_CKSUM
			{	unsigned char sum = 0, j = 0;
				while (bmu_lastrx[j] != '\r')
					sum ^= bmu_lastrx[j++];
				if (sum != 0) {
					// Checksum error; set the error LED and resend the last command
					fault();
					bmu_resendLastPacket();				// Resend
					goto no_bmu_received;			// Don't process this packet
				}
			}
#endif

			if ((chgr_events & CHGR_SOAKING) == 0) {
				// Check for a voltage response
				// Expecting \123:1234 V  ret
				//           0   45    10 11  (note space before the 'V'
				if (bmu_lastrx[0] == '\\' &&
					bmu_lastrx[4] == ':' &&
					bmu_lastrx[10] == 'V') {
						int bmu_id = 100 * (bmu_lastrx[1] - '0') + (bmu_lastrx[2] - '0') * 10 +
							bmu_lastrx[3] - '0';
						if (bmu_id == bmu_curr_cell) {
							bmu_events &= ~BMU_SENT;		// Call this valid and no longer unacknowledged
							unsigned int rxvolts =
#if 0
								(bmu_lastrx[5] - '0') * 100 +
#else
								// The *50 and << 1 below are to work around a mspgcc bug! See
								// http://sourceforge.net/tracker/index.php?func=detail&aid=2082985&group_id=42303&atid=432701
								(((bmu_lastrx[5] - '0') * 50) << 1) +
#endif
								(bmu_lastrx[6] - '0') * 10 +
								(bmu_lastrx[7] - '0');
							// We expect voltage responses during charging and driving; split the logic here
							if (command.state == MODE_CHARGE) {
								if (rxvolts < 359)
									// This cell is not bypassing. So the string of cells known to be in bypass
									// is zero length. Flag this
									first_bmu_in_bypass = -1;
								else {
									// This cell is in bypass. Check if the first bmu in bypass is the next one
									int next_bmu_id = bmu_id+1;
									if (next_bmu_id > NUMBER_OF_BMUS)
										next_bmu_id = 1;
									if (next_bmu_id == first_bmu_in_bypass) {
										// We have detected all cells in bypass. Now we enter the soak phase
										// The idea is to allow the last cell to have gone into bypass some
										// time to stay at that level and balance with the others
										chgr_events |= CHGR_SOAKING;
									}
									else {
										if (first_bmu_in_bypass == -1)
										// This cell is in bypass; we must be starting a new string of bypassed BMUs
										first_bmu_in_bypass = bmu_id;
									}
								}
							} else {
								// Not charging: driving. We use the voltage measurements to find the min and
								//	max cell voltages
								// Get the whole 4-digit number
								rxvolts *= 10; rxvolts += bmu_lastrx[8] - '0';
								if (rxvolts < bmu_min_mV) {
									bmu_min_mV = rxvolts;
									bmu_min_id = bmu_id;
								}
								if (rxvolts > bmu_max_mV) {
									bmu_max_mV = rxvolts;
									bmu_max_id = bmu_id;
								}
								if (bmu_id >= NUMBER_OF_BMUS) {
									// We have the min and max information. Send a CAN packet so the telemetry
									//	software can display them. Use CAN id 0x266, as the IQcell BMS would
									can.identifier = 0x266;
									can.data.data_u16[0] = bmu_min_mV;
									can.data.data_u16[1] = bmu_max_mV;
									can.data.data_u16[2] = bmu_min_id;
									can.data.data_u16[3] = bmu_max_id;
									can_transmit();

									// Reset the min/max data
									bmu_min_mV = 9999;	bmu_max_mV = 0;
									bmu_min_id = 0;		bmu_max_id = 0;
								}
						}
					}
					// Move to the next BMU (driving or charging)
					if (++bmu_curr_cell > NUMBER_OF_BMUS)
						bmu_curr_cell = 1;
					bmu_events |= BMU_VOLTREQ;			// Schedule another voltage request
					
				}
			}
		}
#if USE_CKSUM
no_bmu_received:
#endif

		// Check for CAN packet reception
		if((P2IN & CAN_INTn) == 0x00){
			// IRQ flag is set, so run the receive routine to either get the message, or the error
			can_receive();
			// Check the status
			if(can.status == CAN_OK){
				// We've received a packet, so must be connected to something
					// MVE TODO: Distinguish between connecting to other DCU and to motor controller?
				events |= EVENT_CONNECTED;
				// Process the packet
				switch(can.identifier){
					case MC_CAN_BASE + MC_VELOCITY:
						// Update speed threshold event flags
						if(can.data.data_fp[1] > ENGAGE_VEL_F) events |= EVENT_FORWARD;
						else events &= ~EVENT_FORWARD;
						if(can.data.data_fp[1] < ENGAGE_VEL_R) events |= EVENT_REVERSE;
						else events &= ~EVENT_REVERSE;
						if((can.data.data_fp[1] >= ENGAGE_VEL_R) && (can.data.data_fp[1] <= ENGAGE_VEL_F)) events |= EVENT_SLOW;
						else events &= ~EVENT_SLOW;
						motor_rpm = can.data.data_fp[0];		// DCK: Was [1] for m/s (confirmed with TJ)
						gauge_tach_update( motor_rpm );
						break;
					case MC_CAN_BASE + MC_I_VECTOR:
						// Update regen status flags
						if(can.data.data_fp[0] < REGEN_THRESHOLD) events |= EVENT_REGEN;
						else events &= ~EVENT_REGEN;
						break;
					case MC_CAN_BASE + MC_TEMP1:
						// Update data for temp gauge
						controller_temp = can.data.data_fp[1];
						motor_temp = can.data.data_fp[0];
						gauge_temp_update( motor_temp, controller_temp );
						break;
					case MC_CAN_BASE + MC_BUS:
						// Update battery voltage and current for fuel and power gauges
						battery_voltage = can.data.data_fp[0];
						battery_current = can.data.data_fp[1];
						gauge_power_update( battery_voltage, battery_current );
						gauge_fuel_update( battery_voltage );
						break;
/*					case EL_CAN_ID_H:		// MVE: shouldn't this be EL_CAN_ID_L?
						// MVE: update some globals with charger info
						charger_volt = can.data.data_u16[0];
						charger_curr = can.data.data_u16[1];
						charger_status = can.data.data_u8[4];
						break;
*/				}
			}
			if(can.status == CAN_RTR){
				// Remote request packet received - reply to it
				switch(can.identifier){
					case DC_CAN_BASE:
						can.data.data_u8[3] = 'T';
						can.data.data_u8[2] = '0';
						can.data.data_u8[1] = '8';
						can.data.data_u8[0] = '6';
						can.data.data_u32[1] = DEVICE_SERIAL;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_DRIVE:
						can.data.data_fp[1] = command.current;
						can.data.data_fp[0] = command.rpm;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_POWER:
						can.data.data_fp[1] = command.bus_current;
						can.data.data_fp[0] = 0.0;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_SWITCH:
						can.data.data_u8[7] = command.state;
						can.data.data_u8[6] = command.flags;
						can.data.data_u16[2] = 0;
						can.data.data_u16[1] = 0;
						can.data.data_u16[0] = switches;
						can_transmit();
						break;
				}
			}
			if (can.status == CAN_ERROR) {
				if (can.identifier == 0x0002) {
					// Wake up CAN controller
					can_wake();
				}
				// Comment out for now: will always get CAN errors
				// fault();		// MVE: see the CAN error in fault light
			}
		}
		
		// Check sleep mode requests
/*		if(events & EVENT_REQ_SLEEP){
			events &= ~EVENT_REQ_SLEEP;
			can_abort_transmit();
			can_sleep();
			P4OUT &= ~LED_PWM;
			__bis_SR_register(LPM3_bits);     // Enter LPM3
		}*/
	} // End of while(True) do
	
	// Will never get here, keeps compiler happy
	return(1);
} // End of main


/*
 * Delay function
 */
static void __inline__ brief_pause(register unsigned int n)
{
#ifndef __ICC430__
    __asm__ __volatile__ (
		"1: \n"
		" dec	%[n] \n"
		" jne	1b \n"
        : [n] "+r"(n));
#endif
}

/*
 * Initialise clock module
 *	- Setup MCLK, ACLK, SMCLK dividers and clock sources
 *	- ACLK  = 0
 *	- MCLK  = 16 MHz internal oscillator
 *	- SMCLK = 16 MHz internal oscillator
 *
 * Note: We can also use the 2, 4, 8 or 16MHz crystal clock output from the MCP2515 CAN controller, which
 *       is a more accurate source than the internal oscillator.  However, using the internal
 *       source makes using sleep modes for both the MSP430 and the MCP2515 much simpler.
 */
void clock_init( void )
{
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
//	BCSCTL1 = 0x8F;			// FIXME!
//	DCOCTL = 0x83;
}

/*
 * Initialise I/O port directions and states
 *	- Drive unused pins as outputs to avoid floating inputs
 *
 */
void io_init( void )
{
	P1OUT = 0x00;
	P1DIR = BRAKE_OUT | REVERSE_OUT | CAN_PWR_OUT | P1_UNUSED;
	
	P2OUT = 0x00;
	P2DIR = P2_UNUSED;

//	P3OUT = CAN_CSn | EXPANSION_TXD | LED_REDn | LED_GREENn;
	P3OUT = CAN_CSn | CHARGER_TXD | BMS_TXD;
//	P3DIR = CAN_CSn | CAN_MOSI | CAN_SCLK | EXPANSION_TXD | LED_REDn | LED_GREENn | P3_UNUSED;
	P3DIR = CAN_CSn | CAN_MOSI | CAN_SCLK | CHARGER_TXD | BMS_TXD | P3_UNUSED;
	
	P4OUT = LED_PWM | LED_REDn | LED_GREENn;
	P4DIR = GAUGE_1_OUT | GAUGE_2_OUT | GAUGE_3_OUT | GAUGE_4_OUT | LED_PWM | LED_REDn | LED_GREENn;
	
	P5OUT = 0x00;
	P5DIR = LED_FAULT_1 | LED_FAULT_2 | LED_FAULT_3 | LED_GEAR_BL | LED_GEAR_4 | LED_GEAR_3 | LED_GEAR_2 | LED_GEAR_1 | P5_UNUSED;
	
	P6OUT = 0x00;
	P6DIR = ANLG_V_ENABLE | P6_UNUSED;
	
	// Initialise charger and BMS UARTs
	P3SEL |= CHARGER_TXD | CHARGER_RXD | BMS_TXD | BMS_RXD;// Set pins to peripheral function, not GPIO
	UCA0CTL1 |= UCSSEL_2;					// SMCLK
	UCA1CTL1 |= UCSSEL_2;					// SMCLK
	// Baud rate charger 2400 b/s, 16000 / 2.4 / 16 = 416.667 = 0x01A0 with 0xB1 for the fractional part
	UCA0BR1=0x01; UCA0BR0=0xA0; UCA0MCTL=0xB1;
	// Baud rate BMS     9600 b/s, 16000 / 9.6 / 16 = 104.167 = 0x0068 with 0x31 for the fractional part
	UCA1BR1=0x00; UCA1BR0=0x68; UCA1MCTL=0x31;
	UCA0CTL1 &= ~UCSWRST;					// **Initialize USCI state machine**
	UCA1CTL1 &= ~UCSWRST;
	IE2 |= UCA0RXIE;						// Enable charger RX interrupt
	UC1IE |= UCA1RXIE;						// Enable BMS RX interrupt
}


/*
 * Initialise Timer A
 *	- Provides timer tick timebase at 100 Hz
 */
void timerA_init( void )
{
	TACTL = TASSEL_2 | ID_3 | TACLR;			// MCLK/8, clear TAR
	TACCR0 = (INPUT_CLOCK/8/TICK_RATE);			// Set timer to count to this value = TICK_RATE overflow
	TACCTL0 = CCIE;								// Enable CCR0 interrrupt
	TACTL |= MC_1;								// Set timer to 'up' count mode
}


/*
 * Initialise Timer B
 *	- Provides PWM and pulse outputs for gauges
 *	- 10000Hz timer ISR
 *	- With 16MHz clock/8 = 200 count resolution on PWM
 *
 */
void timerB_init( void )
{
	TBCTL = TBSSEL_2 | ID_3 | TBCLR;			// MCLK/8, clear TBR
	TBCCR0 = GAUGE_PWM_PERIOD;					// Set timer to count to this value
	TBCCR2 = 0;									// Gauge 3
	TBCCTL2 = OUTMOD_7;
	TBCCR1 = 0;									// Gauge 4
	TBCCTL1 = OUTMOD_7;
	P4SEL |= GAUGE_3_OUT | GAUGE_4_OUT;			// PWM -> output pins for fuel and temp gauges (tacho and power are software freq outputs)
	TBCCTL0 = CCIE;								// Enable CCR0 interrupt
	TBCTL |= MC_1;								// Set timer to 'up' count mode
}

/*
 * Initialise A/D converter
 */
void adc_init( void )
{
	// Enable A/D input channels											
	P6SEL |= ANLG_SENSE_A | ANLG_SENSE_B | ANLG_SENSE_C | ANLG_SENSE_V | ANLG_BRAKE_I | ANLG_REVERSE_I | ANLG_CAN_PWR_I;
	// Turn on ADC12, set sampling time = 256 ADCCLK, multiple conv, start internal 2.5V reference
	ADC12CTL0 = ADC12ON | SHT0_8 | SHT1_8 | MSC | REFON | REF2_5V;	
	// Use sampling timer, ADCCLK = MCLK/4, run a single sequence per conversion start
	ADC12CTL1 = ADC12SSEL_2 | ADC12DIV_3 | SHP | CONSEQ_1;
	// Map conversion channels to input channels & reference voltages
	ADC12MCTL0 = INCH_3 | SREF_1;			// Analog A
	ADC12MCTL1 = INCH_2 | SREF_1;			// Analog B
	ADC12MCTL2 = INCH_1 | SREF_1;			// Analog C
	ADC12MCTL3 = INCH_4 | SREF_1;			// Analog V Supply
	ADC12MCTL4 = INCH_5 | SREF_1;			// Brake light current
	ADC12MCTL5 = INCH_6 | SREF_1;			// Reverse light current
	ADC12MCTL6 = INCH_7 | SREF_1 | EOS;		// CAN Bus current / End of sequence
	// Enable interrupts on final conversion in sequence
//	ADC12IE = BIT6;	// Some bug with this - keep using polling for the moment
	// Enable conversions
	ADC12CTL0 |= ENC;											
}

/*
 * Timer B CCR0 Interrupt Service Routine
 *	- Interrupts on Timer B CCR0 match at GAUGE_FREQUENCY (10kHz)
 */
interrupt(TIMERB0_VECTOR) timer_b0(void)
{
	static unsigned int gauge_count;
	static unsigned int gauge1_on, gauge1_off;
	static unsigned int gauge2_on, gauge2_off;
	
	// Toggle gauge 1 & 2 pulse frequency outputs
	if(gauge_count == gauge1_on){
		P4OUT |= GAUGE_1_OUT;
		gauge1_on = gauge_count + gauge.g1_count;
		gauge1_off = gauge_count + (gauge.g1_count >> 2);
	}
	if(gauge_count == gauge1_off){
		P4OUT &= ~GAUGE_1_OUT;
	}

	if(gauge_count == gauge2_on){
		P4OUT |= GAUGE_2_OUT;
		gauge2_on = gauge_count + gauge.g2_count;
		gauge2_off = gauge_count + (gauge.g2_count >> 2);
	}
	if(gauge_count == gauge2_off){
		P4OUT &= ~GAUGE_2_OUT;
	}

	// Update pulse output timebase counter
	gauge_count++;
	
	// Update outputs if necessary
	if(events & EVENT_GAUGE1){
		events &= ~EVENT_GAUGE1;
	}
	if(events & EVENT_GAUGE2){
		events &= ~EVENT_GAUGE2;
	}
	if(events & EVENT_GAUGE3){
		events &= ~EVENT_GAUGE3;
		TBCCR2 = gauge.g3_duty;		
	}
	if(events & EVENT_GAUGE4){
		events &= ~EVENT_GAUGE4;
		TBCCR1 = gauge.g4_duty;		
	}	
}

/*
 * Timer A CCR0 Interrupt Service Routine
 *	- Interrupts on Timer A CCR0 match at 100Hz
 *	- Sets Time_Flag variable
 */

interrupt(TIMERA0_VECTOR) timer_a0(void)
{
	static unsigned char comms_count = COMMS_SPEED;
	static unsigned char charger_count = CHARGER_SPEED;		// MVE
	static unsigned char activity_count;
	static unsigned char fault_count;

	
	// Trigger timer based events
	events |= EVENT_TIMER;	


	// Trigger comms events (command packet transmission)
	comms_count--;
	if( comms_count == 0 ){
		comms_count = COMMS_SPEED;
		events |= EVENT_COMMS;
	}
	// MVE: Trigger charger events (command packet transmission)
	if((command.state == MODE_CHARGE) && (--charger_count == 0) ){
		charger_count = CHARGER_SPEED;
		events |= EVENT_CHARGER;
	}
	
	if (chgr_events & CHGR_SENT) {
		if (--chgr_sent_timeout == 0) {
			fault();				// Turn on fault LED (eventually)
			chgr_resendLastPacket();	// Resend; will loop until a complete packet is recvd
		}
	}
	if (bmu_events & BMU_SENT) {
		if (--bmu_sent_timeout == 0) {
			fault();
			bmu_resendLastPacket();				// Resend
		}
	}

	// Check for CAN activity events and blink LED
	if(events & EVENT_ACTIVITY){
		events &= ~EVENT_ACTIVITY;
		activity_count = ACTIVITY_SPEED;
		LED_PORT &= ~LED_GREENn;
	}
	if( activity_count == 0 ){
		LED_PORT |= LED_GREENn;
	}
	else{
		activity_count--;
	}

	// MVE: similarly for FAULT LED
	if (events & EVENT_FAULT) {
		events &= ~EVENT_FAULT;
		fault_count = FAULT_SPEED;
		LED_PORT &= ~LED_REDn;
	}
	if ( fault_count == 0)
		LED_PORT |= LED_REDn;
	else
		--fault_count;

}

/*
 * Collect switch inputs from hardware and fill out current state, and state changes
 *	- Inverts active low switches so that all bits in register are active high
 */
void update_switches( unsigned int *state, unsigned int *difference)
{
	unsigned int old_switches;
	
	// Save state for difference tracking
	old_switches = *state;

	// Import switches into register

#if 0
	if(P2IN & IN_GEAR_4) *state |= SW_MODE_R;
	else *state &= ~SW_MODE_R;
	
	if(P2IN & IN_GEAR_3) *state |= SW_MODE_N;
	else *state &= ~SW_MODE_N;

	if(P2IN & IN_GEAR_2) *state |= SW_MODE_B;
	else *state &= ~SW_MODE_B;

	if(P2IN & IN_GEAR_1) *state |= SW_MODE_D;
	else *state &= ~SW_MODE_D;
#endif
	
	if(P1IN & IN_IGN_ACCn) *state &= ~SW_IGN_ACC;
	else *state |= SW_IGN_ACC;
	
	if(P1IN & IN_IGN_ONn) *state &= ~SW_IGN_ON;
	else *state |= SW_IGN_ON;

	if(P1IN & IN_IGN_STARTn) *state &= ~SW_IGN_START;
	else *state |= SW_IGN_START;

	if(P1IN & IN_BRAKEn) *state &= ~SW_BRAKE;
	else *state |= SW_BRAKE;

	if(P1IN & IN_FUEL) *state |= SW_CHARGE_CABLE;
	else *state &= ~SW_CHARGE_CABLE;

	// Update changed switches
	*difference = *state ^ old_switches;	
}

/*
 * ADC12 Interrupt Service Routine
 *	- Interrupts on channel 6 conversion (end of sequence)
 */
 /*
interrupt(ADC12_VECTOR) adc_isr(void)
{
	// Read data to clear ISR flag
	ADC12IFG = 0x0000;

	// Trigger adc events
	events |= EVENT_ADC;
}
*/


