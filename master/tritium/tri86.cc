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
//#include <signal.h>
#include <intrinsics.h>
#include "tri86.h"
#include "can.h"
#include "usci.h"
#include "pedal.h"
#include "gauge.h"
#include "bms.h"
#include "charger.h"
#include "pid.h"

#ifdef __ICC430__								// MVE: attempt to make the source code more IAR friendly, in case
#define __inline__								//	press F7, and so that "go to definition of X" works better
#define __volatile__
#define interrupt(x) void
void __eint();
void __dint();
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

// Data from motor controller
float motor_rpm = 0.0;
float motor_temp = 0.0;
float controller_temp = 0.0;
float battery_voltage = 0.0;
float battery_current = 0.0;

unsigned int uChgrCurrLim = CHGR_CURR_LIMIT;	// Default to maximum current limit. Integer tenths of
												//	an ampere, e.g. 55 means 5.5 A.
bool bDCUb;										// True if we are DCU-B; false if we are DCU-A
unsigned char statusB = 0xC8;					// Status from DCU-B, initially assume a bad case
												// i.e. stress 8 with a comms error


void fault() {
	events |= EVENT_FAULT;				// Breakpoint this instruction and use stack backtrace
										//	(but beware the compiler may well inline it)
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
	// Debug
	unsigned int i;

	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;

	// Initialise I/O ports, including enabling the 5V supply for the pedal etc.
	io_init();

	// Wait approx 20 ms for clocks etc to stabilise, and power to come up for external devices
	// MSP430 starts at 1.8V, CAN controller needs 3.3V, pedal needs 5V
	for(i = 0; i < 20; i++) brief_pause(5333); // 20 ms

	// Initialise clock module - internal oscillator
	clock_init();

	// Initialise SPI port for CAN controller (running with SMCLK)
	usci_init(0);

	// Reset CAN controller and initialise
	// This also changes the clock output from the MCP2515, but we're not using it in this software
	can_init( CAN_BITRATE_500 );
	events |= EVENT_CONNECTED;

	// Initialise Timer A (10ms timing ticks)
	timerA_init();

	// Initialise Timer B (gauge outputs PWM / pulses)
	timerB_init();

	// Initialise A/D converter for potentiometer and current sense inputs
	// Includes 20 ms wait for Vref cap to charge
	adc_init();

	// Initialise switch & encoder positions
	update_switches(&switches, &switches_diff);

	// Initialise command state
	command.rpm = 0.0;
	command.current = 0.0;
	command.bus_current = 1.0;
	command.flags = 0x00;
	command.state = MODE_OFF;

	// Init gauges
	gauge_init();

	// Init BMS and charger
	bms_init();
	chgr_init();
	// Delay 3 seconds so the reset problem with BMU 6 has a chance to propagate to the end f the BMU
	//	string. Otherwise, the 'k' and 0K commands won't work from about BMU 14 onwards
	for (i = 0; i < 3000; i++) {
		readChargerBytes();		// So we update chgr_rx_timer
		brief_pause(5333);		// 3000 ms = 3 seconds
	}

	// Enable interrupts
	__eint();

	// Convert potentiometer and current monitoring inputs
	ADC12CTL0 |= ADC12SC;               	// Start A/D conversions. Reset automatically by hardware
	while ( ADC12CTL1 & ADC12BUSY );		// DCK: Busy wait for all conversions to complete TODO: replace with ADC ISR

	// Detect presence of (unpressed) pedal to determine if we are DCU-A or DCU-B
	bDCUb = (ADC12MEM0 < PEDAL_ERROR_MIN) || (ADC12MEM0 > PEDAL_TRAVEL_MIN*2-PEDAL_ERROR_MIN);
	if (bDCUb)		// If position error, i.e. pedal not present
		command.flags |= FAULT_NO_PEDAL;

	// Check switch inputs and generate command packets to motor controller
	// and control charging while monitoring BMUs.
	while(TRUE){
		// Process CAN transmit queue
		can_transmit();

		// Monitor switch positions & analog inputs
		if( events & EVENT_TIMER ) { // Every 10 ms
			events &= ~EVENT_TIMER;

			// Convert potentiometer and current monitoring inputs
			ADC12CTL0 |= ADC12SC;               	// Start A/D conversions. Reset automatically by hardware
			while ( ADC12CTL1 & ADC12BUSY );		// DCK: Busy wait for all conversions to complete TODO: replace with ADC ISR

			// TODO: Check for 5V pedal supply errors
			// TODO: Check for overcurrent errors on 12V outputs
			// Update motor commands based on pedal and slider positions
			if (!bDCUb)
				// MVE: For now, pass constant regen as 3rd arg (like regen pot at max)
				process_pedal( ADC12MEM0, ADC12MEM1, ADC_MAX, motor_rpm );

			// Update current state of the switch inputs
			update_switches(&switches, &switches_diff);

			// Track current operating state
			switch(command.state){
				case MODE_OFF:
					P5OUT &= ~(LED_GEAR_ALL);		// Stop indicating drive mode or charge mode
													// Stop requesting brakelights if we're DCU-A
													// Stop indicating charge mode if we're DCU-B
					P1OUT &= ~CHG_CONT_OUT;			// Turn off our charger contactors
					P1OUT &= ~BRAKE_OUT;			// Turn off traction contactors if we're DCU-A
													// Turn off brake lights if we're DCU-B

					if (switches & SW_CRASH)				// if we've crashed
						next_state = MODE_OFF;				// Stay in the OFF mode
					else if ((switches & SW_CHARGE_CABLE)  	// else if our charge cable is present
					|| (chgr_rx_timer > 0)) {				// or we received data from our charger
						next_state = MODE_CHARGE;			// Go to CHARGE mode
						if (bDCUb)							// If DCU-B
							P5OUT |= LED_GEAR_3;			// tell DCU-A that we're in charge mode
															// so it can inhibit traction
						P1OUT |= CHG_CONT_OUT;				// Turn on our charge contactor
						bmu_changeDirection(TRUE);			// Tell BMUs direction of current flow
						chgr_start();						// Start the charge controller (PID loop)
						P5OUT |= LED_GEAR_2;				// Indicate we're in charge mode
					}
					else if ((!bDCUb)						// else if we're DCU-A
					&& !(switches & SW_BRAKE) 				// and DCU-B is not in charge mode
					&& (switches & SW_IGN_START)) {			// and latched start is on
						next_state = MODE_D;				// Go to drive mode
						P1OUT |= BRAKE_OUT;					// Turn on traction contactors
						P5OUT |= LED_GEAR_1;				// Indicate we're in drive mode
					}
					else
						next_state = MODE_OFF;
					break;
				case MODE_D:
					if ((switches & SW_CRASH) 				// if we've crashed
					|| (switches & SW_CHARGE_CABLE) 		// or our charge cable is present
					|| (chgr_rx_timer > 0)					// or we received data from our charger
					|| !(switches & SW_IGN_START)) {		// or latched start is off
						next_state = MODE_OFF;				// Go to OFF mode
					}
					else
						next_state = MODE_D;
					break;
				case MODE_CHARGE:
					if ((switches & SW_CRASH)  				// if we've crashed
					|| !((switches & SW_CHARGE_CABLE) 		// or we have neither charge cable present
					||   (chgr_rx_timer > 0))) {			// nor received data from our charger
						next_state = MODE_OFF;				// Go to OFF mode
						if (bDCUb)							// If DCU-B
							P5OUT &= ~LED_GEAR_3;			// tell DCU-A that we're not in charge mode
															// so it can allow traction
						bmu_changeDirection(FALSE); 		// Tell BMUs direction of current
						chgr_stop();						// Stop the charge controller (PID loop)
					}
					else
						next_state = MODE_CHARGE;
					break;
				default:
					next_state = MODE_OFF;
					break;
			}

			command.state = next_state;

			// Control brake lights
			if (bDCUb) {
				// If we're DCU-B
				if((switches & SW_BRAKE) || (events & EVENT_REGEN)) // If we're in heavy regen or DCU-B is requesting
					P1OUT |= BRAKE_OUT;		// Turn on brake lights
				else P1OUT &= ~BRAKE_OUT;
			}
			else {
				// else we're DCU-A
				if (events & EVENT_REGEN)   // If we're in heavy regen
					P5OUT |= LED_GEAR_3;	// Request DCU-B to turn on brake lights
				else
					P5OUT &= ~LED_GEAR_3;
			}

			// Control CAN bus and pedal sense power
			if((switches & SW_IGN_ACC) || (switches & SW_IGN_ON) || (switches & SW_IGN_START)) {
			  	P1OUT |= CAN_PWR_OUT;
				P6OUT |= ANLG_V_ENABLE;
			}
			else {
				P1OUT &= ~CAN_PWR_OUT;
				P6OUT &= ~ANLG_V_ENABLE;
				events &= ~EVENT_CONNECTED;
				events |= EVENT_REQ_SLEEP;
			}

			chgr_timer(command.state == MODE_CHARGE);
			bmu_timer();

		} // End of if( events & EVENT_TIMER ) // Every 10 ms

		readBMUbytes();
		readChargerBytes();

		// Handle outgoing communications events (to motor controller)
		if ((events & EVENT_COMMS) && !bDCUb) { 	// Every 100 ms
			events &= ~EVENT_COMMS;

			// Transmit commands and telemetry
			if(events & EVENT_CONNECTED){
				// Blink activity LED
				// events |= EVENT_ACTIVITY;

				// Queue drive command frame
				can_push_ptr->identifier = DC_CAN_BASE + DC_DRIVE;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_fp[1] = command.current;
				can_push_ptr->data.data_fp[0] = command.rpm;
				can_push();
#if 0
				// Queue bus command frame
				can_push_ptr->identifier = DC_CAN_BASE + DC_POWER;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_fp[1] = command.bus_current;
				can_push_ptr->data.data_fp[0] = 0.0;
				can_push();

				// Queue switch position/activity frame and clear switch differences variables
				can_push_ptr->identifier = DC_CAN_BASE + DC_SWITCH;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_u8[7] = command.state;
				can_push_ptr->data.data_u8[6] = command.flags;
				can_push_ptr->data.data_u16[2] = 0;
				can_push_ptr->data.data_u16[1] = 0;
				can_push_ptr->data.data_u16[0] = switches;
				can_push();
#endif
				// Queue our ID frame at a slower rate (every 10 events = 1/second)
				comms_event_count++;
				if(comms_event_count == 10){
					comms_event_count = 0;
					can_push_ptr->identifier = DC_CAN_BASE;
					can_push_ptr->status = 8;
					// Note this same ID is transmitted in reverse in response to RTR packets
					can_push_ptr->data.data_u8[7] = 'T';
					can_push_ptr->data.data_u8[6] = '0';
					can_push_ptr->data.data_u8[5] = '8';
					can_push_ptr->data.data_u8[4] = '6';
					can_push_ptr->data.data_u32[0] = DEVICE_SERIAL;
					can_push();
				}
			} // End of if(events & EVENT_CONNECTED)
		} // End of if(events & EVENT_COMMS) // Every 100 ms

		// Check for CAN packet reception
		if((P2IN & CAN_INTn) == 0x00){
			// IRQ flag is set, so run the receive routine to either get the message, or the error
			can_receive();
			// Check the status
			if(can.status == CAN_OK){
				// We've received a packet, so must be connected to something
				events |= EVENT_CONNECTED;
				// Process the packet
				if (bDCUb) {
					switch(can.identifier) {
					case DC_CAN_BASE + DC_CHGR_LIM:
						uChgrLim = can.data.data_u16[0];
						break;
					}
				} else {										// DCU-A
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
						if (command.state == MODE_D)			// Tacho used for charge current when charging
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
						gauge_fuel_update( battery_voltage );
						break;
					case DC_CAN_BASE + DC_BMUB_STATUS:
					  	statusB = can.data.data_u8[0];		// Save BMS status from DCUB
						break;
					case DC_CAN_BASE + DC_CHGR_CURR:
						uChgrCurrA = can.data.data_u16[0];	// Save charger B actual current
						break;
				    }
				}

				// DCU A or B
				switch (can.identifier) {
					case DC_CAN_BASE + DC_BOOTLOAD:
						// Switch to bootloader
						if (		can.data.data_u8[0] == 'B' && can.data.data_u8[1] == 'O' && can.data.data_u8[2] == 'O' && can.data.data_u8[3] == 'T'
								&&	can.data.data_u8[4] == 'L' && can.data.data_u8[5] == 'O' && can.data.data_u8[6] == 'A' && can.data.data_u8[7] == 'D' )
						{
							WDTCTL = 0x00;	// Force watchdog reset
						}
						break;
					case CHGR_LIM:
					  	// NOTE: at present, this gets overridden by the charger limit pot setting
					  	// The filter and mask for this don't seem to work anyway
						uChgrCurrLim = can.data.data_u16[0];
						break;
				}

			} // End of if(can.status == CAN_OK)
			if ((can.status == CAN_RTR) && !bDCUb) {
				// Remote request packet received - reply to it
				switch(can.identifier){
					case DC_CAN_BASE:
						can_push_ptr->identifier = can.identifier;
						can_push_ptr->status = 8;
						// Note this same ID is transmitted in reverse periodically
						can_push_ptr->data.data_u8[3] = 'T';
						can_push_ptr->data.data_u8[2] = '0';
						can_push_ptr->data.data_u8[1] = '8';
						can_push_ptr->data.data_u8[0] = '6';
						can_push_ptr->data.data_u32[1] = DEVICE_SERIAL;
						can_push();
						break;
					case DC_CAN_BASE + DC_DRIVE:
						can_push_ptr->identifier = can.identifier;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_fp[1] = command.current;
						can_push_ptr->data.data_fp[0] = command.rpm;
						can_push();
						break;
					case DC_CAN_BASE + DC_POWER:
						can_push_ptr->identifier = can.identifier;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_fp[1] = command.bus_current;
						can_push_ptr->data.data_fp[0] = 0.0;
						can_push();
						break;
					case DC_CAN_BASE + DC_SWITCH:
						can_push_ptr->identifier = can.identifier;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_u8[7] = command.state;
						can_push_ptr->data.data_u8[6] = command.flags;
						can_push_ptr->data.data_u16[2] = 0;
						can_push_ptr->data.data_u16[1] = 0;
						can_push_ptr->data.data_u16[0] = switches;
						can_push();
						break;
				}
			} // End of if(can.status == CAN_RTR)
			if (can.status == CAN_ERROR) {
				if (can.identifier == 0x0002) {
					// Wake up CAN controller
					can_wake();
				}
				// Comment out for now: will always get CAN errors
				//fault();		// MVE: see the CAN error in fault light
			}
		} // End of if((P2IN & CAN_INTn) == 0x00)

		// Check sleep mode requests
/*		if(events & EVENT_REQ_SLEEP){
			events &= ~EVENT_REQ_SLEEP;
			can_abort_transmit();
			can_sleep();
			P4OUT &= ~LED_PWM;
			__bis_SR_register(LPM3_bits);     // Enter LPM3
		}
*/	} // End of while(True) do

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
 *	- ACLK  = 1.15 to 1.4 kHz (3rd harmonic) or 2.8 to 4.6 kHz (whatever is loudest on piezo)
 *	- MCLK  = 16 MHz internal oscillator
 *	- SMCLK = 16 MHz internal oscillator
 *
 * Note: We can also use the 2, 4, 8 or 16MHz crystal clock output from the MCP2515 CAN controller, which
 *       is a more accurate source than the internal oscillator.  However, using the internal
 *       source makes using sleep modes for both the MSP430 and the MCP2515 much simpler.
 */
void clock_init( void )
{
	BCSCTL3 = 0x20; 				// ACLK source = VLOCLK (4 to 20 kHz typ 12 kHz)
	BCSCTL1 = CALBC1_16MHZ | 0x20;	// ACLK divider 0x0000 = /1, 0x0100 = /2, 0x0200 = /4, 0x0300 = /8
	DCOCTL = CALDCO_16MHZ;
	P2OUT &= ~0x01;					// Set P2.0 output to zero
	P2DIR |= 0x01;					// Set P2.0 direction to output (piezo speaker)
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
	P1DIR = BRAKE_OUT | CHG_CONT_OUT | CAN_PWR_OUT | P1_UNUSED;

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

	P6OUT = ANLG_V_ENABLE; // Enable the 5V supply for the pedal etc.
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
	TBCCR3 = 0;									// Gauge 2
	TBCCTL3 = OUTMOD_7;
	TBCCR2 = 0;									// Gauge 3
	TBCCTL2 = OUTMOD_7;
	TBCCR1 = 0;									// Gauge 4
	TBCCTL1 = OUTMOD_7;
	P4SEL |= GAUGE_2_OUT | GAUGE_3_OUT | GAUGE_4_OUT;			// PWM -> output pins for stress, fuel and temp gauges (tacho is software freq output)
	TBCCTL0 = CCIE;								// Enable CCR0 interrupt
	TBCTL |= MC_1;								// Set timer to 'up' count mode
}

/*
 * Initialise A/D converter
 */
void adc_init( void )
{
	// Enable A/D input channels
	P6SEL |= ANLG_SENSE_A | ANLG_SENSE_B | ANLG_SENSE_C | ANLG_SENSE_V | ANLG_BRAKE_I | ANLG_CHG_CONT_I | ANLG_CAN_PWR_I;
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
	ADC12MCTL5 = INCH_6 | SREF_1;			// Charger contactor coil current
	ADC12MCTL6 = INCH_7 | SREF_1 | EOS;		// CAN Bus current / End of sequence
	// Enable interrupts on final conversion in sequence
//	ADC12IE = BIT6;	// Some bug with this - keep using polling for the moment
	// Enable conversions
	ADC12CTL0 |= ENC;
	// Wait 20 ms for Vref capacitor to charge up.
	unsigned int i;
	for(i = 0; i < 20; i++) brief_pause(5333);
}

/*
 * Timer B CCR0 Interrupt Service Routine
 *	- Interrupts on Timer B CCR0 match at GAUGE_FREQUENCY (10kHz)
 */
#pragma vector=TIMERB0_VECTOR
__interrupt void timer_b0(void)
{
	static int gauge_count = 0;
	// static int i = 0;
	static int gauge1_toggle, gauge1_last_toggle = 0;

	// Toggle gauge 1 pulse frequency output
	gauge1_toggle = gauge1_last_toggle + gauge.g1_count[0]; // i++ & 3];
	if (gauge_count - gauge1_toggle >= 0) {
		P4OUT ^= GAUGE_1_OUT;
		gauge1_last_toggle = gauge_count;
	}

	// Update pulse output timebase counter
	gauge_count++;

	// Update outputs if necessary
	if (events & EVENT_GAUGE1) {
		events &= ~EVENT_GAUGE1;
	}
	if (events & EVENT_GAUGE2) {
		events &= ~EVENT_GAUGE2;
		TBCCR3 = gauge.g2_duty;
	}
	if (events & EVENT_GAUGE3) {
		events &= ~EVENT_GAUGE3;
		TBCCR2 = gauge.g3_duty;
	}
	if (events & EVENT_GAUGE4) {
		events &= ~EVENT_GAUGE4;
		TBCCR1 = gauge.g4_duty;
	}
}

/*
 * Timer A CCR0 Interrupt Service Routine
 *	- Interrupts on Timer A CCR0 match at 100Hz
 *	- Sets Time_Flag variable
 */

#pragma vector=TIMERA0_VECTOR
__interrupt void timer_a0(void)
{
	static unsigned char comms_count = COMMS_SPEED;
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

	// Check for CAN activity events and blink LED
	if(events & EVENT_ACTIVITY){
		events &= ~EVENT_ACTIVITY;
		activity_count = ACTIVITY_SPEED;
		LED_PORT &= ~LED_GREENn;
	}
	if( activity_count == 0 ){
		LED_PORT |= LED_GREENn;
	}
	else {
		activity_count--;
	}

	// MVE: similarly for FAULT LED
	if (events & EVENT_FAULT) {
		events &= ~EVENT_FAULT;
		fault_count = FAULT_SPEED;
		LED_PORT &= ~LED_REDn;
		P2SEL |= 0x01;  	// Turn on beeper
	}
	if ( fault_count == 0) {
		LED_PORT |= LED_REDn;
		P2SEL &= ~0x01;  	// Turn off beeper
	}
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

//	if(P2IN & IN_GEAR_1) *state |= SW_MODE_D;	// IN_GEAR_1 now repurposed to crash signal
//	else *state &= ~SW_MODE_D;
	// Crash switch is active (crash state) when low at DB37, so active high at the port
	if (P2IN & IN_GEAR_1) *state |= SW_CRASH;
	else *state &= ~SW_CRASH;
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


