/*
 * Tritium gauge driver Interface
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
 * - Implements the following CAN interface functions
 *	- gauge_init
 *	- gauge_tach_update
 *	- gauge_power_update
 *	- gauge_temp_update
 *	- gauge_fuel_update
 */

// Include files
#include <msp430x24x.h>
#include "tri86.h"
#include "gauge.h"

// Public variables
gauge_variables	gauge;

/**************************************************************************************************
 * PUBLIC FUNCTIONS
 *************************************************************************************************/

/*
 * Initialises gauge output channels
 *	- Sets PWM to 'zero' levels for various gauges
 *	- Gauges 1 are a pulse frequency output to simulate reed switches
 *	- Gauges 2 & 3 & 4 are a duty cycle output to simulate a variable resistor to GND
 */
void gauge_init( void )
{
	gauge.g1_half_period = 0;
	gauge.g2_duty = 0;
	gauge.g3_duty = 0;
	gauge.g4_duty = 0;
	events |= (EVENT_GAUGE1 | EVENT_GAUGE2 | EVENT_GAUGE3 | EVENT_GAUGE4);
}

#define max(x, y) (((x)>(y))?(x):(y))
#define min(x, y) (((x)<(y))?(x):(y))

/*
 * Updates the Tachometer gauge output
 */
void gauge_tach_update( float motor_rpm )
{
	unsigned int adj_rpm;
	if( motor_rpm < 0.0) motor_rpm = motor_rpm * -1.0;
	if( motor_rpm > GAUGE1_MAX) motor_rpm = GAUGE1_MAX;
	if( motor_rpm < GAUGE1_MIN) motor_rpm = GAUGE1_MIN;
	adj_rpm = (unsigned int)motor_rpm + 100;
	gauge.g1_half_period = (642335 - (adj_rpm >> 1)) / adj_rpm; // = Round(642335 / adj_rpm) - 1
	events |= EVENT_GAUGE1;
}

/*
 * Updates the Stress gauge (repurposed Oil Pressure gauge) output
 */
void gauge_stress_update( unsigned char BMS_stress )
{
	unsigned int count;
	// Scale for PWM output
	// Stress 0 = gauge 0, Stress 6 = gauge 3, Stress 12 = gauge 6
	if (BMS_stress == 0)
		count = 140;
	else
		count = 145 + (BMS_stress*7)/2;
	if(count > GAUGE_PWM_PERIOD) count = GAUGE_PWM_PERIOD;
	gauge.g2_duty = count;
	events |= EVENT_GAUGE2;
}

/*
 * Updates the Temperature gauge output
 */
void gauge_temp_update( float motor_temp, float controller_temp )
{
	float norm_temp;
	unsigned int count;
	// Scale both temperatures to 0.0 to 1.0 scales
	// Pick highest reading
	norm_temp = max(-0.25, min(1.25, max((motor_temp-40)/(160-40), (controller_temp-40)/(80-40))));
	// C = 40. H = 160 for motor and 80 for controller. Middle = 100 for motor, 60 for controller.
	count = 320.8 * (1 - 1/(norm_temp + 1.295));	// count/GAUGE_PWM_PERIOD = count/200 is duty cycle
	// Check limits
	if(count > GAUGE_PWM_PERIOD) count = GAUGE_PWM_PERIOD;
	gauge.g3_duty = count;
	events |= EVENT_GAUGE3;
}

/*
 * Updates the Fuel gauge output
 */
void gauge_fuel_update( float battery_voltage )
{
	float norm_fuel;
	unsigned int count;
	// Scale to a 0.0 to 1.0 scale between 3.25 and 3.35 V per cell
	norm_fuel = (battery_voltage / 109.0 - 3.25) / 0.1;
//	count = 120 + 100 * norm_fuel;	// count/GAUGE_PWM_PERIOD = count/200 is duty cycle
	count = 160; // testing
	// Check limits
	if(count > GAUGE_PWM_PERIOD) count = GAUGE_PWM_PERIOD;
	gauge.g4_duty = count;
	events |= EVENT_GAUGE4;
}

