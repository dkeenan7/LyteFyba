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
	gauge.g1_count[0] = 0;
	gauge.g1_count[1] = 0;
	gauge.g1_count[2] = 0;
	gauge.g1_count[3] = 0;
	gauge.g2_duty = 0;
	gauge.g3_duty = 0;
	gauge.g4_duty = 0;
	events |= (EVENT_GAUGE1 | EVENT_GAUGE2 | EVENT_GAUGE3 | EVENT_GAUGE4);
}

/*
 * Updates the Tachometer gauge output
 */
void gauge_tach_update( float motor_rpm )
{
	float gauge_count_float;

	if( motor_rpm < 0.0) motor_rpm = motor_rpm * -1.0;
	if( motor_rpm > GAUGE1_MAX) motor_rpm = GAUGE1_MAX;
	if( motor_rpm < GAUGE1_MIN) motor_rpm = GAUGE1_MIN;
	gauge_count_float = ((float)GAUGE_FREQ * GAUGE1_SCALE) / motor_rpm / 2.0;
	gauge.g1_count[0] = (unsigned int) gauge_count_float;
//	gauge.g1_count[1] = (unsigned int)(gauge_count_float * 2.0) - gauge.g1_count[0];
//	gauge.g1_count[2] = (unsigned int)(gauge_count_float * 3.0) - gauge.g1_count[1] - gauge.g1_count[0];
//	gauge.g1_count[3] = (unsigned int)(gauge_count_float * 4.0) - gauge.g1_count[2] - gauge.g1_count[1] - gauge.g1_count[0];
	events |= EVENT_GAUGE1;
}

/*
 * Updates the Stress gauge (repurposed Oil Pressure gauge) output
 */
void gauge_stress_update( unsigned char BMS_stress )
{
	unsigned int count;
	// Scale for PWM output
	count = 64000/(1664-91*(int)BMS_stress); // Stress 0 ~= 52R, Stress 4 ~= 41R, Stress 12 ~= 18R
	if(count > GAUGE_PWM_PERIOD) count = GAUGE_PWM_PERIOD;
	gauge.g2_duty = count;
	events |= EVENT_GAUGE2;
}

/*
 * Updates the Temperature gauge output
 */
void gauge_temp_update( float motor_temp, float controller_temp )
{
	unsigned int temp;
	// Scale both temperatures to 0.0 to 1.0 scales
	// Pick highest reading
	// Scale for PWM output
	temp = controller_temp * 1.33;	// Testing only
	// Check limits
	if(temp > GAUGE_PWM_PERIOD) temp = GAUGE_PWM_PERIOD;
	gauge.g3_duty = temp;
	events |= EVENT_GAUGE3;
}

/*
 * Updates the Fuel gauge output
 */
void gauge_fuel_update( float battery_voltage )
{
	unsigned int temp;
	// Use lookup table to convert battery voltage to SOC
	// Scale for PWM output
	temp = battery_voltage * 0.4;	// Testing only
	// Check limits
	if(temp > GAUGE_PWM_PERIOD) temp = GAUGE_PWM_PERIOD;
	gauge.g4_duty = temp;
	events |= EVENT_GAUGE4;
}

