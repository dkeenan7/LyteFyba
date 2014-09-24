/*
 * Tritium gauges interface header
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
 * - Implements the following gauge interface functions
 *	- gauge_init
 *	- gauge_tach_update
 *	- gauge_stress_update
 *	- gauge_temp_update
 *	- gauge_fuel_update
 *
 */

// Public function prototypes
extern void gauge_init( void );
extern void gauge_tach_update( float motor_rpm );
extern void gauge_stress_update( unsigned char BMS_stress );
extern void gauge_temp_update( float motor_temp, float controller_temp );
extern void gauge_fuel_update( float stateOfCharge );

// Public variables
typedef struct _gauge_variables {
	unsigned int g1_half_period;
	unsigned int g2_duty;
	unsigned int g3_duty;
	unsigned int g4_duty;
} gauge_variables;

extern gauge_variables gauge;

// Overall gauge definitions
// Timer B ISR triggers at GAUGE_FREQUENCY
// Tach output is software driven counter to produce freqency pulses
// Stress, Fuel and Temp outputs are PWM hardware outputs
#define GAUGE_FREQ			10000
#define GAUGE_PWM_PERIOD	(INPUT_CLOCK / 8 / GAUGE_FREQ)



