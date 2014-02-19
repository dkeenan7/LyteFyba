/*
 * Tritium pedal interface header
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
 * - Implements the following pedal interface functions:
 *	- process_pedal
 *
 */

#ifndef __PEDAL_H_
#define __PEDAL_H_

// Public function prototypes
void process_pedal( unsigned int a, unsigned int b, unsigned int c , float motor_rpm);

// Public variables
typedef struct _command_variables {
	float rpm;
	float current;
	float bus_current;
	unsigned char flags;
	unsigned char state;
	float prev_current;
	int ramp_state;
} command_variables;

extern command_variables command;

// Error flags
#define FAULT_ACCEL_LOW			0x01
#define FAULT_ACCEL_HIGH		0x02
#define FAULT_ACCEL_MISMATCH	0x04
#define FAULT_REGEN_LOW			0x10
#define FAULT_REGEN_HIGH		0x20
#define FAULT_NO_PEDAL			0x40				// Set if no pedal found, i.e. DCU B


// Command parameter limits
#define CURRENT_MAX				1.0					// %, absolute value
#define REGEN_MAX				0.5					// %, absolute value
#define RPM_FWD_MAX				6500				// Forwards max speed, rpm
#define RPM_REV_MAX				-2500				// Reverse max speed, rpm

// Analog pedal input scaling
// Potentiometer only (no redundancy)
// Channel A = 0.00 to 5.00 Volts = 0 to 4096 counts
// Channel B = Unused
#define ADC_MAX					4096
#define PEDAL_TRAVEL_MIN		270		// Possibly a little high, but may stop zero pedal jitter
#define PEDAL_TRAVEL_MAX		3850
#define PEDAL_TRAVEL			(PEDAL_TRAVEL_MAX - PEDAL_TRAVEL_MIN)
#define PEDAL_ERROR_MIN			(PEDAL_TRAVEL_MIN >> 1)
#define PEDAL_ERROR_MAX			((ADC_MAX + PEDAL_TRAVEL_MAX) >> 1)
#define PEDAL_MISMATCH_MAX		100

// Analog input for linear slider type pot for regenerative strenght control
// Channel C = 0.00 to 5.00 Volts = 0 to 4096 counts
#define REGEN_TRAVEL_MIN		100
#define REGEN_TRAVEL_MAX		(ADC_MAX - 100)
#define REGEN_TRAVEL			(REGEN_TRAVEL_MAX - REGEN_TRAVEL_MIN)
#define REGEN_ERROR_MIN			0
#define REGEN_ERROR_MAX			(ADC_MAX - 0)

#endif	// #ifndef __PEDAL_H_