/*
 * Tritium pedal Interface
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
 * - Generates target motor rpm and current setpoints
 * - Inputs:
 *		Pedal A & B (redundant dual outputs from hall sensor pedals)
 *		Regen slider C
 *		Vehicle velocity (motor rpm at present, don't know km/h)
 *		Selected operating mode (neutral, drive, etc)
 * - Outputs:
 *		Motor current setpoint
 *		Motor rpm setpoint
 *		Errors
 *
 */

// Include files
#include <msp430x24x.h>
#include <math.h>		// MVE: For fabsf()
#include "tri86.h"
#include "pedal.h"
#include "can.h"		// MVE: for can_variables for debugging

// Public variables
command_variables	command;

/**************************************************************************************************
 * PUBLIC FUNCTIONS
 *************************************************************************************************/

/*
 * Process analog pedal inputs
 * //Basic stuff only at this point: map channel A to 0-100% current, no regen or redundancy
 * Implement Dave's square law algorithm; channel C is ready for a regen pot
 *
 */
void process_pedal( unsigned int analog_a, unsigned int analog_b, unsigned int analog_c , float motor_rpm)
{
	float pedal, regen;

	// Error Flag updates
	// Pedal too low
	if(analog_a < PEDAL_ERROR_MIN) command.flags |= FAULT_ACCEL_LOW;
	else command.flags &= (uchar)~FAULT_ACCEL_LOW;
	// Pedal too high
	if(analog_a > PEDAL_ERROR_MAX) command.flags |= FAULT_ACCEL_HIGH;
	else command.flags &= (uchar)~FAULT_ACCEL_HIGH;
	// Pedal A & B mismatch
	// not implemented...
	// Regen pot too low
	if(analog_c < REGEN_ERROR_MIN) command.flags |= FAULT_REGEN_LOW;
	else command.flags &= (uchar)~FAULT_REGEN_LOW;
	// Regen pot too high
	if(analog_c > REGEN_ERROR_MAX) command.flags |= FAULT_REGEN_HIGH;
	else command.flags &= (uchar)~FAULT_REGEN_HIGH;


	// Run command calculations only if there are no pedal faults detected
	if (command.flags == 0x00) {
		// Scale pedal input to a 0.0 to 1.0 range
		// Clip lower travel region of pedal input
		if(analog_a > PEDAL_TRAVEL_MIN) pedal = (analog_a - PEDAL_TRAVEL_MIN);
		else pedal = 0.0;
		// Scale pedal input
		pedal = pedal / PEDAL_TRAVEL;
		// Check pedal limit and clip upper travel region
		if(pedal > 1.0) pedal = 1.0;
		pedal = 0.15 + 0.85 * pedal;		// Experimental: attempt creep by simulating 15% pedal min

		// Scale regen input to a 0.0 to REGEN_MAX range
		// Clip lower travel region of regen input
		if(analog_c > REGEN_TRAVEL_MIN) regen = (analog_c - REGEN_TRAVEL_MIN);
		else regen = 0.0;
		// Scale regen input
		regen = regen * REGEN_MAX / REGEN_TRAVEL;
		// Check regen limit and clip upper travel region
		if(regen > REGEN_MAX) regen = REGEN_MAX;

		// Choose target motor current and velocity
		switch(command.state){
			// case MODE_R:
			case MODE_D:
			case MODE_B:
			{
				// Dave Keenan's quadratic pedal regen algorithm
			  	// See http://forums.aeva.asn.au/forums/forum_posts.asp?TID=1859&PID=30613#30613
				// Note that gcc doesn't do the obvious strength reduction, hence the 1.0 / RPM...:
				float normalised_rpm = motor_rpm * (1.0 / RPM_FWD_MAX);
				float p2 = pedal*pedal;		// Pedal squared
				command.current = (CURRENT_MAX * (p2 + (p2-1)*regen*normalised_rpm));
				// Literal implementation of Dave's pedal formulae lead to divide by zero or overflow hazards
				// The following, suggested by Dave, avoids both the MIN() macro and the hazards
				if (p2 >= ((1-p2)*regen))
					command.rpm = RPM_FWD_MAX;
				else
					command.rpm = RPM_FWD_MAX * p2/((1-p2)*regen);

// Return the (signed) value of whichever floating point argument has the smallest absolute value
#define fminabs(x, y) ((fabsf(x)<fabsf(y))?(x):(y))

				// Apply a slew-rate-limit to torque zero-crossings
				// to try to avoid problems with backlash causing clutch slip
				// e.g. On a positive-going zero crosssing we spend 40 ms each
				// at -10%, -1%, +1%, +10% motor current (ramp states 3, 2, 1, 0).
				switch (command.tq_ramp_state) {
					case 3:
						command.current = command.prev_current * 0.1;
						command.rpm = motor_rpm + copysignf(300.0, command.current);
						command.tq_ramp_state = 2;
						break;
					case 2:
						command.current = -command.prev_current;
						command.rpm = motor_rpm + copysignf(300.0, command.current);
						command.tq_ramp_state = 1;
						break;
					case 1:
						command.current = fminabs(command.prev_current * 10, command.current);
						command.rpm = motor_rpm + copysignf(300.0, command.current);
						command.tq_ramp_state = 0;
						break;
					default:
						if (copysignf(1.0, command.current) != copysignf(1.0, command.prev_current)) {
							command.current = fminabs(copysignf(0.1, command.prev_current), command.prev_current);
							command.rpm = motor_rpm + copysignf(300.0, command.current);
							command.tq_ramp_state = 3;
						} // End if
				} // End switch (command.tq_ramp_state)

				// Apply a slew-rate-limit to motor rpm
				// to try to prevent the 5 Hz drivetrain oscillation.
#define delta_rpm_limit 28.0	// 28 rpm per 40 ms (700 rpm per second)
				float delta_rpm = command.rpm - command.prev_rpm;
				if (delta_rpm > delta_rpm_limit) command.rpm = command.prev_rpm + delta_rpm_limit;
				else if (delta_rpm < -delta_rpm_limit) command.rpm = command.prev_rpm - delta_rpm_limit;

				command.prev_rpm = command.rpm;
				command.prev_current = command.current;
				break;
			}
			case MODE_CHARGE:
			case MODE_N:
			case MODE_START:
			case MODE_ON:
			case MODE_OFF:
			default:
				command.current = 0.0;
				command.rpm = 0.0;
				break;
		}
	}
	// There was a pedal fault detected
	else {
		command.current = 0.0;
		command.rpm = 0.0;
	}

	// Queue drive command frame
	can_push_ptr->identifier = DC_CAN_BASE + DC_DRIVE;
	can_push_ptr->status = 8;
	can_push_ptr->data.data_fp[1] = command.current;
	can_push_ptr->data.data_fp[0] = command.rpm;
	can_push();
}
