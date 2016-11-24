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
void process_pedal( unsigned int analog_a, unsigned int analog_b, unsigned int analog_c , float motor_rpm, float torque_current)
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
#if 0
				// Tritium's pedal mapping -- no regen
				command.current = pedal * CURRENT_MAX;
				command.rpm = RPM_FWD_MAX;
#endif
#if 1
				// Dave Keenan's quadratic pedal regen algorithm
			  	// See http://forums.aeva.asn.au/forums/forum_posts.asp?TID=1859&PID=30613#30613

				// Note that gcc doesn't do the obvious strength reduction, hence the 1.0 / RPM...:
				float normalised_rpm = motor_rpm * (1.0 / RPM_FWD_MAX);
				pedal = 0.15 + 0.85 * pedal;	// Implement creep by simulating 15% pedal min
				float p2 = pedal*pedal;		// Pedal squared
				command.current = (CURRENT_MAX * (p2 + (p2-1)*regen*normalised_rpm));
				// Literal implementation of Dave's pedal formulae lead to divide by zero or overflow hazards
				// The following, suggested by Dave, avoids both the MIN() macro and the hazards
				if (p2 >= ((1-p2)*regen))
					command.rpm = RPM_FWD_MAX;
				else
					command.rpm = RPM_FWD_MAX * p2/((1-p2)*regen);

				// Drop out of cruise-control if the pedal has been pushed beyond the current speed.
				if (command.rpm > command.prev_rpm)
					command.cruise_control = false;

				if (command.cruise_control) {
					command.rpm = command.prev_rpm;
					// command.current = regen / 3.0;
				}
#endif
#if 0
				// Ross Pink's pedal regen algorithm (best with wsConfig mass = 50 kg)
			  	// See http://forums.aeva.asn.au/forums/ac-drive-programming-and-pedal-mapping_topic1859.html
				// Max torque_current is sqrt(2) * Sine current limit
				command.rpm = RPM_FWD_MAX * (pedal - 0.2 * torque_current / 283.0);
				if (command.rpm < 0.0) command.rpm = 0.0;
				command.current = 0.3 + 0.7 * pedal;
		//		if (pedal > 0.0)
		//			command.current = 1.0;
		//		else
		//			command.current = 0.0;
#endif

#if 0
				// Apply a slew-rate-limit to torque zero-crossings
				// to try to avoid problems with backlash causing clutch slip
				// e.g. On a positive-going zero crosssing we spend 40 ms each
				// at -10%, -1%, +1%, +10% motor current (ramp states 3, 2, 1, 0).
// Return the (signed) value of whichever floating point argument has the smallest absolute value
#define fminabs(x, y) ((fabsf(x)<fabsf(y))?(x):(y))
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
#endif
#if 0
				// Apply a slew-rate-limit to the motor rpm setpoint
				// to try to prevent the 5 Hz drivetrain oscillation.
#define delta_rpm_limit 28.0	// 28 rpm per 40 ms (700 rpm per second)
				float delta_rpm = command.rpm - command.prev_rpm;
				if (delta_rpm > delta_rpm_limit) command.rpm = command.prev_rpm + delta_rpm_limit;
// Down ramp is dangerous.	else if (delta_rpm < -delta_rpm_limit) command.rpm = command.prev_rpm - delta_rpm_limit;
// Even the up ramp is dangerous when changing gears
#endif

#if 1
				// Apply a notch filter to the torque setpoint
				// to try to prevent the 5 Hz drivetrain oscillation.

				// Digital Notch Filter design by Hirano, K.; Nishimura, S.; Mitra, S.K.,
				// “Design of Digital Notch Filters,” Communications, IEEE Transactions on ,
				// vol.22, no.7, pp.964,970, Jul 1974.
				// as described by Krishna Sankar on July 14, 2013 in
				// http://www.dsplog.com/2013/07/14/digital-notch-filter/

				// The parameters a1 and a2 below determine the center frequency and bandwidth
				// (or damping ratio) of the notch filter, but not in any straightforward manner.
				// They must be calculated as follows, where
				// fs is the sample frequency,
				// fn is the notch frequency, and
				// fb is the bandwidth or twice the damping ratio times fn.
				// Typical damping ratios are 0.4 to 0.7.
				// omega0T = 2*pi*fn/fs;
				// deltaT  = 2*pi*fb/fs;
				// a2 = (1-tan(deltaT/2))/(1+tan(deltaT/2));
				// a1 = (1+a2)*cos(omega0T);

				// fs=25, fn=3.8, fb=3.04 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.426783712;
// #define a1 0.824071326;
				// fs=25, fn=3.8, fb=5.32 Hz (damping 0.7)
// #define a2 0.117402225
// #define a1 0.645381024
				// fs=25, fn=5.2, fb=4.16 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.268847301
// #define a1 0.330968041
				// fs=25, fn=5.2, fb=7.28 Hz (damping 0.7)
// #define a2 -0.130161297
// #define a1 0.226890037
				// fs=100, fn=3.4, fb=2.72 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.842197516
// #define a1 1.800320909
				// fs=100, fn=3.4, fb=4.76 Hz (damping 0.7)
// #define a2 0.738126025
// #define a1 1.698615159
				// fs=100, fn=5.2, fb=4.16 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.767659797
// #define a1 1.674147598
				// fs=100, fn=5.2, fb=7.28 Hz (damping 0.7)
// #define a2 0.622348323
// #define a1 1.536523346
				// fs=100, fn=4.2, fb=5.88 Hz (damping 0.7)
#define a2 0.685124545
#define a1 1.626788294
				// fs=100, fn=6.0, fb=8.40 Hz (damping 0.7)
// #define a2 0.574561111
// #define a1 1.463989897
				static float z1 = 0.0;
				static float z2 = 0.0;
/*
      ,------------------------------.
      |      z0        z1        z2  v               ^
in ---+-->(+)-->[z^-1]--+->[z^-1]-->(+)          < arrows >          (+)    adder
           ^            |            |               v               (x)    multiplier
           |            v            |                               (/2)   divide by 2
           |     a1--->(x)    -a2    |            ,-------.          [z^-1] delay
           |            |      |     |            | wires |
           | h          v      v   g |            `-------'           |
           +-----------(+)<---(x)<---+                              --+-- junction
           |                       _ v                                |
           `----------------------->(+)-->(/2)--> out
*/
				float g = command.current + z2;
				float h = a1*z1 - a2*g;
				float z0 = command.current + h;
				command.current = (g - h) / 2.0; // For bandpass use (z0-z2)/2.0
				z2 = z1;
				z1 = z0;
#endif
				// Record the command rpm and current for next time
				command.prev_rpm = command.rpm;
				command.prev_current = command.current;
				break;
			}
			case MODE_CHARGE:
			case MODE_N:
			case MODE_START:
			case MODE_ON:
			case MODE_OFF:
			case MODE_CRASH:
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
