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

float safe_norm_divide( float num, float denom ) {
	// Safe divide of quantities where the result can be limited to -1.0 to +1.0
	if (num == 0.0F) return 0.0F;
	if (denom < 0.0F)
		if (num <= denom) return 1.0F;
		else if (num >= -denom) return -1.0F;
		else return num/denom;
	else if (denom > 0.0F)
		if (num >= denom) return 1.0F;
		else if (num <= -denom) return -1.0F;
		else return num/denom;
	else if (num > 0.0F) return 1.0F;
		else return -1.0F;
}


/**************************************************************************************************
 * PUBLIC FUNCTIONS
 *************************************************************************************************/
#define max(x, y) (((x)>(y))?(x):(y))
#define min(x, y) (((x)<(y))?(x):(y))

/*
 * Process analog pedal inputs
 * //Basic stuff only at this point: map channel A to 0-100% current, no regen or redundancy
 * Implement Dave's square law algorithm; channel C is ready for a regen pot
 *
 */
void process_pedal( unsigned int analog_a, unsigned int analog_b, unsigned int analog_c ,
			float motor_rpm, float torque_current, unsigned int switches, unsigned int switches_diff)
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
		else pedal = 0.0F;
		// Scale pedal input
		pedal = pedal / PEDAL_TRAVEL;
		// Check pedal limit and clip upper travel region
		if(pedal > 1.0F) pedal = 1.0F;

		// Scale regen input to a 0.0 to REGEN_MAX range
		// Clip lower travel region of regen input
		if(analog_c > REGEN_TRAVEL_MIN) regen = (analog_c - REGEN_TRAVEL_MIN);
		else regen = 0.0F;
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

#if 1
				// The original algorithm, intended to be used with the notch filter

				// Note that gcc doesn't do the obvious strength reduction, hence the 1.0 / RPM...:
				float normalised_rpm = motor_rpm * (1.0F / RPM_FWD_MAX);
				pedal = 0.15F + 0.85F * pedal;	// Implement creep by simulating 15% pedal min
				float p2 = pedal*pedal;		// Pedal squared
				command.current = CURRENT_MAX * (p2 + (p2-1.0F)*regen*normalised_rpm);
				// Literal implementation of Dave's pedal formulae lead to divide by zero or overflow hazards
				// The following, suggested by Dave, avoids both the MIN() macro and the hazards
				command.rpm = RPM_FWD_MAX * safe_norm_divide(
					p2,
					(1.0F-p2)*regen );
#else
				// Warrick's attempted alternative to the notch filter,
				// where he aplies a torque correction for the high frequency components of
				// motor rpm. This was successful in 2nd gear, but not 1st.

				// Note that gcc doesn't do the obvious strength reduction, hence the 1.0 / RPM...:
				float normalised_rpm = motor_rpm * (1.0F / RPM_FWD_MAX);

				// Highpass filter the normalised_rpm
/*
  2nd order digital highpass filter with only two multiplies
 (There is no way to get a lowpass out of this with only adds, subtracts and divide by 2)
        ,----->(+)--->(/2)---->(+)--->(/2)---> out               [z^-1] delay by 1 sample
        |       ^               ^                      ^         (+)    adder
        |       | z2            | z1               < arrows >    (-)    change sign
        |       |      ,--(-)---'                      v         (*)    multiplier
        |       `------|--------------.            ,-------.     (/2)   divide by 2
        | z0           | z1           | z2         | wires |
        +---->[z^-1]---+---->[z^-1]---+            `-------'     Fs = sampling frequency
        |              |              |                          f0 = filter frequency
in -+---|----------+---|-----.        |                |         Q = filter peaking
    |   |          |   v     |        v             ---|---        0.577 for Bessel
    `->(+)         `->(+)    `->(-)->(+)               |           0.707 for Butterworth
        ^              |              |            crossing        0.980 for Chebyshev 1dB
        |              v              v                          w0 = 2*pi*f0/Fs
        |        -a1->(*)       -a2->(*)               |         a = sin(w0)/(2*Q)
        |              |              |             ---+---
        |              v              |                |         a1 = -2*cos(w0)/(1+a)
        `-------------(+)<------------'            junction      a2 = (1-a)/(1+a)
*/
/*
  2nd order digital lowpass filter with only two multiplies
 (There is no way to get a highpass out of this with only adds, subtracts and divide by 2)
        ,----->(+)--->(/2)---->(+)--->(/2)---> out               [z^-1] delay by 1 sample
        |       ^               ^                      ^         (+)    adder
        |       | z2            | z1               < arrows >    (-)    change sign
        |       |      ,--------'                      v         (*)    multiplier
        |       `------|--------------.            ,-------.     (/2)   divide by 2
        | z0           | z1           | z2         | wires |
        +---->[z^-1]---+---->[z^-1]---+            `-------'     Fs = sampling frequency
        |              |              |                          f0 = filter frequency
in -+---|-----+--------|-----.        |                |         Q = filter peaking
    |   |     |        v     |        v             ---|---        0.577 for Bessel
    `->(+)    `->(-)->(+)    `->(-)->(+)               |           0.707 for Butterworth
        ^              |              |            crossing        0.980 for Chebyshev 1dB
        |              v              v                          w0 = 2*pi*f0/Fs
        |        -a1->(*)       -a2->(*)               |         a = sin(w0)/(2*Q)
        |              |              |             ---+---
        |              v              |                |         a1 = -2*cos(w0)/(1+a)
        `-------------(+)<------------'            junction      a2 = (1-a)/(1+a)
*/

				// Fs = 25 Hz, f0 = 0.5 Hz, Q = 0.577 (Bessel)
#define A1 -1.789950695F
#define A2 0.804177171F
				static float Z1 = 0.0F;
				static float Z2 = 0.0F;
//				float Z0 = normalised_rpm - A1 * (Z1 + normalised_rpm) - A2 * (Z2 - normalised_rpm); // highpass
//				float hpf_norm_rpm = ((Z0 + Z2)/2.0F - Z1)/2.0F; // highpass
				float Z0 = normalised_rpm - A1 * (Z1 - normalised_rpm) - A2 * (Z2 - normalised_rpm); // lowpass
				float lpf_norm_rpm = ((Z0 + Z2)/2.0F + Z1)/2.0F; // lowpass

				Z2 = Z1;
				Z1 = Z0;
//				float torque_cor = 0.7F * hpf_norm_rpm;

				pedal = 0.15F + 0.85F * pedal;	// Implement creep by simulating 15% pedal min
				float p2 = pedal*pedal;		// Pedal squared
				command.current = CURRENT_MAX * (p2 - (1.0F-p2)*regen*lpf_norm_rpm);
				// The requested rpm must give the correct sign to the torque, so its formula is obtained
				// by equating the torque (i.e. current) formula to zero and solving for the rpm.
				// Literal implementation of Dave's pedal formulae from the AEVA page leads to divide by zero or overflow hazards.
				// The use of safe_norm_divide(), suggested by Dave, avoids both the MIN() macro and the hazards.
				command.rpm = RPM_FWD_MAX * (safe_norm_divide(p2,(1-p2)*regen) + normalised_rpm - lpf_norm_rpm);
#endif
#endif

#if 0
				// Dave Keenan's combined cruise control and speed limiting using the ignition key.
				// See http://forums.aeva.asn.au/forums/weber-and-coulombs-mx5_topic980_post63320.html#63320

				// Make rising edges of IGN_START toggle cruise control or speed limiting.
				if (!bDCUb &&  (switches & switches_diff & SW_IGN_START)) {
					// If either of them is on, turn them off
					if (command.cruise_control || command.speed_limiting) {
						P5OUT &= (uchar)~LED_FAULT_3;	// Turn off the headlight retractor light (cruise/limit)
						command.cruise_control = false;
						command.speed_limiting = false;
					}
					// else they're off, so turn one of them on
					else {
						P5OUT |= LED_FAULT_3;	// Turn on the headlight retractor light (cruise/limit)
						// If we're in regen, choose cruise control
						if (command.rpm < motor_rpm) {
							command.cruise_control = true;
						}
						// else we're not in regen, so choose speed limiting
						else {
							command.speed_limiting = true;
						}
						// Round the motor rpm to the nearest rpm in the array below.
						// Each is the rpm for a near-multiple of 10 km/h in some gear other than first.
						const unsigned int nx10km_rpm[11] =
							{2216,2540,2723,3027,3176,3363,3578,3809,4025,4473,5079};
							// 35  (40)  43   48  (50)  53   56  (60)  63  (70) (80)km/h in 2nd gear
							//(50)  57  (61)  68  (71)  75  (80)  85  (90)(100)	114	km/h in 3rd gear
							// 66   76  (81) (90)  95 (100)	106  113 				km/h in 4th gear
							//(81)  93  (99)(111) 116								km/h in 5th gear
						// The following are the boundaries between the bins for the above.
						// In the worst cases, the capture range is only +- 1.3 km/h.
						const unsigned int nx10km_upr_bound[10] =
							{2361,2624,2879,3071,3297,3455,3673,3936,4198,4761};
						// Here are the pedal positions corresponding to these rpms, assuming
						// REGEN_MAX = 0.7F and RPM_FWD_MAX = 7000.F
						// pedal = sqrt(REGEN_MAX*rpm/RPM_FWD_MAX)/sqrt(REGEN_MAX*rpm/RPM_FWD_MAX + 1)
						const float nx10km_pedal[11] =
							{0.425912480F,0.450057590F,0.462624956F,0.482041028F,0.490962649F,0.501662282F,
							 0.513336643F,0.525199582F,0.535712164F,0.555930073F,0.580367168F};
						unsigned int rpm = (unsigned int)motor_rpm;	// Convert float to integer
						// Determine which bin the present motor rpm falls into
						unsigned int i;
						for ( i = 0; i < 11; i++ ) {
							if (rpm < nx10km_upr_bound[i]) break;
						}
						// Record the standard rpm (and pedal position) for that bin, as our set point.
						command.rpm_limit = nx10km_rpm[i];
						command.pedal_limit = nx10km_pedal[i];
					}
				}
				// Drop out of cruise control if the brake pedal is pushed even slightly.
				// or the clutch pedal is pushed, or we're in neutral.
				if (command.cruise_control && ((switches & SW_BRAKE) || (switches & SW_NEUT_OR_CLCH))) {
					P5OUT &= (uchar)~LED_FAULT_3;	// Turn off the headlight retractor light (cruise/limit)
					command.cruise_control = false;
				}
				// Drop out of speed limiting if the accelerator pedal has been pushed all the way,
				// or the clutch pedal is pushed, or we're in neutral.
				if (command.speed_limiting && ((pedal >= 1.0F) || (switches & SW_NEUT_OR_CLCH))) {
					P5OUT &= (uchar)~LED_FAULT_3;	// Turn off the headlight retractor light (cruise/limit)
					command.speed_limiting = false;
				}

				const float LIMIT_GAIN = 20.0F; // This determines how hard the motor works to maintain
									 		// constant speed when in speed limiting or cruise control
				float fader, current_limit, normalised_rpm_limit, gain_times_delta_rpm;
				// For cruise control, apply a lower limit on requested rpm.
				// For speed limiting, apply an upper limit on requested rpm.
				if ((command.cruise_control && (pedal < command.pedal_limit))
				||  (command.speed_limiting && (pedal > command.pedal_limit))) {
					// Ensure enough torque to maintain speed up hills and prevent overspeed down hills.
					// But do not exceed the maximum allowed regen torque.
					// I use the max regen torque also as the max forward torque
					// in cruise control and speed limiting.
					current_limit = CURRENT_MAX * max(-regen, min(regen,
											(motor_rpm - command.rpm_limit) * (LIMIT_GAIN / RPM_FWD_MAX)));
					// Give a smooth transition between limiting and non-limiting torques by fading
					// between them over 10% of pedal travel. Also fade out speed-limiting above 90% pedal.
					if (command.cruise_control)
						fader = max(0.0F, min(1.0F, (0.1F + pedal - command.pedal_limit) * 10.0F));
					else  // Speed limiting
						if (pedal > 0.9F)
							fader = max(0.0F, min(1.0F, (pedal - 0.9F) * 10.0F));
						else
							fader = max(0.0F, min(1.0F, (0.1F + command.pedal_limit - pedal) * 10.0F));
					command.current = fader * command.current + (1.0F-fader) * current_limit;
					// The WaveSculptor, like most VF drives, ignores the sign of the current, and gives
					// it the sign of the difference between requested rpm and actual rpm.
					// That means we have to calculate the requested rpm very carefully to ensure
					// we get the sign we're expecting for the torque.
					// Its not as simple as fading between rpms in the same way we fade between currents.
					// Thanks to Wolfram Alpha for solving some of the equations to help get this right.
					// I ran it in a spreadsheet and plotted graphs to test it.
					normalised_rpm_limit = command.rpm_limit * (1.0F / RPM_FWD_MAX);
					gain_times_delta_rpm = safe_norm_divide(
						fader*((1.0F-p2)*regen*normalised_rpm_limit - p2)*LIMIT_GAIN,
						(fader*(1.0F-p2)*regen + (1.0F-fader)*LIMIT_GAIN) );
					if (gain_times_delta_rpm <= -regen)
						command.rpm = safe_norm_divide(
							(fader*p2+(1.0F-fader)*-regen),
							(fader*(1.0F-p2)*regen) );
					else if (gain_times_delta_rpm >= regen)
						command.rpm = safe_norm_divide(
							(fader*p2+(1.0F-fader)*regen),
							(fader*(1.0F-p2)*regen) );
					else
						command.rpm = safe_norm_divide(
							(fader*p2+(1.0F-fader)*LIMIT_GAIN*normalised_rpm_limit),
							(fader*(1.0F-p2)*regen+(1.0F-fader)*LIMIT_GAIN) );
					command.rpm = command.rpm * RPM_FWD_MAX;
				}
#endif

#if 0
				// Ross Pink's pedal regen algorithm (best with wsConfig mass = 50 kg)
			  	// See http://forums.aeva.asn.au/forums/ac-drive-programming-and-pedal-mapping_topic1859.html
				// Max torque_current is sqrt(2) * Sine current limit
				command.rpm = RPM_FWD_MAX * (pedal - 0.2F * torque_current / 283.0F);
				if (command.rpm < 0.0F) command.rpm = 0.0F;
				command.current = 0.3F + 0.7F * pedal;
		//		if (pedal > 0.0F)
		//			command.current = 1.0F;
		//		else
		//			command.current = 0.0F;
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
						command.current = command.prev_current * 0.1F;
						command.rpm = motor_rpm + copysignf(300.0F, command.current);
						command.tq_ramp_state = 2;
						break;
					case 2:
						command.current = -command.prev_current;
						command.rpm = motor_rpm + copysignf(300.0F, command.current);
						command.tq_ramp_state = 1;
						break;
					case 1:
						command.current = fminabs(command.prev_current * 10.0F, command.current);
						command.rpm = motor_rpm + copysignf(300.0F, command.current);
						command.tq_ramp_state = 0;
						break;
					default:
						if (copysignf(1.0F, command.current) != copysignf(1.0F, command.prev_current)) {
							command.current = fminabs(copysignf(0.1F, command.prev_current), command.prev_current);
							command.rpm = motor_rpm + copysignf(300.0F, command.current);
							command.tq_ramp_state = 3;
						} // End if
				} // End switch (command.tq_ramp_state)
#endif

#if 0
				// Apply a slew-rate-limit to the motor rpm setpoint
				// to try to prevent the 5 Hz drivetrain oscillation.
#define delta_rpm_limit 28.0F	// 28 rpm per 40 ms (700 rpm per second)
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
				// a1 = -(1+a2)*cos(omega0T);

				// fs=25, fn=3.8, fb=3.04 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.426783712F
// #define a1 -0.824071326F
				// fs=25, fn=3.8, fb=5.32 Hz (damping 0.7)
// #define a2 0.117402225F
// #define a1 -0.645381024F
				// fs=25, fn=5.2, fb=4.16 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.268847301F
// #define a1 -0.330968041F
				// fs=25, fn=5.2, fb=7.28 Hz (damping 0.7)
// #define a2 -0.130161297F
// #define a1 -0.226890037F
				// fs=100, fn=3.4, fb=2.72 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.842197516F
// #define a1 -1.800320909F
				// fs=100, fn=3.4, fb=4.76 Hz (damping 0.7)
// #define a2 0.738126025F
// #define a1 -1.698615159F
				// fs=100, fn=5.2, fb=4.16 Hz (damping 0.4). Set CURRRENT_MAX to 0.9 to allow for overshoot
// #define a2 0.767659797F
// #define a1 -1.674147598F
				// fs=100, fn=5.2, fb=7.28 Hz (damping 0.7)
// #define a2 0.622348323F
// #define a1 -1.536523346F
				// fs=100, fn=4.2, fb=5.88 Hz (damping 0.7)
#define a2 0.685124545F
#define a1 -1.626788294F
				// fs=100, fn=6.0, fb=8.40 Hz (damping 0.7)
// #define a2 0.574561111F
// #define a1 -1.463989897F
				static float z1 = 0.0F;
				static float z2 = 0.0F;
/*
      ,------------------------------.
      |      z0        z1        z2  v               ^
in ---+-->(+)-->[z^-1]--+->[z^-1]-->(+)          < arrows >          (+)    adder
           ^            |            |               v               (x)    multiplier
           |            v            |                               (/2)   divide by 2
           |    -a1--->(x)    -a2    |            ,-------.          [z^-1] delay
           |            |      |     |            | wires |
           | h          v      v   g |            `-------'           |
           +-----------(+)<---(x)<---+                              --+-- junction
           |                       _ v                                |
           `----------------------->(+)-->(/2)--> out
*/
				float g = command.current + z2;
				float h = -a1*z1 - a2*g;
				float z0 = command.current + h;
				command.current = (g - h) / 2.0F; // For bandpass use (z0-z2)/2.0F
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
				command.current = 0.0F;
				command.rpm = 0.0F;
				break;
		}
	}
	// There was a pedal fault detected
	else {
		command.current = 0.0F;
		command.rpm = 0.0F;
	}

	// Queue drive command frame
	can_push_ptr->identifier = DC_CAN_BASE + DC_DRIVE;
	can_push_ptr->status = 8;
	can_push_ptr->data.data_fp[1] = command.current;
	can_push_ptr->data.data_fp[0] = command.rpm;
	can_push();
}
