// Implementation for a simple PID controller
// State variables like measure, set_point and the output are assumed to be signed 2.14 fixedpoint
// values.
// These values are also "normalised", i.e. biased such that the set point is zero and scaled to fit
// within -1.0 .. +1.0.
// So for example a stress setpoint of 3.5 (of a maximum of 7) is represented as 0, with a measured
// stress of +7 biased to +3.5, and shifted by 14-3 = 11 bits to become $1C00; similarly 0 maps to -3.5
// and $E400.
// The output will be clipped at +1.0 and -1.0, and should be scaled and biased appropriately to the
// control signal.

// Control constants Kp, Ki, and Kd are assumed to be 8.8 fixed point signed.

// Note that we use the "velocity form" of the classical PID equations, to avoid "integral wind-up".
// The charger algorithm would be particularly upset by this, causing overflows and worse delaying
// essential charge current cut-back when a cell becomes full.

#include <io.h>
#include <signal.h>		// For dint(), eint()
#include "pid.h"

pid::pid(/*int iSet_point,*/ int iKp, int iKi, int iKd, int measure)
{
	// set_point = iSet_point;
	prev_error = /*iSet_point*/ 0 - measure;
	prev_deriv = 0;
	prev_output = 0;
	Kp = iKp;
	Ki = iKi;
	Kd = iKd;
}

int pid::tick(int measure) {
	int error, deriv, deriv2, output;
	error = /*set_point*/ 0 - measure;
	deriv = error - prev_error;
	deriv2 = deriv - prev_deriv;

	// We right shift part of the result by 8 bits because Ki etc are 8.8 fixedpoint
#if 0					// Use conventional code
						//	(uses hardware multiply, but not multiply-accumulate, with -O2)
    output = prev_output
        +((((long)Kp * deriv)
        +  ((long)Ki * error)
        +  ((long)Kd * deriv2)) >> 8);
#else					// Use the hardware multiply-accumulate resigsters
	dint();				// Disable interrupts, in case use multiply hardware there
	nop();				// First instruction not protected
	MPYS = Kp;			// First operation is a multiply, to ignore previous results
	OP2 = deriv;
	MACS = Ki;			// Subsequent operations are multiply-accumulate
	OP2 = error;
	MACS = Kd;
	OP2 = deriv2;
	output = prev_output + (RESHI << 8) + (RESLO >> 8);
	eint();
#endif
	// Clip the output to +1.0 ($4000) and -1.0 ($C000)
	if (output > 0x4000) output = 0x4000;
	if (output < -0x4000) output = -0x4000;

	prev_error = error;
	prev_deriv = deriv;
	prev_output = output;
	return output;	
}

// This function is for when we have an invalid measurement, e.g. we get a status byte, but it fails
// the checkbits test. We use a dummy measurement of the set point minus the last error.
int pid::dummy() {
	return tick(/*set_point*/ 0 - prev_error);
}