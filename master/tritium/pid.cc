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

//#include <io.h>			// For hardware multiplier special function regs
#include <msp430.h>			// For hardware multiplier special function regs
#include <intrinsics.h>		// For __dint(), __eint()
#include "pid.h"

pid::pid(/*fract iSet_point,*/ short_accum iKp, short_accum iKi, short_accum iKd, fract measure)
{
	// set_point = iSet_point;
	prev_error = /*iSet_point*/ 0 - measure;
	prev_deriv = 0;
	prev_output = 0;
	Kp = iKp;
	Ki = iKi;
	Kd = iKd;
}

fract pid::tick(fract measure) {
	fract error, deriv, deriv2;
	accum output;

	error = sat_minus(/*set_point*/ 0, measure);
	deriv = sat_minus(error, prev_error);
	deriv2 = sat_minus(deriv, prev_deriv);

	// We right shift part of the result by 8 bits because Ki etc are s7.8 fixedpoint
#if 0					// Use conventional code
						//	(uses hardware multiply, but not multiply-accumulate, with -O2)
    output = prev_output
        +((((long)Kp * deriv)
        +  ((long)Ki * error)
        +  ((long)Kd * deriv2)) >> 8);
#else					// Use the hardware multiply-accumulate registers
	__dint();			// Disable interrupts, in case use multiply hardware there
	nop();				// First instruction not protected
	MPYS = (unsigned)Kp;			// First operation is a multiply, to ignore previous results
	OP2  = (unsigned)deriv;
	MACS = (unsigned)Ki;			// Subsequent operations are multiply-accumulate
	OP2  = (unsigned)error;
	MACS = (unsigned)Kd;
	OP2  = (unsigned)deriv2;
	// Allow accessing multiplier result as a _signed_ long instead of two unsigned ints.
	static volatile signed long RESLONG asm("0x013A");
	output = prev_output + (RESLONG >> 8);
	__eint();
#endif
	// Saturate the output to -1.0 ($8000) and almost +1.0 ($7FFF)
	if (output > 0x7FFFL) output = 0x7FFFL;
	if (output < -0x8000L) output = -0x8000L;

	prev_error = error;
	prev_deriv = deriv;
	prev_output = (fract)output;
	return (fract)output;
}

// This function is for when we have an invalid measurement, e.g. we get a status byte, but it fails
// the checkbits test. We use a dummy measurement of the set point minus the last error.
fract pid::tick() {
	return tick(/*set_point*/ 0 - prev_error);
}
