//	pid.h
//	Interface for a simple PID controller
//

#ifndef __PID_H_
#define __PID_H_

#define fract int		// s0.15 fixed point, range -1.0 to almost +1.0
#define accum long		// s16.15 fixed point, range -65536.0 to almost +65536.0
#define short_accum int	// s7.8 fixed point, range -128.0 to almost +128.0

class pid {
//	fract		set_point;			// Desired set point
	fract		prev_error;			// Previous error
	fract		prev_deriv;			// Previous derivative, to find second derivative
	fract		prev_output;		// Previous output
	short_accum	Kp;					// Proportional control gain
	short_accum	Ki;					// Integral control gain / time constant
	short_accum	Kd;					// Derivative control gain * time constant
public:
			pid(/*fract iSet_point,*/ short_accum iKp, short_accum iKi, short_accum iKd, fract measure);
	fract	tick(fract measure);	// Main function every tick of time:
									// Use the measurement to produce an output
	fract	tick();					// As above, when we get an invalid measure

};

#define nop()  __asm__ __volatile__("nop"::)

#endif		// ifndef __PID_H_