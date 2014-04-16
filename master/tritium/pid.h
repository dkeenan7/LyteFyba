//	pid.h
//	Interface for a simple PID controller
//

#ifndef __PID_H_
#define __PID_H_

#define fract int		// s0.15 fixed point, range -1.0 to almost +1.0
#define accum long		// s16.15 fixed point, range -65536.0 to almost +65536.0
#define short_accum int	// s7.8 fixed point, range -128.0 to almost +128.0

class pid {
//	fract		set_point;			// Desired set point, always zero in this implementation
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

//#define nop()  __asm__ __volatile__("nop"::)

// Saturating subtraction of signed ints
// Uses tricky MSP430 assembler
inline int sat_minus (register int x, register int y)
{
#if 0
	long result = (long)x - y;
	if (result > 0x7FFFL) result = 0x7FFFL;
	if (result < -0x8000L) result = -0x8000L;
	return result;
#else
	asm (" sub	%1,%0	\n"
    	 " clrn 		\n" /* Clear N flag so GE condition depends on V flag only (normally N XOR V) */
		 " jge	1f		\n" /* If no oVerflow, jump to local label 1: forward */
    	 " subc	%0,%0	\n" /* Set all bits = NOT C flag = previous N flag (on overflow) */
    	 " rrc	%0		\n" /* C flag is unchanged above, shift it into the sign bit */
		 "1:			\n" /* Local label to jump to */
	: "=r" (x) 			/* output %0 is any reg and is x */
	: "r" (y), "0" (x)	/* input %1 is any reg and is y, another input is same reg as output (%0) and is x */
	: 					/* no clobbered regs */ );

	return x;
#endif
}

#endif		// ifndef __PID_H_
