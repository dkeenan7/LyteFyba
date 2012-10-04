//	pid.h
//	Interface for a simple PID controller
//

#ifndef __PID_H_
#define __PID_H_

class pid {
//	int		set_point;				// Desired set point
	int		prev_error;				// Previous error
	int		prev_deriv;				// Previous derivative, to find second derivative
	int		prev_output;			// Previous output
	int		Kp;						// Proportional control constant
	int		Ki;						// Integral control constant
	int		Kd;						// Derivative control constant
public:
			pid(/*int iSet_point,*/ int iKp, int iKi, int iKd, int measure);
	int		tick(int measure);		// Main function every tick of time:
									// Use the measurement to produce an output
	int		dummy();				// As above, when we get an invalid measure

};

#define nop()  __asm__ __volatile__("nop"::)

#endif		// ifndef __PID_H_