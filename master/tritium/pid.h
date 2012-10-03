//	Control.h
//	Interface for a simple PID controller
//

#ifndef __CONTROL_H_
#define __CONTROL_H_

typedef struct _pid_state {
	int		set_point;		// Desired set point
	int		Kp;				// Proportional control constant
	int		Ki;				// Integral control constant
	int		Kd;				// Derivative control constant
	int		prev_error;		// Previous error
	int		prev_result;	// Previous result
	int		prev_deriv;		// Previous derivative, to find second derivative
} pid_state;

int pid_tick(pid_state* pState, int measure);	// Main function every tick of time:
												// Use the measurement to produce an output

void pid_init(pid_state* pState, int iSet_point, int iKp, int iKi, int iKd, int measure);

#endif		// ifndef __CONTROL_H_