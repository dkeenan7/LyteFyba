// Implementation for a simple PID controller
// State variables like measure, set_point and the output are assumed to be 12.4 fixedpoint values
// Control constants Kp, Ki, and Kd are assumed to be 12.4 fixed point
// Note that we use the "velocity form" of the classical PID equations, to avoid "integral wind-up".
// The charger algorithm would be particularly upset by this, causing overflows and worse delaying
// essential charge current cut-back when a cell becomes full.

#include "control.h"

void ctl_init(ctl_state* pState, int iSet_point, int iKp, int iKi, int iKd, int measure)
{
	pState->set_point = iSet_point;
	pState->Kp = iKp;
	pState->Ki = iKi;
	pState->Kd = iKd;
	pState->prev_deriv = 0;
	pState->prev_error = iSet_point - measure;
	pState->prev_result = iSet_point;
}

int	ctl_tick(ctl_state* pState, int measure) {
	int error, deriv, deriv2, result;
	error = pState->set_point - measure;
	deriv = error - pState->prev_error;
	deriv2 = deriv - pState->prev_deriv;
	// We right shift part of the result by 4 bits because Ki etc are 12.4 fixedpoint
	result = pState->prev_result
		+(((pState->Kp * deriv)
		+  (pState->Ki * error)
		+  (pState->Kd * deriv2)) >> 4);
	pState->prev_error = error;
	pState->prev_deriv = deriv;
	pState->prev_result = result;
	return result;	
}