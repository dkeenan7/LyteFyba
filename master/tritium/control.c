// Implementation for a simple PID controller
// State variables like measure, set_point and the output are assumed to be 12.4 fixedpoint values
// Control constants Kp, Ki, and Kd are assumed to be 12.4 fixed point

#include "control.h"

void ctl_init(ctl_state* pState, int iSet_point, int iKp, int iKi, int iKd, int measure)
{
	pState->set_point = iSet_point;
	pState->Kp = iKp;
	pState->Ki = iKi;
	pState->Kd = iKd;
	pState->integral = 0;
	pState->prev_error = iSet_point - measure;
	
}

int	ctl_tick(ctl_state* pState, int measure) {
	int derivative, error = pState->set_point - measure;
	pState->integral += error;
	derivative = error - pState->prev_error;
	pState->prev_error = error;
	// We right shift the result by 4 bits because Ki etc are 12.4
	return ((pState->Kp * error) + (pState->Ki * pState->integral) + (pState->Kd * derivative)) >> 4;
}