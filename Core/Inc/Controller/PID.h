#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stdbool.h"

typedef struct {

	// Controller gains
	float Kp, Ki, Kd;

	// Sample time (in seconds)
	float T;

	// Derivative low-pass filter
	float tau, alpha;

	// Output limits
	float limMin, limMax;

	// Integrator Anti-windup flag
	bool antiWindup;

	// Controller "memory"
	float prevError;
	float integrator;
	float differentiator;

	// Controller output
	float out;
} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
