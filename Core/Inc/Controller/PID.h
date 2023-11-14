#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stdbool.h"
#include <stdint.h>

typedef struct {

	// Controller gains
	float Kp, Ki, Kd;

	// Sample time (in seconds)
	// Should be less then (T_system / 10)
	float T;
	uint32_t lastTick;

	// Derivative low-pass filter
	float tau, alpha;

	// Output limits
	float limMin, limMax;

	// Integrator Anti-windup flag
	bool antiWindup;

	// Controller "memory"
	float integrator, integrator_result;
	float prev_error;
	float differentiator, differentiator_result;
	float prev_measurement;

	// Controller output
	float out;
} PIDController;

typedef struct {
	PIDController inner, outer;
} DoublePIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
void DoublePIDController_Init(DoublePIDController *pid);
float DoublePIDController_Update(DoublePIDController *pid, float outer_reference, float outer_measurement, float inner_measurement);

#endif
