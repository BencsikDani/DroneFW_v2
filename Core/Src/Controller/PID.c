#include "Controller/PID.h"

void PIDController_Init(PIDController *pid)
{
	// Clear Anti-windup flag
	pid->antiWindup = false;

	// Clear controller variables
	pid->prevError  = 0.0f;
	pid->integrator = 0.0f;
	pid->differentiator  = 0.0f;

	// Clear output
	pid->out = 0.0f;

	// Calculate low-pass filter parameters
	pid->tau = 5 * pid->T;
	pid->alpha = ( 2*pid->T ) / ( 2*pid->tau + pid->T );

	// Reset lastTick
	pid->lastTick = 0;
}

float PIDController_Update(PIDController *pid, float reference, float measurement)
{
	// Check if enough time has passed for the T sample time
	bool run = false;
	uint32_t currentTick = xTaskGetTickCount();
	// First run after initialization
	if (pid->lastTick == 0)
	{
		run = true;
		pid->lastTick = currentTick;
	}
	// If enough time has passed (Tick's unit is ms, while T's unit is s)
	else if (currentTick - pid->lastTick >= (pid->T * 1000))
		run = true;
	// If it is too early
	else
		run = false;

	pid->lastTick = currentTick;

	// PID algorithm
	if (run)
	{
		//Error signal
		float error = reference - measurement;

		// Proportional
		float proportional = pid->Kp * error;

		// Integral with Anti-windup
		if (!pid->antiWindup)
			pid->integrator = pid->integrator + ( pid->Ki * (pid->T / 2) * (error + pid->prevError) );

		// Derivative with low-pass filter
		pid->differentiator = (1 - pid->alpha) * pid->differentiator
				+ pid->alpha *  (pid->Kd * (error - pid->prevError) / pid->T);

		// Compute output
		pid->out = proportional + pid->integrator + pid->differentiator;
		float preSaturationOutput = pid->out;

		// Saturating (clamping) the output
		if (pid->out > pid->limMax)
			pid->out = pid->limMax;
		else if (pid->out < pid->limMin)
			pid->out = pid->limMin;

		// Anti-windup check
		// If clamping had an effect...
		if (preSaturationOutput != pid->out)
		{
			// ...and if the integrator is trying to make saturation worse
			if ((preSaturationOutput > 0 && error > 0)
					|| (preSaturationOutput < 0 && error < 0))
				pid->antiWindup = true;
		}
		else
			pid->antiWindup = false;

		// Store error for later use
		pid->prevError = error;

		// Return controller output
		return pid->out;
	}
}
