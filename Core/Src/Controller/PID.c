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
	pid->alpha = (pid->T) / (pid->T + pid->tau);
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{
	//Error signal based on Anti-windup
	float error;
	if (pid->antiWindup)
		error = 0;
	else
		error = setpoint - measurement;

	// Proportional
    float proportional = pid->Kp * error;

	// Integral
    pid->integrator = pid->integrator + pid->Ki * (pid->T / 2) * (error + pid->prevError);

	// Derivative with low-pass filter
    pid->differentiator = (1 - pid->alpha) * pid->differentiator
    		+ pid->alpha *  (pid->Kd * (error - pid->prevError) / pid->T);

	// Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    float preSaturationOutput = pid->out;

    // Saturating (clamping) the output
    if (pid->out > pid->limMax)
        pid->out = pid->limMax;
    else if (pid->out < pid->limMin)
        pid->out = pid->limMin;

    // Anti-wind-up algorithm
    // If clamping was necessary
    if (preSaturationOutput != pid->out)
    {
    	// And if the integrator is trying to make saturation worse
    	if ((preSaturationOutput > 0 && error > 0)
    			|| (preSaturationOutput < 0 && error < 0))
    		pid->antiWindup = true;
    }
    else
    	pid->antiWindup = false;

	/* Store error for later use */
    pid->prevError = error;

	/* Return controller output */
    return pid->out;
}
