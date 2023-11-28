#include "Controller/PID.h"
#include <stdint.h>

void PIDController_Init(PIDController *pid)
{
	// Clear Anti-windup flag
	pid->antiWindup = false;

	// Clear controller variables
	pid->integrator = 0.0f;
	pid->prev_error = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prev_measurement = 0.0f;

	// Clear output
	pid->out = 0.0f;

	// Calculate low-pass filter parameters
	pid->tau = 5 * pid->T;
	pid->alpha = ( 2*pid->T ) / ( 2*pid->tau + pid->T );

	// Reset lastTick
	pid->lastTick = 0;
}

float PIDController_Update(PIDController *pid, float reference, float measurement, bool enable_integration)
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
		float proportional_result = pid->Kp * error;

		// Integral with Anti-windup
		if (!pid->antiWindup && enable_integration)
		{
			pid->integrator = pid->integrator + error * pid->T;
			pid->integrator_result = pid->Ki * pid->integrator;
		}

		// Derivative
		pid->differentiator = -(measurement - pid->prev_measurement) / pid->T;
		pid->differentiator_result = pid->Kd * pid->differentiator;
		// Store previous measurement for later use
		pid->prev_measurement = measurement;

		// Derivative with low-pass filter
		//pid->differentiator = 0.8f * pid->differentiator + 0.2f * (-(measurement - pid->prev_measurement) / pid->T);
		//pid->differentiator_result = pid->Kd * pid->differentiator;
		// Store previous measurement for later use
		//pid->prev_measurement = measurement;


		// Compute output
		pid->out = proportional_result + pid->integrator_result + pid->differentiator_result;
		float pre_saturation_output = pid->out;

		// Saturating (clamping) the output
		if (pid->out > pid->limMax)
			pid->out = pid->limMax;
		else if (pid->out < pid->limMin)
			pid->out = pid->limMin;

		// Anti-windup check
		// If clamping had an effect...
		if (pre_saturation_output != pid->out
				// ...and if the integrator is trying to make saturation worse
				&& ((pre_saturation_output > 0 && error > 0) || (pre_saturation_output < 0 && error < 0)))
			pid->antiWindup = true;
		else
			pid->antiWindup = false;


		return 0;
	}
}

void DoublePIDController_Init(DoublePIDController *pid)
{
	// Clear Anti-windup flag
	pid->outer.antiWindup = false;
	pid->inner.antiWindup = false;

	// Clear controller variables
	pid->outer.integrator = 0.0f;
	pid->inner.integrator = 0.0f;
	pid->outer.differentiator  = 0.0f;
	pid->inner.differentiator  = 0.0f;

	pid->outer.prev_measurement = 0.0f;
	pid->inner.prev_measurement = 0.0f;

	// Clear output
	pid->outer.out = 0.0f;
	pid->inner.out = 0.0f;

	// Calculate low-pass filter parameters
	//pid->tau = 5 * pid->T;
	//pid->alpha = ( 2*pid->T ) / ( 2*pid->tau + pid->T );

	// Reset lastTick
	pid->outer.lastTick = 0;
	pid->inner.lastTick = 0;
}

float DoublePIDController_Update(DoublePIDController *pid, float outer_reference, float outer_measurement, float inner_measurement, bool enable_integration)
{
	// Check if enough time has passed for the T sample time
	// I use the outer PID's properties for that
	bool run = false;
	uint32_t currentTick = xTaskGetTickCount();
	// First run after initialization
	if (pid->outer.lastTick == 0)
	{
		run = true;
		pid->outer.lastTick = currentTick;
	}
	// If enough time has passed (Tick's unit is ms, while T's unit is s)
	else if (currentTick - pid->outer.lastTick >= (pid->outer.T * 1000))
		run = true;
	// If it is too early
	else
		run = false;

	pid->outer.lastTick = currentTick;

	// PID algorithm
	if (run)
	{
		// Outer PID Controller
		//Error signal
		float outer_error = outer_reference - outer_measurement;

		// Proportional
		float outer_proportional_result = pid->outer.Kp * outer_error;

		// Integral with Anti-windup
		if (!pid->outer.antiWindup && enable_integration)
		{
			pid->outer.integrator = pid->outer.integrator + outer_error * pid->outer.T;
			pid->outer.integrator_result = pid->outer.Ki * pid->outer.integrator;
		}

		// Derivative
		pid->outer.differentiator = -inner_measurement;
		pid->outer.differentiator_result = pid->outer.Kd * pid->outer.differentiator;

		// Derivative with low-pass filter
		//pid->outer.differentiator = 0.4f * pid->outer.differentiator + 0.6f * (-inner_measurement);
		//pid->outer.differentiator_result = pid->outer.Kd * pid->outer.differentiator;

		// Compute output
		pid->outer.out = outer_proportional_result + pid->outer.integrator_result + pid->outer.differentiator_result;
		float outer_pre_saturation_output = pid->outer.out;

		// Saturating (clamping) the output
		if (pid->outer.out > pid->outer.limMax)
			pid->outer.out = pid->outer.limMax;
		else if (pid->outer.out < pid->outer.limMin)
			pid->outer.out = pid->outer.limMin;

		// Anti-windup check
		// If clamping had an effect...
		if (outer_pre_saturation_output != pid->outer.out
				// ...and if the integrator is trying to make saturation worse
				&& ((outer_pre_saturation_output > 0 && outer_error > 0) || (outer_pre_saturation_output < 0 && outer_error < 0)))
			pid->outer.antiWindup = true;
		else
			pid->outer.antiWindup = false;




		// Inner PID Controller
		//Error signal
		float inner_error = pid->outer.out - inner_measurement;

		// Proportional
		float inner_proportional_result = pid->inner.Kp * inner_error;

		// Integral with Anti-windup
		if (!pid->inner.antiWindup && enable_integration)
		{
			pid->inner.integrator = pid->inner.integrator + inner_error * pid->inner.T;
			pid->inner.integrator_result = pid->inner.Ki * pid->inner.integrator;
		}

		// Derivative
		pid->inner.differentiator =-(inner_measurement - pid->inner.prev_measurement) / pid->inner.T;
		pid->inner.differentiator_result = pid->inner.Kd * pid->inner.differentiator;
		// Store previous measurement for later use
		pid->inner.prev_measurement = inner_measurement;

		// Derivative with low-pass filter
		//pid->inner.differentiator = 0.8f * pid->inner.differentiator + 0.2f * (-(inner_measurement - pid->inner.prev_measurement) / pid->inner.T);
		//pid->inner.differentiator_result = pid->inner.Kd * pid->inner.differentiator;
		// Store previous measurement for later use
		//pid->inner.prev_measurement = inner_measurement;

		// Compute output
		pid->inner.out = inner_proportional_result + pid->inner.integrator_result + pid->inner.differentiator_result;
		float inner_pre_saturation_output = pid->inner.out;

		// Saturating (clamping) the output
		if (pid->inner.out > pid->inner.limMax)
			pid->inner.out = pid->inner.limMax;
		else if (pid->inner.out < pid->inner.limMin)
			pid->inner.out = pid->inner.limMin;

		// Anti-windup check
		// If clamping had an effect...
		if (inner_pre_saturation_output != pid->inner.out
				// ...and if the integrator is trying to make saturation worse
				&& ((inner_pre_saturation_output > 0 && inner_error > 0) || (inner_pre_saturation_output < 0 && inner_error < 0)))
			pid->inner.antiWindup = true;
		else
			pid->inner.antiWindup = false;


		return 0;
	}
}
