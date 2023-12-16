#include "Globals.h"
#include "printf.h"
#include "math.h"
#include "Controller/PID.h"

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;

extern osMutexId DistMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ControllerMutexHandle;

void TaskController(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	// Roll
	// Outer
	DPID_Roll.outer.Kp = 8;
	DPID_Roll.outer.Ki = 0.5;
	DPID_Roll.outer.Kd = 0.15;
	DPID_Roll.outer.T = 1.0f / xFrequency;
	DPID_Roll.outer.limMin = -50;
	DPID_Roll.outer.limMax = 50;
	// Inner
	DPID_Roll.inner.Kp = 0.6;
	DPID_Roll.inner.Ki = 0;
	DPID_Roll.inner.Kd = 0.06;
	DPID_Roll.inner.T = 1.0f / xFrequency;
	DPID_Roll.inner.limMin = -500;
	DPID_Roll.inner.limMax = 500;
	// Init
	DoublePIDController_Init(&DPID_Roll);

	// Pitch
	// Outer
	DPID_Pitch.outer.Kp = 8;
	DPID_Pitch.outer.Ki = 0.5;
	DPID_Pitch.outer.Kd = 0.15;
	DPID_Pitch.outer.T = 1.0f / xFrequency;
	DPID_Pitch.outer.limMin = -50;
	DPID_Pitch.outer.limMax = 50;
	// Inner
	DPID_Pitch.inner.Kp = 0.6;
	DPID_Pitch.inner.Ki = 0;
	DPID_Pitch.inner.Kd = 0.06;
	DPID_Pitch.inner.T = 1.0f / xFrequency;
	DPID_Pitch.inner.limMin = -500;
	DPID_Pitch.inner.limMax = 500;
	// Init
	DoublePIDController_Init(&DPID_Pitch);

	// Yaw
	PID_Yaw.Kp = 6;
	PID_Yaw.Ki = 0;
	PID_Yaw.Kd = 0;
	PID_Yaw.T = 1.0f / xFrequency;
	PID_Yaw.limMin = -500;
	PID_Yaw.limMax = 500;
	// Init
	PIDController_Init(&PID_Yaw);

	// Throttle
	PID_Throttle.Kp = 0;
	PID_Throttle.Ki = 0;
	PID_Throttle.Kd = 0;
	PID_Throttle.T = 1.0f / xFrequency;
	PID_Throttle.limMin = 0;
	PID_Throttle.limMax = 800;
	// Init
	PIDController_Init(&PID_Throttle);


	xLastWakeTime = xTaskGetTickCount();
	// Infinite loop
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		if (osMutexWait(ControllerMutexHandle, osWaitForever) == osOK
				&& osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK
				&& osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			if (SWC > 490)
			{
				DPID_Roll.outer.integrator = 0.0f;
				DPID_Roll.outer.integrator_result = 0.0f;
				DPID_Roll.inner.integrator = 0.0f;
				DPID_Roll.inner.integrator_result = 0.0f;
				DPID_Pitch.outer.integrator = 0.0f;
				DPID_Pitch.outer.integrator_result = 0.0f;
				DPID_Pitch.inner.integrator = 0.0f;
				DPID_Pitch.inner.integrator_result = 0.0f;
				PID_Yaw.integrator = 0.0f;
				PID_Yaw.integrator_result = 0.0f;
				PID_Throttle.integrator = 0.0f;
				PID_Throttle.integrator_result = 0.0f;
			}
			/*
			DPID_Roll.outer.Kd = VRA / 2000.0f;
			DPID_Roll.inner.Kd = VRB / 2000.0f;

			DPID_Pitch.outer.Kd = VRA / 2000.0f;
			DPID_Pitch.inner.Kd = VRB / 2000.0f;
			*/
			if (Tune_single_true_double_false)
			{
				// Roll
				PIDController_Update(&DPID_Roll.inner, (Roll_in / 10.0f), GyroData[0], (Throttle_in > 10));
				Roll_controlled = (int16_t)(DPID_Roll.inner.out);

				// Pitch
				PIDController_Update(&DPID_Pitch.inner, (-Pitch_in / 10.0f), GyroData[1], (Throttle_in > 10));
				Pitch_controlled = (int16_t)(DPID_Pitch.inner.out);

				// Yaw
				PIDController_Update(&PID_Yaw, (Yaw_in / 10.0f), GyroData[2], (Throttle_in > 10));
				Yaw_controlled = (int16_t)(PID_Yaw.out);

				// Throttle
				PIDController_Update(&PID_Throttle, Throttle_in, Distance, (Throttle_in > 35));
				Throttle_controlled = (uint16_t)(PID_Throttle.out);
			}
			else
			{
				// Roll
				DoublePIDController_Update(&DPID_Roll, (Roll_in / 25.0f), Fusion_output.angle.roll, GyroData[0], (Throttle_in > 10));
				//DoublePIDController_Update(&DPID_Roll, (SWD / 70.0f), Fusion_output.angle.roll, GyroData[0], (Throttle_in > 10));
				Roll_controlled = (int16_t)(DPID_Roll.inner.out);

				// Pitch
				DoublePIDController_Update(&DPID_Pitch, (-Pitch_in / 25.0f), Fusion_output.angle.pitch, GyroData[1], (Throttle_in > 10));
				//DoublePIDController_Update(&DPID_Pitch, (SWD / 70.0f), Fusion_output.angle.pitch, GyroData[1], (Throttle_in > 10));
				Pitch_controlled = (int16_t)(DPID_Pitch.inner.out);

				// Yaw
				PIDController_Update(&PID_Yaw, (Yaw_in / 10.0f), GyroData[2], (Throttle_in > 10));
				Yaw_controlled = (int16_t)(PID_Yaw.out);

				// Throttle
				PIDController_Update(&PID_Throttle, Throttle_in, Distance, (Throttle_in > 35));
				Throttle_controlled = (uint16_t)(PID_Throttle.out);
			}
		}
		osMutexRelease(ControllerMutexHandle);
		osMutexRelease(RemoteDataMutexHandle);
		osMutexRelease(ImuMutexHandle);

		//LogN(xTaskGetTickCount() - time);
	}
}
