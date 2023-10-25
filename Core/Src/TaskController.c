#include "Globals.h"
#include "Controller/PID.h"

extern osMutexId DistMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ControllerMutexHandle;

void TaskController(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	PID_Roll_Attitude.Kp = 0;
	PID_Roll_Attitude.Ki = 0;
	PID_Roll_Attitude.Kd = 0;
	PID_Roll_Attitude.T = 0;
	PID_Roll_Attitude.limMin = 0;
	PID_Roll_Attitude.limMax = 50;
	PIDController_Init(&PID_Roll_Attitude);

	PID_Roll_AngVel.Kp = 0;
	PID_Roll_AngVel.Ki = 0;
	PID_Roll_AngVel.Kd = 0;
	PID_Roll_AngVel.T = 0.005;
	PID_Roll_AngVel.limMin = -25;
	PID_Roll_AngVel.limMax = 25;
	PIDController_Init(&PID_Roll_AngVel);

	xLastWakeTime = xTaskGetTickCount();
	// Infinite loop
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

//		if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
//		{
//			Throttle_controlled = PIDController_Update(&PID_Thrust, 0.2f, Distance/1000.0f);
//		}
//		osMutexRelease(DistMutexHandle);


		if (osMutexWait(ControllerMutexHandle, osWaitForever) == osOK
				&& osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK
				&& osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			PID_Roll_AngVel.Kp = VRA / 1000.0;
			PID_Roll_AngVel.Kd = VRB / 1000.0;

			//float AngVelRef = PIDController_Update(&PID_Roll_Attitude, Roll_in/5, Roll_measured);
			Roll_controlled = PIDController_Update(&PID_Roll_AngVel, (Roll_in), GyroData[0]);
		}
		osMutexRelease(ControllerMutexHandle);
		osMutexRelease(RemoteDataMutexHandle);
		osMutexRelease(ImuMutexHandle);

		//LogN(xTaskGetTickCount() - time);
	}
}
