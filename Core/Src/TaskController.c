#include "Globals.h"
#include "Controller/PID.h"

extern osMutexId DistMutexHandle;

void TaskController(void const *argument)
{
	// Inner loop controllers
	PIDController PID_Thrust;
	PID_Thrust.Kd = 1;
	PID_Thrust.Ki = 1;
	PID_Thrust.Kd = 1;
	PID_Thrust.T = 0.01f;
	PID_Thrust.limMin = 0;
	PID_Thrust.limMax = 30;
	PIDController_Init(&PID_Thrust);

	PIDController PID_Yaw;
	PID_Yaw.Kd = 0;
	PID_Yaw.Ki = 0;
	PID_Yaw.Kd = 0;
	PID_Yaw.T = 0;
	PID_Yaw.limMin = 0;
	PID_Yaw.limMax = 50;
	PIDController_Init(&PID_Yaw);

	PIDController PID_Pitch;
	PID_Pitch.Kd = 0;
	PID_Pitch.Ki = 0;
	PID_Pitch.Kd = 0;
	PID_Pitch.T = 0;
	PID_Pitch.limMin = 0;
	PID_Pitch.limMax = 50;
	PIDController_Init(&PID_Pitch);

	PIDController PID_Roll;
	PID_Roll.Kd = 0;
	PID_Roll.Ki = 0;
	PID_Roll.Kd = 0;
	PID_Roll.T = 0;
	PID_Roll.limMin = 0;
	PID_Roll.limMax = 50;
	PIDController_Init(&PID_Roll);

	// Infinite loop
	while (1)
	{

		if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
		{
			PIDController_Update(&PID_Thrust, 0.2f, Distance/1000.0f);
			Throttle_controlled = PID_Thrust.out;
		}
		osMutexRelease(DistMutexHandle);


		osDelay(10);
	}
}
