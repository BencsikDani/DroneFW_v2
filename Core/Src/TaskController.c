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
	PID_Roll_Attitude.Kp = 1.92;
	PID_Roll_Attitude.Ki = 0;
	PID_Roll_Attitude.Kd = 0.0648;
	PID_Roll_Attitude.T = 0.005;
	PID_Roll_Attitude.limMin = -50; // deg/s
	PID_Roll_Attitude.limMax = 50; // deg/s
	PIDController_Init(&PID_Roll_Attitude);

	PID_Roll_AngVel.Kp = 2.0;
	PID_Roll_AngVel.Ki = 0;
	PID_Roll_AngVel.Kd = 0.1;
	PID_Roll_AngVel.T = 0.005;
	PID_Roll_AngVel.limMin = -500; // Motor control unit
	PID_Roll_AngVel.limMax = 500; // Motor control unit
	PIDController_Init(&PID_Roll_AngVel);

	// Pitch
	PID_Pitch_Attitude.Kp = 0;
	PID_Pitch_Attitude.Ki = 0;
	PID_Pitch_Attitude.Kd = 0;
	PID_Pitch_Attitude.T = 0.005;
	PID_Pitch_Attitude.limMin = -50; // deg/s
	PID_Pitch_Attitude.limMax = 50; // deg/s
	PIDController_Init(&PID_Pitch_Attitude);

	PID_Pitch_AngVel.Kp = 0;
	PID_Pitch_AngVel.Ki = 0;
	PID_Pitch_AngVel.Kd = 0;
	PID_Pitch_AngVel.T = 0.005;
	PID_Pitch_AngVel.limMin = -500; // Motor control unit
	PID_Pitch_AngVel.limMax = 500; // Motor control unit
	PIDController_Init(&PID_Pitch_AngVel);


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
				PID_Roll_Attitude.integrator = 0;
				PID_Roll_AngVel.integrator = 0;
				PID_Pitch_Attitude.integrator = 0;
				PID_Pitch_AngVel.integrator = 0;
			}

			if (Tune_single_true_double_false)
			{
				// Roll
				PID_Roll_AngVel.Kp = VRA / 100.0; // 0-10
				PID_Roll_AngVel.Kd = VRB / 1000.0; // 0-1

				PIDController_Update(&PID_Roll_AngVel, (Roll_in / 10.0f), GyroData[0]);
				Roll_controlled = (int16_t)(PID_Roll_AngVel.out);

				// Pitch
				PID_Pitch_AngVel.Kp = VRA / 100.0; // 0-10
				PID_Pitch_AngVel.Kd = VRB / 1000.0; // 0-1

				PIDController_Update(&PID_Pitch_AngVel, (Pitch_in / 10.0f), GyroData[1]);
				Pitch_controlled = (int16_t)(PID_Pitch_AngVel.out);
			}
			else
			{
				// Roll
				//PID_Roll_Attitude.Kp = VRA / 100.0; // 0-1
				//PID_Roll_Attitude.Kd = VRB / 1000.0; // 0-1

				//PIDController_Update(&PID_Roll_Attitude, (Roll_in / 25.0f), Roll_measured);
				PIDController_Update(&PID_Roll_Attitude, (SWD / 50.0f), Roll_measured);
				PIDController_Update(&PID_Roll_AngVel, PID_Roll_Attitude.out, GyroData[0]);
				Roll_controlled = (int16_t)(PID_Roll_AngVel.out);

				// Pitch
				//PID_Pitch_Attitude.Kp = VRA / 100.0; // 0-1
				//PID_Pitch_Attitude.Kd = VRB / 1000.0; // 0-1

				PIDController_Update(&PID_Pitch_Attitude, (Pitch_in / 25.0f), Pitch_measured);
				PIDController_Update(&PID_Pitch_AngVel, PID_Pitch_Attitude.out, GyroData[1]);
				Pitch_controlled = (int16_t)(PID_Pitch_AngVel.out);
			}

//			char str[20];
//			//sprintf(str, "%1.2f\r\n", PID_Roll_AngVel.out);
//			//sprintf(str, "%d\r\n", Roll_controlled);
//			sprintf(str, "%1.2f, %d\r\n", PID_Roll_AngVel.out, Roll_controlled);
//			HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
		}
		osMutexRelease(ControllerMutexHandle);
		osMutexRelease(RemoteDataMutexHandle);
		osMutexRelease(ImuMutexHandle);

		//LogN(xTaskGetTickCount() - time);
	}
}
