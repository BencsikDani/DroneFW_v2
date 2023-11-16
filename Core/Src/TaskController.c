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


//	// Roll
//	PID_Roll_Attitude.Kp = 1.92;
//	PID_Roll_Attitude.Ki = 0;
//	PID_Roll_Attitude.Kd = 0.0648;
//	PID_Roll_Attitude.T = 0.005;
//	PID_Roll_Attitude.limMin = -50; // deg/s
//	PID_Roll_Attitude.limMax = 50; // deg/s
//	PIDController_Init(&PID_Roll_Attitude);
//
//	PID_Roll_AngVel.Kp = 2.0;
//	PID_Roll_AngVel.Ki = 0;
//	PID_Roll_AngVel.Kd = 0.1;
//	PID_Roll_AngVel.T = 0.005;
//	PID_Roll_AngVel.limMin = -500; // Motor control unit
//	PID_Roll_AngVel.limMax = 500; // Motor control unit
//	PIDController_Init(&PID_Roll_AngVel);
//
//	// Pitch
//	PID_Pitch_Attitude.Kp = 0;
//	PID_Pitch_Attitude.Ki = 0;
//	PID_Pitch_Attitude.Kd = 0;
//	PID_Pitch_Attitude.T = 0.005;
//	PID_Pitch_Attitude.limMin = -50; // deg/s
//	PID_Pitch_Attitude.limMax = 50; // deg/s
//	PIDController_Init(&PID_Pitch_Attitude);
//
//	PID_Pitch_AngVel.Kp = 0;
//	PID_Pitch_AngVel.Ki = 0;
//	PID_Pitch_AngVel.Kd = 0;
//	PID_Pitch_AngVel.T = 0.005;
//	PID_Pitch_AngVel.limMin = -500; // Motor control unit
//	PID_Pitch_AngVel.limMax = 500; // Motor control unit
//	PIDController_Init(&PID_Pitch_AngVel);

	// Roll
	// Outer
	DPID_Roll.outer.Kp = 5;
	DPID_Roll.outer.Ki = 8;
	DPID_Roll.outer.Kd = 0.15;
	DPID_Roll.outer.T = 0.005;
	DPID_Roll.outer.limMin = -50;
	DPID_Roll.outer.limMax = 50;
	// Inner
	DPID_Roll.inner.Kp = 1.5;
	DPID_Roll.inner.Ki = 0;
	DPID_Roll.inner.Kd = 0.1;
	DPID_Roll.inner.T = 0.005;
	DPID_Roll.inner.limMin = -500;
	DPID_Roll.inner.limMax = 500;
	// Init
	DoublePIDController_Init(&DPID_Roll);

	// Pitch
	// Outer
	DPID_Pitch.outer.Kp = 0;
	DPID_Pitch.outer.Ki = 0;
	DPID_Pitch.outer.Kd = 0;
	DPID_Pitch.outer.T = 0.005;
	DPID_Pitch.outer.limMin = -50;
	DPID_Pitch.outer.limMax = 50;
	// Inner
	DPID_Pitch.inner.Kp = 0;
	DPID_Pitch.inner.Ki = 0;
	DPID_Pitch.inner.Kd = 0;
	DPID_Pitch.inner.T = 0.005;
	DPID_Pitch.inner.limMin = -500;
	DPID_Pitch.inner.limMax = 500;
	// Init
	DoublePIDController_Init(&DPID_Pitch);


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
//				PID_Roll_Attitude.integrator = 0;
//				PID_Roll_AngVel.integrator = 0;
//				PID_Pitch_Attitude.integrator = 0;
//				PID_Pitch_AngVel.integrator = 0;

				DPID_Roll.outer.integrator = 0.0;
				DPID_Roll.inner.integrator = 0.0;
				DPID_Pitch.outer.integrator = 0.0;
				DPID_Pitch.inner.integrator = 0.0;
			}

			if (Tune_single_true_double_false)
			{
				DPID_Roll.outer.Kd = VRA / 200.0;
				DPID_Roll.inner.Kd = VRB / 1000.0;

				// Roll
				PIDController_Update(&DPID_Roll.inner, (Roll_in / 10.0f), GyroData[0]);
				Roll_controlled = (int16_t)(DPID_Roll.inner.out);

				// Pitch
				PIDController_Update(&DPID_Pitch.inner, (Pitch_in / 10.0f), GyroData[1]);
				Pitch_controlled = (int16_t)(DPID_Pitch.inner.out);
			}
			else
			{
				DPID_Roll.outer.Kd = VRA / 200.0;
				DPID_Roll.inner.Kd = VRB / 1000.0;

				// Roll
				//DoublePIDController_Update(&DPID_Roll, (Roll_in / 25.0f), Fusion_output.angle.roll, GyroData[0]);
				DoublePIDController_Update(&DPID_Roll, (SWD / 70.0f), Fusion_output.angle.roll, GyroData[0]);
				Roll_controlled = (int16_t)(DPID_Roll.inner.out);

				// Pitch
				//DoublePIDController_Update(&DPID_Pitch, (Pitch_in / 25.0f), Fusion_output.angle.pitch, GyroData[1]);
				DoublePIDController_Update(&DPID_Pitch, (SWD / 50.0f), Fusion_output.angle.pitch, GyroData[1]);
				Pitch_controlled = (int16_t)(DPID_Pitch.inner.out);
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
