#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"
#include "Debug.h"

extern TIM_HandleTypeDef htim1;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ControllerMutexHandle;

uint32_t ConvertToPwm(int32_t raw)
{
	// Norm raw data to 0-1000
	if (raw < 0)
		raw = 0;
	else if (raw > 1000)
		raw = 1000;

	// Add 50, so the range will be 1000-2000
	return (uint32_t)(raw + 1000);
}

void TaskMotor(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	uint8_t ESC1_start_signal;
	uint8_t ESC2_start_signal;
	uint8_t ESC3_start_signal;
	uint8_t ESC4_start_signal;

	TIM1->CCR1 = (uint32_t) (50);
	TIM1->CCR2 = (uint32_t) (50);
	TIM1->CCR3 = (uint32_t) (50);
	TIM1->CCR4 = (uint32_t) (50);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		//Log("Mot - RDMutEnter");
		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			// Hardware safety
			if (SWA < 10)
				HAL_GPIO_WritePin(ESC_DOWN_OUT_GPIO_Port, ESC_DOWN_OUT_Pin, GPIO_PIN_SET);

			else
				HAL_GPIO_WritePin(ESC_DOWN_OUT_GPIO_Port, ESC_DOWN_OUT_Pin, GPIO_PIN_RESET);

			// Software safety
			if (SWB < 10)
				Rotors = false;
			else
				Rotors = true;

			// Setting PWM speed
			if (Rotors)
			{
				int32_t ESC1_Speed;
				int32_t ESC2_Speed;
				int32_t ESC3_Speed;
				int32_t ESC4_Speed;

				if (Throttle_in > 10)
				{
					if (osMutexWait(ControllerMutexHandle, osWaitForever) == osOK)
					{
						ESC1_Speed = Throttle_in + Roll_controlled;//  - Pitch_controlled - Yaw_controlled;
						ESC2_Speed = Throttle_in - Roll_controlled;// - Pitch_controlled + Yaw_controlled;
						ESC3_Speed = Throttle_in - Roll_controlled;//  + Pitch_controlled - Yaw_controlled;
						ESC4_Speed = Throttle_in + Roll_controlled;//  + Pitch_controlled + Yaw_controlled;
					}
					osMutexRelease(ControllerMutexHandle);
				}
				else
				{
					ESC1_Speed = 0;
					ESC2_Speed = 0;
					ESC3_Speed = 0;
					ESC4_Speed = 0;
				}

				TIM1->CCR1 = ConvertToPwm(ESC1_Speed);
				TIM1->CCR2 = ConvertToPwm(ESC2_Speed);
				TIM1->CCR3 = ConvertToPwm(ESC3_Speed);
				TIM1->CCR4 = ConvertToPwm(ESC4_Speed);
			}
			else
			{
				TIM1->CCR1 = ConvertToPwm(0);
				TIM1->CCR2 = ConvertToPwm(0);
				TIM1->CCR3 = ConvertToPwm(0);
				TIM1->CCR4 = ConvertToPwm(0);
			}
		}
		osMutexRelease(RemoteDataMutexHandle);

		//LogN(xTaskGetTickCount() - time);
	}
}
