#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"

#include "Debug.h"

extern TIM_HandleTypeDef htim3;
extern osMutexId RemoteDataMutexHandle;

void TaskMotor(void const *argument)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	uint8_t ESC1_start_signal;
	uint8_t ESC2_start_signal;
	uint8_t ESC3_start_signal;
	uint8_t ESC4_start_signal;

	TIM3->CCR3 = (uint32_t) (50);
	TIM3->CCR4 = (uint32_t) (50);
	TIM3->CCR1 = (uint32_t) (50);
	TIM3->CCR2 = (uint32_t) (50);

	/* Infinite loop */
	while (1)
	{
		//Log("Mot - RDMutEnter");
		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			if (SWA < 10)
				HAL_GPIO_WritePin(ESC_DOWN_OUT_GPIO_Port, ESC_DOWN_OUT_Pin, GPIO_PIN_SET);

			else
				HAL_GPIO_WritePin(ESC_DOWN_OUT_GPIO_Port, ESC_DOWN_OUT_Pin, GPIO_PIN_RESET);

			if (SWB < 10)
				Rotors = false;
			else
				Rotors = true;


			if (SWD < 10)
			{
				ESC1_start_signal = 1;
				ESC2_start_signal = 1;
				ESC3_start_signal = 1;
				ESC4_start_signal = 1;
			}
			else
			{
				ESC1_start_signal = 1;
				ESC2_start_signal = 1;
				ESC3_start_signal = 2;
				ESC4_start_signal = 3;
			}


			// Setting PWM speed
			if (Rotors)
			{
				TIM3->CCR3 = (uint32_t) ((Throttle_controlled * (50-(ESC1_start_signal-1)) / 50) + (50+ESC1_start_signal-1));
				TIM3->CCR4 = (uint32_t) ((Throttle_controlled * (50-(ESC2_start_signal-1)) / 50) + (50+ESC2_start_signal-1));
				TIM3->CCR1 = (uint32_t) ((Throttle_controlled * (50-(ESC3_start_signal-1)) / 50) + (50+ESC3_start_signal-1));
				TIM3->CCR2 = (uint32_t) ((Throttle_controlled * (50-(ESC4_start_signal-1)) / 50) + (50+ESC4_start_signal-1));
			}
			else
			{
				TIM3->CCR3 = (uint32_t) (50);
				TIM3->CCR4 = (uint32_t) (50);
				TIM3->CCR1 = (uint32_t) (50);
				TIM3->CCR2 = (uint32_t) (50);
			}
		}
		osMutexRelease(RemoteDataMutexHandle);

		osDelay(100);
	}
}
