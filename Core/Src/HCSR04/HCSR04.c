#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "HCSR04/HCSR04.h"

extern osSemaphoreId DistSemaphoreHandle;

uint8_t HCSR04_Init(HCSR04_t* pHCSR04, TIM_HandleTypeDef* htim)
{
	pHCSR04->TRIG_GPIO = DIS_TRIG_OUT_GPIO_Port;
	pHCSR04->TRIG_PIN = DIS_TRIG_OUT_Pin;
	pHCSR04->TIM_Instance = TIM1;
	pHCSR04->IC_TIM_CH = TIM_CHANNEL_3;
	pHCSR04->TIM_CLK_MHz = 16;
	pHCSR04->Triggered = false;

	if(pHCSR04->IC_TIM_CH == TIM_CHANNEL_1)
	{
		pHCSR04->ACTIV_CH = HAL_TIM_ACTIVE_CHANNEL_1;
	}
	else if(pHCSR04->IC_TIM_CH == TIM_CHANNEL_2)
	{
		pHCSR04->ACTIV_CH = HAL_TIM_ACTIVE_CHANNEL_2;
	}
	else if(pHCSR04->IC_TIM_CH == TIM_CHANNEL_3)
	{
		pHCSR04->ACTIV_CH = HAL_TIM_ACTIVE_CHANNEL_3;
	}
	else if(pHCSR04->IC_TIM_CH == TIM_CHANNEL_4)
	{
		pHCSR04->ACTIV_CH = HAL_TIM_ACTIVE_CHANNEL_4;
	}
	/*--------[ Start The ICU Channel ]-------*/

	HAL_TIM_Base_Start_IT(htim);
	HAL_TIM_IC_Start_IT(htim, pHCSR04->IC_TIM_CH);

	return 0;
}


void HCSR04_TMR_OVF_ISR(HCSR04_t* pHCSR04, TIM_HandleTypeDef* htim)
{
	if(htim->Instance == pHCSR04->TIM_Instance)
	{
		pHCSR04->TMR_OVC++;
	}
}


void HCSR04_TMR_IC_ISR(HCSR04_t* pHCSR04, TIM_HandleTypeDef* htim)
{
	uint32_t PS = 0;

	if((htim->Instance == pHCSR04->TIM_Instance) && (htim->Channel == pHCSR04->ACTIV_CH))
	{
		if (pHCSR04->EDGE_STATE == 0)
		{
			// Capture T1 & Reverse The ICU Edge Polarity
			pHCSR04->T1 = HAL_TIM_ReadCapturedValue(htim, pHCSR04->IC_TIM_CH);
			pHCSR04->EDGE_STATE = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, pHCSR04->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
			pHCSR04->TMR_OVC = 0;
		}
		else if (pHCSR04->EDGE_STATE == 1)
		{
			// Read The Current ARR & Prescaler Values For The Timer
			PS = pHCSR04->TIM_Instance->PSC;
			pHCSR04->TMR_ARR = pHCSR04->TIM_Instance->ARR;
			// Capture T2 & Calculate The Distance
			pHCSR04->T2 = HAL_TIM_ReadCapturedValue(htim, pHCSR04->IC_TIM_CH);
			//pHCSR04->T2 += (pHCSR04->TMR_OVC * (pHCSR04->TMR_ARR+1));

			if (pHCSR04->T1 < pHCSR04->T2)
				pHCSR04->DIFF = pHCSR04->T2 - pHCSR04->T1;
			else if (pHCSR04->T2 < pHCSR04->T1)
				pHCSR04->DIFF = (pHCSR04->T2 + 65535) - pHCSR04->T1;

			// Write The Distance Value To The Global Struct & Reverse The ICU Edge
			pHCSR04->DISTANCE = ((pHCSR04->DIFF / 1000.0f) * 340.0f / 2.0f) / (pHCSR04->TIM_CLK_MHz/(PS+1));
			//pHCSR04->DISTANCE = pHCSR04->DIFF;
			pHCSR04->EDGE_STATE = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, pHCSR04->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);

			osSemaphoreRelease(DistSemaphoreHandle);
		}
	}
}

void HCSR04_Trigger(HCSR04_t* pHCSR04)
{
	HAL_GPIO_WritePin(pHCSR04->TRIG_GPIO, pHCSR04->TRIG_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(pHCSR04->TRIG_GPIO, pHCSR04->TRIG_PIN, GPIO_PIN_RESET);
}

float HCSR04_Read(HCSR04_t* pHCSR04)
{
	return pHCSR04->DISTANCE;
}
