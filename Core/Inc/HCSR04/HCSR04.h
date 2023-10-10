#ifndef HCSR04_H_
#define HCSR04_H_

#define HAL_TIM_MODULE_ENABLED

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"

typedef struct
{
	GPIO_TypeDef *TRIG_GPIO;
	uint16_t TRIG_PIN;
	TIM_TypeDef *TIM_Instance;
	uint32_t IC_TIM_CH;
	uint32_t TIM_CLK_MHz;
	bool Triggered;

	uint8_t EDGE_STATE;
	uint16_t TMR_OVC;
	uint32_t TMR_PSC;
	uint32_t TMR_ARR;
	uint32_t T1;
	uint32_t T2;
	uint32_t DIFF;
	float DISTANCE;
	TIM_HandleTypeDef *HTIM;
	HAL_TIM_ActiveChannel ACTIV_CH;
} HCSR04_t;

/*-----[ Prototypes For All Functions ]-----*/

uint8_t HCSR04_Init(HCSR04_t* pHCSR04, TIM_HandleTypeDef* TMR_Handle);
void HCSR04_TMR_OVF_ISR(HCSR04_t* pHCSR04, TIM_HandleTypeDef* htim);
void HCSR04_TMR_IC_ISR(HCSR04_t* pHCSR04, TIM_HandleTypeDef* htim);
void HCSR04_Trigger(HCSR04_t* pHCSR04);
float HCSR04_Read(HCSR04_t* pHCSR04);

#endif /* HCSR04_H_ */
