/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UART4_GPS_TX_Pin GPIO_PIN_0
#define UART4_GPS_TX_GPIO_Port GPIOA
#define UART4_GPS_RX_Pin GPIO_PIN_1
#define UART4_GPS_RX_GPIO_Port GPIOA
#define GPS_PPS_IN_Pin GPIO_PIN_2
#define GPS_PPS_IN_GPIO_Port GPIOA
#define SPI1_ESP_NSS_Pin GPIO_PIN_4
#define SPI1_ESP_NSS_GPIO_Port GPIOA
#define SPI1_ESP_SCK_Pin GPIO_PIN_5
#define SPI1_ESP_SCK_GPIO_Port GPIOA
#define SPI1_ESP_MISO_Pin GPIO_PIN_6
#define SPI1_ESP_MISO_GPIO_Port GPIOA
#define SPI1_ESP_MOSI_Pin GPIO_PIN_7
#define SPI1_ESP_MOSI_GPIO_Port GPIOA
#define TIM1_ESC1_CH1_Pin GPIO_PIN_9
#define TIM1_ESC1_CH1_GPIO_Port GPIOE
#define TIM1_ESC2_CH2_Pin GPIO_PIN_11
#define TIM1_ESC2_CH2_GPIO_Port GPIOE
#define TIM1_ESC3_CH3_Pin GPIO_PIN_13
#define TIM1_ESC3_CH3_GPIO_Port GPIOE
#define TIM1_ESC4_CH4_Pin GPIO_PIN_14
#define TIM1_ESC4_CH4_GPIO_Port GPIOE
#define ESC_DOWN_OUT_Pin GPIO_PIN_15
#define ESC_DOWN_OUT_GPIO_Port GPIOE
#define SPI2_IMU_SCK_Pin GPIO_PIN_13
#define SPI2_IMU_SCK_GPIO_Port GPIOB
#define SPI2_IMU_MISO_Pin GPIO_PIN_14
#define SPI2_IMU_MISO_GPIO_Port GPIOB
#define SPI2_IMU_MOSI_Pin GPIO_PIN_15
#define SPI2_IMU_MOSI_GPIO_Port GPIOB
#define SPI2_IMU_CSIMU_Pin GPIO_PIN_8
#define SPI2_IMU_CSIMU_GPIO_Port GPIOD
#define SPI2_IMU_CSBM_Pin GPIO_PIN_9
#define SPI2_IMU_CSBM_GPIO_Port GPIOD
#define DIS_TRIG_OUT_Pin GPIO_PIN_15
#define DIS_TRIG_OUT_GPIO_Port GPIOD
#define TIM3_DIS_ECHO_CH1_Pin GPIO_PIN_6
#define TIM3_DIS_ECHO_CH1_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define UART3_PC_TX_Pin GPIO_PIN_10
#define UART3_PC_TX_GPIO_Port GPIOC
#define UART3_PC_RX_Pin GPIO_PIN_11
#define UART3_PC_RX_GPIO_Port GPIOC
#define UART2_IBUS_TX_Pin GPIO_PIN_5
#define UART2_IBUS_TX_GPIO_Port GPIOD
#define UART2_IBUS_RX_Pin GPIO_PIN_6
#define UART2_IBUS_RX_GPIO_Port GPIOD
#define I2C1_MAG_SCL_Pin GPIO_PIN_6
#define I2C1_MAG_SCL_GPIO_Port GPIOB
#define I2C1_MAG_SDA_Pin GPIO_PIN_7
#define I2C1_MAG_SDA_GPIO_Port GPIOB
#define MAG_RDY_IN_Pin GPIO_PIN_8
#define MAG_RDY_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
