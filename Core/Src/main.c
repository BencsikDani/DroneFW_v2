/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "stdbool.h"
//#include "core_cm4.h"
#include "Globals.h"

#include "TaskSensorData.h"
#include "TaskController.h"
#include "TaskRemote.h"
#include "TaskMotor.h"
#include "TaskPower.h"
#include "TaskDiagnostics.h"

#include "Debug.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId TaskSensorDataHandle;
osThreadId TaskControllerHandle;
osThreadId TaskRemoteHandle;
osThreadId TaskMotorHandle;
osThreadId TaskPowerHandle;
osThreadId TaskDiagnosticsHandle;
osMutexId MagnMutexHandle;
osMutexId RemoteDataMutexHandle;
osMutexId ImuMutexHandle;
osMutexId GpsDataMutexHandle;
osMutexId DistMutexHandle;
osMutexId RemoteBufferMutexHandle;
osSemaphoreId DistSemaphoreHandle;
osSemaphoreId GpsBufferSemaphoreHandle;
osSemaphoreId RemoteBufferFullSemaphoreHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void RunTaskSensorData(void const * argument);
void RunTaskController(void const * argument);
void RunTaskRemote(void const * argument);
void RunTaskMotor(void const * argument);
void RunTaskPower(void const * argument);
void RunTaskDiagnostics(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		//Log("U2CB");
		Uart2CallbackCounter++;

		if (osMutexWait(RemoteBufferMutexHandle, 0) == osOK)
		{
			//Log("U2CB-RBM-WE");
			for (int i = 0; i < REM_BUF_SIZE; i++)
				RemoteBuffer[i] = Uart2Buffer[i];

			osMutexRelease(RemoteBufferMutexHandle);

			// Signal to TaskTemote
			osSemaphoreRelease(RemoteBufferFullSemaphoreHandle);
		}


//		if (RemoteBufferInProgress)
//		{
//			// If we are just getting the header bytes or the actual data
//			if ((RemoteBufferIndex == 0 && Uart2Buffer == 0x20)
//					|| (RemoteBufferIndex == 1 && Uart2Buffer == 0x40)
//					|| (1 < RemoteBufferIndex && RemoteBufferIndex < REM_BUF_SIZE))
//			{
//				//Log("U2CB-F");
//				RemoteBuffer[RemoteBufferIndex] = Uart2Buffer;
//
//				if (RemoteBufferIndex < REM_BUF_SIZE-1)
//					RemoteBufferIndex++;
//				else
//				{
//					RemoteBufferIndex = 0;
//					RemoteBufferInProgress = false;
//
//					//Log("U2CB-RBFS-RS");
//					// Signal to TaskRemote with the binary semaphore
//					osSemaphoreRelease(RemoteBufferFullSemaphoreHandle);
//					//Log("U2CB-RBFS-RE");
//				}
//			}
//			else
//			{
//				RemoteBufferIndex = 0;
//
//				char str[32];
//				sprintf(str, "UART Receive Error: [%d]\r\n", Uart2CallbackCounter);
//				HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
//			}
//		}

		HAL_UART_Receive_IT(&huart2, &Uart2Buffer, 64);
	}
	else if (huart == &huart4)
	{
		if ((GPSPackageIndex == 0 && Uart4Buffer == '$')
				|| (GPSPackageIndex == 1 && Uart4Buffer == 'G')
				|| (1 < GPSPackageIndex && GPSPackageIndex < GPS_BUFFSIZE))
		{
			GPSPackageBuffer[GPSPackageIndex] = Uart4Buffer;

			if (GPSPackageIndex < GPS_BUFFSIZE-1)
				GPSPackageIndex++;
			else
			{
				GPSPackageIndex = 0;
				ProcessGPSPackageBuffer = true;

				osSemaphoreRelease(GpsBufferSemaphoreHandle);
			}
		}
		else
		{
			GPSPackageIndex = 0;

			// TODO
			//char str[32];
			//sprintf(str, "UART Receive Error: GPS\r\n");
			//HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
		}

		HAL_UART_Receive_DMA(&huart4, &Uart4Buffer, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		if (huart->ErrorCode != 0)
		{
			//Diag = false;

			char str[32];
			sprintf(str, "UART2 Error Callback: %d\r\n", huart->ErrorCode);
			HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
		}
	}
	else if (huart == &huart4)
	{
		if (huart->ErrorCode != 0)
		{
			Diag = false;

			char str[32];
			sprintf(str, "UART4 Error Callback: %lu\r\n", huart->ErrorCode);
			HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		HCSR04_TMR_IC_ISR(&HCSR04, htim);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Init IMU
  	if(MPU_Init(&hspi2, &MPU9250) == 0 && BMP280_initialize(&hspi2, &BMP280) == 0)
  		IsImuAvailable = true;
  	else
  		IsImuAvailable = false;

  	// Init Magnetometer
  	if (HMC5883L_Init() == 0)
  		IsMagnAvailable = true;
  	else
  		IsMagnAvailable = false;

  	// Init Distance sensor
  	if (HCSR04_Init(&HCSR04, &htim3) == 0)
  		IsDistAvailable = false;
  	else
  		IsDistAvailable = false;

  	// Init GPS
  //	if (GPS_Init() == 0)
  //		IsGpsAvailable = true;
  //	else
  //		IsGpsAvailable = false;

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MagnMutex */
  osMutexDef(MagnMutex);
  MagnMutexHandle = osMutexCreate(osMutex(MagnMutex));

  /* definition and creation of RemoteDataMutex */
  osMutexDef(RemoteDataMutex);
  RemoteDataMutexHandle = osMutexCreate(osMutex(RemoteDataMutex));

  /* definition and creation of ImuMutex */
  osMutexDef(ImuMutex);
  ImuMutexHandle = osMutexCreate(osMutex(ImuMutex));

  /* definition and creation of GpsDataMutex */
  osMutexDef(GpsDataMutex);
  GpsDataMutexHandle = osMutexCreate(osMutex(GpsDataMutex));

  /* definition and creation of DistMutex */
  osMutexDef(DistMutex);
  DistMutexHandle = osMutexCreate(osMutex(DistMutex));

  /* definition and creation of RemoteBufferMutex */
  osMutexDef(RemoteBufferMutex);
  RemoteBufferMutexHandle = osMutexCreate(osMutex(RemoteBufferMutex));

  /* USER CODE BEGIN RTOS_MUTEX */

	osMutexRelease(MagnMutexHandle);
	osMutexRelease(RemoteDataMutexHandle);
	osMutexRelease(ImuMutexHandle);
	osMutexRelease(GpsDataMutexHandle);
	osMutexRelease(RemoteBufferMutexHandle);

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of DistSemaphore */
  osSemaphoreDef(DistSemaphore);
  DistSemaphoreHandle = osSemaphoreCreate(osSemaphore(DistSemaphore), 1);

  /* definition and creation of GpsBufferSemaphore */
  osSemaphoreDef(GpsBufferSemaphore);
  GpsBufferSemaphoreHandle = osSemaphoreCreate(osSemaphore(GpsBufferSemaphore), 1);

  /* definition and creation of RemoteBufferFullSemaphore */
  osSemaphoreDef(RemoteBufferFullSemaphore);
  RemoteBufferFullSemaphoreHandle = osSemaphoreCreate(osSemaphore(RemoteBufferFullSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  if (osSemaphoreGetCount(DistSemaphoreHandle) == 1)
  	  osSemaphoreWait(DistSemaphoreHandle, osWaitForever);
  if (osSemaphoreGetCount(GpsBufferSemaphoreHandle) == 1)
  	  osSemaphoreWait(GpsBufferSemaphoreHandle, osWaitForever);

  if (osSemaphoreGetCount(RemoteBufferFullSemaphoreHandle) == 1)
  	  osSemaphoreWait(RemoteBufferFullSemaphoreHandle, osWaitForever);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskSensorData */
  osThreadDef(TaskSensorData, RunTaskSensorData, osPriorityRealtime, 0, 512);
  TaskSensorDataHandle = osThreadCreate(osThread(TaskSensorData), NULL);

  /* definition and creation of TaskController */
  osThreadDef(TaskController, RunTaskController, osPriorityAboveNormal, 0, 128);
  TaskControllerHandle = osThreadCreate(osThread(TaskController), NULL);

  /* definition and creation of TaskRemote */
  osThreadDef(TaskRemote, RunTaskRemote, osPriorityHigh, 0, 512);
  TaskRemoteHandle = osThreadCreate(osThread(TaskRemote), NULL);

  /* definition and creation of TaskMotor */
  osThreadDef(TaskMotor, RunTaskMotor, osPriorityNormal, 0, 128);
  TaskMotorHandle = osThreadCreate(osThread(TaskMotor), NULL);

  /* definition and creation of TaskPower */
  osThreadDef(TaskPower, RunTaskPower, osPriorityBelowNormal, 0, 128);
  TaskPowerHandle = osThreadCreate(osThread(TaskPower), NULL);

  /* definition and creation of TaskDiagnostics */
  osThreadDef(TaskDiagnostics, RunTaskDiagnostics, osPriorityLow, 0, 512);
  TaskDiagnosticsHandle = osThreadCreate(osThread(TaskDiagnostics), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20808DD4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2160-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESC_DOWN_OUT_GPIO_Port, ESC_DOWN_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI2_IMU_CSIMU_Pin|SPI2_IMU_CSBM_Pin|DIS_TRIG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPS_PPS_IN_Pin */
  GPIO_InitStruct.Pin = GPS_PPS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_PPS_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESC_DOWN_OUT_Pin */
  GPIO_InitStruct.Pin = ESC_DOWN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESC_DOWN_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_IMU_CSIMU_Pin SPI2_IMU_CSBM_Pin DIS_TRIG_OUT_Pin */
  GPIO_InitStruct.Pin = SPI2_IMU_CSIMU_Pin|SPI2_IMU_CSBM_Pin|DIS_TRIG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_RDY_IN_Pin */
  GPIO_InitStruct.Pin = MAG_RDY_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAG_RDY_IN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_RunTaskSensorData */
/**
  * @brief  Function implementing the TaskSensorData thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RunTaskSensorData */
void RunTaskSensorData(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TaskSensorData(argument);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_RunTaskController */
/**
* @brief Function implementing the TaskController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskController */
void RunTaskController(void const * argument)
{
  /* USER CODE BEGIN RunTaskController */
	TaskController(argument);
  /* USER CODE END RunTaskController */
}

/* USER CODE BEGIN Header_RunTaskRemote */
/**
* @brief Function implementing the TaskRemote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskRemote */
void RunTaskRemote(void const * argument)
{
  /* USER CODE BEGIN RunTaskRemote */
	TaskRemote(argument);
  /* USER CODE END RunTaskRemote */
}

/* USER CODE BEGIN Header_RunTaskMotor */
/**
* @brief Function implementing the TaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskMotor */
void RunTaskMotor(void const * argument)
{
  /* USER CODE BEGIN RunTaskMotor */
	TaskMotor(argument);
  /* USER CODE END RunTaskMotor */
}

/* USER CODE BEGIN Header_RunTaskPower */
/**
* @brief Function implementing the TaskPower thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskPower */
void RunTaskPower(void const * argument)
{
  /* USER CODE BEGIN RunTaskPower */
	TaskPower(argument);
  /* USER CODE END RunTaskPower */
}

/* USER CODE BEGIN Header_RunTaskDiagnostics */
/**
* @brief Function implementing the TaskDiagnostics thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskDiagnostics */
void RunTaskDiagnostics(void const * argument)
{
  /* USER CODE BEGIN RunTaskDiagnostics */
	TaskDiagnostics(argument);
  /* USER CODE END RunTaskDiagnostics */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
