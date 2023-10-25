#include "main.h"
#include "Globals.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

extern UART_HandleTypeDef huart3;

void Log(const char* msg)
{
	if (DebugIsOn)
	{
		char str[100];

		sprintf(str, "%s\r\n", msg);
		HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
	}
}

void LogN(const uint32_t n)
{
	if (/* DebugIsOn*/ 1)
	{
		char str[100];

		sprintf(str, "%d\r\n", n);
		HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
	}
}
