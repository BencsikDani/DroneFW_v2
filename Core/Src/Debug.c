#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

extern UART_HandleTypeDef huart5;

static bool DebugIsOn = true;

void Log(const char* msg)
{
	if (DebugIsOn)
	{
		char str[100];

		sprintf(str, "%s\r\n", msg);
		HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);
	}
}
