#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern osSemaphoreId RemoteBufferSemaphoreHandle;
extern osMutexId RemoteDataMutexHandle;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Task Remote
// - Starts Interrupt UART communication with the Receiver
// - If a complete package of data has arrived from the Remote Controller to the Buffer,
//   it processes that and saves it to the corresponding Global variables.
void TaskRemote(void const *argument)
{
	static uint16_t channelValues[IBUS_MAXCHANNELS];// Output values of the channels (1000 ... 2000)

	HAL_UART_Receive_DMA(&huart2, &Uart2Buffer, 1);

	/* Infinite loop */
	while (1)
	{
		//Log("Rem - RBSemEnter");
		if (osSemaphoreWait(RemoteBufferSemaphoreHandle, osWaitForever) == osOK)
		{
			//Log("Rem - RBSemEntered");
			if (ProcessIbusPackageBuffer)
			{
				// And cycle through the raw data and convert it to actual integer values
				// ibus pattern example:
				// i=0  1     2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21  22 23 24  25  26 27  28 28  30 31
				//   20 40    DB 5  DC 5  54 5  DC 5  E8 3  D0 7  D2 5  E8 3  DC 5  DC 5   DC 5   DC 5   DC 5   DC 5   DA F3
				// | Header | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 | CH9 | CH10 | CH11 | CH12 | CH13 | CH14 | Checksum |
				for (int i = 0; i < IBUS_MAXCHANNELS; i++)
					channelValues[i] = (IbusPackageBuffer[3 + 2 * i] << 8) + IbusPackageBuffer[2 + 2 * i];

				// Setting the speed
				//Log("Rem - RDMutEnter");
				if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
				{
//					char str[10];
//					for (int i = 0; i < IBUS_MAXCHANNELS; i++)
//					{
//						sprintf(str, "%d ", channelValues[i]);
//						HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);
//					}
//					HAL_UART_Transmit(&huart3, "\r\n", sizeof("\r\n"), HAL_MAX_DELAY);

					//Log("Rem - RDMutEntered");
					Throttle_in = (channelValues[THROTTLE_CHANNEL] / 20) - 50;
					Pitch_in = (channelValues[PITCH_CHANNEL] / 20) - 75;
					Roll_in = (channelValues[ROLL_CHANNEL] / 20) - 75;
					Yaw_in = (channelValues[YAW_CHANNEL] / 20) - 75;
					SWA = (channelValues[SWA_CHANNEL] / 20) - 50;
					SWB = (channelValues[SWB_CHANNEL] / 20) - 50;
					SWC = (channelValues[SWC_CHANNEL] / 20) - 50;
					SWD = (channelValues[SWD_CHANNEL] / 20) - 50;
					VRA = (channelValues[VRA_CHANNEL] / 20) - 50;
					VRB = (channelValues[VRB_CHANNEL] / 20) - 50;

					//char str1[40];
					//sprintf(str1, "In: %d\r\n", channelValues[THROTTLE_CHANNEL]);
					//HAL_UART_Transmit(&huart3, str1, 11, HAL_MAX_DELAY);

					//Log("Rem - RDMutRelease");
					//osMutexRelease(RemoteDataMutexHandle);
					//Log("Rem - RDMutReleased");
				}
				osMutexRelease(RemoteDataMutexHandle);

				//char str2[40];
				//sprintf(str2, "Out: %d\r\n\r\n", channelValues[2]);
				//HAL_UART_Transmit(&huart3, str2, 13, HAL_MAX_DELAY);

				ProcessIbusPackageBuffer = false;
			}
		}

		osDelay(100);
	}
}
