#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern osSemaphoreId RemoteBufferFullSemaphoreHandle;
extern osMutexId RemoteBufferMutexHandle;
extern osMutexId RemoteDataMutexHandle;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Task Remote
// - Starts Interrupt UART communication with the Receiver
// - If a complete package of data has arrived from the Remote Controller to the Buffer,
//   it processes that and saves it to the corresponding Global variables.
void TaskRemote(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	static uint16_t channelValues[IBUS_MAXCHANNELS];// Output values of the channels (1000 ... 2000)

	HAL_UART_Receive_DMA(&huart2, &Uart2Buffer, 64);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		if (osSemaphoreWait(RemoteBufferFullSemaphoreHandle, osWaitForever) == osOK)
		{
			//Find the last complete 32 bit iBus packet in the 64 bit RemoteBuffer
			if (osMutexWait(RemoteBufferMutexHandle, osWaitForever) == osOK)
			{
				for (int i = 32; i >= 0; i--)
				{
					if (RemoteBuffer[i] == 0x20 && RemoteBuffer[i+1] == 0x40)
					{
						for (int j = 0; j < IBUS_PACKET_SIZE; j++)
							LastIbusPacket[j] = RemoteBuffer[i+j];
					}
				}
				osMutexRelease(RemoteBufferMutexHandle);
			}


			// And cycle through the raw data and convert it to actual integer values
			// ibus pattern example:
			// i=0  1     2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21  22 23 24  25  26 27  28 28  30 31
			//   20 40    DB 5  DC 5  54 5  DC 5  E8 3  D0 7  D2 5  E8 3  DC 5  DC 5   DC 5   DC 5   DC 5   DC 5   DA F3
			// | Header | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 | CH9 | CH10 | CH11 | CH12 | CH13 | CH14 | Checksum |
			for (int i = 0; i < IBUS_MAXCHANNELS; i++)
				channelValues[i] = (LastIbusPacket[3 + 2 * i] << 8) + LastIbusPacket[2 + 2 * i];

			// Setting the speed
			if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
			{
				Throttle_in = channelValues[THROTTLE_CHANNEL] - 1000;
				// Limit input Throttle, so the Controllers can keep the drone stable at high RPM, too
				if (Throttle_in > 800)
					Throttle_in = 800;

				Pitch_in = channelValues[PITCH_CHANNEL] - 1500;
				Roll_in = channelValues[ROLL_CHANNEL] - 1500;
				Yaw_in = channelValues[YAW_CHANNEL] - 1500;

				SWA = channelValues[SWA_CHANNEL] - 1000;
				SWB = channelValues[SWB_CHANNEL] - 1000;
				SWC = channelValues[SWC_CHANNEL] - 1000;
				SWD = channelValues[SWD_CHANNEL] - 1000;
				VRA = channelValues[VRA_CHANNEL] - 1000;
				VRB = channelValues[VRB_CHANNEL] - 1000;
			}
			osMutexRelease(RemoteDataMutexHandle);
		}

		//LogN(xTaskGetTickCount() - time);
	}
}
