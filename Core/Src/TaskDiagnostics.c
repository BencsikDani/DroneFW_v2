#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "printf.h"
#include "Globals.h"

#include "Debug.h"

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId RemoteBufferSemaphoreHandle;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsDataMutexHandle;
extern osMutexId ControllerMutexHandle;

void TaskDiagnostics(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	char UARTstr[512];
	uint8_t SpiIntData[64];
	uint8_t SpiFloatData1[64];
	uint8_t SpiFloatData2[64];

	SpiIntData[0] = (uint8_t)('i');
	SpiFloatData1[0] = (uint8_t)('f');
	SpiFloatData2[0] = (uint8_t)('g');

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			uint16_t Throttle1 = (uint16_t)(TIM1->CCR1-1000);
			uint16_t Throttle2 = (uint16_t)(TIM1->CCR2-1000);
			uint16_t Throttle3 = (uint16_t)(TIM1->CCR3-1000);
			uint16_t Throttle4 = (uint16_t)(TIM1->CCR4-1000);

			sprintf(UARTstr, "Throttle: (%d) %d %d %d %d\r\n", Throttle_in, Throttle1, Throttle2, Throttle3, Throttle4);
			Uint16ToUint8s(&Throttle_in, SpiIntData, 1);
			Uint16ToUint8s(&Throttle1, SpiIntData, 3);
			Uint16ToUint8s(&Throttle2, SpiIntData, 5);
			Uint16ToUint8s(&Throttle3, SpiIntData, 7);
			Uint16ToUint8s(&Throttle4, SpiIntData, 9);

			sprintf(UARTstr, "%sPitch: %d\r\n", UARTstr, Pitch_in);
			Int16ToUint8s(&Pitch_in, SpiIntData, 11);

			int16_t Roll_in_devided = Roll_in / 10;
			sprintf(UARTstr, "%sRoll: %d\r\n", UARTstr, Roll_in_devided);
			Int16ToUint8s(&Roll_in_devided, SpiIntData, 13);

			sprintf(UARTstr, "%sYaw: %d\r\n", UARTstr, Yaw_in);
			Int16ToUint8s(&Yaw_in, SpiIntData, 15);

			sprintf(UARTstr, "%sSWA: %d\r\n", UARTstr, SWA);
			Uint16ToUint8s(&SWA, SpiIntData, 17);

			sprintf(UARTstr, "%sSWB: %d\r\n", UARTstr, SWB);
			Uint16ToUint8s(&SWB, SpiIntData, 19);

			sprintf(UARTstr, "%sSWC: %d\r\n", UARTstr, SWC);
			Uint16ToUint8s(&SWC, SpiIntData, 21);

			sprintf(UARTstr, "%sSWD: %d\r\n", UARTstr, SWD);
			Uint16ToUint8s(&SWD, SpiIntData, 23);

			sprintf(UARTstr, "%sVRA: %d\r\n", UARTstr, VRA);
			Uint16ToUint8s(&VRA, SpiIntData, 25);

			sprintf(UARTstr, "%sVRB: %d\r\n", UARTstr, VRB);
			Uint16ToUint8s(&VRB, SpiIntData, 27);
		}
		osMutexRelease(RemoteDataMutexHandle);

		if (IsImuAvailable)
		{
			if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
			{
				sprintf(UARTstr,
						"%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\nRoll: %1.2f ° ; Pitch: %1.2f ° ; Yaw: %1.2f °\r\n",
						UARTstr,
						TempData,
						AccData[0], AccData[1], AccData[2],
						GyroData[0], GyroData[1], GyroData[2],
						Roll_measured, Pitch_measured, Yaw_measured);
				FloatToUint8s(&TempData, SpiFloatData1, 1);
				FloatToUint8s(AccData, SpiFloatData1, 5);
				FloatToUint8s(AccData+1, SpiFloatData1, 9);
				FloatToUint8s(AccData+2, SpiFloatData1, 13);
				FloatToUint8s(GyroData, SpiFloatData1, 17);
				FloatToUint8s(GyroData+1, SpiFloatData1, 21);
				FloatToUint8s(GyroData+2, SpiFloatData1, 25);

				sprintf(UARTstr,
						"%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
						UARTstr,
						BMP_Temp, BMP_Pres, BMP_Alt);
				FloatToUint8s(&BMP_Temp, SpiFloatData1, 29);
				FloatToUint8s(&BMP_Pres, SpiFloatData1, 33);
				FloatToUint8s(&BMP_Alt, SpiFloatData1, 37);
			}
			osMutexRelease(ImuMutexHandle);
		}

		if ( /*IsMagnAvailable*/ 0 )
		{
			if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
			{
				sprintf(UARTstr,
						"%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
						UARTstr,
						MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW, MAG_dir);
				FloatToUint8s(&MAG_X_RAW, SpiFloatData1, 41);
				FloatToUint8s(&MAG_Y_RAW, SpiFloatData1, 45);
				FloatToUint8s(&MAG_Z_RAW, SpiFloatData1, 49);
				FloatToUint8s(&MAG_dir, SpiFloatData1, 53);
			}
			osMutexRelease(MagnMutexHandle);
		}

		if (IsDistAvailable)
		{
			if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
			{
				sprintf(UARTstr, "%sDistance: %.0f mm\r\n", UARTstr, Distance);
				FloatToUint8s(&Distance, SpiFloatData1, 57);
			}
			osMutexRelease(DistMutexHandle);
		}

		if ( /*IsGpsAvailable */ 0 )
		{
			if (osMutexWait(GpsDataMutexHandle, osWaitForever) == osOK)
			{
				sprintf(UARTstr, "%sGPS:\tLat -> %.4f %c\r\n\tLong -> %.4f %c\r\n\tFix -> %d\r\n\tNOS -> %d\r\n\tHDOP -> %.4f\r\n\tAlt -> %.4f %c\r\n",
						UARTstr,
						GPS.dec_latitude, GPS.ns, GPS.dec_longitude, GPS.ew, GPS.fix, GPS.num_of_satelites, GPS.horizontal_dilution_of_precision, GPS.mean_sea_level_altitude, GPS.altitude_unit);
				FloatToUint8s(&GPS.dec_latitude, SpiFloatData2, 1);
				SpiIntData[28] = (uint8_t)GPS.ns;
				FloatToUint8s(&GPS.dec_longitude, SpiFloatData2, 5);
				SpiIntData[29] = (uint8_t)GPS.ew;
				SpiIntData[30] = (uint8_t)(GPS.fix & 0x000000ff);
				SpiIntData[31] = (uint8_t)(GPS.num_of_satelites & 0x000000ff);
				FloatToUint8s(&GPS.horizontal_dilution_of_precision, SpiFloatData2, 9);
				FloatToUint8s(&GPS.mean_sea_level_altitude, SpiFloatData2, 13);
				SpiIntData[32] = (uint8_t)GPS.altitude_unit;
			}
			osMutexRelease(GpsDataMutexHandle);
		}

		sprintf(UARTstr, "%s\r\n\r\n", UARTstr);

		// Sending log info
		if (Diag)
		{
			HAL_UART_Transmit(&huart3, UARTstr, strlen(UARTstr), HAL_MAX_DELAY);

			HAL_SPI_Transmit(&hspi1, SpiIntData, 64, HAL_MAX_DELAY);
			osDelay(10);
			HAL_SPI_Transmit(&hspi1, SpiFloatData1, 64, HAL_MAX_DELAY);
			osDelay(10);
			HAL_SPI_Transmit(&hspi1, SpiFloatData2, 64, HAL_MAX_DELAY);
		}

		//LogN(xTaskGetTickCount() - time);
	}
}
