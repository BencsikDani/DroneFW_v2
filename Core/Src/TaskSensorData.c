#include "GY-91/BMP280.h"
#include "GY-91/MPU9250.h"
#include "GY-271/HMC5883L.h"
#include "HCSR04/HCSR04.h"
#include "GPS/GPS.h"
#include "LPF.h"
#include "cmsis_os.h"
#include "math.h"
#include "Globals.h"
#include "Debug.h"
#include "string.h"

extern SPI_HandleTypeDef hspi2;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

extern osSemaphoreId DistSemaphoreHandle;
extern osSemaphoreId GpsBufferSemaphoreHandle;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsDataMutexHandle;

void TaskSensorData(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	bool Recalibrate = false;

	/*
	LPF GyroLPF[3];

	GyroLPF[0].T = 0.005;
	GyroLPF[0].f_cutoff = 100;
	LPF_Init(&(GyroLPF[0]));

	GyroLPF[1].T = 0.005;
	GyroLPF[1].f_cutoff = 100;
	LPF_Init(&(GyroLPF[1]));

	GyroLPF[2].T = 0.005;
	GyroLPF[2].f_cutoff = 100;
	LPF_Init(&(GyroLPF[2]));
	*/

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		// IMU Data
		if (IsImuAvailable)
		{
			Log("SD-RDM-WS");
			if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
			{
				Log("SD-RDM-WE");
				if (SWC > 990)
				{
					Recalibrate = true;
				}

			}
			Log("SD-RDM-RS");
			osMutexRelease(RemoteDataMutexHandle);
			Log("SD-RDM-RE");

			//MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);
			//MPU_readRawData(&hspi2, &MPU9250);
			MPU_calcAttitude(&hspi2, &MPU9250);

			BMP280_measure(&BMP280);

			Log("SD-IM-WS");
			if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
			{
				Log("SD-IM-WE");
				if (Recalibrate)
				{
					HAL_UART_Transmit(&huart3, "CALIBRATING...\r\n", strlen("CALIBRATING...\r\n"), HAL_MAX_DELAY);
					MPU_calibrateGyro(&hspi2, &MPU9250, 20);

					Recalibrate = false;
				}

				AccData[0] = MPU9250.sensorData.ax;
				AccData[1] = MPU9250.sensorData.ay;
				AccData[2] = MPU9250.sensorData.az;
				TempData = MPU9250.sensorData.temp;
				GyroData[0] = MPU9250.sensorData.gx;
				GyroData[1] = MPU9250.sensorData.gy;
				GyroData[2] = MPU9250.sensorData.gz;
				//GyroData[0] = LPF_Update(&(GyroLPF[0]), MPU9250.sensorData.gx);
				//GyroData[1] = LPF_Update(&(GyroLPF[1]), MPU9250.sensorData.gy);
				//GyroData[2] = LPF_Update(&(GyroLPF[2]), MPU9250.sensorData.gz);
				Roll_measured = MPU9250.attitude.roll;
				Pitch_measured = MPU9250.attitude.pitch;
				Yaw_measured = MPU9250.attitude.yaw;

				BMP_Temp = BMP280.measurement.temperature;
				BMP_Pres = BMP280.measurement.pressure;
				BMP_Alt = BMP280.measurement.altitude;

			}
			Log("SD-IM-RS");
			osMutexRelease(ImuMutexHandle);
			Log("SD-IM-RE");
		}

		// Magnetometer Data
		if (IsMagnAvailable)
		{
			Log("SD-MM-WS");
			if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
			{
				Log("SD-MM-WE");

				struct Vector res = HMC5883L_readRaw();
				MAG_X_RAW = res.XAxis;
				MAG_Y_RAW = res.YAxis;
				MAG_Z_RAW = res.ZAxis;

//				magnitude = sqrtf((float)(MAG_X_RAW * MAG_X_RAW)
//											+ (float)(MAG_Y_RAW * MAG_Y_RAW)
//											+ (float)(MAG_Z_RAW * MAG_Z_RAW));

				//MAG_X_NORM = MAG_X_RAW / magnitude;
				//MAG_Y_NORM = MAG_Y_RAW / magnitude;
				//MAG_Z_NORM = MAG_Z_RAW / magnitude;

				//MAG_dir = atan2f(MAG_X_NORM, MAG_Y_NORM)*180.0f/M_PI;

				if (MAG_Y_RAW != 0)
				{
					if (MAG_Y_RAW > 0)
						MAG_dir = 90.0f - (atan2f(MAG_X_RAW, MAG_Y_RAW)*180.0f/M_PI);
					else if (MAG_Y_RAW < 0)
						MAG_dir = 270.0f - (atan2f(MAG_X_RAW, MAG_Y_RAW)*180.0f/M_PI);
				}
				else if (MAG_Y_RAW == 0)
				{
					if (MAG_X_RAW > 0)
						MAG_dir = 0.0f;
					else if (MAG_X_RAW < 0)
						MAG_dir = 180.0f;

				}

				MAG_dir += declination;

				if (MAG_dir < 0)
					MAG_dir += 360.0f;
				if (MAG_dir > 360.0f)
					MAG_dir -= 360.0f;
			}
			Log("SD-MM-RS");
			osMutexRelease(MagnMutexHandle);
			Log("SD-MM-RE");
		}

		// Distance Data
		if (IsDistAvailable)
		{
			//Log("SD-DA");
			if (!HCSR04.Triggered)
			{
				HCSR04_Trigger(&HCSR04);
				HCSR04.Triggered = true;
			}
			else if (HCSR04.Triggered)
			{
				Log("SD-DS-WS");
				if (osSemaphoreWait(DistSemaphoreHandle, 0) == osOK)
				{
					Log("SD-DS-WE");
					Log("SD-DM-WS");
					if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
					{
						Log("SD-DM-WE");

						Distance = HCSR04_Read(&HCSR04);
					}
					Log("SD-DM-RS");
					osMutexRelease(DistMutexHandle);
					Log("SD-DM-RE");

					HCSR04.Triggered = false;
				}
			}
		}

		// GPS Data
		if (IsGpsAvailable)
		{
			//Log("SD-GA");
			if (osSemaphoreWait(GpsBufferSemaphoreHandle, osWaitForever) == osOK)
			{
				if (ProcessGPSPackageBuffer)
				{
					//HAL_UART_Transmit(&huart3, GPSPackageBuffer, GPS_BUFFSIZE, HAL_MAX_DELAY);
					//HAL_UART_Transmit(&huart3, "\r\n", sizeof("\r\n"), HAL_MAX_DELAY);

					if (osMutexWait(GpsDataMutexHandle, osWaitForever) == osOK)
					{
						if (GPS_validate((char*) GPSPackageBuffer))
							GPS_parse((char*) GPSPackageBuffer);
						memset(GPSPackageBuffer, 0, sizeof(GPSPackageBuffer));
					}
					osMutexRelease(GpsDataMutexHandle);

					ProcessGPSPackageBuffer = false;
				}
			}
		}

		//LogN(xTaskGetTickCount() - time);
	}
}
