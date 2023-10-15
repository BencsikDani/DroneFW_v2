#include "GY-91/BMP280.h"
#include "GY-91/MPU9250.h"
#include "GY-271/HMC5883L.h"
#include "HCSR04/HCSR04.h"
#include "GPS/GPS.h"
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
	/* Infinite loop */
	while (1)
	{
		// IMU Data
		if (IsImuAvailable)
		{
			//Log("SD-IA");
			//Log("SD-IMW-S");
			if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
			{
				//Log("SD-IMW-E");

				//MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);
				//MPU_readRawData(&hspi2, &MPU9250);
				MPU_readProcessedData(&hspi2, &MPU9250);

				AccData[0] = MPU9250.sensorData.ax;
				AccData[1] = MPU9250.sensorData.ay;
				AccData[2] = MPU9250.sensorData.az;
				TempData = MPU9250.sensorData.temp;
				GyroData[0] = MPU9250.sensorData.gx;
				GyroData[1] = MPU9250.sensorData.gy;
				GyroData[2] = MPU9250.sensorData.gz;

				BMP280_measure(&BMP280);

				BMP_Temp = BMP280.measurement.temperature;
				BMP_Pres = BMP280.measurement.pressure;
				BMP_Alt = BMP280.measurement.altitude;

				//Log("SD-IMR-S");
			}
			osMutexRelease(ImuMutexHandle);
			//Log("SD-IMR-E");
		}

		// Magnetometer Data
		if (IsMagnAvailable)
		{
			//Log("SD-MA");
			//Log("SD-MMW-S");
			if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
			{
				//Log("SD-MMW-E");

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

				//Log("SD-MMR-S");
			}
			osMutexRelease(MagnMutexHandle);
			//Log("SD-MMR-E");
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
				//Log("SD-DSW-S");
				if (osSemaphoreWait(DistSemaphoreHandle, osWaitForever) == osOK)
				{
					//Log("SD-DSW-E");
					//Log("SD-DMW-S");
					if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
					{
						//Log("SD-DMW-E");

						Distance = HCSR04_Read(&HCSR04);

						//Log("SD-DMR-S");
					}
					osMutexRelease(DistMutexHandle);
					//Log("SD-DMR-E");

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

		osDelay(100);
	}
}
