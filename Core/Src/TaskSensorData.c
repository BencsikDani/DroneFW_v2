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
#include "Fusion/Fusion.h"
#include "time.h"

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

	// Low Pass Filters
	LPF GyroLPF[3];

	GyroLPF[0].T = 1.0 / xFrequency;
	GyroLPF[0].f_cutoff = 40;
	LPF_Init(&(GyroLPF[0]));

	GyroLPF[1].T = 1.0 / xFrequency;
	GyroLPF[1].f_cutoff = 40;
	LPF_Init(&(GyroLPF[1]));

	GyroLPF[2].T = 1.0 / xFrequency;
	GyroLPF[2].f_cutoff = 40;
	LPF_Init(&(GyroLPF[2]));


	// Fusion algorithm

	// Define calibration (replace with actual calibration data if available)
	const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
	const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
	const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

	// Initialize
	FusionOffset offset;
	FusionAhrs ahrs;

	FusionOffsetInitialise(&offset, xFrequency);
	FusionAhrsInitialise(&ahrs);

	// Set AHRS algorithm settings
	FusionAhrsSettings fusionSettings;
	fusionSettings.convention = FusionConventionNwu;
	fusionSettings.gain = 0.5f;
	fusionSettings.gyroscopeRange = 2000.0f;
	fusionSettings.accelerationRejection = 10.0f;
	fusionSettings.magneticRejection = 10.0f;
	fusionSettings.recoveryTriggerPeriod = 5.0f * xFrequency;
	FusionAhrsSetSettings(&ahrs, &fusionSettings);



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
			if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
			{
				if (SWC > 990)
					Recalibrate = true;
			}
			osMutexRelease(RemoteDataMutexHandle);

			MPU_readProcessedData(&hspi2, &MPU9250);
			BMP280_measure(&BMP280);

			if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
			{
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

				GyroData[0] = LPF_Calculate(&(GyroLPF[0]), MPU9250.sensorData.gx);
				GyroData[1] = LPF_Calculate(&(GyroLPF[1]), MPU9250.sensorData.gy);
				GyroData[2] = LPF_Calculate(&(GyroLPF[2]), MPU9250.sensorData.gz);

				BMP_Temp = BMP280.measurement.temperature;
				BMP_Pres = BMP280.measurement.pressure;
				BMP_Alt = BMP280.measurement.altitude;

			}
			osMutexRelease(ImuMutexHandle);
		}

		// Magnetometer Data
		if (IsMagnAvailable)
		{
			if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
			{
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
			osMutexRelease(MagnMutexHandle);
		}

		// Calculate Fusion algorithm
		if ((osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
			&& (osMutexWait(MagnMutexHandle, osWaitForever) == osOK))
		{
			// Acquire latest sensor data
			//const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp

			FusionVector accelerometer = {AccData[0], AccData[1], AccData[2]}; // accelerometer data in g
			FusionVector gyroscope = {GyroData[0], GyroData[1], GyroData[2]}; // gyroscope data in degrees/s
			FusionVector magnetometer = {MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW}; // magnetometer data in arbitrary units

			// Apply run-time calibration
			gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
			accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
			magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

			// Update gyroscope offset correction algorithm
			gyroscope = FusionOffsetUpdate(&offset, gyroscope);

			// Calculate delta time (in seconds) to account for gyroscope sample clock error
			//static clock_t previousTimestamp;
			//const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
			//previousTimestamp = timestamp;

			// Update gyroscope AHRS algorithm
			FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 1.0f / xFrequency);
			//FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0f / xFrequency);

			// Algorithm outputs
			Fusion_output = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		}
		osMutexRelease(ImuMutexHandle);
		osMutexRelease(MagnMutexHandle);

		// Distance Data
		if (IsDistAvailable)
		{
			if (!HCSR04.Triggered)
			{
				HCSR04_Trigger(&HCSR04);
				HCSR04.Triggered = true;
			}
			else if (HCSR04.Triggered)
			{
				if (osSemaphoreWait(DistSemaphoreHandle, 0) == osOK)
				{
					if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
						Distance = HCSR04_Read(&HCSR04);
					osMutexRelease(DistMutexHandle);

					HCSR04.Triggered = false;
				}
			}
		}

		// GPS Data
		if (IsGpsAvailable)
		{
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
