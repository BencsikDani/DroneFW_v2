#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern osMutexId ControllerMutexHandle;
extern osMutexId ImuMutexHandle;;
extern osMutexId RemoteDataMutexHandle;


void TaskTune(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	//HAL_SPI_Receive_IT(&hspi1, Spi1Buffer, 64);

	uint8_t SpiTuneData1[64];
	uint8_t SpiTuneData2[64];
	SpiTuneData1[0] = (uint8_t)('t');
	SpiTuneData2[0] = (uint8_t)('u');


	// PID controllers to be tuned
	PIDController* PID_Ro = &DPID_Roll.outer;
	int16_t PID_Ro_ref_devided; // Calculated in every cycle
	float PID_Ro_ref_devided_float; // Needed only for single loop tuning
	float* PID_Ro_meas = &(Fusion_output.angle.roll);
	float* PID_Ro_out = &(PID_Ro->out);

	PIDController* PID_Ri = &DPID_Roll.inner;
	float* PID_Ri_ref = &(PID_Ro->out);
	//float* PID_Ri_ref = &(Fusion_output.angle.roll);
	float* PID_Ri_meas = GyroData;
	//float* PID_Ri_meas = &Roll_measured;
	int16_t* PID_Ri_out = &Roll_controlled;

	PIDController* PID_Po = &DPID_Pitch.outer;
	int16_t PID_Po_ref_devided; // Calculated in every cycle
	float PID_Po_ref_devided_float; // Needed only for single loop tuning
	float* PID_Po_meas = &(Fusion_output.angle.pitch);
	float* PID_Po_out = &(PID_Po->out);

	PIDController* PID_Pi = &DPID_Pitch.inner;
	float* PID_Pi_ref = &(PID_Po->out);
	float* PID_Pi_meas = GyroData+1;
	int16_t* PID_Pi_out = &Pitch_controlled;

	PIDController* PID_Y = &PID_Yaw;
	int16_t PID_Y_ref_devided; // Calculated in every cycle
	float* PID_Y_meas = GyroData+2;
	int16_t* PID_Y_out = &Yaw_controlled;


	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		if (Tune)
		{
			if (osMutexWait(ControllerMutexHandle, osWaitForever) == osOK
					&& osMutexWait(ImuMutexHandle, osWaitForever) == osOK
					&& osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
			{
//				if (SWD < 10)
//					Tune_single_true_double_false = false;
//				else
//					Tune_single_true_double_false = true;

				// Pack PID_Ro data
				//Gains
				FloatToUint8s(&(PID_Ro->Kp), SpiTuneData1, 1);
				FloatToUint8s(&(PID_Ro->Ki), SpiTuneData1, 5);
				FloatToUint8s(&(PID_Ro->Kd), SpiTuneData1, 9);
				// Reference -> Calculated in every cycle
				//PID1_ref_devided = Roll_in / 25;
				PID_Ro_ref_devided = SWD / 70;
				Int16ToUint8s(&PID_Ro_ref_devided, SpiTuneData1, 13);
				// Measurement
				FloatToUint8s(PID_Ro_meas, SpiTuneData1, 15);
				// Output
				FloatToUint8s(PID_Ro_out, SpiTuneData1, 19);


				// Pack PID_Ri data
				// Gains
				FloatToUint8s(&(PID_Ri->Kp), SpiTuneData1, 23);
				FloatToUint8s(&(PID_Ri->Ki), SpiTuneData1, 27);
				FloatToUint8s(&(PID_Ri->Kd), SpiTuneData1, 31);
				// Reference
				if (Tune_single_true_double_false)
				{
					PID_Ro_ref_devided_float = Roll_in / 10.0;
					FloatToUint8s(&PID_Ro_ref_devided_float, SpiTuneData1, 35);
				}
				else
					FloatToUint8s(PID_Ri_ref, SpiTuneData1, 35);
				// Measurement
				FloatToUint8s(PID_Ri_meas, SpiTuneData1, 39);
				// Output
				Int16ToUint8s(PID_Ri_out, SpiTuneData1, 43);


				// Pack PID_Po data
				//Gains
				FloatToUint8s(&(PID_Po->Kp), SpiTuneData2, 1);
				FloatToUint8s(&(PID_Po->Ki), SpiTuneData2, 5);
				FloatToUint8s(&(PID_Po->Kd), SpiTuneData2, 9);
				// Reference -> Calculated in every cycle
				PID_Po_ref_devided = Pitch_in / 25;
				Int16ToUint8s(&PID_Po_ref_devided, SpiTuneData2, 13);
				// Measurement
				FloatToUint8s(PID_Po_meas, SpiTuneData2, 15);
				// Output
				FloatToUint8s(PID_Po_out, SpiTuneData2, 19);


				// Pack PID_Pi data
				// Gains
				FloatToUint8s(&(PID_Pi->Kp), SpiTuneData2, 23);
				FloatToUint8s(&(PID_Pi->Ki), SpiTuneData2, 27);
				FloatToUint8s(&(PID_Pi->Kd), SpiTuneData2, 31);
				// Reference
				if (Tune_single_true_double_false)
				{
					PID_Po_ref_devided_float = Pitch_in / 10.0;
					FloatToUint8s(&PID_Po_ref_devided_float, SpiTuneData2, 35);
				}
				else
					FloatToUint8s(PID_Pi_ref, SpiTuneData2, 35);
				// Measurement
				FloatToUint8s(PID_Pi_meas, SpiTuneData2, 39);
				// Output
				Int16ToUint8s(PID_Pi_out, SpiTuneData2, 43);


				// Pack PID_Y data
				// Gains
				FloatToUint8s(&(PID_Y->Kp), SpiTuneData1, 45);
				FloatToUint8s(&(PID_Y->Ki), SpiTuneData1, 49);
				FloatToUint8s(&(PID_Y->Kd), SpiTuneData1, 53);
				// Reference -> Calculated in every cycle
				PID_Y_ref_devided = Yaw_in / 10;
				Int16ToUint8s(&PID_Y_ref_devided, SpiTuneData2, 45);
				// Measurement
				FloatToUint8s(PID_Y_meas, SpiTuneData2, 47);
				// Output
				Int16ToUint8s(PID_Y_out, SpiTuneData2, 51);

			}
			osMutexRelease(ControllerMutexHandle);
			osMutexRelease(ImuMutexHandle);
			osMutexRelease(RemoteDataMutexHandle);
		}

		// Send packed data and receive tune data
		HAL_SPI_TransmitReceive(&hspi1, SpiTuneData1, Spi1Buffer, 64, HAL_MAX_DELAY);
		osDelay(5);
		HAL_SPI_TransmitReceive(&hspi1, SpiTuneData2, Spi1Buffer, 64, HAL_MAX_DELAY);

		// Tune controllers based on the received data
		float float_value = 0;
		FloatFromUint8s(Spi1Buffer, 3, &float_value);

		if (osMutexWait(ControllerMutexHandle, osWaitForever) == osOK)
		{
			if (Spi1Buffer[1] == 1) // PID_Ro
			{
				if (Spi1Buffer[2] == 'p')
					PID_Ro->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID_Ro->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID_Ro->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 2) // PID_Ri
			{
				if (Spi1Buffer[2] == 'p')
					PID_Ri->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID_Ri->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID_Ri->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 3) // PID_Po
			{
				if (Spi1Buffer[2] == 'p')
					PID_Po->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID_Po->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID_Po->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 4) // PID_Pi
			{
				if (Spi1Buffer[2] == 'p')
					PID_Pi->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID_Pi->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID_Pi->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 5) // PID_Y
			{
				if (Spi1Buffer[2] == 'p')
					PID_Y->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID_Y->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID_Y->Kd = float_value;
			}
		}
		osMutexRelease(ControllerMutexHandle);
	}
}
