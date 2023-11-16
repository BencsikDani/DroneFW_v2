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
	PIDController* PID1 = &DPID_Roll.outer;
	int16_t PID1_ref_devided; // Calculated in every cycle
	float PID1_ref_devided_float; // Needed only for single loop tuning
	float* PID1_meas = &(Fusion_output.angle.roll);
	float* PID1_out = &(PID1->out);

	PIDController* PID2 = &DPID_Roll.inner;
	float* PID2_ref = &(PID1->out);
	//float* PID2_ref = &(Fusion_output.angle.roll);
	float* PID2_meas = GyroData;
	//float* PID2_meas = &Roll_measured;
	int16_t* PID2_out = &Roll_controlled;

	PIDController* PID3 = &DPID_Pitch.outer;
	int16_t PID3_ref_devided; // Calculated in every cycle
	float PID3_ref_devided_float; // Needed only for single loop tuning
	float* PID3_meas = &(Fusion_output.angle.pitch);
	float* PID3_out = &(PID3->out);

	PIDController* PID4 = &DPID_Pitch.inner;
	float* PID4_ref = &(PID3->out);
	float* PID4_meas = GyroData+1;
	int16_t* PID4_out = &Pitch_controlled;


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

				// Pack PID1 data
				//Gains
				FloatToUint8s(&(PID1->Kp), SpiTuneData1, 1);
				FloatToUint8s(&(PID1->Ki), SpiTuneData1, 5);
				FloatToUint8s(&(PID1->Kd), SpiTuneData1, 9);
				// Reference -> Calculated in every cycle
				//PID1_ref_devided = Roll_in / 25;
				PID1_ref_devided = SWD / 70;
				Int16ToUint8s(&PID1_ref_devided, SpiTuneData1, 13);
				// Measurement
				FloatToUint8s(PID1_meas, SpiTuneData1, 15);
				// Output
				FloatToUint8s(PID1_out, SpiTuneData1, 19);


				// Pack PID2 data
				// Gains
				FloatToUint8s(&(PID2->Kp), SpiTuneData1, 23);
				FloatToUint8s(&(PID2->Ki), SpiTuneData1, 27);
				FloatToUint8s(&(PID2->Kd), SpiTuneData1, 31);
				// Reference
				if (Tune_single_true_double_false)
				{
					PID1_ref_devided_float = Roll_in / 10.0;
					FloatToUint8s(&PID1_ref_devided_float, SpiTuneData1, 35);
				}
				else
					FloatToUint8s(PID2_ref, SpiTuneData1, 35);
				// Measurement
				FloatToUint8s(PID2_meas, SpiTuneData1, 39);
				// Output
				Int16ToUint8s(PID2_out, SpiTuneData1, 43);


				// Pack PID3 data
				//Gains
				FloatToUint8s(&(PID3->Kp), SpiTuneData2, 1);
				FloatToUint8s(&(PID3->Ki), SpiTuneData2, 5);
				FloatToUint8s(&(PID3->Kd), SpiTuneData2, 9);
				// Reference -> Calculated in every cycle
				PID3_ref_devided = Pitch_in / 25;
				Int16ToUint8s(&PID3_ref_devided, SpiTuneData2, 13);
				// Measurement
				FloatToUint8s(PID3_meas, SpiTuneData2, 15);
				// Output
				FloatToUint8s(PID3_out, SpiTuneData2, 19);


				// Pack PID4 data
				// Gains
				FloatToUint8s(&(PID4->Kp), SpiTuneData2, 23);
				FloatToUint8s(&(PID4->Ki), SpiTuneData2, 27);
				FloatToUint8s(&(PID4->Kd), SpiTuneData2, 31);
				// Reference
				if (Tune_single_true_double_false)
				{
					PID3_ref_devided_float = Pitch_in / 10.0;
					FloatToUint8s(&PID3_ref_devided_float, SpiTuneData2, 35);
				}
				else
					FloatToUint8s(PID4_ref, SpiTuneData2, 35);
				// Measurement
				FloatToUint8s(PID4_meas, SpiTuneData2, 39);
				// Output
				Int16ToUint8s(PID4_out, SpiTuneData2, 43);
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
			if (Spi1Buffer[1] == 1) // PID1
			{
				if (Spi1Buffer[2] == 'p')
					PID1->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID1->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID1->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 2) // PID2
			{
				if (Spi1Buffer[2] == 'p')
					PID2->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID2->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID2->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 3) // PID3
			{
				if (Spi1Buffer[2] == 'p')
					PID3->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID3->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID3->Kd = float_value;
			}
			else if (Spi1Buffer[1] == 4) // PID4
			{
				if (Spi1Buffer[2] == 'p')
					PID4->Kp = float_value;
				else if (Spi1Buffer[2] == 'i')
					PID4->Ki = float_value;
				else if (Spi1Buffer[2] == 'd')
					PID4->Kd = float_value;
			}
		}
		osMutexRelease(ControllerMutexHandle);

		//HAL_UART_Transmit(&huart3, Spi1Buffer, 64, HAL_MAX_DELAY);
	}
}
