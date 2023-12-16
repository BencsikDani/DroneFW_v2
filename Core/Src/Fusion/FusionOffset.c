/**
 * @file FusionOffset.c
 * @author Seb Madgwick
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

//------------------------------------------------------------------------------
// Includes

#include "Fusion/FusionAhrs.h"
#include "Fusion/FusionOffset.h"
#include <math.h> // fabs

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Cutoff frequency in Hz.
 */
#define CUTOFF_FREQUENCY (0.02f)

/**
 * @brief Timeout in seconds.
 */
#define TIMEOUT (5)

/**
 * @brief Threshold in degrees per second.
 */
#define THRESHOLD (3.0f)

//------------------------------------------------------------------------------
// Functions

void FusionOffsetUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, uint16_t numCalPoints)
{
	// Init
	FusionEuler temp;
	float x = 0;
	float y = 0;
	float z = 0;

	// Zero guard
	if (numCalPoints == 0)
	{
		numCalPoints = 1;
	}

	// Save specified number of points
	for (uint16_t ii = 0; ii < numCalPoints; ii++)
	{
		FusionAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, 0.005);
		temp = FusionQuaternionToEuler(FusionAhrsGetQuaternion(ahrs));
		x += temp.angle.roll;
		y += temp.angle.pitch;
		z += temp.angle.yaw;

		HAL_Delay(5);
	}

    // Average the saved data points to find the fusion offset
	ahrs->fusionEulerOffset.angle.roll = x / (float)numCalPoints;
	ahrs->fusionEulerOffset.angle.pitch = y / (float)numCalPoints;
	ahrs->fusionEulerOffset.angle.yaw = z / (float)numCalPoints;
}

FusionEuler FusionApplyEulerOffset(FusionEuler uncalibrated, FusionEuler eulerOffset)
{
	const FusionEuler result = {.angle = {
			.roll = uncalibrated.angle.roll - eulerOffset.angle.roll,
			.pitch = uncalibrated.angle.pitch - eulerOffset.angle.pitch,
			.yaw = uncalibrated.angle.yaw - eulerOffset.angle.yaw,
	    }};
	return result;
}


/**
 * @brief Initialises the gyroscope offset algorithm.
 * @param offset Gyroscope offset algorithm structure.
 * @param sampleRate Sample rate in Hz.
 */
void FusionGyroOffsetInitialise(FusionGyroOffset *const offset, const unsigned int sampleRate) {
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO;
}

/**
 * @brief Updates the gyroscope offset algorithm and returns the corrected
 * gyroscope measurement.
 * @param offset Gyroscope offset algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @return Corrected gyroscope measurement in degrees per second.
 */
FusionVector FusionGyroOffsetUpdate(FusionGyroOffset *const offset, FusionVector gyroscope) {

    // Subtract offset from gyroscope measurement
    gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);

    // Reset timer if gyroscope not stationary
    if ((fabs(gyroscope.axis.x) > THRESHOLD) || (fabs(gyroscope.axis.y) > THRESHOLD) || (fabs(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }

    // Increment timer while gyroscope stationary
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }

    // Adjust offset if timer has elapsed
    offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset, FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    return gyroscope;
}

//------------------------------------------------------------------------------
// End of file
