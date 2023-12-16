/**
 * @file FusionOffset.h
 * @author Seb Madgwick
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

#ifndef FUSION_OFFSET_H
#define FUSION_OFFSET_H

//------------------------------------------------------------------------------
// Includes

#include "Fusion/FusionMath.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Gyroscope offset algorithm structure.  Structure members are used
 * internally and must not be accessed by the application.
 */
typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    FusionVector gyroscopeOffset;
} FusionGyroOffset;

//------------------------------------------------------------------------------
// Function declarations

void FusionOffsetUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, uint16_t numCalPoints);

FusionEuler FusionApplyEulerOffset(FusionEuler uncalibrated, FusionEuler eulerOffset);

void FusionGyroOffsetInitialise(FusionGyroOffset *const offset, const unsigned int sampleRate);

FusionVector FusionGyroOffsetUpdate(FusionGyroOffset *const offset, FusionVector gyroscope);

#endif

//------------------------------------------------------------------------------
// End of file
