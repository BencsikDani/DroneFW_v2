#ifndef GLOBALS_H
#define GLOBALS_H


#include "GY-91/BMP280.h"
#include "GY-91/MPU9250.h"
#include "GY-271/HMC5883L.h"
#include "HCSR04/HCSR04.h"
#include "GPS/GPS.h"
#include "stdio.h"
#include "stdbool.h"


// Transmitter channel details
#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 10 // My TX only has 10 channels, no point in polling the rest

#define GPS_BUFFSIZE  255       // GPS buffer size

// Transmitter channel numbers
#define THROTTLE_CHANNEL 3-1
#define YAW_CHANNEL 4-1
#define PITCH_CHANNEL 2-1
#define ROLL_CHANNEL 1-1
#define SWA_CHANNEL 5-1
#define SWB_CHANNEL 6-1
#define SWC_CHANNEL 7-1
#define SWD_CHANNEL 8-1
#define VRA_CHANNEL 9-1
#define VRB_CHANNEL 10-1


#define MPU9250_SPI hspi2

extern bool Diag;
extern int Uart2CallbackCounter;

extern bool Rotors;

extern bool IsImuAvailable;
extern bool IsMagnAvailable;
extern bool IsDistAvailable;
extern bool IsGpsAvailable;

// Transmitter channel variables
extern uint16_t Throttle_in;
extern uint16_t Throttle_controlled;
extern int16_t Yaw_in;
extern int16_t Yaw_controlled;
extern int16_t Pitch_in;
extern int16_t Pitch_controlled;
extern int16_t Roll_in;
extern int16_t Roll_controlled;
extern uint16_t SWA;
extern uint16_t SWB;
extern uint16_t SWC;
extern uint16_t SWD;
extern uint16_t VRA;
extern uint16_t VRB;

extern MPU9250_t MPU9250;
extern float AccData[3];
extern float TempData;
extern float GyroData[3];
extern int16_t MagData[3];

extern BMP280_t BMP280;
extern float BMP_Temp;
extern float BMP_Pres;
extern float BMP_Alt;

extern float mG_per_LSB;
extern Vector v;
extern int xOffset, yOffset;
extern float declination;
extern float MAG_X_RAW;
extern float MAG_Y_RAW;
extern float MAG_Z_RAW;
extern float MAG_X_NORM;
extern float MAG_Y_NORM;
extern float MAG_Z_NORM;
extern float magnitude;
extern float MAG_dir;

extern HCSR04_t HCSR04;
extern float Distance;

extern volatile uint8_t Uart2Buffer;
extern volatile uint8_t IbusPackageIndex;	// Current position in the ibus packet
extern volatile uint8_t IbusPackageBuffer[IBUS_BUFFSIZE];	// Ibus packet buffer
extern volatile bool ProcessIbusPackageBuffer;

extern GPS_t GPS;
extern volatile uint8_t Uart4Buffer;
extern volatile uint8_t GPSPackageIndex;
extern volatile uint8_t GPSPackageBuffer[GPS_BUFFSIZE];
extern volatile bool ProcessGPSPackageBuffer;


#endif /* GLOBALS_H */
