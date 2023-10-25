#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

// Libraries
#include "main.h"
#include <stdint.h>
#include <math.h>

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_9250_ANS 0x70
#define WHO_AM_I          0x75
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_XOUT_H      0x3B
#define READWRITE         0x80
#define CS_SELECT         0
#define CS_DESELECT       1
#define SPI_TIMOUT_MS     1000
#define TEMP_SENS		  333.87

#define MAGN_X_OFFS_L     0x03
#define MAGN_X_OFFS_H     0x04
#define MAGN_Y_OFFS_L     0x05
#define MAGN_Y_OFFS_H     0x06
#define MAGN_Z_OFFS_L     0x07
#define MAGN_Z_OFFS_H     0x08

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Master structure
typedef struct MPU9250
{
	// Raw values in binary
    struct RawData
    {
        int16_t ax, ay, az, temp, gx, gy, gz, mx, my, mz;
    } rawData;

    // Converted values in...
    struct SensorData
    {
        float aScaleFactor; // LSB/g
        float ax, ay, az; // g

        float temp; //

        float gScaleFactor; // LSB/(deg/s)
        float gx, gy, gz; //  deg/s

		float mx, my, mz;
    } sensorData;

    struct GyroBias
    {
        float x, y, z;
    } gyroBias;

    struct Attitude
    {
        float tau; // Complementary filter ratio
		uint32_t lastTick; // Last tick value for calculating the time elapsed between sensor readings
		float dt; // Time elapsed between sensor readings
        float roll, pitch, yaw;
    } attitude;

    struct Settings
    {
    	uint8_t aFullScaleRange, gFullScaleRange;
    	GPIO_TypeDef *CS_PORT;
        uint16_t CS_PIN;
    } settings;
} MPU9250_t;

// Functions
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *mpuStruct);
uint8_t MPU_Init(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t *pAddr, uint8_t *pVal);
void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale);
void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
void MPU_calibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints);
void MPU_readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state);

#endif /* INC_MPU9250_H_ */
