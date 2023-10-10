#ifndef BMP280_H
#define BMP280_H

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>


#define BMP280_CHIP_ID 0x58

#define BMP280_SPI_MASK_WRITE 0b01111111

#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS	  0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS      0xF7
#define BMP280_REG_TEMP       0xFA
#define BMP280_REG_CTRL       0xF4
#define BMP280_REG_CALIB      0x88

// For reading all data (pressure + temperature) at once
#define BMP280_REG_DATA       0xF7

#define BMP280_RESET_VALUE    0xB6

enum Oversampling
{
	oversampling_skipped = 0b000,
	oversampling_x1 = 0b001,
	oversampling_x2 = 0b010,
	oversampling_x4 = 0b011,
	oversampling_x8 = 0b100,
	oversampling_x16 = 0b101
};

enum PowerMode
{
	mode_sleep = 0b00,
	mode_forced = 0b01,
	mode_normal = 0b11
};

enum StandbyTime
{
	standby_time_500us = 0b000,
	standby_time_62500us = 0b001,
	standby_time_125ms = 0b010,
	standby_time_250ms = 0b011,
	standby_time_500ms = 0b100,
	standby_time_1000ms = 0b101,
	standby_time_2000ms = 0b110,
	standby_time_4000ms = 0b111
};

enum FilterSetting
{
	filter_off = 0b000,
	filter_coeff_2 = 0b001,
	filter_coeff_4 = 0b010,
	filter_coeff_8 = 0b011,
	filter_coeff_16 = 0b100
};

typedef struct BMP280Measurement {
    float temperature;
    float pressure;
    float altitude;
} BMP280Measurement;

typedef struct BMP280CompensationParameters {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} BMP280CompensationParameters;

typedef struct BMP280 {
    SPI_HandleTypeDef *spiHandle;
    float p_reference;
    int32_t t_fine;
    BMP280Measurement measurement;
    BMP280CompensationParameters compensationParameters;
} BMP280_t;

uint8_t BMP280_initialize(SPI_HandleTypeDef *SPIx, BMP280_t *bmp280);
void BMP280_reset(BMP280_t *bmp280);
uint8_t BMP280_getID(BMP280_t *bmp280);
void BMP280_readCompensationParameters(BMP280_t *bmp280);
void BMP280_setReferencePressure(BMP280_t *bmp280, uint16_t samples, uint8_t delay);
void BMP280_setPressureOversampling(BMP280_t *bmp280, uint8_t osrs_p);
void BMP280_setTemperatureOversampling(BMP280_t *bmp280, uint8_t osrs_t);
void BMP280_setPowerMode(BMP280_t *bmp280, uint8_t mode);
void BMP280_setStandbyTime(BMP280_t *bmp280, uint8_t t_sb);
void BMP280_setFilterCoefficient(BMP280_t *bmp280, uint8_t filter);
void BMP280_measure(BMP280_t *bmp280);
uint8_t BMP280_readRegister(BMP280_t *bmp280, uint8_t address);
void BMP280_writeRegister(BMP280_t *bmp280, uint8_t address, uint8_t value);
void BMP280_readMBRegister(BMP280_t *bmp280, uint8_t address, uint8_t *values, uint8_t length);
uint8_t BMP280_spiReadWrite(BMP280_t *bmp280, uint8_t tx_message);
void BMP280_spiCSNhigh(BMP280_t *bmp280);
void BMP280_spiCSNlow(BMP280_t *bmp280);
int32_t BMP280_compensate_temperature(BMP280_t *bmp280, int32_t uncomp_temp);
uint32_t BMP280_compensate_pressure(BMP280_t *bmp280, int32_t uncomp_pres);


#endif /* BMP280_H */
