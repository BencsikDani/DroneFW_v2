#include <GY-91/BMP280.h>
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdint.h>


/** Initialize the device with desired configuration
 * @return 1, if device is not recognized, 0 otherwise.
 * */
uint8_t BMP280_initialize(SPI_HandleTypeDef *SPIx, BMP280_t *bmp280)
{
	// Disable MPU9250
	HAL_GPIO_WritePin(SPI2_IMU_CSIMU_GPIO_Port, SPI2_IMU_CSIMU_Pin, GPIO_PIN_SET);

	bmp280->spiHandle = SPIx;

    if (BMP280_getID(bmp280) != BMP280_CHIP_ID)
    {
        return 1;
    }

    // Reset device and wait
    BMP280_reset(bmp280);
    HAL_Delay(500);

    // BEGIN OF CONFIGURATION ----------------------------------
    BMP280_setPressureOversampling(bmp280, oversampling_x16);
    BMP280_setTemperatureOversampling(bmp280, oversampling_x2);

    BMP280_setPowerMode(bmp280, mode_normal);
    BMP280_setFilterCoefficient(bmp280, filter_coeff_16);
    BMP280_setStandbyTime(bmp280, standby_time_500us);
    // END OF CONFIGURATION --------------------------

    BMP280_readCompensationParameters(bmp280);
    BMP280_setReferencePressure(bmp280, 100, 50);

    return 0;
}

/** Perform power-on reset procedure */
void BMP280_reset(BMP280_t *bmp280)
{
    BMP280_writeRegister(bmp280, BMP280_REG_RESET, BMP280_RESET_VALUE);
}

/**
 * Read chip identification number.
 * @return chip ID
 * */
uint8_t BMP280_getID(BMP280_t *bmp280)
{
    return BMP280_readRegister(bmp280, BMP280_REG_ID);
}

void BMP280_readCompensationParameters(BMP280_t *bmp280)
{
    uint8_t buf[24];
    BMP280_readMBRegister(bmp280, BMP280_REG_CALIB, buf, 24);
    bmp280->compensationParameters.dig_t1 = ((buf[1] << 8) | buf[0]);
    bmp280->compensationParameters.dig_t2 = ((buf[3] << 8) | buf[2]);
    bmp280->compensationParameters.dig_t3 = ((buf[5] << 8) | buf[4]);
    bmp280->compensationParameters.dig_p1 = ((buf[7] << 8) | buf[6]);
    bmp280->compensationParameters.dig_p2 = ((buf[9] << 8) | buf[8]);
    bmp280->compensationParameters.dig_p3 = ((buf[11] << 8) | buf[10]);
    bmp280->compensationParameters.dig_p4 = ((buf[13] << 8) | buf[12]);
    bmp280->compensationParameters.dig_p5 = ((buf[15] << 8) | buf[14]);
    bmp280->compensationParameters.dig_p6 = ((buf[17] << 8) | buf[16]);
    bmp280->compensationParameters.dig_p7 = ((buf[19] << 8) | buf[18]);
    bmp280->compensationParameters.dig_p8 = ((buf[21] << 8) | buf[20]);
    bmp280->compensationParameters.dig_p9 = ((buf[23] << 8) | buf[22]);
}

/**
 * Set reference pressure for altitude calculation by averaging pressure measurements.
 * @param samples: Number of measurements to average.
 * @param delay: Delay between measurements (in ms).
 * */
void BMP280_setReferencePressure(BMP280_t *bmp280, uint16_t samples, uint8_t delay)
{
    HAL_Delay(500);
    float sum = 0;
    for (int i = 0; i < samples; i++)
    {
        BMP280_measure(bmp280);
        sum += bmp280->measurement.pressure;
        HAL_Delay(delay);
    }
    bmp280->p_reference = sum / samples;
}

/** Configure pressure oversampling */
void BMP280_setPressureOversampling(BMP280_t *bmp280, uint8_t osrs_p)
{
    uint8_t ctrl = BMP280_readRegister(bmp280, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b11100011) | (osrs_p << 2);
    BMP280_writeRegister(bmp280, BMP280_REG_CTRL, ctrl);
}

/** Configure temperature oversampling */
void BMP280_setTemperatureOversampling(BMP280_t *bmp280, uint8_t osrs_t)
{
    uint8_t ctrl = BMP280_readRegister(bmp280, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b00011111) | (osrs_t << 5);
    BMP280_writeRegister(bmp280, BMP280_REG_CTRL, ctrl);
}

/** Configure power mode */
void BMP280_setPowerMode(BMP280_t *bmp280, uint8_t mode)
{
    uint8_t ctrl = BMP280_readRegister(bmp280, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b11111100) | mode;
    BMP280_writeRegister(bmp280, BMP280_REG_CTRL, ctrl);
}

/** Configure standby time */
void BMP280_setStandbyTime(BMP280_t *bmp280, uint8_t t_sb)
{
    uint8_t conf = BMP280_readRegister(bmp280, BMP280_REG_CONFIG);
    conf = (conf & 0b00011111) | (t_sb << 5);
    BMP280_writeRegister(bmp280, BMP280_REG_CONFIG, conf);
}

/** Configure IIR filter */
void BMP280_setFilterCoefficient(BMP280_t *bmp280, uint8_t filter)
{
    uint8_t conf = BMP280_readRegister(bmp280, BMP280_REG_CONFIG);
    conf = (conf & 0b11100011) | (filter << 2);
    BMP280_writeRegister(bmp280, BMP280_REG_CONFIG, conf);
}

/* ---------------------------------------------------------------------------
 * Measurements and compensation ---------------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * Read latest measurement from sensor and execute compensation.
 * Stores the results in measurement member variable.
 * */
void BMP280_measure(BMP280_t *bmp280)
{
    uint8_t data[6];
    BMP280_readMBRegister(bmp280, BMP280_REG_DATA, data, 6);

    int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    int32_t adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;

    bmp280->measurement.temperature = (float)BMP280_compensate_temperature(bmp280, adc_T) / 100.0;
    bmp280->measurement.pressure = (float)BMP280_compensate_pressure(bmp280, adc_P) / 256.0;

    if (bmp280->p_reference > 0)
    {
        bmp280->measurement.altitude = (1.0 - pow(bmp280->measurement.pressure / bmp280->p_reference, 0.1903)) * 4433076.0;
    }
}

/* ---------------------------------------------------------------------------
 * Register read/write definitions -------------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * Read a register
 * @param address: Register address.
 * @return Register value.
 * */
uint8_t BMP280_readRegister(BMP280_t *bmp280, uint8_t address)
{
    BMP280_spiCSNlow(bmp280);
    BMP280_spiReadWrite(bmp280, address);
    uint8_t value = BMP280_spiReadWrite(bmp280, 0);
    BMP280_spiCSNhigh(bmp280);
    return value;
}

/**
 * Write to a register
 * @param address: Register address.
 * @param value: Value to write.
 * */
void BMP280_writeRegister(BMP280_t *bmp280, uint8_t address, uint8_t value)
{
    BMP280_spiCSNlow(bmp280);
    BMP280_spiReadWrite(bmp280, address & BMP280_SPI_MASK_WRITE);
    BMP280_spiReadWrite(bmp280, value);
    BMP280_spiCSNhigh(bmp280);
}

/**
 * Read a multi-byte register
 * @param address: Register address.
 * @param values: Array pointer to store values in.
 * @param length: Number of bytes to read.
 * */
void BMP280_readMBRegister(BMP280_t *bmp280, uint8_t address, uint8_t *values, uint8_t length)
{
    BMP280_spiCSNlow(bmp280);
    BMP280_spiReadWrite(bmp280, address);
    while (length--)
    {
        *values++ = BMP280_spiReadWrite(bmp280, 0);
    }
    BMP280_spiCSNhigh(bmp280);
}

/* ---------------------------------------------------------------------------
 * SPI interface definitions -------------------------------------------------
 * (ADAPT THESE METHODS TO YOUR HARDWARE) ------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * SPI transmit and receive one byte simultaneously
 * @param tx_message: Transmit byte.
 * @return Received byte.
 * */
uint8_t BMP280_spiReadWrite(BMP280_t *bmp280, uint8_t tx_message)
{
    uint8_t rx_message = 255;
    HAL_SPI_TransmitReceive(bmp280->spiHandle, &tx_message, &rx_message, 1, HAL_MAX_DELAY);
    return rx_message;
}

/** Pull chip select high (inactive) */
void BMP280_spiCSNhigh(BMP280_t *bmp280)
{
    HAL_GPIO_WritePin(SPI2_IMU_CSBM_GPIO_Port, SPI2_IMU_CSBM_Pin, GPIO_PIN_SET);
}

/** Pull chip select low (active) */
void BMP280_spiCSNlow(BMP280_t *bmp280)
{
    HAL_GPIO_WritePin(SPI2_IMU_CSBM_GPIO_Port, SPI2_IMU_CSBM_Pin, GPIO_PIN_RESET);
}

/**
 * Calculate sensor temperature from measurement and compensation parameters.
 * @param uncomp_temp: Raw temperature measurement.
 * @return Temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * */
int32_t BMP280_compensate_temperature(BMP280_t *bmp280, int32_t uncomp_temp)
{
    int32_t var1, var2;
    var1 = ((((uncomp_temp / 8) - ((int32_t)bmp280->compensationParameters.dig_t1 << 1))) * ((int32_t)bmp280->compensationParameters.dig_t2)) / 2048;
    var2 = (((((uncomp_temp / 16) - ((int32_t)bmp280->compensationParameters.dig_t1)) * ((uncomp_temp / 16) - ((int32_t)bmp280->compensationParameters.dig_t1))) / 4096) * ((int32_t)bmp280->compensationParameters.dig_t3)) / 16384;
    bmp280->t_fine = var1 + var2;
    return (bmp280->t_fine * 5 + 128) / 256;
}

/**
 * Calculate pressure from measurement and compensation parameters.
 * @param uncomp_pres: Raw pressure measurement.
 * @return Pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * */
uint32_t BMP280_compensate_pressure(BMP280_t *bmp280, int32_t uncomp_pres)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)(bmp280->t_fine)) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280->compensationParameters.dig_p6;
    var2 = var2 + ((var1 * (int64_t)bmp280->compensationParameters.dig_p5) * 131072);
    var2 = var2 + (((int64_t)bmp280->compensationParameters.dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)bmp280->compensationParameters.dig_p3) / 256) + ((var1 * (int64_t)bmp280->compensationParameters.dig_p2) * 4096);
    var1 = ((INT64_C(0x800000000000) + var1) * ((int64_t)bmp280->compensationParameters.dig_p1)) / 8589934592;
    if (var1 == 0)
    {
        return 0;
    }
    p = 1048576 - uncomp_pres;
    p = (((((p * 2147483648U)) - var2) * 3125) / var1);
    var1 = (((int64_t)bmp280->compensationParameters.dig_p9) * (p / 8192) * (p / 8192)) / 33554432;
    var2 = (((int64_t)bmp280->compensationParameters.dig_p8) * p) / 524288;
    p = ((p + var1 + var2) / 256) + (((int64_t)bmp280->compensationParameters.dig_p7) * 16);
    return (uint32_t)p;
}
