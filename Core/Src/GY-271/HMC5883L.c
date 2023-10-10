#include "main.h"
#include "cmsis_os.h"
#include "GY-271/HMC5883L.h"
#include "Globals.h"


uint8_t HMC5883L_Init()
{
	HMC5883L_setRange(HMC5883L_RANGE_8_1GA);
	HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
	HMC5883L_setDataRate(HMC5883L_DATARATE_30HZ);
	HMC5883L_setSamples(HMC5883L_SAMPLES_4);
	HMC5883L_setOffset(0, 0);

	return 0;
}

Vector HMC5883L_readRaw(void)
{
    v.XAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_X_M) - xOffset;
    v.YAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_Z_M);

    return v;
}

Vector HMC5883L_read_mGauss(void)
{
    v.XAxis = ((float)HMC5883L_readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mG_per_LSB;
    v.YAxis = ((float)HMC5883L_readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mG_per_LSB;
    v.ZAxis = (float)HMC5883L_readRegister16(HMC5883L_REG_OUT_Z_M) * mG_per_LSB;

    return v;
}

void HMC5883L_setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void HMC5883L_setRange(uint8_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mG_per_LSB = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mG_per_LSB = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mG_per_LSB = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mG_per_LSB = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mG_per_LSB = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mG_per_LSB = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mG_per_LSB = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mG_per_LSB = 4.35f;
	    break;

	default:
	    break;
    }

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

uint8_t HMC5883L_getRange(void)
{
    return (uint8_t)((HMC5883L_readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L_setMeasurementMode(uint8_t mode)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    HMC5883L_writeRegister8(HMC5883L_REG_MODE, value);
}

uint8_t HMC5883L_getMeasurementMode(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return value;
}

void HMC5883L_setDataRate(uint8_t dataRate)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

uint8_t HMC5883L_getDataRate(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return value;
}

void HMC5883L_setSamples(uint8_t samples)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

uint8_t HMC5883L_getSamples(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return value;
}

// Write byte to register
void HMC5883L_writeRegister8(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&I2C, HMC5883L_DEFAULT_ADDRESS, reg, 1 , &value, 1, 500);
}

// Read byte to register
uint8_t HMC5883L_fastRegister8(uint8_t reg)
{
    uint8_t value;
//    HAL_I2C_Mem_Write(&I2C, HMC5883L_ADDRESS, reg, 1 ,value,1,500)
    HAL_I2C_Mem_Read(&I2C, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 1, 500);
    return value;
}

// Read byte from register
uint8_t HMC5883L_readRegister8(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&I2C, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 1, 500);
    return value;
}

// Read word from register
int16_t HMC5883L_readRegister16(uint8_t reg)
{
    int16_t value;
    HAL_I2C_Mem_Read(&I2C, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 2, 500);
    return value;
}
