#include "GY-91/BMP280.h"
#include "GY-91/MPU9250.h"
#include "GY-271/HMC5883L.h"
#include "HCSR04/HCSR04.h"
#include "GPS/GPS.h"
#include "math.h"
#include "Globals.h"
#include "Fusion/Fusion.h"

#include "stdbool.h"

bool Diag = false;
bool DebugIsOn = false;
bool Tune = true;
bool Tune_single_true_double_false = false;

int Uart2CallbackCounter = 0;

bool Rotors = true;

bool IsImuAvailable = false;
bool IsMagnAvailable = false;
bool IsDistAvailable = false;
bool IsGpsAvailable = false;

// Controllers
DoublePIDController DPID_Roll;
DoublePIDController DPID_Pitch;
PIDController PID_Yaw;


// Transmitter channel variables
uint16_t Throttle_in = 0;
uint16_t Throttle_controlled = 0;
int16_t Roll_in = 0;
int16_t Roll_controlled = 0;
int16_t Pitch_in = 0;
int16_t Pitch_controlled = 0;
int16_t Yaw_in = 0;
int16_t Yaw_controlled = 0;
uint16_t SWA = 0;
uint16_t SWB = 0;
uint16_t SWC = 0;
uint16_t SWD = 0;
uint16_t VRA = 0;
uint16_t VRB = 0;

MPU9250_t MPU9250;
float AccData[3] = { 0 };
float TempData = 0;
float GyroData[3] = { 0 };
float Roll_measured = 0;
float Pitch_measured = 0;
float Yaw_measured = 0;
FusionEuler Fusion_output;
int16_t MagData[3] = { 0 };

BMP280_t BMP280;
float BMP_Temp = 0;
float BMP_Pres = 0;
float BMP_Alt = 0;

float mG_per_LSB;
Vector v;
int xOffset, yOffset;
float declination = 5.58;
float MAG_X_RAW = 0;
float MAG_Y_RAW = 0;
float MAG_Z_RAW = 0;
float MAG_X_NORM = 0;
float MAG_Y_NORM = 0;
float MAG_Z_NORM = 0;
float magnitude = 0;
float MAG_dir = 0;

HCSR04_t HCSR04 = {0};
float Distance = 0.0;

volatile uint8_t Uart2Buffer[64] = { 0 };
volatile uint8_t RemoteBufferIndex = 0;	// Current position in the ibus packet
volatile uint8_t RemoteBuffer[REM_BUF_SIZE] = { 0 };	// Circular buffer which is constantly written by UART2 Callback
volatile uint8_t LastIbusPacket[IBUS_PACKET_SIZE] = { 0 };
volatile bool RemoteBufferInProgress = true;


GPS_t GPS;
volatile uint8_t Uart4Buffer = 0;
volatile uint8_t GPSPackageIndex = 0;
volatile uint8_t GPSPackageBuffer[GPS_BUFFSIZE] = { 0 };
volatile bool ProcessGPSPackageBuffer = false;

volatile uint8_t Spi1Buffer[64] = { 0 };
volatile uint8_t Spi1ReceivedData[64] = { 0 };

void FloatToUint8s(float* src, uint8_t* array, int position)
{
	memcpy(array+position, src, sizeof(float));
}

void FloatFromUint8s(uint8_t* array, int position, float* dest)
{
  memcpy(dest, array+position, sizeof(float));
}

void Uint16ToUint8s(uint16_t* src, uint8_t* array, int position)
{
	memcpy(array+position, src, sizeof(uint16_t));
}

void Int16ToUint8s(int16_t* src, uint8_t* array, int position)
{
	memcpy(array+position, src, sizeof(int16_t));
}

