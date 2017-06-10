/*
 * MPU9250.c
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

/* TODO: Implement MSP for all IMU data and Attitude */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "serial.h"
#include "telemetry.h"
#include "MadgwickAHRS.h"
#include "MPU9250.h"
#include <math.h>

//#define IMU_DEBUG
#define sq(x) ((x)*(x))
#define _USE_MATH_DEFINES

extern I2C_HandleTypeDef hi2c1;
extern struct txFrame txf;

uint32_t lastUpdate = 0, timeNow = 0;

float AHRS_Angle[3] = {0, 0, 0};
float mRes = 10.0*4912.0/32760.0;

struct IMUData
{
	volatile float x;
	volatile float y;
	volatile float z;
}accelData, gyroData, magData, magCalib, magBias,
gyroBias = {-1.82, -0.08, -1.17}, magScale = {0.97, 0.95, 1.09};

/**********************************
 Function name	:	IMU_WriteByte
 Functionality	:	To write a byte to a register on the I2C device
 Arguments		:	I2C Device Address, Register Address, Register Value
 Return Value	:	None
 Example Call	:	IMU_WriteByte()
 ***********************************/
void IMU_WriteByte(uint16_t device_add, uint16_t register_add, uint8_t register_val)
{
	uint8_t byte[] = {register_val}, ret;
	ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(__FILE__, __LINE__);

	// Delay for device setup
	HAL_Delay(50);
}

/**********************************
 Function name	:	IMU_ReadByte
 Functionality	:	To read a byte from a register on the I2C device
 Arguments		:	I2C Device Address, Register Address
 Return Value	:	None
 Example Call	:	IMU_ReadByte()
 ***********************************/
uint8_t IMU_ReadByte(uint16_t device_add, uint16_t register_add)
{
	uint8_t byte[] = {0x00}, ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) &byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(__FILE__, __LINE__);
	return byte[0];
}

/**********************************
 Function name	:	IMU_ReadByteArray
 Functionality	:	To read multiples byte from series of register on the I2C device
 Arguments		:	I2C Device Address, Start Register Address, Buffer, Size
 Return Value	:	None
 Example Call	:	IMU_ReadByteArray()
 ***********************************/
void IMU_ReadByteArray(uint16_t device_add, uint16_t register_add, uint8_t* byte_array, uint16_t size)
{
	int i = 0;
	while (i<size)
	{
		byte_array[i++] = IMU_ReadByte(device_add, register_add++);
	}
}

/**********************************
 Function name	:	MPU9250_Init
 Functionality	:	To setup the Accelerometer and Gyroscope
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_Init()
 ***********************************/
void MPU9250_Init()
{
	// Verify device
	uint8_t data = IMU_ReadByte(MPU9250_ADDRESS, WHO_AM_I);
	if (data != WHO_AM_I_VALUE) _Error_Handler(__FILE__, __LINE__);

	// Device configuration
	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x80);	// Reset
	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x01);	// Set clock source to be PLL with x-axis gyroscope reference
	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_2, 0x00);	// Enable Accel and Gyro
	IMU_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 	   0x00);	// Sample Rate Divider (Not set)
	IMU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2,  0x03);	// DLPF 184Hz
	IMU_WriteByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);	// DLPF 184Hz

	// Full scale settings
	IMU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_1000_DPS);
	IMU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FS_4_G);
}

/**********************************
 Function name	:	MPU9250_ReadAccelData
 Functionality	:	To read the raw data from the accelerometer and convert it Gs
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_ReadAccelData()
 ***********************************/
void MPU9250_ReadAccelData()
{
	// Data buffer
	volatile int8_t raw_data[] = {0, 0, 0, 0, 0, 0};

	// Read raw data
	IMU_ReadByteArray(MPU9250_ADDRESS, ACCEL_XOUT_H, &raw_data, 6);

	// Convert and store it
	accelData.x = (float) ((int16_t)(((int16_t)raw_data[0]<<8) + raw_data[1])) * 4.0f/32768.0f;
	accelData.y = (float) ((int16_t)(((int16_t)raw_data[2]<<8) + raw_data[3])) * 4.0f/32768.0f;
	accelData.z = (float) ((int16_t)(((int16_t)raw_data[4]<<8) + raw_data[5])) * 4.0f/32768.0f;

#ifdef IMU_DEBUG
	serialFloat(accelData.x);
	serialWrite('\t');
	serialFloat(accelData.y);
	serialWrite('\t');
	serialFloat(accelData.z);
	serialWrite('\n');
#endif
}

/**********************************
 Function name	:	MPU9250_ReadGyroData
 Functionality	:	To read the raw data from the gyroscope and convert it DPS
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_ReadGyroData()
 ***********************************/
void MPU9250_ReadGyroData()
{
	// Data buffer
	volatile int8_t raw_data[] = {0, 0, 0, 0, 0, 0};

	// Read raw data
	IMU_ReadByteArray(MPU9250_ADDRESS, GYRO_XOUT_H, &raw_data, 6);

	// Convert and store it
	gyroData.x = (float) (((int16_t)(((int16_t)raw_data[0]<<8) + raw_data[1])) * 1000.0f/32768.0f - gyroBias.x);
	gyroData.y = (float) (((int16_t)(((int16_t)raw_data[2]<<8) + raw_data[3])) * 1000.0f/32768.0f - gyroBias.y);
	gyroData.z = (float) (((int16_t)(((int16_t)raw_data[4]<<8) + raw_data[5])) * 1000.0f/32768.0f - gyroBias.z);

#ifdef IMU_DEBUG
	serialFloat(gyroData.x);
	serialWrite('\t');
	serialFloat(gyroData.y);
	serialWrite('\t');
	serialFloat(gyroData.z);
	serialWrite('\n');
#endif
}

/**********************************
 Function name	:	AK8963_Init
 Functionality	:	To setup the magnetometer
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AK8963_Init()
 ***********************************/
void AK8963_Init()
{
	// Enable access to Magnetometer via MPU
	IMU_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);	// Set bypass mode for external I2C master connection
	IMU_WriteByte(MPU9250_ADDRESS, USER_CTRL,   0x01); 	// Disable master mode and clear all signal paths

	// Verify magnetometer
	uint8_t data = IMU_ReadByte(MAG_ADDRESS, MAG_WIA);
	if (data != MAG_WIA_VALUE) _Error_Handler(__FILE__, __LINE__);

	// Calibrate magnetometer bias
	magBias.x = -1;
	magBias.y = 330;
	magBias.z = -343;

	// Calibrate magnetometer factory offset
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL2, 0x01);	// Reset magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00);	// Power down magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x0F); 	// Enter Fuse ROM access mode

	// Read calibration registers
	uint8_t rawData[3];
	IMU_ReadByteArray(MAG_ADDRESS, MAG_ASAX, &rawData, 3);

	magCalib.x =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values
	magCalib.y =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	magCalib.z =  (float)(rawData[2] - 128)/256.0f + 1.0f;

#ifdef IMU_DEBUG
	serialFloat(magCalib.x);
	serialWrite('\t');
	serialFloat(magCalib.y);
	serialWrite('\t');
	serialFloat(magCalib.z);
	serialWrite('\n');
#endif

	// Magnetometer settings
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00);	// Power down magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x16); 	// Res: 16 Bit, Mode: Continuous Mode 2 (100Hz)
}

/**********************************
 Function name	:	AK8963_ReadData
 Functionality	:	To read the raw data from the magnetometer and convert it milliGauss
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AK8963_ReadData()
 ***********************************/
void AK8963_ReadData()
{
	volatile uint8_t raw_data[] = {0, 0, 0, 0, 0, 0, 0};

	// Check if data is ready
	if (IMU_ReadByte(MAG_ADDRESS, MAG_ST1) & 0x01)
	{
		// Read data registers and ST2 register to check overflow
		IMU_ReadByteArray(MAG_ADDRESS, MAG_HXL, &raw_data, 7);
		uint8_t OVF = raw_data[6];

		// Store data if no overflow occurred
		if (!(OVF & 0x08))
		{
			magData.x = (float) (((int16_t)((((int16_t)raw_data[1]<<8) + raw_data[0])) * mRes * magCalib.x) - magBias.x) * magScale.x;
			magData.y = (float) (((int16_t)((((int16_t)raw_data[3]<<8) + raw_data[2])) * mRes * magCalib.y) - magBias.y) * magScale.y;
			magData.z = (float) (((int16_t)((((int16_t)raw_data[5]<<8) + raw_data[4])) * mRes * magCalib.z) - magBias.z) * magScale.z;

#ifdef IMU_DEBUG
			serialFloat(magData.x);
			serialWrite('\t');
			serialFloat(magData.y);
			serialWrite('\t');
			serialFloat(magData.z);
			serialWrite('\n');
#endif
		}

#ifdef IMU_DEBUG
		else
		{
			printString("\n* Magnetometer Overflow *\n;");
		}
#endif
	}
}

/**********************************
 Function name	:	IMU_Init
 Functionality	:	Initialize the IMU and AHRS filter  parameters
 Arguments		:	None
 Return Value	:	None
 Example Call	:	IMU_Init()
 ***********************************/
void IMU_Init()
{
	MPU9250_Init();
	AK8963_Init();

	float GyroMeasError = M_PI * (60.0f / 180.0f);	// Gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;	// Compute beta

#ifdef IMU_DEBUG
	serialPrint("Beta: ");
	serialFloat(beta);
	serialLine();
#endif

	MadgwickSetBeta(beta);
	MadgwickSetDelta(0.0f);
}

/**********************************
 Function name	:	AHRS_ComputeAngles
 Functionality	:	Update the 9DOF data from IMU and compute pitch, roll and yaw
  	  	  	  	  	using Madgwick's AHRS filter
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AHRS_ComputeAngles()
 ***********************************/
void AHRS_ComputeAngles()
{
	// Read IMU data
	MPU9250_ReadAccelData();
	MPU9250_ReadGyroData();
	AK8963_ReadData();

	// Filter data and obtain the anagles
	MadgwickQuaternionUpdate(-accelData.y, -accelData.x, accelData.z, gyroData.y,
			gyroData.x, -gyroData.z, magData.x,	magData.y, magData.z, &AHRS_Angle);

	// Set integration time by time elapsed since last filter update
	timeNow = HAL_GetTick();
	float delta = (float)((timeNow - lastUpdate)/1000.0f) ;
	MadgwickSetDelta(delta);
	lastUpdate = timeNow;

	// Update and send data frame
	txf.x = AHRS_Angle[0];
	txf.y = AHRS_Angle[1];
	txf.z = AHRS_Angle[2];
	sendFrame(&txf);
}

/**********************************
 Function name	:	AHRS_GetPitch
 Functionality	:	Returns the pitch angle
 Arguments		:	None
 Return Value	:	Pitch angle
 Example Call	:	AHRS_GetPitch()
 ***********************************/
float AHRS_GetPitch()
{
	return AHRS_Angle[0];
}

/**********************************
 Function name	:	AHRS_GetRoll
 Functionality	:	Returns the roll angle
 Arguments		:	None
 Return Value	:	Roll angle
 Example Call	:	AHRS_GetRoll()
 ***********************************/
float AHRS_GetRoll()
{
	return AHRS_Angle[1];
}

/**********************************
 Function name	:	AHRS_GetYaw
 Functionality	:	Returns the yaw angle
 Arguments		:	None
 Return Value	:	Yaw angle
 Example Call	:	AHRS_GetYaw()
 ***********************************/
float AHRS_GetYaw()
{
	return AHRS_Angle[2];
}
