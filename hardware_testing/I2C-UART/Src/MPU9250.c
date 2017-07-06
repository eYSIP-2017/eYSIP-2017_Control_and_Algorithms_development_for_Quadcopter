/*
 * MPU9250.c
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "serial.h"
#include "telemetry.h"
#include "MadgwickAHRS.h"
#include "MPU9250.h"
#include <math.h>

extern struct txFrame txf;

//#define IMU_DEBUG
#define sq(x) ((x)*(x))

#define _USE_MATH_DEFINES
#define COMP_FILTER_ALPHA 0.98

extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

uint32_t gyro_last_time_x = 0;
uint32_t gyro_last_time_y = 0;
uint32_t gyro_last_time_z = 0;
uint32_t lastUpdate = 0, timeNow = 0;

float AHRS_Angle[3] = {0, 0, 0};
float mRes = 10.0*4912.0/32760.0;

struct IMUData
{
	volatile float x;
	volatile float y;
	volatile float z;
}accelData, gyroData, magData, accelAngle, compAngle,
magCalib, magBias, gyroBias = {-1.82, -0.08, -1.17}, magScale = {0.97, 0.95, 1.09};

void IMU_WriteByte(uint16_t device_add, uint16_t register_add, uint8_t register_val)
{
	uint8_t byte[] = {register_val};
	//HAL_StatusTypeDef ret;
	uint8_t ret;
	ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(__FILE__, __LINE__);

	HAL_Delay(50);
}

uint8_t IMU_ReadByte(uint16_t device_add, uint16_t register_add)
{
	uint8_t byte[] = {0x00};
	//HAL_StatusTypeDef ret;
	uint8_t ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) &byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(__FILE__, __LINE__);
	return byte[0];
}

void IMU_ReadByteArray(uint16_t device_add, uint16_t register_add, uint8_t* byte_array, uint16_t size)
{
	int i = 0;
	while (i<size)
	{
		byte_array[i++] = IMU_ReadByte(device_add, register_add++);
		//HAL_Delay(5);
	}
}

void MPU9250_Init(void)
{
	uint8_t data;
	data = IMU_ReadByte(MPU9250_ADDRESS, WHO_AM_I);
	if (data != WHO_AM_I_VALUE) _Error_Handler(__FILE__, __LINE__);
	printChar(data);
	printChar('\n');

	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x80);	// Reset
	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x01);	// Set clock source to be PLL with x-axis gyroscope reference
	IMU_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_2, 0x00);	// Enable Accel and Gyro
	IMU_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 	   0x00);	// Sample Rate Divider (1000/100 = 10Hz)
	IMU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2,  0x03);	// DLPF 10Hz
	IMU_WriteByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);	// DLPF 10Hz

	IMU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_1000_DPS);
	IMU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FS_4_G);
}

void MPU9250_ReadAccelData()
{
	volatile int8_t raw_accel_data[] = {0, 0, 0, 0, 0, 0};

	IMU_ReadByteArray(MPU9250_ADDRESS, ACCEL_XOUT_H, raw_accel_data, 6);

	accelData.x = (float) ((int16_t)(((int16_t)raw_accel_data[0]<<8) + raw_accel_data[1])) * 4.0f/32768.0f;
	accelData.y = (float) ((int16_t)(((int16_t)raw_accel_data[2]<<8) + raw_accel_data[3])) * 4.0f/32768.0f;
	accelData.z = (float) ((int16_t)(((int16_t)raw_accel_data[4]<<8) + raw_accel_data[5])) * 4.0f/32768.0f;

	/*txf.x = accelData.x;
	txf.y = accelData.y;
	txf.z = accelData.z;

	sendFrame(&txf);*/

#ifdef IMU_DEBUG
	printFloat(accelData.x*10);
	printChar('\t');
	printFloat(accelData.y*10);
	printChar('\t');
	printFloat(accelData.z*10);
	printChar('\n');
#endif
}

void MPU9250_ReadGyroData()
{
	volatile int8_t raw_data[] = {0, 0, 0, 0, 0, 0};

	IMU_ReadByteArray(MPU9250_ADDRESS, GYRO_XOUT_H, &raw_data, 6);

	gyroData.x = (float) (((int16_t)(((int16_t)raw_data[0]<<8) + raw_data[1])) * 1000.0f/32768.0f - gyroBias.x);
	gyroData.y = (float) (((int16_t)(((int16_t)raw_data[2]<<8) + raw_data[3])) * 1000.0f/32768.0f - gyroBias.y);
	gyroData.z = (float) (((int16_t)(((int16_t)raw_data[4]<<8) + raw_data[5])) * 1000.0f/32768.0f - gyroBias.z);

#ifdef IMU_DEBUG
	printFloat(gyroData.x);
	printChar('\t');
	printFloat(gyroData.y);
	printChar('\t');
	printFloat(gyroData.z);
	printChar('\n');
#endif
}

float MPU9250_ComputeAngle(float accel_data, float gyro_data, float comp_angle, uint32_t *last_time)
{
	float angle = 0, accel_angle = 0, gyro_angle = 0, delta = 0;

	accel_angle = acos((accel_data)/sqrt(sq(accelData.x)+sq(accelData.y)+sq(accelData.z)))*180.0f/M_PI;

	/*delta = (float)(HAL_GetTick() - *last_time) * 0.001;
	gyro_angle = (gyro_data*delta) + comp_angle;
	angle = (gyro_angle * (COMP_FILTER_ALPHA)) + (accel_angle * (1-COMP_FILTER_ALPHA));
	*last_time = HAL_GetTick();*/
	angle = accel_angle;
	return angle;
}

float MPU9250_GetPitch()
{
	//compAngle.y = MPU9250_ComputeAngle(accelData.y, gyroData.y, compAngle.y, &gyro_last_time_y);
	//return compAngle.y;

	accelAngle.x = atan2(accelData.x, accelData.z)*180.0/M_PI;
	//accelAngle.x = atan2(accelData.x, sqrt(sq(accelData.y) + sq(accelData.z)))*180.0/M_PI;
	return accelAngle.x;
}

float MPU9250_GetRoll()
{
	//compAngle.x = MPU9250_ComputeAngle(accelData.x, gyroData.x, compAngle.x, &gyro_last_time_x);
	//return compAngle.x;

	accelAngle.y = atan2(accelData.y, accelData.z)*180.0/M_PI;
	return accelAngle.y;
}

float MPU9250_GetYaw()
{
	//compAngle.z = MPU9250_ComputeAngle(accelData.z, gyroData.z, compAngle.z, &gyro_last_time_z);
	//return compAngle.z;
	return 0;
}

void AK8963_Init()
{
	// Enable access to Magnetometer via MPU
	IMU_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);	// Set bypass mode for external I2C master connection
	IMU_WriteByte(MPU9250_ADDRESS, USER_CTRL,   0x01); 	// Disable master mode and clear all signal paths

	// Verify Magnetometer
	uint8_t data;
	data = IMU_ReadByte(MAG_ADDRESS, MAG_WIA);
	if (data != MAG_WIA_VALUE) _Error_Handler(__FILE__, __LINE__);
	printChar(data);
	printChar('\n');

	// Calibrate Magnetometer
	magBias.x = -1;
	magBias.y = 330;
	magBias.z = -343;

	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL2, 0x01);	// Reset magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00);	// Power down magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x0F); 	// Enter Fuse ROM access mode

	uint8_t rawData[3];
	IMU_ReadByteArray(MAG_ADDRESS, MAG_ASAX, &rawData, 3);

	magCalib.x =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values
	magCalib.y =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	magCalib.z =  (float)(rawData[2] - 128)/256.0f + 1.0f;

#ifdef IMU_DEBUG
	printString("Mag Calib Values: \n");
	printInt(magCalib.x);
	printChar('\t');
	printInt(magCalib.y);
	printChar('\t');
	printInt(magCalib.z);
	printChar('\n');
#endif

	// Magnetometer Settings
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00);	// Power down magnetometer
	IMU_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x16); 	// Res: 16 Bit, Mode: Continuous Mode 2 (100Hz)
}

void AK8963_ReadData()
{
	volatile uint8_t raw_data[] = {0, 0, 0, 0, 0, 0, 0};

	if (IMU_ReadByte(MAG_ADDRESS, MAG_ST1) & 0x01)
	{
		IMU_ReadByteArray(MAG_ADDRESS, MAG_HXL, &raw_data, 7);
		uint8_t OVF = raw_data[6];
		if (!(OVF & 0x08))
		{
			magData.x = (float) (((int16_t)((((int16_t)raw_data[1]<<8) + raw_data[0])) * mRes * magCalib.x) - magBias.x) * magScale.x;
			magData.y = (float) (((int16_t)((((int16_t)raw_data[3]<<8) + raw_data[2])) * mRes * magCalib.y) - magBias.y) * magScale.y;
			magData.z = (float) (((int16_t)((((int16_t)raw_data[5]<<8) + raw_data[4])) * mRes * magCalib.z) - magBias.z) * magScale.z;

#ifdef IMU_DEBUG
			printInt(magData.x);
			printChar('\t');
			printInt(magData.y);
			printChar('\t');
			printInt(magData.z);
			printChar('\n');
#endif
		}

		else
		{
			printString("\n* Mag Overflow *\n;");
		}
	}
}

void IMU_Init()
{
	MPU9250_Init();
	AK8963_Init();

	float GyroMeasError = M_PI * (60.0f / 180.0f);	// Gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;	// Compute beta

#ifdef IMU_DEBUG
	printString("Beta: ");
	printFloat(beta);
	printChar('\n');
#endif

	MadgwickSetBeta(beta);
	MadgwickSetDelta(0.0f);
}

void IMU_Update()
{
	MPU9250_ReadAccelData();
	MPU9250_ReadGyroData();
	AK8963_ReadData();

	MadgwickQuaternionUpdate(-accelData.y,	-accelData.x, 	accelData.z,
							 gyroData.y, 	gyroData.x, 	-gyroData.z,
							 magData.x,		magData.y, 		magData.z, 	&AHRS_Angle);

	// Set integration time by time elapsed since last filter update
	timeNow = HAL_GetTick();
	float delta = (float)((timeNow - lastUpdate)/1000.0f) ;
	MadgwickSetDelta(delta);
	lastUpdate = timeNow;

	txf.x = AHRS_Angle[0];
	txf.y = AHRS_Angle[1];
	txf.z = AHRS_Angle[2];

	sendFrame(&txf);

	/*printInt(AHRS_Angle[0]);
	printChar('\t');
	printInt(AHRS_Angle[1]);
	printChar('\t');
	printInt(AHRS_Angle[2]);
	printChar('\n');*/
}
