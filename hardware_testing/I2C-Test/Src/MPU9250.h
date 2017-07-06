/*
 * MPU9250.h
 *
 *  Created on: May 29, 2017
 *      Author: Heethesh
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#define MPU9250_ADDRESS				0x68 << 1
#define MAG_ADDRESS					0x0C << 1

#define WHO_AM_I					0x75

#define ACCEL_XOUT_H 				0x3B
#define ACCEL_XOUT_L 				0x3C
#define ACCEL_YOUT_H 				0x3D
#define ACCEL_YOUT_L 				0x3E
#define ACCEL_ZOUT_H 				0x3F
#define ACCEL_ZOUT_L 				0x40

#define GYRO_XOUT_H					0x43
#define GYRO_XOUT_L					0x44
#define GYRO_YOUT_H					0x45
#define GYRO_YOUT_L					0x46
#define GYRO_ZOUT_H					0x47
#define GYRO_ZOUT_L					0x48

#define GYRO_FULL_SCALE_250_DPS		0x00
#define GYRO_FULL_SCALE_500_DPS		0x08
#define GYRO_FULL_SCALE_1000_DPS	0x10
#define GYRO_FULL_SCALE_2000_DPS	0x18

#define ACC_FULL_SCALE_2_G			0x00
#define ACC_FULL_SCALE_4_G			0x08
#define ACC_FULL_SCALE_8_G			0x10
#define ACC_FULL_SCALE_16_G			0x18

unsigned char* debug_TX(void);
void test_MPU();

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
void _Error_Handler(char *, int);

#endif /* MPU9250_H_ */
