/*
 * MS5611.c
 *
 *  Created on: Jun 22, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "timing.h"
#include "i2c.h"
#include "MS5611.h"
#include "serial.h"
#include "msp.h"
#include "math.h"
#include "common.h"

//#define BARO_DEBUG

extern msp_altitude msp_txf_altitude;
struct LPF lpf_altitude = {0, 0.7};

// Public variables
float MS5611_CompensatedTemperature;  					// Compensated temperature measurement
float MS5611_CompensatedPressure;  						// Compensated pressure measurement
float MS5611_Altitude;									// Computed altitude

// Private variables
static uint32_t MS5611_RawTemperature;  				// Raw temperature measurement
static uint32_t MS5611_RawPressure;  					// Raw pressure measurement
static uint16_t MS5611_Coefficients[MS5611_PROM_NB];	// On-chip PROM calibration coefficients
static uint8_t  MS5611_OSR = MS5611_OSR_4096;			// Set over-sampling rate
static uint8_t	TEMP_READY = 0;							// Temperature data ready flag
static uint8_t	PRESSURE_READY = 0; 					// Pressure data ready flag
static uint32_t temp_time = 0;							// Temperature measurement time
static uint32_t pressure_time = 0;						// Pressure measurement time

// Private functions
static int8_t MS5611_CRC(uint16_t *prom)
{
	int32_t i, j;
	uint32_t res = 0;
	uint8_t zero = 1;
	uint8_t crc = prom[7] & 0xF;
	prom[7] &= 0xFF00;

	for (i = 0; i < 8; i++)
	{
		if (prom[i] != 0) zero = 0;
	}
	if (zero) return -1;

	for (i = 0; i < 16; i++)
	{
		if (i & 1) res ^= ((prom[i >> 1]) & 0x00FF);
		else res ^= (prom[i >> 1] >> 8);
		for (j = 8; j > 0; j--)
		{
			if (res & 0x8000) res ^= 0x1800;
			res <<= 1;
		}
	}
	prom[7] |= crc;
	if (crc == ((res >> 12) & 0xF)) return 0;

	return -1;
}

static void MS5611_Reset()
{
	I2C_WriteByte(MS5611_ADDRESS, MS5611_CMD_RESET, 1, 1);
	delay_ms(10); // Minimum delay of 2.8 ms required
}

static void MS5611_SetOSR(uint8_t osr)
{
	MS5611_OSR = osr;
}

static uint16_t MS5611_ReadPROM(int8_t coef_num)
{
	uint8_t rxbuf[2] = {0, 0};
	I2C_ReadByteArray(MS5611_ADDRESS, MS5611_CMD_READ_PROM + (coef_num * 2), rxbuf, 2); // Send PROM READ command
	return ((rxbuf[0] << 8) | rxbuf[1]);
}

static uint32_t MS5611_ReadADC(void)
{
	uint8_t rxbuf[3];
	I2C_ReadBytes(MS5611_ADDRESS, MS5611_CMD_ADC_READ, rxbuf, 3); // Read ADC 3 bytes

	// Combine the 3 bytes to form the 24 bit ADC value
	return ((uint32_t)(rxbuf[0] << 16) | ((uint32_t)rxbuf[1] << 8) | (uint32_t)rxbuf[2]);
}

static void MS5611_StartPressureConversion(void)
{
	I2C_WriteByte(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + MS5611_OSR, 1, 0); // Start D1 (pressure) conversion
}

static void MS5611_StartTemperatureConversion(void)
{
	I2C_WriteByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + MS5611_OSR, 1, 0); // Start D2 (temperature) conversion
}

static void MS5611_ReadTemperature(void)
{
	MS5611_RawTemperature = MS5611_ReadADC();
}

static void MS5611_ReadPressure(void)
{
	MS5611_RawPressure = MS5611_ReadADC();
}

/* * * * * * * * * *
 * C1 --> SENST1	--> Pressure sensitivity
 * C2 --> OFFT1		--> Pressure offset
 * C3 --> TCS		--> Temperature coefficient of pressure sensitivity
 * C4 --> TCO		--> Temperature coefficient of pressure offset
 * C5 --> TREF		--> Reference temperature
 * C6 --> TEMPSENS	--> Temperature coefficient of the temperature
 * * * * * * * * * */

// Refer MS5611 datasheet for further details
static void MS5611_Compute(void)
{
	int32_t TEMP, P, TT, OFF2 = 0, SENS2 = 0, T2 = 0;

	/** Calculate Temperature */
	// Difference between actual and reference temperature
	// dT = D2 - TREF --> D2 - C5 * 2^8
	int32_t dT = MS5611_RawTemperature - ((uint32_t) MS5611_Coefficients[5] << 8);

	// Actual temperature (-40 ... 85°C with 0.01°C resolution)
	// TEMP = 20°C + dT * TEMPSENS --> 2000 + dT * C6 / 2^23
	TEMP = 2000 + (((int64_t) dT * MS5611_Coefficients[6]) >> 23);

	/** Calculate Temperature Compensated Pressure */
	// Offset at actual temperature
	// OFF = OFFT1 + TCO * dT --> C2 * 2^16 + (C4 * dT) / 2^7
	int64_t OFF = ((uint32_t) MS5611_Coefficients[2] << 16) + (((int64_t) dT * MS5611_Coefficients[4]) >> 7);

	// Sensitivity at actual temperature
	// SENS = SENST1 + TCS * dT --> C1 * 2^15 + (C3 * dT) / 2^8
	int64_t SENS = ((uint32_t) MS5611_Coefficients[1] << 15) + (((int64_t) dT * MS5611_Coefficients[3]) >> 8);

	/** Second Order Temperature Compensation */
	// Temperature lower than 20°C
	if (TEMP < 2000)
	{
		T2 = (dT * dT) >> 31;
		TT = (TEMP - 2000);
		TT = TT * TT;
		OFF2 = (5 * TT) >> 1;
		SENS2 = (5 * TT) >> 2;

		// Temperature lower than -15°C
		if (TEMP < -1500)
		{
			TT = TEMP + 1500;
			TT = T2 * T2;
			OFF2 += 7 * TT;
			SENS2 += (11 * TT) >> 1;
		}
	}

	// Temperature greater than 20°C
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	// Calculate pressure and temperature
	TEMP -= T2;
	OFF  -= OFF2;
	SENS -= SENS2;

	// Temperature compensated pressure (10 ... 1200mbar with 0.01mbar resolution)
	// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
	P = (((MS5611_RawPressure * SENS) >> 21) - OFF) >> 15;

	// Update global variables and scale values
	MS5611_CompensatedPressure = (float) P / 100.0f;
	MS5611_CompensatedTemperature = (float) TEMP / 100.0f;
}

// Calculate altitude from Pressure & Sea level pressure
static void MS5611_ComputeAltitude()
{
	MS5611_Altitude = lowPassFilter(&lpf_altitude, 44330.0f * (1.0f - pow(MS5611_CompensatedPressure / SEA_LEVEL_PRESSURE, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
static float MS5611_ComputeSeaLevel(int32_t pressure, int32_t altitude)
{
    return ((float) pressure / pow(1.0f - ((float) altitude / 44330.0f), 5.255f));
}

/****************************** PUBLIC FUNCTIONS ******************************/

float MS5611_GetTemperature(void)
{
	return MS5611_CompensatedTemperature;
}

float MS5611_GetPressure(void)
{
	return MS5611_CompensatedPressure;
}

float MS5611_GetAltitude(void)
{
	return MS5611_Altitude;
}

void MS5611_Update(void)
{
	if (TEMP_READY && PRESSURE_READY)
	{
		// Compute pressure and temperature
		MS5611_Compute();

		// Compute altitude
		MS5611_ComputeAltitude();

		// Update MSP frame
		msp_txf_altitude.est_alt = (int32_t) MS5611_Altitude;

#ifdef BARO_DEBUG
		serialPrint("Temp: ");
		serialInt(MS5611_CompensatedTemperature);
		serialPrint(" C\tPressure: ");
		serialFloat(MS5611_CompensatedPressure);
		serialPrint(" hPa\tAltitude: ");
		serialFloat(MS5611_ComputeAltitude(MS5611_CompensatedPressure));
		serialPrint(" m\n");
#endif

		// Reset conversion flags for next reading
		TEMP_READY = 0;
		PRESSURE_READY = 0;

		// Start new conversion
		MS5611_StartTemperatureConversion();
		temp_time = millis();
	}

	// 10ms delay for ADC conversion
	if (((millis() - temp_time) > 10) && !(TEMP_READY) && !(PRESSURE_READY))
	{
		MS5611_ReadTemperature();
		TEMP_READY = 1;

		MS5611_StartPressureConversion();
		pressure_time = millis();
	}

	if (((millis() - pressure_time) > 10) && TEMP_READY && !(PRESSURE_READY))
	{
		MS5611_ReadPressure();
		PRESSURE_READY = 1;
	}
}

void MS5611_Init(void)
{
	MS5611_Reset();
	MS5611_SetOSR(MS5611_OSR_4096);

	for (int i=0; i<MS5611_PROM_NB; i++)
		MS5611_Coefficients[i] = MS5611_ReadPROM(i);
}
