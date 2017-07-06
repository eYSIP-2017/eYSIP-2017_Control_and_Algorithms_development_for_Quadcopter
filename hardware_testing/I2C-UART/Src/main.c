#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "serial.h"
#include "MPU9250.h"
#include "telemetry.h"
#include "main.h"

void setup(void)
{
	Peripherals_Init();
	//IMU_Init();
}

int main(void)
{
	setup();
	while (1)
	{
		//IMU_Update();
		printString("Hello!\n");

		//MPU9250_ReadAccelData();
		/*MPU9250_ReadGyroData();
		AK8963_ReadData();*/
		HAL_Delay(100);
	}
}





