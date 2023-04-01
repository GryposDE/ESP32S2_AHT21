
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "../AHT21/AHT21.h"

void app_main(void)
{
	// Sensor configuration.
    sAHT21_t mySensor;
    mySensor.I2C_DeviceAddr = 0x38;
    mySensor.I2C_SCL_Pin = 5;
    mySensor.I2C_SDA_Pin = 6;

    // Sensor initialization.
    vAHT21_Init(&mySensor);

    while(1)
    {
        printf("\n");

        printf("AHT21::  current temp: %.2f C\n", f32AHT21_GetTemperature(&mySensor));
        printf("AHT21::  current humidity: %.2f%%\n", f32AHT21_GetHumidity(&mySensor));

        vTaskDelay(300);
    }

    vTaskDelay(10000);
}