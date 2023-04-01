/**
 * @file    : AHT21.h
 * @author  : Christian Roth.
 * @brief   : Implements the AHT21 Humidity and Temperature Sensor for the ESP32S2.
 *            It takes notes for device specific timing.
 *            ToDo: Let user define which ESP32S2 I2C_Device_Port should get used (currently default = 0). 
 *            ToDo: ERROR detection and handnling.
 * 
 * @version : 0.1
 * @date    : 01.04.2023
 * 
 * @copyright Copyright (c) 2023
 */

#ifndef AHT21_H
#define AHT21_H

#include <stdint.h>
typedef float  f32_t;
typedef double f64_t;



typedef struct
{
    uint8_t PADDING;

    uint8_t I2C_DeviceAddr; // 7bit i2c device address.

    uint8_t I2C_SDA_Pin;
    uint8_t I2C_SCL_Pin;
    
} sAHT21_t;


/**
 * @brief: Initialized the AHT21 device with the given configuration.
 *         Note: Each initialization will take at least 100ms for device specific reasons.
 * 
 * @param _this[in] : The configuration for the AHT21 device.
 * @param _this[out]: The AHT21 device handler used for other functions.
 */
void vAHT21_Init(sAHT21_t const* const _this);

/**
 * @brief: Measures the current temperature and returns the current temperature in Celsius.
 *         Note: Each measurement will take at least 90ms for device specific reasons.
 * 
 * @param _this: The AHT21 device handler.
 * 
 * @return f32_t: The current temperature in Celsius.
 */
f32_t f32AHT21_GetTemperature(sAHT21_t const* const _this);

/**
 * @brief: Measures the current humidity and returns the current humidity in percent (RH%).
 *         Note: Each measurement will take at least 90ms for device specific reasons.
 * 
 * @param _this: The AHT21 device handler.
 * 
 * @return f32_t: The current humidity in percent (RH%).
 */
f32_t f32AHT21_GetHumidity(sAHT21_t const* const _this);


#endif /* AHT21_H */
