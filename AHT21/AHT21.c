/**
 * @file    : AHT21.c
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

#include "AHT21.h"
#include "driver/i2c.h"


#define AHT21_TRIGGER_MEASUREMENT   0xAC
#define AHT21_MEASUREMENT_DATA0     0x33
#define AHT21_MEASUREMENT_DATA1     0x00

#define AHT21_STATUS_CHECK_MASK     0x18
#define AHT21_STATUS_OK             0x18


typedef struct
{
    int32_t humidity;
    int32_t temperature;
    
} sAHT21_Data_t;

/**
 *  Used for I2C data transmission.
 */
static int i2c_master_port;


/**
 * @brief: Checks the status of the AHT21 device.
 *         ToDo: If status check fails, recalibrates the device (need to lookup at official website).
 *         Note: After restart of the device, wait at least 100ms.
 * 
 * @param _this: The represented device that should get tested.
 */
static void vAHT21_CheckStatus(sAHT21_t const* const _this);

/**
 * @brief: Requests the last measurement of temperature and humidity and updates "measurement".
 * 
 * @param I2C_DeviceAddr  : The I2C device address.
 * @param measurement[out]: Will be loaded with the requested measurement data.
 */
static void vAHT21_ReadMeasurement(uint8_t I2C_DeviceAddr, sAHT21_Data_t* const measurement);

/**
 * @brief: Request a new measurement of the AHT21 device. 
 * 
 * @param I2C_DeviceAddr: The I2C device address. 
 */
static void vAHT21_StartMeasurement(uint8_t I2C_DeviceAddr);



/* *** public functions *** */

void vAHT21_Init(sAHT21_t const* const _this)
{
    // Initialize I2C
    i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = _this->I2C_SDA_Pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = _this->I2C_SCL_Pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    // Wait at least 100ms after startup before requesting status byte
    vTaskDelay(100/portTICK_PERIOD_MS); 

    // Check AHT21 status
    vAHT21_CheckStatus(_this);
}

f32_t f32AHT21_GetTemperature(sAHT21_t const* const _this)
{
    vAHT21_StartMeasurement(_this->I2C_DeviceAddr);

    sAHT21_Data_t measurement;
    vAHT21_ReadMeasurement(_this->I2C_DeviceAddr, &measurement);

    // Conversion function from datasheet p.13
    return ( (((float)measurement.temperature / (1 << 20U)) * 200.0) - 50 );
}

f32_t f32AHT21_GetHumidity(sAHT21_t const* const _this)
{
    vAHT21_StartMeasurement(_this->I2C_DeviceAddr);

    sAHT21_Data_t measurement;
    vAHT21_ReadMeasurement(_this->I2C_DeviceAddr, &measurement);

    // Conversion function from datasheet p.13
    // Note: Changed multiplication by 100% to multiplication by 100.
    return (((float)measurement.humidity / (1 << 20U)) * 100);
}



/* *** private functions *** */

static void vAHT21_CheckStatus(sAHT21_t const* const _this)
{
    // Send read request
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (_this->I2C_DeviceAddr << 1U) | I2C_MASTER_READ, I2C_MASTER_ACK);

    // Read status byte
    uint8_t status;
    i2c_master_read(i2c_cmd, &status, 1U, I2C_MASTER_LAST_NACK);

    // End data transfer
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(i2c_master_port, i2c_cmd, portMAX_DELAY);
    i2c_cmd_link_delete(i2c_cmd);

    // Check if ATH21 is calibrated
    if( (status & AHT21_STATUS_CHECK_MASK) != AHT21_STATUS_OK )
    {
        // ToDo: recalibrate the sensor (lookup on the official website)
        printf("ATH21 INITIALIZATION ERROR!\n");
    }
}

static void vAHT21_ReadMeasurement(uint8_t I2C_DeviceAddr, sAHT21_Data_t* const measurement)
{
    // Wait 80ms before reading measurement
    vTaskDelay(80/portTICK_PERIOD_MS);

    // Send read request
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (I2C_DeviceAddr << 1U) | I2C_MASTER_READ, I2C_MASTER_ACK);

    // Read status byte
    uint8_t data[7U];
    i2c_master_read(i2c_cmd, data, 7U, I2C_MASTER_LAST_NACK);

    // End data transfer
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(i2c_master_port, i2c_cmd, portMAX_DELAY);
    i2c_cmd_link_delete(i2c_cmd);

    measurement->humidity = ( (data[1] << 12U) | (data[2] << 4U) | ((data[3U] & 0xF0) >> 4U) );
    measurement->temperature = ( ((data[3] & 0x0F) << 16U) | (data[4] << 8U) | (data[5]) );
}

static void vAHT21_StartMeasurement(uint8_t I2C_DeviceAddr)
{
    // Wait 10ms before sending measurement command
    vTaskDelay(10/portTICK_PERIOD_MS);

    // Send which register should get written to
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (I2C_DeviceAddr << 1U) | I2C_MASTER_WRITE, I2C_MASTER_ACK);

    // Trigger measurement
    i2c_master_write_byte(i2c_cmd, AHT21_TRIGGER_MEASUREMENT, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, AHT21_MEASUREMENT_DATA0, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, AHT21_MEASUREMENT_DATA1, I2C_MASTER_ACK);

    // End data transfer
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(i2c_master_port, i2c_cmd, portMAX_DELAY);
    i2c_cmd_link_delete(i2c_cmd);
}
