#include <stdio.h>
#include "esp_log.h"

#include "bme280.h"

// #define BME280_I2C_ADDR     BME280_I2C_ADDR_SEC
#define BME280_I2C_ADDR     BME280_I2C_ADDR_PRIM

uint8_t dev_addr;

/**
 * @brief function for initializing the BME280 sensor using I2C
 */
int8_t bme280_sensor_init(bme280_read_fptr_t user_i2c_read, 
                        bme280_write_fptr_t user_i2c_write, bme280_delay_us_fptr_t user_delay_us);

/**
 * @brief Configure sensor BME280
 * Set oversamplings and IIR filter coef 
 */
int8_t bme280_config();

/**
 * @brief function for reading sensor data
 * Set mode as forced, start measurement
 * Read sensor data and print on the monitor
 */
int8_t bme280_meas_forcedmode(struct bme280_data *comp_data);
