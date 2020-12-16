#include "esp32_bme280.h"

uint8_t dev_addr = BME280_I2C_ADDR;
struct bme280_dev dev;

/**
 * @brief function for printing data on the monitor
 */
static void print_sensor_data(struct bme280_data *comp_data) {
    #ifdef BME280_FLOAT_ENABLE
            printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
    #else
            printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
    #endif
}

/**
 * @brief function for initializing the BME280 sensor using I2C
 */
int8_t bme280_sensor_init(bme280_read_fptr_t user_i2c_read, bme280_write_fptr_t user_i2c_write, bme280_delay_us_fptr_t user_delay_us){
    int8_t rslt = BME280_OK;

    dev.intf_ptr = &dev_addr;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    rslt = bme280_init(&dev);
    return rslt;
}

/**
 * @brief Configure sensor BME280
 * Set oversamplings and IIR filter coef 
 */
int8_t bme280_config(void) {
    int8_t rslt;
    uint8_t settings_sel;

    /* Recommended mode of operation: Weather monitoring */
    dev.settings.osr_h = BME280_OVERSAMPLING_16X; 
    dev.settings.osr_p = BME280_OVERSAMPLING_1X; 
    dev.settings.osr_t = BME280_OVERSAMPLING_16X;
    dev.settings.filter = BME280_FILTER_COEFF_4;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);

    printf("Temperature, Pressure, Humidity\r\n");
    return rslt;
}

/**
 * @brief function for reading sensor data
 * Set mode as forced, start measurement
 * Read sensor data and print on the monitor
 */
int8_t bme280_meas_forcedmode(struct bme280_data *comp_data) { 
/* Continuously stream sensor data - in loop */
    int8_t rslt;
	uint32_t req_delay;

	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev.settings);

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    /* Wait for the measurement to complete and print data @25Hz */
    dev.delay_us(req_delay, dev.intf_ptr);
    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, &dev);
    // print_sensor_data(comp_data); 
    return rslt;
}

/*@*/