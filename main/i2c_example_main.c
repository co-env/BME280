/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <string.h>

#include "bme280.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_NUM      I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO   CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define ACK_VAL             0x0                             /*!< I2C ack value */
#define NACK_VAL            0x1                            /*!< I2C nack value */
#define ACK_CHECK_EN        0x1                          /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS       0x0                         /*!< I2C master will not check ack from slave */

#define BME280_I2C_ADDR     BME280_I2C_ADDR_SEC
#define I2C_MASTER_FREQ_HZ  100000        /*!< I2C master clock frequency = 1MHZ */

i2c_port_t i2c_num = I2C_MASTER_NUM;
struct bme280_dev dev;

SemaphoreHandle_t xSemaphore = NULL;
static const char *TAG = "BME280_EXAMPLE";

void print_sensor_data(struct bme280_data *comp_data);

/**
 * @brief i2c master initialization
 
 * > Used in BME280, AS7262 and SGP30
 */
static esp_err_t i2c_master_init() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/**
 * @brief generic function for reading I2C data
 * 
 * @param reg_addr register adress to read from 
 * @param reg_data pointer to save the data read 
 * @param len length of data to be read
 * @param intf_ptr pointer to device address
 * 
 * @return ESP_OK/BME280_OK if reading was successful
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    rslt = ret;
    return rslt;

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
    //  * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

}

/**
 * @brief generic function for writing data via I2C 
 *  
 * @param reg_addr register adress to write to 
 * @param reg_data register data to be written 
 * @param len length of data to be written
 * @param intf_ptr pointer to device address
 * 
 * @return ESP_OK/BME280_OK if writing was successful
 */
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    // for (int i = 0; i < len; i++) {
    //     i2c_master_write_byte(cmd, reg_data[i], ACK_CHECK_EN);
    // }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    rslt = ret;
    return rslt;

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

}

/**
 * @brief generic delay function for BME280 library
 */
void user_delay_us(uint32_t period, void *intf_ptr) {
    /**
     * Return control or wait,
     * for a period amount of milliseconds
     */
    TickType_t delay = period / (1000 * portTICK_PERIOD_MS);
    vTaskDelay(delay);
}

/**
 * @brief function for initializing the BME280 sensor using I2C
 */
int8_t bme280_sensor_init(){
    int8_t rslt = BME280_OK;
    uint8_t dev_addr = BME280_I2C_ADDR;

    dev.intf_ptr = &dev_addr;
    dev.intf = BME280_I2C_INTF;
    dev.read = &user_i2c_read;
    dev.write = &user_i2c_write;
    dev.delay_us = &user_delay_us;

    rslt = bme280_init(&dev);
    return rslt;
}

/**
 * @brief Configure sensor BME280
 * Set oversamplings and IIR filter coef 
 */
int8_t bme280_config(struct bme280_dev *dev) { 
    int8_t rslt;
    uint8_t settings_sel;

    /* Recommended mode of operation: Weather monitoring */
    dev->settings.osr_h = BME280_OVERSAMPLING_16X; //umidade
    dev->settings.osr_p = BME280_OVERSAMPLING_1X; 
    dev->settings.osr_t = BME280_OVERSAMPLING_16X;
    dev->settings.filter = BME280_FILTER_COEFF_4;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);

    printf("Temperature, Pressure, Humidity\r\n");
    return rslt;
}

/**
 * @brief function for reading sensor data
 * Set mode as forced, start measurement
 * Read sensor data and print on the monitor
 */
int8_t bme280_meas_forcedmode(struct bme280_dev *dev) {
/* Continuously stream sensor data - in loop */
    int8_t rslt;
	uint32_t req_delay;
    struct bme280_data comp_data;

	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev->settings);

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
    /* Wait for the measurement to complete and print data @25Hz */
    dev->delay_us(req_delay, dev->intf_ptr);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    print_sensor_data(&comp_data);
    return rslt;
}

/** 
 * @brief BME280 main task 
 */
static void bme280_sensor_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");
    esp_err_t erro = ESP_OK;

    //* init bme280
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        erro = bme280_sensor_init();
        xSemaphoreGive(xSemaphore);
    }

    if(erro == BME280_OK) printf("Init check\n");
    else printf("Could not init BME280\n");

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        erro = bme280_config(&dev);
        xSemaphoreGive(xSemaphore);
    }

    if(erro == BME280_OK) printf("Config check\n");
    else printf("Could not config BME280\n");

    while (1) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            erro = bme280_meas_forcedmode(&dev);
            xSemaphoreGive(xSemaphore);
        }
        if(erro != BME280_OK) printf("Could not measure :(");
    }
    

}

/**
 * @brief function for printing data on the monitor
 */
void print_sensor_data(struct bme280_data *comp_data) {
    #ifdef BME280_FLOAT_ENABLE
            printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
    #else
            printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
    #endif
}

/**
 * @brief main application function 
 */
void app_main() {
    xSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xSemaphore);

    esp_err_t erro = i2c_master_init();

    if(erro == ESP_OK){

        xTaskCreate(bme280_sensor_task, "bme280_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);
        // printf("Sensor init!\n");
        // stream_sensor_data_forced_mode();
    }
    else {
        printf("Seila bixo");
    }

    // while(1) {
        
    // }
}