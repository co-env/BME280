# BME280
 BME280 Sensor Library

## Pinout 
SDA - GPIO18; SCL - GPIO19 

CSB em VCC

**Address** 0x77 com SD0 em VCC; 
0x76 com SD0 em GND

## Chip
SPI max 10MHz, I2C max 3.4MHz

Modes:
- sleep
- **forced**
- normal

### Data readout
faster to read pressure even if there was no measurement

#### Burst read uncompensated data
0xF7 to 0xFE (temp, press, hum)
temp e press 20bit unsigned
hum 16bit unsigned

#### Compensation
Using calibration parameters (trimming values)
Especific from the device, set in production (can't be changed)
0x88 to 0xA1, and 0xE1 to 0xF0
> Given formulas in page 25

### Chip ID
register address    0xD0
expected answer     0x60

### Configurations
0xF5, 0xF4

# Bosch Driver

[github /BoschSensortec/BME280_driver](https://github.com/BoschSensortec/BME280_driver)

# I2C 
(kolban's book page 271 - Using I2C)

> The ESP-IDF provides an I2C driver that allows us to control I2C functions from a C program at a high level without having to resort to low level register manipulation.

## **Configure pins** 
> Before we can use an I2C port, we need to configure it.Arbitrary exposed pins can be used for I2C.

## **Configure environment**

> To use I2C, we must configure the I2C environment

`i2c_param_config()` 
- port
- structure with details on the configuration
  - mode: I2C_MODE_MASTER or I2C_MODE_SLAVE
  - sda_io_num, scl_io_num (any two arbitrary pins)
  - sda_pullup_en, scl_pullup_en 
  - clk_speed 

`i2c_driver_install()`
- port
- buffer sizes if in slave mode

## Actions with I2C

### Command structure

> To send a command, we build the structure of that command then ask for it to be transmitted.

`i2c_cmd_link_create()` 
- command handle

`i2c_master_start(cmd)` 
- declare that the command (cmd) starts with an I2C request

`i2c_master_write_byte()`/`i2c_master_write()`
- associate the data we want to send
- Read/Write operation: constants I2C_MASTER_READ and I2C_MASTER_WRITE
  - (ADDRESS << 1) | I2C_MASTER_READ
  - (ADDRESS << 1) | I2C_MASTER_WRITE


`i2c_master_stop(cmd)` 
- indicate the command population is completed

### Perform commands

`i2c_master_cmd_begin()`
- all commands buffered are transmitted

> release the command handle and create a new one for the next transmission

