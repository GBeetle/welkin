/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __BMP280_H__
#define __BMP280_H__

#include <stdint.h>
#include <stdbool.h>
#include <error_handle.h>
#include <mpu_types.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <esp_timer.h>

typedef enum {  //
    BMP280_I2C_ADDRESS_0  = 0x76, //!< I2C address when SDO pin is low,
    BMP280_I2C_ADDRESS_1 = 0x77 //!< I2C address when SDO pin is high
} bmp_i2caddr_t;

#ifdef CONFIG_BMP_I2C
#include "i2c_bus.h"
typedef struct i2c bmp_bus_t;
typedef bmp_i2caddr_t bmp_addr_handle_t;
#elif CONFIG_BMP_SPI
#include "spi_bus.h"
typedef struct spi bmp_bus_t;
typedef spi_device_handle_t bmp_addr_handle_t;
#endif

#define BMP280_CHIP_ID  0x58 //!< BMP280 has chip-id 0x58
#define BME280_CHIP_ID  0x60 //!< BME280 has chip-id 0x60

/**
 * Mode of BMP280 module operation.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,  //!< Sleep mode
    BMP280_MODE_FORCED = 1, //!< Measurement is initiated by user
    BMP280_MODE_NORMAL = 3  //!< Continues measurement
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED = 0,          //!< no measurement
    BMP280_ULTRA_LOW_POWER = 1,  //!< oversampling x1
    BMP280_LOW_POWER = 2,        //!< oversampling x2
    BMP280_STANDARD = 3,         //!< oversampling x4
    BMP280_HIGH_RES = 4,         //!< oversampling x8
    BMP280_ULTRA_HIGH_RES = 5    //!< oversampling x16
} BMP280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 = 0,      //!< stand by time 0.5ms
    BMP280_STANDBY_62 = 1,      //!< stand by time 62.5ms
    BMP280_STANDBY_125 = 2,     //!< stand by time 125ms
    BMP280_STANDBY_250 = 3,     //!< stand by time 250ms
    BMP280_STANDBY_500 = 4,     //!< stand by time 500ms
    BMP280_STANDBY_1000 = 5,    //!< stand by time 1s
    BMP280_STANDBY_2000 = 6,    //!< stand by time 2s BMP280, 10ms BME280
    BMP280_STANDBY_4000 = 7,    //!< stand by time 4s BMP280, 20ms BME280
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function ::bmp280_init_default_params() to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;

/**
 * Device descriptor
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    bmp_bus_t *bus;     //!< I2C(SPI) device descriptor
    bmp_addr_handle_t addr;
    uint8_t   id;       //!< Chip ID
} bmp280_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param bus Device bus(i2c | spi)
 * @param addr BMP280 address
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_init_desc(bmp280_t *dev, bmp_bus_t* bus, bmp_addr_handle_t addr);

/**
 * @brief Initialize default parameters
 *
 * Default configuration:
 *
 *  - mode: NORMAL
 *  - filter: OFF
 *  - oversampling: x4
 *  - standby time: 250ms
 *
 * @param[out] params Default parameters
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_init_default_params(bmp280_params_t *params);

/**
 * @brief Initialize BMP280 module
 *
 * Probes for the device, soft resets the device, reads the calibration
 * constants, and configures the device using the supplied parameters.
 *
 * This may be called again to soft reset the device and initialize it again.
 *
 * @param dev Device descriptor
 * @param params Parameters
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_init(bmp280_t *dev, bmp280_params_t *params);

/**
 * @brief Start measurement in forced mode
 *
 * The module remains in forced mode after this call.
 * Do not call this method in normal mode.
 *
 * @param dev Device descriptor
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_force_measurement(bmp280_t *dev);

/**
 * @brief Check if BMP280 is busy
 *
 * @param dev Device descriptor
 * @param[out] busy true if BMP280 measures temperature/pressure
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_is_measuring(bmp280_t *dev, bool *busy);

/**
 * @brief Read raw compensated temperature and pressure data
 *
 * Temperature in degrees Celsius times 100.
 *
 * Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 * Humidity is optional and only read for the BME280, in percent relative
 * humidity as a fixed point 22 bit integer and 10 bit fraction format.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, deg.C * 100
 * @param[out] pressure Pressure
 * @param[out] humidity Humidity, optional
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_read_fixed(bmp280_t *dev, int32_t *temperature,
                            uint32_t *pressure, uint32_t *humidity);

/**
 * @brief Read compensated temperature and pressure data
 *
 * Humidity is optional and only read for the BME280.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, deg.C
 * @param[out] pressure Pressure, Pascal
 * @param[out] humidity Relative humidity, percents (optional)
 * @return `WK_OK` on success
 */
WK_RESULT bmp280_read_float(bmp280_t *dev, float *temperature,
                            float *pressure, float *humidity);


/**@}*/

WK_RESULT altitudeUpdate(float *altitude);
float pressureToAltitude(const float pressure);

extern bmp280_t bmp280_device;

#endif  // __BMP280_H__

