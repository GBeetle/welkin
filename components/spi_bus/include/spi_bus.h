/* =========================================================================
SPIbus library is placed under the MIT License
Copyright 2017 Natanael Josue Rabello. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
 ========================================================================= */

#ifndef _SPIBUS_H_
#define _SPIBUS_H_

#include <stdint.h>

#include <stdio.h>
 #include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
 #include "driver/spi_common.h"
 #include "driver/spi_master.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "sdkconfig.h"

// Defaults
#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

// Forward declaration
struct spi;

// Default objects
extern struct spi fspi;
extern struct spi hspi;

void init_spi(struct spi *spi, spi_host_device_t host);
/* Get default objects */
struct spi* getSPI(spi_host_device_t host);

/* ^^^
 * spi
 * ^^^ */
struct spi {
    spi_host_device_t host;     /*!< HSPI_HOST or VSPI_HOST */

    /**
     * @brief   Config spi bus and initialize
     * @param   mosi_io_num     [GPIO number for Master-out Slave-in]
     * @param   miso_io_num     [GPIO number for Master-in Slave-out]
     * @param   sclk_io_num     [GPIO number for clock line]
     * @param   max_transfer_sz [Maximum transfer size, in bytes. Defaults to 4094 if 0.]
     * @return  - ESP_ERR_INVALID_ARG   if configuration is invalid
     *          - ESP_ERR_INVALID_STATE if host already is in use
     *          - ESP_ERR_NO_MEM        if out of memory
     *          - ESP_OK                on success
     * */
    esp_err_t (*begin)(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz);

    /**
     * @brief   Free the spi bus
     * @warning In order for this to succeed, all devices have to be removed first.
     * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
     *          - ESP_ERR_INVALID_STATE if not all devices on the bus are freed
     *          - ESP_OK                on success
     * */
    esp_err_t (*close)(struct spi *spi);

    /**
     * @brief   Allocate a device on a spi bus. (Up to three devices per peripheral)
     * @param   mode            [spi mode (0-3)]
     * @param   clock_speed_hz  [Clock speed, in Hz]
     * @param   cs_io_num       [ChipSelect GPIO pin for this device, or -1 if not used]
     * @param   handle          [Pointer to variable to hold the device handle]
     * @param   dev_config      [spi interface protocol config for the device (for more custom configs)]
     *                          @see driver/spi_master.h
     * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
     *          - ESP_ERR_NOT_FOUND     if host doesn't have any free CS slots
     *          - ESP_ERR_NO_MEM        if out of memory
     *          - ESP_OK                on success
     * */
    esp_err_t (*addDevice)(struct spi *spi, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle);
    esp_err_t (*addDevice_cfg)(struct spi *spi, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);
    esp_err_t (*removeDevice)(struct spi *spi, spi_device_handle_t handle);

    /**
     * *** WRITING interface ***
     * @brief  spi commands for writing to a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes. So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  handle    [spi device handle]
     * @param  regAddr   [Register address to write to]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Value(s) to be write to the register]
     * @param  length    [Number of bytes to write (should be within the data buffer size)]
     *                   [writeBits() -> Number of bits after bitStart (including)]
     * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
     *          - ESP_OK                on success
     */
    esp_err_t (*writeBit)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t (*writeBits)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t (*writeByte)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
    esp_err_t (*writeBytes)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);

    /**
     * *** READING interface ***
     * @breif  spi commands for reading a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes.So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  handle    [spi device handle]
     * @param  regAddr   [Register address to read from]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Buffer to store the read value(s)]
     * @param  length    [Number of bytes to read (should be within the data buffer size)]
     * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
     *          - ESP_OK                on success
     */
    esp_err_t (*readBit)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    esp_err_t (*readBits)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    esp_err_t (*readByte)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);
    esp_err_t (*readBytes)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);
};

#endif  // end of include guard: _SPIBUS_HPP_
