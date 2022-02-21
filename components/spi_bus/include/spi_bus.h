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
 #include "log_sys.h"
 #include "error_handle.h"
 #include "esp_intr_alloc.h"

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
     * @return  - WK_SPI_INI_FAIL      spi init failed
     *          - WK_OK                on success
     * */
    WK_RESULT (*begin)(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz);

    /**
     * @brief   Free the spi bus
     * @warning In order for this to succeed, all devices have to be removed first.
     * @return  - WK_SPI_FREE_FAIL     spi free failed
     *          - WK_OK                on success
     * */
    WK_RESULT (*close)(struct spi *spi);

    /**
     * @brief   Allocate a device on a spi bus. (Up to three devices per peripheral)
     * @param   mode            [spi mode (0-3)]
     * @param   clock_speed_hz  [Clock speed, in Hz]
     * @param   cs_io_num       [ChipSelect GPIO pin for this device, or -1 if not used]
     * @param   handle          [Pointer to variable to hold the device handle]
     * @param   dev_config      [spi interface protocol config for the device (for more custom configs)]
     *                          @see driver/spi_master.h
     * @return  - WK_SPI_CFG_FAIL      spi config failed
     *          - WK_OK                on success
     * */
    WK_RESULT (*addDevice)(struct spi *spi, uint8_t address_len, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle);
    WK_RESULT (*addDevice_cfg)(struct spi *spi, uint8_t address_len, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);
    WK_RESULT (*removeDevice)(struct spi *spi, spi_device_handle_t handle);

    /**
     * *** WRITING interface ***
     * @brief  spi commands for writing to a 8-bit slave device register.
     *         All of them returns standard WK_RESULT codes. So it can be used
     *         with CHK_RES();
     * @param  handle    [spi device handle]
     * @param  regAddr   [Register address to write to]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Value(s) to be write to the register]
     * @param  length    [Number of bytes to write (should be within the data buffer size)]
     *                   [writeBits() -> Number of bits after bitStart (including)]
     * @return  - WK_SPI_RW_FAIL       spi read/write failed
     *          - WK_OK                on success
     */
    WK_RESULT (*writeBit)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    WK_RESULT (*writeBits)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    WK_RESULT (*writeByte)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
    WK_RESULT (*writeBytes)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);

    /**
     * *** READING interface ***
     * @breif  spi commands for reading a 8-bit slave device register.
     *         All of them returns standard WK_RESULT codes.So it can be used
     *         with CHK_RES();
     * @param  handle    [spi device handle]
     * @param  regAddr   [Register address to read from]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Buffer to store the read value(s)]
     * @param  length    [Number of bytes to read (should be within the data buffer size)]
     * @return  - WK_SPI_RW_FAIL       spi read/write failed
     *          - WK_OK                on success
     */
    WK_RESULT (*readBit)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    WK_RESULT (*readBits)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    WK_RESULT (*readByte)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);
    WK_RESULT (*readBytes)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);
    WK_RESULT (*readWriteBytes)(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *r_data, uint8_t *w_data);
};

#endif  // end of include guard: _SPIBUS_HPP_
