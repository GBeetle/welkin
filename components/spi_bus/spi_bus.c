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

 #include "spi_bus.h"

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct spi fspi;
struct spi hspi;
//init_spi(&fspi, SPI2_HOST);
//init_spi(&hspi, SPI3_HOST);

static WK_RESULT begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz);
static WK_RESULT close(struct spi *spi);
static WK_RESULT addDevice(struct spi *spi, uint8_t address_len, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle);
static WK_RESULT addDevice_cfg(struct spi *spi, uint8_t address_len, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);
static WK_RESULT removeDevice(struct spi *spi, spi_device_handle_t handle);
/*******************************************************************************
 * WRITING
 ******************************************************************************/
static WK_RESULT writeBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static WK_RESULT writeBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
static WK_RESULT writeByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
static WK_RESULT writeBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
/*******************************************************************************
 * READING
 ******************************************************************************/
static WK_RESULT readBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
static WK_RESULT readBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
static WK_RESULT readByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);
static WK_RESULT readBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);
static WK_RESULT readWriteBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *r_data, uint8_t *w_data);

/* Get default objects */
struct spi* getSPI(spi_host_device_t host) {
    return host == 1 ? &fspi : &hspi;
}

/*******************************************************************************
 * SETUP
 ******************************************************************************/
void init_spi(struct spi *spi, spi_host_device_t host){
    spi->host           = host;
    spi->begin          = &begin;
    spi->close          = &close;
    spi->addDevice      = &addDevice;
    spi->addDevice_cfg  = &addDevice_cfg;
    spi->removeDevice   = &removeDevice;
    spi->writeBit       = &writeBit;
    spi->writeBits      = &writeBits;
    spi->writeByte      = &writeByte;
    spi->writeBytes     = &writeBytes;
    spi->readBit        = &readBit;
    spi->readBits       = &readBits;
    spi->readByte       = &readByte;
    spi->readBytes      = &readBytes;
    spi->readWriteBytes = &readWriteBytes;
    // must delay some time ??? don't know why
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static WK_RESULT begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz) {
    spi_bus_config_t config = {0};
    config.mosi_io_num = mosi_io_num;
    config.miso_io_num = miso_io_num;
    config.sclk_io_num = sclk_io_num;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = max_transfer_sz;
    config.flags = SPICOMMON_BUSFLAG_MASTER;
    esp_err_t err = spi_bus_initialize(spi->host, &config, SPI_DMA_DISABLED);
    if(err != ESP_OK) {
        WK_DEBUGE(ERROR_TAG, "spi init failed, error: %s\n", esp_err_to_name(err));
        return WK_SPI_INI_FAIL;
    }  // 0 DMA not used
    return WK_OK;
}

static WK_RESULT close(struct spi *spi) {
    if(spi_bus_free(spi->host) != ESP_OK) {
        return WK_SPI_FREE_FAIL;
    }
    return WK_OK;
}

static WK_RESULT addDevice(struct spi *spi, uint8_t address_len, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle) {
    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = address_len;
    dev_config.dummy_bits = 0;
    dev_config.mode = mode;
    dev_config.duty_cycle_pos = 128;  // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans = 0;  // 0 not used
    dev_config.cs_ena_posttrans = 0;  // 0 not used
    dev_config.clock_speed_hz = clock_speed_hz;
    dev_config.spics_io_num = cs_io_num;
    dev_config.flags = 0;  // 0 not used
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    if (spi_bus_add_device(spi->host, &dev_config, handle) != ESP_OK) {
        return WK_SPI_CFG_FAIL;
    }
    return WK_OK;
}

static WK_RESULT addDevice_cfg(struct spi *spi, uint8_t address_len, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle) {
    dev_config->address_bits = address_len;  // must be set, SPIbus uses this 8-bits to send the regAddr
    if (spi_bus_add_device(spi->host, dev_config, handle) != ESP_OK) {
        return WK_SPI_CFG_FAIL;
    }
    return WK_OK;
}

static WK_RESULT removeDevice(struct spi *spi, spi_device_handle_t handle) {
    if (spi_bus_remove_device(handle) != ESP_OK) {
        return WK_SPI_RMV_FAIL;
    }
    return WK_OK;
}


/*******************************************************************************
 * WRITING
 ******************************************************************************/
static WK_RESULT writeBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t buffer;
    WK_RESULT res = WK_OK;

    CHK_RES(spi->readByte(spi, handle, regAddr, &buffer));
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    CHK_RES(spi->writeByte(spi, handle, regAddr, buffer));
error_exit:
    return res;
}

static WK_RESULT writeBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t buffer;
    WK_RESULT res = WK_OK;

    CHK_RES(spi->readByte(spi, handle, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    CHK_RES(spi->writeByte(spi, handle, regAddr, buffer));
error_exit:
    return res;
}

static WK_RESULT writeByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
    WK_RESULT res = WK_OK;
    CHK_RES(spi->writeBytes(spi, handle, regAddr, 1, &data));
error_exit:
    return res;
}

static WK_RESULT writeBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    if (err != ESP_OK) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s\n", (spi->host == 1 ? "FSPI" : "HSPI"), (uint32_t)handle, length, regAddr, str);
        return WK_SPI_RW_FAIL;
    }
    return WK_OK;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
static WK_RESULT readBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    WK_RESULT res = WK_OK;
    CHK_RES(spi->readBits(spi, handle, regAddr, bitNum, 1, data));
error_exit:
    return res;
}

static WK_RESULT readBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t buffer;
    WK_RESULT res = WK_OK;

    CHK_RES(spi->readByte(spi, handle, regAddr, &buffer));
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    buffer &= mask;
    buffer >>= (bitStart - length + 1);
    *data = buffer;
error_exit:
    return res;
}

static WK_RESULT readByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) {
    WK_RESULT res = WK_OK;

    CHK_RES(spi->readBytes(spi, handle, regAddr, 1, data));
error_exit:
    return res;
}

static WK_RESULT readBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return WK_SPI_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | SPIBUS_READ;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    if (err != ESP_OK) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s\n", (spi->host == 1 ? "FHPI" : "HSPI"), (uint32_t)handle, length, regAddr, str);
        return WK_SPI_RW_FAIL;
    }
    return WK_OK;
}

static WK_RESULT readWriteBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *r_data, uint8_t *w_data) {
    if(length == 0) return WK_SPI_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = w_data;
    transaction.rx_buffer = r_data;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    if (err != ESP_OK) {
        char rstr[length*5+1];
        char wstr[length*5+1];
        for(size_t i = 0; i < length; i++) {
            sprintf(rstr+i*5, "0x%s%X ", (r_data[i] < 0x10 ? "0" : ""), r_data[i]);
            sprintf(wstr+i*5, "0x%s%X ", (w_data[i] < 0x10 ? "0" : ""), w_data[i]);
        }
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s\n", (spi->host == 1 ? "FHPI" : "HSPI"), (uint32_t)handle, length, regAddr, rstr);
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s\n", (spi->host == 1 ? "FSPI" : "HSPI"), (uint32_t)handle, length, regAddr, wstr);
        return WK_SPI_RW_FAIL;
    }
    return WK_OK;
}


