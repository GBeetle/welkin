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
 #include "spi_bus.h"

//#define CONFIG_SPIBUS_LOG_READWRITES


#if defined   CONFIG_SPIBUS_LOG_RW_LEVEL_INFO
#define SPIBUS_LOG_RW(format, ... ) WK_DEBUGI(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_SPIBUS_LOG_RW_LEVEL_DEBUG
#define SPIBUS_LOG_RW(format, ... ) WK_DEBUGD(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_SPIBUS_LOG_RW_LEVEL_VERBOSE
#define SPIBUS_LOG_RW(format, ... ) WK_DEBUGV(TAG, format, ##__VA_ARGS__)
#endif
#define SPIBUS_LOGE(format, ... )   WK_DEBUGE(TAG, format, ##__VA_ARGS__)


static const char* TAG __attribute__((unused)) = "SPIbus";

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
struct spi fspi;
struct spi hspi;
//init_spi(&fspi, SPI2_HOST);
//init_spi(&hspi, SPI3_HOST);

static esp_err_t begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz);
static esp_err_t close(struct spi *spi);
static esp_err_t addDevice(struct spi *spi, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle);
static esp_err_t addDevice_cfg(struct spi *spi, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);
static esp_err_t removeDevice(struct spi *spi, spi_device_handle_t handle);
/*******************************************************************************
 * WRITING
 ******************************************************************************/
static esp_err_t writeBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static esp_err_t writeBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
static esp_err_t writeByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
static esp_err_t writeBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
/*******************************************************************************
 * READING
 ******************************************************************************/
static esp_err_t readBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
static esp_err_t readBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
static esp_err_t readByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);
static esp_err_t readBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);

/* Get default objects */
struct spi* getSPI(spi_host_device_t host) {
    return host == 1 ? &fspi : &hspi;
}

/*******************************************************************************
 * SETUP
 ******************************************************************************/
void init_spi(struct spi *spi, spi_host_device_t host){
    spi->host = host;
    spi->begin = &begin;
    spi->close = &close;
    spi->addDevice = &addDevice;
    spi->addDevice_cfg = &addDevice_cfg;
    spi->removeDevice = &removeDevice;
    spi->writeBit = &writeBit;
    spi->writeBits = &writeBits;
    spi->writeByte = &writeByte;
    spi->writeBytes = &writeBytes;
    spi->readBit = &readBit;
    spi->readBits = &readBits;
    spi->readByte = &readByte;
    spi->readBytes = &readBytes;
    // must delay some time ??? don't know why
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static esp_err_t begin(struct spi *spi, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz) {
    spi_bus_config_t config;
    config.mosi_io_num = mosi_io_num;
    config.miso_io_num = miso_io_num;
    config.sclk_io_num = sclk_io_num;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = max_transfer_sz;
    config.flags = SPICOMMON_BUSFLAG_NATIVE_PINS;
    return spi_bus_initialize(spi->host, &config, SPI_DMA_DISABLED);  // 0 DMA not used
}

static esp_err_t close(struct spi *spi) {
    return spi_bus_free(spi->host);
}

static esp_err_t addDevice(struct spi *spi, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle) {
    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
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
    return spi_bus_add_device(spi->host, &dev_config, handle);
}

static esp_err_t addDevice_cfg(struct spi *spi, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle) {
    dev_config->address_bits = 8;  // must be set, SPIbus uses this 8-bits to send the regAddr
    return spi_bus_add_device(spi->host, dev_config, handle);
}

static esp_err_t removeDevice(struct spi *spi, spi_device_handle_t handle) {
    return spi_bus_remove_device(handle);
}


/*******************************************************************************
 * WRITING
 ******************************************************************************/
static esp_err_t writeBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t buffer;
    esp_err_t err = spi->readByte(spi, handle, regAddr, &buffer);
    if (err) return err;
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    return spi->writeByte(spi, handle, regAddr, buffer);
}

static esp_err_t writeBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t buffer;
    esp_err_t err = spi->readByte(spi, handle, regAddr, &buffer);
    if (err) return err;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    return spi->writeByte(spi, handle, regAddr, buffer);
}

static esp_err_t writeByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
    return spi->writeBytes(spi, handle, regAddr, 1, &data);
}

static esp_err_t writeBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
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
#if defined CONFIG_SPIBUS_LOG_READWRITES
    if (!err) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s\n", (spi->host == 1 ? "FSPI" : "HSPI"), (uint32_t)handle, length, regAddr, str);
    }
#endif
    return err;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
static esp_err_t readBit(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    return spi->readBits(spi, handle, regAddr, bitNum, 1, data);
}

static esp_err_t readBits(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t buffer;
    esp_err_t err = spi->readByte(spi, handle, regAddr, &buffer);
    if(!err) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        buffer &= mask;
        buffer >>= (bitStart - length + 1);
        *data = buffer;
    }
    return err;
}

static esp_err_t readByte(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) {
    return spi->readBytes(spi, handle, regAddr, 1, data);
}

static esp_err_t readBytes(struct spi *spi, spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
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
#if defined CONFIG_SPIBUS_LOG_READWRITES
    if (!err) {
        char str[length*5+1];
        for(size_t i = 0; i < length; i++)
        sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        WK_DEBUGE(ERROR_TAG, "[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s\n", (spi->host == 1 ? "FHPI" : "HSPI"), (uint32_t)handle, length, regAddr, str);
    }
#endif
    return err;
}


