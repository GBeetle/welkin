/* =========================================================================
I2Cbus library is placed under the MIT License
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

#ifndef _I2CBUS_HPP_
#define _I2CBUS_HPP_

#include <stdint.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_TIMEOUT_MS 1000


/* ^^^^^^
 * I2Cbus
 * ^^^^^^ */
extern const uint32_t kDefaultClockSpeed;  /*!< Clock speed in Hz, default: 100KHz */
extern const uint32_t kDefaultTimeout;       /*!< Timeout in milliseconds, default: 1000ms */

struct i2c;


// Default Objects
extern struct i2c i2c0;        /*!< port: I2C_NUM_0 */
extern struct i2c i2c1;        /*!< port: I2C_NUM_1 */

/* Get default objects */
struct i2c* getI2C(i2c_port_t port);
/*******************************************************************************
 * SETUP
 ******************************************************************************/
void init_i2c(struct i2c *i2c, i2c_port_t port);


// i2c class definition
struct i2c {
    i2c_port_t port;            /*!< i2c port: I2C_NUM_0 or I2C_NUM_1 */
    uint32_t ticksToWait;       /*!< Timeout in ticks for read and write */

    /** *** i2c Begin ***
     * @brief  Config i2c bus and Install Driver
     * @param  sda_io_num    [GPIO number for SDA line]
     * @param  scl_io_num    [GPIO number for SCL line]
     * @param  sda_pullup_en [Enable internal pullup on SDA line]
     * @param  scl_pullup_en [Enable internal pullup on SCL line]
     * @param  clk_speed     [i2c clock frequency for master mode, (no higher than 1MHz for now), Default 100KHz]
     *                       @see "driver/i2c.h"
     * @return               - ESP_OK   Success
     *                       - ESP_ERR_INVALID_ARG Parameter error
     *                       - ESP_FAIL Driver install error
     */
    esp_err_t (*begin)(struct i2c *i2c, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed);

    esp_err_t (*begin_pull_enable)(struct i2c *i2c, gpio_num_t sda_io_num, gpio_num_t scl_io_num,
                    gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en,
                    uint32_t clk_speed);

    /**
     * Stop i2c bus and unninstall driver
     */
    esp_err_t (*close)(struct i2c *i2c);

    /**
     * Timeout read and write in milliseconds
     */
    void (*setTimeout)(struct i2c *i2c, uint32_t ms);


    /**
     * *** WRITING interface ***
     * @brief  i2c commands for writing to a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes. So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  devAddr   [i2c slave device register]
     * @param  regAddr   [Register address to write to]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Value(s) to be write to the register]
     * @param  length    [Number of bytes to write (should be within the data buffer size)]
     *                   [writeBits() -> Number of bits after bitStart (including)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE i2c driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
     */
    esp_err_t (*writeBit)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t (*writeBits)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t (*writeByte)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    esp_err_t (*writeBytes)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data);

    /**
     * *** READING interface ***
     * @breif  i2c commands for reading a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes.So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  devAddr   [i2c slave device register]
     * @param  regAddr   [Register address to read from]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Buffer to store the read value(s)]
     * @param  length    [Number of bytes to read (should be within the data buffer size)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE i2c driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.]
     */
    esp_err_t (*readBit)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    esp_err_t (*readBits)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    esp_err_t (*readByte)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data);
    esp_err_t (*readBytes)(struct i2c *i2c, uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data);

    /**
     * @brief  Quick check to see if a slave device responds.
     * @param  devAddr   [i2c slave device register]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE i2c driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.]
     */
    esp_err_t (*testConnection)(struct i2c *i2c, uint8_t devAddr, int32_t timeout);

    /**
     * i2c scanner utility, prints out all device addresses found on this i2c bus.
     */
    void (*scanner)(struct i2c *i2c);
};


#endif /* end of include guard: _I2CBUS_H_ */
