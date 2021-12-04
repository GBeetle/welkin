/*
 * Driver for LIS3MDL 3-axes digital magnetometer connected to I2C or SPI.
 *
 * This driver is for the usage with the ESP8266 and FreeRTOS (esp-open-rtos)
 * [https://github.com/SuperHouse/esp-open-rtos]. It is also working with ESP32
 * and ESP-IDF [https://github.com/espressif/esp-idf.git] as well as Linux
 * based systems using a wrapper library for ESP8266 functions.
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2017 Gunar Schorcht (https://github.com/gschorcht)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The information provided is believed to be accurate and reliable. The
 * copyright holder assumes no responsibility for the consequences of use
 * of such information nor for any infringement of patents or other rights
 * of third parties which may result from its use. No license is granted by
 * implication or otherwise under any patent or patent rights of the copyright
 * holder.
 */

#include <string.h>
#include <stdlib.h>

#include "lis3mdl.h"

#if defined(LIS3MDL_DEBUG_LEVEL_2)
#define debug(s, f, ...) printf("%s %s: " s "\n", "LIS3MDL", f, ## __VA_ARGS__)
#define debug_dev(s, f, d, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3MDL", f, d->bus, d->addr, ## __VA_ARGS__)
#else
#define debug(s, f, ...)
#define debug_dev(s, f, d, ...)
#endif

#if defined(LIS3MDL_DEBUG_LEVEL_1) || defined(LIS3MDL_DEBUG_LEVEL_2)
#define error(s, f, ...) printf("%s %s: " s "\n", "LIS3MDL", f, ## __VA_ARGS__)
#define error_dev(s, f, d, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3MDL", f, d->bus, d->addr, ## __VA_ARGS__)
#else
#define error(s, f, ...)
#define error_dev(s, f, d, ...)
#endif

// register structure definitions
struct lis3mdl_reg_status 
{
    uint8_t XDA      :1;    // STATUS<0>   X axis new data available
    uint8_t YDA      :1;    // STATUS<1>   Y axis new data available
    uint8_t ZDA      :1;    // STATUS<2>   Z axis new data available
    uint8_t ZYXDA    :1;    // STATUS<3>   X, Y and Z axis new data available
    uint8_t XOR      :1;    // STATUS<4>   X axis data overrun
    uint8_t YOR      :1;    // STATUS<5>   Y axis data overrun 
    uint8_t ZOR      :1;    // STATUS<6>   Z axis data overrun
    uint8_t ZYXOR    :1;    // STATUS<7>   X, Y and Z axis data overrun
};

#define LIS3MDL_ANY_DATA_READY    0x0f    // LIS3MDL_REG_STATUS<3:0>

struct lis3mdl_reg_ctrl1 
{
    uint8_t ST       :1;    // CTRL1<0>    Self-test enable
    uint8_t FAST_ODR :1;    // CTRL1<1>    Data rates higher 80 Hz enabled
    uint8_t DO       :3;    // CTRL1<4:2>  Output data rate
    uint8_t OM       :2;    // CTRL1<6:5>  X and Y axes operative mode
    uint8_t TEMP_EN  :1;    // CTRL1<7>    Temperature sensor enabled
};

struct lis3mdl_reg_ctrl2 
{
    uint8_t unused1  :2;    // CTRL2<1:0>  unused
    uint8_t SOFT_RST :1;    // CTRL2<2>    configuration and user regs reset
    uint8_t REBOOT   :1;    // CTRL2<3>    Reboot memory content
    uint8_t unused2  :1;    // CTRL2<4>    unused
    uint8_t FS       :2;    // CTRL2<6:5>  
    uint8_t unused3  :1;    // CTRL2<7>    unused
};

struct lis3mdl_reg_ctrl3 
{
    uint8_t MD       :2;    // CTRL3<1:0>  Operation mode selection
    uint8_t SIM      :1;    // CTRL3<2>    SPI serial interface mode selection
    uint8_t unused1  :2;    // CTRL3<4:3>  unused
    uint8_t LP       :1;    // CTRL3<5>    Low power mode configuration
    uint8_t unused2  :2;    // CTRL3<7:6>  unused
};

struct lis3mdl_reg_ctrl4 
{
    uint8_t unused1  :1;    // CTRL4<0>    unused
    uint8_t BLE      :1;    // CTRL4<1>    Big/litle endian data selection
    uint8_t OMZ      :2;    // CTRL4<3:2>  Z axis operative mode
    uint8_t unused2  :4;    // CTRL4<7:4>  unused
};

struct lis3mdl_reg_ctrl5 
{
    uint8_t unused   :6;    // CTRL5<5:0>  unused
    uint8_t BDU      :1;    // CTRL5<6>    Block data update
    uint8_t FAST_READ:1;    // CTRL5<7>    Fast read enabled
};


struct lis3mdl_reg_int_cfg
{
    uint8_t IEN      :1;    // INT_CFG<0>   Interrupt enabled
    uint8_t LIR      :1;    // INT_CFG<1>   Latch interrupt request
    uint8_t IEA      :1;    // INT_CFG<2>   Interrupt active
    uint8_t unused   :2;    // INT_CFG<4:3> unused
    uint8_t ZIEN     :1;    // INT_CFG<5>   Z axis threshold interrupt enabled
    uint8_t YIEN     :1;    // INT_CFG<6>   Y axis threshold interrupt enabled
    uint8_t XIEN     :1;    // INT_CFG<7>   X axis threshold interrupt enabled
};

struct lis3mdl_reg_int_src
{
    uint8_t PTH_X    :1;    // INT_SRC<0>   X exceeds threshold on positive side
    uint8_t PTH_Y    :1;    // INT_SRC<1>   Y exceeds threshold on positive side
    uint8_t PTH_Z    :1;    // INT_SRC<2>   Z exceeds threshold on positive side
    uint8_t NTH_X    :1;    // INT_SRC<3>   X exceeds threshold on negative side
    uint8_t NTH_Y    :1;    // INT_SRC<4>   Y exceeds threshold on negative side
    uint8_t NTH_Z    :1;    // INT_SRC<5>   Z exceeds threshold on negative side
    uint8_t MROI     :1;    // INT_SRC<6>   Internal measurement range overflow
    uint8_t INT      :1;    // INT_SRC<7>   Interrupt event occurs
};

#define msb_lsb_to_type(t,b,o) (t)(((t)b[o] << 8) | b[o+1])
#define lsb_msb_to_type(t,b,o) (t)(((t)b[o+1] << 8) | b[o])
#define lsb_to_type(t,b,o)     (t)(b[o])

#define lis3mdl_update_reg(dev,addr,type,elem,value) \
        { \
            struct type __reg; \
            if (!lis3mdl_reg_read (dev, (addr), (uint8_t*)&__reg, 1)) \
                return false; \
            __reg.elem = (value); \
            if (!lis3mdl_reg_write (dev, (addr), (uint8_t*)&__reg, 1)) \
                return false; \
        }
