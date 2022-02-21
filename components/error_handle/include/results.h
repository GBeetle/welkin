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

#ifndef _WK_RESULTS__
#define _WK_RESULTS__

#include "log_sys.h"

typedef int32_t WK_RESULT;

#define WK_OK                   0x00000000
#define WK_FAIL                 0x80000000

#define WK_CHK_BOOL_FAIL        0x80000500

#define WK_SPI_RW_FAIL          0x80000001
#define WK_SPI_INI_FAIL         0x80000002
#define WK_SPI_FREE_FAIL        0x80000003
#define WK_SPI_CFG_FAIL         0x80000004
#define WK_SPI_RMV_FAIL         0x80000005
#define WK_SPI_INVALID_SIZE     0x80000006

#define WK_I2C_RW_FAIL          0x80000010
#define WK_I2C_CFG_FAIL         0x80000011
#define WK_I2C_INS_FAIL         0x80000012
#define WK_I2C_RMV_FAIL         0x80000013
#define WK_I2C_CONNECT_FAIL     0x80000014

#define WK_MPU_NOT_FOUND        0X80000100
#define WK_MPU_DUMP_REG_FAIL    0X80000101
#define WK_MPU_RW_TEST_FAIL     0X80000102
#define WK_MPU_AUX_RW_FAIL      0x80000103
#define WK_MPU_AUX_NOT_ENABLE   0x80000104
#define WK_MPU_AUX_NOT_FOUND    0x80000105
#define WK_MPU_AUX_LOST_ARB     0x80000106
#define WK_MPU_AUX_RW_TIMEOUT   0x80000106

#define WK_COMPASS_W_SCALE      0x80000200
#define WK_COMPASS_W_MODE       0x80000201

#define WK_BMP_DEV_ID_ERROR     0X80000300
#define WK_BMP_DEVICE_NULL      0x80000301
#define WK_BMP_DEVICE_BUS_NULL  0x80000302

#define WK_RF24_INVALID_PIN     0x80000401
#define WK_RF24_IO_CFG_FAIL     0x80000402
#define WK_RF24_IO_SET_FAIL     0x80000403
#define WK_RF24_W_DATA_FAIL     0x80000404
#define WK_RF24_W_FAST_FAIL     0x80000405

#define CHK_RES(val) do {           \
        if (val != WK_OK) {         \
            res = val;              \
            WK_DEBUGE(CHK_TAG, "[CHK_RES] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            goto error_exit;        \
        }                           \
    } while(0)

#define CHK_BOOL(val) do {          \
        if (!(val)) {                 \
            WK_DEBUGE(CHK_TAG, "[CHK_BOOL] failed at file: %s, func: %s, line: %d, val = %d", __FILE__, __FUNCTION__, __LINE__, val); \
            res = WK_CHK_BOOL_FAIL; \
            goto error_exit;        \
        }                           \
    } while(0)

#define CHK_NULL(val, error_code) do {         \
        if (val == NULL) {         \
            WK_DEBUGE(CHK_TAG, "[CHK_RES] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, error_code); \
            res = error_code;      \
            goto error_exit;       \
        }                          \
    } while(0)

#define CHK_LOGE(x, msg, ...) do { \
        WK_RESULT __ = x; \
        if (__ != WK_OK) { \
            WK_DEBUGE(CHK_TAG, msg, ## __VA_ARGS__); \
            goto error_exit; \
        } \
    } while (0)

#define CHK_VAL(val) do {           \
        if (val != WK_OK) {         \
            WK_DEBUGE(CHK_TAG, "[CHK_VAL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
        }                           \
    } while(0)

#define CHK_EXIT(val) do {          \
        if (val != WK_OK) {         \
            WK_DEBUGE(CHK_TAG, "[CHK_VAL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            return;                 \
        }                           \
    } while(0)

#endif /* end of include guard: _WK_RESULTS__ */
