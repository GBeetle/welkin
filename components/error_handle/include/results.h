#ifndef _WK_RESULTS__
#define _WK_RESULTS__

#include "log_sys.h"

typedef int32_t WK_RESULT;

#define WK_OK                   0x00000000
#define WK_FAIL                 0x80000000

#define WK_SPI_RW_FAIL          0x80000001
#define WK_SPI_INI_FAIL         0x80000002
#define WK_SPI_FREE_FAIL        0x80000003
#define WK_SPI_CFG_FAIL         0x80000004
#define WK_SPI_RMV_FAIL         0x80000005

#define WK_I2C_RW_FAIL          0x80000010
#define WK_I2C_CFG_FAIL         0x80000011
#define WK_I2C_INS_FAIL         0x80000012
#define WK_I2C_RMV_FAIL         0x80000013
#define WK_I2C_CONNECT_FAIL     0x80000014

#define WK_MPU_NOT_FOUND        0X80000100
#define WK_MPU_DUMP_REG_FAIL    0X80000101
#define WK_MPU_RW_TEST_FAIL     0X80000102

#define WK_COMPASS_W_SCALE      0x80000200
#define WK_COMPASS_W_MODE       0x80000201

#define CHK_RES(val) do {           \
        if (val != WK_OK) {         \
            res = val;              \
            WK_DEBUGE(ERROR_TAG, "[CHK_RES] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
            goto error_exit;        \
        }                           \
    } while(0)

#define CHK_VAL(val) do {           \
        if (val != WK_OK) {         \
            WK_DEBUGE(ERROR_TAG, "[CHK_VAL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
        }                           \
    } while(0)

#define CHK_EXIT(val) do {          \
        if (val != WK_OK) {         \
            WK_DEBUGE(ERROR_TAG, "[CHK_VAL] failed at file: %s, func: %s, line: %d, res = %08x", __FILE__, __FUNCTION__, __LINE__, val); \
        }                           \
        return;                     \
    } while(0)

#endif /* end of include guard: _WK_RESULTS__ */
