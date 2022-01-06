// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu.h
 * mpu library main file. Declare mpu class.
 *
 * @attention
 *  mpu library requires I2Cbus or SPIbus library.
 *  Select the communication protocol in `menuconfig`
 *  and include the corresponding library to project components.
 *
 * @note
 *  The following is taken in the code:
 *  - MPU9250 is the same as MPU6500 + AK8963
 *  - MPU9150 is the same as MPU6050 + AK8975
 *  - MPU6000 code equals MPU6050
 *  - MPU6555 code equals MPU6500
 *  - MPU9255 code equals MPU9250
 * */

#ifndef _MPU_9250_H__
#define _MPU_9250_H__

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "mpu_math.h"
#include "registers.h"
#include "mpu_types.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "lis3mdl.h"
#include "isr_manager.h"
#include "AHRS.h"
#include "log_sys.h"
#include "error_handle.h"

#define MPU_ERR_CHECK(x) (x)

#ifdef CONFIG_MPU_I2C
/*
#if !defined I2CBUS_COMPONENT_TRUE
#error ''mpu component requires I2Cbus library. \
Make sure the I2Cbus library is included in your components directory. \
See MPUs README.md for more information.''
#endif
*/
#include "i2c_bus.h"

#elif CONFIG_MPU_SPI
/*
#if !defined SPIBUS_COMPONENT_TRUE
#error ''mpu component requires SPIbus library. \
Make sure the SPIbus library is included in your components directory. \
See MPUs README.md for more information.''
#endif
#include "spi_bus.h"
#else
#error ''mpu communication protocol not specified''
*/
#endif

#include "mpu_types.h"

/*! Motion Processing Unit */
struct mpu
{
    //! \}
    //! \name Basic
    //! \{
    struct mpu* (*setBus)(struct mpu *mpu, mpu_bus_t *bus);
    struct mpu* (*setAddr)(struct mpu *mpu, mpu_addr_handle_t addr);
    mpu_bus_t* (*getBus)(struct mpu *mpu);
    mpu_addr_handle_t (*getAddr)(struct mpu *mpu);
    WK_RESULT (*lastError)(struct mpu *mpu);
    //! \}
    //! \name Setup
    //! \{
    WK_RESULT (*initialize)(struct mpu *mpu);
    WK_RESULT (*reset)(struct mpu *mpu);
    WK_RESULT (*setSleep)(struct mpu *mpu, bool enable);
    WK_RESULT (*testConnection)(struct mpu *mpu);
    WK_RESULT (*selfTest)(struct mpu *mpu, selftest_t* result);
    WK_RESULT (*setGyroBias)(struct mpu *mpu);
    WK_RESULT (*resetSignalPath)(struct mpu *mpu);
    uint8_t (*whoAmI)(struct mpu *mpu);
    bool (*getSleep)(struct mpu *mpu);
    //! \}
    //! \name Main configurations
    //! \{
    WK_RESULT (*setSampleRate)(struct mpu *mpu, uint16_t rate);
    WK_RESULT (*setClockSource)(struct mpu *mpu, clock_src_t clockSrc);
    WK_RESULT (*setDigitalLowPassFilter)(struct mpu *mpu, dlpf_t dlpf);
    uint16_t (*getSampleRate)(struct mpu *mpu);
    clock_src_t (*getClockSource)(struct mpu *mpu);
    dlpf_t (*getDigitalLowPassFilter)(struct mpu *mpu);
    //! \}
    //! \name Power management
    //! \{
    WK_RESULT (*setLowPowerAccelMode)(struct mpu *mpu, bool enable);
    WK_RESULT (*setLowPowerAccelRate)(struct mpu *mpu, lp_accel_rate_t rate);
    lp_accel_rate_t (*getLowPowerAccelRate)(struct mpu *mpu);
    bool (*getLowPowerAccelMode)(struct mpu *mpu);
    WK_RESULT (*setStandbyMode)(struct mpu *mpu, stby_en_t mask);
    stby_en_t (*getStandbyMode)(struct mpu *mpu);
    //! \}
    //! \name Full-Scale Range
    //! \{
    WK_RESULT (*setGyroFullScale)(struct mpu *mpu, gyro_fs_t fsr);
    WK_RESULT (*setAccelFullScale)(struct mpu *mpu, accel_fs_t fsr);
    gyro_fs_t (*getGyroFullScale)(struct mpu *mpu);
    accel_fs_t (*getAccelFullScale)(struct mpu *mpu);
    //! \}
    //! \name Offset / Bias
    //! \{
    WK_RESULT (*setGyroOffset)(struct mpu *mpu, raw_axes_t bias);
    WK_RESULT (*setAccelOffset)(struct mpu *mpu, raw_axes_t bias);
    raw_axes_t (*getGyroOffset)(struct mpu *mpu);
    raw_axes_t (*getAccelOffset)(struct mpu *mpu);
    WK_RESULT (*computeOffsets)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    WK_RESULT (*setInterruptConfig)(struct mpu *mpu, int_config_t config);
    WK_RESULT (*setInterruptEnabled)(struct mpu *mpu, int_en_t mask);
    int_stat_t (*getInterruptStatus)(struct mpu *mpu);
    int_config_t (*getInterruptConfig)(struct mpu *mpu);
    int_en_t (*getInterruptEnabled)(struct mpu *mpu);
    //! \}
    //! \name FIFO
    //! \{
    WK_RESULT (*setFIFOMode)(struct mpu *mpu, fifo_mode_t mode);
    WK_RESULT (*setFIFOConfig)(struct mpu *mpu, fifo_config_t config);
    WK_RESULT (*setFIFOEnabled)(struct mpu *mpu, bool enable);
    WK_RESULT (*resetFIFO)(struct mpu *mpu);
    uint16_t (*getFIFOCount)(struct mpu *mpu);
    WK_RESULT (*readFIFO)(struct mpu *mpu, size_t length, uint8_t* data);
    WK_RESULT (*writeFIFO)(struct mpu *mpu, size_t length, const uint8_t* data);
    fifo_mode_t (*getFIFOMode)(struct mpu *mpu);
    fifo_config_t (*getFIFOConfig)(struct mpu *mpu);
    bool (*getFIFOEnabled)(struct mpu *mpu);
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    WK_RESULT (*setAuxI2CConfig)(struct mpu *mpu, const auxi2c_config_t *config);
    WK_RESULT (*setAuxI2CEnabled)(struct mpu *mpu, bool enable);
    WK_RESULT (*setAuxI2CReset)(struct mpu *mpu);
    WK_RESULT (*setAuxI2CSlaveConfig)(struct mpu *mpu, const auxi2c_slv_config_t *config);
    WK_RESULT (*setAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave, bool enable);
    WK_RESULT (*setAuxI2CBypass)(struct mpu *mpu, bool enable);
    WK_RESULT (*readAuxI2CRxData)(struct mpu *mpu, size_t length, uint8_t* data, size_t skip);
    WK_RESULT (*restartAuxI2C)(struct mpu *mpu);
    auxi2c_stat_t (*getAuxI2CStatus)(struct mpu *mpu);
    auxi2c_config_t (*getAuxI2CConfig)(struct mpu *mpu);
    auxi2c_slv_config_t (*getAuxI2CSlaveConfig)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CEnabled)(struct mpu *mpu);
    bool (*getAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CBypass)(struct mpu *mpu);
    WK_RESULT (*auxI2CWriteByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    WK_RESULT (*auxI2CReadByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    WK_RESULT (*setMotionDetectConfig)(struct mpu *mpu, mot_config_t *config);
    mot_config_t (*getMotionDetectConfig)(struct mpu *mpu);
    WK_RESULT (*setMotionFeatureEnabled)(struct mpu *mpu, bool enable);
    bool (*getMotionFeatureEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    WK_RESULT (*setZeroMotionConfig)(struct mpu *mpu, zrmot_config_t *config);
    zrmot_config_t (*getZeroMotionConfig)(struct mpu *mpu);
    WK_RESULT (*setFreeFallConfig)(struct mpu *mpu, ff_config_t *config);
    ff_config_t (*getFreeFallConfig)(struct mpu *mpu);
    mot_stat_t (*getMotionDetectStatus)(struct mpu *mpu);
#endif
    //! \}
    //! \name Compass | Magnetometer
    //! \{
#if defined CONFIG_LIS3MDL
    WK_RESULT (*compassInit)(struct mpu *mpu);
    WK_RESULT (*compassWhoAmI)(struct mpu *mpu);
    WK_RESULT (*compassReset)(struct mpu *mpu);
    WK_RESULT (*compassReadByte)(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
    WK_RESULT (*compassWriteByte)(struct mpu *mpu, uint8_t regAddr, uint8_t data);
    WK_RESULT (*compassSetSampleMode)(struct mpu *mpu, mag_mode_t mode);
    WK_RESULT (*compassSetMeasurementMode)(struct mpu *mpu, lis3mdl_measurement_mode_t mode);
    WK_RESULT (*setMagfullScale)(struct mpu *mpu, lis3mdl_scale_t scale);

    WK_RESULT (*heading)(struct mpu *mpu, raw_axes_t* mag);
    WK_RESULT (*heading_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    WK_RESULT (*motion_mag)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
#endif
    //! \}
    //! \name Miscellaneous
    //! \{
    WK_RESULT (*setFsyncConfig)(struct mpu *mpu, int_lvl_t level);
    WK_RESULT (*setFsyncEnabled)(struct mpu *mpu, bool enable);
    int_lvl_t (*getFsyncConfig)(struct mpu *mpu);
    bool (*getFsyncEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6500
    WK_RESULT (*setFchoice)(struct mpu *mpu, fchoice_t fchoice);
    fchoice_t (*getFchoice)(struct mpu *mpu);
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    WK_RESULT (*setAuxVDDIOLevel)(struct mpu *mpu, auxvddio_lvl_t level);
    auxvddio_lvl_t (*getAuxVDDIOLevel)(struct mpu *mpu);
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    WK_RESULT (*readBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    WK_RESULT (*readBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    WK_RESULT (*readByte)(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
    WK_RESULT (*readBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, uint8_t* data);
    WK_RESULT (*writeBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    WK_RESULT (*writeBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    WK_RESULT (*writeByte)(struct mpu *mpu, uint8_t regAddr, uint8_t data);
    WK_RESULT (*writeBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, const uint8_t* data);
    WK_RESULT (*registerDump)(struct mpu *mpu, uint8_t start, uint8_t end);
    //! \}
    //! \name Sensor readings
    //! \{
    WK_RESULT (*acceleration)(struct mpu *mpu, raw_axes_t* accel);
    WK_RESULT (*acceleration_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    WK_RESULT (*rotation)(struct mpu *mpu, raw_axes_t* gyro);
    WK_RESULT (*rotation_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    WK_RESULT (*temperature)(struct mpu *mpu, int16_t* temp);
    WK_RESULT (*motion)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    WK_RESULT (*sensors)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    WK_RESULT (*sensors_sen)(struct mpu *mpu, sensors_t* sensors, size_t extsens_len);
    //! \}

    WK_RESULT (*accelSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    WK_RESULT (*gyroSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    WK_RESULT (*getBiases)(struct mpu *mpu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);
    WK_RESULT (*setOffsets)(struct mpu *mpu);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    WK_RESULT err;          /*!< Holds last error code */
};

void init_mpu(struct mpu *mpu, mpu_bus_t *bus, mpu_addr_handle_t addr);

extern const accel_fs_t accel_fs;
extern const gyro_fs_t gyro_fs;
extern uint8_t compass_enabled;
extern uint8_t int_enabled;
extern isr_manager mpu_isr_manager;

#endif /* end of include guard: _MPU_9250_H__ */
