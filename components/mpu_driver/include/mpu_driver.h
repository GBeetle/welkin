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
    esp_err_t (*lastError)(struct mpu *mpu);
    //! \}
    //! \name Setup
    //! \{
    esp_err_t (*initialize)(struct mpu *mpu);
    esp_err_t (*reset)(struct mpu *mpu);
    esp_err_t (*setSleep)(struct mpu *mpu, bool enable);
    esp_err_t (*testConnection)(struct mpu *mpu);
    esp_err_t (*selfTest)(struct mpu *mpu, selftest_t* result);
    esp_err_t (*resetSignalPath)(struct mpu *mpu);
    uint8_t (*whoAmI)(struct mpu *mpu);
    bool (*getSleep)(struct mpu *mpu);
    //! \}
    //! \name Main configurations
    //! \{
    esp_err_t (*setSampleRate)(struct mpu *mpu, uint16_t rate);
    esp_err_t (*setClockSource)(struct mpu *mpu, clock_src_t clockSrc);
    esp_err_t (*setDigitalLowPassFilter)(struct mpu *mpu, dlpf_t dlpf);
    uint16_t (*getSampleRate)(struct mpu *mpu);
    clock_src_t (*getClockSource)(struct mpu *mpu);
    dlpf_t (*getDigitalLowPassFilter)(struct mpu *mpu);
    //! \}
    //! \name Power management
    //! \{
    esp_err_t (*setLowPowerAccelMode)(struct mpu *mpu, bool enable);
    esp_err_t (*setLowPowerAccelRate)(struct mpu *mpu, lp_accel_rate_t rate);
    lp_accel_rate_t (*getLowPowerAccelRate)(struct mpu *mpu);
    bool (*getLowPowerAccelMode)(struct mpu *mpu);
    esp_err_t (*setStandbyMode)(struct mpu *mpu, stby_en_t mask);
    stby_en_t (*getStandbyMode)(struct mpu *mpu);
    //! \}
    //! \name Full-Scale Range
    //! \{
    esp_err_t (*setGyroFullScale)(struct mpu *mpu, gyro_fs_t fsr);
    esp_err_t (*setAccelFullScale)(struct mpu *mpu, accel_fs_t fsr);
    gyro_fs_t (*getGyroFullScale)(struct mpu *mpu);
    accel_fs_t (*getAccelFullScale)(struct mpu *mpu);
    //! \}
    //! \name Offset / Bias
    //! \{
    esp_err_t (*setGyroOffset)(struct mpu *mpu, raw_axes_t bias);
    esp_err_t (*setAccelOffset)(struct mpu *mpu, raw_axes_t bias);
    raw_axes_t (*getGyroOffset)(struct mpu *mpu);
    raw_axes_t (*getAccelOffset)(struct mpu *mpu);
    esp_err_t (*computeOffsets)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    esp_err_t (*setInterruptConfig)(struct mpu *mpu, int_config_t config);
    esp_err_t (*setInterruptEnabled)(struct mpu *mpu, int_en_t mask);
    int_stat_t (*getInterruptStatus)(struct mpu *mpu);
    int_config_t (*getInterruptConfig)(struct mpu *mpu);
    int_en_t (*getInterruptEnabled)(struct mpu *mpu);
    //! \}
    //! \name FIFO
    //! \{
    esp_err_t (*setFIFOMode)(struct mpu *mpu, fifo_mode_t mode);
    esp_err_t (*setFIFOConfig)(struct mpu *mpu, fifo_config_t config);
    esp_err_t (*setFIFOEnabled)(struct mpu *mpu, bool enable);
    esp_err_t (*resetFIFO)(struct mpu *mpu);
    uint16_t (*getFIFOCount)(struct mpu *mpu);
    esp_err_t (*readFIFO)(struct mpu *mpu, size_t length, uint8_t* data);
    esp_err_t (*writeFIFO)(struct mpu *mpu, size_t length, const uint8_t* data);
    fifo_mode_t (*getFIFOMode)(struct mpu *mpu);
    fifo_config_t (*getFIFOConfig)(struct mpu *mpu);
    bool (*getFIFOEnabled)(struct mpu *mpu);
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    esp_err_t (*setAuxI2CConfig)(struct mpu *mpu, const auxi2c_config_t *config);
    esp_err_t (*setAuxI2CEnabled)(struct mpu *mpu, bool enable);
    esp_err_t (*setAuxI2CReset)(struct mpu *mpu);
    esp_err_t (*setAuxI2CSlaveConfig)(struct mpu *mpu, const auxi2c_slv_config_t *config);
    esp_err_t (*setAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave, bool enable);
    esp_err_t (*setAuxI2CBypass)(struct mpu *mpu, bool enable);
    esp_err_t (*readAuxI2CRxData)(struct mpu *mpu, size_t length, uint8_t* data, size_t skip);
    esp_err_t (*restartAuxI2C)(struct mpu *mpu);
    auxi2c_stat_t (*getAuxI2CStatus)(struct mpu *mpu);
    auxi2c_config_t (*getAuxI2CConfig)(struct mpu *mpu);
    auxi2c_slv_config_t (*getAuxI2CSlaveConfig)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CEnabled)(struct mpu *mpu);
    bool (*getAuxI2CSlaveEnabled)(struct mpu *mpu, auxi2c_slv_t slave);
    bool (*getAuxI2CBypass)(struct mpu *mpu);
    esp_err_t (*auxI2CWriteByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t data);
    esp_err_t (*auxI2CReadByte)(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    esp_err_t (*setMotionDetectConfig)(struct mpu *mpu, mot_config_t *config);
    mot_config_t (*getMotionDetectConfig)(struct mpu *mpu);
    esp_err_t (*setMotionFeatureEnabled)(struct mpu *mpu, bool enable);
    bool (*getMotionFeatureEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    esp_err_t (*setZeroMotionConfig)(struct mpu *mpu, zrmot_config_t *config);
    zrmot_config_t (*getZeroMotionConfig)(struct mpu *mpu);
    esp_err_t (*setFreeFallConfig)(struct mpu *mpu, ff_config_t *config);
    ff_config_t (*getFreeFallConfig)(struct mpu *mpu);
    mot_stat_t (*getMotionDetectStatus)(struct mpu *mpu);
#endif
    //! \}
    //! \name Compass | Magnetometer
    //! \{
#if defined CONFIG_LIS3MDL
    esp_err_t (*compassInit)(struct mpu *mpu);
    esp_err_t (*compassWhoAmI)(struct mpu *mpu);
    esp_err_t (*compassReset)(struct mpu *mpu);
    esp_err_t (*compassReadByte)(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
    esp_err_t (*compassWriteByte)(struct mpu *mpu, uint8_t regAddr, uint8_t data);
    esp_err_t (*compassSetSampleMode)(struct mpu *mpu, mag_mode_t mode);
    esp_err_t (*compassSetMeasurementMode)(struct mpu *mpu, lis3mdl_measurement_mode_t mode);
    esp_err_t (*setMagfullScale)(struct mpu *mpu, lis3mdl_scale_t scale);

    esp_err_t (*heading)(struct mpu *mpu, raw_axes_t* mag);
    esp_err_t (*heading_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    esp_err_t (*motion_mag)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
#endif
    //! \}
    //! \name Miscellaneous
    //! \{
    esp_err_t (*setFsyncConfig)(struct mpu *mpu, int_lvl_t level);
    esp_err_t (*setFsyncEnabled)(struct mpu *mpu, bool enable);
    int_lvl_t (*getFsyncConfig)(struct mpu *mpu);
    bool (*getFsyncEnabled)(struct mpu *mpu);
#if defined CONFIG_MPU6500
    esp_err_t (*setFchoice)(struct mpu *mpu, fchoice_t fchoice);
    fchoice_t (*getFchoice)(struct mpu *mpu);
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    esp_err_t (*setAuxVDDIOLevel)(struct mpu *mpu, auxvddio_lvl_t level);
    auxvddio_lvl_t (*getAuxVDDIOLevel)(struct mpu *mpu);
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    esp_err_t (*readBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    esp_err_t (*readBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    esp_err_t (*readByte)(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
    esp_err_t (*readBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, uint8_t* data);
    esp_err_t (*writeBit)(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t (*writeBits)(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t (*writeByte)(struct mpu *mpu, uint8_t regAddr, uint8_t data);
    esp_err_t (*writeBytes)(struct mpu *mpu, uint8_t regAddr, size_t length, const uint8_t* data);
    esp_err_t (*registerDump)(struct mpu *mpu, uint8_t start, uint8_t end);
    //! \}
    //! \name Sensor readings
    //! \{
    esp_err_t (*acceleration)(struct mpu *mpu, raw_axes_t* accel);
    esp_err_t (*acceleration_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    esp_err_t (*rotation)(struct mpu *mpu, raw_axes_t* gyro);
    esp_err_t (*rotation_xyz)(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
    esp_err_t (*temperature)(struct mpu *mpu, int16_t* temp);
    esp_err_t (*motion)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
    esp_err_t (*sensors)(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    esp_err_t (*sensors_sen)(struct mpu *mpu, sensors_t* sensors, size_t extsens_len);
    //! \}

    esp_err_t (*accelSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    esp_err_t (*gyroSelfTest)(struct mpu *mpu, raw_axes_t *regularBias, raw_axes_t *selfTestBias, uint8_t* result);
    esp_err_t (*getBiases)(struct mpu *mpu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);
    esp_err_t (*setOffsets)(struct mpu *mpu);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    esp_err_t err;          /*!< Holds last error code */
};

void init_mpu(struct mpu *mpu, mpu_bus_t *bus, mpu_addr_handle_t addr);

extern const accel_fs_t accel_fs;
extern const gyro_fs_t gyro_fs;
extern uint8_t compass_enabled;
extern uint8_t int_enabled;
extern isr_manager mpu_isr_manager;

#endif /* end of include guard: _MPU_9250_H__ */
