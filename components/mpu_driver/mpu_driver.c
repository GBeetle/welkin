// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu.cpp
 * Implement mpu class.
 */

#include "mpu_driver.h"

static esp_err_t initialize(struct mpu *mpu);
static esp_err_t reset(struct mpu *mpu);
static esp_err_t setSleep(struct mpu *mpu, bool enable);
static bool getSleep(struct mpu *mpu);
static esp_err_t testConnection(struct mpu *mpu);
static uint8_t whoAmI(struct mpu *mpu);
static esp_err_t setSampleRate(struct mpu *mpu, uint16_t rate);
static uint16_t getSampleRate(struct mpu *mpu);
static esp_err_t setClockSource(struct mpu *mpu, clock_src_t clockSrc);
static clock_src_t getClockSource(struct mpu *mpu);
static esp_err_t setDigitalLowPassFilter(struct mpu *mpu, dlpf_t dlpf);
static dlpf_t getDigitalLowPassFilter(struct mpu *mpu);
static esp_err_t resetSignalPath(struct mpu *mpu);
static esp_err_t setLowPowerAccelMode(struct mpu *mpu, bool enable);
static bool getLowPowerAccelMode(struct mpu *mpu);
static esp_err_t setLowPowerAccelRate(struct mpu *mpu, lp_accel_rate_t rate);
static lp_accel_rate_t getLowPowerAccelRate(struct mpu *mpu);
static esp_err_t setMotionFeatureEnabled(struct mpu *mpu, bool enable);
static bool getMotionFeatureEnabled(struct mpu *mpu);
static esp_err_t setMotionDetectConfig(struct mpu *mpu, mot_config_t* config);
static mot_config_t getMotionDetectConfig(struct mpu *mpu);
#if defined CONFIG_MPU6050
static esp_err_t setFreeFallConfig(struct mpu *mpu, ff_config_t* config);
static ff_config_t getFreeFallConfig(struct mpu *mpu);
static mot_stat_t getMotionDetectStatus(struct mpu *mpu);
#endif  // MPU6050's stuff
static esp_err_t setStandbyMode(struct mpu *mpu, stby_en_t mask);
static stby_en_t getStandbyMode(struct mpu *mpu);

#if defined CONFIG_MPU6500
static esp_err_t setFchoice(struct mpu *mpu, fchoice_t fchoice);
static fchoice_t getFchoice(struct mpu *mpu);
#endif

static esp_err_t setGyroFullScale(struct mpu *mpu, gyro_fs_t fsr);
static gyro_fs_t getGyroFullScale(struct mpu *mpu);
static esp_err_t setAccelFullScale(struct mpu *mpu, accel_fs_t fsr);
static accel_fs_t getAccelFullScale(struct mpu *mpu);
static esp_err_t setGyroOffset(struct mpu *mpu, raw_axes_t bias);
static raw_axes_t getGyroOffset(struct mpu *mpu);
static esp_err_t setAccelOffset(struct mpu *mpu, raw_axes_t bias);
static raw_axes_t getAccelOffset(struct mpu *mpu);
static esp_err_t computeOffsets(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
static esp_err_t acceleration(struct mpu *mpu, raw_axes_t* accel);
static esp_err_t acceleration_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
static esp_err_t rotation(struct mpu *mpu, raw_axes_t* gyro);
static esp_err_t rotation_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
static esp_err_t temperature(struct mpu *mpu, int16_t* temp);
static esp_err_t motion(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro);
static esp_err_t sensors(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
static esp_err_t sensors_sen(struct mpu *mpu, sensors_t* sensors, size_t extsens_len);

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
static esp_err_t setAuxVDDIOLevel(struct mpu *mpu, auxvddio_lvl_t level);
static auxvddio_lvl_t getAuxVDDIOLevel(struct mpu *mpu);
#endif

static esp_err_t setInterruptConfig(struct mpu *mpu, int_config_t config);
static int_config_t getInterruptConfig(struct mpu *mpu);
static esp_err_t setInterruptEnabled(struct mpu *mpu, int_en_t mask);
static int_en_t getInterruptEnabled(struct mpu *mpu);
static int_stat_t getInterruptStatus(struct mpu *mpu);
static esp_err_t setFIFOMode(struct mpu *mpu, fifo_mode_t mode);
static fifo_mode_t getFIFOMode(struct mpu *mpu);
static esp_err_t setFIFOConfig(struct mpu *mpu, fifo_config_t config);
static fifo_config_t getFIFOConfig(struct mpu *mpu);
static esp_err_t setFIFOEnabled(struct mpu *mpu, bool enable);
static bool getFIFOEnabled(struct mpu *mpu);
static esp_err_t resetFIFO(struct mpu *mpu);
static uint16_t getFIFOCount(struct mpu *mpu);
static esp_err_t readFIFO(struct mpu *mpu, size_t length, uint8_t* data);
static esp_err_t writeFIFO(struct mpu *mpu, size_t length, const uint8_t* data);
static esp_err_t setAuxI2CConfig(struct mpu *mpu, const auxi2c_config_t* config);
static auxi2c_config_t getAuxI2CConfig(struct mpu *mpu);
static esp_err_t setAuxI2CEnabled(struct mpu *mpu, bool enable);
static esp_err_t setAuxI2CReset(struct mpu *mpu);
static bool getAuxI2CEnabled(struct mpu *mpu);
static esp_err_t setAuxI2CSlaveConfig(struct mpu *mpu, const auxi2c_slv_config_t* config);
static auxi2c_slv_config_t getAuxI2CSlaveConfig(struct mpu *mpu, auxi2c_slv_t slave);
static esp_err_t setAuxI2CSlaveEnabled(struct mpu *mpu, auxi2c_slv_t slave, bool enable);
static bool getAuxI2CSlaveEnabled(struct mpu *mpu, auxi2c_slv_t slave);
static esp_err_t setAuxI2CBypass(struct mpu *mpu, bool enable);
static bool getAuxI2CBypass(struct mpu *mpu);
static esp_err_t readAuxI2CRxData(struct mpu *mpu, size_t length, uint8_t* data, size_t skip);
static esp_err_t restartAuxI2C(struct mpu *mpu);
static auxi2c_stat_t getAuxI2CStatus(struct mpu *mpu);
static esp_err_t auxI2CWriteByte(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, const uint8_t data);
static esp_err_t auxI2CReadByte(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t* data);
static esp_err_t setFsyncConfig(struct mpu *mpu, int_lvl_t level);
static int_lvl_t getFsyncConfig(struct mpu *mpu);
static esp_err_t setFsyncEnabled(struct mpu *mpu, bool enable);
static bool getFsyncEnabled(struct mpu *mpu);
static esp_err_t registerDump(struct mpu *mpu, uint8_t start, uint8_t end);

#if defined CONFIG_LIS3MDL
static esp_err_t compassInit(struct mpu *mpu);
static esp_err_t compassWhoAmI(struct mpu *mpu);
static esp_err_t compassReset(struct mpu *mpu);
static esp_err_t compassReadByte(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
static esp_err_t compassWriteByte(struct mpu *mpu, uint8_t regAddr, uint8_t data);
static esp_err_t compassSetMeasurementMode(struct mpu *mpu, lis3mdl_measurement_mode_t mode);
static esp_err_t compassSetSampleMode(struct mpu *mpu, mag_mode_t mode);
static esp_err_t setMagfullScale(struct mpu *mpu, lis3mdl_scale_t scale);

static esp_err_t heading(struct mpu *mpu, raw_axes_t* mag);
static esp_err_t heading_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z);
static esp_err_t motion_mag(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
//static bool compassSelfTest(struct mpu *mpu, raw_axes_t* result);
#endif //CONFIG_LIS3MDL

static esp_err_t selfTest(struct mpu *mpu, selftest_t* result);
static esp_err_t setGyroBias(struct mpu *mpu);
static esp_err_t accelSelfTest(struct mpu *mpu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static esp_err_t gyroSelfTest(struct mpu *mpu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result);
static esp_err_t getBiases(struct mpu *mpu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest);
static esp_err_t setOffsets(struct mpu *mpu);

const accel_fs_t accel_fs = ACCEL_FS_16G;
const gyro_fs_t gyro_fs = GYRO_FS_2000DPS;
uint8_t compass_enabled = 0;
isr_manager mpu_isr_manager = {
    .mpu_isr_status = DATA_NOT_READY,
    .mpu_gyro_data_status = DATA_NOT_READY,
    .mpu_accel_data_status = DATA_NOT_READY,
    .mpu_mag_data_status = DATA_NOT_READY
};


/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
static struct mpu* setBus(struct mpu *mpu, mpu_bus_t *bus);
/**
 * @brief Return communication bus object.
 */
static mpu_bus_t* getBus(struct mpu *mpu);
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
static struct mpu* setAddr(struct mpu *mpu, mpu_addr_handle_t addr);
/**
 * @brief Return I2C address or SPI device handle.
 */
static mpu_addr_handle_t getAddr(struct mpu *mpu);
/*! Return last error code. */
static esp_err_t lastError(struct mpu *mpu);
/*! Read a single bit from a register*/
static esp_err_t readBit(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
/*! Read a range of bits from a register */
static esp_err_t readBits(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
/*! Read a single register */
static esp_err_t readByte(struct mpu *mpu, uint8_t regAddr, uint8_t* data);
/*! Read data from sequence of registers */
static esp_err_t readBytes(struct mpu *mpu, uint8_t regAddr, size_t length, uint8_t* data);
/*! Write a single bit to a register */
static esp_err_t writeBit(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t data);
/*! Write a range of bits to a register */
static esp_err_t writeBits(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
/*! Write a value to a register */
static esp_err_t writeByte(struct mpu *mpu, uint8_t regAddr, uint8_t data);
/*! Write a sequence to data to a sequence of registers */
static esp_err_t writeBytes(struct mpu *mpu, uint8_t regAddr, size_t length, const uint8_t* data);

// ==============
// Inline methods
// ==============

/**
 * @brief Construct a mpu in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
void init_mpu(struct mpu *mpu, mpu_bus_t* bus, mpu_addr_handle_t addr) {
    mpu->bus = bus;
    mpu->addr = addr;
    memset(mpu->buffer, 0xff, 16);
    mpu->err = ESP_OK;

    mpu->initialize = &initialize;
    mpu->reset = &reset;
    mpu->setSleep = &setSleep;
    mpu->getSleep = &getSleep;
    mpu->testConnection = &testConnection;
    mpu->whoAmI = &whoAmI;
    mpu->setSampleRate = &setSampleRate;
    mpu->getSampleRate = &getSampleRate;
    mpu->setClockSource = &setClockSource;
    mpu->getClockSource = &getClockSource;
    mpu->setDigitalLowPassFilter = &setDigitalLowPassFilter;
    mpu->getDigitalLowPassFilter = &getDigitalLowPassFilter;
    mpu->resetSignalPath = &resetSignalPath;
    mpu->setLowPowerAccelMode = &setLowPowerAccelMode;
    mpu->getLowPowerAccelMode = &getLowPowerAccelMode;
    mpu->setLowPowerAccelRate = &setLowPowerAccelRate;
    mpu->getLowPowerAccelRate = &getLowPowerAccelRate;
    mpu->setMotionFeatureEnabled = &setMotionFeatureEnabled;
    mpu->getMotionFeatureEnabled = &getMotionFeatureEnabled;
    mpu->setMotionDetectConfig = &setMotionDetectConfig;
    mpu->getMotionDetectConfig = &getMotionDetectConfig;
#if defined CONFIG_MPU6050
    mpu->setFreeFallConfig = &setFreeFallConfig;
    mpu->getFreeFallConfig = &getFreeFallConfig;
    mpu->getMotionDetectStatus = &getMotionDetectStatus;
#endif  // MPU6050's stuff
    mpu->setStandbyMode = &setStandbyMode;
    mpu->getStandbyMode = &getStandbyMode;

#if defined CONFIG_MPU6500
    mpu->setFchoice = &setFchoice;
    mpu->getFchoice = &getFchoice;
#endif

    mpu->setGyroFullScale = &setGyroFullScale;
    mpu->getGyroFullScale = &getGyroFullScale;
    mpu->setAccelFullScale = &setAccelFullScale;
    mpu->getAccelFullScale = &getAccelFullScale;
    mpu->setGyroOffset = &setGyroOffset;
    mpu->getGyroOffset = &getGyroOffset;
    mpu->setAccelOffset = &setAccelOffset;
    mpu->getAccelOffset = &getAccelOffset;
    mpu->computeOffsets = &computeOffsets;
    mpu->acceleration = &acceleration;
    mpu->acceleration_xyz = &acceleration_xyz;
    mpu->rotation = &rotation;
    mpu->rotation_xyz = &rotation_xyz;
    mpu->temperature = &temperature;
    mpu->motion = &motion;
    mpu->sensors = &sensors;
    mpu->sensors_sen = &sensors_sen;

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    mpu->setAuxVDDIOLevel = &setAuxVDDIOLevel;
    mpu->getAuxVDDIOLevel = &getAuxVDDIOLevel;
#endif
    mpu->setInterruptConfig = &setInterruptConfig;
    mpu->getInterruptConfig = &getInterruptConfig;
    mpu->setInterruptEnabled = &setInterruptEnabled;
    mpu->getInterruptEnabled = &getInterruptEnabled;
    mpu->getInterruptStatus = &getInterruptStatus;
    mpu->setFIFOMode = &setFIFOMode;
    mpu->getFIFOMode = &getFIFOMode;
    mpu->setFIFOConfig = &setFIFOConfig;
    mpu->getFIFOConfig = &getFIFOConfig;
    mpu->setFIFOEnabled = &setFIFOEnabled;
    mpu->getFIFOEnabled = &getFIFOEnabled;
    mpu->resetFIFO = &resetFIFO;
    mpu->getFIFOCount = &getFIFOCount;
    mpu->readFIFO = &readFIFO;
    mpu->writeFIFO = &writeFIFO;
    mpu->setAuxI2CConfig = &setAuxI2CConfig;
    mpu->getAuxI2CConfig = &getAuxI2CConfig;
    mpu->setAuxI2CEnabled = &setAuxI2CEnabled;
    mpu->setAuxI2CReset = &setAuxI2CReset;
    mpu->getAuxI2CEnabled = &getAuxI2CEnabled;
    mpu->setAuxI2CSlaveConfig = &setAuxI2CSlaveConfig;
    mpu->getAuxI2CSlaveConfig = &getAuxI2CSlaveConfig;
    mpu->setAuxI2CSlaveEnabled = &setAuxI2CSlaveEnabled;
    mpu->getAuxI2CSlaveEnabled = &getAuxI2CSlaveEnabled;
    mpu->setAuxI2CBypass = &setAuxI2CBypass;
    mpu->getAuxI2CBypass = &getAuxI2CBypass;
    mpu->readAuxI2CRxData = &readAuxI2CRxData;
    mpu->restartAuxI2C = &restartAuxI2C;
    mpu->getAuxI2CStatus = &getAuxI2CStatus;
    mpu->auxI2CWriteByte = &auxI2CWriteByte;
    mpu->auxI2CReadByte = &auxI2CReadByte;
    mpu->setFsyncConfig = &setFsyncConfig;
    mpu->getFsyncConfig = &getFsyncConfig;
    mpu->setFsyncEnabled = &setFsyncEnabled;
    mpu->getFsyncEnabled = &getFsyncEnabled;
    mpu->registerDump = &registerDump;

#if defined CONFIG_LIS3MDL
    mpu->compassInit = &compassInit;
    mpu->compassSetSampleMode = &compassSetSampleMode;
    mpu->compassWhoAmI = &compassWhoAmI;
    mpu->compassReset = &compassReset;
    mpu->compassReadByte = &compassReadByte;
    mpu->compassWriteByte = &compassWriteByte;
    mpu->compassSetMeasurementMode = &compassSetMeasurementMode;
    mpu->setMagfullScale = &setMagfullScale;

    mpu->heading = &heading;
    mpu->heading_xyz = &heading_xyz;
    mpu->motion_mag = &motion_mag;
#endif

    mpu->selfTest = &selfTest;
    mpu->setGyroBias = &setGyroBias;
    mpu->accelSelfTest = &accelSelfTest;
    mpu->gyroSelfTest = &gyroSelfTest;
    mpu->getBiases = &getBiases;
    mpu->setOffsets = &setOffsets;

    mpu->setBus = &setBus;
    mpu->getBus = &getBus;
    mpu->setAddr = &setAddr;
    mpu->getAddr = &getAddr;
    mpu->lastError = &lastError;
    mpu->readBit = &readBit;
    mpu->readBits = &readBits;
    mpu->readByte = &readByte;
    mpu->readBytes = &readBytes;
    mpu->writeBit = &writeBit;
    mpu->writeBits = &writeBits;
    mpu->writeByte = &writeByte;
    mpu->writeBytes = &writeBytes;
}

/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
static struct mpu* setBus(struct mpu *mpu, mpu_bus_t *bus)
{
    mpu->bus = bus;
    return mpu;
}
/**
 * @brief Return communication bus object.
 */
static mpu_bus_t* getBus(struct mpu *mpu)
{
    return mpu->bus;
}
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
static struct mpu* setAddr(struct mpu *mpu, mpu_addr_handle_t addr)
{
    mpu->addr = addr;
    return mpu;
}
/**
 * @brief Return I2C address or SPI device handle.
 */
static mpu_addr_handle_t getAddr(struct mpu *mpu)
{
    return mpu->addr;
}
/*! Return last error code. */
static esp_err_t lastError(struct mpu *mpu)
{
    return mpu->err;
}
/*! Read a single bit from a register*/
static esp_err_t readBit(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    return mpu->bus->readBit(mpu->bus, mpu->addr, regAddr, bitNum, data);
}
/*! Read a range of bits from a register */
static esp_err_t readBits(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    return mpu->bus->readBits(mpu->bus, mpu->addr, regAddr, bitStart, length, data);
}
/*! Read a single register */
static esp_err_t readByte(struct mpu *mpu, uint8_t regAddr, uint8_t* data)
{
    return mpu->bus->readByte(mpu->bus, mpu->addr, regAddr, data);
}
/*! Read data from sequence of registers */
static esp_err_t readBytes(struct mpu *mpu, uint8_t regAddr, size_t length, uint8_t* data)
{
    return mpu->bus->readBytes(mpu->bus, mpu->addr, regAddr, length, data);
}
/*! Write a single bit to a register */
static esp_err_t writeBit(struct mpu *mpu, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    return mpu->bus->writeBit(mpu->bus, mpu->addr, regAddr, bitNum, data);
}
/*! Write a range of bits to a register */
static esp_err_t writeBits(struct mpu *mpu, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    return mpu->bus->writeBits(mpu->bus, mpu->addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
static esp_err_t writeByte(struct mpu *mpu, uint8_t regAddr, uint8_t data)
{
    return mpu->bus->writeByte(mpu->bus, mpu->addr, regAddr, data);
}
/*! Write a sequence to data to a sequence of registers */
static esp_err_t writeBytes(struct mpu *mpu, uint8_t regAddr, size_t length, const uint8_t* data)
{
    return mpu->bus->writeBytes(mpu->bus, mpu->addr, regAddr, length, data);
}

/**
 * @brief Initialize mpu device and set basic configurations.
 * @details
 *  Init configuration:
 *  - Accel FSR: 4G
 *  - Gyro FSR: 500DPS
 *  - Sample rate: 100Hz
 *  - DLPF: 42Hz
 *  - INT pin: disabled
 *  - FIFO: disabled
 *  - Clock source: gyro PLL \n
 *  For MPU9150 and MPU9250:
 *  - Aux I2C Master: enabled, clock: 400KHz
 *  - Compass: enabled on Aux I2C's Slave 0 and Slave 1
 *
 * @note
 *  - A soft reset is performed first, which takes 100-200ms.
 *  - When using SPI, the primary I2C Slave module is disabled right away.
 * */
static esp_err_t initialize(struct mpu *mpu)
{
    // reset device (wait a little to clear all registers)
    if (MPU_ERR_CHECK(mpu->reset(mpu))) return mpu->err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // wake-up the device (power on-reset state is asleep for some models)
    if (MPU_ERR_CHECK(mpu->setSleep(mpu, false))) return mpu->err;
    // disable mpu's I2C slave module when using SPI
#ifdef CONFIG_MPU_SPI
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_I2C_IF_DIS_BIT, 1))) return mpu->err;
#endif
    // set clock source to gyro PLL which is better than internal clock
    if (MPU_ERR_CHECK(mpu->setClockSource(mpu, CLOCK_PLL))) return mpu->err;

#if defined CONFIG_MPU6500
    // MPU6500 / MPU9250 share 4kB of memory between the DMP and the FIFO. Since the
    // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, ACCEL_CONFIG2, ACONFIG2_FIFO_SIZE_BIT, ACONFIG2_FIFO_SIZE_LENGTH,
                                FIFO_SIZE_1K))) {
        return mpu->err;
    }
#endif

    // set Full Scale range
    if (MPU_ERR_CHECK(mpu->setGyroFullScale(mpu, gyro_fs))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setAccelFullScale(mpu, accel_fs))) return mpu->err;

    vTaskDelay(50 / portTICK_PERIOD_MS);

#if defined CONFIG_LIS3MDL
    compass_enabled = 1;
    if (MPU_ERR_CHECK(mpu->compassInit(mpu))) return mpu->err;
    imuInit();
#endif

    // set gyro to 32kHz fchoice 0 | 1 & set accel to 4KHz fchoice 0
    //if (mpu->setFchoice(mpu, FCHOICE_0)) return mpu->err;

    // set gyro to 8kHz & set accel to 4kHz
    //if (mpu->setFchoice(mpu, FCHOICE_3)) return mpu->err;
    //if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, DLPF_3600HZ_NOLPF))) return mpu->err;

    // set Digital Low Pass Filter to get smoother data
    if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, DLPF_42HZ))) return mpu->err;
    // set sample rate to 1000Hz  from 4Hz - 1kHz
    if (MPU_ERR_CHECK(mpu->setSampleRate(mpu, 250))) return mpu->err;

    int_config_t config = {
        .level = INT_LVL_ACTIVE_HIGH,
        .drive = INT_DRV_PUSHPULL,
        .mode = INT_MODE_PULSE50US,
        .clear = INT_CLEAR_STATUS_REG
    };
    if (mpu->setInterruptConfig(mpu, config)) return mpu->err;

    int_en_t mask = INT_EN_RAWDATA_READY;
    if (mpu->setInterruptEnabled(mpu, mask)) return mpu->err;

    //MPU_LOGI("Initialization complete");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return mpu->err;
}

/**
 * @brief Reset internal registers and restore to default start-up state.
 * @note
 *  - This function delays 100ms when using I2C and 200ms when using SPI.
 *  - It does not initialize the mpu again, just call initialize() instead.
 * */
static esp_err_t reset(struct mpu *mpu)
{
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, PWR_MGMT1, PWR1_DEVICE_RESET_BIT, 1))) return mpu->err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
#ifdef CONFIG_MPU_SPI
    if (MPU_ERR_CHECK(mpu->resetSignalPath(mpu))) {
        return mpu->err;
    }
#endif
    //MPU_LOGI("Reset!");
    return mpu->err;
}

/**
 * @brief Enable / disable sleep mode
 * @param enable enable value
 * */
static esp_err_t setSleep(struct mpu *mpu, bool enable)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, PWR_MGMT1, PWR1_SLEEP_BIT, (uint8_t) enable));
}

/**
 * @brief Get current sleep state.
 * @return
 *  - `true`: sleep enabled.
 *  - `false`: sleep disabled.
 */
bool getSleep(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, PWR_MGMT1, PWR1_SLEEP_BIT, mpu->buffer));
    return mpu->buffer[0];
}

/**
 * @brief Test connection with mpu.
 * @details It reads the WHO_AM_IM register and check its value against the correct chip model.
 * @return
 *  - `ESP_OK`: The mpu is connected and matchs the model.
 *  - `ESP_ERR_NOT_FOUND`: A device is connect, but does not match the chip selected in _menuconfig_.
 *  - May return other communication bus errors. e.g: `ESP_FAIL`, `ESP_ERR_TIMEOUT`.
 * */
static esp_err_t testConnection(struct mpu *mpu)
{
    const uint8_t wai = mpu->whoAmI(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    return (wai == 0x68) ? ESP_OK : ESP_ERR_NOT_FOUND;
#elif defined CONFIG_MPU9255
    return (wai == 0x73) ? ESP_OK : ESP_ERR_NOT_FOUND;
#elif defined CONFIG_MPU9250
    return (wai == 0x71) ? ESP_OK : ESP_ERR_NOT_FOUND;
#elif defined CONFIG_MPU6555
    return (wai == 0x7C) ? ESP_OK : ESP_ERR_NOT_FOUND;
#elif defined CONFIG_MPU6500
    return (wai == 0x70) ? ESP_OK : ESP_ERR_NOT_FOUND;
#endif
}

/**
 * @brief Returns the value from WHO_AM_I register.
 */
uint8_t whoAmI(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, WHO_AM_I, mpu->buffer));
    return mpu->buffer[0];
}

/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * This is the update rate of sensor register. \n
 * Formula: Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 *
 * @param rate 4Hz ~ 1KHz
 *  - For sample rate 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF.
 *  - For sample rate 32KHZ [MPU6500 / MPU9250]: set fchoice to FCHOICE_0, see setFchoice().
 *
 * @note
 *  For MPU9150 & MPU9250:
 *   - When using compass, this function alters Aux I2C Master `sample_delay` property
 *     to adjust the compass sample rate. (also, `wait_for_es` property to adjust interrupt).
 *   - If sample rate lesser than 100 Hz, data-ready interrupt will wait for compass data.
 *   - If sample rate greater than 100 Hz, data-ready interrupt will not be delayed by the compass.
 * */
static esp_err_t setSampleRate(struct mpu *mpu, uint16_t rate)
{
    // Check value range
    if (rate < 4) {
        WK_DEBUGE(ERROR_TAG, "INVALID_SAMPLE_RATE %d, minimum rate is 4", rate);
        rate = 4;
    }
    else if (rate > 1000) {
        WK_DEBUGE(ERROR_TAG, "INVALID_SAMPLE_RATE %d, maximum rate is 1000", rate);
        rate = 1000;
    }

#if defined CONFIG_MPU6500
    fchoice_t fchoice = mpu->getFchoice(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (fchoice != FCHOICE_3) {
        WK_DEBUGE(ERROR_TAG, "INVALID_STATE, sample rate divider is not effective when Fchoice != 3");
    }
#endif
    // Check dlpf configuration
    dlpf_t dlpf = mpu->getDigitalLowPassFilter(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (dlpf == 0 || dlpf == 7)
        WK_DEBUGE(ERROR_TAG, "INVALID_STATE, sample rate divider is not effective when DLPF is (0 or 7)");

    const uint16_t internalSampleRate = 1000;
    uint16_t divider                      = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
    }
    else {
    }
    // Write divider to register
    if (MPU_ERR_CHECK(mpu->writeByte(mpu, SMPLRT_DIV, (uint8_t) divider))) return mpu->err;

    return mpu->err;
}

/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 */
uint16_t getSampleRate(struct mpu *mpu)
{
#if defined CONFIG_MPU6500
    fchoice_t fchoice = mpu->getFchoice(mpu);
    MPU_ERR_CHECK(mpu->lastError(mpu));
    if (fchoice != FCHOICE_3) return SAMPLE_RATE_MAX;
#endif

    const uint16_t sampleRateMax_nolpf = 8000;
    dlpf_t dlpf                            = mpu->getDigitalLowPassFilter(mpu);
    MPU_ERR_CHECK(mpu->lastError(mpu));
    if (dlpf == 0 || dlpf == 7) return sampleRateMax_nolpf;

    const uint16_t internalSampleRate = 1000;
    MPU_ERR_CHECK(mpu->readByte(mpu, SMPLRT_DIV, mpu->buffer));
    uint16_t rate = internalSampleRate / (1 + mpu->buffer[0]);
    return rate;
}

/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
static esp_err_t setClockSource(struct mpu *mpu, clock_src_t clockSrc)
{
    return MPU_ERR_CHECK(mpu->writeBits(mpu, PWR_MGMT1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, clockSrc));
}

/**
 * @brief Return clock source.
 */
clock_src_t getClockSource(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBits(mpu, PWR_MGMT1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, mpu->buffer));
    return (clock_src_t) mpu->buffer[0];
}

/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
static esp_err_t setDigitalLowPassFilter(struct mpu *mpu, dlpf_t dlpf)
{
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, CONFIG, CONFIG_DLPF_CFG_BIT, CONFIG_DLPF_CFG_LENGTH, dlpf))) {
        return mpu->err;
    }
#if defined CONFIG_MPU6500
    MPU_ERR_CHECK(
        mpu->writeBits(mpu, ACCEL_CONFIG2, ACONFIG2_A_DLPF_CFG_BIT, ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
#endif
    return mpu->err;
}

/**
 * @brief Return Digital Low Pass Filter configuration
 */
dlpf_t getDigitalLowPassFilter(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBits(mpu, CONFIG, CONFIG_DLPF_CFG_BIT, CONFIG_DLPF_CFG_LENGTH, mpu->buffer));
    return (dlpf_t) mpu->buffer[0];
}

/**
 * @brief Reset sensors signal path.
 *
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 *
 * @note This function delays 100 ms, needed for reset to complete.
 * */
static esp_err_t resetSignalPath(struct mpu *mpu)
{
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_SIG_COND_RESET_BIT, 1))) return mpu->err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return mpu->err;
}

/**
 * @brief Enter Low Power Accelerometer mode.
 *
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency.
 * See setLowPowerAccelRate() to set the frequency.
 *
 * @param enable value
 *  + This function does the following to enable:
 *   - Set CYCLE bit to 1
 *   - Set SLEEP bit to 0
 *   - Set TEMP_DIS bit to 1
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *   - Disable Auxiliary I2C Master I/F
 *
 *  + This function does the following to disable:
 *   - Set CYCLE bit to 0
 *   - Set TEMP_DIS bit to 0
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *   - Enable Auxiliary I2C Master I/F
 * */
static esp_err_t setLowPowerAccelMode(struct mpu *mpu, bool enable)
{
// set FCHOICE
#if defined CONFIG_MPU6500
    fchoice_t fchoice = enable ? FCHOICE_0 : FCHOICE_3;
    if (MPU_ERR_CHECK(mpu->setFchoice(mpu, fchoice))) return mpu->err;
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, PWR_MGMT1, 2, mpu->buffer))) return mpu->err;
    if (enable) {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        mpu->buffer[0] |= 1 << PWR1_CYCLE_BIT;
        mpu->buffer[0] &= ~(1 << PWR1_SLEEP_BIT);
        mpu->buffer[0] |= 1 << PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        mpu->buffer[1] |= PWR2_STBY_XYZG_BITS;
    }
    else {  // disable
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        mpu->buffer[0] &= ~(1 << PWR1_CYCLE_BIT);
        mpu->buffer[0] &= ~(1 << PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        mpu->buffer[1] &= ~(PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    mpu->buffer[1] &= ~(PWR2_STBY_XYZA_BITS);
    // write back PWR_MGMT1 and PWR_MGMT2 at once
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, PWR_MGMT1, 2, mpu->buffer))) return mpu->err;
    // disable Auxiliary I2C Master I/F in case it was active
    if (MPU_ERR_CHECK(mpu->setAuxI2CEnabled(mpu, !enable))) return mpu->err;
    return mpu->err;
}

/**
 * @brief Return Low Power Accelerometer state.
 *
 * Condition to return true:
 *  - CYCLE bit is 1
 *  - SLEEP bit is 0
 *  - TEMP_DIS bit is 1
 *  - STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  - STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  - FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 *
 * */
bool getLowPowerAccelMode(struct mpu *mpu)
{
// check FCHOICE
#if defined CONFIG_MPU6500
    fchoice_t fchoice = mpu->getFchoice(mpu);
    MPU_ERR_CHECK(mpu->lastError(mpu));
    if (fchoice != FCHOICE_0) {
        return false;
    }
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    MPU_ERR_CHECK(mpu->readBytes(mpu, PWR_MGMT1, 2, mpu->buffer));
    // define configuration bits
    const uint8_t LPACCEL_CONFIG_BITMASK[2] = {
        (1 << PWR1_SLEEP_BIT) | (1 << PWR1_CYCLE_BIT) | (1 << PWR1_TEMP_DIS_BIT),
        PWR2_STBY_XYZA_BITS | PWR2_STBY_XYZG_BITS};
    const uint8_t LPACCEL_ENABLED_VALUE[2] = {(1 << PWR1_CYCLE_BIT) | (1 << PWR1_TEMP_DIS_BIT),
                                                  PWR2_STBY_XYZG_BITS};
    // get just the configuration bits
    mpu->buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    mpu->buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    return mpu->buffer[0] == LPACCEL_ENABLED_VALUE[0] && mpu->buffer[1] == LPACCEL_ENABLED_VALUE[1];
}

/**
 * @brief Set Low Power Accelerometer frequency of wake-up.
 * */
static esp_err_t setLowPowerAccelRate(struct mpu *mpu, lp_accel_rate_t rate)
{
#if defined CONFIG_MPU6050
    return MPU_ERR_CHECK(mpu->writeBits(mpu, PWR_MGMT2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH, rate));
#elif defined CONFIG_MPU6500
    return MPU_ERR_CHECK(mpu->writeBits(mpu, LP_ACCEL_ODR, LPA_ODR_CLKSEL_BIT, LPA_ODR_CLKSEL_LENGTH, rate));
#endif
}

/**
 * @brief Get Low Power Accelerometer frequency of wake-up.
 */
lp_accel_rate_t getLowPowerAccelRate(struct mpu *mpu)
{
#if defined CONFIG_MPU6050
    MPU_ERR_CHECK(mpu->readBits(mpu, PWR_MGMT2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH, mpu->buffer));
#elif defined CONFIG_MPU6500
    MPU_ERR_CHECK(mpu->readBits(mpu, LP_ACCEL_ODR, LPA_ODR_CLKSEL_BIT, LPA_ODR_CLKSEL_LENGTH, mpu->buffer));
#endif
    return (lp_accel_rate_t) mpu->buffer[0];
}

/**
 * @brief Enable/disable Motion modules (Motion detect, Zero-motion, Free-Fall).
 *
 * @attention
 *  The configurations must've already been set with setMotionDetectConfig() before
 *  enabling the module!
 * @note
 *  - Call getMotionDetectStatus() to find out which axis generated motion interrupt. [MPU6000, MPU6050, MPU9150].
 *  - It is recommended to set the Motion Interrupt to propagate to the INT pin. To do that, use setInterruptEnabled().
 * @param enable
 *  - On _true_, this function modifies the DLPF, put gyro and temperature in standby,
 *    and disable Auxiliary I2C Master I/F.
 *  - On _false_, this function sets DLPF to 42Hz and enables Auxiliary I2C master I/F.
 * */
static esp_err_t setMotionFeatureEnabled(struct mpu *mpu, bool enable)
{
#if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(
            mpu->writeBits(mpu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, ACCEL_DHPF_RESET))) {
        return mpu->err;
    }
#endif
    /* enabling */
    if (enable) {
#if defined CONFIG_MPU6050
        const dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
        const dlpf_t kDLPF = DLPF_188HZ;
#endif
        if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, kDLPF))) return mpu->err;
#if defined CONFIG_MPU6050
        // give a time for accumulation of samples
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (MPU_ERR_CHECK(
                mpu->writeBits(mpu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, ACCEL_DHPF_HOLD))) {
            return mpu->err;
        }
#elif defined CONFIG_MPU6500
        if (MPU_ERR_CHECK(
                mpu->writeByte(mpu, ACCEL_INTEL_CTRL, (1 << ACCEL_INTEL_EN_BIT) | (1 << ACCEL_INTEL_MODE_BIT))))
            return mpu->err;
#endif
        /* disabling */
    }
    else {
#if defined CONFIG_MPU6500
        if (MPU_ERR_CHECK(mpu->writeBits(mpu, ACCEL_INTEL_CTRL, ACCEL_INTEL_EN_BIT, 2, 0x0))) {
            return mpu->err;
        }
#endif
        const dlpf_t kDLPF = DLPF_42HZ;
        if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, kDLPF))) return mpu->err;
    }
    // disable Auxiliary I2C Master I/F in case it was active
    if (MPU_ERR_CHECK(mpu->setAuxI2CEnabled(mpu, !enable))) return mpu->err;
    return mpu->err;
}

/**
 * @brief Return true if a Motion Dectection module is enabled.
 */
bool getMotionFeatureEnabled(struct mpu *mpu)
{
    uint8_t data;
#if defined CONFIG_MPU6050
    MPU_ERR_CHECK(mpu->readBits(mpu, ACCEL_CONFIG, ACONFIG_HPF_BIT, ACONFIG_HPF_LENGTH, &data));
    if (data != ACCEL_DHPF_HOLD) return false;
    const dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
    MPU_ERR_CHECK(mpu->readByte(mpu, ACCEL_INTEL_CTRL, &data));
    const uint8_t kAccelIntel = (1 << ACCEL_INTEL_EN_BIT) | (1 << ACCEL_INTEL_MODE_BIT);
    if ((data & kAccelIntel) != kAccelIntel) return false;
    const dlpf_t kDLPF = DLPF_188HZ;
#endif
    dlpf_t dlpf = mpu->getDigitalLowPassFilter(mpu);
    MPU_ERR_CHECK(mpu->lastError(mpu));
    if (dlpf != kDLPF) return false;
    return true;
}

/**
 * @brief Configure Motion-Detect or Wake-on-motion feature.
 *
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 *
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high
 * passed sample from any axis has an absolute value exceeding a user-programmable threshold. A
 * counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the
 * MOT_DETECT_STATUS register.
 *
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 *
 * @note
 * It is possible to enable **wake-on-motion** mode by doing the following:
 *  1. Enter Low Power Accelerometer mode with setLowPowerAccelMode();
 *  2. Select the wake-up rate with setLowPowerAccelRate();
 *  3. Configure motion-detect interrupt with setMotionDetectConfig();
 *  4. Enable the motion detection module with setMotionFeatureEnabled();
 * */
static esp_err_t setMotionDetectConfig(struct mpu *mpu, mot_config_t* config)
{
#if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(mpu->writeByte(mpu, MOTION_DUR, config.time))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, MOTION_DETECT_CTRL, MOTCTRL_ACCEL_ON_DELAY_BIT,
                                MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay))) {
        return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, MOTION_DETECT_CTRL, MOTCTRL_MOT_COUNT_BIT, MOTCTRL_MOT_COUNT_LENGTH,
                                config.counter))) {
        return mpu->err;
    }
#endif
    return MPU_ERR_CHECK(mpu->writeByte(mpu, MOTION_THR, config->threshold));
}

/**
 * @brief Return Motion Detection Configuration.
 */
mot_config_t getMotionDetectConfig(struct mpu *mpu)
{
    mot_config_t config;
#if defined CONFIG_MPU6050
    MPU_ERR_CHECK(mpu->readByte(mpu, MOTION_DUR, &config.time));
    MPU_ERR_CHECK(mpu->readByte(mpu, MOTION_DETECT_CTRL, mpu->buffer));
    config.accel_on_delay =
        (mpu->buffer[0] >> (MOTCTRL_ACCEL_ON_DELAY_BIT - MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((mpu->buffer[0] >> (MOTCTRL_MOT_COUNT_BIT - MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
#endif
    MPU_ERR_CHECK(mpu->readByte(mpu, MOTION_THR, &config.threshold));
    return config;
}

#if defined CONFIG_MPU6050
/**
 * @brief Configure Zero-Motion.
 *
 * The Zero Motion detection capability uses the digital high pass filter (DHPF) and a similar
 * threshold scheme to that of Free Fall detection. Each axis of the high passed accelerometer
 * measurement must have an absolute value less than a threshold specified in the ZRMOT_THR
 * register, which can be increased in 1 mg increments. Each time a motion sample meets this
 * condition, a counter increments. When this counter reaches a threshold specified in ZRMOT_DUR, an
 * interrupt is generated.
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an interrupt both when Zero
 * Motion is first detected and when Zero Motion is no longer detected. While Free Fall and Motion
 * are indicated with a flag which clears after being read, reading the state of the Zero Motion
 * detected from the MOT_DETECT_STATUS register does not clear its status.
 *
 * @note Enable by calling setMotionFeatureEnabled();
 * */
static esp_err_t setZeroMotionConfig(struct mpu *mpu, zrmot_config_t* config)
{
    mpu->buffer[0] = config.threshold;
    mpu->buffer[1] = config.time;
    return MPU_ERR_CHECK(mpu->writeBytes(mpu, ZRMOTION_THR, 2, mpu->buffer));
}

/**
 * @brief Return Zero-Motion configuration.
 */
zrmot_config_t getZeroMotionConfig(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBytes(mpu, ZRMOTION_THR, 2, mpu->buffer));
    zrmot_config_t config{};
    config.threshold = mpu->buffer[0];
    config.time      = mpu->buffer[1];
    return config;
}

/**
 * @brief Configure Free-Fall.
 *
 * Free fall is detected by checking if the accelerometer measurements from all 3 axes have an
 * absolute value below a user-programmable threshold (acceleration threshold). For each sample
 * where this condition is true (a qualifying sample), a counter is incremented. For each sample
 * where this condition is false (a non- qualifying sample), the counter is decremented. Once the
 * counter reaches a user-programmable threshold (the counter threshold), the Free Fall interrupt is
 * triggered and a flag is set. The flag is cleared once the counter has decremented to zero. The
 * counter does not increment above the counter threshold or decrement below zero.
 *
 * @note Enable by calling setMotionFeatureEnabled().
 * */
static esp_err_t setFreeFallConfig(struct mpu *mpu, ff_config_t* config)
{
    mpu->buffer[0] = config.threshold;
    mpu->buffer[1] = config.time;
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, FF_THR, 2, mpu->buffer))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, MOTION_DETECT_CTRL, MOTCTRL_ACCEL_ON_DELAY_BIT,
                                MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay))) {
        return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, MOTION_DETECT_CTRL, MOTCTRL_MOT_COUNT_BIT, MOTCTRL_MOT_COUNT_LENGTH,
                                config.counter))) {
        return mpu->err;
    }
    return mpu->err;
}

/**
 * @brief Return Free-Fall Configuration.
 */
ff_config_t getFreeFallConfig(struct mpu *mpu)
{
    ff_config_t config{};
    MPU_ERR_CHECK(mpu->readBytes(mpu, FF_THR, 2, mpu->buffer));
    config.threshold = mpu->buffer[0];
    config.time      = mpu->buffer[1];
    MPU_ERR_CHECK(mpu->readByte(mpu, MOTION_DETECT_CTRL, mpu->buffer));
    config.accel_on_delay =
        (mpu->buffer[0] >> (MOTCTRL_ACCEL_ON_DELAY_BIT - MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((mpu->buffer[0] >> (MOTCTRL_MOT_COUNT_BIT - MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
    return config;
}

/**
 * @brief Return Motion Detection Status.
 * @note Reading this register clears all motion detection status bits.
 * */
mot_stat_t getMotionDetectStatus(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, MOTION_DETECT_STATUS, mpu->buffer));
    return (mot_stat_t) mpu->buffer[0];
}
#endif  // MPU6050's stuff

/**
 * @brief Configure sensors' standby mode.
 * */
static esp_err_t setStandbyMode(struct mpu *mpu, stby_en_t mask)
{
    const uint8_t kPwr1StbyBits = mask >> 6;
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, PWR_MGMT1, PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits))) {
        return mpu->err;
    }
    return MPU_ERR_CHECK(mpu->writeBits(mpu, PWR_MGMT2, PWR2_STBY_XA_BIT, 6, mask));
}

/**
 * @brief Return Standby configuration.
 * */
stby_en_t getStandbyMode(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBytes(mpu, PWR_MGMT1, 2, mpu->buffer));
    const uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
    stby_en_t mask                            = mpu->buffer[0] << 3 & kStbyTempAndGyroPLLBits;
    const uint8_t kStbyAccelAndGyroBits   = STBY_EN_ACCEL | STBY_EN_GYRO;
    mask |= mpu->buffer[1] & kStbyAccelAndGyroBits;
    return mask;
}

#if defined CONFIG_MPU6500
/**
 * @brief Select FCHOICE.
 *
 * Dev note: FCHOICE is the inverted value of FCHOICE_B (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).
 * Reset value is FCHOICE_3
 * */
static esp_err_t setFchoice(struct mpu *mpu, fchoice_t fchoice)
{
    mpu->buffer[0] = (~(fchoice) & 0x3);  // invert to fchoice_b
    if (MPU_ERR_CHECK(
            mpu->writeBits(mpu, GYRO_CONFIG, GCONFIG_FCHOICE_B, GCONFIG_FCHOICE_B_LENGTH, mpu->buffer[0]))) {
        return mpu->err;
    }
    return MPU_ERR_CHECK(mpu->writeBit(mpu, ACCEL_CONFIG2, ACONFIG2_ACCEL_FCHOICE_B_BIT, (mpu->buffer[0] == 0) ? 0 : 1));
}

/**
 * @brief Return FCHOICE.
 */
fchoice_t getFchoice(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBits(mpu, GYRO_CONFIG, GCONFIG_FCHOICE_B, GCONFIG_FCHOICE_B_LENGTH, mpu->buffer));
    return (fchoice_t)(~(mpu->buffer[0]) & 0x3);
}
#endif

/**
 * @brief Select Gyroscope Full-scale range.
 * */
static esp_err_t setGyroFullScale(struct mpu *mpu, gyro_fs_t fsr)
{
    return MPU_ERR_CHECK(mpu->writeBits(mpu, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, fsr));
}

/**
 * @brief Return Gyroscope Full-scale range.
 */
gyro_fs_t getGyroFullScale(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBits(mpu, GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, mpu->buffer));
    return (gyro_fs_t) mpu->buffer[0];
}

/**
 * @brief Select Accelerometer Full-scale range.
 * */
static esp_err_t setAccelFullScale(struct mpu *mpu, accel_fs_t fsr)
{
    return MPU_ERR_CHECK(mpu->writeBits(mpu, ACCEL_CONFIG, ACONFIG_FS_SEL_BIT, ACONFIG_FS_SEL_LENGTH, fsr));
}

/**
 * @brief Return Accelerometer Full-scale range.
 */
accel_fs_t getAccelFullScale(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBits(mpu, ACCEL_CONFIG, ACONFIG_FS_SEL_BIT, ACONFIG_FS_SEL_LENGTH, mpu->buffer));
    return (accel_fs_t) mpu->buffer[0];
}

/**
 * @brief Push biases to the gyro offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-1000dps format.
 * */
static esp_err_t setGyroOffset(struct mpu *mpu, raw_axes_t bias)
{
    mpu->buffer[0] = (uint8_t)(bias.data.x >> 8);
    mpu->buffer[1] = (uint8_t)(bias.data.x);
    mpu->buffer[2] = (uint8_t)(bias.data.y >> 8);
    mpu->buffer[3] = (uint8_t)(bias.data.y);
    mpu->buffer[4] = (uint8_t)(bias.data.z >> 8);
    mpu->buffer[5] = (uint8_t)(bias.data.z);
    return MPU_ERR_CHECK(mpu->writeBytes(mpu, XG_OFFSET_H, 6, mpu->buffer));
}

/**
 * @brief Return biases from the gyro offset registers.
 *
 * Note: Bias output are LSB in +-1000dps format.
 * */
raw_axes_t getGyroOffset(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBytes(mpu, XG_OFFSET_H, 6, mpu->buffer));
    raw_axes_t bias;
    bias.data.x = (mpu->buffer[0] << 8) | mpu->buffer[1];
    bias.data.y = (mpu->buffer[2] << 8) | mpu->buffer[3];
    bias.data.z = (mpu->buffer[4] << 8) | mpu->buffer[5];
    return bias;
}

/**
 * @brief Push biases to the accel offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-16G format.
 * */
static esp_err_t setAccelOffset(struct mpu *mpu, raw_axes_t bias)
{
    raw_axes_t facBias;
    // first, read OTP values of Accel factory trim

#if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, XA_OFFSET_H, 6, mpu->buffer))) return mpu->err;
    facBias.x = (mpu->buffer[0] << 8) | mpu->buffer[1];
    facBias.y = (mpu->buffer[2] << 8) | mpu->buffer[3];
    facBias.z = (mpu->buffer[4] << 8) | mpu->buffer[5];

#elif defined CONFIG_MPU6500
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, XA_OFFSET_H, 8, mpu->buffer))) return mpu->err;
    // note: mpu->buffer[2] and mpu->buffer[5], stay the same,
    //  they are read just to keep the burst reading
    facBias.data.x = (mpu->buffer[0] << 8) | mpu->buffer[1];
    facBias.data.y = (mpu->buffer[3] << 8) | mpu->buffer[4];
    facBias.data.z = (mpu->buffer[6] << 8) | mpu->buffer[7];
#endif

    // note: preserve bit 0 of factory value (for temperature compensation)
    facBias.data.x += (bias.data.x & ~1);
    facBias.data.y += (bias.data.y & ~1);
    facBias.data.z += (bias.data.z & ~1);

#if defined CONFIG_MPU6050
    mpu->buffer[0] = (uint8_t)(facBias.x >> 8);
    mpu->buffer[1] = (uint8_t)(facBias.x);
    mpu->buffer[2] = (uint8_t)(facBias.y >> 8);
    mpu->buffer[3] = (uint8_t)(facBias.y);
    mpu->buffer[4] = (uint8_t)(facBias.z >> 8);
    mpu->buffer[5] = (uint8_t)(facBias.z);
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, XA_OFFSET_H, 6, mpu->buffer))) return mpu->err;

#elif defined CONFIG_MPU6500
    mpu->buffer[0] = (uint8_t)(facBias.data.x >> 8);
    mpu->buffer[1] = (uint8_t)(facBias.data.x);
    mpu->buffer[3] = (uint8_t)(facBias.data.y >> 8);
    mpu->buffer[4] = (uint8_t)(facBias.data.y);
    mpu->buffer[6] = (uint8_t)(facBias.data.z >> 8);
    mpu->buffer[7] = (uint8_t)(facBias.data.z);
    return MPU_ERR_CHECK(mpu->writeBytes(mpu, XA_OFFSET_H, 8, mpu->buffer));
#endif

    return mpu->err;
}

/**
 * @brief Return biases from accel offset registers.
 * This returns the biases with OTP values from factory trim added,
 * so returned values will be different than that ones set with setAccelOffset().
 *
 * Note: Bias output are LSB in +-16G format.
 * */
raw_axes_t getAccelOffset(struct mpu *mpu)
{
    raw_axes_t bias;

#if defined CONFIG_MPU6050
    MPU_ERR_CHECK(mpu->readBytes(mpu, XA_OFFSET_H, 6, mpu->buffer));
    bias.x = (mpu->buffer[0] << 8) | mpu->buffer[1];
    bias.y = (mpu->buffer[2] << 8) | mpu->buffer[3];
    bias.z = (mpu->buffer[4] << 8) | mpu->buffer[5];

#elif defined CONFIG_MPU6500
    MPU_ERR_CHECK(mpu->readBytes(mpu, XA_OFFSET_H, 8, mpu->buffer));
    bias.data.x                        = (mpu->buffer[0] << 8) | mpu->buffer[1];
    bias.data.y                        = (mpu->buffer[3] << 8) | mpu->buffer[4];
    bias.data.z                        = (mpu->buffer[6] << 8) | mpu->buffer[7];
#endif

    return bias;
}

/**
 * @brief Compute Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the mpu must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
static esp_err_t computeOffsets(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro)
{
    const accel_fs_t kAccelFS = ACCEL_FS_2G;     // most sensitive
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;  // most sensitive
    if (MPU_ERR_CHECK(mpu->getBiases(mpu, kAccelFS, kGyroFS, accel, gyro, false))) {
        WK_DEBUGE(ERROR_TAG, "[computeOffsets] Error: Failed to get biases\n");
        return mpu->err;
    }
    // convert offsets to 16G and 1000DPS format and invert values
    for (int i = 0; i < 3; i++) {
        //acel bias / 8 (16 / 2)
        (*accel).xyz[i] = -((*accel).xyz[i] >> (ACCEL_FS_16G - kAccelFS));
        //gyro bias / 4 (1000 / 250)
        (*gyro).xyz[i]  = -((*gyro).xyz[i] >> (GYRO_FS_1000DPS - kGyroFS));
    }
    return mpu->err;
}

/**
 * @brief Read accelerometer raw data.
 * */
static esp_err_t acceleration(struct mpu *mpu, raw_axes_t* accel)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, 6, mpu->buffer))) return mpu->err;
    accel->data.x = mpu->buffer[0] << 8 | mpu->buffer[1];
    accel->data.y = mpu->buffer[2] << 8 | mpu->buffer[3];
    accel->data.z = mpu->buffer[4] << 8 | mpu->buffer[5];
    return mpu->err;
}

/**
 * @brief Read accelerometer raw data.
 * */
static esp_err_t acceleration_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, 6, mpu->buffer))) return mpu->err;
    *x = mpu->buffer[0] << 8 | mpu->buffer[1];
    *y = mpu->buffer[2] << 8 | mpu->buffer[3];
    *z = mpu->buffer[4] << 8 | mpu->buffer[5];
    return mpu->err;
}

/**
 * @brief Read gyroscope raw data.
 * */
static esp_err_t rotation(struct mpu *mpu, raw_axes_t* gyro)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, GYRO_XOUT_H, 6, mpu->buffer))) return mpu->err;
    gyro->data.x = mpu->buffer[0] << 8 | mpu->buffer[1];
    gyro->data.y = mpu->buffer[2] << 8 | mpu->buffer[3];
    gyro->data.z = mpu->buffer[4] << 8 | mpu->buffer[5];
    return mpu->err;
}

/**
 * @brief Read gyroscope raw data.
 * */
static esp_err_t rotation_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, GYRO_XOUT_H, 6, mpu->buffer))) return mpu->err;
    *x = mpu->buffer[0] << 8 | mpu->buffer[1];
    *y = mpu->buffer[2] << 8 | mpu->buffer[3];
    *z = mpu->buffer[4] << 8 | mpu->buffer[5];
    return mpu->err;
}

/**
 * Read temperature raw data.
 * */
static esp_err_t temperature(struct mpu *mpu, int16_t* temp)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, TEMP_OUT_H, 2, mpu->buffer))) return mpu->err;
    *temp = mpu->buffer[0] << 8 | mpu->buffer[1];
    return mpu->err;
}

/**
 * @brief Read accelerometer and gyroscope data at once.
 * */
static esp_err_t motion(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, 14, mpu->buffer))) return mpu->err;
    accel->data.x = mpu->buffer[0] << 8 | mpu->buffer[1];
    accel->data.y = mpu->buffer[2] << 8 | mpu->buffer[3];
    accel->data.z = mpu->buffer[4] << 8 | mpu->buffer[5];
    gyro->data.x  = mpu->buffer[8] << 8 | mpu->buffer[9];
    gyro->data.y  = mpu->buffer[10] << 8 | mpu->buffer[11];
    gyro->data.z  = mpu->buffer[12] << 8 | mpu->buffer[13];
    return mpu->err;
}

/**
 * @brief Read data from all internal sensors.
 * */
static esp_err_t sensors(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, 14, mpu->buffer))) return mpu->err;
    accel->data.x = mpu->buffer[0] << 8 | mpu->buffer[1];
    accel->data.y = mpu->buffer[2] << 8 | mpu->buffer[3];
    accel->data.z = mpu->buffer[4] << 8 | mpu->buffer[5];
    *temp    = mpu->buffer[6] << 8 | mpu->buffer[7];
    gyro->data.x  = mpu->buffer[8] << 8 | mpu->buffer[9];
    gyro->data.y  = mpu->buffer[10] << 8 | mpu->buffer[11];
    gyro->data.z  = mpu->buffer[12] << 8 | mpu->buffer[13];
    return mpu->err;
}

/**
 * @brief Read data from all sensors, including external sensors in Aux I2C.
 * */
static esp_err_t sensors_sen(struct mpu *mpu, sensors_t* sensors, size_t extsens_len)
{
    const size_t kIntSensLenMax = 14;  // internal sensors data length max
    const size_t kExtSensLenMax = 24;  // external sensors data length max
    uint8_t buffer[kIntSensLenMax + kExtSensLenMax];
#if defined AK89xx
    const size_t kMagLen = 8;  // magnetometer data length
    const size_t length      = kIntSensLenMax + extsens_len + kMagLen;
#else
    const size_t length           = kIntSensLenMax + extsens_len;
#endif
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, length, buffer))) return mpu->err;
    sensors->accel.data.x = buffer[0] << 8 | buffer[1];
    sensors->accel.data.y = buffer[2] << 8 | buffer[3];
    sensors->accel.data.z = buffer[4] << 8 | buffer[5];
    sensors->temp    = buffer[6] << 8 | buffer[7];
    sensors->gyro.data.x  = buffer[8] << 8 | buffer[9];
    sensors->gyro.data.y  = buffer[10] << 8 | buffer[11];
    sensors->gyro.data.z  = buffer[12] << 8 | buffer[13];
#if defined AK89xx
    sensors->mag.data.x = buffer[16] << 8 | buffer[15];
    sensors->mag.data.y = buffer[18] << 8 | buffer[17];
    sensors->mag.data.z = buffer[20] << 8 | buffer[19];
#endif
    memcpy(sensors->extsens, buffer + (length - extsens_len), extsens_len);
    return mpu->err;
}

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
/**
 * @brief The mpu-6050’s I/O logic levels are set to be either VDD or VLOGIC.
 *
 * VLOGIC may be set to be equal to VDD or to another voltage. However, VLOGIC must be ≤ VDD at all
 * times. When AUX_VDDIO is set to 0 (its power-on-reset value), VLOGIC is the power supply voltage
 * for both the microprocessor system bus and the auxiliary I C bus. When AUX_VDDIO is set to 1,
 * VLOGIC is the power supply voltage for the microprocessor system bus and VDD is the supply for
 * the auxiliary I2C bus
 * */
static esp_err_t setAuxVDDIOLevel(struct mpu *mpu, auxvddio_lvl_t level)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, YG_OTP_OFFSET_TC, TC_PWR_MODE_BIT, level));
}

/**
 * Return mpu-6050’s I/O logic levels.
 */
auxvddio_lvl_t getAuxVDDIOLevel(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, YG_OTP_OFFSET_TC, TC_PWR_MODE_BIT, mpu->buffer));
    return (auxvddio_lvl_t) mpu->buffer[0];
}
#endif

/**
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
static esp_err_t setInterruptConfig(struct mpu *mpu, int_config_t config)
{
    if (MPU_ERR_CHECK(mpu->readByte(mpu, INT_PIN_CONFIG, mpu->buffer))) return mpu->err;
    // zero the bits we're setting, but keep the others we're not setting as they are;
    const uint8_t INT_PIN_CONFIG_BITMASK = (1 << INT_CFG_LEVEL_BIT) | (1 << INT_CFG_OPEN_BIT) |
                                               (1 << INT_CFG_LATCH_EN_BIT) |
                                               (1 << INT_CFG_ANYRD_2CLEAR_BIT);
    mpu->buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    // set the configurations
    mpu->buffer[0] |= config.level << INT_CFG_LEVEL_BIT;
    mpu->buffer[0] |= config.drive << INT_CFG_OPEN_BIT;
    mpu->buffer[0] |= config.mode << INT_CFG_LATCH_EN_BIT;
    mpu->buffer[0] |= config.clear << INT_CFG_ANYRD_2CLEAR_BIT;
    return MPU_ERR_CHECK(mpu->writeByte(mpu, INT_PIN_CONFIG, mpu->buffer[0]));
}

/**
 * @brief Return Interrupt pin (INT) configuration.
 */
int_config_t getInterruptConfig(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, INT_PIN_CONFIG, mpu->buffer));
    int_config_t config;
    config.level = (int_lvl_t)((mpu->buffer[0] >> INT_CFG_LEVEL_BIT) & 0x1);
    config.drive = (int_drive_t)((mpu->buffer[0] >> INT_CFG_OPEN_BIT) & 0x1);
    config.mode  = (int_mode_t)((mpu->buffer[0] >> INT_CFG_LATCH_EN_BIT) & 0x1);
    config.clear = (int_clear_t)((mpu->buffer[0] >> INT_CFG_ANYRD_2CLEAR_BIT) & 0x1);
    return config;
}

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @param mask ORed features.
 */
static esp_err_t setInterruptEnabled(struct mpu *mpu, int_en_t mask)
{
    return MPU_ERR_CHECK(mpu->writeByte(mpu, INT_ENABLE, mask));
}

/**
 * @brief Return enabled features configured to generate signal at Interrupt pin.
 */
int_en_t getInterruptEnabled(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, INT_ENABLE, mpu->buffer));
    return (int_en_t) mpu->buffer[0];
}

/**
 * @brief Return the Interrupt status from INT_STATUS register.
 *
 * Note: Reading this register, clear all bits.
 */
int_stat_t getInterruptStatus(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, INT_STATUS, mpu->buffer));
    return (int_stat_t) mpu->buffer[0];
}

/**
 * @brief Change FIFO mode.
 *
 * Options:
 * `FIFO_MODE_OVERWRITE`: When the fifo is full, additional writes will be
 *  written to the fifo,replacing the oldest data.
 * `FIFO_MODE_STOP_FULL`: When the fifo is full, additional writes will not be written to fifo.
 * */
static esp_err_t setFIFOMode(struct mpu *mpu, fifo_mode_t mode)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, CONFIG, CONFIG_FIFO_MODE_BIT, mode));
}

/**
 * @brief Return FIFO mode.
 */
fifo_mode_t getFIFOMode(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, CONFIG, CONFIG_FIFO_MODE_BIT, mpu->buffer));
    return (fifo_mode_t) mpu->buffer[0];
}

/**
 * @brief Configure the sensors that will be written to the FIFO.
 * */
static esp_err_t setFIFOConfig(struct mpu *mpu, fifo_config_t config)
{
    if (MPU_ERR_CHECK(mpu->writeByte(mpu, FIFO_EN, (uint8_t) config))) return mpu->err;
    return MPU_ERR_CHECK(mpu->writeBit(mpu, I2C_MST_CTRL, I2CMST_CTRL_SLV_3_FIFO_EN_BIT, config >> 8));
}

/**
 * @brief Return FIFO configuration.
 */
fifo_config_t getFIFOConfig(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBytes(mpu, FIFO_EN, 2, mpu->buffer));
    fifo_config_t config = mpu->buffer[0];
    config |= (mpu->buffer[1] & (1 << I2CMST_CTRL_SLV_3_FIFO_EN_BIT)) << 3;
    return config;
}

/**
 * @brief Enabled / disable FIFO module.
 * */
static esp_err_t setFIFOEnabled(struct mpu *mpu, bool enable)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_FIFO_EN_BIT, (uint8_t) enable));
}

/**
 * @brief Return FIFO module state.
 */
bool getFIFOEnabled(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, USER_CTRL, USERCTRL_FIFO_EN_BIT, mpu->buffer));
    return mpu->buffer[0];
}

/**
 * @brief Reset FIFO module.
 *
 * Zero FIFO count, reset is asynchronous. \n
 * The bit auto clears after one clock cycle.
 * */
static esp_err_t resetFIFO(struct mpu *mpu)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_FIFO_RESET_BIT, 1));
}

/**
 * @brief Return number of written bytes in the FIFO.
 * @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
 * */
uint16_t getFIFOCount(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBytes(mpu, FIFO_COUNT_H, 2, mpu->buffer));
    uint16_t count = mpu->buffer[0] << 8 | mpu->buffer[1];
    return count;
}

/**
 * @brief Read data contained in FIFO mpu->buffer.
 * */
static esp_err_t readFIFO(struct mpu *mpu, size_t length, uint8_t* data)
{
    return MPU_ERR_CHECK(mpu->readBytes(mpu, FIFO_R_W, length, data));
}

/**
 * @brief Write data to FIFO mpu->buffer.
 * */
static esp_err_t writeFIFO(struct mpu *mpu, size_t length, const uint8_t* data)
{
    return MPU_ERR_CHECK(mpu->writeBytes(mpu, FIFO_R_W, length, data));
}

/**
 * @brief Configure the Auxiliary I2C Master.
 * @note For [MPU9150, MPU9250]: The Auxiliary I2C is configured in the initialization stage
 *  to connect with the compass in Slave 0 and Slave 1.
 * */
static esp_err_t setAuxI2CConfig(struct mpu *mpu, const auxi2c_config_t* config)
{
    // TODO: check compass enabled, to constrain sample_delay which defines the compass read sample
    // rate
    if (MPU_ERR_CHECK(mpu->readBit(mpu, I2C_MST_CTRL, I2CMST_CTRL_SLV_3_FIFO_EN_BIT, mpu->buffer))) {
        return mpu->err;
    }
    mpu->buffer[0] <<= I2CMST_CTRL_SLV_3_FIFO_EN_BIT;
    mpu->buffer[0] |= config->multi_master_en << I2CMST_CTRL_MULT_EN_BIT;
    mpu->buffer[0] |= config->wait_for_es << I2CMST_CTRL_WAIT_FOR_ES_BIT;
    mpu->buffer[0] |= config->transition << I2CMST_CTRL_P_NSR_BIT;
    mpu->buffer[0] |= config->clock;
    if (MPU_ERR_CHECK(mpu->writeByte(mpu, I2C_MST_CTRL, mpu->buffer[0]))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->writeBits(mpu, I2C_SLV4_CTRL, I2C_SLV4_MST_DELAY_BIT, I2C_SLV4_MST_DELAY_LENGTH,
                                config->sample_delay))) {
        return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, I2C_MST_DELAY_CRTL, I2CMST_DLY_ES_SHADOW_BIT, config->shadow_delay_en))) {
        return mpu->err;
    }
    /*
    WK_DEBUGE(ERROR_TAG, "EMPTY, Master:: multi_master_en: %d, wait_for_es: %d,"
                "transition: %d, clock: %d, sample_delay: %d, shadow_delay_en: %d\n",
                config->multi_master_en, config->wait_for_es, config->transition, config->clock, config->sample_delay,
                config->shadow_delay_en);
    */
    return mpu->err;
}

/**
 * @brief Get Auxiliary I2C Master configuration.
 */
auxi2c_config_t getAuxI2CConfig(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_CTRL, mpu->buffer));
    auxi2c_config_t config;
    config.multi_master_en = mpu->buffer[0] >> I2CMST_CTRL_MULT_EN_BIT;
    config.wait_for_es     = (mpu->buffer[0] >> I2CMST_CTRL_WAIT_FOR_ES_BIT) & 0x1;
    config.transition      = (auxi2c_trans_t)((mpu->buffer[0] >> I2CMST_CTRL_P_NSR_BIT) & 0x1);
    config.clock           = (auxi2c_clock_t)(mpu->buffer[0] & ((1 << I2CMST_CTRL_CLOCK_LENGTH) - 1));
    MPU_ERR_CHECK(
        mpu->readBits(mpu, I2C_SLV4_CTRL, I2C_SLV4_MST_DELAY_BIT, I2C_SLV4_MST_DELAY_LENGTH, mpu->buffer + 1));
    config.sample_delay = mpu->buffer[1];
    MPU_ERR_CHECK(mpu->readBit(mpu, I2C_MST_DELAY_CRTL, I2CMST_DLY_ES_SHADOW_BIT, mpu->buffer + 2));
    config.shadow_delay_en = mpu->buffer[2];
    return config;
}

/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
static esp_err_t setAuxI2CEnabled(struct mpu *mpu, bool enable)
{
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, (uint8_t) enable))) return mpu->err;
    if (enable) {
        return MPU_ERR_CHECK(mpu->writeBit(mpu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, 0));
    }
    return mpu->err;
}

/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
static esp_err_t setAuxI2CReset(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, 1));
    return mpu->err;
}

/**
 * @brief Return Auxiliary I2C Master state.
 */
bool getAuxI2CEnabled(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, mpu->buffer));
    MPU_ERR_CHECK(mpu->readBit(mpu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, mpu->buffer + 1));
    return mpu->buffer[0] && (!mpu->buffer[1]);
}

/**
 * @brief Configure communication with a Slave connected to Auxiliary I2C bus.
 * */
static esp_err_t setAuxI2CSlaveConfig(struct mpu *mpu, const auxi2c_slv_config_t* config)
{
    // slaves' config registers are grouped as 3 regs in a row
    const uint8_t regAddr = config->slave * 3 + I2C_SLV0_ADDR;
    // data for I2C_SLVx_ADDR
    mpu->buffer[0] = config->rw << I2C_SLV_RNW_BIT;
    mpu->buffer[0] |= config->addr;
    // data for I2C_SLVx_REG
    mpu->buffer[1] = config->reg_addr;
    // data for I2C_SLVx_CTRL
    if (MPU_ERR_CHECK(mpu->readByte(mpu, regAddr + 2, mpu->buffer + 2))) return mpu->err;
    if (config->rw == AUXI2C_READ) {
        mpu->buffer[2] &= 1 << I2C_SLV_EN_BIT;  // keep enable bit, clear the rest
        mpu->buffer[2] |= config->reg_dis << I2C_SLV_REG_DIS_BIT;
        mpu->buffer[2] |= config->swap_en << I2C_SLV_BYTE_SW_BIT;
        mpu->buffer[2] |= config->end_of_word << I2C_SLV_GRP_BIT;
        mpu->buffer[2] |= config->rxlength & 0xF;
    }
    else {                                                     // AUXI2C_WRITE
        mpu->buffer[2] &= ~(1 << I2C_SLV_REG_DIS_BIT | 0xF);  // clear length bits and register disable bit
        mpu->buffer[2] |= config->reg_dis << I2C_SLV_REG_DIS_BIT;
        mpu->buffer[2] |= 0x1;  // set length to write 1 byte
        if (MPU_ERR_CHECK(mpu->writeByte(mpu, I2C_SLV0_DO + config->slave, config->txdata))) return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, regAddr, 3, mpu->buffer))) return mpu->err;
    // sample_delay enable/disable
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, I2C_MST_DELAY_CRTL, config->slave, config->sample_delay_en))) {
        return mpu->err;
    }
    /*
    WKDEBUGE(ERROR_TAG, "EMPTY, Slave%d:: r/w: %s, addr: 0x%X, reg_addr: 0x%X, reg_dis: %d, %s: 0x%X, sample_delay_en: %d\n",
        config->slave, (config->rw == AUXI2C_READ ? "read" : "write"), config->addr, config->reg_addr, config->reg_dis,
        (config->rw == AUXI2C_READ ? "rxlength" : "txdata"), config->txdata, config->sample_delay_en);
    */
    return mpu->err;
}

/**
 * @brief Return configuration of a Aux I2C Slave.
 * @param slave slave number.
 */
auxi2c_slv_config_t getAuxI2CSlaveConfig(struct mpu *mpu, auxi2c_slv_t slave)
{
    auxi2c_slv_config_t config;
    const uint8_t regAddr = slave * 3 + I2C_SLV0_ADDR;
    config.slave          = slave;
    MPU_ERR_CHECK(mpu->readBytes(mpu, regAddr, 3, mpu->buffer));
    config.rw       = (auxi2c_rw_t)((mpu->buffer[0] >> I2C_SLV_RNW_BIT) & 0x1);
    config.addr     = mpu->buffer[0] & 0x7F;
    config.reg_addr = mpu->buffer[1];
    config.reg_dis  = (mpu->buffer[2] >> I2C_SLV_REG_DIS_BIT) & 0x1;
    if (config.rw == AUXI2C_READ) {
        config.swap_en     = (mpu->buffer[2] >> I2C_SLV_BYTE_SW_BIT) & 0x1;
        config.end_of_word = (auxi2c_eow_t)((mpu->buffer[2] >> I2C_SLV_GRP_BIT) & 0x1);
        config.rxlength    = mpu->buffer[2] & 0xF;
    }
    else {
        MPU_ERR_CHECK(mpu->readByte(mpu, I2C_SLV0_DO + slave, mpu->buffer + 3));
        config.txdata = mpu->buffer[3];
    }
    MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_DELAY_CRTL, mpu->buffer + 4));
    config.sample_delay_en = (mpu->buffer[4] >> slave) & 0x1;
    return config;
}

/**
 * @brief Enable the Auxiliary I2C module to transfer data with a slave at sample rate.
 * */
static esp_err_t setAuxI2CSlaveEnabled(struct mpu *mpu, auxi2c_slv_t slave, bool enable)
{
    const uint8_t regAddr = slave * 3 + I2C_SLV0_CTRL;
    return MPU_ERR_CHECK(mpu->writeBit(mpu, regAddr, I2C_SLV_EN_BIT, enable));
}

/**
 * @brief Return enable state of a Aux I2C's Slave.
 */
bool getAuxI2CSlaveEnabled(struct mpu *mpu, auxi2c_slv_t slave)
{
    const uint8_t regAddr = slave * 3 + I2C_SLV0_CTRL;
    MPU_ERR_CHECK(mpu->readBit(mpu, regAddr, I2C_SLV_EN_BIT, mpu->buffer));
    return mpu->buffer[0];
}

/**
 * @brief Enable / disable Auxiliary I2C bypass mode.
 * @param enable
 *  - `true`: Auxiliar I2C Master I/F is disabled, and Bypass enabled.
 *  - `false`: Bypass is disabled, but the Auxiliar I2C Master I/F is not enabled back,
 *             if needed, enabled it again with setAuxI2CmasterEnabled().
 * */
static esp_err_t setAuxI2CBypass(struct mpu *mpu, bool enable)
{
#ifdef CONFIG_MPU_SPI
    if (enable) {
        WK_DEBUGE(ERROR_TAG, "EMPTY, Setting Aux I2C to bypass mode while mpu is connected via SPI");
    }
#endif
    if (enable) {
        if (MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, 0))) return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, enable))) {
        return mpu->err;
    }
    return mpu->err;
}

/**
 * @brief Return Auxiliary I2C Master bypass mode state.
 */
bool getAuxI2CBypass(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, mpu->buffer));
    MPU_ERR_CHECK(mpu->readBit(mpu, INT_PIN_CONFIG, INT_CFG_I2C_BYPASS_EN_BIT, mpu->buffer + 1));
    return (!mpu->buffer[0]) && mpu->buffer[1];
}

/**
 * @brief Read data from slaves connected to Auxiliar I2C bus.
 *
 * Data is placed in these external sensor data registers according to I2C_SLV0_CTRL,
 * I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When
 * more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the
 * slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in
 * Register 52 and 103). During each sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the order will be Slave
 * 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of available
 * EXT_SENS_DATA registers, the excess bytes will be dropped. There are 24 EXT_SENS_DATA
 * registers and hence the total read lengths between all the slaves cannot be greater than 24 or
 * some bytes will be lost.
 *
 * @attention Set `skip` to `8` when using compass, because compass data takes up the first `8` bytes.
 * */
static esp_err_t readAuxI2CRxData(struct mpu *mpu, size_t length, uint8_t* data, size_t skip)
{
    if (length + skip > 24) {
        WK_DEBUGE(ERROR_TAG, "INVALID_LENGTH,  %d, mpu has only 24 external sensor data registers!", length);
        return mpu->err = ESP_ERR_INVALID_SIZE;
    }
// check if I2C Master is enabled, just for warning and debug
    const bool kAuxI2CEnabled = getAuxI2CEnabled(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (!kAuxI2CEnabled) WK_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , better turn on.\n");
    // read the specified amount of registers
    return MPU_ERR_CHECK(mpu->readBytes(mpu, EXT_SENS_DATA_00 + skip, length, data));
}

/**
 * @brief Restart Auxiliary I2C Master module, reset is asynchronous.
 *
 * This bit (I2C_MST_RST) should only be set when the I2C master has hung. If this bit
 * is set during an active I2C master transaction, the I2C slave will hang, which
 * will require the host to reset the slave.
 * */
static esp_err_t restartAuxI2C(struct mpu *mpu)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, 1));
}

/**
 * @brief Return Auxiliary I2C Master status from register I2C_MST_STATUS.
 * Reading this register clear all its bits.
 * */
auxi2c_stat_t getAuxI2CStatus(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_STATUS, mpu->buffer));
    return (auxi2c_stat_t) mpu->buffer[0];
}

/**
 * @brief Write to a slave a single byte just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - `ESP_ERR_INVALID_STATE`: Auxiliary I2C Master not enabled;
 *  - `ESP_ERR_NOT_FOUND`:     Slave doesn't ACK the transfer;
 *  - `ESP_FAIL`:               Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
static esp_err_t auxI2CWriteByte(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, const uint8_t data)
{
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = mpu->getAuxI2CEnabled(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (!kAuxI2CEnabled) {
        WK_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , must enable first\n");
        return mpu->err = ESP_ERR_INVALID_STATE;
    }
    // data for I2C_SLV4_ADDR
    mpu->buffer[0] = AUXI2C_WRITE << I2C_SLV_RNW_BIT;
    mpu->buffer[0] |= devAddr & (0x7F);
    // data for I2C_SLV4_REG
    mpu->buffer[1] = regAddr;
    // data for I2C_SLV4_DO
    mpu->buffer[2] = data;
    // write configuration above to slave 4 registers
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, I2C_SLV4_ADDR, 3, mpu->buffer))) return mpu->err;
    // clear status register before enable this transfer
    if (MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_STATUS, mpu->buffer + 15))) return mpu->err;
    // enable transfer in slave 4
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, 1))) return mpu->err;
    // check status until transfer is done
    TickType_t startTick = xTaskGetTickCount();
    TickType_t endTick   = startTick + pdMS_TO_TICKS(1000);
    auxi2c_stat_t status = 0x00;
    do {
        if (MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_STATUS, &status))) return mpu->err;
        if (status & (1 << I2CMST_STAT_SLV4_NACK_BIT)) {
            WK_DEBUGE(ERROR_TAG, "AUX_I2C_SLAVE_NACK, %02x\n", (uint8_t)status);
            return mpu->err = ESP_ERR_NOT_FOUND;
        }
        if (status & (1 << I2CMST_STAT_LOST_ARB_BIT)) {
            WK_DEBUGE(ERROR_TAG, "AUX_I2C_LOST_ARB, ");
            return mpu->err = ESP_FAIL;
        }
        if (xTaskGetTickCount() >= endTick) {
            WK_DEBUGE(ERROR_TAG, "TIMEOUT, . Aux I2C might've hung. Restart it.");
            return mpu->err = ESP_ERR_TIMEOUT;
        }
    } while (!(status & (1 << I2C_SLV4_DONE_INT_BIT)));
    return mpu->err = ESP_OK;
}

/**
 * @brief Read a single byte frome slave just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - ESP_ERR_INVALID_STATE  Auxiliary I2C Master not enabled;
 *  - ESP_ERR_NOT_FOUND      Slave doesn't ACK the transfer;
 *  - ESP_FAIL               Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
static esp_err_t auxI2CReadByte(struct mpu *mpu, uint8_t devAddr, uint8_t regAddr, uint8_t* data)
{
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = mpu->getAuxI2CEnabled(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (!kAuxI2CEnabled) {
        WK_DEBUGE(ERROR_TAG, "AUX_I2C_DISABLED, , must enable first\n");
        return mpu->err = ESP_ERR_INVALID_STATE;
    }
    // data for I2C_SLV4_ADDR
    mpu->buffer[0] = AUXI2C_READ << I2C_SLV_RNW_BIT;
    mpu->buffer[0] |= devAddr & (0x7F);
    // data for I2C_SLV4_REG
    mpu->buffer[1] = regAddr;
    // write configuration above to slave 4 registers
    if (MPU_ERR_CHECK(mpu->writeBytes(mpu, I2C_SLV4_ADDR, 2, mpu->buffer))) return mpu->err;
    // clear status register before enable this transfer
    if (MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_STATUS, mpu->buffer + 15))) return mpu->err;
    // enable transfer in slave 4
    if (MPU_ERR_CHECK(mpu->writeBit(mpu, I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, 1))) return mpu->err;
    // check status until transfer is done
    TickType_t startTick = xTaskGetTickCount();
    TickType_t endTick   = startTick + pdMS_TO_TICKS(1000);
    auxi2c_stat_t status = 0x00;
    do {
        if (MPU_ERR_CHECK(mpu->readByte(mpu, I2C_MST_STATUS, &status))) return mpu->err;
        if (status & (1 << I2CMST_STAT_SLV4_NACK_BIT)) {
            WK_DEBUGE(ERROR_TAG, "AUX_I2C_SLAVE_NACK, %02x\n", (uint8_t)status);
            return mpu->err = ESP_ERR_NOT_FOUND;
        }
        if (status & (1 << I2CMST_STAT_LOST_ARB_BIT)) {
            WK_DEBUGE(ERROR_TAG, "AUX_I2C_LOST_ARB, ");
            return mpu->err = ESP_FAIL;
        }
        if (xTaskGetTickCount() >= endTick) {
            WK_DEBUGE(ERROR_TAG, "TIMEOUT, . Aux I2C might've hung. Restart it.");
            return mpu->err = ESP_ERR_TIMEOUT;
        }
    } while (!(status & (1 << I2C_SLV4_DONE_INT_BIT)));
    // get read value
    return MPU_ERR_CHECK(mpu->readByte(mpu, I2C_SLV4_DI, data));
}

/**
 * @brief Configure the active level of FSYNC pin that will cause an interrupt.
 * @details Use setFsyncEnabled() to enable / disable this interrupt.
 * */
static esp_err_t setFsyncConfig(struct mpu *mpu, int_lvl_t level)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, INT_PIN_CONFIG, INT_CFG_FSYNC_LEVEL_BIT, level));
}

/**
 * @brief Return FSYNC pin active level configuration.
 */
int_lvl_t getFsyncConfig(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, INT_PIN_CONFIG, INT_CFG_FSYNC_LEVEL_BIT, mpu->buffer));
    return (int_lvl_t) mpu->buffer[0];
}

/**
 * @brief Enable / disable FSYNC pin to cause an interrupt.
 * @note
 * - The interrupt status is located in I2C_MST_STATUS register, so use
 *   the method getAuxI2CStatus() which reads this register to get FSYNC status.
 *   Keep in mind that a read from I2C_MST_STATUS register clears all its status bits,
 *   so take care to miss status bits when using Auxiliary I2C bus too.
 *
 * - It is possible to enable the FSYNC interrupt propagate to INT pin
 *   with setInterruptEnabled(), then the status can also be read with getInterruptStatus().
 *
 * @see setFsyncConfig().
 * */
static esp_err_t setFsyncEnabled(struct mpu *mpu, bool enable)
{
    return MPU_ERR_CHECK(mpu->writeBit(mpu, INT_PIN_CONFIG, INT_CFG_FSYNC_INT_MODE_EN_BIT, enable));
}

/**
 * @brief Return FSYNC enable state.
 */
bool getFsyncEnabled(struct mpu *mpu)
{
    MPU_ERR_CHECK(mpu->readBit(mpu, INT_PIN_CONFIG, INT_CFG_FSYNC_INT_MODE_EN_BIT, mpu->buffer));
    return mpu->buffer[0];
}

/**
 * @brief Print out register values for debugging purposes.
 * @param start first register number.
 * @param end last register number.
 */
static esp_err_t registerDump(struct mpu *mpu, uint8_t start, uint8_t end)
{
    const uint8_t kNumOfRegs = 128;
    if (end - start < 0 || start >= kNumOfRegs || end >= kNumOfRegs) return mpu->err = ESP_FAIL;
    WK_DEBUGE(ERROR_TAG, "LOG_COLOR_W >>  CONFIG_MPU_CHIP_MODEL  register dump: LOG_RESET_COLOR \n");
    uint8_t data;
    for (int i = start; i <= end; i++) {
        if (MPU_ERR_CHECK(mpu->readByte(mpu, i, &data))) {
            WK_DEBUGE(ERROR_TAG, "Reading Error.");
            return mpu->err;
        }
        WK_DEBUGE(ERROR_TAG, "mpu: reg[ 0x%s%X ]  data( 0x%s%X )\n", i < 0x10 ? "0" : "", i, data < 0x10 ? "0" : "", data);
    }
    return mpu->err;
}

#if defined CONFIG_LIS3MDL
/**
 * @brief Read a single byte from magnetometer.
 *
 * How it's done: \n
 * It will check the communication protocol which the mpu is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
static esp_err_t compassReadByte(struct mpu *mpu, uint8_t regAddr, uint8_t* data)
{
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = mpu->getAuxI2CBypass(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (kPrevAuxI2CBypassState == false) {
        if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, true))) return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->bus->readByte(mpu->bus, COMPASS_I2CADDRESS, regAddr, data))) return mpu->err;
    if (kPrevAuxI2CBypassState == false) {
        if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, false))) return mpu->err;
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    return MPU_ERR_CHECK(mpu->auxI2CReadByte(mpu, COMPASS_I2CADDRESS, regAddr, data));
#endif
    return mpu->err;
}

/**
 * @brief Write a single byte to magnetometer.
 *
 * How it's done: \n
 * It will check the communication protocol which the mpu is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
static esp_err_t compassWriteByte(struct mpu *mpu, uint8_t regAddr, const uint8_t data)
{
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = mpu->getAuxI2CBypass(mpu);
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    if (kPrevAuxI2CBypassState == false) {
        if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, true))) return mpu->err;
    }
    if (MPU_ERR_CHECK(mpu->bus->writeByte(mpu->bus, COMPASS_I2CADDRESS, regAddr, data))) return mpu->err;
    if (kPrevAuxI2CBypassState == false) {
        if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, false))) return mpu->err;
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    return MPU_ERR_CHECK(mpu->auxI2CWriteByte(mpu, COMPASS_I2CADDRESS, regAddr, data));
#endif
    return mpu->err;
}

/**
 * @brief Initialize Magnetometer sensor.
 *
 * Initial configuration:
 *  - Mode: single measurement (permits variable sample rate).
 *  - Sensitivity: 0.15 uT/LSB  =  16-bit output.
 *
 * To disable the compass, call compassSetMode(MAG_MODE_POWER_DOWN).
 * */
static esp_err_t compassInit(struct mpu *mpu)
{
    if (MPU_ERR_CHECK(mpu->setAuxI2CReset(mpu))) return mpu->err;
    // must delay, or compass may not be initialized
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // I2C 接口设置旁路模式
#ifdef CONFIG_MPU_I2C
    if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, true))) return mpu->err;
    // SPI接口设置外接磁力计
#elif CONFIG_MPU_SPI
    const auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART  //
    };
    if (MPU_ERR_CHECK(mpu->setAuxI2CConfig(mpu, &kAuxI2CConfig))) return mpu->err;
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    if (MPU_ERR_CHECK(mpu->setAuxI2CEnabled(mpu, true))) return mpu->err;
    //vTaskDelay(50 / portTICK_PERIOD_MS);
#endif

    // slave 0 reads from magnetometer data register
    const auxi2c_slv_config_t kSlaveReadDataConfig = {
        .slave           = MAG_SLAVE_READ_DATA,
        .addr            = COMPASS_I2CADDRESS,
        .rw              = AUXI2C_READ,
        .reg_addr        = LIS3MDL_REG_OUT_X_L,
        .reg_dis         = 0,
        .sample_delay_en = 0,
        {{
            .swap_en     = 0,
            .end_of_word = (auxi2c_eow_t) 0,
            .rxlength    = 6  //
        }}                    //
    };
    if (MPU_ERR_CHECK(mpu->setAuxI2CSlaveConfig(mpu, &kSlaveReadDataConfig))) return mpu->err;

    /*
    // slave 1 changes mode to single measurement
    const auxi2c_slv_config_t kSlaveChgModeConfig = {
        .slave           = MAG_SLAVE_CHG_MODE,
        .addr            = COMPASS_I2CADDRESS,
        .rw              = AUXI2C_WRITE,
        .reg_addr        = CONTROL1,
        .reg_dis         = 0,
        .sample_delay_en = 1,
        {.txdata = kControl1Value}  //
    };
    if (MPU_ERR_CHECK(mpu->setAuxI2CSlaveConfig(mpu, &kSlaveChgModeConfig))) return mpu->err;
    // enable slaves
    if (MPU_ERR_CHECK(mpu->setAuxI2CSlaveEnabled(mpu, MAG_SLAVE_CHG_MODE, true))) return mpu->err;
    */

    if (MPU_ERR_CHECK(mpu->setAuxI2CSlaveEnabled(mpu, MAG_SLAVE_READ_DATA, true))) return mpu->err;

    // who am i
    compassWhoAmI(mpu);
    //while (1) {
    while (mpu->buffer[0] != LIS3MDL_CHIP_ID) {
        esp_err_t r_error = compassWhoAmI(mpu);
        //esp_err_t w_error = compassReset(mpu);
        WK_DEBUGE(ERROR_TAG, "LIS3MDL who am i error: %s\n", esp_err_to_name(r_error));
        //if (r_error == w_error) {
        //    return ESP_FAIL;
        //}
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);

    /* configure the magnetometer */
    //if (MPU_ERR_CHECK(mpu->compassReset(mpu))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setMagfullScale(mpu, lis3mdl_scale_12_Gs))) return mpu->err;
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    if (MPU_ERR_CHECK(mpu->compassSetSampleMode(mpu, lis3mdl_lpm_1000))) return mpu->err;
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    if (MPU_ERR_CHECK(mpu->compassSetMeasurementMode(mpu, lis3mdl_continuous_measurement))) return mpu->err;
    //vTaskDelay(50 / portTICK_PERIOD_MS);

    // finished configs, disable bypass mode
#ifdef CONFIG_MPU_I2C
    if (MPU_ERR_CHECK(mpu->setAuxI2CBypass(mpu, false))) return mpu->err;
#endif

    return mpu->err;
}
/**
 * @brief Soft reset LIS3MDL.
 * */
static esp_err_t compassReset(struct mpu *mpu)
{
    return (mpu->compassWriteByte(mpu, LIS3MDL_REG_CTRL2, 0x04));
}

/**
 * @brief Return value from WHO_I_AM register.
 * @details Should be `0x48` for AK8963 and AK8975.
 * */
static esp_err_t compassWhoAmI(struct mpu *mpu)
{
    return mpu->compassReadByte(mpu, LIS3MDL_REG_WHO_AM_I, mpu->buffer);
}

/**
 * @brief Change magnetometer's measurement mode.
 * @note
 *  - When user wants to change operation mode, transit to power-down mode first and then transit to other modes.
 *    After power-down mode is set, at least 100µs(Twat) is needed before setting another mode.
 *  - Setting to MAG_MODE_POWER_DOWN will disable readings from compass and disable (free) Aux I2C slaves 0 and 1.
 *    It will not disable Aux I2C Master I/F though! To enable back, use compassInit().
 * */
static esp_err_t compassSetSampleMode(struct mpu *mpu, mag_mode_t mode)
{
    uint8_t ctrl_reg1 = 0x00;
    uint8_t ctrl_reg4 = 0x00;

    switch (mode) {
    case lis3mdl_lpm_0_625:  // low power mode at 0.625 Hz
        ctrl_reg1 = 0x00;
        break;
    case lis3mdl_lpm_1_25:       // low power mode at 1.25 Hz
        ctrl_reg1 = 0x04;
        break;
    case lis3mdl_lpm_2_5:        // low power mode at 2.5 Hz
        ctrl_reg1 = 0x08;
        break;
    case lis3mdl_lpm_5:          // low power mode at 5 Hz
        ctrl_reg1 = 0x0c;
        break;
    case lis3mdl_lpm_10:         // low power mode at 10 Hz
        ctrl_reg1 = 0x10;
        break;
    case lis3mdl_lpm_20:         // low power mode at 20 Hz
        ctrl_reg1 = 0x14;
        break;
    case lis3mdl_lpm_40:         // low power mode at 40 Hz
        ctrl_reg1 = 0x18;
        break;
    case lis3mdl_lpm_80:         // low power mode at 80 Hz
        ctrl_reg1 = 0x1c;
        break;
    case lis3mdl_lpm_1000:       // low power mode at 1000 Hz
        ctrl_reg1 = 0x02;
        break;
    case lis3mdl_mpm_560:        // medium performance mode at 560 Hz
        ctrl_reg1 = 0x22;
        ctrl_reg4 = 0x04;
        break;
    case lis3mdl_hpm_300:        // high performance mode at 300 Hz
        ctrl_reg1 = 0x42;
        ctrl_reg4 = 0x08;
        break;
    case lis3mdl_uhpm_155:       // ultra high performance mode at 155 Hz
        ctrl_reg1 = 0x62;
        ctrl_reg4 = 0x0c;
        break;
    case lis3mdl_low_power:      // low power mode at 0.625 Hz
        break;
    default:
        WK_DEBUGE(ERROR_TAG, "[compassSetSampleMode] should never get to here\n");
    }
    MPU_ERR_CHECK(compassWriteByte(mpu, LIS3MDL_REG_CTRL1, ctrl_reg1));
    MPU_ERR_CHECK(compassWriteByte(mpu, LIS3MDL_REG_CTRL4, ctrl_reg4));

    return mpu->err = ESP_OK;
}

/**
 * @brief set LIS3MDL FULL-scale range
 * @details
 * */
static esp_err_t setMagfullScale(struct mpu *mpu, lis3mdl_scale_t scale)
{
    uint8_t ctrl_reg2;
    if (MPU_ERR_CHECK(compassReadByte(mpu, LIS3MDL_REG_CTRL2, &ctrl_reg2))) {
        WK_DEBUGE(ERROR_TAG, "error: setMagfullScale compassReadByte error\n");
    }
    switch (scale) {
    case lis3mdl_scale_4_Gs:
        ctrl_reg2 &= 0x9f;
        break;
    case lis3mdl_scale_8_Gs:
        ctrl_reg2 &= 0xbf;
        ctrl_reg2 |= 0x20;
        break;
    case lis3mdl_scale_12_Gs:
        ctrl_reg2 |= 0x40;
        ctrl_reg2 &= 0xdf;
        break;
    case lis3mdl_scale_16_Gs:
        ctrl_reg2 |= 0x60;
        break;
    default:
        WK_DEBUGE(ERROR_TAG, "[setMagfullScale] should never get to here\n");
    }
    return MPU_ERR_CHECK(mpu->compassWriteByte(mpu, LIS3MDL_REG_CTRL2, ctrl_reg2));
}

/**
 * @brief Change magnetometer's measurement mode.
 * @note
 *  - Setting to MAG_MODE_POWER_DOWN will disable readings from compass and disable (free) Aux I2C slaves 0 and 1.
 *    It will not disable Aux I2C Master I/F though! To enable back, use compassInit().
 * */
static esp_err_t compassSetMeasurementMode(struct mpu *mpu, lis3mdl_measurement_mode_t mode)
{
    uint8_t ctrl_reg3 = 0x00;

    //MPU_ERR_CHECK(compassReadByte(mpu, LIS3MDL_REG_CTRL3, &ctrl_reg3));
    switch (mode) {
    case lis3mdl_power_down:
        ctrl_reg3 |= 0x02;
        break;
    case lis3mdl_single_measurement:
        ctrl_reg3 &= 0xfd;
        ctrl_reg3 |= 0x01;
        break;
    case lis3mdl_continuous_measurement:
        ctrl_reg3 &= 0xfc;
        break;
    default:
        WK_DEBUGE(ERROR_TAG, "[compassSetMeasurementMode] should never get to here\n");
    }
    return MPU_ERR_CHECK(mpu->compassWriteByte(mpu, LIS3MDL_REG_CTRL3, ctrl_reg3));
}

/**
 * @brief Read compass data.
 * */
static esp_err_t heading(struct mpu *mpu, raw_axes_t* mag)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, EXT_SENS_DATA_00, 6, mpu->buffer))) return mpu->err;
    mag->data.x = mpu->buffer[1] << 8 | mpu->buffer[0];
    mag->data.y = mpu->buffer[3] << 8 | mpu->buffer[2];
    mag->data.z = mpu->buffer[5] << 8 | mpu->buffer[4];
    return mpu->err;
}

/**
 * @brief Read compass data.
 * */
static esp_err_t heading_xyz(struct mpu *mpu, int16_t* x, int16_t* y, int16_t* z)
{
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, EXT_SENS_DATA_01, 6, mpu->buffer))) return mpu->err;
    *x = mpu->buffer[1] << 8 | mpu->buffer[0];
    *y = mpu->buffer[3] << 8 | mpu->buffer[2];
    *z = mpu->buffer[5] << 8 | mpu->buffer[4];
    return mpu->err;
}

/**
 * @brief Read accelerometer, gyroscope, compass raw data.
 * */
static esp_err_t motion_mag(struct mpu *mpu, raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag)
{
    uint8_t buffer[22];
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, ACCEL_XOUT_H, 22, buffer))) return mpu->err;
    accel->data.x = buffer[0] << 8 | buffer[1];
    accel->data.y = buffer[2] << 8 | buffer[3];
    accel->data.z = buffer[4] << 8 | buffer[5];
    gyro->data.x  = buffer[8] << 8 | buffer[9];
    gyro->data.y  = buffer[10] << 8 | buffer[11];
    gyro->data.z  = buffer[12] << 8 | buffer[13];
    mag->data.x   = buffer[16] << 8 | buffer[15];
    mag->data.y   = buffer[18] << 8 | buffer[17];
    mag->data.z   = buffer[20] << 8 | buffer[19];
    return mpu->err;
}
#endif // CONFIG_LIS3MDL

/**
 * @brief Trigger gyro and accel hardware self-test.
 * @attention when calling this function, the mpu must remain as horizontal as possible (0 degrees), facing up.
 * @param result Should be ZERO if gyro and accel passed.
 * @todo Elaborate doc.
 * */
static esp_err_t selfTest(struct mpu *mpu, selftest_t* result)
{
#ifdef CONFIG_MPU6050
    const accel_fs_t kAccelFS = ACCEL_FS_16G;
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#elif defined CONFIG_MPU6500
    const accel_fs_t kAccelFS = ACCEL_FS_2G;
    const gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#endif
    raw_axes_t gyroRegBias, accelRegBias;
    raw_axes_t gyroSTBias, accelSTBias;
    // get regular biases
    if (MPU_ERR_CHECK(mpu->getBiases(mpu, kAccelFS, kGyroFS, &accelRegBias, &gyroRegBias, false))) return mpu->err;
    // get self-test biases
    if (MPU_ERR_CHECK(mpu->getBiases(mpu, kAccelFS, kGyroFS, &accelSTBias, &gyroSTBias, true))) return mpu->err;
    // perform self-tests
    uint8_t accelST, gyroST;
    if (MPU_ERR_CHECK(mpu->accelSelfTest(mpu, &accelRegBias, &accelSTBias, &accelST))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->gyroSelfTest(mpu, &gyroRegBias, &gyroSTBias, &gyroST))) return mpu->err;
    // check results
    *result = 0;
    if (accelST != 0) *result |= SELF_TEST_ACCEL_FAIL;
    if (gyroST != 0) *result |= SELF_TEST_GYRO_FAIL;
    return mpu->err;
}

/**
 * @brief set gyro bias.
 * @attention when calling this function, the mpu must remain as horizontal as possible (0 degrees), facing up.
 * @param result Should be ZERO if gyro and accel passed.
 * @todo Elaborate doc.
 * */
static esp_err_t setGyroBias(struct mpu *mpu)
{
    const accel_fs_t kAccelFS = accel_fs;
    const gyro_fs_t kGyroFS   = gyro_fs;
    raw_axes_t gyroRegBias, accelRegBias;
    // get regular biases
    if (MPU_ERR_CHECK(mpu->getBiases(mpu, kAccelFS, kGyroFS, &accelRegBias, &gyroRegBias, false))) return mpu->err;
    gyroRegBias.x = -gyroRegBias.x; gyroRegBias.y = -gyroRegBias.y; gyroRegBias.z = -gyroRegBias.z;
    if (MPU_ERR_CHECK(mpu->setGyroOffset(mpu, gyroRegBias))) return mpu->err;
    return mpu->err;
}

#if defined CONFIG_MPU6500
// Production Self-Test table for MPU6500 based models,
// used in accel and gyro self-test code below.
static const uint16_t kSelfTestTable[256] = {
    2620,  2646,  2672,  2699,  2726,  2753,  2781,  2808,   // 7
    2837,  2865,  2894,  2923,  2952,  2981,  3011,  3041,   // 15
    3072,  3102,  3133,  3165,  3196,  3228,  3261,  3293,   // 23
    3326,  3359,  3393,  3427,  3461,  3496,  3531,  3566,   // 31
    3602,  3638,  3674,  3711,  3748,  3786,  3823,  3862,   // 39
    3900,  3939,  3979,  4019,  4059,  4099,  4140,  4182,   // 47
    4224,  4266,  4308,  4352,  4395,  4439,  4483,  4528,   // 55
    4574,  4619,  4665,  4712,  4759,  4807,  4855,  4903,   // 63
    4953,  5002,  5052,  5103,  5154,  5205,  5257,  5310,   // 71
    5363,  5417,  5471,  5525,  5581,  5636,  5693,  5750,   // 79
    5807,  5865,  5924,  5983,  6043,  6104,  6165,  6226,   // 87
    6289,  6351,  6415,  6479,  6544,  6609,  6675,  6742,   // 95
    6810,  6878,  6946,  7016,  7086,  7157,  7229,  7301,   // 103
    7374,  7448,  7522,  7597,  7673,  7750,  7828,  7906,   // 111
    7985,  8065,  8145,  8227,  8309,  8392,  8476,  8561,   // 119
    8647,  8733,  8820,  8909,  8998,  9088,  9178,  9270,   //
    9363,  9457,  9551,  9647,  9743,  9841,  9939,  10038,  //
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,  //
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,  //
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,  //
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,  //
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,  //
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,  //
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,  //
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,  //
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,  //
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,  //
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,  //
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,  //
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,  //
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,  //
    30903, 31212, 31524, 31839, 32157, 32479, 32804, 33132   //
};
#endif

/**
 * @brief Accel Self-test.
 * @param result self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 16G format for MPU6050 and 2G for MPU6500 based models.
 * */
static esp_err_t accelSelfTest(struct mpu *mpu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result)
{
#if defined CONFIG_MPU6050
    const accel_fs_t kAccelFS = ACCEL_FS_16G;
    // Criteria A: must be within 14% variation
    const float kMaxVariation = .14f;
    // Criteria B: must be between 300 mg and 950 mg
    const float kMinGravity = .3f, kMaxGravity = .95f;

#elif defined CONFIG_MPU6500
    const accel_fs_t kAccelFS = ACCEL_FS_2G;
    // Criteria A: must be within 50% variation
    const float kMaxVariation = .5f;
    // Criteria B: must be between 255 mg and 675 mg
    const float kMinGravity = .225f, kMaxGravity = .675f;
    // Criteria C: 500 mg for accel
    const float kMaxGravityOffset = .5f;
#endif

    /* Convert biases */
    float_axes_t regularBiasGravity  = accelGravity_raw(regularBias, kAccelFS);
    float_axes_t selfTestBiasGravity = accelGravity_raw(selfTestBias, kAccelFS);
    WK_DEBUGD(ST_TAG, "EMPTY, regularBias: %+d %+d %+d | regularBiasGravity: %+.2f %+.2f %+.2f\n", regularBias->data.x,
                regularBias->data.y, regularBias->data.z, regularBiasGravity.data.x, regularBiasGravity.data.y, regularBiasGravity.data.z);
    WK_DEBUGD(ST_TAG, "EMPTY, selfTestBias: %+d %+d %+d | selfTestBiasGravity: %+.2f %+.2f %+.2f\n", selfTestBias->data.x,
                selfTestBias->data.y, selfTestBias->data.z, selfTestBiasGravity.data.x, selfTestBiasGravity.data.y, selfTestBiasGravity.data.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, SELF_TEST_X, 4, mpu->buffer))) return mpu->err;
    shiftCode[0] = ((mpu->buffer[0] & 0xE0) >> 3) | ((mpu->buffer[3] & 0x30) >> 4);
    shiftCode[1] = ((mpu->buffer[1] & 0xE0) >> 3) | ((mpu->buffer[3] & 0x0C) >> 2);
    shiftCode[2] = ((mpu->buffer[2] & 0xE0) >> 3) | (mpu->buffer[3] & 0x03);

#elif defined CONFIG_MPU6500
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, SELF_TEST_X_ACCEL, 3, shiftCode))) return mpu->err;
#endif
    WK_DEBUGD(ST_TAG, "EMPTY, shiftCode: %+d %+d %+d\n", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            // Equivalent to.. shiftProduction[i] = 0.34f * powf(0.92f/0.34f, (shiftCode[i]-1)
            // / 30.f)
            shiftProduction[i] = 0.34f;
            while (--shiftCode[i]) shiftProduction[i] *= 1.034f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= accelSensitivity(ACCEL_FS_2G);
#endif
        }
    }
    WK_DEBUGD(ST_TAG, "EMPTY, shiftProduction: %+.2f %+.2f %+.2f\n", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasGravity.xyz[i] - regularBiasGravity.xyz[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinGravity || shiftResponse[i] > kMaxGravity) {
            *result |= 1 << i;
        }
// Criteria C
#if defined CONFIG_MPU6050
            // no criteria C
#elif defined CONFIG_MPU6500
        if (fabs(regularBiasGravity.xyz[i] > kMaxGravityOffset)) *result |= 1 << i;
#endif
    }
    WK_DEBUGD(ST_TAG, "EMPTY, shiftResponse: %+.2f %+.2f %+.2f\n", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    WK_DEBUGD(ST_TAG, "EMPTY, shiftVariation: %+.2f %+.2f %+.2f\n", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    WK_DEBUGD(ST_TAG, "Accel self-test: [X=%s] [Y=%s] [Z=%s]\n", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
    return mpu->err;
}

/**
 * @brief Gyro Self-test.
 * @param result Self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 250DPS format for both MPU6050 and MPU6500 based models.
 * */
static esp_err_t gyroSelfTest(struct mpu *mpu, raw_axes_t* regularBias, raw_axes_t* selfTestBias, uint8_t* result)
{
    const gyro_fs_t kGyroFS = GYRO_FS_250DPS;

#if defined CONFIG_MPU6050
    // Criteria A: must not exceed +14% variation
    const float kMaxVariation = .14f;
    // Criteria B: must be between 10 dps and 105 dps
    const float kMinDPS = 10.f, kMaxDPS = 105.f;

#elif defined CONFIG_MPU6500
    // Criteria A: must be within 50% variation
    const float kMaxVariation = .5f;
    // Criteria B: must be between 20 dps and 60 dps
    const float kMinDPS = 20.f, kMaxDPS = 60.f;
#endif

    /* Convert biases */
    float_axes_t regularBiasDPS  = gyroDegPerSec_raw(regularBias, kGyroFS);
    float_axes_t selfTestBiasDPS = gyroDegPerSec_raw(selfTestBias, kGyroFS);
    WK_DEBUGD(ST_TAG, "EMPTY, regularBias: %+d %+d %+d | regularBiasDPS: %+.2f %+.2f %+.2f\n", regularBias->data.x,
                regularBias->data.y, regularBias->data.z, regularBiasDPS.data.x, regularBiasDPS.data.y, regularBiasDPS.data.z);
    WK_DEBUGD(ST_TAG, "EMPTY, selfTestBias: %+d %+d %+d | selfTestBiasDPS: %+.2f %+.2f %+.2f\n", selfTestBias->data.x,
                selfTestBias->data.y, selfTestBias->data.z, selfTestBiasDPS.data.x, selfTestBiasDPS.data.y, selfTestBiasDPS.data.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(readBytes(SELF_TEST_X, 3, mpu->buffer))) return mpu->err;
    shiftCode[0] = mpu->buffer[0] & 0x1F;
    shiftCode[1] = mpu->buffer[1] & 0x1F;
    shiftCode[2] = mpu->buffer[2] & 0x1F;

#elif defined CONFIG_MPU6500
    if (MPU_ERR_CHECK(mpu->readBytes(mpu, SELF_TEST_X_GYRO, 3, shiftCode))) return mpu->err;
#endif
    WK_DEBUGD(ST_TAG, "EMPTY, shiftCode: %+d %+d %+d\n", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            shiftProduction[i] = 3275.f / gyroSensitivity(kGyroFS);  // should yield 25
            while (--shiftCode[i]) shiftProduction[i] *= 1.046f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= gyroSensitivity(kGyroFS);
#endif
        }
    }
    WK_DEBUGD(ST_TAG, "EMPTY, shiftProduction: %+.2f %+.2f %+.2f\n", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasDPS.xyz[i] - regularBiasDPS.xyz[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinDPS || shiftResponse[i] > kMaxDPS) {
            *result |= 1 << i;
        }
    }
    WK_DEBUGD(ST_TAG, "EMPTY, shiftResponse: %+.2f %+.2f %+.2f\n", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    WK_DEBUGD(ST_TAG, "EMPTY, shiftVariation: %+.2f %+.2f %+.2f\n", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    WK_DEBUGD(ST_TAG, "Gyro self-test: [X=%s] [Y=%s] [Z=%s]\n", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
    return mpu->err;
}

/**
 * @brief Compute the Biases in regular mode and self-test mode.
 * @attention When calculating the biases the mpu must remain as horizontal as possible (0 degrees), facing up.
 * This algorithm takes about ~400ms to compute offsets.
 * */
static esp_err_t getBiases(struct mpu *mpu, accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest)
{
    // configurations to compute biases
    const uint16_t kSampleRate      = 1000;
    const dlpf_t kDLPF              = DLPF_188HZ;
    const fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    int32_t accelAvgx = 0, accelAvgy = 0, accelAvgz = 0;
    int32_t gyroAvgx  = 0, gyroAvgy  = 0, gyroAvgz  = 0;
    float_axes_t accelG;   // accel axes in (g) gravity format
    float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    // backup previous configuration
    const uint16_t prevSampleRate      = mpu->getSampleRate(mpu);
    const dlpf_t prevDLPF              = mpu->getDigitalLowPassFilter(mpu);
    const accel_fs_t prevAccelFS       = mpu->getAccelFullScale(mpu);
    const gyro_fs_t prevGyroFS         = mpu->getGyroFullScale(mpu);
    const fifo_config_t prevFIFOConfig = mpu->getFIFOConfig(mpu);
    const bool prevFIFOState           = mpu->getFIFOEnabled(mpu);

    // setup
    if (MPU_ERR_CHECK(mpu->setSampleRate(mpu, kSampleRate))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, kDLPF))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setAccelFullScale(mpu, accelFS))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setGyroFullScale(mpu, gyroFS))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setFIFOConfig(mpu, kFIFOConfig))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setFIFOEnabled(mpu, true))) return mpu->err;
    if (selftest) {
        if (MPU_ERR_CHECK(mpu->writeBits(mpu, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, 3, 0x7))) {
            return mpu->err;
        }
        if (MPU_ERR_CHECK(mpu->writeBits(mpu, GYRO_CONFIG, GCONFIG_XG_ST_BIT, 3, 0x7))) {
            return mpu->err;
        }
    }
    // wait for 200ms for sensors to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    /*
    //fifo doesn't work well
    // fill FIFO for 100ms
    if (MPU_ERR_CHECK(mpu->resetFIFO(mpu))) return mpu->err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //if (MPU_ERR_CHECK(mpu->setFIFOConfig(mpu, FIFO_CFG_NONE))) return mpu->err;
    // get FIFO count
    uint16_t fifoCount          = mpu->getFIFOCount(mpu);
    const    size_t kPacketSize = 12;
    while (fifoCount == 0) {
        //WK_DEBUGE(ERROR_TAG, "FIFO count = 0, waitting for 100ms\n");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        fifoCount = mpu->getFIFOCount(mpu);
    }
    if (MPU_ERR_CHECK(mpu->lastError(mpu))) return mpu->err;
    const int packetCount = fifoCount / kPacketSize;
    // read overrun bytes, if any
    const int overrunCount = fifoCount - (packetCount * kPacketSize);
    uint8_t buffer[kPacketSize];
    memset(buffer, 0, kPacketSize);
    if (overrunCount > 0) {
        if (MPU_ERR_CHECK(mpu->readFIFO(mpu, overrunCount, buffer))) return mpu->err;
    }
    // fetch data and add up
    for (int i = 0; i < packetCount; i++) {
        if (MPU_ERR_CHECK(mpu->readFIFO(mpu, kPacketSize, buffer))) {
            WK_DEBUGE(ERROR_TAG, "getBiases read FIFO done size: %d\n", kPacketSize);
            return mpu->err;
        }
        // retrieve data
        raw_axes_t accelCur, gyroCur;
        accelCur.data.x = (buffer[0] << 8) | buffer[1];
        accelCur.data.y = (buffer[2] << 8) | buffer[3];
        accelCur.data.z = (buffer[4] << 8) | buffer[5];
        gyroCur.data.x  = (buffer[6] << 8) | buffer[7];
        gyroCur.data.y  = (buffer[8] << 8) | buffer[9];
        gyroCur.data.z  = (buffer[10] << 8) | buffer[11];
        // add up
        accelAvgx += accelCur.data.x;
        accelAvgy += accelCur.data.y;
        accelAvgz += accelCur.data.z;
        gyroAvgx += gyroCur.data.x;
        gyroAvgy += gyroCur.data.y;
        gyroAvgz += gyroCur.data.z;
    }
    */
    raw_axes_t accelRaw;   // x, y, z axes as int16
    raw_axes_t gyroRaw;    // x, y, z axes as int16
    const int packetCount = 10;

    for (int i = 0; i < packetCount; i++) {
        vTaskDelay(4 / portTICK_PERIOD_MS);
        mpu->acceleration(mpu, &accelRaw);  // fetch raw data from the registers
        mpu->rotation(mpu, &gyroRaw);       // fetch raw data from the registers
        // add up
        accelAvgx += accelRaw.data.x;
        accelAvgy += accelRaw.data.y;
        accelAvgz += accelRaw.data.z;
        gyroAvgx += gyroRaw.data.x;
        gyroAvgy += gyroRaw.data.y;
        gyroAvgz += gyroRaw.data.z;

        float_axes_t accelG1  = accelGravity_raw(&accelRaw, accelFS);
        float_axes_t gyroDPS1 = gyroDegPerSec_raw(&gyroRaw, gyroFS);
        WK_DEBUGD(ST_TAG, "[sample]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS1.xyz[0], gyroDPS1.xyz[1], gyroDPS1.xyz[2]);
        WK_DEBUGD(ST_TAG, "[sample]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG1.data.x, accelG1.data.y, accelG1.data.z);
    }
    raw_axes_t accelAvg;   // x, y, z axes as int16
    raw_axes_t gyroAvg;    // x, y, z axes as int16
    // calculate average
    accelAvg.data.x = accelAvgx / packetCount;
    accelAvg.data.y = accelAvgy / packetCount;
    accelAvg.data.z = accelAvgz / packetCount;
    gyroAvg.data.x = gyroAvgx / packetCount;
    gyroAvg.data.y = gyroAvgy / packetCount;
    gyroAvg.data.z = gyroAvgz / packetCount;

    accelG  = accelGravity_raw(&accelAvg, accelFS);
    gyroDPS = gyroDegPerSec_raw(&gyroAvg, gyroFS);
    WK_DEBUGD(ST_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    WK_DEBUGD(ST_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    // remove gravity from Accel Z axis
    const uint16_t gravityLSB = INT16_MAX >> (accelFS + 1);
    accelAvg.data.z -= gravityLSB;

    accelG  = accelGravity_raw(&accelAvg, accelFS);
    gyroDPS = gyroDegPerSec_raw(&gyroAvg, gyroFS);
    WK_DEBUGD(ST_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    WK_DEBUGD(ST_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    // save biases
    for (int i = 0; i < 3; i++) {
        (*accelBias).xyz[i] = (int16_t) accelAvg.xyz[i];
        (*gyroBias).xyz[i]  = (int16_t) gyroAvg.xyz[i];
    }
    // set back previous configs
    if (MPU_ERR_CHECK(mpu->setSampleRate(mpu, prevSampleRate))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setDigitalLowPassFilter(mpu, prevDLPF))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setAccelFullScale(mpu, prevAccelFS))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setGyroFullScale(mpu, prevGyroFS))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setFIFOConfig(mpu, prevFIFOConfig))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setFIFOEnabled(mpu, prevFIFOState))) return mpu->err;

    // reset self test mode
    if (selftest) {
        if (MPU_ERR_CHECK(mpu->writeBits(mpu, ACCEL_CONFIG, ACONFIG_XA_ST_BIT, 3, 0x0))) {
            return mpu->err;
        }
        if (MPU_ERR_CHECK(mpu->writeBits(mpu, GYRO_CONFIG, GCONFIG_XG_ST_BIT, 3, 0x0))) {
            return mpu->err;
        }
    }

    return mpu->err;
}

/**
 * @brief Set Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the mpu must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
static esp_err_t setOffsets(struct mpu *mpu)
{
    raw_axes_t accel;   // x, y, z axes as int16
    raw_axes_t gyro;    // x, y, z axes as int16

    const accel_fs_t prevAccelFS       = mpu->getAccelFullScale(mpu);
    const gyro_fs_t prevGyroFS         = mpu->getGyroFullScale(mpu);

    if (MPU_ERR_CHECK(mpu->setAccelFullScale(mpu, ACCEL_FS_16G))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setGyroFullScale(mpu, GYRO_FS_1000DPS))) return mpu->err;

    if (MPU_ERR_CHECK(mpu->computeOffsets(mpu, &accel, &gyro))) return mpu->err;

    //WK_DEBUGE(ERROR_TAG, "set Bias gyro: %x, %x, %x\n", gyro.data.x, gyro.data.y, gyro.data.z);
    //WK_DEBUGE(ERROR_TAG, "set Bias accel: %x, %x, %x\n", accel.data.x, accel.data.y, accel.data.z);
    if (MPU_ERR_CHECK(mpu->setGyroOffset(mpu, gyro))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setAccelOffset(mpu, accel))) return mpu->err;

    if (MPU_ERR_CHECK(mpu->setAccelFullScale(mpu, prevAccelFS))) return mpu->err;
    if (MPU_ERR_CHECK(mpu->setGyroFullScale(mpu, prevGyroFS))) return mpu->err;

    //gyro = mpu->getGyroOffset(mpu);
    //accel = mpu->getAccelOffset(mpu);
    //WK_DEBUGE(ERROR_TAG, "get Bias gyro: %x, %x, %x\n", gyro.data.x, gyro.data.y, gyro.data.z);
    //WK_DEBUGE(ERROR_TAG, "get Bias accel: %x, %x, %x\n", accel.data.x, accel.data.y, accel.data.z);

    return mpu->err;
}
