/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu_dmp_motion_driver.h
 *      @brief      DMP image and interface functions.
 *      @details    All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */
/* To initialize the DMP:
 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
 * 2. Push the gyro and accel orientation matrix to the DMP.
 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
 *    executed unless the corresponding feature is enabled.
 * 4. Call dmp_enable_feature(mask) to enable different features.
 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
 * 6. Call any feature-specific control functions.
 *
 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
 * be called repeatedly to enable and disable the DMP at runtime.
 *
 * The following is a short summary of the features supported in the DMP
 * image provided in inv_mpu_dmp_motion_driver.c:
 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
 * 200Hz. Integrating the gyro data at higher rates reduces numerical
 * errors (compared to integration on the MCU at a lower sampling rate).
 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
 * an event at the four orientations where the screen should rotate.
 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
 * no motion.
 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
**/
#ifndef _MPU6500_DMP_H_
#define _MPU6500_DMP_H_

#include "mpu_driver.h"
#include "ml_math_func.h"
#include "esp_timer.h"

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

int dmp_initialize(struct mpu *mpu);

/* Set up functions. */
int dmp_load_motion_driver_firmware(struct mpu *mpu);
int dmp_set_fifo_rate(struct mpu *mpu, unsigned short rate);
int dmp_get_fifo_rate(unsigned short *rate);
int dmp_enable_feature(struct mpu *mpu, unsigned short mask);
int dmp_get_enabled_features(unsigned short *mask);
int dmp_set_interrupt_mode(struct mpu *mpu, unsigned char mode);
int dmp_set_orientation(struct mpu *mpu, unsigned short orient);
int dmp_set_gyro_bias(struct mpu *mpu, long *bias);
int dmp_set_accel_bias(struct mpu *mpu, long *bias);

/* Tap functions. */
int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char));
int dmp_set_tap_thresh(struct mpu *mpu, unsigned char axis, unsigned short thresh);
int dmp_set_tap_axes(struct mpu *mpu, unsigned char axis);
int dmp_set_tap_count(struct mpu *mpu, unsigned char min_taps);
int dmp_set_tap_time(struct mpu *mpu, unsigned short time);
int dmp_set_tap_time_multi(struct mpu *mpu, unsigned short time);
int dmp_set_shake_reject_thresh(struct mpu *mpu, long sf, unsigned short thresh);
int dmp_set_shake_reject_time(struct mpu *mpu, unsigned short time);
int dmp_set_shake_reject_timeout(struct mpu *mpu, unsigned short time);

/* Android orientation functions. */
int dmp_register_android_orient_cb(void (*func)(unsigned char));

/* LP quaternion functions. */
int dmp_enable_lp_quat(struct mpu *mpu, unsigned char enable);
int dmp_enable_6x_lp_quat(struct mpu *mpu, unsigned char enable);

/* Pedometer functions. */
int dmp_get_pedometer_step_count(struct mpu *mpu, unsigned long *count);
int dmp_set_pedometer_step_count(struct mpu *mpu, unsigned long count);
int dmp_get_pedometer_walk_time(struct mpu *mpu, unsigned long *time);
int dmp_set_pedometer_walk_time(struct mpu *mpu, unsigned long time);

/* DMP gyro calibration functions. */
int dmp_enable_gyro_cal(struct mpu *mpu, unsigned char enable);

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */
int dmp_read_fifo(struct mpu *mpu, short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more);

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(struct mpu *mpu, unsigned short mem_addr, unsigned short length,
        unsigned char *data);

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(struct mpu *mpu, unsigned short mem_addr, unsigned short length,
        unsigned char *data);

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(struct mpu *mpu);

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(struct mpu *mpu, unsigned short length, unsigned char *data,
    unsigned char *more);

int mpu_reset_fifo(struct mpu *mpu);

#endif  /* #ifndef _MPU6500_DMP_H_ */

