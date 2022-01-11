#ifndef _IO_DEFINE__
#define _IO_DEFINE__

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
// MPU SPI IO 11 13 12 10
#define MPU_FSPI_MOSI 11
#define MPU_FSPI_MISO 13
#define MPU_FSPI_MPU_SCLK 12
#define MPU_FSPI_CS 10
// up to 1MHz for all registers, and 20MHz for sensor data registers only
#define MPU_SPI_CLOCK_SPEED SPI_MASTER_FREQ_20M

// MPU I2C IO
#define MPU_SDA 11
#define MPU_SCL 12
#define MPU_I2C_CLOCK_SPEED 400000  // range from 100 KHz ~ 400Hz

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

// MPU CONFIG
#define MPU_INT_ENABLE
//#define MPU_DMP
#define SOFT_IMU_UPDATE
#define MPU_DMP_INT 14
#define MPU_GPIO_INPUT_PIN_SEL  ((1ULL<<MPU_DMP_INT))




#endif /* end of include guard: _IO_DEFINE__ */
