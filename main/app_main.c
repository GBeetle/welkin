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

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_master.h"
#include "esp_spi_flash.h"
#include "rom/ets_sys.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "mpu6500_dmp.h"
#include "motion.h"
#include "log_sys.h"
#include "io_define.h"
#include "task_manager.h"
#include "isr_manager.h"
#include "bmp280.h"
#include "nrf24_interface.h"

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

//Main application
void app_main(void)
{
    welkin_log_system_init();
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    init_mpu(&mpu);

    bmp280_params_t bmp280_params;

    CHK_EXIT(bmp280_init_default_params(&bmp280_params));
    CHK_EXIT(bmp280_init(&bmp280_device, &bmp280_params));

    // waitting for bmp280 initialized done
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // bme280 has humidity sensor
    float pressure, temperature, humidity;
    for (int i = 0; i < 1; i++) {
        CHK_EXIT(bmp280_read_float(&bmp280_device, &temperature, &pressure, &humidity));
        WK_DEBUGD(BMP_TAG, "Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        float altitude = pressureToAltitude(pressure);
        WK_DEBUGD(BMP_TAG, "Altitude: %.2f m", altitude);
    }

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (mpu.testConnection(&mpu)) {
        WK_DEBUGE(ERROR_TAG, "Failed to connect to the MPU\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Initialize
    CHK_EXIT(mpu.initialize(&mpu));  // initialize the chip and set initial configurations
    CHK_EXIT(mpu_rw_test(&mpu));

    // test for sensor is good & horizontal
    selftest_t st_result;
    mpu.selfTest(&mpu, &st_result);
    if (st_result != SELF_TEST_PASS) {
        if (st_result == SELF_TEST_GYRO_FAIL)
            WK_DEBUGE(ERROR_TAG, "SELF_TEST_GYRO_FAIL\n");
        else if (st_result == SELF_TEST_ACCEL_FAIL)
            WK_DEBUGE(ERROR_TAG, "SELF_TEST_ACCEL_FAIL\n");
        else
            WK_DEBUGE(ERROR_TAG, "SELT_TEST_FAIL 0x%x\n", (uint8_t)st_result);
        return;
    }

    CHK_EXIT(mpu.setGyroBias(&mpu));

    //CHK_EXIT(dmp_initialize(&mpu));

    /***********************************/
    /*********** NRF24 INIT ************/
    /***********************************/
    rf24_init(&radio);
    //radio.printDetails(&radio);
    CHK_EXIT(radio.begin(&radio));
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    CHK_EXIT(radio.setPALevel(&radio, RF24_PA_LOW, true));  // RF24_PA_MAX is default.
    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    CHK_EXIT(radio.setPayloadSize(&radio, sizeof(float))); // float datatype occupies 4 bytes
    CHK_EXIT(radio.maskIRQ(&radio, false, false, true));
    // set the TX address of the RX node into the TX pipe
    CHK_EXIT(radio.openWritingPipeAddr(&radio, address[radioNumber]));     // always uses pipe 0
    // set the RX address of the TX node into a RX pipe
    CHK_EXIT(radio.openReadingPipeAddr(&radio, 1, address[!radioNumber])); // using pipe 1
    // additional setup specific to the node's role
    CHK_EXIT(radio.startListening(&radio)); // put radio in RX mode
    ESP_LOGI(RF24_TAG, "NRF24 initialization DONE, %p, %p", &radio, &(radio.get_status));

    /***********************************/
    /*********** NRF24 INT *************/
    /***********************************/
    gpio_config_t io_conf;

    io_conf.intr_type    = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = NRF24_GPIO_INPUT_PIN_SEL;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = 1;
    gpio_config(&io_conf);

    gpio_set_intr_type(NRF24_INT, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(NRF24_INT, nrf24_interrupt_handler, (void*) NRF24_INT);

#ifdef MPU_INT_ENABLE
    gpio_config_t mpu_io_conf;

    mpu_io_conf.intr_type    = GPIO_INTR_POSEDGE;
    mpu_io_conf.pin_bit_mask = MPU_GPIO_INPUT_PIN_SEL;
    mpu_io_conf.mode         = GPIO_MODE_INPUT;
    mpu_io_conf.pull_up_en   = 1;

    gpio_config(&mpu_io_conf);
    gpio_set_intr_type(MPU_DMP_INT, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(MPU_DMP_INT, mpu_dmp_isr_handler, (void*) MPU_DMP_INT);
#endif

    xTaskCreate(mpu_get_sensor_data, "mpu_get_sensor_data", 2048, NULL, 2 | portPRIVILEGE_BIT, &mpu_isr_handle);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(nrf24_interrupt_func, "nrf24 interrupt", 2048, NULL, 2 | portPRIVILEGE_BIT, &nrf24_isr_handle);

    /*
    const fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    const size_t kPacketSize        = 12;

    CHK_EXIT(mpu.setFIFOConfig(&mpu, kFIFOConfig));
    CHK_EXIT(mpu.setFIFOEnabled(&mpu, true));
    // wait for 200ms for sensors to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    CHK_EXIT(mpu.resetFIFO(&mpu));
    */

    while (true) {
        /*
        // get FIFO count
        uint16_t fifoCount = mpu.getFIFOCount(&mpu);
        uint8_t buffer[kPacketSize];

        memset(buffer, 0, kPacketSize);
        while (fifoCount < kPacketSize) {
            fifoCount = mpu.getFIFOCount(&mpu);
        }
        WK_DEBUGD(SENSOR_TAG, "fifo_count: %d\n", fifoCount);
        CHK_EXIT(mpu.lastError(&mpu));
        const int packetCount = fifoCount / kPacketSize;
        for (int i = 0; i < packetCount; i++) {
            if (MPU_ERR_CHECK(mpu.readFIFO(&mpu, kPacketSize, buffer))) {
                WK_DEBUGE(ERROR_TAG, "getBiases read FIFO done size: %d\n", kPacketSize);
                return;
            }
            raw_axes_t accelCur, gyroCur;
            // retrieve data
            accelCur.data.x = (buffer[0] << 8) | buffer[1];
            accelCur.data.y = (buffer[2] << 8) | buffer[3];
            accelCur.data.z = (buffer[4] << 8) | buffer[5];
            gyroCur.data.x  = (buffer[6] << 8) | buffer[7];
            gyroCur.data.y  = (buffer[8] << 8) | buffer[9];
            gyroCur.data.z  = (buffer[10] << 8) | buffer[11];

            float_axes_t accelG  = accelGravity_raw(&accelCur, accel_fs);
            float_axes_t gyroDPS = gyroDegPerSec_raw(&gyroCur, gyro_fs);
            WK_DEBUGD(SENSOR_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (??/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
            WK_DEBUGD(SENSOR_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);
        }
        */
#ifdef MPU_DMP
        // Reading sensor data
        raw_axes_t accelRaw;   // x, y, z axes as int16
        raw_axes_t gyroRaw;    // x, y, z axes as int16
        raw_axes_t magRaw;     // x, y, z axes as int16
        float_axes_t accelG;   // accel axes in (g) gravity format
        float_axes_t gyroDPS;  // gyro axes in (DPS) ??/s format
        float_axes_t magDPS;   // gyro axes in (Gauss) format

        unsigned long sensor_timestamp;
        short sensors;
        unsigned char more;
        long quat[4];
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */

        memset(&accelRaw, 0, 3 * sizeof(short));
        memset(&gyroRaw, 0, 3 * sizeof(short));
        memset(quat, 0, 4 * sizeof(long));

        if (dmp_read_fifo(&mpu, gyroRaw.xyz, accelRaw.xyz, quat, &sensor_timestamp, &sensors, &more)) {
            WK_DEBUGE(ERROR_TAG, "dmp read fifo error\n");
            //memset(quat, 0, 4 * sizeof(long));
        }
#ifdef CONFIG_ANOTIC_DEBUG
        //memset(&gyroRaw, 0, 3 * sizeof(short));
        anotc_init_data(send_buffer, 4, sizeof(uint16_t), quat[0] >> 16, sizeof(uint16_t), quat[1] >> 16,
            sizeof(uint16_t), quat[2] >> 16, sizeof(uint16_t), quat[3] >> 16, sizeof(uint8_t), 0x01);

        uart_write_bytes(UART_NUM_0, (const uint8_t *)send_buffer, send_buffer[3] + 6);
#else

        accelG  = accelGravity_raw(&accelRaw, accel_fs);
        gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
        magDPS  = magGauss_raw(&magRaw, lis3mdl_scale_12_Gs);

        // Debug
        WK_DEBUGE(ERROR_TAG, "[gy]: [%+7.2f %+7.2f %+7.2f ] (??/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
        WK_DEBUGE(ERROR_TAG, "[acc]: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
        WK_DEBUGE(ERROR_TAG, "[m]: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);

        WK_DEBUGE(ERROR_TAG, "quat[0]:%ld quat[1]:%ld quat[2]:%ld quat[3]:%ld\n", quat[0], quat[1], quat[2], quat[3]);
#endif
#endif
    }
}
