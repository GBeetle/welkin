/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
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

//Main application
void app_main(void)
{
    //fflush(stdout);
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
    /*
    // release the pre registered UART handler/subroutine
	ESP_ERROR_CHECK(uart_isr_free(UART_NUM_0));
	// register new UART subroutine
    uart_intr_config_t uart_isr_config = {
        .intr_enable_mask         = UART_INTR_RXFIFO_FULL,
        .rx_timeout_thresh        = 120,
        .txfifo_empty_intr_thresh = 10,
        .rxfifo_full_thresh       = 10,
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM_0, &uart_isr_config));
	ESP_ERROR_CHECK(uart_isr_register(UART_NUM_0, uart_intr_handle, NULL, 0, NULL));
	// enable RX interrupt
	ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_0));
    */

#if defined CONFIG_MPU_SPI
    spi_device_handle_t mpu_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    init_spi(&fspi, SPI2_HOST);
    // disable SPI DMA in begin
    CHK_EXIT(fspi.begin(&fspi, MPU_FSPI_MOSI, MPU_FSPI_MISO, MPU_FSPI_MPU_SCLK, SPI_MAX_DMA_LEN));
    CHK_EXIT(fspi.addDevice(&fspi, 0, MPU_SPI_CLOCK_SPEED, MPU_FSPI_CS, &mpu_spi_handle));

    init_mpu(&mpu, &fspi, mpu_spi_handle);
#endif

#if defined CONFIG_MPU_I2C
    init_i2c(&i2c0, I2C_NUM_0);
    CHK_EXIT(i2c0.begin(&i2c0, MPU_SDA, MPU_SCL, MPU_I2C_CLOCK_SPEED));
    mpu_addr_handle_t  MPU_DEFAULT_I2CADDRESS = MPU_I2CADDRESS_AD0_LOW;

    init_mpu(&mpu, &i2c0, MPU_DEFAULT_I2CADDRESS);
#endif


    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (mpu.testConnection(&mpu)) {
        WK_DEBUGE(ERROR_TAG, "Failed to connect to the MPU\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    //WK_DEBUGI(TAG, "MPU connection successful!");

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

    //printf("Start to set Offset\n");
    //CHK_EXIT(mpu.setOffsets(&mpu));
    //raw_axes_t acc;
    //memset(&acc, 0x3f, sizeof(acc));
    //mpu.setGyroOffset(&mpu, acc);

    //CHK_EXIT(dmp_initialize(&mpu));

#ifdef MPU_INT_ENABLE
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = MPU_GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(MPU_DMP_INT, GPIO_INTR_ANYEDGE);
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(MPU_DMP_INT, mpu_dmp_isr_handler, (void*) MPU_DMP_INT);
#endif

    xTaskCreate(mpu_get_sensor_data, "mpu_get_sensor_data", 2048, NULL, 2 | portPRIVILEGE_BIT, &mpu_isr_handle);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //vTaskStartScheduler();

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
            WK_DEBUGD(SENSOR_TAG, "[Bias]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
            WK_DEBUGD(SENSOR_TAG, "[Bias]accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);
        }
        */
#ifdef MPU_DMP
        // Reading sensor data
        raw_axes_t accelRaw;   // x, y, z axes as int16
        raw_axes_t gyroRaw;    // x, y, z axes as int16
        raw_axes_t magRaw;     // x, y, z axes as int16
        float_axes_t accelG;   // accel axes in (g) gravity format
        float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
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
        WK_DEBUGE(ERROR_TAG, "[gy]: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
        WK_DEBUGE(ERROR_TAG, "[acc]: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
        WK_DEBUGE(ERROR_TAG, "[m]: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);

        WK_DEBUGE(ERROR_TAG, "quat[0]:%ld quat[1]:%ld quat[2]:%ld quat[3]:%ld\n", quat[0], quat[1], quat[2], quat[3]);
#endif
#endif
    }
}
