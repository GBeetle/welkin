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
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "hal/uart_ll.h"

#include "mpu_driver.h"
#include "mpu6500_dmp.h"
#include "motion.h"
#include "anotic_debug.h"
#include "log_sys.h"

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
// 11 13 12 10
#define FSPI_MOSI 11
#define FSPI_MISO 13
#define FSPI_SCLK 12
#define FSPI_CS 10
// up to 1MHz for all registers, and 20MHz for sensor data registers only
#define SPI_CLOCK_SPEED SPI_MASTER_FREQ_20M

#define MPU_INT_ENABLE
//#define MPU_DMP
#define SOFT_IMU_UPDATE

#define SDA 11
#define SCL 12
#define I2C_CLOCK_SPEED 400000  // range from 100 KHz ~ 400Hz

#define MPU_DMP_INT 14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<MPU_DMP_INT))

struct mpu mpu;  // create a default MPU object

TaskHandle_t mpu_isr_handle;
uint32_t isr_counter = 0;
static uint8_t rx_command_id = 0x00;

#ifdef MPU_INT_ENABLE
static void IRAM_ATTR mpu_dmp_isr_handler(void* arg)
{
    mpu_isr_manager.mpu_isr_status = DATA_READY;
    isr_counter++;
    //ets_printf("isr before:[%s] stat:[%d] prid:[%d]\n", pcTaskGetTaskName(mpu_isr_handle), eTaskGetState(mpu_isr_handle), uxTaskPriorityGetFromISR(mpu_isr_handle));
    //xTaskResumeFromISR(mpu_isr_handle);
    //ets_printf("isr after:[%s] stat:[%d]\n", pcTaskGetTaskName(mpu_isr_handle), eTaskGetState(mpu_isr_handle));
    /* Notify the task that the transmission is complete. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_isr_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

static void mpu_get_sensor_data(void* arg)
{
    raw_axes_t accelRaw;   // x, y, z axes as int16
    raw_axes_t gyroRaw;    // x, y, z axes as int16
    raw_axes_t magRaw;     // x, y, z axes as int16
    float_axes_t accelG;   // accel axes in (g) gravity format
    float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    float_axes_t magDPS;   // gyro axes in (Gauss) format

    while (1) {
#ifndef CONFIG_ANOTIC_DEBUG
        ets_printf("[SAMPLE] %u\n", isr_counter);
#endif
        if(mpu_isr_manager.mpu_isr_status) {
            // Read
            mpu.acceleration(&mpu, &accelRaw);  // fetch raw data from the registers
            mpu.rotation(&mpu, &gyroRaw);       // fetch raw data from the registers
            mpu.heading(&mpu, &magRaw);

            // Convert
            accelG  = accelGravity_raw(&accelRaw, accel_fs);
            gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
            magDPS  = magGauss_raw(&magRaw, lis3mdl_scale_12_Gs);
#ifdef SOFT_IMU_UPDATE
            motion_state_t state;
            //imuUpdate(accelG, gyroDPS, &state, 1.0 / 250);
            imuUpdateAttitude(accelG, gyroDPS, magDPS, &state, 1.0 / 250);
#endif

#if defined CONFIG_ANOTIC_DEBUG
            uint8_t send_buffer[100];
            switch (rx_command_id) {
                case 0x00:
                    break;
                case 0x01:
                    anotc_init_data(send_buffer, 0x01, 6, sizeof(uint16_t), accelRaw.x, sizeof(uint16_t), accelRaw.y, sizeof(uint16_t), accelRaw.z,
                        sizeof(uint16_t), gyroRaw.x, sizeof(uint16_t), gyroRaw.y, sizeof(uint16_t), gyroRaw.z, sizeof(uint8_t), 0x00);
                    break;
                case 0x02:
                    anotc_init_data(send_buffer, 0x02, 6, sizeof(uint16_t), magRaw.x, sizeof(uint16_t), magRaw.y, sizeof(uint16_t), magRaw.z,
                        sizeof(uint32_t), 0x00, sizeof(uint16_t), 0x00, sizeof(uint8_t), 0x00, sizeof(uint8_t), 0x00);
                    break;
                case 0x03:
                    anotc_init_data(send_buffer, 0x03, 3, sizeof(uint16_t), float2int16(state.attitude.roll), sizeof(uint16_t), float2int16(state.attitude.pitch),
                        sizeof(uint16_t), float2int16(state.attitude.yaw), sizeof(uint8_t), 0x01);
                    break;
                default:
                    WK_DEBUGE(ERROR_TAG, "wrong command id from uart: %02x\n", rx_command_id);
                    rx_command_id = 0x00;
            }
            if (rx_command_id >= 0x01 && rx_command_id <= 0x03)
                uart_write_bytes(UART_NUM_0, (const uint8_t *)send_buffer, send_buffer[3] + 6);
#else
            WK_DEBUGD(SENSOR_TAG, "roll:%f pitch:%f yaw:%f\n", state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
#endif

#ifndef CONFIG_ANOTIC_DEBUG
            // Debug
            WK_DEBUGD(SENSOR_TAG, "gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
            WK_DEBUGD(SENSOR_TAG, "accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
            WK_DEBUGD(SENSOR_TAG, "mag: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);
#endif
            mpu_isr_manager.mpu_isr_status = DATA_NOT_READY;
        }
        //vTaskSuspend(mpu_isr_handle);
        /* Wait to be notified that the transmission is complete.  Note
        the first parameter is pdTRUE, which has the effect of clearing
        the task's notification value back to 0, making the notification
        value act like a binary (rather than a counting) semaphore.  */
        uint32_t ul_notification_value;
        const TickType_t max_block_time = pdMS_TO_TICKS( 200 );
        ul_notification_value = ulTaskNotifyTake(pdTRUE, max_block_time );

        if( ul_notification_value == 1 ) {
            /* The transmission ended as expected. */
        }
        else {
            /* The call to ulTaskNotifyTake() timed out. */
        }
    }
}

static void test(void* arg)
{
    while (1) {
        //WK_DEBUGE(ERROR_TAG, "[test idle task start] prio:[%d]\n", uxTaskPriorityGet(NULL));
        /*
        char task[200];
        vTaskList(task);
        WK_DEBUGE(ERROR_TAG, "%s\n",task);
        */
        vTaskDelay(100);
        //WK_DEBUGE(ERROR_TAG, "[test idle task end]\n");
    }
}

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
/*
uint8_t rxbuf[256];
static void uart_intr_handle(void *param)
{
    ets_printf("[UART_RECEIVE ISR]\n");
    volatile uart_dev_t *uart = &UART0;
    uart->int_clr.rxfifo_full = 1;
    uart->int_clr.frm_err = 1;
    uart->int_clr.rxfifo_tout = 1;
    while (uart->status.rxfifo_cnt) {
        uint8_t c = uart->ahb_fifo.rw_byte;
        uart_write_bytes(UART_NUM_0, (char *)&c, 1);  //把接收的数据重新打印出来
    }
}
*/

// receive frame format 0xAA_ID_AA
static void uart_rx_task(void *arg)
{
    const int RX_BUF_SIZE = 1024;
    uint8_t* data = (uint8_t*)malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes <= 0)
            continue;
        if (rxBytes != 3 || data[0] != 0xaa || data[2] != 0xaa)
            rx_command_id = 0xff;
        else
            rx_command_id = data[1];
        /*
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            WK_DEBUGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        */
    }
    free(data);
}

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
    CHK_EXIT(fspi.begin(&fspi, FSPI_MOSI, FSPI_MISO, FSPI_SCLK, SPI_MAX_DMA_LEN));
    CHK_EXIT(fspi.addDevice(&fspi, 0, SPI_CLOCK_SPEED, FSPI_CS, &mpu_spi_handle));

    init_mpu(&mpu, &fspi, mpu_spi_handle);
#endif

#if defined CONFIG_MPU_I2C
    init_i2c(&i2c0, I2C_NUM_0);
    CHK_EXIT(i2c0.begin(&i2c0, SDA, SCL, I2C_CLOCK_SPEED));
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
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
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
    xTaskCreate(test, "test", 2048, NULL, 1 | portPRIVILEGE_BIT, NULL);
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
