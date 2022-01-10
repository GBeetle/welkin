#include "task_manager.h"

struct mpu mpu;  // create a default MPU object

void mpu_get_sensor_data(void* arg)
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

// receive frame format 0xAA_ID_AA
void uart_rx_task(void *arg)
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
