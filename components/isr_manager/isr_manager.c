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

#include "isr_manager.h"

uint32_t isr_counter = 0;
uint8_t rx_command_id = 0x00;
TaskHandle_t mpu_isr_handle;
TaskHandle_t nrf24_isr_handle;

isr_manager_t mpu_isr_manager = {
    .mpu_isr_status = DATA_NOT_READY,
    .mpu_gyro_data_status = DATA_NOT_READY,
    .mpu_accel_data_status = DATA_NOT_READY,
    .mpu_mag_data_status = DATA_NOT_READY
};

#ifdef MPU_INT_ENABLE
void mpu_dmp_isr_handler(void* arg)
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

void nrf24_interrupt_handler(void* arg)
{
    // notify task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(nrf24_isr_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    while (uart->rf_status.rxfifo_cnt) {
        uint8_t c = uart->ahb_fifo.rw_byte;
        uart_write_bytes(UART_NUM_0, (char *)&c, 1);  //把接收的数据重新打印出来
    }
}
*/
