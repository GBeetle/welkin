#ifndef _TASK_MANAGER__
#define _TASK_MANAGER__

#include "isr_manager.h"
#include "mpu_driver.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "hal/uart_ll.h"
#include "anotic_debug.h"

void mpu_get_sensor_data(void* arg);
void uart_rx_task(void *arg);

extern struct mpu mpu;

#endif /* end of include guard: _TASK_MANAGER__ */
