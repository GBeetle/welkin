#ifndef _ISR_MANAGER__
#define _ISR_MANAGER__

#include <inttypes.h>
#include <esp_attr.h>
#include "mpu_driver.h"
#include "io_define.h"

typedef enum { DATA_NOT_READY = 0, DATA_READY = 1 } data_status;

typedef struct isr_manager {
    data_status mpu_isr_status : 1;
    data_status mpu_gyro_data_status : 1;
    data_status mpu_accel_data_status : 1;
    data_status mpu_mag_data_status : 1;
    data_status reserved : 4;
} isr_manager_t;

void mpu_dmp_isr_handler(void* arg);

extern uint32_t isr_counter;
extern uint8_t rx_command_id;
extern TaskHandle_t mpu_isr_handle;
extern isr_manager_t mpu_isr_manager;

#endif /* end of include guard: _TASK_MANAGER__ */
