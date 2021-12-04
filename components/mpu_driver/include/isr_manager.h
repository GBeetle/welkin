#ifndef ISR_MANAGER_H__
#define ISR_MANAGER_H__

typedef enum { DATA_NOT_READY = 0, DATA_READY = 1 } data_status;

typedef struct isr_manager {
    data_status mpu_isr_status : 1;
    data_status mpu_gyro_data_status : 1;
    data_status mpu_accel_data_status : 1;
    data_status mpu_mag_data_status : 1;
    data_status reserved : 4;
} isr_manager;

#endif


