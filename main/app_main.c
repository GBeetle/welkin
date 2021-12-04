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
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "rom/ets_sys.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"

#include "mpu_driver.h"
#include "mpu6500_dmp.h"
#include "motion.h"

static const char* TAG = "[mwm]";


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
#define DEBUG_LOG
#define MPU_NO_DMP
#define SOFT_IMU_UPDATE

#define SDA 11
#define SCL 12
#define I2C_CLOCK_SPEED 400000  // range from 100 KHz ~ 400Hz

#define MPU_DMP_INT 14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<MPU_DMP_INT))

#ifdef CONFIG_MPU_SPI
struct mpu mpu;  // create a default MPU object
#endif

/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */
typedef struct tskTaskControlBlock 			/* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
	volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
	#endif

	ListItem_t			xStateListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t			*pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	BaseType_t			xCoreID;			/*< Core this task is pinned to */

	#if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
		StackType_t		*pxEndOfStack;		/*< Points to the highest valid address for the stack. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		UBaseType_t		uxCriticalNesting;	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		UBaseType_t		uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		UBaseType_t		uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
		UBaseType_t		uxMutexesHeld;
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		TaskHookFunction_t pxTaskTag;
	#endif

	#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
		void			*pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#if ( configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS )
		TlsDeleteCallbackFunction_t pvThreadLocalStoragePointersDelCallback[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#endif
	#endif

	#if( configGENERATE_RUN_TIME_STATS == 1 )
		uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
	#endif

	#if ( configUSE_NEWLIB_REENTRANT == 1 )
		/* Allocate a Newlib reent structure that is specific to this task.
		Note Newlib support has been included by popular demand, but is not
		used by the FreeRTOS maintainers themselves.  FreeRTOS is not
		responsible for resulting newlib operation.  User must be familiar with
		newlib and must provide system-wide implementations of the necessary
		stubs. Be warned that (at the time of writing) the current newlib design
		implements a system-wide malloc() that must be provided with locks. */
		struct	_reent xNewLib_reent;
	#endif

	#if( configUSE_TASK_NOTIFICATIONS == 1 )
		volatile uint32_t ulNotifiedValue;
		volatile uint8_t ucNotifyState;
	#endif

	/* See the comments in FreeRTOS.h with the definition of
	tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
	#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
		uint8_t	ucStaticallyAllocated; 		/*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
	#endif

	#if( INCLUDE_xTaskAbortDelay == 1 )
		uint8_t ucDelayAborted;
	#endif

	#if( configUSE_POSIX_ERRNO == 1 )
		int iTaskErrno;
	#endif

} tskTCB;

TaskHandle_t mpu_isr_handle;

uint8_t __bswap_8(uint8_t value)
{
    return (value>>4) | (value<<4);
}

uint16_t float2int16(float value)
{
    uint32_t num = value * 100;
    return num & 0xffffffff;
}

int anotc_init_data(uint8_t *send_buffer, uint32_t arg_nums, ...)
{
    va_list args;
    uint32_t idx = 0;
    uint8_t checksum = 0;
    uint8_t addcheck = 0;

    va_start(args, arg_nums);
    send_buffer[idx++] = 0xAA;
    send_buffer[idx++] = 0xFF;
    send_buffer[idx++] = 0x03;
    send_buffer[idx++] = 0x00;

    for (uint32_t i = 0; i < arg_nums; i++) {
        uint32_t size = va_arg(args, uint32_t);

        send_buffer[3] += size;
        switch (size) {
            case 4: {
                uint32_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 4);
                break;
            }
            case 2: {
                uint16_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 2);
                break;
            }
            case 1: {
                uint8_t num = va_arg(args, uint32_t);
                memcpy(send_buffer + idx, &num, 1);
                break;
            }
            default: {
                printf("Wrong format for size: %d\n", size);
                return -1;
            }
        }
        idx += size;
    }

    for (int i = 0; i < send_buffer[3] + 4; i++) {
        checksum += send_buffer[i];
        addcheck += checksum;
    }
    send_buffer[idx++] = checksum;
    send_buffer[idx++] = addcheck;

    va_end(args);
    return 0;
}

uint32_t isr_counter = 0;

#ifdef MPU_INT_ENABLE
static void IRAM_ATTR mpu_dmp_isr_handler(void* arg)
{
    mpu_isr_manager.mpu_isr_status = DATA_READY;
    isr_counter++;
    //xTaskResumeFromISR(mpu_isr_handle);
    //ets_printf("[%s] \n", ((tskTCB *)mpu_isr_handle)->pcTaskName);
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
        ets_printf("[SAMPLE] %u\n", isr_counter);
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
            imuUpdate(accelG, gyroDPS, &state , 1.0 / 250);
#endif

#ifndef DEBUG_LOG
            uint8_t send_buffer[100];
            anotc_init_data(send_buffer, 3, sizeof(uint16_t), float2int16(state.attitude.roll), sizeof(uint16_t), float2int16(state.attitude.pitch),
                sizeof(uint16_t), float2int16(state.attitude.yaw), sizeof(uint8_t), 0x01);

            uart_write_bytes(UART_NUM_0, (const uint8_t *)send_buffer, send_buffer[3] + 6);
#elif defined SOFT_IMU_UPDATE
            //printf("roll:%f pitch:%f yaw:%f\n", state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
#endif

#ifdef DEBUG_LOG
            // Debug
            //printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
            //printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
            //printf("mag: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);
#endif
            mpu_isr_manager.mpu_isr_status = DATA_NOT_READY;
        }
        //vTaskSuspend(mpu_isr_handle);
    }
}

static void test(void* arg)
{
    while (1) {
        printf("[test idle task]\n");
        //vTaskResume(mpu_isr_handle);
        vTaskDelay(100);
    }
}

//Main application
void app_main(void)
{
    //fflush(stdout);
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

#if defined CONFIG_MPU_SPI
    spi_device_handle_t mpu_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    init_spi(&fspi, SPI2_HOST);
    // disable SPI DMA in begin
    ESP_ERROR_CHECK(fspi.begin(&fspi, FSPI_MOSI, FSPI_MISO, FSPI_SCLK, SPI_MAX_DMA_LEN));
    ESP_ERROR_CHECK(fspi.addDevice(&fspi, 0, SPI_CLOCK_SPEED, FSPI_CS, &mpu_spi_handle));

    init_mpu(&mpu, &fspi, mpu_spi_handle);
#endif

#if defined CONFIG_MPU_I2C
    init_i2c(&i2c0, I2C_NUM_0);
    ESP_ERROR_CHECK(i2c0.begin(&i2c0, SDA, SCL, I2C_CLOCK_SPEED));
    mpu_addr_handle_t  MPU_DEFAULT_I2CADDRESS = MPU_I2CADDRESS_AD0_LOW;

    struct mpu mpu;  // create a default MPU object
    init_mpu(&mpu, &i2c0, MPU_DEFAULT_I2CADDRESS);
#endif


    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (mpu.testConnection(&mpu)) {
        printf("Failed to connect to the MPU\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    //ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(mpu.initialize(&mpu));  // initialize the chip and set initial configurations
    // Setup with your configurations
    // ESP_ERROR_CHECK(MPU.setSampleRate(50));  // set sample rate to 50 Hz
    // ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
    // ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

    /*
    selftest_t st_result;
    mpu.selfTest(&mpu, &st_result);
    if (st_result != SELF_TEST_PASS) {
        if (st_result == SELF_TEST_GYRO_FAIL)
            printf("SELF_TEST_GYRO_FAIL\n");
        else if (st_result == SELF_TEST_ACCEL_FAIL)
            printf("SELF_TEST_ACCEL_FAIL\n");
        else
            printf("SELT_TEST_FAIL 0x%x\n", (uint8_t)st_result);
        return;
    }
    */

    unsigned char data[4];
    for (int i = 0; i < 4; i++) {
        data[i] = i + 5;
    }
    //mpu.writeBytes(&mpu, 0x63, 4, data);
    //mpu.readBytes(&mpu, 0x63, 4, data);
    mpu_write_mem(&mpu, 0, 4, data);
    mpu_read_mem(&mpu, 0, 4, data);
    for (int i = 0; i < 4; i++) {
        if (data[i] != i + 5) {
            printf("Read Write Error");
            printf("%d %d %d %d\n", data[0], data[1], data[2], data[3]);
            return;
        }
    }

    //ESP_ERROR_CHECK(dmp_initialize(&mpu));

    //printf("Start to set Offset\n");
    //ESP_ERROR_CHECK(mpu.setOffsets(&mpu));
    //raw_axes_t acc;
    //memset(&acc, 0x3f, sizeof(acc));
    //mpu.setGyroOffset(&mpu, acc);

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

    // Reading sensor data
    raw_axes_t accelRaw;   // x, y, z axes as int16
    raw_axes_t gyroRaw;    // x, y, z axes as int16
    raw_axes_t magRaw;     // x, y, z axes as int16
    float_axes_t accelG;   // accel axes in (g) gravity format
    float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    float_axes_t magDPS;   // gyro axes in (Gauss) format

#ifdef NEVER_DEBUG
    memset(&accelRaw, 0, 3 * sizeof(short));
    memset(&gyroRaw, 0, 3 * sizeof(short));
    accelG  = accelGravity_raw(&accelRaw, accel_fs);
    gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
    printf("[test]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    memset(&accelRaw, 0x80, 3 * sizeof(short));
    memset(&gyroRaw, 0x80, 3 * sizeof(short));
    accelG  = accelGravity_raw(&accelRaw, accel_fs);
    gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
    printf("[test]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    memset(&accelRaw, 0x7f, 3 * sizeof(short));
    memset(&gyroRaw, 0x7f, 3 * sizeof(short));
    accelG  = accelGravity_raw(&accelRaw, accel_fs);
    gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
    printf("[test]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);

    memset(&accelRaw, 0xff, 3 * sizeof(short));
    memset(&gyroRaw, 0xff, 3 * sizeof(short));
    accelG  = accelGravity_raw(&accelRaw, accel_fs);
    gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
    printf("[test]gyro: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
    printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t\n", accelG.data.x, accelG.data.y, accelG.data.z);
#endif

    xTaskCreate(mpu_get_sensor_data, "mpu_get_sensor_data", 2048, 2 | portPRIVILEGE_BIT, 10, &mpu_isr_handle);
    xTaskCreate(test, "test", 2048, NULL, 1 | portPRIVILEGE_BIT, NULL);
    //vTaskStartScheduler();

    while (true) {
        //printf("main while\n");
        //vTaskDelay(100);
        /*
        mpu.readBytes(&mpu, INT_PIN_CONFIG, 1, &dmp_int);
        printf("dmp int config: %02x\n", dmp_int);
        mpu.readBytes(&mpu, INT_ENABLE, 1, &dmp_int);
        printf("dmp int enable: %02x\n", dmp_int);
        mpu.readBytes(&mpu, INT_STATUS, 1, &dmp_int);
        printf("dmp int status: %02x\n", dmp_int);
        */

#ifndef MPU_NO_DMP
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
#ifdef DEBUG_LOG
            printf("dmp read fifo error\n");
#endif
            //memset(quat, 0, 4 * sizeof(long));
        }
        //memset(&gyroRaw, 0, 3 * sizeof(short));
        anotc_init_data(send_buffer, 4, sizeof(uint16_t), quat[0] >> 16, sizeof(uint16_t), quat[1] >> 16,
            sizeof(uint16_t), quat[2] >> 16, sizeof(uint16_t), quat[3] >> 16, sizeof(uint8_t), 0x01);

#ifndef DEBUG_LOG
        uart_write_bytes(UART_NUM_0, (const uint8_t *)send_buffer, send_buffer[3] + 6);
#endif

        accelG  = accelGravity_raw(&accelRaw, accel_fs);
        gyroDPS = gyroDegPerSec_raw(&gyroRaw, gyro_fs);
        magDPS  = magGauss_raw(&magRaw, lis3mdl_scale_12_Gs);

#ifdef DEBUG_LOG
        // Debug
        printf("[gy]: [%+7.2f %+7.2f %+7.2f ] (º/s) \t", gyroDPS.xyz[0], gyroDPS.xyz[1], gyroDPS.xyz[2]);
        printf("[acc]: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.data.x, accelG.data.y, accelG.data.z);
        printf("[m]: [%+6.2f %+6.2f %+6.2f ] (Gauss) \n", magDPS.data.x, magDPS.data.y, magDPS.data.z);

        printf("quat[0]:%ld quat[1]:%ld quat[2]:%ld quat[3]:%ld\n", quat[0], quat[1], quat[2], quat[3]);
#endif
#endif
    }
}
