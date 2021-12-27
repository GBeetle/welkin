#include "anotic_debug.h"

uint8_t __bswap_8(uint8_t value)
{
    return (value>>4) | (value<<4);
}

uint16_t float2int16(float value)
{
    uint32_t num = value * 100;
    return num & 0xffffffff;
}

int anotc_init_data(uint8_t *send_buffer, uint8_t command_id, uint32_t arg_nums, ...)
{
    va_list args;
    uint32_t idx = 0;
    uint8_t checksum = 0;
    uint8_t addcheck = 0;

    va_start(args, arg_nums);
    send_buffer[idx++] = 0xAA;
    send_buffer[idx++] = 0xFF;
    send_buffer[idx++] = command_id;
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

