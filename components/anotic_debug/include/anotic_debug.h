#ifndef _ANOTIC_DEBUG__
#define _ANOTIC_DEBUG__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>

uint8_t __bswap_8(uint8_t value);
uint16_t float2int16(float value);
int anotc_init_data(uint8_t *send_buffer, uint8_t f_id, uint32_t arg_nums, ...);

#endif /* end of include guard: _ANOTIC_DEBUG__ */
