#ifndef _LOG_SYS__
#define _LOG_SYS__

#include "esp_log.h"

extern const char* SENSOR_TAG;
extern const char* ERROR_TAG;
extern const char* ST_TAG;

void welkin_log_system_init(void);

#endif /* end of include guard: _LOG_SYS__ */
