#ifndef _LOG_SYS__
#define _LOG_SYS__

#include "esp_log.h"

#define WK_DEBUGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define WK_DEBUGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define WK_DEBUGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define WK_DEBUGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define WK_DEBUGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)

extern const char* SENSOR_TAG;
extern const char* ERROR_TAG;
extern const char* ST_TAG;
extern const char* CHK_TAG;

void welkin_log_system_init(void);

#endif /* end of include guard: _LOG_SYS__ */
