#include "log_sys.h"

const char* SENSOR_TAG = "[SENSOR_CHECK]";
const char* ERROR_TAG = "[ERROR]";

void welkin_log_system_init()
{
    esp_log_level_set(SENSOR_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(ERROR_TAG, ESP_LOG_ERROR);
}

