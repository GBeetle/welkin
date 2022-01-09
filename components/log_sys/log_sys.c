#include "log_sys.h"

const char* SENSOR_TAG = "[SENSOR_CHECK]";
const char* ERROR_TAG = "[ERROR]";
const char* ST_TAG = "[SELF_TEST]";
const char* CHK_TAG = "[CHK]";

void welkin_log_system_init()
{
    esp_log_level_set(SENSOR_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(ERROR_TAG, ESP_LOG_ERROR);
    esp_log_level_set(ST_TAG, ESP_LOG_ERROR);
    esp_log_level_set(CHK_TAG, ESP_LOG_ERROR);
}

