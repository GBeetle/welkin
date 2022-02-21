/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "log_sys.h"

const char* SENSOR_TAG = "[SENSOR_CHECK]";
const char* ERROR_TAG = "[ERROR]";
const char* ST_TAG = "[SELF_TEST]";
const char* CHK_TAG = "[CHK]";
const char* BMP_TAG = "[BMP]";
const char* RF24_TAG = "[RF24]";

void welkin_log_system_init()
{
    esp_log_level_set(SENSOR_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(ERROR_TAG, ESP_LOG_ERROR);
    esp_log_level_set(ST_TAG, ESP_LOG_ERROR);
    esp_log_level_set(CHK_TAG, ESP_LOG_ERROR);
    esp_log_level_set(RF24_TAG, ESP_LOG_DEBUG);
}

