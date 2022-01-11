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
