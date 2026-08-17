/**
    ble_monitor.h : Passive BLE observation interface
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#ifndef BLE_MONITOR_H
#define BLE_MONITOR_H

#include "esp_err.h"

/**
 * @brief Initialize NimBLE and start continuous passive BLE scanning.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t ble_monitor_init( void );

#endif
