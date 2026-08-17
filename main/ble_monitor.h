/**
    ble_monitor.h : Passive BLE observation interface
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_OBSERVATION_NAME_MAX       32
#define BLE_OBSERVATION_MFG_DATA_MAX   16
#define BLE_OBSERVATION_UUID16_MAX     8
#define BLE_OBSERVATION_UUID128_MAX    4

typedef struct
{
    uint8_t address[6];
    uint8_t address_type;
    int8_t  rssi;

    char    name[BLE_OBSERVATION_NAME_MAX];
    bool    has_name;

    uint16_t company_id;
    uint8_t  manufacturer_data[BLE_OBSERVATION_MFG_DATA_MAX];
    uint8_t  manufacturer_data_len;
    bool     has_manufacturer_data;

    uint16_t service_uuids16[BLE_OBSERVATION_UUID16_MAX];
    uint8_t  service_uuid16_count;

    uint8_t  service_uuids128[BLE_OBSERVATION_UUID128_MAX][16];
    uint8_t  service_uuid128_count;
} ble_observation_t;

/**
 * @brief Callback invoked for each parsed BLE advertisement.
 * @param a_observation Parsed advertiser identity and advertisement fields.
 */
typedef void ( *ble_observation_cb_t )( const ble_observation_t *a_observation );

/**
 * @brief Initialize NimBLE and start continuous passive BLE scanning.
 * @param a_callback Optional callback for parsed BLE advertisements.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t ble_monitor_init( ble_observation_cb_t a_callback );
