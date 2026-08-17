/**
    ble_monitor.h : Passive BLE observation interface for OUI-SPY C5
    Description: Exposes parsed BLE advertisements for detector matching while
                 keeping NimBLE scanning and advertisement parsing isolated.
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

/**
 * @brief Parsed fields from one passively received BLE advertisement.
 * @details The address is normalized into normal display order. Random/private
 *          address types remain explicitly identified because their leading
 *          bytes must not be treated as vendor OUIs. Advertisement fields are
 *          bounded to fixed-size storage suitable for callback processing.
 */
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
 * @details The observation is stack-backed by the BLE monitor and is valid
 *          only for the duration of the callback. Copy fields that must persist.
 */
typedef void ( *ble_observation_cb_t )( const ble_observation_t *a_observation );

/**
 * @brief Initialize NimBLE and start continuous passive BLE scanning.
 * @param a_callback Optional callback for parsed BLE advertisements.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 * @details Scanning is passive and duplicate filtering is disabled so changing
 *          advertisements remain visible to the detector. WiFi/BLE RF sharing
 *          is handled by the ESP32-C5 coexistence configuration.
 */
esp_err_t ble_monitor_init( ble_observation_cb_t a_callback );
