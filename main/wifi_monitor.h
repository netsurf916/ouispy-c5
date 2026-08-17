/**
    wifi_monitor.h : Passive WiFi monitor interface for OUI-SPY C5
    Description: Provides AP/client classification and channel-hopping support
                 for the ESP32-C5 promiscuous receiver.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#pragma once

#include "esp_err.h"

#include <stdint.h>

/**
 * @brief WiFi endpoint type inferred from an observed 802.11 frame.
 * @details Access points are identified from beacon/probe-response frames and
 *          infrastructure data direction. Clients are identified from probe
 *          requests and the station side of infrastructure data frames.
 */
typedef enum
{
    WIFI_OBSERVATION_AP = 0,
    WIFI_OBSERVATION_CLIENT
} wifi_observation_type_t;

/**
 * @brief Callback invoked for classified WiFi AP/client MAC observations.
 * @param a_type Inferred endpoint type.
 * @param a_mac Six-byte MAC address. The pointer is valid only for the duration
 *              of the callback and must be copied if retained.
 * @param a_channel Receive channel reported by the ESP-IDF promiscuous packet.
 * @param a_rssi Received signal strength in dBm.
 */
typedef void ( *wifi_observation_cb_t )( wifi_observation_type_t a_type,
                                         const uint8_t *a_mac,
                                         uint8_t a_channel,
                                         int8_t a_rssi );

/**
 * @brief Initialize receive-only WiFi promiscuous monitoring.
 * @param a_callback Optional callback for classified AP/client observations.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 * @details WiFi is started in WIFI_MODE_NULL, promiscuous reception is enabled
 *          for management and data frames, and no network association is made.
 */
esp_err_t wifi_monitor_init( wifi_observation_cb_t a_callback );

/**
 * @brief Advance the passive monitor by one channel dwell.
 * @details Repeated calls rotate across the configured 2.4 GHz and 5 GHz
 *          channel plans. Unsupported channels are skipped when rejected by
 *          ESP-IDF. The call blocks for the configured dwell time on a valid
 *          channel so promiscuous callbacks can collect traffic.
 */
void wifi_monitor_step( void );
