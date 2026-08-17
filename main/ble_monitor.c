/**
    ble_monitor.c : Passive BLE observation for OUI-SPY C5
    Description: Parses passive BLE advertisements into stable signature fields.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "ble_monitor.h"

#include "esp_log.h"
#include "esp_private/startup_internal.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "ouispy_ble";

#define BLE_SCAN_ITVL    0x0040
#define BLE_SCAN_WINDOW  0x0030

static uint8_t own_address_type;
static ble_observation_cb_t observation_callback;

static int ble_gap_event( struct ble_gap_event *a_event, void *a_arg );
static void ble_on_sync( void );
static void ble_host_task( void *a_arg );
static void ble_start_scan( void );
static void ble_parse_advertisement( const struct ble_gap_disc_desc *a_disc,
                                     ble_observation_t *a_observation );
static void ble_parse_field( uint8_t a_type,
                             const uint8_t *a_data,
                             size_t a_length,
                             ble_observation_t *a_observation );

/**
 * @brief Initialize NimBLE and start its host task.
 */
esp_err_t ble_monitor_init( ble_observation_cb_t a_callback )
{
    observation_callback = a_callback;
    int rc = nimble_port_init();

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "nimble_port_init failed: %d", rc );
        return ESP_FAIL;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init( ble_host_task );
    return ESP_OK;
}

static void ble_start_scan( void )
{
    struct ble_gap_disc_params params =
    {
        .itvl = BLE_SCAN_ITVL,
        .window = BLE_SCAN_WINDOW,
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc( own_address_type, BLE_HS_FOREVER,
                           &params, ble_gap_event, NULL );

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "BLE scan start failed: %d", rc );
    }
    else
    {
        ESP_LOGI( TAG, "Passive BLE monitor enabled" );
    }
}

static void ble_on_sync( void )
{
    int rc = ble_hs_util_ensure_addr( 0 );

    if( rc == 0 )
    {
        rc = ble_hs_id_infer_auto( 0, &own_address_type );
    }

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "BLE identity setup failed: %d", rc );
        return;
    }

    ble_start_scan();
}

/**
 * @brief Parse and forward one BLE discovery report.
 */
static int ble_gap_event( struct ble_gap_event *a_event, void *a_arg )
{
    (void)a_arg;

    if( a_event->type == BLE_GAP_EVENT_DISC )
    {
        ble_observation_t observation;
        ble_parse_advertisement( &a_event->disc, &observation );

        if( observation_callback != NULL )
        {
            observation_callback( &observation );
        }
    }
    else if( a_event->type == BLE_GAP_EVENT_DISC_COMPLETE )
    {
        ble_start_scan();
    }

    return 0;
}

/**
 * @brief Parse BLE AD structures used for device fingerprinting.
 * @details Captures local names, manufacturer company/data, and advertised
 *          16-bit and 128-bit service UUIDs. These remain useful when the
 *          advertiser uses a randomized/private BLE address.
 */
static void ble_parse_advertisement( const struct ble_gap_disc_desc *a_disc,
                                     ble_observation_t *a_observation )
{
    memset( a_observation, 0, sizeof( *a_observation ) );
    memcpy( a_observation->address, a_disc->addr.val, 6 );
    a_observation->address_type = a_disc->addr.type;
    a_observation->rssi = a_disc->rssi;

    size_t offset = 0;

    while( offset < a_disc->length_data )
    {
        const uint8_t field_length = a_disc->data[offset];

        if( field_length == 0 )
        {
            break;
        }

        if( offset + 1U + field_length > a_disc->length_data )
        {
            break;
        }

        const uint8_t type = a_disc->data[offset + 1U];
        ble_parse_field( type,
                         &a_disc->data[offset + 2U],
                         field_length - 1U,
                         a_observation );
        offset += (size_t)field_length + 1U;
    }
}

static void ble_parse_field( uint8_t a_type,
                             const uint8_t *a_data,
                             size_t a_length,
                             ble_observation_t *a_observation )
{
    if( ( a_type == 0x08U || a_type == 0x09U ) && a_length > 0 )
    {
        size_t length = a_length;
        if( length >= sizeof( a_observation->name ) )
        {
            length = sizeof( a_observation->name ) - 1U;
        }
        memcpy( a_observation->name, a_data, length );
        a_observation->name[length] = '\0';
        a_observation->has_name = true;
    }
    else if( a_type == 0xFFU && a_length >= 2U )
    {
        a_observation->company_id =
            (uint16_t)a_data[0] | ( (uint16_t)a_data[1] << 8 );
        size_t length = a_length - 2U;
        if( length > sizeof( a_observation->manufacturer_data ) )
        {
            length = sizeof( a_observation->manufacturer_data );
        }
        memcpy( a_observation->manufacturer_data, &a_data[2], length );
        a_observation->manufacturer_data_len = (uint8_t)length;
        a_observation->has_manufacturer_data = true;
    }
    else if( a_type == 0x02U || a_type == 0x03U )
    {
        for( size_t offset = 0;
             offset + 1U < a_length &&
             a_observation->service_uuid16_count < BLE_OBSERVATION_UUID16_MAX;
             offset += 2U )
        {
            a_observation->service_uuids16[a_observation->service_uuid16_count++] =
                (uint16_t)a_data[offset] | ( (uint16_t)a_data[offset + 1U] << 8 );
        }
    }
    else if( a_type == 0x06U || a_type == 0x07U )
    {
        for( size_t offset = 0;
             offset + 15U < a_length &&
             a_observation->service_uuid128_count < BLE_OBSERVATION_UUID128_MAX;
             offset += 16U )
        {
            memcpy( a_observation->service_uuids128[
                        a_observation->service_uuid128_count++],
                    &a_data[offset], 16 );
        }
    }
}

static void ble_host_task( void *a_arg )
{
    (void)a_arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

ESP_SYSTEM_INIT_FN( ouispy_ble_init, SECONDARY, BIT( 0 ), 700 )
{
    return ble_monitor_init( NULL );
}
