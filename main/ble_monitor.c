/**
    ble_monitor.c : Passive BLE observation for OUI-SPY C5
    Description: Continuously scans BLE advertisements without initiating
                 connections and logs newly observed advertiser identities.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "ble_monitor.h"

#include "esp_log.h"
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

#define MAX_OBSERVED_BLE_DEVICES  256
#define BLE_SCAN_ITVL             0x0040
#define BLE_SCAN_WINDOW           0x0030

typedef struct
{
    uint8_t address[6];
    uint8_t address_type;
    bool    used;
} observed_ble_t;

static observed_ble_t observed_ble[MAX_OBSERVED_BLE_DEVICES];
static uint8_t own_address_type;

static int ble_gap_event( struct ble_gap_event *a_event, void *a_arg );
static void ble_on_sync( void );
static void ble_host_task( void *a_arg );
static bool ble_remember_device( const ble_addr_t *a_address );
static void ble_start_scan( void );

/**
 * @brief Initialize NimBLE and start its host task.
 * @return ESP_OK on success.
 */
esp_err_t ble_monitor_init( void )
{
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

/**
 * @brief Start a continuous passive BLE discovery procedure.
 */
static void ble_start_scan( void )
{
    struct ble_gap_disc_params params =
    {
        .itvl = BLE_SCAN_ITVL,
        .window = BLE_SCAN_WINDOW,
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 1,
    };

    int rc = ble_gap_disc( own_address_type,
                           BLE_HS_FOREVER,
                           &params,
                           ble_gap_event,
                           NULL );

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "BLE scan start failed: %d", rc );
    }
    else
    {
        ESP_LOGI( TAG, "Passive BLE monitor enabled" );
    }
}

/**
 * @brief Complete NimBLE identity setup after the host synchronizes.
 */
static void ble_on_sync( void )
{
    int rc = ble_hs_util_ensure_addr( 0 );

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "Unable to create BLE identity: %d", rc );
        return;
    }

    rc = ble_hs_id_infer_auto( 0, &own_address_type );

    if( rc != 0 )
    {
        ESP_LOGE( TAG, "Unable to select BLE identity: %d", rc );
        return;
    }

    ble_start_scan();
}

/**
 * @brief Process BLE discovery events.
 */
static int ble_gap_event( struct ble_gap_event *a_event, void *a_arg )
{
    (void)a_arg;

    switch( a_event->type )
    {
        case BLE_GAP_EVENT_DISC:
        {
            const struct ble_gap_disc_desc *disc = &a_event->disc;

            if( ble_remember_device( &disc->addr ) )
            {
                ESP_LOGI( TAG,
                          "BLE observed: %02X:%02X:%02X:%02X:%02X:%02X type=%u rssi=%d adv_len=%u",
                          disc->addr.val[5], disc->addr.val[4], disc->addr.val[3],
                          disc->addr.val[2], disc->addr.val[1], disc->addr.val[0],
                          disc->addr.type,
                          disc->rssi,
                          disc->length_data );
            }
            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ble_start_scan();
            break;

        default:
            break;
    }

    return 0;
}

/**
 * @brief Remember an advertiser identity once for diagnostic logging.
 */
static bool ble_remember_device( const ble_addr_t *a_address )
{
    size_t free_slot = MAX_OBSERVED_BLE_DEVICES;

    for( size_t i = 0; i < MAX_OBSERVED_BLE_DEVICES; i++ )
    {
        if( observed_ble[i].used )
        {
            if( observed_ble[i].address_type == a_address->type &&
                memcmp( observed_ble[i].address,
                        a_address->val,
                        sizeof( observed_ble[i].address ) ) == 0 )
            {
                return false;
            }
        }
        else if( free_slot == MAX_OBSERVED_BLE_DEVICES )
        {
            free_slot = i;
        }
    }

    if( free_slot == MAX_OBSERVED_BLE_DEVICES )
    {
        return false;
    }

    memcpy( observed_ble[free_slot].address,
            a_address->val,
            sizeof( observed_ble[free_slot].address ) );
    observed_ble[free_slot].address_type = a_address->type;
    observed_ble[free_slot].used = true;

    return true;
}

/**
 * @brief Run the NimBLE host until the port is stopped.
 */
static void ble_host_task( void *a_arg )
{
    (void)a_arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}
