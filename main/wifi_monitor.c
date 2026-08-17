/**
    wifi_monitor.c : Passive WiFi observation for OUI-SPY C5
    Description: Classifies AP/client traffic and hops the ESP32-C5 receiver
                 across 2.4 GHz and 5 GHz channels.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "wifi_monitor.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static const char *TAG = "ouispy_wifi";

#define WIFI_2G_CHANNEL_DWELL_MS  80
#define WIFI_5G_CHANNEL_DWELL_MS  60
#define IEEE80211_ADDR1_OFFSET    4
#define IEEE80211_ADDR2_OFFSET    10
#define IEEE80211_ADDR2_MIN_LEN   16

typedef struct
{
    wifi_band_mode_t band;
    const uint8_t   *channels;
    size_t           channel_count;
    uint32_t         dwell_ms;
    size_t           next_index;
} wifi_sweep_t;

static const uint8_t wifi_2g_channels[] =
{
    1, 2, 3, 4, 5, 6, 7,
    8, 9, 10, 11, 12, 13, 14
};

static const uint8_t wifi_5g_channels[] =
{
    36, 40, 44, 48,
    52, 56, 60, 64,
    100, 104, 108, 112,
    116, 120, 124, 128,
    132, 136, 140, 144,
    149, 153, 157, 161,
    165, 169, 173, 177
};

static wifi_sweep_t sweeps[] =
{
    {
        .band = WIFI_BAND_MODE_2G_ONLY,
        .channels = wifi_2g_channels,
        .channel_count = sizeof( wifi_2g_channels ) / sizeof( wifi_2g_channels[0] ),
        .dwell_ms = WIFI_2G_CHANNEL_DWELL_MS,
    },
    {
        .band = WIFI_BAND_MODE_5G_ONLY,
        .channels = wifi_5g_channels,
        .channel_count = sizeof( wifi_5g_channels ) / sizeof( wifi_5g_channels[0] ),
        .dwell_ms = WIFI_5G_CHANNEL_DWELL_MS,
    },
};

static wifi_observation_cb_t observation_callback;
static size_t current_sweep;

static void wifi_promiscuous_rx( void *a_buffer,
                                 wifi_promiscuous_pkt_type_t a_type );
static void wifi_classify_frame( const uint8_t *a_frame,
                                 wifi_promiscuous_pkt_type_t a_type,
                                 uint8_t a_channel,
                                 int8_t a_rssi );
static void wifi_report( wifi_observation_type_t a_type,
                         const uint8_t *a_mac,
                         uint8_t a_channel,
                         int8_t a_rssi );

/**
 * @brief Initialize WiFi without a station/AP interface and enable sniffing.
 */
esp_err_t wifi_monitor_init( wifi_observation_cb_t a_callback )
{
    observation_callback = a_callback;

    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_NULL ) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    wifi_promiscuous_filter_t filter =
    {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };

    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter( &filter ) );
    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_rx_cb( wifi_promiscuous_rx ) );
    ESP_ERROR_CHECK( esp_wifi_set_promiscuous( true ) );

    ESP_LOGI( TAG, "Passive WiFi monitor enabled" );
    return ESP_OK;
}

/**
 * @brief Receive and classify one management or data frame.
 */
static void wifi_promiscuous_rx( void *a_buffer,
                                 wifi_promiscuous_pkt_type_t a_type )
{
    if( a_buffer == NULL ||
        ( a_type != WIFI_PKT_MGMT && a_type != WIFI_PKT_DATA ) )
    {
        return;
    }

    const wifi_promiscuous_pkt_t *packet =
        (const wifi_promiscuous_pkt_t *)a_buffer;

    if( packet->rx_ctrl.sig_len < IEEE80211_ADDR2_MIN_LEN )
    {
        return;
    }

    wifi_classify_frame( packet->payload,
                         a_type,
                         packet->rx_ctrl.channel,
                         packet->rx_ctrl.rssi );
}

/**
 * @brief Classify APs and clients from 802.11 management/data frames.
 * @details Beacon and probe-response transmitters are APs. Probe-request
 *          transmitters are clients. Infrastructure data direction bits
 *          identify the station and BSSID endpoints.
 */
static void wifi_classify_frame( const uint8_t *a_frame,
                                 wifi_promiscuous_pkt_type_t a_type,
                                 uint8_t a_channel,
                                 int8_t a_rssi )
{
    const uint16_t fc =
        (uint16_t)a_frame[0] | ( (uint16_t)a_frame[1] << 8 );

    if( a_type == WIFI_PKT_MGMT )
    {
        const uint8_t subtype = ( fc >> 4 ) & 0x0FU;

        if( subtype == 8 || subtype == 5 )
        {
            wifi_report( WIFI_OBSERVATION_AP,
                         &a_frame[IEEE80211_ADDR2_OFFSET],
                         a_channel,
                         a_rssi );
        }
        else if( subtype == 4 )
        {
            wifi_report( WIFI_OBSERVATION_CLIENT,
                         &a_frame[IEEE80211_ADDR2_OFFSET],
                         a_channel,
                         a_rssi );
        }

        return;
    }

    const bool to_ds = ( fc & 0x0100U ) != 0;
    const bool from_ds = ( fc & 0x0200U ) != 0;

    if( to_ds && !from_ds )
    {
        wifi_report( WIFI_OBSERVATION_AP,
                     &a_frame[IEEE80211_ADDR1_OFFSET],
                     a_channel,
                     a_rssi );
        wifi_report( WIFI_OBSERVATION_CLIENT,
                     &a_frame[IEEE80211_ADDR2_OFFSET],
                     a_channel,
                     a_rssi );
    }
    else if( !to_ds && from_ds )
    {
        wifi_report( WIFI_OBSERVATION_CLIENT,
                     &a_frame[IEEE80211_ADDR1_OFFSET],
                     a_channel,
                     a_rssi );
        wifi_report( WIFI_OBSERVATION_AP,
                     &a_frame[IEEE80211_ADDR2_OFFSET],
                     a_channel,
                     a_rssi );
    }
}

/**
 * @brief Forward a valid unicast endpoint to the application observer.
 */
static void wifi_report( wifi_observation_type_t a_type,
                         const uint8_t *a_mac,
                         uint8_t a_channel,
                         int8_t a_rssi )
{
    if( a_mac == NULL || ( a_mac[0] & 0x01U ) != 0 )
    {
        return;
    }

    if( observation_callback != NULL )
    {
        observation_callback( a_type, a_mac, a_channel, a_rssi );
    }
}

/**
 * @brief Tune to one channel, dwell, then advance to the next channel/band.
 */
void wifi_monitor_step( void )
{
    wifi_sweep_t *sweep = &sweeps[current_sweep];
    esp_err_t err = esp_wifi_set_band_mode( sweep->band );

    if( err == ESP_OK )
    {
        const uint8_t channel = sweep->channels[sweep->next_index];
        err = esp_wifi_set_channel( channel, WIFI_SECOND_CHAN_NONE );

        if( err == ESP_OK )
        {
            vTaskDelay( pdMS_TO_TICKS( sweep->dwell_ms ) );
        }
        else
        {
            ESP_LOGD( TAG,
                      "Skipping channel %u: %s",
                      channel,
                      esp_err_to_name( err ) );
        }
    }
    else
    {
        ESP_LOGW( TAG, "Unable to select band: %s", esp_err_to_name( err ) );
    }

    sweep->next_index++;

    if( sweep->next_index >= sweep->channel_count )
    {
        sweep->next_index = 0;
        current_sweep = ( current_sweep + 1 ) %
                        ( sizeof( sweeps ) / sizeof( sweeps[0] ) );
    }
}
