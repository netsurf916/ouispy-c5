/**
    scan.c : T-Dongle C5 passive OUI detector
    Description: Passively monitors WiFi traffic, matches observed MAC OUIs
                 against surveillance-device categories, and updates the LCD.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "t_dongle_lcd.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "ouispy";

#define WIFI_2G_CHANNEL_DWELL_MS   80
#define WIFI_5G_CHANNEL_DWELL_MS   60
#define WIFI_LCD_REFRESH_MS        750
#define MAX_OBSERVED_MACS          128
#define MAX_OBSERVED_CLIENTS       256
#define CATEGORY_HIGH_COUNT        6
#define IEEE80211_ADDR1_OFFSET     4
#define IEEE80211_ADDR2_OFFSET     10
#define IEEE80211_ADDR2_MIN_LEN    16
#define STATUS_DIVIDER_Y           9
#define STATUS_FIRST_ROW_Y         13
#define STATUS_ROW_HEIGHT          13
#define STATUS_LABEL_X             3
#define STATUS_COUNT_X             91
#define STATUS_BAR_X               108
#define STATUS_BAR_WIDTH           49
#define STATUS_BAR_HEIGHT          5

/**
 * @brief Detection categories shown on the LCD.
 */
typedef enum
{
    CATEGORY_FLOCK = 0,
    CATEGORY_BODY_CAM,
    CATEGORY_DRONE,
    CATEGORY_DOORBELL_CAMERA,
    CATEGORY_SMARTGLASSES,
    CATEGORY_COUNT
} oui_category_t;

/**
 * @brief One OUI-to-category mapping.
 */
typedef struct
{
    uint8_t        prefix[3];
    oui_category_t category;
} oui_entry_t;

/**
 * @brief Runtime state for one detector category.
 */
typedef struct
{
    const char        *name;
    volatile uint16_t  count;
} category_state_t;

/**
 * @brief Unique matched MAC retained to prevent packet-count inflation.
 */
typedef struct
{
    uint8_t        mac[6];
    oui_category_t category;
    bool           used;
} observed_mac_t;

/**
 * @brief Unique client MAC retained for diagnostic logging.
 */
typedef struct
{
    uint8_t mac[6];
    bool    used;
} observed_client_t;

/**
 * @brief Channel sweep description for one WiFi band.
 */
typedef struct
{
    wifi_band_mode_t band;
    const uint8_t   *channels;
    size_t           channel_count;
    uint32_t         dwell_ms;
    size_t           start_offset;
} wifi_sweep_t;

/*
 * ESP32-C5 RF coverage extends through 2484 MHz in 2.4 GHz and 5885 MHz in
 * 5 GHz. The driver may reject channels that are unavailable under the active
 * country/regulatory configuration; rejected channels are simply skipped.
 */
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

#define WIFI_2G_CHANNEL_COUNT \
    ( sizeof( wifi_2g_channels ) / sizeof( wifi_2g_channels[0] ) )

#define WIFI_5G_CHANNEL_COUNT \
    ( sizeof( wifi_5g_channels ) / sizeof( wifi_5g_channels[0] ) )

static category_state_t categories[CATEGORY_COUNT] =
{
    [CATEGORY_FLOCK]           = { "FLOCK / ALPR", 0 },
    [CATEGORY_BODY_CAM]        = { "BODY CAM", 0 },
    [CATEGORY_DRONE]           = { "DRONE", 0 },
    [CATEGORY_DOORBELL_CAMERA] = { "DOORBELL / CAM", 0 },
    [CATEGORY_SMARTGLASSES]    = { "SMARTGLASSES", 0 },
};

static observed_mac_t observed_macs[MAX_OBSERVED_MACS];
static observed_client_t observed_clients[MAX_OBSERVED_CLIENTS];
static TickType_t lcd_last_refresh;

/**
 * @brief OUI database used for passive category detection.
 * @details The baseline entries come from the OUI-SPY detector project. Extra
 *          entries are restricted to vendors with a narrow product scope so
 *          that an OUI match remains useful as a category signal.
 */
static const oui_entry_t oui_database[] =
{
    /* Flock Safety / ALPR */
    { { 0xB4, 0x1E, 0x52 }, CATEGORY_FLOCK },
    { { 0x70, 0xC9, 0x4E }, CATEGORY_FLOCK },
    { { 0x3C, 0x91, 0x80 }, CATEGORY_FLOCK },
    { { 0xD8, 0xF3, 0xBC }, CATEGORY_FLOCK },
    { { 0x80, 0x30, 0x49 }, CATEGORY_FLOCK },
    { { 0xB8, 0x35, 0x32 }, CATEGORY_FLOCK },
    { { 0x14, 0x5A, 0xFC }, CATEGORY_FLOCK },
    { { 0x74, 0x4C, 0xA1 }, CATEGORY_FLOCK },
    { { 0x08, 0x3A, 0x88 }, CATEGORY_FLOCK },
    { { 0x9C, 0x2F, 0x9D }, CATEGORY_FLOCK },
    { { 0xC0, 0x35, 0x32 }, CATEGORY_FLOCK },
    { { 0x94, 0x08, 0x53 }, CATEGORY_FLOCK },
    { { 0xE4, 0xAA, 0xEA }, CATEGORY_FLOCK },
    { { 0xF4, 0x6A, 0xDD }, CATEGORY_FLOCK },
    { { 0xF8, 0xA2, 0xD6 }, CATEGORY_FLOCK },
    { { 0x24, 0xB2, 0xB9 }, CATEGORY_FLOCK },
    { { 0x00, 0xF4, 0x8D }, CATEGORY_FLOCK },
    { { 0xD0, 0x39, 0x57 }, CATEGORY_FLOCK },
    { { 0xE8, 0xD0, 0xFC }, CATEGORY_FLOCK },
    { { 0xE0, 0x4F, 0x43 }, CATEGORY_FLOCK },
    { { 0xB8, 0x1E, 0xA4 }, CATEGORY_FLOCK },
    { { 0x70, 0x08, 0x94 }, CATEGORY_FLOCK },
    { { 0x58, 0x8E, 0x81 }, CATEGORY_FLOCK },
    { { 0xEC, 0x1B, 0xBD }, CATEGORY_FLOCK },
    { { 0x3C, 0x71, 0xBF }, CATEGORY_FLOCK },
    { { 0x58, 0x00, 0xE3 }, CATEGORY_FLOCK },
    { { 0x90, 0x35, 0xEA }, CATEGORY_FLOCK },
    { { 0x5C, 0x93, 0xA2 }, CATEGORY_FLOCK },
    { { 0x64, 0x6E, 0x69 }, CATEGORY_FLOCK },
    { { 0x48, 0x27, 0xEA }, CATEGORY_FLOCK },
    { { 0xA4, 0xCF, 0x12 }, CATEGORY_FLOCK },

    /* Axon body cameras / law enforcement */
    { { 0x00, 0x25, 0xDF }, CATEGORY_BODY_CAM },

    /* DJI / Parrot / Skydio */
    { { 0x0C, 0x9A, 0xE6 }, CATEGORY_DRONE },
    { { 0x8C, 0x58, 0x23 }, CATEGORY_DRONE },
    { { 0x04, 0xA8, 0x5A }, CATEGORY_DRONE },
    { { 0x58, 0xB8, 0x58 }, CATEGORY_DRONE },
    { { 0xE4, 0x7A, 0x2C }, CATEGORY_DRONE },
    { { 0x60, 0x60, 0x1F }, CATEGORY_DRONE },
    { { 0x48, 0x1C, 0xB9 }, CATEGORY_DRONE },
    { { 0x34, 0xD2, 0x62 }, CATEGORY_DRONE },
    { { 0x00, 0x12, 0x1C }, CATEGORY_DRONE },
    { { 0x00, 0x26, 0x7E }, CATEGORY_DRONE },
    { { 0x90, 0x03, 0xB7 }, CATEGORY_DRONE },
    { { 0x90, 0x3A, 0xE6 }, CATEGORY_DRONE },
    { { 0xA0, 0x14, 0x3D }, CATEGORY_DRONE },
    { { 0x38, 0x1D, 0x14 }, CATEGORY_DRONE },

    /* Ring LLC */
    { { 0x18, 0x7F, 0x88 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x24, 0x2B, 0xD6 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x34, 0x3E, 0xA4 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x50, 0xE4, 0x67 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x54, 0xE0, 0x19 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x5C, 0x47, 0x5E }, CATEGORY_DOORBELL_CAMERA },
    { { 0x64, 0x9A, 0x63 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x90, 0x48, 0x6C }, CATEGORY_DOORBELL_CAMERA },
    { { 0x9C, 0x76, 0x13 }, CATEGORY_DOORBELL_CAMERA },
    { { 0xAC, 0x9F, 0xC3 }, CATEGORY_DOORBELL_CAMERA },
    { { 0xC4, 0xDB, 0xAD }, CATEGORY_DOORBELL_CAMERA },
    { { 0xCC, 0x3B, 0xFB }, CATEGORY_DOORBELL_CAMERA },
    { { 0x00, 0xB4, 0x63 }, CATEGORY_DOORBELL_CAMERA },

    /* Meta / Ray-Ban plus Vuzix */
    { { 0x7C, 0x2A, 0x9E }, CATEGORY_SMARTGLASSES },
    { { 0xCC, 0x66, 0x0A }, CATEGORY_SMARTGLASSES },
    { { 0xF4, 0x03, 0x43 }, CATEGORY_SMARTGLASSES },
    { { 0x5C, 0xE9, 0x1E }, CATEGORY_SMARTGLASSES },
    { { 0x98, 0x59, 0x49 }, CATEGORY_SMARTGLASSES },
    { { 0x98, 0xDA, 0x92 }, CATEGORY_SMARTGLASSES },
};

#define OUI_DATABASE_COUNT \
    ( sizeof( oui_database ) / sizeof( oui_database[0] ) )

/**
 * @brief Initialize ESP-IDF WiFi for receive-only promiscuous monitoring.
 */
static void wifi_init( void );

/**
 * @brief Enable ESP-IDF promiscuous receive mode.
 */
static void wifi_monitor_start( void );

/**
 * @brief Handle one received management or data frame.
 * @param a_buffer Received promiscuous packet buffer.
 * @param a_type Packet type reported by ESP-IDF.
 */
static void wifi_promiscuous_rx( void *a_buffer,
                                 wifi_promiscuous_pkt_type_t a_type );

/**
 * @brief Identify client addresses from an observed 802.11 frame.
 * @param a_frame Raw 802.11 frame payload.
 * @param a_type ESP-IDF packet type.
 * @param a_channel Channel on which the frame was received.
 * @param a_rssi Received signal strength in dBm.
 */
static void wifi_observe_client_frame( const uint8_t *a_frame,
                                       wifi_promiscuous_pkt_type_t a_type,
                                       uint8_t a_channel,
                                       int8_t a_rssi );

/**
 * @brief Log a client MAC the first time it is observed.
 * @param a_mac Six-byte client MAC address.
 * @param a_channel Channel on which the client was observed.
 * @param a_rssi Received signal strength in dBm.
 */
static void wifi_observe_client( const uint8_t *a_mac,
                                 uint8_t a_channel,
                                 int8_t a_rssi );

/**
 * @brief Match an observed MAC address against the OUI database.
 * @param a_mac Six-byte MAC address.
 */
static void oui_observe_mac( const uint8_t *a_mac );

/**
 * @brief Remember a matching MAC if it has not already been counted.
 * @param a_mac Six-byte MAC address.
 * @param a_category Category associated with the address.
 * @return True when the MAC was newly stored; false otherwise.
 */
static bool oui_remember_mac( const uint8_t *a_mac,
                              oui_category_t a_category );

/**
 * @brief Select the LCD color for a category count.
 * @param a_count Number of unique matching devices.
 * @return White for zero, yellow for 1-5, and magenta for 6 or more.
 */
static uint16_t category_color( uint16_t a_count );

/**
 * @brief Draw detector counts and prevalence bars into the LCD framebuffer.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_status( void );

/**
 * @brief Refresh the LCD if its periodic update interval has elapsed.
 */
static void lcd_refresh_if_due( void );

/**
 * @brief Hop through every channel in one WiFi sweep.
 * @param a_sweep Sweep state and channel plan.
 */
static void wifi_monitor_sweep( wifi_sweep_t *a_sweep );

/**
 * @brief Application entry point.
 */
void app_main( void )
{
    esp_err_t ret = nvs_flash_init();

    if( ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK( lcd_init() );
    ESP_ERROR_CHECK( lcd_draw_status() );
    ESP_ERROR_CHECK( lcd_flush() );

    lcd_last_refresh = xTaskGetTickCount();

    wifi_init();
    wifi_monitor_start();

    wifi_sweep_t sweep_2g =
    {
        .band = WIFI_BAND_MODE_2G_ONLY,
        .channels = wifi_2g_channels,
        .channel_count = WIFI_2G_CHANNEL_COUNT,
        .dwell_ms = WIFI_2G_CHANNEL_DWELL_MS,
        .start_offset = 0,
    };

    wifi_sweep_t sweep_5g =
    {
        .band = WIFI_BAND_MODE_5G_ONLY,
        .channels = wifi_5g_channels,
        .channel_count = WIFI_5G_CHANNEL_COUNT,
        .dwell_ms = WIFI_5G_CHANNEL_DWELL_MS,
        .start_offset = 0,
    };

    while( 1 )
    {
        wifi_monitor_sweep( &sweep_2g );
        wifi_monitor_sweep( &sweep_5g );
    }
}

/**
 * @brief Initialize WiFi without a station/AP interface.
 * @details Promiscuous mode is supported in WIFI_MODE_NULL. Avoiding a station
 *          interface keeps this application receive-only and eliminates
 *          association/power-save behavior that is unnecessary for sniffing.
 */
static void wifi_init( void )
{
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_NULL ) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

/**
 * @brief Configure and enable passive promiscuous monitoring.
 */
static void wifi_monitor_start( void )
{
    wifi_promiscuous_filter_t filter =
    {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };

    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter( &filter ) );
    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_rx_cb( wifi_promiscuous_rx ) );
    ESP_ERROR_CHECK( esp_wifi_set_promiscuous( true ) );

    ESP_LOGI( TAG, "Passive monitor enabled" );
}

/**
 * @brief Handle one received 802.11 management or data frame.
 * @param a_buffer Received promiscuous packet buffer.
 * @param a_type Packet type reported by ESP-IDF.
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

    const uint8_t *frame = packet->payload;

    wifi_observe_client_frame( frame,
                               a_type,
                               packet->rx_ctrl.channel,
                               packet->rx_ctrl.rssi );

    oui_observe_mac( &frame[IEEE80211_ADDR1_OFFSET] );
    oui_observe_mac( &frame[IEEE80211_ADDR2_OFFSET] );
}

/**
 * @brief Identify likely client addresses from an observed 802.11 frame.
 * @param a_frame Raw 802.11 frame payload.
 * @param a_type ESP-IDF packet type.
 * @param a_channel Channel on which the frame was received.
 * @param a_rssi Received signal strength in dBm.
 * @details Probe requests identify the transmitter as a client. Infrastructure
 *          data frames identify the station using the To DS / From DS bits.
 */
static void wifi_observe_client_frame( const uint8_t *a_frame,
                                       wifi_promiscuous_pkt_type_t a_type,
                                       uint8_t a_channel,
                                       int8_t a_rssi )
{
    const uint16_t frame_control =
        (uint16_t)a_frame[0] | ( (uint16_t)a_frame[1] << 8 );

    if( a_type == WIFI_PKT_MGMT )
    {
        const uint8_t subtype = ( frame_control >> 4 ) & 0x0FU;

        if( subtype == 4 )
        {
            wifi_observe_client( &a_frame[IEEE80211_ADDR2_OFFSET],
                                 a_channel,
                                 a_rssi );
        }

        return;
    }

    const bool to_ds = ( frame_control & 0x0100U ) != 0;
    const bool from_ds = ( frame_control & 0x0200U ) != 0;

    if( to_ds && !from_ds )
    {
        wifi_observe_client( &a_frame[IEEE80211_ADDR2_OFFSET],
                             a_channel,
                             a_rssi );
    }
    else if( !to_ds && from_ds )
    {
        wifi_observe_client( &a_frame[IEEE80211_ADDR1_OFFSET],
                             a_channel,
                             a_rssi );
    }
}

/**
 * @brief Log a client MAC the first time it is observed.
 * @param a_mac Six-byte client MAC address.
 * @param a_channel Channel on which the client was observed.
 * @param a_rssi Received signal strength in dBm.
 */
static void wifi_observe_client( const uint8_t *a_mac,
                                 uint8_t a_channel,
                                 int8_t a_rssi )
{
    if( a_mac == NULL || ( a_mac[0] & 0x01U ) != 0 )
    {
        return;
    }

    size_t free_slot = MAX_OBSERVED_CLIENTS;

    for( size_t i = 0; i < MAX_OBSERVED_CLIENTS; i++ )
    {
        if( observed_clients[i].used )
        {
            if( memcmp( observed_clients[i].mac,
                        a_mac,
                        sizeof( observed_clients[i].mac ) ) == 0 )
            {
                return;
            }
        }
        else if( free_slot == MAX_OBSERVED_CLIENTS )
        {
            free_slot = i;
        }
    }

    if( free_slot == MAX_OBSERVED_CLIENTS )
    {
        return;
    }

    memcpy( observed_clients[free_slot].mac,
            a_mac,
            sizeof( observed_clients[free_slot].mac ) );
    observed_clients[free_slot].used = true;

    ESP_LOGI( TAG,
              "Client observed: %02X:%02X:%02X:%02X:%02X:%02X ch=%u rssi=%d",
              a_mac[0], a_mac[1], a_mac[2],
              a_mac[3], a_mac[4], a_mac[5],
              a_channel, a_rssi );
}

/**
 * @brief Match an observed MAC and count it once per unique address.
 * @param a_mac Six-byte MAC address.
 */
static void oui_observe_mac( const uint8_t *a_mac )
{
    if( a_mac == NULL || ( a_mac[0] & 0x01U ) != 0 )
    {
        return;
    }

    for( size_t i = 0; i < OUI_DATABASE_COUNT; i++ )
    {
        if( memcmp( a_mac, oui_database[i].prefix, 3 ) == 0 )
        {
            const oui_category_t category = oui_database[i].category;

            if( oui_remember_mac( a_mac, category ) )
            {
                categories[category].count++;
                ESP_LOGI( TAG,
                          "New %s device (%u total)",
                          categories[category].name,
                          categories[category].count );
            }

            return;
        }
    }
}

/**
 * @brief Remember a matching MAC if it has not already been counted.
 * @param a_mac Six-byte MAC address.
 * @param a_category Category associated with the address.
 * @return True when the MAC was newly stored; false otherwise.
 */
static bool oui_remember_mac( const uint8_t *a_mac,
                              oui_category_t a_category )
{
    size_t free_slot = MAX_OBSERVED_MACS;

    for( size_t i = 0; i < MAX_OBSERVED_MACS; i++ )
    {
        if( observed_macs[i].used )
        {
            if( memcmp( observed_macs[i].mac,
                        a_mac,
                        sizeof( observed_macs[i].mac ) ) == 0 )
            {
                return false;
            }
        }
        else if( free_slot == MAX_OBSERVED_MACS )
        {
            free_slot = i;
        }
    }

    if( free_slot == MAX_OBSERVED_MACS )
    {
        return false;
    }

    memcpy( observed_macs[free_slot].mac,
            a_mac,
            sizeof( observed_macs[free_slot].mac ) );

    observed_macs[free_slot].category = a_category;
    observed_macs[free_slot].used = true;

    return true;
}

/**
 * @brief Select the LCD color for a category count.
 * @param a_count Number of unique matching devices.
 * @return White for zero, yellow for 1-5, and magenta for 6 or more.
 */
static uint16_t category_color( uint16_t a_count )
{
    if( a_count == 0 )
    {
        return LCD_WHITE;
    }

    if( a_count < CATEGORY_HIGH_COUNT )
    {
        return LCD_YELLOW;
    }

    return LCD_MAGENTA;
}

/**
 * @brief Draw category counts, total count, and prevalence bars.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_status( void )
{
    uint16_t max_count = 1;
    uint16_t total_count = 0;
    char text[24];

    for( size_t i = 0; i < CATEGORY_COUNT; i++ )
    {
        total_count += categories[i].count;

        if( categories[i].count > max_count )
        {
            max_count = categories[i].count;
        }
    }

    ESP_ERROR_CHECK( lcd_fill( LCD_BLACK ) );

    snprintf( text, sizeof( text ), "OUI-SPY C5 N:%u", total_count );
    lcd_set_cursor( 4, 0 );
    ESP_ERROR_CHECK( lcd_draw_text( text, LCD_CYAN, LCD_BLACK ) );

    ESP_ERROR_CHECK( lcd_draw_line( 2,
                                    STATUS_DIVIDER_Y,
                                    LCD_WIDTH - 3,
                                    STATUS_DIVIDER_Y,
                                    LCD_CYAN ) );

    for( size_t i = 0; i < CATEGORY_COUNT; i++ )
    {
        const int y = STATUS_FIRST_ROW_Y + (int)i * STATUS_ROW_HEIGHT;
        const uint16_t color = category_color( categories[i].count );

        lcd_set_cursor( STATUS_LABEL_X, y );
        ESP_ERROR_CHECK( lcd_draw_text( categories[i].name,
                                        color,
                                        LCD_BLACK ) );

        snprintf( text, sizeof( text ), "%u", categories[i].count );
        lcd_set_cursor( STATUS_COUNT_X, y );
        ESP_ERROR_CHECK( lcd_draw_text( text, color, LCD_BLACK ) );

        ESP_ERROR_CHECK( lcd_draw_line( STATUS_BAR_X,
                                        y + 6,
                                        STATUS_BAR_X + STATUS_BAR_WIDTH,
                                        y + 6,
                                        LCD_WHITE ) );

        if( categories[i].count > 0 )
        {
            int width = ( STATUS_BAR_WIDTH * categories[i].count ) / max_count;

            if( width < 1 )
            {
                width = 1;
            }

            ESP_ERROR_CHECK( lcd_fill_rect( STATUS_BAR_X,
                                            y + 4,
                                            width,
                                            STATUS_BAR_HEIGHT,
                                            color ) );
        }
    }

    return ESP_OK;
}

/**
 * @brief Refresh the LCD if its periodic update interval has elapsed.
 * @details Display traffic is intentionally decoupled from channel changes so
 *          the radio spends nearly all of each sweep interval listening.
 */
static void lcd_refresh_if_due( void )
{
    const TickType_t now = xTaskGetTickCount();
    const TickType_t refresh_ticks = pdMS_TO_TICKS( WIFI_LCD_REFRESH_MS );

    if( ( now - lcd_last_refresh ) < refresh_ticks )
    {
        return;
    }

    ESP_ERROR_CHECK( lcd_draw_status() );
    ESP_ERROR_CHECK( lcd_flush() );
    lcd_last_refresh = now;
}

/**
 * @brief Hop through every channel in one WiFi sweep.
 * @param a_sweep Sweep state and channel plan.
 * @details The starting point rotates after each sweep. This avoids always
 *          visiting a particular channel at the same point in the cycle and
 *          reduces systematic misses from periodic/bursty transmitters.
 */
static void wifi_monitor_sweep( wifi_sweep_t *a_sweep )
{
    esp_err_t err = esp_wifi_set_band_mode( a_sweep->band );

    if( err != ESP_OK )
    {
        ESP_LOGW( TAG,
                  "Unable to select band: %s",
                  esp_err_to_name( err ) );
        return;
    }

    for( size_t i = 0; i < a_sweep->channel_count; i++ )
    {
        const size_t index =
            ( a_sweep->start_offset + i ) % a_sweep->channel_count;
        const uint8_t channel = a_sweep->channels[index];

        err = esp_wifi_set_channel( channel, WIFI_SECOND_CHAN_NONE );

        if( err != ESP_OK )
        {
            ESP_LOGD( TAG,
                      "Skipping channel %u: %s",
                      channel,
                      esp_err_to_name( err ) );
            continue;
        }

        vTaskDelay( pdMS_TO_TICKS( a_sweep->dwell_ms ) );
        lcd_refresh_if_due();
    }

    a_sweep->start_offset =
        ( a_sweep->start_offset + 1 ) % a_sweep->channel_count;
}
