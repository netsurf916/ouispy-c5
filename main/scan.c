/**
    scan.c : T-Dongle C5 passive OUI detector
    Description: Passively monitors WiFi traffic, matches observed MAC OUIs
                 against surveillance-device categories, and updates the LCD.
*/

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "t_dongle_lcd.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "ouispy";

#define WIFI_CHANNEL_DWELL_MS    250
#define CATEGORY_ALERT_MS        12000
#define IEEE80211_ADDR1_OFFSET   4
#define IEEE80211_ADDR2_OFFSET   10
#define IEEE80211_ADDR2_MIN_LEN  16

/**
 * @brief Detection categories shown on the LCD.
 * @details Categories are derived from the OUI-SPY detector database.
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
    uint8_t prefix[3];
    oui_category_t category;
} oui_entry_t;

/**
 * @brief Runtime state for an LCD category.
 */
typedef struct
{
    const char *name;
    volatile TickType_t last_seen;
} category_state_t;

static const uint8_t wifi_2g_channels[] =
{
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

static const uint8_t wifi_5g_channels[] =
{
    36, 40, 44, 48,
    52, 56, 60, 64,
    100, 104, 108, 112,
    116, 120, 124, 128,
    132, 136, 140, 144,
    149, 153, 157, 161, 165
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

/*
 * OUI data adapted from colonelpanichacks/ouispy-detector/ouis.md.
 * Flock includes the promiscuous-mode prefixes credited there to
 * @NitekryDPaul in addition to the explicitly assigned Flock Safety OUI.
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

    /* DJI, Parrot, and Skydio drones */
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

    /* Ring doorbells / security cameras */
    { { 0x18, 0x7F, 0x88 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x24, 0x2B, 0xD6 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x34, 0x3E, 0xA4 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x54, 0xE0, 0x19 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x5C, 0x47, 0x5E }, CATEGORY_DOORBELL_CAMERA },
    { { 0x64, 0x9A, 0x63 }, CATEGORY_DOORBELL_CAMERA },
    { { 0x90, 0x48, 0x6C }, CATEGORY_DOORBELL_CAMERA },
    { { 0x9C, 0x76, 0x13 }, CATEGORY_DOORBELL_CAMERA },
    { { 0xAC, 0x9F, 0xC3 }, CATEGORY_DOORBELL_CAMERA },
    { { 0xC4, 0xDB, 0xAD }, CATEGORY_DOORBELL_CAMERA },
    { { 0xCC, 0x3B, 0xFB }, CATEGORY_DOORBELL_CAMERA },

    /* Meta / Ray-Ban smartglasses */
    { { 0x7C, 0x2A, 0x9E }, CATEGORY_SMARTGLASSES },
    { { 0xCC, 0x66, 0x0A }, CATEGORY_SMARTGLASSES },
    { { 0xF4, 0x03, 0x43 }, CATEGORY_SMARTGLASSES },
    { { 0x5C, 0xE9, 0x1E }, CATEGORY_SMARTGLASSES },
    { { 0x98, 0x59, 0x49 }, CATEGORY_SMARTGLASSES },
};

#define OUI_DATABASE_COUNT \
    ( sizeof( oui_database ) / sizeof( oui_database[0] ) )

/**
 * @brief Initialize NVS, networking, and WiFi for passive monitoring.
 */
static void wifi_init( void );

/**
 * @brief Enable ESP-IDF promiscuous receive mode.
 */
static void wifi_monitor_start( void );

/**
 * @brief ESP-IDF promiscuous receive callback.
 * @param a_buffer Received promiscuous packet buffer.
 * @param a_type Packet type reported by ESP-IDF.
 */
static void wifi_promiscuous_rx( void *a_buffer, wifi_promiscuous_pkt_type_t a_type );

/**
 * @brief Compare a MAC address against the OUI database.
 * @param a_mac Six-byte MAC address.
 */
static void oui_observe_mac( const uint8_t *a_mac );

/**
 * @brief Draw the category status screen into the LCD framebuffer.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_status( void );

/**
 * @brief Monitor all configured channels in one WiFi band.
 * @param a_band ESP-IDF band mode to select.
 * @param a_channels Channel list to monitor.
 * @param a_channel_count Number of channels in a_channels.
 */
static void wifi_monitor_band( wifi_band_mode_t a_band,
                               const uint8_t *a_channels,
                               size_t a_channel_count );

/**
 * @brief Application entry point.
 * @details Initializes the LCD and WiFi monitor, then continuously hops 2.4
 *          GHz and 5 GHz channels while updating category alerts on screen.
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

    wifi_init();
    wifi_monitor_start();

    while( 1 )
    {
        wifi_monitor_band( WIFI_BAND_MODE_2G_ONLY,
                           wifi_2g_channels,
                           WIFI_2G_CHANNEL_COUNT );

        wifi_monitor_band( WIFI_BAND_MODE_5G_ONLY,
                           wifi_5g_channels,
                           WIFI_5G_CHANNEL_COUNT );
    }
}

/**
 * @brief Initialize ESP-IDF networking and start WiFi in station mode.
 */
static void wifi_init( void )
{
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert( sta_netif );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps( WIFI_PS_NONE ) );
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

    ESP_LOGI( TAG, "Passive OUI monitoring enabled" );
}

/**
 * @brief Handle one received 802.11 management or data frame.
 * @param a_buffer Received promiscuous packet buffer.
 * @param a_type Packet type reported by ESP-IDF.
 * @details Both addr1 and addr2 are checked. This intentionally follows the
 *          OUI-SPY Flock research, which notes that examining receiver and
 *          transmitter addresses improves detection of burst-sleep devices.
 */
static void wifi_promiscuous_rx( void *a_buffer, wifi_promiscuous_pkt_type_t a_type )
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
    oui_observe_mac( &frame[IEEE80211_ADDR1_OFFSET] );
    oui_observe_mac( &frame[IEEE80211_ADDR2_OFFSET] );
}

/**
 * @brief Check an observed MAC against all known surveillance-device OUIs.
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
            oui_category_t category = oui_database[i].category;
            categories[category].last_seen = xTaskGetTickCount();
            return;
        }
    }
}

/**
 * @brief Draw the passive detector status screen.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_status( void )
{
    const TickType_t now = xTaskGetTickCount();
    const TickType_t alert_ticks = pdMS_TO_TICKS( CATEGORY_ALERT_MS );

    ESP_ERROR_CHECK( lcd_fill( LCD_BLACK ) );

    lcd_set_cursor( 4, 0 );
    ESP_ERROR_CHECK( lcd_draw_text( "OUI-SPY C5", LCD_CYAN, LCD_BLACK ) );

    for( size_t i = 0; i < CATEGORY_COUNT; i++ )
    {
        const TickType_t last_seen = categories[i].last_seen;
        const bool active = last_seen != 0 && ( now - last_seen ) <= alert_ticks;
        const uint16_t color = active ? LCD_RED : LCD_WHITE;

        lcd_set_cursor( 4, 12 + (int)i * 12 );
        ESP_ERROR_CHECK( lcd_draw_text( categories[i].name, color, LCD_BLACK ) );
    }

    return ESP_OK;
}

/**
 * @brief Hop across the supplied channels while keeping the LCD current.
 * @param a_band ESP-IDF band mode to select.
 * @param a_channels Channel list to monitor.
 * @param a_channel_count Number of channels in a_channels.
 */
static void wifi_monitor_band( wifi_band_mode_t a_band,
                               const uint8_t *a_channels,
                               size_t a_channel_count )
{
    esp_err_t err = esp_wifi_set_band_mode( a_band );
    if( err != ESP_OK )
    {
        ESP_LOGW( TAG, "Unable to select band: %s", esp_err_to_name( err ) );
        return;
    }

    for( size_t i = 0; i < a_channel_count; i++ )
    {
        err = esp_wifi_set_channel( a_channels[i], WIFI_SECOND_CHAN_NONE );
        if( err != ESP_OK )
        {
            ESP_LOGD( TAG, "Skipping channel %u: %s",
                      a_channels[i], esp_err_to_name( err ) );
            continue;
        }

        vTaskDelay( pdMS_TO_TICKS( WIFI_CHANNEL_DWELL_MS ) );

        ESP_ERROR_CHECK( lcd_draw_status() );
        ESP_ERROR_CHECK( lcd_flush() );
    }
}
