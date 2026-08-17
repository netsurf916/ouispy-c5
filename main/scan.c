/**
    scan.c : T-Dongle C5 passive OUI detector
    Description: Matches radio observations against surveillance-device OUIs
                 and maintains the LCD status display.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "t_dongle_lcd.h"
#include "wifi_monitor.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "ouispy";

#define WIFI_LCD_REFRESH_MS    750
#define MAX_OBSERVED_MACS      128
#define MAX_WIFI_ENDPOINTS     256
#define CATEGORY_HIGH_COUNT    6
#define STATUS_DIVIDER_Y       9
#define STATUS_FIRST_ROW_Y     13
#define STATUS_ROW_HEIGHT      13
#define STATUS_LABEL_X         3
#define STATUS_COUNT_X         91
#define STATUS_BAR_X           108
#define STATUS_BAR_WIDTH       49
#define STATUS_BAR_HEIGHT      5

typedef enum
{
    CATEGORY_FLOCK = 0,
    CATEGORY_BODY_CAM,
    CATEGORY_DRONE,
    CATEGORY_DOORBELL_CAMERA,
    CATEGORY_SMARTGLASSES,
    CATEGORY_COUNT
} oui_category_t;

typedef struct
{
    uint8_t        prefix[3];
    oui_category_t category;
} oui_entry_t;

typedef struct
{
    const char        *name;
    volatile uint16_t  count;
} category_state_t;

typedef struct
{
    uint8_t        mac[6];
    oui_category_t category;
    bool           used;
} observed_mac_t;

typedef struct
{
    uint8_t                 mac[6];
    wifi_observation_type_t type;
    bool                    used;
} wifi_endpoint_t;

static category_state_t categories[CATEGORY_COUNT] =
{
    [CATEGORY_FLOCK]           = { "FLOCK / ALPR", 0 },
    [CATEGORY_BODY_CAM]        = { "BODY CAM", 0 },
    [CATEGORY_DRONE]           = { "DRONE", 0 },
    [CATEGORY_DOORBELL_CAMERA] = { "DOORBELL / CAM", 0 },
    [CATEGORY_SMARTGLASSES]    = { "SMARTGLASSES", 0 },
};

static observed_mac_t observed_macs[MAX_OBSERVED_MACS];
static wifi_endpoint_t wifi_endpoints[MAX_WIFI_ENDPOINTS];
static TickType_t lcd_last_refresh;

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

    /* Axon body cameras */
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

static void wifi_observation( wifi_observation_type_t a_type,
                              const uint8_t *a_mac,
                              uint8_t a_channel,
                              int8_t a_rssi );
static void oui_observe_mac( const uint8_t *a_mac );
static bool oui_remember_mac( const uint8_t *a_mac,
                              oui_category_t a_category );
static uint16_t category_color( uint16_t a_count );
static esp_err_t lcd_draw_status( void );
static void lcd_refresh_if_due( void );

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
    ESP_ERROR_CHECK( wifi_monitor_init( wifi_observation ) );

    while( 1 )
    {
        wifi_monitor_step();
        lcd_refresh_if_due();
    }
}

/**
 * @brief Consume a classified AP or client observation from wifi_monitor.
 */
static void wifi_observation( wifi_observation_type_t a_type,
                              const uint8_t *a_mac,
                              uint8_t a_channel,
                              int8_t a_rssi )
{
    size_t free_slot = MAX_WIFI_ENDPOINTS;

    for( size_t i = 0; i < MAX_WIFI_ENDPOINTS; i++ )
    {
        if( wifi_endpoints[i].used )
        {
            if( wifi_endpoints[i].type == a_type &&
                memcmp( wifi_endpoints[i].mac, a_mac, 6 ) == 0 )
            {
                oui_observe_mac( a_mac );
                return;
            }
        }
        else if( free_slot == MAX_WIFI_ENDPOINTS )
        {
            free_slot = i;
        }
    }

    if( free_slot < MAX_WIFI_ENDPOINTS )
    {
        memcpy( wifi_endpoints[free_slot].mac, a_mac, 6 );
        wifi_endpoints[free_slot].type = a_type;
        wifi_endpoints[free_slot].used = true;

        ESP_LOGI( TAG,
                  "%s observed: %02X:%02X:%02X:%02X:%02X:%02X ch=%u rssi=%d",
                  a_type == WIFI_OBSERVATION_AP ? "AP" : "Client",
                  a_mac[0], a_mac[1], a_mac[2],
                  a_mac[3], a_mac[4], a_mac[5],
                  a_channel, a_rssi );
    }

    oui_observe_mac( a_mac );
}

/**
 * @brief Match an observed MAC and count it once per unique address.
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
 */
static bool oui_remember_mac( const uint8_t *a_mac,
                              oui_category_t a_category )
{
    size_t free_slot = MAX_OBSERVED_MACS;

    for( size_t i = 0; i < MAX_OBSERVED_MACS; i++ )
    {
        if( observed_macs[i].used )
        {
            if( memcmp( observed_macs[i].mac, a_mac, 6 ) == 0 )
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

    memcpy( observed_macs[free_slot].mac, a_mac, 6 );
    observed_macs[free_slot].category = a_category;
    observed_macs[free_slot].used = true;
    return true;
}

/**
 * @brief Select the LCD color for a category count.
 */
static uint16_t category_color( uint16_t a_count )
{
    if( a_count == 0 )
    {
        return LCD_WHITE;
    }

    return a_count < CATEGORY_HIGH_COUNT ? LCD_YELLOW : LCD_MAGENTA;
}

/**
 * @brief Draw category counts, total count, and prevalence bars.
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
    ESP_ERROR_CHECK( lcd_draw_line( 2, STATUS_DIVIDER_Y,
                                    LCD_WIDTH - 3, STATUS_DIVIDER_Y,
                                    LCD_CYAN ) );

    for( size_t i = 0; i < CATEGORY_COUNT; i++ )
    {
        const int y = STATUS_FIRST_ROW_Y + (int)i * STATUS_ROW_HEIGHT;
        const uint16_t color = category_color( categories[i].count );

        lcd_set_cursor( STATUS_LABEL_X, y );
        ESP_ERROR_CHECK( lcd_draw_text( categories[i].name, color, LCD_BLACK ) );
        snprintf( text, sizeof( text ), "%u", categories[i].count );
        lcd_set_cursor( STATUS_COUNT_X, y );
        ESP_ERROR_CHECK( lcd_draw_text( text, color, LCD_BLACK ) );
        ESP_ERROR_CHECK( lcd_draw_line( STATUS_BAR_X, y + 6,
                                        STATUS_BAR_X + STATUS_BAR_WIDTH,
                                        y + 6, LCD_WHITE ) );

        if( categories[i].count > 0 )
        {
            int width = ( STATUS_BAR_WIDTH * categories[i].count ) / max_count;
            if( width < 1 )
            {
                width = 1;
            }
            ESP_ERROR_CHECK( lcd_fill_rect( STATUS_BAR_X, y + 4,
                                            width, STATUS_BAR_HEIGHT, color ) );
        }
    }

    return ESP_OK;
}

/**
 * @brief Refresh the LCD when its periodic update interval has elapsed.
 */
static void lcd_refresh_if_due( void )
{
    const TickType_t now = xTaskGetTickCount();

    if( ( now - lcd_last_refresh ) < pdMS_TO_TICKS( WIFI_LCD_REFRESH_MS ) )
    {
        return;
    }

    ESP_ERROR_CHECK( lcd_draw_status() );
    ESP_ERROR_CHECK( lcd_flush() );
    lcd_last_refresh = now;
}
