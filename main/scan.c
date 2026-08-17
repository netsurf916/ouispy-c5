/**
    scan.c : T-Dongle C5 passive surveillance-device detector
    Description: Matches WiFi and BLE observations against device signatures
                 and maintains the LCD status display.
    Copyright 2026 Daniel Wilson
    SPDX-License-Identifier: MIT
*/

#include "ble_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "t_dongle_lcd.h"
#include "wifi_monitor.h"

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "ouispy";

#define LCD_REFRESH_MS          750
#define MAX_OBSERVED_DEVICES    256
#define MAX_WIFI_ENDPOINTS      256
#define MAX_BLE_ENDPOINTS       256
#define CATEGORY_HIGH_COUNT     6
#define STATUS_DIVIDER_Y        9
#define STATUS_FIRST_ROW_Y      13
#define STATUS_ROW_HEIGHT       13
#define STATUS_LABEL_X          3
#define STATUS_COUNT_X          91
#define STATUS_BAR_X            108
#define STATUS_BAR_WIDTH        49
#define STATUS_BAR_HEIGHT       5

typedef enum
{
    CATEGORY_FLOCK = 0,
    CATEGORY_BODY_CAM,
    CATEGORY_DRONE,
    CATEGORY_DOORBELL_CAMERA,
    CATEGORY_SMARTGLASSES,
    CATEGORY_COUNT
} detector_category_t;

typedef enum
{
    DETECTOR_SOURCE_WIFI = 0,
    DETECTOR_SOURCE_BLE
} detector_source_t;

typedef struct
{
    uint8_t             prefix[3];
    detector_category_t category;
} oui_entry_t;

typedef struct
{
    const char        *name;
    volatile uint16_t  count;
} category_state_t;

typedef struct
{
    detector_source_t   source;
    uint8_t             address[6];
    uint8_t             address_type;
    detector_category_t category;
    bool                used;
} observed_device_t;

typedef struct
{
    uint8_t                 mac[6];
    wifi_observation_type_t type;
    bool                    used;
} wifi_endpoint_t;

typedef struct
{
    uint8_t address[6];
    uint8_t address_type;
    bool    used;
} ble_endpoint_t;

typedef struct
{
    const char         *name_fragment;
    detector_category_t category;
} ble_name_signature_t;

typedef struct
{
    uint16_t            company_id;
    const uint8_t      *prefix;
    size_t              prefix_length;
    detector_category_t category;
} ble_manufacturer_signature_t;

static category_state_t categories[CATEGORY_COUNT] =
{
    [CATEGORY_FLOCK]           = { "FLOCK / ALPR", 0 },
    [CATEGORY_BODY_CAM]        = { "BODY CAM", 0 },
    [CATEGORY_DRONE]           = { "DRONE", 0 },
    [CATEGORY_DOORBELL_CAMERA] = { "DOORBELL / CAM", 0 },
    [CATEGORY_SMARTGLASSES]    = { "SMARTGLASSES", 0 },
};

static observed_device_t observed_devices[MAX_OBSERVED_DEVICES];
static wifi_endpoint_t wifi_endpoints[MAX_WIFI_ENDPOINTS];
static ble_endpoint_t ble_endpoints[MAX_BLE_ENDPOINTS];
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

/*
 * BLE signatures intentionally use narrow product/vendor identifiers. Generic
 * company identifiers alone are not used for broad vendors because that would
 * turn unrelated phones, computers, and accessories into detector matches.
 */
static const ble_name_signature_t ble_name_signatures[] =
{
    { "AXON",        CATEGORY_BODY_CAM },
    { "BODY",        CATEGORY_BODY_CAM },
    { "DJI",         CATEGORY_DRONE },
    { "PARROT",      CATEGORY_DRONE },
    { "SKYDIO",      CATEGORY_DRONE },
    { "RING",        CATEGORY_DOORBELL_CAMERA },
    { "RAY-BAN",     CATEGORY_SMARTGLASSES },
    { "RAYBAN",      CATEGORY_SMARTGLASSES },
    { "VUZI",        CATEGORY_SMARTGLASSES },
    { "SMART GLASS", CATEGORY_SMARTGLASSES },
};

#define BLE_NAME_SIGNATURE_COUNT \
    ( sizeof( ble_name_signatures ) / sizeof( ble_name_signatures[0] ) )

static void wifi_observation( wifi_observation_type_t a_type,
                              const uint8_t *a_mac,
                              uint8_t a_channel,
                              int8_t a_rssi );
static void ble_observation( const ble_observation_t *a_observation );
static bool detector_match_oui( const uint8_t *a_mac,
                                detector_category_t *a_category );
static bool detector_match_ble( const ble_observation_t *a_observation,
                                detector_category_t *a_category );
static bool detector_remember( detector_source_t a_source,
                               const uint8_t *a_address,
                               uint8_t a_address_type,
                               detector_category_t a_category );
static bool string_contains_case_insensitive( const char *a_value,
                                              const char *a_fragment );
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
    ESP_ERROR_CHECK( ble_monitor_init( ble_observation ) );

    while( 1 )
    {
        wifi_monitor_step();
        lcd_refresh_if_due();
    }
}

/**
 * @brief Consume a classified AP or client observation from wifi_monitor.
 * @param a_type Inferred WiFi endpoint type.
 * @param a_mac Six-byte MAC address.
 * @param a_channel Receive channel.
 * @param a_rssi Received signal strength in dBm.
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

    detector_category_t category;

    if( detector_match_oui( a_mac, &category ) &&
        detector_remember( DETECTOR_SOURCE_WIFI, a_mac, 0, category ) )
    {
        categories[category].count++;
        ESP_LOGI( TAG,
                  "New %s WiFi device (%u total)",
                  categories[category].name,
                  categories[category].count );
    }
}

/**
 * @brief Consume and classify one parsed BLE advertisement.
 * @param a_observation Parsed BLE advertiser and fingerprint fields.
 */
static void ble_observation( const ble_observation_t *a_observation )
{
    if( a_observation == NULL )
    {
        return;
    }

    size_t free_slot = MAX_BLE_ENDPOINTS;
    bool known = false;

    for( size_t i = 0; i < MAX_BLE_ENDPOINTS; i++ )
    {
        if( ble_endpoints[i].used )
        {
            if( ble_endpoints[i].address_type == a_observation->address_type &&
                memcmp( ble_endpoints[i].address,
                        a_observation->address,
                        sizeof( ble_endpoints[i].address ) ) == 0 )
            {
                known = true;
                break;
            }
        }
        else if( free_slot == MAX_BLE_ENDPOINTS )
        {
            free_slot = i;
        }
    }

    if( !known && free_slot < MAX_BLE_ENDPOINTS )
    {
        memcpy( ble_endpoints[free_slot].address,
                a_observation->address,
                sizeof( ble_endpoints[free_slot].address ) );
        ble_endpoints[free_slot].address_type = a_observation->address_type;
        ble_endpoints[free_slot].used = true;

        ESP_LOGI( TAG,
                  "BLE observed: %02X:%02X:%02X:%02X:%02X:%02X type=%u rssi=%d%s%s",
                  a_observation->address[0], a_observation->address[1],
                  a_observation->address[2], a_observation->address[3],
                  a_observation->address[4], a_observation->address[5],
                  a_observation->address_type,
                  a_observation->rssi,
                  a_observation->has_name ? " name=" : "",
                  a_observation->has_name ? a_observation->name : "" );
    }

    detector_category_t category;

    if( detector_match_ble( a_observation, &category ) &&
        detector_remember( DETECTOR_SOURCE_BLE,
                           a_observation->address,
                           a_observation->address_type,
                           category ) )
    {
        categories[category].count++;
        ESP_LOGI( TAG,
                  "New %s BLE device (%u total)",
                  categories[category].name,
                  categories[category].count );
    }
}

/**
 * @brief Match a WiFi/public MAC address against the OUI database.
 * @param a_mac Six-byte MAC address.
 * @param a_category Receives the matched category.
 * @return True when an OUI match is found; false otherwise.
 */
static bool detector_match_oui( const uint8_t *a_mac,
                                detector_category_t *a_category )
{
    if( a_mac == NULL || a_category == NULL || ( a_mac[0] & 0x01U ) != 0 )
    {
        return false;
    }

    for( size_t i = 0; i < OUI_DATABASE_COUNT; i++ )
    {
        if( memcmp( a_mac, oui_database[i].prefix, 3 ) == 0 )
        {
            *a_category = oui_database[i].category;
            return true;
        }
    }

    return false;
}

/**
 * @brief Match a BLE advertisement using stable signature data first.
 * @param a_observation Parsed BLE advertiser and fingerprint fields.
 * @param a_category Receives the matched category.
 * @return True when a signature or safe public-address OUI matches.
 * @details Device-name signatures are intentionally product-specific. A
 *          public BLE address may fall back to the existing OUI database.
 *          Random/private addresses never use OUI matching.
 */
static bool detector_match_ble( const ble_observation_t *a_observation,
                                detector_category_t *a_category )
{
    if( a_observation == NULL || a_category == NULL )
    {
        return false;
    }

    if( a_observation->has_name )
    {
        for( size_t i = 0; i < BLE_NAME_SIGNATURE_COUNT; i++ )
        {
            if( string_contains_case_insensitive(
                    a_observation->name,
                    ble_name_signatures[i].name_fragment ) )
            {
                *a_category = ble_name_signatures[i].category;
                return true;
            }
        }
    }

    /* BLE public address type is zero in the NimBLE host API. */
    if( a_observation->address_type == 0 )
    {
        return detector_match_oui( a_observation->address, a_category );
    }

    return false;
}

/**
 * @brief Remember a categorized radio device once.
 * @param a_source Radio source used to observe the device.
 * @param a_address Six-byte WiFi/BLE address.
 * @param a_address_type BLE address type; zero for WiFi.
 * @param a_category Matched detector category.
 * @return True when the device was newly stored; false if already counted.
 */
static bool detector_remember( detector_source_t a_source,
                               const uint8_t *a_address,
                               uint8_t a_address_type,
                               detector_category_t a_category )
{
    size_t free_slot = MAX_OBSERVED_DEVICES;

    for( size_t i = 0; i < MAX_OBSERVED_DEVICES; i++ )
    {
        if( observed_devices[i].used )
        {
            if( observed_devices[i].source == a_source &&
                observed_devices[i].address_type == a_address_type &&
                memcmp( observed_devices[i].address, a_address, 6 ) == 0 )
            {
                return false;
            }
        }
        else if( free_slot == MAX_OBSERVED_DEVICES )
        {
            free_slot = i;
        }
    }

    if( free_slot == MAX_OBSERVED_DEVICES )
    {
        return false;
    }

    observed_devices[free_slot].source = a_source;
    memcpy( observed_devices[free_slot].address, a_address, 6 );
    observed_devices[free_slot].address_type = a_address_type;
    observed_devices[free_slot].category = a_category;
    observed_devices[free_slot].used = true;

    return true;
}

/**
 * @brief Test whether one ASCII string contains another ignoring case.
 * @param a_value String to search.
 * @param a_fragment String fragment to locate.
 * @return True when a_fragment occurs within a_value; false otherwise.
 */
static bool string_contains_case_insensitive( const char *a_value,
                                              const char *a_fragment )
{
    if( a_value == NULL || a_fragment == NULL || a_fragment[0] == '\0' )
    {
        return false;
    }

    const size_t fragment_length = strlen( a_fragment );

    for( const char *value = a_value; *value != '\0'; value++ )
    {
        size_t i = 0;

        while( i < fragment_length && value[i] != '\0' &&
               toupper( (unsigned char)value[i] ) ==
               toupper( (unsigned char)a_fragment[i] ) )
        {
            i++;
        }

        if( i == fragment_length )
        {
            return true;
        }
    }

    return false;
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

    return a_count < CATEGORY_HIGH_COUNT ? LCD_YELLOW : LCD_MAGENTA;
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
 * @brief Refresh the LCD when its periodic update interval has elapsed.
 */
static void lcd_refresh_if_due( void )
{
    const TickType_t now = xTaskGetTickCount();

    if( ( now - lcd_last_refresh ) < pdMS_TO_TICKS( LCD_REFRESH_MS ) )
    {
        return;
    }

    ESP_ERROR_CHECK( lcd_draw_status() );
    ESP_ERROR_CHECK( lcd_flush() );
    lcd_last_refresh = now;
}
