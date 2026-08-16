/**
    scan.c : T-Dongle C5 WiFi scanner
    Description: Scans 2.4 GHz and 5 GHz WiFi bands and displays channel
                 utilization as LCD bar graphs.
*/

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "t_dongle_lcd.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "scan";

#define WIFI_OBSERVATION_MS 200

static const uint8_t wifi_2g_channels[] =
{
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

#define WIFI_2G_CHANNEL_COUNT \
    ( sizeof( wifi_2g_channels ) / sizeof( wifi_2g_channels[0] ) )

static const uint8_t wifi_5g_channels[] =
{
    36, 40, 44, 48,
    52, 56, 60, 64,
    100, 104, 108, 112,
    116, 120, 124, 128,
    132, 136, 140, 144,
    149, 153, 157, 161, 165
};

#define WIFI_5G_CHANNEL_COUNT \
    ( sizeof( wifi_5g_channels ) / sizeof( wifi_5g_channels[0] ) )

/**
 * @brief Initialize ESP-IDF networking and the WiFi station interface.
 */
static void wifi_init( void );

/**
 * @brief Scan a WiFi band using an ESP-IDF channel bitmap.
 * @param a_band WiFi band to scan.
 * @param a_channels Channel numbers represented in the result array.
 * @param a_channel_count Number of entries in a_channels and a_counts.
 * @param a_observation_ms Passive observation time per channel in milliseconds.
 * @param a_counts Output array populated with AP counts for each channel.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 * @details The scan is performed once for the complete selected band. Returned
 *          AP records are grouped by their reported primary channel to produce
 *          the per-channel counts used by the LCD graph.
 */
static esp_err_t wifi_scan_band( wifi_band_t a_band,
                                 const uint8_t *a_channels,
                                 size_t a_channel_count,
                                 uint32_t a_observation_ms,
                                 uint16_t *a_counts );

/**
 * @brief Render a channel-count bar graph into the LCD framebuffer.
 * @param a_title Graph title.
 * @param a_channels Channel numbers represented by the bars.
 * @param a_values Access-point counts corresponding to a_channels.
 * @param a_count Number of channels and values.
 * @param a_bar_color RGB565 bar color.
 * @param a_axis_color RGB565 axis, grid, and label color.
 * @param a_background RGB565 background color.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_bar_graph( const char *a_title,
                                     const uint8_t *a_channels,
                                     const uint16_t *a_values,
                                     int a_count,
                                     uint16_t a_bar_color,
                                     uint16_t a_axis_color,
                                     uint16_t a_background );

/**
 * @brief Application entry point.
 * @details Initializes persistent storage, LCD, and WiFi, then continuously
 *          alternates complete bitmap-based 2.4 GHz and 5 GHz scans on the
 *          display.
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
    ESP_ERROR_CHECK( lcd_fill( LCD_BLACK ) );
    ESP_ERROR_CHECK( lcd_flush() );

    wifi_init();

    uint16_t channel_2g_count[WIFI_2G_CHANNEL_COUNT] = { 0 };
    uint16_t channel_5g_count[WIFI_5G_CHANNEL_COUNT] = { 0 };

    while( 1 )
    {
        ESP_ERROR_CHECK( wifi_scan_band( WIFI_BAND_2G,
                                         wifi_2g_channels,
                                         WIFI_2G_CHANNEL_COUNT,
                                         WIFI_OBSERVATION_MS,
                                         channel_2g_count ) );

        ESP_ERROR_CHECK( lcd_draw_bar_graph( "WiFi 2.4 GHz",
                                             wifi_2g_channels,
                                             channel_2g_count,
                                             WIFI_2G_CHANNEL_COUNT,
                                             LCD_GREEN,
                                             LCD_WHITE,
                                             LCD_BLACK ) );
        ESP_ERROR_CHECK( lcd_flush() );

        ESP_ERROR_CHECK( wifi_scan_band( WIFI_BAND_5G,
                                         wifi_5g_channels,
                                         WIFI_5G_CHANNEL_COUNT,
                                         WIFI_OBSERVATION_MS,
                                         channel_5g_count ) );

        ESP_ERROR_CHECK( lcd_draw_bar_graph( "WiFi 5 GHz",
                                             wifi_5g_channels,
                                             channel_5g_count,
                                             WIFI_5G_CHANNEL_COUNT,
                                             LCD_CYAN,
                                             LCD_WHITE,
                                             LCD_BLACK ) );
        ESP_ERROR_CHECK( lcd_flush() );
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
}

/**
 * @brief Scan a complete WiFi band and count APs by primary channel.
 * @param a_band WiFi band to scan.
 * @param a_channels Channel numbers represented in the result array.
 * @param a_channel_count Number of entries in a_channels and a_counts.
 * @param a_observation_ms Passive observation time per channel in milliseconds.
 * @param a_counts Output array populated with AP counts for each channel.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t wifi_scan_band( wifi_band_t a_band,
                                 const uint8_t *a_channels,
                                 size_t a_channel_count,
                                 uint32_t a_observation_ms,
                                 uint16_t *a_counts )
{
    if( a_channels == NULL || a_counts == NULL || a_channel_count == 0 )
    {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_band_mode_t band_mode;
    wifi_scan_config_t scan_config =
    {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time =
        {
            .passive = a_observation_ms,
        },
        .channel_bitmap =
        {
            .ghz_2_channels = 1U,
            .ghz_5_channels = 1U,
        },
    };

    memset( a_counts, 0, a_channel_count * sizeof( a_counts[0] ) );

    if( a_band == WIFI_BAND_2G )
    {
        band_mode = WIFI_BAND_MODE_2G_ONLY;
        scan_config.channel_bitmap.ghz_2_channels = 0;

        for( size_t i = 0; i < a_channel_count; i++ )
        {
            scan_config.channel_bitmap.ghz_2_channels |= CHANNEL_TO_BIT( a_channels[i] );
        }
    }
    else if( a_band == WIFI_BAND_5G )
    {
        band_mode = WIFI_BAND_MODE_5G_ONLY;
        scan_config.channel_bitmap.ghz_5_channels = 0;

        for( size_t i = 0; i < a_channel_count; i++ )
        {
            scan_config.channel_bitmap.ghz_5_channels |= CHANNEL_TO_BIT( a_channels[i] );
        }
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_wifi_set_band_mode( band_mode );
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG, "Unable to select WiFi band: %s", esp_err_to_name( err ) );
        return err;
    }

    err = esp_wifi_scan_start( &scan_config, true );
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG, "Band scan failed: %s", esp_err_to_name( err ) );
        return err;
    }

    uint16_t ap_count = 0;
    err = esp_wifi_scan_get_ap_num( &ap_count );
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG, "Unable to get AP count: %s", esp_err_to_name( err ) );
        return err;
    }

    if( ap_count == 0 )
    {
        ESP_LOGI( TAG, "%s scan found no APs",
                  a_band == WIFI_BAND_2G ? "2.4 GHz" : "5 GHz" );
        return ESP_OK;
    }

    wifi_ap_record_t *records = calloc( ap_count, sizeof( wifi_ap_record_t ) );
    if( records == NULL )
    {
        ESP_LOGE( TAG, "Unable to allocate %u AP records", ap_count );
        return ESP_ERR_NO_MEM;
    }

    uint16_t number = ap_count;
    err = esp_wifi_scan_get_ap_records( &number, records );
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG, "Unable to get AP records: %s", esp_err_to_name( err ) );
        free( records );
        return err;
    }

    for( uint16_t record = 0; record < number; record++ )
    {
        for( size_t channel = 0; channel < a_channel_count; channel++ )
        {
            if( records[record].primary == a_channels[channel] )
            {
                a_counts[channel]++;
                break;
            }
        }

        ESP_LOGI( TAG,
                  "%s channel=%u SSID=%s RSSI=%d",
                  a_band == WIFI_BAND_2G ? "2.4 GHz" : "5 GHz",
                  records[record].primary,
                  records[record].ssid,
                  records[record].rssi );
    }

    ESP_LOGI( TAG, "%s scan found %u APs",
              a_band == WIFI_BAND_2G ? "2.4 GHz" : "5 GHz", number );

    free( records );
    return ESP_OK;
}

/**
 * @brief Render a channel-count bar graph into the LCD framebuffer.
 * @param a_title Graph title.
 * @param a_channels Channel numbers represented by the bars.
 * @param a_values Access-point counts corresponding to a_channels.
 * @param a_count Number of channels and values.
 * @param a_bar_color RGB565 bar color.
 * @param a_axis_color RGB565 axis, grid, and label color.
 * @param a_background RGB565 background color.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
static esp_err_t lcd_draw_bar_graph( const char *a_title,
                                     const uint8_t *a_channels,
                                     const uint16_t *a_values,
                                     int a_count,
                                     uint16_t a_bar_color,
                                     uint16_t a_axis_color,
                                     uint16_t a_background )
{
    if( a_title == NULL || a_channels == NULL || a_values == NULL || a_count <= 0 )
    {
        return ESP_ERR_INVALID_ARG;
    }

    const int graph_x = 8;
    const int graph_y = 10;
    const int graph_width = LCD_WIDTH - graph_x - 1;
    const int graph_height = LCD_HEIGHT - graph_y - 16;
    const int graph_bottom = graph_y + graph_height - 1;
    const int graph_right = graph_x + graph_width - 1;

    uint16_t max_value = 1;
    for( int i = 0; i < a_count; i++ )
    {
        if( a_values[i] > max_value )
        {
            max_value = a_values[i];
        }
    }

    ESP_ERROR_CHECK( lcd_fill( a_background ) );

    lcd_set_cursor( 4, 0 );
    ESP_ERROR_CHECK( lcd_draw_text( a_title, a_axis_color, a_background ) );

    ESP_ERROR_CHECK( lcd_draw_line( graph_x, graph_y,
                                    graph_x, graph_bottom, a_axis_color ) );
    ESP_ERROR_CHECK( lcd_draw_line( graph_x, graph_bottom,
                                    graph_right, graph_bottom, a_axis_color ) );

    const int divisions = 4;
    for( int i = 0; i <= divisions; i++ )
    {
        int y = graph_bottom - ( ( graph_height - 1 ) * i / divisions );
        ESP_ERROR_CHECK( lcd_draw_line( graph_x - 2, y, graph_x, y, a_axis_color ) );

        if( i > 0 )
        {
            for( int x = graph_x + 2; x <= graph_right; x += 3 )
            {
                ESP_ERROR_CHECK( lcd_draw_pixel( x, y, a_axis_color ) );
            }
        }
    }

    char label[8];
    snprintf( label, sizeof( label ), "%u", max_value );
    lcd_set_cursor( 0, graph_y );
    ESP_ERROR_CHECK( lcd_draw_text( label, a_axis_color, a_background ) );

    int slot_width = ( graph_width - 2 ) / a_count;
    if( slot_width < 1 )
    {
        slot_width = 1;
    }

    for( int i = 0; i < a_count; i++ )
    {
        int bar_height = ( ( graph_height - 2 ) * a_values[i] ) / max_value;
        int x = graph_x + 2 + i * slot_width;
        int bar_width = slot_width - 1;

        if( bar_width < 1 )
        {
            bar_width = 1;
        }
        if( x + bar_width > graph_right )
        {
            bar_width = graph_right - x;
        }

        if( bar_height > 0 && bar_width > 0 )
        {
            int y = graph_bottom - bar_height;
            ESP_ERROR_CHECK( lcd_fill_rect( x, y, bar_width,
                                            bar_height, a_bar_color ) );
        }
    }

    for( int i = 0; i < a_count; i++ )
    {
        int x = graph_x + 2 + i * slot_width + slot_width / 2;
        ESP_ERROR_CHECK( lcd_draw_line( x, graph_bottom,
                                        x, graph_bottom + 2, a_axis_color ) );
    }

    int min_label_spacing = 16;
    int last_label_right = -min_label_spacing;

    for( int i = 0; i < a_count; i+=2 )
    {
        int x = graph_x + 2 + i * slot_width + slot_width / 2;
        snprintf( label, sizeof( label ), "%u", a_channels[i] );

        int text_width = strlen( label ) * 6;
        int text_x = x - text_width / 2;
        if( text_x < 0 )
        {
            text_x = 0;
        }
        if( text_x + text_width > LCD_WIDTH )
        {
            text_x = LCD_WIDTH - text_width;
        }

        if( i != 0 && i != a_count - 1 && text_x < last_label_right + 2 )
        {
            continue;
        }

        lcd_set_cursor( text_x, graph_bottom + 4 );
        ESP_ERROR_CHECK( lcd_draw_text( label, a_axis_color, a_background ) );
        last_label_right = text_x + text_width;
    }

    return ESP_OK;
}
