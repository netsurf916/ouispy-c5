/**
    t_dongle_rgb_led.c : T-Dongle C5 RGB LED implementation
    Description: Bit-banged APA102-2020 control for the on-board RGB LED.
*/

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#include "t_dongle_rgb_led.h"

static bool rgb_led_initialized;

/**
 * @brief Write one byte to the APA102 data interface.
 * @param a_value Byte to transmit, most-significant bit first.
 */
static void rgb_led_write_byte( uint8_t a_value )
{
    for( uint8_t mask = 0x80; mask != 0; mask >>= 1 )
    {
        gpio_set_level( RGB_LED_CLOCK_PIN, 0 );
        gpio_set_level( RGB_LED_DATA_PIN, ( a_value & mask ) != 0 );
        gpio_set_level( RGB_LED_CLOCK_PIN, 1 );
    }
}

/**
 * @brief Write a complete APA102 frame for the on-board LED.
 * @param a_red Red intensity.
 * @param a_green Green intensity.
 * @param a_blue Blue intensity.
 */
static void rgb_led_write_frame( uint8_t a_red, uint8_t a_green,
                                 uint8_t a_blue )
{
    for( int i = 0; i < 4; i++ )
    {
        rgb_led_write_byte( 0x00 );
    }

    rgb_led_write_byte( 0xFF );
    rgb_led_write_byte( a_blue );
    rgb_led_write_byte( a_green );
    rgb_led_write_byte( a_red );

    for( int i = 0; i < 4; i++ )
    {
        rgb_led_write_byte( 0xFF );
    }

    gpio_set_level( RGB_LED_CLOCK_PIN, 0 );
    gpio_set_level( RGB_LED_DATA_PIN, 0 );
}

esp_err_t rgb_led_init( void )
{
    const gpio_config_t gpio_cfg =
    {
        .pin_bit_mask = ( 1ULL << RGB_LED_CLOCK_PIN ) |
                        ( 1ULL << RGB_LED_DATA_PIN ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config( &gpio_cfg );
    if( err != ESP_OK )
    {
        return err;
    }

    rgb_led_initialized = true;

    return rgb_led_off();
}

esp_err_t rgb_led_set( uint8_t a_red, uint8_t a_green, uint8_t a_blue )
{
    if( !rgb_led_initialized )
    {
        return ESP_ERR_INVALID_STATE;
    }

    rgb_led_write_frame( a_red, a_green, a_blue );

    return ESP_OK;
}

esp_err_t rgb_led_off( void )
{
    return rgb_led_set( 0, 0, 0 );
}
