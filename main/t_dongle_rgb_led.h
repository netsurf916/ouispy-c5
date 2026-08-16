/**
    t_dongle_rgb_led.h : T-Dongle C5 RGB LED interface
    Description: Interface for the on-board APA102-2020 RGB LED.
*/

#pragma once

#include <stdint.h>

#include "esp_err.h"

#define RGB_LED_CLOCK_PIN 4
#define RGB_LED_DATA_PIN  5

/**
 * @brief Initialize the on-board APA102-2020 RGB LED interface.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t rgb_led_init( void );

/**
 * @brief Set the on-board RGB LED color.
 * @param a_red Red intensity from 0 through 255.
 * @param a_green Green intensity from 0 through 255.
 * @param a_blue Blue intensity from 0 through 255.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t rgb_led_set( uint8_t a_red, uint8_t a_green, uint8_t a_blue );

/**
 * @brief Turn off the on-board RGB LED.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t rgb_led_off( void );
