/**
    t_dongle_lcd.h : T-Dongle C5 LCD interface
    Description: Framebuffer-backed drawing interface for the ST7735 LCD.
*/

#pragma once

#include <stdint.h>

#include "esp_err.h"

#define LCD_BLACK       0x0000
#define LCD_WHITE       0xFFFF
#define LCD_RED         0x00F8
#define LCD_GREEN       0xE007
#define LCD_BLUE        0x1F00
#define LCD_YELLOW      0xE0FF
#define LCD_CYAN        0xFF07
#define LCD_MAGENTA     0x1FF8

#define LCD_HOST        SPI2_HOST

#define LCD_PIN_MOSI    2
#define LCD_PIN_MISO    7
#define LCD_PIN_SCLK    6
#define LCD_PIN_CS      10
#define LCD_PIN_DC      3
#define LCD_PIN_RST     1
#define LCD_PIN_BL      0

#define LCD_WIDTH       160
#define LCD_HEIGHT      80

/**
 * @brief ST7735 GRAM X offset for the rotated 160x80 display.
 */
#define LCD_X_GAP       1

/**
 * @brief ST7735 GRAM Y offset for the rotated 160x80 display.
 */
#define LCD_Y_GAP       26

/**
 * @brief Initialize the T-Dongle C5 LCD and framebuffer.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t lcd_init( void );

/**
 * @brief Transfer the current framebuffer to the LCD panel.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t lcd_flush( void );

/**
 * @brief Fill the framebuffer with a single color.
 * @param a_color RGB565 display color.
 * @return ESP_OK on success.
 */
esp_err_t lcd_fill( uint16_t a_color );

/**
 * @brief Set the text cursor position.
 * @param a_x X coordinate in pixels.
 * @param a_y Y coordinate in pixels.
 */
void lcd_set_cursor( int a_x, int a_y );

/**
 * @brief Retrieve the current text cursor position.
 * @param a_x Optional output pointer for the X coordinate.
 * @param a_y Optional output pointer for the Y coordinate.
 */
void lcd_get_cursor( int *a_x, int *a_y );

/**
 * @brief Reset the cursor to the configured starting position.
 */
void lcd_cursor_reset( void );

/**
 * @brief Advance the cursor by one text row.
 */
void lcd_cursor_linefeed( void );

/**
 * @brief Return the cursor to the beginning of the current row.
 */
void lcd_cursor_carriagereturn( void );

/**
 * @brief Advance the cursor to the beginning of the next row.
 */
void lcd_cursor_crlf( void );

/**
 * @brief Advance the cursor by one character cell, wrapping as needed.
 */
void lcd_update_cursor( void );

/**
 * @brief Draw one printable character into the framebuffer.
 * @param a_character Character to draw; unsupported values render as '?'.
 * @param a_foreground Foreground RGB565 color.
 * @param a_background Background RGB565 color.
 * @return ESP_OK on success; ESP_ERR_INVALID_SIZE if the glyph does not fit.
 */
esp_err_t lcd_draw_char( char a_character, uint16_t a_foreground,
                         uint16_t a_background );

/**
 * @brief Draw a null-terminated text string into the framebuffer.
 * @param a_text Text to draw.
 * @param a_foreground Foreground RGB565 color.
 * @param a_background Background RGB565 color.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t lcd_draw_text( const char *a_text, uint16_t a_foreground,
                         uint16_t a_background );

/**
 * @brief Draw one pixel into the framebuffer.
 * @param a_x X coordinate in pixels.
 * @param a_y Y coordinate in pixels.
 * @param a_color RGB565 color.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG for an invalid coordinate.
 */
esp_err_t lcd_draw_pixel( int a_x, int a_y, uint16_t a_color );

/**
 * @brief Draw a line into the framebuffer using Bresenham's algorithm.
 * @param a_x0 Starting X coordinate.
 * @param a_y0 Starting Y coordinate.
 * @param a_x1 Ending X coordinate.
 * @param a_y1 Ending Y coordinate.
 * @param a_color RGB565 color.
 * @return ESP_OK on success; an ESP-IDF error code on failure.
 */
esp_err_t lcd_draw_line( int a_x0, int a_y0, int a_x1, int a_y1,
                         uint16_t a_color );

/**
 * @brief Fill a rectangular area in the framebuffer.
 * @param a_x Left coordinate.
 * @param a_y Top coordinate.
 * @param a_width Rectangle width in pixels.
 * @param a_height Rectangle height in pixels.
 * @param a_color RGB565 color.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG for invalid bounds.
 */
esp_err_t lcd_fill_rect( int a_x, int a_y, int a_width, int a_height,
                         uint16_t a_color );
