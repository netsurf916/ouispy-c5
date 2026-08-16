#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7735.h"

#include "t_dongle_lcd.h"

#include <stdlib.h>

static const char *TAG = "lcd";
static esp_lcd_panel_handle_t panel = NULL;
static volatile bool color_transfer_done = false;
static uint16_t lcd_framebuffer[LCD_WIDTH * LCD_HEIGHT];

static const uint8_t font_width = 6;
static const uint8_t font_height = 9;
static const uint8_t line_start_x = 1;
static const uint8_t line_start_y = 4;
static int lcd_cursor_x = line_start_x;
static int lcd_cursor_y = line_start_y;

/* Printable ASCII 0x20 through 0x7e.  Each glyph is five columns, bit 0 top. */
static const uint8_t font_5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08}
};

static bool lcd_color_transfer_done(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    *(volatile bool *)user_ctx = true;
    return false;
}

static esp_err_t lcd_panel_draw_bitmap(int x_start, int y_start, int x_end,
                                       int y_end, const void *pixels)
{
    if (panel == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    color_transfer_done = false;
    esp_err_t err = esp_lcd_panel_draw_bitmap(panel, x_start, y_start,
                                               x_end, y_end, pixels);
    if (err != ESP_OK) {
        return err;
    }

    while (!color_transfer_done) {
    }
    return ESP_OK;
}

static esp_err_t lcd_buffer_draw_bitmap(int x_start, int y_start, int x_end,
                                        int y_end, const uint16_t *pixels)
{
    if (pixels == NULL || x_start < 0 || y_start < 0 ||
        x_end > LCD_WIDTH || y_end > LCD_HEIGHT ||
        x_start >= x_end || y_start >= y_end) {
        return ESP_ERR_INVALID_ARG;
    }

    const int width = x_end - x_start;
    for (int y = y_start; y < y_end; y++) {
        for (int x = 0; x < width; x++) {
            lcd_framebuffer[y * LCD_WIDTH + x_start + x] =
                pixels[(y - y_start) * width + x];
        }
    }
    return ESP_OK;
}

esp_err_t lcd_init(void)
{
    esp_err_t err;
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << LCD_PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bl_cfg));
    gpio_set_level(LCD_PIN_BL, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = LCD_PIN_MISO,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
    };
    err = spi_bus_initialize(LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                   &io_cfg, &io);
    if (err != ESP_OK) return err;

    const esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = lcd_color_transfer_done,
    };
    err = esp_lcd_panel_io_register_event_callbacks(
        io, &io_callbacks, (void *)&color_transfer_done);
    if (err != ESP_OK) return err;

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7735(io, &panel_cfg, &panel);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, LCD_X_GAP, LCD_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    lcd_fill(LCD_BLACK);
    ESP_ERROR_CHECK(lcd_flush());
    gpio_set_level(LCD_PIN_BL, 0);
    ESP_LOGI(TAG, "T-Dongle-C5 LCD initialized with framebuffer");
    return ESP_OK;
}

esp_err_t lcd_flush(void)
{
    return lcd_panel_draw_bitmap(0, 0, LCD_WIDTH, LCD_HEIGHT, lcd_framebuffer);
}

esp_err_t lcd_fill(uint16_t color)
{
    for (size_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        lcd_framebuffer[i] = color;
    }
    return ESP_OK;
}

void lcd_set_cursor(int x, int y) { lcd_cursor_x = x; lcd_cursor_y = y; }
void lcd_get_cursor(int *x, int *y) { if (x) *x = lcd_cursor_x; if (y) *y = lcd_cursor_y; }
void lcd_cursor_reset(void) { lcd_cursor_x = line_start_x; lcd_cursor_y = line_start_y; }
void lcd_cursor_linefeed(void)
{
    lcd_cursor_y += font_height;
    if ((lcd_cursor_y + font_height) >= LCD_HEIGHT) lcd_cursor_y = line_start_y;
}
void lcd_cursor_carriagereturn(void) { lcd_cursor_x = line_start_x; }
void lcd_cursor_crlf(void) { lcd_cursor_linefeed(); lcd_cursor_carriagereturn(); }
void lcd_update_cursor(void)
{
    lcd_cursor_x += font_width;
    if (lcd_cursor_x + font_width > LCD_WIDTH) {
        lcd_cursor_x = line_start_x;
        lcd_cursor_linefeed();
    }
}

esp_err_t lcd_draw_char(char c, uint16_t fg, uint16_t bg)
{
    if (c < 0x20 || c > 0x7E) c = '?';

    const int left_spacing = lcd_cursor_x == line_start_x ? 1 : 0;
    const int draw_x = lcd_cursor_x - left_spacing;
    const int draw_width = font_width + left_spacing;
    if (draw_x < 0 || lcd_cursor_y < 0 ||
        draw_x + draw_width > LCD_WIDTH ||
        lcd_cursor_y + font_height > LCD_HEIGHT) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint8_t *glyph = font_5x7[(uint8_t)c - 0x20];
    uint16_t pixels[(font_width + 1) * font_height];
    for (int py = 0; py < font_height; py++) {
        for (int px = 0; px < draw_width; px++) {
            uint16_t color = bg;
            const int glyph_x = px - left_spacing;
            if (glyph_x >= 0 && glyph_x < (font_width - 1) &&
                py > 0 && py < (font_height - 1) &&
                (glyph[glyph_x] & (1U << (py - 1)))) {
                color = fg;
            }
            pixels[py * draw_width + px] = color;
        }
    }

    esp_err_t err = lcd_buffer_draw_bitmap(draw_x, lcd_cursor_y,
                                            draw_x + draw_width,
                                            lcd_cursor_y + font_height,
                                            pixels);
    if (err == ESP_OK) lcd_update_cursor();
    return err;
}

esp_err_t lcd_draw_text(const char *text, uint16_t fg, uint16_t bg)
{
    if (text == NULL) return ESP_ERR_INVALID_ARG;
    while (*text != '\0') {
        if (*text == '\n') { lcd_cursor_crlf(); text++; continue; }
        if (*text == '\r') { lcd_cursor_carriagereturn(); text++; continue; }
        esp_err_t err = lcd_draw_char(*text++, fg, bg);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t lcd_draw_pixel(int x, int y, uint16_t color)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT)
        return ESP_ERR_INVALID_ARG;
    lcd_framebuffer[y * LCD_WIDTH + x] = color;
    return ESP_OK;
}

esp_err_t lcd_fill_rect(int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0 || x < 0 || y < 0 ||
        x + width > LCD_WIDTH || y + height > LCD_HEIGHT)
        return ESP_ERR_INVALID_ARG;

    for (int py = y; py < y + height; py++) {
        uint16_t *row = &lcd_framebuffer[py * LCD_WIDTH + x];
        for (int px = 0; px < width; px++) row[px] = color;
    }
    return ESP_OK;
}

esp_err_t lcd_draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        esp_err_t ret = lcd_draw_pixel(x0, y0, color);
        if (ret != ESP_OK) return ret;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return ESP_OK;
}
