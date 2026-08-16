idf_component_register(
    SRCS
        "scan.c"
        "t_dongle_lcd.c"
    PRIV_REQUIRES
        esp_wifi
        nvs_flash
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_lcd
    INCLUDE_DIRS "."
)
