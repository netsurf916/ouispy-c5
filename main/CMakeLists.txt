idf_component_register(
    SRCS
        "scan.c"
        "wifi_monitor.c"
        "ble_monitor.c"
        "t_dongle_lcd.c"
    PRIV_REQUIRES
        bt
        esp_system
        esp_wifi
        nvs_flash
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_lcd
    INCLUDE_DIRS "."
)
