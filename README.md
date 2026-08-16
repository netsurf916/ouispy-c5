# donglescan

`donglescan` is an ESP-IDF application for the LILYGO T-Dongle C5. It scans nearby Wi-Fi access points across the supported 2.4 GHz and 5 GHz bands and displays per-channel counts as bar graphs on the built-in LCD.

## Hardware

- LILYGO T-Dongle C5
- ESP32-C5
- 160x80 ST7735 LCD
- On-board RGB LED
- USB Serial/JTAG interface for flashing and monitoring

## Development environment

The project is built with Espressif ESP-IDF. A typical development setup uses VS Code with the Espressif IDF extension, although the command-line tools are sufficient.

Make sure the ESP-IDF environment has been activated before using `idf.py`.

## Build

From the repository root:

```sh
idf.py set-target esp32c5
idf.py build
```

If the target has already been configured, only `idf.py build` is required.

## Flash and monitor

With the T-Dongle C5 connected as `/dev/ttyACM0`:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

Exit the ESP-IDF monitor with `Ctrl-]`.

## Wi-Fi scanning

The scanner performs passive scans on individual channels and records the number of observed access points. Separate result arrays are maintained for 2.4 GHz and 5 GHz channels.

The application scans an entire band before updating the display, then alternates to the other band. The completed graph remains visible while the opposite band is being scanned.

The current observation interval is controlled by `WIFI_OBSERVATION_MS` in `main/scan.c`.

### 2.4 GHz channels

The application scans channels 1 through 13.

### 5 GHz channels

The application uses an explicit channel list because 5 GHz Wi-Fi channel numbers are not contiguous:

```text
36 40 44 48
52 56 60 64
100 104 108 112 116 120 124 128 132 136 140 144
149 153 157 161 165
```

Availability of particular channels may still depend on regulatory configuration and the ESP-IDF Wi-Fi driver.

## LCD drawing

LCD rendering is framebuffer based. Drawing functions update an in-memory RGB565 framebuffer rather than writing directly to the panel.

Call:

```c
lcd_flush();
```

to transfer the completed frame to the display. This allows complex screens such as the Wi-Fi bar graph to be rendered off-screen and displayed as a single update, reducing visible flicker.

The LCD implementation provides primitives for:

- framebuffer fill
- pixels
- lines
- filled rectangles
- 5x7 text rendering with cursor support

## Source layout

```text
main/
    scan.c                Wi-Fi scanning and graph rendering
    t_dongle_lcd.c        LCD and framebuffer implementation
    t_dongle_lcd.h        LCD public interface
    t_dongle_rgb_led.c    On-board RGB LED support
    t_dongle_rgb_led.h    RGB LED public interface
```

## Notes

The project is currently focused on channel occupancy visualization rather than connecting to a wireless network. Counts represent access points observed during each passive scan interval, not associated Wi-Fi client stations.
