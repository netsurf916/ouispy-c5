# ouispy-c5

`ouispy-c5` is a passive Wi-Fi surveillance-device detector for the LILYGO T-Dongle C5 and ESP32-C5. It listens to 802.11 management and data frames in promiscuous mode, compares observed MAC prefixes against a small OUI database, and highlights matching device categories on the built-in LCD.

The project is inspired by the defensive detection work in [OUI-SPY](https://github.com/colonelpanichacks/oui-spy) and its [OUI detector database](https://github.com/colonelpanichacks/ouispy-detector).

## Current categories

The LCD displays these categories continuously:

- **FLOCK / ALPR** — Flock Safety infrastructure prefixes, including the Wi-Fi promiscuous-mode prefixes documented by the OUI-SPY project
- **BODY CAM** — Axon body-camera / law-enforcement OUI
- **DRONE** — DJI, Parrot, and Skydio OUIs
- **DOORBELL / CAM** — Ring doorbell and security-camera OUIs
- **SMARTGLASSES** — Meta / Ray-Ban smartglasses OUIs

A category is normally displayed in white. When a matching MAC is observed, that category changes to red for a short alert period. Repeated observations keep the alert active.

## Passive monitoring

The firmware does not associate with nearby networks. ESP-IDF promiscuous mode receives management and data frames while the application hops through configured 2.4 GHz and 5 GHz channels.

Both the receiver (`addr1`) and transmitter (`addr2`) addresses are checked. This follows the Flock promiscuous-mode research documented by OUI-SPY, where examining both addresses can improve detection of devices with burst/sleep behavior.

The detector ignores multicast/broadcast addresses before performing OUI matching.

## Limitations

OUI matching is an indicator, not proof of a particular device or activity. Vendors may share hardware manufacturers, MAC addresses can be randomized, devices may remain silent while a channel is being monitored, and regulatory settings can make some channels unavailable.

The current implementation is Wi-Fi only. Some OUI-SPY detections, particularly Flock/Raven and other devices, also use BLE fingerprints; BLE monitoring can be added separately.

## Hardware

- LILYGO T-Dongle C5
- ESP32-C5
- 160x80 ST7735 LCD
- USB Serial/JTAG interface

## Build

Activate the ESP-IDF environment, then from the repository root:

```sh
idf.py set-target esp32c5
idf.py build
```

## Flash and monitor

With the T-Dongle C5 connected as `/dev/ttyACM0`:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

Exit the ESP-IDF monitor with `Ctrl-]`.

## LCD rendering

The inherited T-Dongle LCD driver uses an RGB565 framebuffer. The detector renders the complete category status screen in memory and calls `lcd_flush()` to transfer it to the display.

## Source layout

```text
main/
    scan.c                Passive Wi-Fi monitoring and OUI matching
    t_dongle_lcd.c        LCD/framebuffer implementation
    t_dongle_lcd.h        LCD public interface
```

## OUI source

The initial OUI table is derived from `colonelpanichacks/ouispy-detector/ouis.md`. It includes Flock Safety, Axon, DJI, Parrot, Skydio, Ring, and Meta/Ray-Ban entries documented there.

This firmware is passive and detection-only. It does not transmit probe frames, associate with observed devices, or attempt to access their traffic contents.
