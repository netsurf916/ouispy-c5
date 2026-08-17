# ouispy-c5

`ouispy-c5` is a passive Wi-Fi and Bluetooth Low Energy surveillance-device detector for the LILYGO T-Dongle C5 / ESP32-C5. It observes nearby 802.11 traffic and BLE advertisements without associating with devices, compares the observations against a deliberately narrow set of device signatures, and summarizes matching categories on the built-in LCD.

The project is inspired by the defensive detection work in [OUI-SPY](https://github.com/colonelpanichacks/oui-spy) and its [OUI detector database](https://github.com/colonelpanichacks/ouispy-detector).

## Current categories

The LCD continuously displays:

- **FLOCK / ALPR** — Flock Safety / ALPR Wi-Fi prefixes documented by OUI-SPY
- **BODY CAM** — Axon and related narrow body-camera signatures
- **DRONE** — DJI, Parrot, and Skydio signatures
- **DOORBELL / CAM** — Ring doorbell and security-camera signatures
- **SMARTGLASSES** — Meta / Ray-Ban and Vuzix signatures

Each category includes a count of unique matching devices and a horizontal prevalence bar. Categories with no detections are white, counts from 1 through 5 are yellow, and counts of 6 or more are magenta. The title also shows the total number of categorized devices observed.

Counts are based on unique observed addresses rather than packet or advertisement volume, so a chatty device does not continuously increase the displayed total.

## Wi-Fi monitoring

Wi-Fi monitoring uses ESP-IDF promiscuous mode and does not associate with nearby access points.

The monitor hops across configured 2.4 GHz and 5 GHz channels and classifies both access points and clients from 802.11 management and data frames:

- beacon and probe-response transmitters are classified as access points
- probe-request transmitters are classified as clients
- infrastructure data frames use the 802.11 `To DS` / `From DS` direction bits to identify the AP/BSSID and client endpoints

Newly observed Wi-Fi endpoints are logged with their type, MAC address, channel, and RSSI. Categorization is then performed against the Wi-Fi OUI database.

The channel plan attempts the broad ESP32-C5 range. Channels rejected by the active ESP-IDF/regulatory configuration are skipped rather than treated as fatal errors.

### Channel and display timing

The Wi-Fi monitor is optimized for passive collection rather than screen updates on every channel transition:

- 2.4 GHz channel dwell: approximately 80 ms
- 5 GHz channel dwell: approximately 60 ms
- LCD refresh: approximately 750 ms

LCD updates are independent of channel hopping so display I/O does not consume every dwell interval.

## BLE monitoring

BLE monitoring uses NimBLE in passive observer mode. The application scans advertisements continuously while ESP-IDF software coexistence shares the ESP32-C5 radio between Wi-Fi and BLE.

BLE advertisements are parsed for:

- public or random/private advertiser address
- RSSI
- local/advertised device name
- manufacturer company identifier and manufacturer-specific data
- 16-bit service UUIDs
- 128-bit service UUIDs

BLE duplicate filtering is disabled so changing advertisements from the same device remain available to the detector.

Because BLE devices frequently use randomized/private addresses, BLE matching does not rely on OUIs alone. Product-specific advertised-name signatures are currently used for narrow device families such as Axon/body cameras, DJI/Parrot/Skydio drones, Ring devices, and Ray-Ban/Vuzix smart glasses. Public BLE addresses may also fall back to the Wi-Fi OUI database; random/private BLE addresses are not treated as meaningful OUIs.

The BLE parser also exposes manufacturer data and service UUIDs so more precise fingerprints can be added as verified signatures become available.

## Detection philosophy

The signature database is intentionally conservative. Broad manufacturers that sell many unrelated kinds of electronics are generally avoided because an OUI or company identifier from such a vendor would create too many false positives.

A match should therefore be treated as an indicator that a device may belong to a category, not proof of a specific model, owner, or activity. MAC randomization, shared component vendors, silent devices, channel dwell timing, and RF coexistence can all affect observations.

## Logging

Serial logging is useful for confirming that passive scanning is active even when no categorized devices are present.

Typical Wi-Fi observations include endpoint type, MAC, channel, and RSSI. BLE observations include address, address type, RSSI, and advertised name when available. Categorized matches are logged separately when they increment a category count.

## Hardware

- LILYGO T-Dongle C5
- ESP32-C5
- 16 MB flash
- 160x80 ST7735 LCD
- USB Serial/JTAG interface

The project uses a custom partition table with a 3 MB factory application partition because the combined Wi-Fi, NimBLE, and LCD firmware is larger than ESP-IDF's default 1 MB factory slot.

## Build

Activate the ESP-IDF environment, then from the repository root:

```sh
idf.py set-target esp32c5
idf.py reconfigure
idf.py build
```

The repository's `sdkconfig.defaults` enables the required NimBLE observer/coexistence options, configures the board for 16 MB flash, tunes Wi-Fi receive buffering, and selects the custom partition table.

If configuration or partition settings have changed significantly, perform a clean configure first:

```sh
idf.py fullclean
idf.py reconfigure
idf.py build
```

## Flash and monitor

With the T-Dongle C5 connected as `/dev/ttyACM0`:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

Exit the ESP-IDF monitor with `Ctrl-]`.

## LCD rendering

The T-Dongle LCD driver uses an RGB565 framebuffer. The detector renders the complete category status screen in memory and calls `lcd_flush()` to transfer the completed frame to the display.

The status screen includes the project title, total categorized-device count, a divider, per-category counts, severity colors, and relative horizontal prevalence bars.

## Source layout

```text
main/
    scan.c                Detection policy, signature/OUI matching, counts, LCD
    wifi_monitor.c        Wi-Fi promiscuous capture and AP/client classification
    wifi_monitor.h        Wi-Fi monitor public interface
    ble_monitor.c         NimBLE passive scanning and advertisement parsing
    ble_monitor.h         BLE observation structures and public interface
    t_dongle_lcd.c        LCD/framebuffer implementation
    t_dongle_lcd.h        LCD public interface

partitions.csv            Custom 3 MB factory application partition
sdkconfig.defaults        ESP32-C5 Wi-Fi/BLE/flash configuration defaults
```

## OUI and signature sources

The Wi-Fi OUI table began with entries from `colonelpanichacks/ouispy-detector/ouis.md` and includes documented Flock Safety, Axon, DJI, Parrot, Skydio, Ring, Meta / Ray-Ban, and Vuzix-related entries that were selected for useful category specificity.

BLE detection complements those OUIs with advertisement-level signatures because BLE address randomization often makes prefix matching ineffective.

## Passive operation

This firmware is passive and detection-only. It does not associate with observed Wi-Fi devices, initiate BLE connections, transmit probe frames for discovery, or attempt to access encrypted traffic contents.
