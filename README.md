# m5stickc_espnow_imu_test

ESP-IDF project for M5StickC Plus, M5StickC Plus2 and M5StickS3. All three
boards share the same ESP-NOW app logic; board differences (pins, display, IMU)
live in `main/board.c` and the board-specific drivers.

## Current Demo

This firmware is an ESP-NOW sensor sender (IMU, GPS, UTC time) with a broadcast
button demo. The IMU stream is started and stopped by a control packet from a
receiver such as `espnow_logger_cyd`.

- Press `Button A / GPIO37` to broadcast a button down event.
- Release `Button A / GPIO37` to broadcast a button release event.
- Devices receiving the broadcast turn the red LED on for button down and off for button release.
- When the hardware LED is otherwise off, it flashes for `1` ms every `5` seconds as a power heartbeat.
- The red LED is active-low on M5StickC Plus and active-high on M5StickC Plus2.
- M5StickS3 uses a red ESP-NOW status circle in the display's upper-right corner and optionally drives an active-low LED on `GPIO2`.
- ESP-NOW runs in station mode on Wi-Fi channel `13`.
- The onboard IMU is initialized on boot: `MPU6886` on Plus/Plus2 and `BMI270` on StickS3.
- Unit GPS v1.1 (`AT6668 + MAX2659`) is read over UART at `115200bps / 8N1` on all three boards.
  The LCD shows NMEA reception state, fix state, satellite count, latitude, and longitude.
- The IMU stream is off at boot. A receiver sends an IMU control packet
  (`start` / `stop`, see `docs/ESPNOW_PROTOCOL.md`); while a stream is active the
  device sends one 178-byte typed IMU batch packet every `10` samples
  (`10` ms per sample, `100 Hz`) by unicast to the requester's MAC. The "IMU TX
  select" GPIO in the board tables is configured and its level is logged at
  startup, but it no longer gates streaming (remote control is authoritative).
- GPS fixes are broadcast as a 40-byte typed packet, and UTC time anchors are
  broadcast as an 11-byte typed packet at `00`, `15`, `30`, and `45` seconds.

Flash the same firmware to two or more M5StickC devices, then press Button A on one device to control the red LED on the others.

## Building

Use `tools/idf.sh <plus|plus2|s3> <idf.py arguments>`. It runs `idf.py` (ESP-IDF
v5.5.6) with the board's own build directory, `sdkconfig`, `sdkconfig.defaults.<board>`
and `IDF_TARGET`, so one board's configuration is never reused for another:

```sh
tools/idf.sh plus2 build
tools/idf.sh plus2 -p /dev/cu.usbserial-XXXX flash monitor
tools/idf.sh s3 build
tools/idf.sh plus build
```

| Board | Build directory | Defaults file | Target | Flash header |
| --- | --- | --- | --- | --- |
| `plus` (M5StickC Plus) | `build-plus/` | `sdkconfig.defaults.plus` | `esp32` | 4 MB |
| `plus2` (M5StickC Plus2) | `build-plus2/` | `sdkconfig.defaults.plus2` | `esp32` | 8 MB |
| `s3` (M5StickS3) | `build-s3/` | `sdkconfig.defaults.s3` | `esp32s3` | 8 MB |

Notes:

- `sdkconfig.defaults.<board>` is not loaded automatically by ESP-IDF (only
  `sdkconfig.defaults` and `sdkconfig.defaults.<target>` are). Plain `idf.py build`
  in the repo root ignores it and picks the Kconfig default board, which is how a
  Plus image ends up on a Plus2. Always build through the wrapper.
- Defaults are applied only when the `sdkconfig` is created. To change the board
  or flash size, delete the `build-<board>/` directory; the wrapper refuses to run
  on an `sdkconfig` that belongs to a different board.
- M5StickC Plus (not Plus2) fails to flash at idf.py's default 460800 bps with
  `Unable to verify flash chip connection`. For `plus`, `tools/idf.sh` therefore adds
  `-b 115200` to any command containing `flash` unless you pass `-b` yourself.
  `tools/flash_slow.sh <board> [port] [baud]` does flash + monitor at 115200 with the
  port auto-detected, for any board.
- The boot log's first app line names the board (`Starting M5StickC Plus2 ...`);
  check it after flashing.
- ESP-IDF v5.5.6 is the default; `main/espnow.c` also builds on 5.4.x (the ESP-NOW
  send callback signature changed in 5.5 and is selected by `ESP_IDF_VERSION`).
- `idf.py` v5.5.6 has no CMake preset support (`--preset` exists only in ESP-IDF v6),
  hence the wrapper instead of `CMakePresets.json`.

## Board Abstraction

The firmware is split into:

- Shared app logic in `main/app.c` and `main/espnow.c`
- Board selection and per-board pin/config data in `main/board.c`
- Board-specific display driver(s) such as `main/display_m5stickc.c`
- Board-specific IMU drivers in `main/imu_mpu6886.c` and `main/imu_bmi270.c`

| Signal | M5StickC Plus | M5StickC Plus2 |
| --- | ---: | ---: |
| Red LED | `GPIO10` | `GPIO19` |
| IR TX | `GPIO9` | `GPIO19` |
| LCD MOSI | `GPIO15` | `GPIO15` |
| LCD CLK | `GPIO13` | `GPIO13` |
| LCD CS | `GPIO5` | `GPIO5` |
| LCD DC | `GPIO23` | `GPIO14` |
| LCD RST | `GPIO18` | `GPIO12` |
| LCD BL | AXP192 | `GPIO27` |
| Button A | `GPIO37` | `GPIO37` |
| Button B | `GPIO39` | `GPIO39` |
| Button C / Wake | AXP192 power button | `GPIO35` |
| HOLD | none | `GPIO4` |
| PMU INT | `GPIO35` | none |
| Battery ADC | AXP192 | `GPIO38` |
| IMU TX select (level logged only) | `GPIO26` | `GPIO26` |
| GPS host TX | `GPIO32` | `GPIO32` |
| GPS host RX | `GPIO33` | `GPIO33` |

## M5StickS3 GPIO Mapping

The M5StickS3 board configuration uses the following GPIO mapping:

| Signal | M5StickS3 |
| --- | ---: |
| Button 1 | `GPIO11` |
| Button 2 | `GPIO12` |
| LCD MOSI | `GPIO39` |
| LCD CLK | `GPIO40` |
| LCD RS/DC | `GPIO45` |
| LCD RST | `GPIO21` |
| LCD CS | `GPIO41` |
| LCD BL | `GPIO38` |
| I2C SDA | `GPIO47` |
| I2C SCL | `GPIO48` |
| IMU | `BMI270` at `0x68` |
| IMU TX select (level logged only) | `GPIO5` |
| GPS host TX | `GPIO9` |
| GPS host RX | `GPIO10` |
| Optional external LED | `GPIO2` (active-low) |

Current M5StickS3 support:

- The LCD uses an `ST7789P3`-compatible init path through the existing ST7789 ESP-IDF panel driver.
- The panel uses a `240x135` landscape layout with StickS3-specific power initialization through M5PM1.
- The panel's effective 16-bit color byte order does not match a plain `RGB565` literal as expected during raw framebuffer writes; the current status indicator uses `0x00f8` so it appears red on the real panel.
- Since StickS3 has no directly controlled onboard status LED, the upper-right display indicator mirrors the LED state.
- An external LED can be connected to `GPIO2`; its output is active-low.
- Button 2 cycles LCD brightness through levels `0` to `5`. On StickS3, level `0` turns the backlight off but shows level `2` for `0.5` seconds every `10` seconds and whenever an ESP-NOW button event arrives.
- The onboard `BMI270` runs at `100 Hz`, with a `+/-2 g` accelerometer range and `+/-250 dps` gyro range. Its output uses the same ESP-NOW packet format as Plus/Plus2.
- The BMI270 implementation uses Bosch Sensortec's BSD-3-Clause SensorAPI, vendored under `components/bmi270_bosch`.

The firmware sets `HOLD GPIO4` high on M5StickC Plus2 to keep the board powered.

## Unit GPS v1.1

The GPS support targets the M5Stack Unit GPS v1.1 marked
`GPSV1.1 AT6668 + MAX2659`. This is not the older AT6558 unit: the v1.1 default
UART rate is `115200bps`, and it outputs NMEA 0183 4.1 sentences.

Connect the Grove/UART signals as follows. RX and TX names in the board tables
above are from the M5Stick host's point of view:

| GPS Unit wire/signal | M5StickC Plus/Plus2 | M5StickS3 |
| --- | ---: | ---: |
| Black / GND | GND | GND |
| Red / 5V | 5V | 5V |
| Yellow / GPS `UART_RX` | host TX `GPIO32` | host TX `GPIO9` |
| White / GPS `UART_TX` | host RX `GPIO33` | host RX `GPIO10` |

On M5StickS3, the firmware enables the M5PM1 `BOOST5V_EN_PP` output before
starting the GPS UART so the Grove connector supplies 5V to the GPS unit.
Do not feed an external 5V supply into the Grove or Hat `EXT_5V` pin while this
output is enabled; power the StickS3 through USB-C or the Hat2 `5VIN` pin.

The firmware accepts talker prefixes such as `GP`, `GN`, and `BD`, validates
NMEA checksums, and parses `GGA`, `RMC`, and `ZDA` sentences. GPS is
receive-only at the application level; the host TX pin is configured so module
commands can be added later. GPS data is displayed and logged locally, and UTC
time is also broadcast as a separate ESP-NOW packet for downstream consumers at
`00`, `15`, `30`, and `45` seconds.

## IMU packet

The packed single-sample IMU packet is 24 bytes and contains the typed `0xA55A`
header, 32-bit sequence number, acceleration in `mg`, angular velocity in
`0.01 dps`, temperature in `0.01 degrees C`, and a trailing flags field.

The batch IMU packet is 178 bytes, uses packet type `4`, and carries `10`
consecutive raw IMU samples. The batch header stores the first sample's
temperature once, plus `gps_utc_ms`, and the summary block carries the batch
average, minimum, and maximum for accel and gyro. The raw samples follow as
accel/gyro pairs. At `100 Hz`, the batch is emitted every `100` ms.

The complete wire format, button packet definition, compatibility rules, and C
and Python decoder examples are documented in
[`docs/ESPNOW_PROTOCOL.md`](docs/ESPNOW_PROTOCOL.md).
