# m5stickc_espnow_imu_test

ESP-IDF project for M5StickC Plus / M5StickC Plus2.

## Current Demo

This firmware is an ESP-NOW broadcast button demo with onboard IMU initialization.

- Press `Button A / GPIO37` to broadcast a button down event.
- Release `Button A / GPIO37` to broadcast a button release event.
- Devices receiving the broadcast turn the red LED on for button down and off for button release.
- When the red LED is otherwise off, it briefly flashes for `1` ms every `5` seconds as a power heartbeat.
- The red LED is active-low on M5StickC Plus and active-high on M5StickC Plus2.
- ESP-NOW runs in station mode on Wi-Fi channel `1`.
- The onboard `MPU6886` IMU is initialized on boot.
- `GPIO26` is sampled once at startup with its internal pull-up enabled. When it is
  held low, the device broadcasts one 26-byte IMU packet every `10` ms (`100 Hz`).
  A device that starts with `GPIO26` high does not broadcast IMU packets.

Flash the same firmware to two or more M5StickC devices, then press Button A on one device to control the red LED on the others.

## Board Selection

This project uses one source file for both boards. Select the board in:

```sh
idf.py menuconfig
```

Then open:

```text
M5StickC board -> Target board
```

Choose one of:

- `M5StickC Plus (4 MB flash)`
- `M5StickC Plus2 (8 MB flash)`

The current default is `M5StickC Plus`.

Flash size is detected from the connected device when flashing, so switching the
target board does not require a separate flash-size change. The separate build
configurations below still use the matching size in generated images.

## Separate Builds

You can also keep separate sdkconfig/build directories:

```sh
idf.py -D SDKCONFIG=sdkconfig.plus \
  -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.plus \
  -B build-plus build
```

```sh
idf.py -D SDKCONFIG=sdkconfig.plus2 \
  -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.plus2 \
  -B build-plus2 build
```

## Pin Differences

Board-specific pins are defined in `main/board_pins.h`.

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
| IMU TX select | `GPIO26` | `GPIO26` |

The firmware sets `HOLD GPIO4` high on M5StickC Plus2 to keep the board powered.

## IMU packet

The packed IMU packet is 26 bytes and contains a packet magic/version/type,
32-bit sequence number, 32-bit sender uptime in milliseconds, acceleration in
`mg`, angular velocity in `0.01 dps`, and temperature in `0.01 degrees C`.
