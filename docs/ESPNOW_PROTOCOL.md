# ESP-NOW Broadcast Payload Protocol

This document is the receiver-facing source of truth for the application
payloads broadcast by this project. Button events keep the compact `0xA5`
format, while IMU, GPS, and GPS time packets use the typed `0xA55A` sensor
format.

## Transport

| Item | Value |
| --- | --- |
| Wi-Fi mode | Station (`WIFI_MODE_STA`) |
| Wi-Fi channel | `13` |
| Destination MAC | `FF:FF:FF:FF:FF:FF` (broadcast) |
| Encryption | Disabled |
| Button magic | `0xA5` |
| Sensor magic | `0xA55A` |
| Application version | `2` |
| Byte order | Little-endian for all multi-byte integers |

The sender's station MAC address is not included in the application payload.
Receivers obtain it from the ESP-NOW receive metadata (`src_addr`).

ESP-NOW broadcast delivery is best effort. Packets can be lost, duplicated, or
received out of order. There is no acknowledgement, retry, checksum, encryption,
or authentication at the application-protocol level.

## Packet Identification

Version 2 uses payload length to distinguish packet formats:

| Payload length | Format |
| ---: | --- |
| `3` bytes | Button event |
| `24` bytes | Typed IMU sample |
| `178` bytes | Typed IMU batch sample |
| `40` bytes | Typed GPS sample |
| `11` bytes | GPS UTC time |
| `26` bytes | Legacy IMU sample |

A receiver should first check the exact payload length, then verify `magic` and
`version`. Unknown lengths, versions, packet types, and event values should be
ignored. Do not identify a button packet from byte 2 alone: byte 2 is an event in
the 3-byte format but the version byte in the sensor formats.

For `espnow_logger`, valid typed sensor packets are decoded and written as
sensor records in `sensor_####.bin`. A valid 3-byte button packet is not a
sensor packet and remains in `espnow_####.csv`.

## Button Event Packet

Total length: `3` bytes.

| Offset | Size | Type | Field | Value |
| ---: | ---: | --- | --- | --- |
| `0` | `1` | `uint8` | `magic` | `0xA5` |
| `1` | `1` | `uint8` | `version` | `2` |
| `2` | `1` | `uint8` | `event` | See event table |

Event values:

| Value | Name | Meaning |
| ---: | --- | --- |
| `1` | `BUTTON_DOWN` | Button A changed to pressed |
| `2` | `BUTTON_RELEASE` | Button A changed to released |

Examples:

```text
A5 02 01    Button down
A5 02 02    Button release
```

One packet is sent on each debounced Button A state transition. Repeated packets
are not sent while the button remains in the same state.

## Typed IMU Sample Packet

Total length: `24` bytes. Signed fields use two's-complement representation.

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `2` | `uint16_le` | `magic` | `0xA55A` |
| `2` | `1` | `uint8` | `version` | `2` |
| `3` | `1` | `uint8` | `type` | `1` (`IMU`) |
| `4` | `4` | `uint32_le` | `sequence` | Sample sequence number |
| `8` | `2` | `int16_le` | `accel_x_mg` | X acceleration, `1 mg = 0.001 g` |
| `10` | `2` | `int16_le` | `accel_y_mg` | Y acceleration, `1 mg = 0.001 g` |
| `12` | `2` | `int16_le` | `accel_z_mg` | Z acceleration, `1 mg = 0.001 g` |
| `14` | `2` | `int16_le` | `gyro_x_centi_dps` | X angular rate, `0.01 degree/s` |
| `16` | `2` | `int16_le` | `gyro_y_centi_dps` | Y angular rate, `0.01 degree/s` |
| `18` | `2` | `int16_le` | `gyro_z_centi_dps` | Z angular rate, `0.01 degree/s` |
| `20` | `2` | `int16_le` | `temperature_centi_c` | Temperature, `0.01 degree C` |
| `22` | `2` | `uint16_le` | `flags` | Currently `0` |

Conversion to floating-point SI-style values:

```text
acceleration_g = accel_mg / 1000.0
angular_rate_dps = gyro_centi_dps / 100.0
temperature_c = temperature_centi_c / 100.0
```

`sequence` starts at zero when the IMU broadcast task starts and increments
after every successfully read sample, including when the subsequent ESP-NOW send
call reports an error. It wraps modulo `2^32`.

IMU packets are normally generated every `10 ms` (`100 Hz`). The sender does
not emit them continuously. It starts streaming only after receiving an IMU
control packet, then sends IMU batches back to that requester's source MAC via
ESP-NOW unicast.

## Typed IMU Batch Sample Packet

Total length: `178` bytes. Signed fields use two's-complement representation.
This packet groups `10` consecutive raw IMU samples into a single broadcast.
The payload stores them as `raw[10]`, and the batch header carries the first
sample's temperature plus `gps_utc_ms`.
The summary block contains batch average, minimum, and maximum values for accel
and gyro so receivers can inspect the batch without unpacking every sample.
The current sender emits this batch packet only while an IMU stream request is
active.

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `2` | `uint16_le` | `magic` | `0xA55A` |
| `2` | `1` | `uint8` | `version` | `2` |
| `3` | `1` | `uint8` | `type` | `4` (`IMU batch`) |
| `4` | `4` | `uint32_le` | `sequence_start` | First sample sequence number |
| `8` | `1` | `uint8` | `sample_count` | Number of raw samples, currently `10` |
| `9` | `1` | `uint8` | `sample_period_ms` | Interval between samples |
| `10` | `2` | `uint16_le` | `flags` | Currently `0` |
| `12` | `2` | `int16_le` | `temperature_centi_c` | Temperature of the first sample, `0.01 degree C` |
| `14` | `8` | `uint64_le` | `gps_utc_ms` | UTC timestamp in Unix milliseconds |
| `22` | `36` | `imu_batch_summary_v1_t` | `summary` | Batch summary block |
| `58` | `12` | `imu_raw_sample_v1_t` | `raw[0]` | Sample 1 in the batch |
| `70` | `12` | `imu_raw_sample_v1_t` | `raw[1]` | Sample 2 in the batch |
| `82` | `12` | `imu_raw_sample_v1_t` | `raw[2]` | Sample 3 in the batch |
| `94` | `12` | `imu_raw_sample_v1_t` | `raw[3]` | Sample 4 in the batch |
| `106` | `12` | `imu_raw_sample_v1_t` | `raw[4]` | Sample 5 in the batch |
| `118` | `12` | `imu_raw_sample_v1_t` | `raw[5]` | Sample 6 in the batch |
| `130` | `12` | `imu_raw_sample_v1_t` | `raw[6]` | Sample 7 in the batch |
| `142` | `12` | `imu_raw_sample_v1_t` | `raw[7]` | Sample 8 in the batch |
| `154` | `12` | `imu_raw_sample_v1_t` | `raw[8]` | Sample 9 in the batch |
| `166` | `12` | `imu_raw_sample_v1_t` | `raw[9]` | Sample 10 in the batch |

The batch packet stores the first sample temperature once in the header,
includes `gps_utc_ms` for cross-stream alignment, and exposes a summary block
covering the whole batch.

Summary block layout:

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `6` | `int16_le[3]` | `accel_mg_avg` | Average acceleration over the batch |
| `6` | `6` | `int16_le[3]` | `accel_mg_min` | Minimum acceleration over the batch |
| `12` | `6` | `int16_le[3]` | `accel_mg_max` | Maximum acceleration over the batch |
| `18` | `6` | `int16_le[3]` | `gyro_centi_dps_avg` | Average gyro over the batch |
| `24` | `6` | `int16_le[3]` | `gyro_centi_dps_min` | Minimum gyro over the batch |
| `30` | `6` | `int16_le[3]` | `gyro_centi_dps_max` | Maximum gyro over the batch |

`sequence_start` is the sequence number of the first raw sample in the batch.
The following samples are consecutive and can be reconstructed by adding
`sample_period_ms` per sample. At `10 ms`, the batch cadence is `100 ms`.

## IMU Control Packet

Total length: `5` bytes. The receiver sends this packet as an ESP-NOW broadcast
packet. The sender uses the ESP-NOW receive metadata `src_addr` as the unicast
destination for subsequent IMU batch packets.

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `2` | `uint16_le` | `magic` | `0xA55A` |
| `2` | `1` | `uint8` | `version` | `2` |
| `3` | `1` | `uint8` | `type` | `5` (`IMU control`) |
| `4` | `1` | `uint8` | `command` | `1` = start, `2` = stop |

Rules:

- `start` begins IMU batch streaming toward the request packet's `src_addr`.
- `stop` ends streaming only when sent by the currently active requester.
- A later `start` request from another receiver replaces the previous stream
  destination.

## Typed GPS Sample Packet

Total length: `40` bytes.

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `2` | `uint16_le` | `magic` | `0xA55A` |
| `2` | `1` | `uint8` | `version` | `2` |
| `3` | `1` | `uint8` | `type` | `2` (`GPS`) |
| `4` | `4` | `uint32_le` | `sequence` | Sample sequence number |
| `8` | `4` | `int32_le` | `latitude_e7` | Latitude, degrees times `10^7` |
| `12` | `4` | `int32_le` | `longitude_e7` | Longitude, degrees times `10^7` |
| `16` | `4` | `int32_le` | `altitude_mm` | Altitude above MSL in millimeters |
| `20` | `4` | `uint32_le` | `speed_mmps` | Ground speed in millimeters per second |
| `24` | `2` | `uint16_le` | `course_cdeg` | Track course in centi-degrees |
| `26` | `2` | `uint16_le` | `hdop_centi` | Horizontal dilution in centi-units |
| `28` | `1` | `uint8` | `satellites` | Satellite count |
| `29` | `1` | `uint8` | `fix_type` | GPS fix quality/type |
| `30` | `2` | `uint16_le` | `flags` | Currently `0` |
| `32` | `8` | `uint64_le` | `gps_utc_ms` | UTC timestamp in Unix milliseconds |

`sequence` is local to the GPS broadcast stream. `gps_utc_ms` is derived from
the parsed UTC date/time and lets receivers align GPS fixes with wall-clock time.

## GPS UTC Time Packet

Total length: `11` bytes. This packet carries UTC time directly from the GPS
receiver. It is broadcast only when the sender observes a new valid `RMC` or
`ZDA` time stamp whose second field is one of `00`, `15`, `30`, or `45`.

| Offset | Size | Type | Field | Unit / meaning |
| ---: | ---: | --- | --- | --- |
| `0` | `2` | `uint16_le` | `magic` | `0xA55A` |
| `2` | `1` | `uint8` | `version` | `2` |
| `3` | `1` | `uint8` | `type` | `3` (`GPS UTC time`) |
| `4` | `2` | `uint16_le` | `utc_year` | UTC year, e.g. `2026` |
| `6` | `1` | `uint8` | `utc_month` | Month, `1`-`12` |
| `7` | `1` | `uint8` | `utc_day` | Day of month, `1`-`31` |
| `8` | `1` | `uint8` | `utc_hour` | Hour, `0`-`23` |
| `9` | `1` | `uint8` | `utc_minute` | Minute, `0`-`59` |
| `10` | `1` | `uint8` | `utc_second` | Second, `0`-`60` |

Example:

```text
5A A5 02 03 E2 07 06 1B 0A 22 19
```

That example encodes `2026-06-27T10:34:25Z`.

## Receiver Example in C

Decode byte fields explicitly or copy them into aligned local variables. Do not
cast an arbitrary receive buffer to an unpacked native structure.

```c
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define BUTTON_MAGIC 0xA5
#define SENSOR_MAGIC 0xA55A
#define PROTOCOL_VERSION 2
#define PACKET_TYPE_IMU 1
#define PACKET_TYPE_GPS 2
#define PACKET_TYPE_GPS_TIME 3

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t event;
} button_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    int16_t accel_mg[3];
    int16_t gyro_centi_dps[3];
    uint16_t flags;
} sensor_imu_packet_v1_t;

typedef struct __attribute__((packed)) {
    int16_t accel_mg[3];
    int16_t gyro_centi_dps[3];
} imu_raw_sample_v1_t;

typedef struct __attribute__((packed)) {
    int16_t accel_mg_avg[3];
    int16_t accel_mg_min[3];
    int16_t accel_mg_max[3];
    int16_t gyro_centi_dps_avg[3];
    int16_t gyro_centi_dps_min[3];
    int16_t gyro_centi_dps_max[3];
} imu_batch_summary_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence_start;
    uint8_t sample_count;
    uint8_t sample_period_ms;
    uint16_t flags;
    int16_t temperature_centi_c;
    uint64_t gps_utc_ms;
    imu_batch_summary_v1_t summary;
    imu_raw_sample_v1_t raw[10];
} sensor_imu_batch_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t speed_mmps;
    uint16_t course_cdeg;
    uint16_t hdop_centi;
    uint8_t satellites;
    uint8_t fix_type;
    uint16_t flags;
    uint64_t gps_utc_ms;
} sensor_gps_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
} sensor_utc_time_packet_v1_t;

_Static_assert(sizeof(button_packet_v1_t) == 3, "button packet size");
_Static_assert(sizeof(sensor_imu_packet_v1_t) == 24, "IMU packet size");
_Static_assert(sizeof(imu_raw_sample_v1_t) == 12, "IMU raw sample size");
_Static_assert(sizeof(sensor_imu_batch_packet_v1_t) == 178, "IMU batch packet size");
_Static_assert(sizeof(sensor_gps_packet_v1_t) == 40, "GPS packet size");
_Static_assert(sizeof(sensor_utc_time_packet_v1_t) == 11, "GPS time packet size");

bool decode_sensor_imu_v1(const uint8_t *data, size_t length, sensor_imu_packet_v1_t *out)
{
    if (data == NULL || out == NULL || length != sizeof(*out)) {
        return false;
    }

    memcpy(out, data, sizeof(*out));
    return out->magic == SENSOR_MAGIC &&
           out->version == PROTOCOL_VERSION &&
           out->type == PACKET_TYPE_IMU;
}
```

This example assumes a little-endian receiver, including ESP32-family devices.
A big-endian receiver must decode each 16-bit and 32-bit field as little-endian.

## Receiver Example in Python

```python
import struct

BUTTON_V1 = struct.Struct("<BBB")
IMU_V1 = struct.Struct("<HBBIhhhhhhhH")
IMU_BATCH_SUMMARY_V1 = struct.Struct("<" + "hhh" * 6)
IMU_BATCH_V1 = struct.Struct("<HBBIBBHhQ" + "hhh" * 6 + "hhhhhh" * 10)
GPS_V1 = struct.Struct("<HBBIiiiIHHBBHQ")
GPS_TIME_V1 = struct.Struct("<HBBHBBBBB")


def decode(payload: bytes):
    if len(payload) == BUTTON_V1.size:
        magic, version, event = BUTTON_V1.unpack(payload)
        if magic == 0xA5 and version == 2 and event in (1, 2):
            return {"kind": "button", "event": event}

    if len(payload) == IMU_V1.size:
        values = IMU_V1.unpack(payload)
        magic, version, packet_type = values[:3]
        if magic == 0xA55A and version == 2 and packet_type == 1:
            sequence = values[3]
            ax, ay, az, gx, gy, gz, temperature, flags = values[4:]
            return {
                "kind": "imu",
                "sequence": sequence,
                "accel_g": (ax / 1000.0, ay / 1000.0, az / 1000.0),
                "gyro_dps": (gx / 100.0, gy / 100.0, gz / 100.0),
                "temperature_c": temperature / 100.0,
                "flags": flags,
            }

    if len(payload) == IMU_BATCH_V1.size:
        values = IMU_BATCH_V1.unpack(payload)
        magic, version, packet_type = values[:3]
        if magic == 0xA55A and version == 2 and packet_type == 4:
            sequence_start = values[3]
            sample_count = values[4]
            sample_period_ms = values[5]
            flags = values[6]
            temperature_centi_c = values[7]
            gps_utc_ms = values[8]
            summary_values = IMU_BATCH_SUMMARY_V1.unpack(payload[22:58])
            samples = [values[27 + i * 6:33 + i * 6] for i in range(sample_count)]
            return {
                "kind": "imu_batch",
                "sequence_start": sequence_start,
                "sample_count": sample_count,
                "sample_period_ms": sample_period_ms,
                "flags": flags,
                "temperature_c": temperature_centi_c / 100.0,
                "gps_utc_ms": gps_utc_ms,
                "summary": {
                    "accel_g_avg": (
                        summary_values[0] / 1000.0,
                        summary_values[1] / 1000.0,
                        summary_values[2] / 1000.0,
                    ),
                    "accel_g_min": (
                        summary_values[3] / 1000.0,
                        summary_values[4] / 1000.0,
                        summary_values[5] / 1000.0,
                    ),
                    "accel_g_max": (
                        summary_values[6] / 1000.0,
                        summary_values[7] / 1000.0,
                        summary_values[8] / 1000.0,
                    ),
                    "gyro_dps_avg": (
                        summary_values[9] / 100.0,
                        summary_values[10] / 100.0,
                        summary_values[11] / 100.0,
                    ),
                    "gyro_dps_min": (
                        summary_values[12] / 100.0,
                        summary_values[13] / 100.0,
                        summary_values[14] / 100.0,
                    ),
                    "gyro_dps_max": (
                        summary_values[15] / 100.0,
                        summary_values[16] / 100.0,
                        summary_values[17] / 100.0,
                    ),
                },
                "raw": [
                    {
                        "accel_g": (sample[0] / 1000.0, sample[1] / 1000.0, sample[2] / 1000.0),
                        "gyro_dps": (sample[3] / 100.0, sample[4] / 100.0, sample[5] / 100.0),
                    }
                    for sample in samples
                ],
            }

    if len(payload) == GPS_V1.size:
        values = GPS_V1.unpack(payload)
        magic, version, packet_type = values[:3]
        if magic == 0xA55A and version == 2 and packet_type == 2:
            sequence = values[3]
            latitude_e7, longitude_e7, altitude_mm = values[4:7]
            speed_mmps, course_cdeg, hdop_centi = values[7:10]
            satellites, fix_type = values[10:12]
            flags, gps_utc_ms = values[12:]
            return {
                "kind": "gps",
                "sequence": sequence,
                "latitude_deg": latitude_e7 / 1e7,
                "longitude_deg": longitude_e7 / 1e7,
                "altitude_m": altitude_mm / 1000.0,
                "speed_mps": speed_mmps / 1000.0,
                "course_deg": course_cdeg / 100.0,
                "hdop": hdop_centi / 100.0,
                "satellites": satellites,
                "fix_type": fix_type,
                "flags": flags,
                "gps_utc_ms": gps_utc_ms,
            }

    if len(payload) == GPS_TIME_V1.size:
        magic, version, packet_type, year, month, day, hour, minute, second = GPS_TIME_V1.unpack(payload)
        if magic == 0xA55A and version == 2 and packet_type == 3:
            return {
                "kind": "gps_time",
                "utc": f"{year:04d}-{month:02d}-{day:02d}T{hour:02d}:{minute:02d}:{second:02d}Z",
            }

    return None
```

## Compatibility Rules

- Version 2 uses the typed `0xA55A` sensor layout for IMU, GPS, and GPS time.
- The 26-byte legacy IMU layout remains only as a compatibility fallback for
  older receivers that still expect it.
- The 178-byte batch IMU layout uses `type = 4`, stores the first sample's
  temperature once in the header, includes `gps_utc_ms`, and carries a summary
  block; receivers should not assume the single-sample IMU layout applies to
  batch payloads.
- New fields that change packet length require a new application version or a
  separately identifiable packet format.
- A receiver should track sequence numbers independently for each source MAC.
- A sender restart can be detected heuristically when its sequence and uptime
  return to small values; neither field is a persistent boot identifier.
- Values outside an application's expected physical range should be rejected or
  flagged by that application. Version 1 uses a dedicated flags field in the
  sensor packets rather than an embedded validity bitmask in every field.
