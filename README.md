# vineyard-frost-monitor

Wireless frost monitoring for vineyards. Battery-powered ESP32 sensor nodes read
temperature and humidity, send readings over ESP-NOW to an always-on base
station, and the base uploads batched telemetry to InfluxDB for dashboards and
alerting.

Spring frost damages primary buds after budbreak, and the difference between a
lost crop and an intact one is often a couple of degrees for a couple of hours.
Cold air drains downhill and pools in low spots, so a single thermometer at the
house tells you very little about the block at the bottom of the slope. This
system puts a sensor at each elevation that matters and reports the whole
picture in one place.

## Architecture

```
  sensor node (top of slope)  ──┐
                                ├── ESP-NOW ──►  base node  ──► WiFi ──► InfluxDB ──► Grafana
  sensor node (mid slope)     ──┘                (TFT display,
                                                  ring buffer)
```

**Sensor nodes** spend nearly all their time in deep sleep. On each wake they
read the SHT4x probe, compute dew point, broadcast one packet, and sleep again.

**The base node** stays powered, receives packets, shows current frost state on a
TFT, buffers readings in RAM, and POSTs them to InfluxDB in batches.

| Path | Contents |
|---|---|
| `firmware/sensor_node/` | Deep-sleep sensor node firmware |
| `firmware/base_node/` | Always-on gateway: TFT UI, buffering, cloud upload |
| `firmware/shared/` | `SensorPacket` schema and dew point math, shared by both |
| `firmware/_tools/` | Minimal bench sketches: ESP-NOW tx/rx pair and an I2C/SHT4x probe |

## The packet

One packed struct is the contract between every node and the base:

```c
typedef struct __attribute__((packed)) {
  uint8_t  node_id;         // 0 = top, 1 = mid, 2 = base
  uint8_t  vineyard_id;
  float    temperature_c;
  float    humidity_pct;
  float    dew_point_c;
  uint16_t battery_mv;
  uint32_t timestamp;       // base stamps this for top/mid nodes
  uint8_t  packet_version;  // for OTA-safe schema migration
} SensorPacket;
```

`packet_version` exists so a base running new firmware can still parse packets
from a node that has not been updated yet. Once nodes are deployed across
multiple sites, a schema change without a version field means either a flag day
or bricked nodes.

## Implementation notes

**Dew point is computed on the sensor node**, using the Magnus formula with
Sonntag coefficients, so it travels inside the packet rather than being derived
later. Frost risk tracks dew point closely, and computing it at the source means
the reading survives even if downstream processing changes.

**SHT4x reads are CRC-validated.** The driver is written directly against the
sensor rather than pulled from a library: it issues the high-precision measure
command, validates both Sensirion CRC-8 checksums, and retries up to three times.
A corrupted read that passes silently is worse than no read, because it can
trigger or suppress a frost alert.

**Sends are broadcast, not unicast.** Broadcast requires no MAC-layer ACK, so a
node is not blocked by an unresponsive peer, and nodes are distinguished by
`node_id` in the payload instead. The application layer still waits for the
`onSent` callback and retries up to three times.

**ESP-NOW peers are re-added on every wake.** Peer registration does not survive
deep sleep. The node also explicitly sets channel and TX power each cycle rather
than assuming radio state carries over.

**The base recovers from silent ESP-NOW failures.** Receive can stop working
after certain WiFi events while the radio still appears healthy, so the base
tracks time since the last packet and tears down and re-initialises ESP-NOW if
nothing arrives within a timeout.

**Telemetry survives connectivity gaps.** Readings go into a mutex-protected
ring buffer and are only dropped from it after a successful POST. If the upload
fails the batch is retained and retried; if the buffer fills, the oldest entries
are dropped first.

## Frost states

The base classifies the coldest current reading and drives the TFT accordingly:

| State | Threshold | Display |
|---|---|---|
| OK | above +2 °C | green |
| WATCH | at or below +2 °C | yellow |
| WARNING | at or below +1 °C | orange |
| FROST | at or below 0 °C | flashing red |

## Building

Built with [PlatformIO](https://platformio.org/). Each firmware directory is its
own project.

```bash
cd firmware/sensor_node && pio run -t upload
cd firmware/base_node   && pio run -t upload
```

The base node needs credentials before it will build:

```bash
cp firmware/base_node/include/secrets.h.example firmware/base_node/include/secrets.h
```

Fill in WiFi and InfluxDB details. `secrets.h` is gitignored.

ESP-NOW and WiFi share one radio, so **the base node's ESP-NOW channel must match
the WiFi channel of the access point it associates with.** Mismatched channels
are the most common reason packets stop arriving.

## Hardware

- ESP32-WROOM-32U dev boards
- SHT4x temperature/humidity probe on I2C (SDA 21, SCL 22)
- ST7789 320x240 TFT on the base node, driven via TFT_eSPI

Sensor nodes are intended for solar and 18650 power in IP65 plastic enclosures.
Metal enclosures block 2.4 GHz.

## Status

Field-tested prototype, not a certified product. Frost alerting is only as
reliable as the link and the power budget behind it. Do not use this as the sole
input to a decision that a crop depends on.

## License

MIT
