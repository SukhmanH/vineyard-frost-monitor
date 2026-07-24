#pragma once

#include <stdint.h>

#define SENSOR_PACKET_VERSION 1

enum NodeId : uint8_t {
  NODE_TOP  = 0,
  NODE_MID  = 1,
  NODE_BASE = 2,
};

typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint8_t  vineyard_id;
  float    temperature_c;
  float    humidity_pct;
  float    dew_point_c;
  uint16_t battery_mv;
  uint32_t timestamp;       // base fills this for top/mid
  uint8_t  packet_version;
} SensorPacket;
