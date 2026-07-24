#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include "sensor_packet.h"
#include "dew_point.h"

// Broadcast destination — every ESP32 in range on the same channel sees these.
// No MAC-layer ACK required, so we don't depend on a single peer being responsive.
// Multiple-sensor topology distinguishes nodes via node_id inside the payload.
uint8_t baseMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// TODO: load from NVS once role-on-NVS lands (roadmap step 7)
static const uint8_t MY_NODE_ID  = NODE_MID;
static const uint8_t MY_VINEYARD = 0;

// 5 s for bench-now testing, 30 s for normal bench, 600 s for field
#define SLEEP_SECONDS      5
#define SEND_TIMEOUT_MS    200    // per-attempt onSent wait (project spec)
#define SEND_MAX_ATTEMPTS  3
#define SEND_RETRY_GAP_MS  30

// Must match the base's WiFi channel (= router's WiFi channel).
#define ESPNOW_CHANNEL     1

// SHT40 wiring
//   red    -> 3V3
//   black  -> GND
//   green  -> SDA  (GPIO 21)
//   yellow -> SCL  (GPIO 22)
#define SDA_PIN            21
#define SCL_PIN            22
#define I2C_HZ             100000
#define SHT4X_ADDR         0x44
#define SHT4X_READ_RETRIES 3

RTC_DATA_ATTR static uint32_t bootCount = 0;

volatile bool sendDone = false;
volatile bool sendOk   = false;

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  sendOk   = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// Sensirion CRC-8, polynomial 0x31, init 0xFF
static uint8_t sht_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

// One I2C read attempt against an SHT4x. Returns true on success + good CRC.
static bool readSHT4xOnce(float &t_c, float &rh_pct) {
  Wire.beginTransmission(SHT4X_ADDR);
  Wire.write(0xFD);                             // measure with high precision
  if (Wire.endTransmission() != 0) return false;

  delay(15);                                    // ~8.2 ms typ measurement time

  if (Wire.requestFrom((uint8_t)SHT4X_ADDR, (uint8_t)6) != 6) return false;

  uint8_t buf[6];
  for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();

  if (sht_crc8(buf,     2) != buf[2]) return false;
  if (sht_crc8(buf + 3, 2) != buf[5]) return false;

  uint16_t t_raw  = ((uint16_t)buf[0] << 8) | buf[1];
  uint16_t rh_raw = ((uint16_t)buf[3] << 8) | buf[4];

  t_c    = -45.0f + 175.0f * (float)t_raw  / 65535.0f;
  rh_pct =  -6.0f + 125.0f * (float)rh_raw / 65535.0f;
  if (rh_pct > 100.0f) rh_pct = 100.0f;
  if (rh_pct <   0.0f) rh_pct =   0.0f;
  return true;
}

// Try a few times. Bus glitches can drop a single read; CRC catches them.
static bool readSensor(SensorPacket &p) {
  float t_c, rh_pct;
  for (uint8_t i = 0; i < SHT4X_READ_RETRIES; i++) {
    if (readSHT4xOnce(t_c, rh_pct)) {
      p.node_id        = MY_NODE_ID;
      p.vineyard_id    = MY_VINEYARD;
      p.temperature_c  = t_c;
      p.humidity_pct   = rh_pct;
      p.dew_point_c    = dew_point_magnus_c(t_c, rh_pct);
      p.battery_mv     = 3900;          // placeholder until ADC divider is wired
      p.timestamp      = 0;             // base stamps on receive
      p.packet_version = SENSOR_PACKET_VERSION;
      if (i > 0) Serial.printf("sensor: ok on retry %u\n", i + 1);
      Serial.printf("sensor: T=%.2fC RH=%.1f%% Td=%.2fC\n",
                    p.temperature_c, p.humidity_pct, p.dew_point_c);
      return true;
    }
    Serial.printf("sensor: read fail (attempt %u/%u)\n", i + 1, SHT4X_READ_RETRIES);
    delay(20);
  }
  return false;
}

bool sendWithRetry(const SensorPacket &pkt) {
  for (uint8_t attempt = 1; attempt <= SEND_MAX_ATTEMPTS; attempt++) {
    sendDone = false;
    sendOk   = false;

    esp_err_t r = esp_now_send(baseMAC, (uint8_t *)&pkt, sizeof(pkt));
    if (r != ESP_OK) {
      Serial.printf("attempt %u: esp_now_send err=%d\n", attempt, r);
    } else {
      uint32_t t0 = millis();
      while (!sendDone && (millis() - t0) < SEND_TIMEOUT_MS) {
        delay(2);
      }
    }

    if (sendDone && sendOk) {
      Serial.printf("send: OK on attempt %u/%u  T=%.2fC RH=%.1f%% Td=%.2fC\n",
                    attempt, SEND_MAX_ATTEMPTS,
                    pkt.temperature_c, pkt.humidity_pct, pkt.dew_point_c);
      return true;
    }

    Serial.printf("attempt %u: %s\n", attempt, !sendDone ? "TIMEOUT" : "FAIL");
    if (attempt < SEND_MAX_ATTEMPTS) delay(SEND_RETRY_GAP_MS);
  }
  Serial.printf("send: GIVE UP after %u attempts\n", SEND_MAX_ATTEMPTS);
  return false;
}

void goToSleep() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  Serial.printf("sleep %d s\n", SLEEP_SECONDS);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  bootCount++;
  Serial.printf("\n--- wake #%lu  cause=%d ---\n",
                (unsigned long)bootCount,
                (int)esp_sleep_get_wakeup_cause());

  // I2C up first so a bad read can short-circuit the rest of the cycle
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);

  SensorPacket out;
  if (!readSensor(out)) {
    Serial.println("sensor: GIVE UP, sleeping without send");
    goToSleep();
  }

  WiFi.mode(WIFI_STA);
  // Note: deliberately NOT calling WiFi.disconnect() — on some core versions it
  // puts the radio into a state that interferes with ESP-NOW.
  esp_wifi_set_ps(WIFI_PS_NONE);          // never power-down the radio
  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // explicit max TX power
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    goToSleep();
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, baseMAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    goToSleep();
  }

  // verify radio state right before sending
  uint8_t prim_ch;
  wifi_second_chan_t sec_ch;
  esp_wifi_get_channel(&prim_ch, &sec_ch);
  Serial.printf("radio: my MAC=%s  ch=%u  -> base MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                WiFi.macAddress().c_str(), prim_ch,
                baseMAC[0], baseMAC[1], baseMAC[2], baseMAC[3], baseMAC[4], baseMAC[5]);

  sendWithRetry(out);

  goToSleep();
}

void loop() {
  // unreachable: deep sleep restarts from setup() on every wake
}
