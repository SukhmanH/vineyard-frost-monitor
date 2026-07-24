#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <time.h>
#include "sensor_packet.h"
#include "secrets.h"

TFT_eSPI tft = TFT_eSPI();

// Sensor MAC (physical board currently running sensor firmware)
uint8_t sensorMAC[] = {0x30, 0x76, 0xF5, 0xEE, 0xEB, 0xB8};

#define BUTTON_PIN 0

static const uint8_t MY_VINEYARD = 0;

// frost thresholds (deg C), per PROJECT.md
static const float WATCH_C = 2.0f;
static const float WARN_C  = 1.0f;
static const float FROST_C = 0.0f;

static const uint16_t FLASH_PERIOD_MS = 500;

// 565 colour for orange
#define COL_ORANGE 0xFD20

// upload pacing
#define POST_PERIOD_MS   (20UL * 1000UL)   // 5 min per PROJECT.md
#define BUF_SIZE         128                     // ~21 hr at one packet/10min for 1 node
#define WIFI_TIMEOUT_MS  15000
#define NTP_TIMEOUT_MS   10000

enum FrostState : uint8_t {
  STATE_NONE = 0,
  STATE_OK,
  STATE_WATCH,
  STATE_WARN,
  STATE_FROST,
};

// telemetry buffer (ring, drop oldest on overflow)
static SensorPacket buf[BUF_SIZE];
static uint32_t     bufStamp[BUF_SIZE];
static volatile uint16_t bufHead  = 0;
static volatile uint16_t bufCount = 0;
static portMUX_TYPE bufMux = portMUX_INITIALIZER_UNLOCKED;

// last RX (for UI), one-element latch
SensorPacket incoming;
volatile bool newData = false;

bool lastBtnState = HIGH;

bool     wifiUp     = false;
bool     timeSynced = false;
uint8_t  rfChannel  = 1;
uint32_t lastPostMs = 0;
uint32_t lastPostEpoch = 0;
bool     lastPostOk = false;

FrostState   curState     = STATE_NONE;
SensorPacket lastShown    = {};
bool         haveShown    = false;
bool         flashOn      = true;
uint32_t     lastFlashMs  = 0;

const char* nodeName(uint8_t id) {
  switch (id) {
    case NODE_TOP:  return "top";
    case NODE_MID:  return "mid";
    case NODE_BASE: return "base";
    default:        return "??";
  }
}

FrostState classify(float t) {
  if (t <= FROST_C) return STATE_FROST;
  if (t <= WARN_C)  return STATE_WARN;
  if (t <= WATCH_C) return STATE_WATCH;
  return STATE_OK;
}

const char* stateLabel(FrostState s) {
  switch (s) {
    case STATE_OK:    return "OK";
    case STATE_WATCH: return "WATCH";
    case STATE_WARN:  return "WARNING";
    case STATE_FROST: return "FROST";
    default:          return "...";
  }
}

uint16_t stateColor(FrostState s) {
  switch (s) {
    case STATE_OK:    return TFT_DARKGREEN;
    case STATE_WATCH: return TFT_YELLOW;
    case STATE_WARN:  return COL_ORANGE;
    case STATE_FROST: return TFT_RED;
    default:          return TFT_DARKGREY;
  }
}

uint16_t stateTextColor(FrostState s) {
  return (s == STATE_WATCH) ? TFT_BLACK : TFT_WHITE;
}

uint32_t epochNow() {
  if (!timeSynced) return (uint32_t)(millis() / 1000);
  time_t now = time(nullptr);
  return (uint32_t)now;
}

void appendPacket(const SensorPacket &p, uint32_t epoch) {
  portENTER_CRITICAL(&bufMux);
  if (bufCount == BUF_SIZE) {
    bufHead = (bufHead + 1) % BUF_SIZE;
    bufCount--;
  }
  uint16_t slot = (bufHead + bufCount) % BUF_SIZE;
  buf[slot]      = p;
  bufStamp[slot] = epoch;
  bufCount++;
  portEXIT_CRITICAL(&bufMux);
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Failed");
}

// Tracks last successful ESP-NOW receive — used by the staleness recovery
// in loop() to re-init ESP-NOW if nothing has arrived for ESPNOW_STALE_MS.
uint32_t lastRxMs = 0;

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  lastRxMs = millis();
  if (len != sizeof(SensorPacket)) {
    Serial.printf("Bad packet len=%d expected=%u\n", len, (unsigned)sizeof(SensorPacket));
    return;
  }
  memcpy(&incoming, data, sizeof(incoming));
  uint32_t now = epochNow();
  if (incoming.node_id != NODE_BASE) incoming.timestamp = now;
  appendPacket(incoming, now);
  newData = true;
}

// ---- TFT ----
void drawBanner(FrostState s, bool on) {
  uint16_t bg = on ? stateColor(s) : TFT_BLACK;
  uint16_t fg = on ? stateTextColor(s) : TFT_DARKGREY;
  tft.fillRect(0, 0, 320, 70, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(4);
  const char *label = stateLabel(s);
  int16_t w = tft.textWidth(label);
  tft.setCursor((320 - w) / 2, 22);
  tft.print(label);
}

void drawTemp(float t, FrostState s) {
  tft.fillRect(0, 80, 320, 60, TFT_BLACK);
  uint16_t col = (s == STATE_OK) ? TFT_WHITE : stateColor(s);
  tft.setTextColor(col, TFT_BLACK);
  tft.setTextSize(6);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f C", t);
  int16_t w = tft.textWidth(buf);
  tft.setCursor((320 - w) / 2, 90);
  tft.print(buf);
}

void drawDetails(const SensorPacket &p) {
  tft.fillRect(0, 150, 320, 60, TFT_BLACK);
  tft.setTextSize(2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 155);
  tft.printf("RH  %.1f%%", p.humidity_pct);

  tft.setCursor(170, 155);
  tft.printf("Dew %.1fC", p.dew_point_c);

  tft.setCursor(10, 185);
  tft.printf("node:%s", nodeName(p.node_id));

  tft.setCursor(170, 185);
  tft.printf("bat %umV", p.battery_mv);
}

void drawStatusBar() {
  bool wifiNow = (WiFi.status() == WL_CONNECTED);
  tft.fillRect(0, 215, 320, 25, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(wifiNow ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setCursor(5, 220);
  tft.printf("wifi:%s", wifiNow ? "OK" : "DOWN");

  tft.setTextColor(timeSynced ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(75, 220);
  tft.printf("ntp:%s", timeSynced ? "OK" : "--");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(140, 220);
  tft.printf("buf:%u", (unsigned)bufCount);

  tft.setTextColor(lastPostOk ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(200, 220);
  if (lastPostEpoch == 0) {
    tft.print("up:never");
  } else {
    uint32_t age = (millis() - lastPostMs) / 1000;
    tft.printf("up:%lus", (unsigned long)age);
  }

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(5, 232);
  tft.printf("ch%u  ts%lu", rfChannel, (unsigned long)epochNow());
}

void drawIdle() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(40, 110);
  tft.println("Waiting for packet...");
  drawStatusBar();
}

void renderPacket(const SensorPacket &p) {
  FrostState newState = classify(p.temperature_c);

  bool stateChanged = (newState != curState);
  bool tempChanged  = !haveShown || lastShown.temperature_c != p.temperature_c;
  bool detailsChanged = !haveShown
      || lastShown.humidity_pct != p.humidity_pct
      || lastShown.dew_point_c  != p.dew_point_c
      || lastShown.battery_mv   != p.battery_mv
      || lastShown.node_id      != p.node_id;

  curState = newState;
  flashOn  = true;
  lastFlashMs = millis();

  if (stateChanged || !haveShown) drawBanner(curState, flashOn);
  if (tempChanged  || stateChanged || !haveShown) drawTemp(p.temperature_c, curState);
  if (detailsChanged) drawDetails(p);

  drawStatusBar();

  lastShown = p;
  haveShown = true;
}

// ---- WiFi / NTP ----
bool connectWifi() {
  // AP_STA, not STA: the dummy AP role keeps the WiFi MAC permissive enough
  // to receive ESP-NOW frames from non-associated stations. With pure STA
  // mode the driver filters out random sender MACs once associated to an AP.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("frost-base-internal", nullptr, /*channel=*/1, /*hidden=*/1);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi: FAILED, continuing offline");
    return false;
  }
  // power-save flips the radio off between AP beacons; that confuses concurrent
  // ESP-NOW and (more importantly here) gets us silently disassociated under
  // sparse traffic. Disable it for the always-on base.
  WiFi.setSleep(false);
  rfChannel = WiFi.channel();
  // belt + suspenders: force everything that affects ESP-NOW receive reliability
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(rfChannel, WIFI_SECOND_CHAN_NONE);
  Serial.printf("WiFi: connected ch=%u IP=%s RSSI=%d\n",
                rfChannel, WiFi.localIP().toString().c_str(), WiFi.RSSI());
  Serial.printf("BASE STA MAC: %s\n", WiFi.macAddress().c_str());
  return true;
}

// best-effort reconnect with short timeout. Called before each POST so a
// transient drop doesn't lose a 5-min batch.
bool ensureWifi(uint32_t timeout_ms = 5000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.print("WiFi reconnect");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  bool ok = (WiFi.status() == WL_CONNECTED);
  if (ok) {
    WiFi.setSleep(false);
    Serial.printf("WiFi: back ch=%u RSSI=%d\n", WiFi.channel(), WiFi.RSSI());
  } else {
    Serial.println("WiFi: still down");
  }
  return ok;
}

void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP");
  uint32_t t0 = millis();
  time_t now = 0;
  while ((now = time(nullptr)) < 1700000000 && millis() - t0 < NTP_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (now < 1700000000) {
    Serial.println("NTP: FAILED, using millis-based timestamps");
    timeSynced = false;
  } else {
    Serial.printf("NTP: epoch=%lu\n", (unsigned long)now);
    timeSynced = true;
  }
}

// ---- Influx ----
size_t buildBatchPayload(uint16_t n, char *out, size_t cap) {
  size_t off = 0;
  for (uint16_t i = 0; i < n && off < cap; i++) {
    uint16_t idx = (bufHead + i) % BUF_SIZE;
    const SensorPacket &p = buf[idx];
    uint64_t tsNs = (uint64_t)bufStamp[idx] * 1000000000ULL;

    int w = snprintf(out + off, cap - off,
      "vineyard,vineyard_id=%u,node_id=%s "
      "temp_c=%.2f,humidity_pct=%.2f,dew_point_c=%.2f,battery_mv=%ui,packet_version=%ui "
      "%llu\n",
      (unsigned)p.vineyard_id, nodeName(p.node_id),
      p.temperature_c, p.humidity_pct, p.dew_point_c,
      (unsigned)p.battery_mv, (unsigned)p.packet_version,
      (unsigned long long)tsNs);
    if (w < 0 || (size_t)w >= cap - off) break;
    off += (size_t)w;
  }
  return off;
}

bool postToInflux(const char *payload, size_t len) {
  if (!ensureWifi()) {
    Serial.println("POST: skipped, WiFi down");
    wifiUp = false;
    return false;
  }
  wifiUp = true;
  WiFiClientSecure client;
  client.setInsecure();   // TODO: pin InfluxData CA for v1 deploy
  HTTPClient http;

  String url = String(INFLUX_URL) + "/api/v2/write?bucket=" + INFLUX_BUCKET
             + "&org=" + INFLUX_ORG + "&precision=ns";
  if (!http.begin(client, url)) {
    Serial.println("POST: http.begin failed");
    return false;
  }
  http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");

  int code = http.POST((uint8_t *)payload, len);
  bool ok = (code >= 200 && code < 300);
  Serial.printf("POST: %d %s (%u bytes)\n", code, ok ? "OK" : "FAIL", (unsigned)len);
  if (!ok) {
    String body = http.getString();
    if (body.length()) Serial.println(body);
  }
  http.end();
  return ok;
}

void tryPostBatch() {
  uint16_t snapshot;
  portENTER_CRITICAL(&bufMux);
  snapshot = bufCount;
  portEXIT_CRITICAL(&bufMux);

  if (snapshot == 0) {
    Serial.println("POST: nothing to send");
    lastPostMs = millis();
    return;
  }

  // worst case ~200 bytes/line; cap at 24 KB for headroom on 320 KB DRAM
  static char payload[24 * 1024];
  size_t len = buildBatchPayload(snapshot, payload, sizeof(payload));

  if (postToInflux(payload, len)) {
    portENTER_CRITICAL(&bufMux);
    bufHead   = (bufHead + snapshot) % BUF_SIZE;
    bufCount -= snapshot;
    portEXIT_CRITICAL(&bufMux);
    lastPostOk    = true;
    lastPostEpoch = epochNow();
  } else {
    lastPostOk = false;
    // keep packets, ring will drop oldest on overflow
  }
  lastPostMs = millis();
}

// ---- ESP-NOW init / recovery ----
// Wraps init + peer add + recv callback. Safe to call multiple times.
#define ESPNOW_STALE_MS  60000   // if no packet in this many ms, re-init

bool initEspNow() {
  esp_now_deinit();   // safe even if not previously initialized

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init FAILED");
    return false;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  // unicast peer (sensor) for any send-to-sensor we might do
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, sensorMAC, 6);
  peer.channel = rfChannel;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // also accept broadcasts (sensor uses broadcast for its readings)
  esp_now_peer_info_t bpeer = {};
  for (int i = 0; i < 6; i++) bpeer.peer_addr[i] = 0xFF;
  bpeer.channel = rfChannel;
  bpeer.encrypt = false;
  esp_now_add_peer(&bpeer);

  Serial.printf("ESP-NOW: ready on ch=%u\n", rfChannel);
  return true;
}

// ---- setup / loop ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BASE NODE ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(1);
  drawIdle();

  wifiUp = connectWifi();
  if (wifiUp) syncTime();

  if (!initEspNow()) return;

  lastRxMs = millis();
  Serial.printf("READY. POST every %lu s, channel %u\n",
                (unsigned long)(POST_PERIOD_MS / 1000), rfChannel);
  drawStatusBar();
  lastPostMs = millis();
}

void loop() {
  // bench demo: button on base sends a self packet
  bool currentBtn = digitalRead(BUTTON_PIN);
  if (lastBtnState == HIGH && currentBtn == LOW) {
    SensorPacket self = {};
    self.node_id        = NODE_BASE;
    self.vineyard_id    = MY_VINEYARD;
    self.temperature_c  = 21.0f;
    self.humidity_pct   = 45.0f;
    self.dew_point_c    = 0.0f;
    self.battery_mv     = 0;
    self.timestamp      = epochNow();
    self.packet_version = SENSOR_PACKET_VERSION;
    esp_now_send(sensorMAC, (uint8_t *)&self, sizeof(self));
    Serial.println("Base sent self packet");
    delay(50);
  }
  lastBtnState = currentBtn;

  if (newData) {
    newData = false;
    Serial.printf("RX node=%s T=%.2fC RH=%.1f%% Td=%.2fC bat=%umV ver=%u  buf=%u\n",
                  nodeName(incoming.node_id),
                  incoming.temperature_c,
                  incoming.humidity_pct,
                  incoming.dew_point_c,
                  incoming.battery_mv,
                  incoming.packet_version,
                  (unsigned)bufCount);
    renderPacket(incoming);
  }

  if (curState == STATE_FROST) {
    uint32_t now = millis();
    if (now - lastFlashMs >= FLASH_PERIOD_MS) {
      flashOn = !flashOn;
      lastFlashMs = now;
      drawBanner(curState, flashOn);
    }
  }

  if (millis() - lastPostMs >= POST_PERIOD_MS) {
    tryPostBatch();
    drawStatusBar();
  }

  // heartbeat so we can confirm base is alive and on channel during silent stretches
  static uint32_t lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= 10000) {
    lastHeartbeatMs = millis();
    uint8_t cur_ch;
    wifi_second_chan_t sec_ch;
    esp_wifi_get_channel(&cur_ch, &sec_ch);
    uint32_t silence_s = (millis() - lastRxMs) / 1000;
    Serial.printf("alive: ch=%u  wifi=%s  RSSI=%d  buf=%u  uptime=%lus  silence=%lus\n",
                  cur_ch,
                  WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                  WiFi.RSSI(),
                  (unsigned)bufCount,
                  (unsigned long)(millis() / 1000),
                  (unsigned long)silence_s);
  }

  // ESP-NOW receive can silently break after some WiFi events. If we go
  // ESPNOW_STALE_MS without any packet, tear it down and re-init.
  if (millis() - lastRxMs >= ESPNOW_STALE_MS) {
    Serial.println("ESP-NOW stale, re-initialising");
    esp_wifi_set_channel(rfChannel, WIFI_SECOND_CHAN_NONE);
    initEspNow();
    lastRxMs = millis();   // give it another window before retrying
  }

  // refresh "up:Xs" age once a second so it doesn't look frozen
  static uint32_t lastStatusMs = 0;
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    drawStatusBar();
  }

  delay(10);
}
