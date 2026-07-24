// Minimal ESP-NOW broadcast transmitter.
// No WiFi connection, no deep sleep, no anything else.
// Pair with rx tool on the other board.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t counter = 0;

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("  callback: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP-NOW MIN TX ===");

  WiFi.mode(WIFI_STA);
  Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());

  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  uint8_t prim_ch;
  wifi_second_chan_t sec_ch;
  esp_wifi_get_channel(&prim_ch, &sec_ch);
  Serial.printf("channel verified: %u\n", prim_ch);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL");
    while (1) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add_peer FAIL");
    while (1) delay(1000);
  }

  Serial.println("ready, broadcasting every 2 s\n");
}

void loop() {
  counter++;
  uint8_t buf[12];
  memcpy(buf, &counter, 4);
  memcpy(buf + 4, "HELLO_RX", 8);

  esp_err_t r = esp_now_send(broadcastMAC, buf, sizeof(buf));
  Serial.printf("send #%lu  esp_err=%d  (%s)\n",
                (unsigned long)counter, r,
                r == ESP_OK ? "queued" : "queue FAIL");
  delay(2000);
}
