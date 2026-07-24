// Minimal ESP-NOW broadcast receiver.
// No WiFi connection to any AP, no anything else.
// Pair with tx tool on the other board.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

uint32_t rxCount = 0;

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  rxCount++;
  Serial.printf("RX #%lu from %02X:%02X:%02X:%02X:%02X:%02X len=%d : ",
                (unsigned long)rxCount,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
  for (int i = 0; i < len && i < 12; i++) {
    if (data[i] >= 32 && data[i] < 127) Serial.printf("%c", data[i]);
    else Serial.printf("\\x%02X", data[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP-NOW MIN RX ===");

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
  esp_now_register_recv_cb(onReceive);

  // need broadcast peer registered to receive broadcasts on some core versions
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("listening on ch 1 ...\n");
}

void loop() {
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 5000) {
    lastBeat = millis();
    Serial.printf("(alive, %lu packets received so far)\n", (unsigned long)rxCount);
  }
  delay(100);
}
