// I2C diagnostic + SHT4x reader.
// Wiring assumed:
//   red    -> 3V3
//   black  -> GND
//   yellow -> SCL  (GPIO 22)
//   green  -> SDA  (GPIO 21)
// Internal pull-ups are enabled by Wire.begin() but are weak; if the cable
// is long or this is unreliable, add 4.7k externals from SDA/SCL to 3V3.

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN     21
#define SCL_PIN     22
#define I2C_HZ      50000   // half-speed for marginal pull-up situations

const char *guessChip(uint8_t addr) {
  switch (addr) {
    case 0x40: return "SHT20 / HTU21";
    case 0x44: return "SHT30 / SHT31 / SHT35 / SHT40 / SHT45";
    case 0x45: return "SHT30 alt / SHT31 alt";
    case 0x70: return "SHTC3";
    default:   return "unknown";
  }
}

void scan() {
  Serial.println("\nI2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X  (%s)\n", addr, guessChip(addr));
      found++;
    }
  }
  Serial.printf("  total: %u device(s)\n", found);
}

// Sensirion CRC-8, polynomial 0x31, init 0xFF
uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

bool readSHT4x(float &t_c, float &rh_pct) {
  // 0xFD = measure with high precision (~8.2 ms typ)
  Wire.beginTransmission(0x44);
  Wire.write(0xFD);
  if (Wire.endTransmission() != 0) return false;

  delay(15);

  size_t got = Wire.requestFrom((uint8_t)0x44, (uint8_t)6);
  if (got != 6) return false;

  uint8_t buf[6];
  for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();

  if (crc8(buf, 2)     != buf[2]) { Serial.println("  T CRC fail");  return false; }
  if (crc8(buf + 3, 2) != buf[5]) { Serial.println("  RH CRC fail"); return false; }

  uint16_t t_raw  = ((uint16_t)buf[0] << 8) | buf[1];
  uint16_t rh_raw = ((uint16_t)buf[3] << 8) | buf[4];

  t_c    = -45.0f + 175.0f * (float)t_raw  / 65535.0f;
  rh_pct =  -6.0f + 125.0f * (float)rh_raw / 65535.0f;
  if (rh_pct > 100.0f) rh_pct = 100.0f;
  if (rh_pct <   0.0f) rh_pct =   0.0f;
  return true;
}

bool readSerial(uint32_t &serial) {
  // 0x89 = read serial number
  Wire.beginTransmission(0x44);
  Wire.write(0x89);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  size_t got = Wire.requestFrom((uint8_t)0x44, (uint8_t)6);
  if (got != 6) return false;
  uint8_t buf[6];
  for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();
  if (crc8(buf, 2)     != buf[2]) return false;
  if (crc8(buf + 3, 2) != buf[5]) return false;
  serial = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
         | ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== I2C / SHT4x diagnostic ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);

  scan();

  uint32_t sn;
  if (readSerial(sn)) {
    Serial.printf("\nSHT4x serial: 0x%08lX\n", (unsigned long)sn);
  } else {
    Serial.println("\nSerial read failed (no SHT4x at 0x44, or pull-up / wiring issue)");
  }

  Serial.println("\nReading every 1 s. Press EN to reset.\n");
}

void loop() {
  float t, rh;
  if (readSHT4x(t, rh)) {
    Serial.printf("T = %6.2f C   RH = %5.1f %%\n", t, rh);
  } else {
    Serial.println("read FAIL");
  }
  delay(1000);
}
