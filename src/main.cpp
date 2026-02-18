#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "LoRaWan_APP.h"
#include "secrets.h"

namespace {
constexpr uint32_t kBaud = 115200;

constexpr uint8_t kPinSoilTemp = 4;
constexpr uint8_t kPinSoilMoisture = 7;
constexpr uint8_t kPinBattery = 2;
constexpr uint8_t kPinSensorPower = 47;
constexpr uint32_t kSensorPowerWarmupMs = 500;
constexpr uint8_t kPinI2cScl = 42;
constexpr uint8_t kPinI2cSda = 41;

constexpr uint8_t kAmbientI2cAddr = 0x44;
constexpr uint8_t kCmdMeasHighMsb = 0x24;
constexpr uint8_t kCmdMeasHighLsb = 0x00;

constexpr uint32_t kNtpValidEpochMin = 1700000000UL;
constexpr uint32_t kNtpRetryMs = 5UL * 60UL * 1000UL;
constexpr uint32_t kNtpResyncMs = 24UL * 60UL * 60UL * 1000UL;

constexpr uint32_t kDebugLogMs = 2000;

constexpr uint16_t kAdcMax = 4095;
#if NODE_ID == 1
constexpr float kAdcVref = 3.3f;   // Node1 calibrated ADC reference voltage.
#elif NODE_ID == 2
constexpr float kAdcVref = 3.3f; // Node2 calibrated ADC reference voltage.
#else
constexpr float kAdcVref = 3.3f;  // Node3 calibrated ADC reference voltage.
#endif
constexpr float kVbatCalInputV = 12.00f;   // True divider input during calibration.
constexpr float kVbatCalAdcV = 2.080f;     // Measured divider node voltage at ADC pin.
constexpr float kVbatScale = kVbatCalInputV / kVbatCalAdcV;
#if NODE_ID == 1
constexpr float kVbatFineTrim = 1.0431f;   // Node1 battery trim.
#elif NODE_ID == 2
constexpr float kVbatFineTrim = 1.0508f;   // Node2 battery trim.
#else
constexpr float kVbatFineTrim = 1.0067f;   // Node3 battery trim.
#endif
constexpr uint8_t kAdcSamples = 32;
constexpr uint8_t kAdcDiscardSamples = 4;

constexpr float kSmGmcIntercept = 114.608f;   // Regression intercept for GMC(%).
constexpr float kSmGmcSlope = -43.252f;       // Regression slope for GMC(%) vs voltage.
#if NODE_ID == 1
constexpr float kSmFineTrim = 0.9941f;     // Node1 moisture voltage trim.
#elif NODE_ID == 2
constexpr float kSmFineTrim = 0.9969f;     // Node2 moisture voltage trim.
#else
constexpr float kSmFineTrim = 0.9818f;     // Node3 moisture voltage trim.
#endif

constexpr uint8_t kErrAmbient = 0x01;
constexpr uint8_t kErrSoilTemp = 0x02;

#ifndef NODE_ID
#define NODE_ID 1
#endif

struct Measurements {
  uint16_t sm_raw;
  float sm_v;
  uint8_t sm_pct;
  float st_c;
  float at_c;
  float ah_pct;
  uint16_t vb_raw;
  float vb_v;
  uint8_t err;
};

RadioEvents_t g_radio_events;
volatile bool g_tx_done = false;
volatile bool g_tx_timeout = false;
constexpr bool kEnableLoRa = true;
volatile bool g_tx_in_progress = false;
volatile bool g_tx_timeout_flag = false;
constexpr uint8_t kBurstCount = 3;
constexpr uint32_t kBurstGapMs = 500;
constexpr uint32_t kBurstMinGapMs = 1200;

bool g_ambient_present = false;
bool g_soil_present = false;

OneWire g_one_wire(kPinSoilTemp);
DallasTemperature g_ds18b20(&g_one_wire);

uint32_t g_last_ntp_ms = 0;
uint32_t g_last_ntp_attempt_ms = 0;
bool g_time_valid = false;
uint32_t g_last_slot_id = 0;

uint8_t calculateCRC(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool i2c_ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool checkSEN0385() {
  return i2c_ping(kAmbientI2cAddr);
}

bool readSEN0385(float *temp, float *hum) {
  uint8_t data[6] = {0};

  Wire.setClock(100000);
  Wire.beginTransmission(kAmbientI2cAddr);
  Wire.write(kCmdMeasHighMsb);
  Wire.write(kCmdMeasHighLsb);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(20);

  uint8_t bytesReceived = Wire.requestFrom(kAmbientI2cAddr, static_cast<uint8_t>(6));
  if (bytesReceived != 6) {
    Serial.print("SEN0385 read fail: bytes=");
    Serial.println(bytesReceived);
    return false;
  }
  for (uint8_t i = 0; i < 6; ++i) {
    data[i] = Wire.read();
  }

  uint8_t tempCRC = calculateCRC(data, 2);
  uint8_t humCRC = calculateCRC(&data[3], 2);
  if (tempCRC != data[2] || humCRC != data[5]) {
    // Keep data path alive even with occasional CRC mismatch, same behavior as your test sketch.
    Serial.println("SEN0385 warning: CRC mismatch, using raw sample");
  }

  uint16_t tempRaw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  uint16_t humRaw = (static_cast<uint16_t>(data[3]) << 8) | data[4];

  *temp = -45.0f + 175.0f * (static_cast<float>(tempRaw) / 65535.0f);
  *hum = 100.0f * (static_cast<float>(humRaw) / 65535.0f);

  if (!isfinite(*temp) || !isfinite(*hum)) {
    return false;
  }
  if (*temp < -40.0f || *temp > 125.0f || *hum < 0.0f || *hum > 100.0f) {
    return false;
  }

  return true;
}

bool read_ambient(float &temp_c, float &hum_pct) {
  return readSEN0385(&temp_c, &hum_pct);
}

bool read_soil_temp(float &temp_c) {
  g_ds18b20.requestTemperatures();
  float t = g_ds18b20.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) {
    return false;
  }
  temp_c = t;
  return true;
}

uint8_t soil_moisture_pct(float sm_v) {
  float gmc = kSmGmcIntercept + (kSmGmcSlope * sm_v);
  if (gmc < 0.0f) {
    gmc = 0.0f;
  } else if (gmc > 100.0f) {
    gmc = 100.0f;
  }
  return static_cast<uint8_t>(gmc + 0.5f);
}

uint16_t read_adc_avg(uint8_t pin) {
  // Let ADC sampling cap settle on this channel before averaging.
  for (uint8_t i = 0; i < kAdcDiscardSamples; ++i) {
    (void)analogRead(pin);
    delayMicroseconds(250);
  }

  uint32_t sum = 0;
  for (uint8_t i = 0; i < kAdcSamples; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }
  return static_cast<uint16_t>(sum / kAdcSamples);
}

float calc_battery_voltage(uint16_t raw_adc) {
  float adc_v = (static_cast<float>(raw_adc) / kAdcMax) * kAdcVref;
  return adc_v * kVbatScale * kVbatFineTrim;
}

bool wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000UL) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP=");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi FAIL");
  return false;
}

bool ntp_sync() {
  if (!wifi_connect()) {
    return false;
  }
  configTime(0, 0, "pool.ntp.org");
  time_t now = 0;
  uint32_t start = millis();
  while ((millis() - start) < 10000UL) {
    now = time(nullptr);
    if (static_cast<uint32_t>(now) >= kNtpValidEpochMin) {
      break;
    }
    delay(200);
  }
  if (static_cast<uint32_t>(now) >= kNtpValidEpochMin) {
    g_time_valid = true;
    g_last_ntp_ms = millis();
    Serial.print("NTP OK, epoch=");
    Serial.println(static_cast<uint32_t>(now));
    return true;
  }
  Serial.println("NTP FAIL");
  return false;
}

void IRAM_ATTR on_tx_done() {
  g_tx_in_progress = false;
  if (kEnableLoRa) {
    Radio.Sleep();
  }
}

void IRAM_ATTR on_tx_timeout() {
  g_tx_in_progress = false;
  g_tx_timeout_flag = true;
  if (kEnableLoRa) {
    Radio.Sleep();
  }
}

bool in_tx_window(const tm &utc) {
  uint8_t mod = static_cast<uint8_t>(utc.tm_min % 10);
  bool minute_match = false;
  if (NODE_ID == 1) {
    minute_match = (mod == 1) || (mod == 5);
  } else if (NODE_ID == 2) {
    minute_match = (mod == 2) || (mod == 6);
  } else {
    minute_match = (mod == 3) || (mod == 7);
  }
  bool second_match = (utc.tm_sec >= 0 && utc.tm_sec <= 10);
  return minute_match && second_match;
}

uint16_t slot_id_from_utc(const tm &utc) {
  return static_cast<uint16_t>(utc.tm_yday * 24 * 60 + utc.tm_hour * 60 + utc.tm_min);
}

int next_slot_minute_from_utc(const tm &utc) {
  for (int offset = 0; offset <= 60; ++offset) {
    int cand_min = (utc.tm_min + offset) % 60;
    int mod = cand_min % 10;
    bool match = false;
    if (NODE_ID == 1) {
      match = (mod == 1) || (mod == 5);
    } else if (NODE_ID == 2) {
      match = (mod == 2) || (mod == 6);
    } else {
      match = (mod == 3) || (mod == 7);
    }
    if (!match) {
      continue;
    }
    if (offset == 0 && utc.tm_sec > 10) {
      continue;
    }
    return cand_min;
  }
  return -1;
}

Measurements read_measurements() {
  Measurements m{};
  m.err = 0;

  m.sm_raw = read_adc_avg(kPinSoilMoisture);
  m.sm_v = ((static_cast<float>(m.sm_raw) / kAdcMax) * kAdcVref) * kSmFineTrim;
  m.sm_pct = soil_moisture_pct(m.sm_v);

  m.vb_raw = read_adc_avg(kPinBattery);
  m.vb_v = calc_battery_voltage(m.vb_raw);

  m.at_c = -127.0f;
  m.ah_pct = -1.0f;
  if (g_ambient_present) {
    float t = 0.0f;
    float h = 0.0f;
    if (!read_ambient(t, h)) {
      delay(5);
      if (read_ambient(t, h)) {
        m.at_c = t;
        m.ah_pct = h;
      } else {
        m.err |= kErrAmbient;
      }
    } else {
      m.at_c = t;
      m.ah_pct = h;
    }
  } else {
    // Retry presence detection at runtime in case sensor was not ready at boot.
    g_ambient_present = checkSEN0385();
    if (g_ambient_present) {
      float t = 0.0f;
      float h = 0.0f;
      if (read_ambient(t, h)) {
        m.at_c = t;
        m.ah_pct = h;
      } else {
        m.err |= kErrAmbient;
      }
    } else {
      m.err |= kErrAmbient;
    }
  }

  m.st_c = -127.0f;
  if (g_soil_present) {
    float t = 0.0f;
    if (read_soil_temp(t)) {
      m.st_c = t;
    } else {
      m.err |= kErrSoilTemp;
    }
  } else {
    m.err |= kErrSoilTemp;
  }

  return m;
}

void log_measurements(const Measurements &m) {
  Serial.print("SM_RAW=");
  Serial.print(m.sm_raw);
  Serial.print(" counts, SM_V=");
  Serial.print(m.sm_v, 2);
  Serial.print(" V, SM_PCT=");
  Serial.print(m.sm_pct);
  Serial.println(" %");

  Serial.print("ST_C=");
  Serial.print(m.st_c, 2);
  Serial.println(" C");

  Serial.print("AT_C=");
  Serial.print(m.at_c, 2);
  Serial.print(" C, AH_PCT=");
  Serial.print(m.ah_pct, 2);
  Serial.println(" %RH");

  Serial.print("VB_RAW=");
  Serial.print(m.vb_raw);
  Serial.print(" counts, VB_V=");
  Serial.print(m.vb_v, 2);
  Serial.println(" V");
}

}  // namespace

void app_setup() {
  Serial.begin(kBaud);
  delay(50);
  Serial.print("System alive, Node ID = ");
  Serial.println(NODE_ID);

  pinMode(kPinSensorPower, OUTPUT);
  digitalWrite(kPinSensorPower, HIGH);

  ntp_sync();

  if (kEnableLoRa) {
    Serial.println("LoRa init: Mcu.begin...");
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    Serial.println("LoRa init: Radio.Init...");
    g_radio_events.TxDone = on_tx_done;
    g_radio_events.TxTimeout = on_tx_timeout;
    Radio.Init(&g_radio_events);
    Serial.println("LoRa init: Radio.SetChannel...");
    Radio.SetChannel(868100000);
    Serial.println("LoRa init: Radio.SetTxConfig...");
    Radio.SetTxConfig(MODEM_LORA,
                      14,
                      0,
                      0,
                      12,
                      1,
                      8,
                      false,
                      true,
                      0,
                      0,
                      false,
                      3000);
    Serial.println("LoRa init OK");
  } else {
    Serial.println("LoRa init SKIPPED (disabled)");
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Sensor power is active-low: drive LOW and let rails stabilize before any sensor IO.
  digitalWrite(kPinSensorPower, LOW);
  delay(kSensorPowerWarmupMs);

  Wire.begin(kPinI2cSda, kPinI2cScl);
  Wire.setClock(100000);
  g_ambient_present = checkSEN0385();
  Serial.print("SEN0385 ");
  Serial.println(g_ambient_present ? "OK @0x44" : "MISSING/INVALID");

  g_ds18b20.begin();
  g_soil_present = (g_ds18b20.getDeviceCount() > 0);
  if (!g_soil_present) {
    for (uint8_t i = 0; i < 3; ++i) {
      delay(250);
      if (g_ds18b20.getDeviceCount() > 0) {
        g_soil_present = true;
        break;
      }
    }
  }
  Serial.print("DFR0198 ");
  Serial.println(g_soil_present ? "OK" : "MISSING");

  read_measurements();
}

void app_loop() {
  if (kEnableLoRa) {
    Radio.IrqProcess();
  }

  uint32_t now_ms = millis();
  static uint32_t last_meas_log_ms = 0;
  if (now_ms - last_meas_log_ms >= 10000UL) {
    last_meas_log_ms = now_ms;
    Measurements m = read_measurements();
    Serial.println("MEAS (10s update):");
    log_measurements(m);
  }

  if (g_time_valid && (now_ms - g_last_ntp_ms) >= kNtpResyncMs) {
    ntp_sync();
  } else if (!g_time_valid && (now_ms - g_last_ntp_attempt_ms) >= kNtpRetryMs) {
    g_last_ntp_attempt_ms = now_ms;
    ntp_sync();
  }

  static uint32_t last_log_ms = 0;
  time_t now = time(nullptr);
  struct tm utc{};
  gmtime_r(&now, &utc);
  if (now_ms - last_log_ms >= kDebugLogMs) {
    last_log_ms = now_ms;
    if (static_cast<uint32_t>(now) >= kNtpValidEpochMin) {
      int next_slot = next_slot_minute_from_utc(utc);
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d", utc.tm_hour, utc.tm_min, utc.tm_sec);
      Serial.print("UTC ");
      Serial.print(buf);
      Serial.print(" next_slot=");
      if (next_slot >= 0) {
        if (next_slot < 10) {
          Serial.print('0');
        }
        Serial.println(next_slot);
      } else {
        Serial.println("?");
      }
    } else {
      Serial.println("UTC --:--:-- next_slot=--");
    }
  }

  if (!g_time_valid || static_cast<uint32_t>(now) < kNtpValidEpochMin) {
    return;
  }

  if (g_tx_timeout_flag) {
    g_tx_timeout_flag = false;
    Serial.println("TX TIMEOUT");
  }

  static bool burst_active = false;
  static uint8_t burst_seq = 0;
  static uint32_t next_burst_ms = 0;
  static uint16_t last_sent_slot_id = 0xFFFF;
  static uint32_t last_tx_ms = 0;

  uint16_t slot_id = slot_id_from_utc(utc);
  if (!burst_active && in_tx_window(utc) && slot_id != last_sent_slot_id) {
    burst_active = true;
    burst_seq = 0;
    next_burst_ms = now_ms;
    last_sent_slot_id = slot_id;
  }

  if (burst_active && now_ms >= next_burst_ms) {
    if (g_tx_in_progress) {
      delay(10);
      return;
    }

    time_t tx_now = time(nullptr);
    tm tx_utc{};
    gmtime_r(&tx_now, &tx_utc);
    if (!in_tx_window(tx_utc)) {
      burst_active = false;
      delay(10);
      return;
    }

    Measurements m = read_measurements();
    log_measurements(m);

    char payload[220];
    snprintf(payload,
             sizeof(payload),
             "NODE=%d TIME=%lu SEQ=%u SM_PCT=%u ST_C=%.2f AT_C=%.2f AH_PCT=%.2f VB_V=%.2f",
             NODE_ID,
             static_cast<unsigned long>(tx_now),
             burst_seq,
             static_cast<unsigned>(m.sm_pct),
             m.st_c,
             m.at_c,
             m.ah_pct,
             m.vb_v);
    Serial.print("TX ");
    Serial.println(payload);

  g_tx_in_progress = true;
  last_tx_ms = now_ms;
  Radio.Send(reinterpret_cast<uint8_t *>(payload), strlen(payload));

  burst_seq++;
  if (burst_seq >= kBurstCount) {
    burst_active = false;
  } else {
    uint32_t gap = kBurstGapMs;
    if (gap < kBurstMinGapMs) {
      gap = kBurstMinGapMs;
    }
    next_burst_ms = last_tx_ms + gap;
  }
}
}

