#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <vector>

#define RADAR_I2C_ADDR 0x56

// Poll intervals
static const uint32_t POLL_INTERVAL_MS = 200;
static const uint32_t PERIODIC_PUBLISH_MS = 5000; // publish all registers every 5s
static const uint32_t IP_PUBLISH_MS = 10000;      // publish IP every 10s
static const uint32_t PWM_SAMPLE_INTERVAL_MS = 500; // measure PWM every 500 ms

// LEDC for RGB
static const int RGB_TIMER = 0;
static const int RGB_FREQ = 5000;
static const int RGB_RES_BITS = 8;
static const int RGB_CH_R = 0;
static const int RGB_CH_G = 1;
static const int RGB_CH_B = 2;

// Web server, WebSocket and MQTT
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Persistent config
Preferences prefs;

struct AppConfig {
  // WiFi / MQTT
  String wifiSsid;
  String wifiPass;
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPass;
  String mqttBaseTopic; // always ends with '/'

  // GPIO config
  uint8_t sdaPin = 21;       // default ESP32
  uint8_t sclPin = 22;       // default ESP32
  uint8_t digitalPin = 27;   // EN_LED_state input pin
  bool digitalPullup = true; // default pull-up for digital input
  uint8_t pwmPin = 26;       // PWM input pin

  // RGB LED config (PWM via LEDC)
  bool rgbEnable = true;
  uint8_t rgbRPin = 23;
  uint8_t rgbGPin = 32; // GPIO34 is input-only; using 32 by default
  uint8_t rgbBPin = 33;
  bool rgbActiveLow = false; // set true for common-anode LEDs

  // I2C device address (start address)
  uint8_t i2cAddr = RADAR_I2C_ADDR;
};

AppConfig config;

// Derived MQTT topics
String topicSetExact;     // <base>set
String topicSetWildcard;  // <base>+/set
String topicRgbSet;       // <base>rgb/set

// Register definition
struct RegisterDef {
  String name;
  uint8_t address;
  uint8_t length; // 1..4 bytes
};

std::vector<RegisterDef> registers;
std::vector<uint32_t> lastValues;

// EN_LED_state cached
int lastDigitalState = -1;

// PWM cached
int lastPwmDuty = -1; // 0..100 (%)
int lastPwmHz = -1;   // Hz

// RGB cached
int lastRgbR = -1, lastRgbG = -1, lastRgbB = -1;
bool rgbReady = false;

// OTA state
bool otaStarted = false;

// Timers
uint32_t lastPollMs = 0;
uint32_t lastAllPublishMs = 0;
uint32_t lastIpPublishMs = 0;
uint32_t lastPwmSampleMs = 0;

String chipIdHex() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[17];
  snprintf(buf, sizeof(buf), "%06llX%06llX", (mac >> 24) & 0xFFFFFF, mac & 0xFFFFFF);
  return String(buf);
}

static inline bool isOutputCapable(uint8_t pin) {
  return !(pin >= 34 && pin <= 39); // 34-39 are input-only
}

static const char* wifiAuthModeToString(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-EAP";
    case WIFI_AUTH_WPA3_PSK: return "WPA3-PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3-PSK";
#if defined(WIFI_AUTH_WAPI_PSK)
    case WIFI_AUTH_WAPI_PSK: return "WAPI-PSK";
#endif
#if defined(WIFI_AUTH_OWE)
    case WIFI_AUTH_OWE: return "OWE";
#endif
    default: return "UNKNOWN";
  }
}

void rebuildControlTopics() {
  topicSetExact = config.mqttBaseTopic + "set";
  topicSetWildcard = config.mqttBaseTopic + "+/set";
  topicRgbSet = config.mqttBaseTopic + "rgb/set";
}

void loadConfig() {
  prefs.begin("cfg", true);
  config.wifiSsid = prefs.getString("ssid", "");
  config.wifiPass = prefs.getString("wpass", "");
  config.mqttHost = prefs.getString("mhost", "");
  config.mqttPort = prefs.getUShort("mport", 1883);
  config.mqttUser = prefs.getString("muser", "");
  config.mqttPass = prefs.getString("mpass", "");
  config.mqttBaseTopic = prefs.getString("mbase", "");
  // GPIO
  config.sdaPin = prefs.getUChar("sda", 21);
  config.sclPin = prefs.getUChar("scl", 22);
  config.digitalPin = prefs.getUChar("dpin", 27);
  config.digitalPullup = prefs.getBool("dpull", true);
  config.pwmPin = prefs.getUChar("ppin", 26);
  // RGB
  config.rgbEnable = prefs.getBool("rgbEn", true);
  config.rgbRPin = prefs.getUChar("rpin", 23);
  config.rgbGPin = prefs.getUChar("gpin", 32); // default 32 (not 34)
  config.rgbBPin = prefs.getUChar("bpin", 33);
  config.rgbActiveLow = prefs.getBool("rgbInv", false);
  // I2C address
  config.i2cAddr = prefs.getUChar("i2cAddr", RADAR_I2C_ADDR);
  prefs.end();

  if (config.mqttBaseTopic.isEmpty()) {
    config.mqttBaseTopic = "radar/" + chipIdHex() + "/";
  } else {
    if (!config.mqttBaseTopic.endsWith("/")) config.mqttBaseTopic += "/";
  }
  rebuildControlTopics();

  Serial.printf("MQTT base topic: %s\n", config.mqttBaseTopic.c_str());
  Serial.printf("GPIO: SDA=%u, SCL=%u, EN_LED=%u (pullup=%s), PWM=%u\n",
                config.sdaPin, config.sclPin, config.digitalPin, config.digitalPullup ? "true" : "false", config.pwmPin);
  Serial.printf("RGB: enable=%s R=%u G=%u B=%u activeLow=%s\n",
                config.rgbEnable ? "true" : "false", config.rgbRPin, config.rgbGPin, config.rgbBPin,
                config.rgbActiveLow ? "true" : "false");
  Serial.printf("I2C device address: 0x%02X\n", config.i2cAddr);
}

void saveConfig(const AppConfig &c) {
  prefs.begin("cfg", false);
  prefs.putString("ssid", c.wifiSsid);
  prefs.putString("wpass", c.wifiPass);
  prefs.putString("mhost", c.mqttHost);
  prefs.putUShort("mport", c.mqttPort);
  prefs.putString("muser", c.mqttUser);
  prefs.putString("mpass", c.mqttPass);
  prefs.putString("mbase", c.mqttBaseTopic);
  // GPIO
  prefs.putUChar("sda", c.sdaPin);
  prefs.putUChar("scl", c.sclPin);
  prefs.putUChar("dpin", c.digitalPin);
  prefs.putBool("dpull", c.digitalPullup);
  prefs.putUChar("ppin", c.pwmPin);
  // RGB
  prefs.putBool("rgbEn", c.rgbEnable);
  prefs.putUChar("rpin", c.rgbRPin);
  prefs.putUChar("gpin", c.rgbGPin);
  prefs.putUChar("bpin", c.rgbBPin);
  prefs.putBool("rgbInv", c.rgbActiveLow);
  // I2C address
  prefs.putUChar("i2cAddr", c.i2cAddr);
  prefs.end();
}

// Forward declarations for RGB status animations
void startWifiConnectingBlink();
void stopWifiConnectingBlink();
void rgbBlinkAsync(int r, int g, int b, int times, int onMs, int offMs);

bool connectWiFi(uint32_t timeoutMs = 15000) {
  if (config.wifiSsid.isEmpty()) {
    Serial.println("No stored WiFi SSID. Skipping STA connect.");
    return false;
  }
  Serial.printf("Connecting to WiFi SSID: %s\n", config.wifiSsid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);       // don't write creds to system NVS
  WiFi.setAutoConnect(true);    // ensure auto-connect is on
  WiFi.setAutoReconnect(true);  // auto-reconnect after drops
  WiFi.setSleep(false);         // avoid light sleep issues on some APs

  // Start red blink while connecting
  startWifiConnectingBlink();

  WiFi.begin(config.wifiSsid.c_str(), config.wifiPass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  // Stop red blink (either connected or timed out)
  stopWifiConnectingBlink();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
    // Blink blue 3 times on successful connect
    rgbBlinkAsync(0, 0, 255, 3, 120, 120);
    return true;
  } else {
    Serial.println("WiFi connect timed out.");
    return false;
  }
}

void startAP() {
  String apName = "RadarESP32-" + chipIdHex().substring(6);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str());
  Serial.printf("AP Mode started. SSID: %s, IP: %s\n",
                apName.c_str(), WiFi.softAPIP().toString().c_str());
}

// I2C helpers
bool readRegisterRaw(uint8_t deviceAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t got = Wire.requestFrom(deviceAddr, len);
  if (got != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool writeRegisterRaw(uint8_t deviceAddr, uint8_t reg, const uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(buf[i]);
  }
  return Wire.endTransmission(true) == 0;
}

bool writeRegisterValue(const RegisterDef &def, uint32_t value) {
  uint8_t buf[4] = {0};
  for (uint8_t i = 0; i < def.length; i++) {
    buf[i] = (uint8_t)((value >> (8 * i)) & 0xFF);
  }
  return writeRegisterRaw(config.i2cAddr, def.address, buf, def.length);
}

// Load register list from /registers.txt
bool loadRegistersFromFS() {
  registers.clear();
  lastValues.clear();

  if (!LittleFS.exists("/registers.txt")) {
    Serial.println("No /registers.txt found. Creating default.");
    File f = LittleFS.open("/registers.txt", FILE_WRITE);
    if (f) {
      f.println("# name, 0xADDR, length(1..4)");
      f.println("app_info.product,0x00,2");
      f.println("config.tuning.energy_threshold_offset_default,0x30,2");
      f.println("config.tuning.on_duration_default,0x33,2");
      f.println("config.tuning.als_turnOn_threshold_default,0x36,2");
      f.close();
    }
  }

  File file = LittleFS.open("/registers.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open /registers.txt");
    return false;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith("#")) continue;

    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) continue;

    String name = line.substring(0, p1); name.trim();
    String addrStr = line.substring(p1 + 1, p2); addrStr.trim();
    String lenStr = line.substring(p2 + 1); lenStr.trim();

    uint32_t addr = 0;
    if (addrStr.startsWith("0x") || addrStr.startsWith("0X")) {
      addr = strtoul(addrStr.c_str(), nullptr, 16);
    } else {
      addr = strtoul(addrStr.c_str(), nullptr, 10);
    }
    uint8_t len = (uint8_t)strtoul(lenStr.c_str(), nullptr, 10);
    if (len == 0 || len > 4) len = 1;

    RegisterDef def;
    def.name = name;
    def.address = (uint8_t)(addr & 0xFF);
    def.length = len;

    registers.push_back(def);
    lastValues.push_back(0xFFFFFFFF); // sentinel
  }
  file.close();

  Serial.printf("Loaded %u register definitions\n", (unsigned)registers.size());
  return !registers.empty();
}

uint32_t readRegisterValue(const RegisterDef &def, bool &ok) {
  uint8_t buf[4] = {0};
  ok = readRegisterRaw(config.i2cAddr, def.address, buf, def.length);
  if (!ok) return 0;

  // Assemble little-endian into 32-bit
  uint32_t val = 0;
  for (uint8_t i = 0; i < def.length; i++) {
    val |= ((uint32_t)buf[i]) << (8 * i);
  }
  return val;
}

// Unified publisher by address, with optional UI name
void publishAddressValue(uint8_t addr, uint32_t value, const char* uiName = nullptr) {
  // WebSocket JSON message
  JsonDocument doc;
  doc["type"] = "update";
  if (uiName && uiName[0]) {
    doc["reg"] = uiName;
  } else {
    char nm[5]; // "0xNN"
    snprintf(nm, sizeof(nm), "0x%02X", addr);
    doc["reg"] = nm;
  }
  doc["addr"] = addr;
  doc["value"] = value;
  String msg;
  serializeJson(doc, msg);
  ws.textAll(msg);

  // MQTT publish using address as topic suffix: base/0xNN (non-retained)
  if (mqtt.connected() && !config.mqttBaseTopic.isEmpty()) {
    char suffix[6]; // "0x" + 2 hex
    snprintf(suffix, sizeof(suffix), "0x%02X", addr);
    String topic = config.mqttBaseTopic + String(suffix);
    char payload[16];
    snprintf(payload, sizeof(payload), "%lu", (unsigned long)value);
    mqtt.publish(topic.c_str(), payload, false);
  }
}

void broadcastRegisterUpdate(const RegisterDef &def, uint32_t value) {
  publishAddressValue(def.address, value, def.name.c_str());
}

void publishAllRegistersByAddress() {
  if (!mqtt.connected() || config.mqttBaseTopic.isEmpty()) return;
  for (size_t i = 0; i < registers.size(); i++) {
    uint32_t value = lastValues[i];
    if (value == 0xFFFFFFFF) value = 0;
    publishAddressValue(registers[i].address, value, registers[i].name.c_str());
  }
}

// EN_LED_state publisher (digital input)
void publishENLEDState(bool force = false) {
  int state = digitalRead(config.digitalPin) == HIGH ? 1 : 0;
  if (!force && state == lastDigitalState) return;

  lastDigitalState = state;

  // WebSocket notify
  {
    JsonDocument doc;
    doc["type"] = "EN_LED_state";
    doc["value"] = state; // 1 for HIGH, 0 for LOW
    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
  }

  // MQTT publish at <base>EN_LED_state (non-retained)
  if (mqtt.connected() && !config.mqttBaseTopic.isEmpty()) {
    String topic = config.mqttBaseTopic + "EN_LED_state";
    char payload[4];
    snprintf(payload, sizeof(payload), "%d", state);
    mqtt.publish(topic.c_str(), payload, false);
  }
}

// PWM measurement and publisher
bool samplePwmOnce(int &dutyOut, int &hzOut) {
  const uint32_t timeoutUs = 25000; // 25 ms each pulseIn
  uint32_t highUs = pulseIn(config.pwmPin, HIGH, timeoutUs);
  uint32_t lowUs  = pulseIn(config.pwmPin, LOW,  timeoutUs);

  if (highUs == 0 || lowUs == 0) {
    return false;
  }

  uint32_t periodUs = highUs + lowUs;
  if (periodUs == 0) return false;

  float duty = (100.0f * (float)highUs) / (float)periodUs;
  float hz = 1000000.0f / (float)periodUs;

  dutyOut = (int)roundf(duty);
  hzOut = (int)roundf(hz);
  dutyOut = constrain(dutyOut, 0, 100);
  if (hzOut < 0) hzOut = 0;
  return true;
}

void publishPwm(bool force = false) {
  int duty = -1, hz = -1;
  bool ok = samplePwmOnce(duty, hz);
  if (!ok) { duty = -1; hz = -1; }

  bool changed = (duty != lastPwmDuty) || (hz != lastPwmHz);
  if (!force && !changed) return;

  lastPwmDuty = duty;
  lastPwmHz = hz;

  // WebSocket notify
  {
    JsonDocument doc;
    doc["type"] = "pwm_input";
    doc["duty"] = duty; // -1 if no signal
    doc["hz"] = hz;     // -1 if no signal
    String msg; serializeJson(doc, msg);
    ws.textAll(msg);
  }

  // MQTT topics (non-retained)
  if (mqtt.connected() && !config.mqttBaseTopic.isEmpty()) {
    String t1 = config.mqttBaseTopic + "pwm_input_duty";
    String t2 = config.mqttBaseTopic + "pwm_input_hz";
    char b1[8], b2[16];
    snprintf(b1, sizeof(b1), "%d", duty);
    snprintf(b2, sizeof(b2), "%d", hz);
    mqtt.publish(t1.c_str(), b1, false);
    mqtt.publish(t2.c_str(), b2, false);
  }
}

// RGB control
static inline int rgbDutyFromValue(int v, bool activeLow) {
  v = constrain(v, 0, 255);
  return activeLow ? (255 - v) : v;
}

void publishRgbState(bool wsOnly = false) {
  // WebSocket broadcast
  {
    JsonDocument doc;
    doc["type"] = "rgb";
    doc["r"] = lastRgbR;
    doc["g"] = lastRgbG;
    doc["b"] = lastRgbB;
    String msg; serializeJson(doc, msg);
    ws.textAll(msg);
  }

  if (!wsOnly && mqtt.connected() && !config.mqttBaseTopic.isEmpty()) {
    JsonDocument doc;
    doc["r"] = lastRgbR;
    doc["g"] = lastRgbG;
    doc["b"] = lastRgbB;
    String payload; serializeJson(doc, payload);
    mqtt.publish((config.mqttBaseTopic + "rgb").c_str(), payload.c_str(), false);
  }
}

void rgbApply(int r, int g, int b, bool force = false) {
  if (!config.rgbEnable || !rgbReady) return;

  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  bool changed = force || (r != lastRgbR) || (g != lastRgbG) || (b != lastRgbB);
  if (!changed) return;

  ledcWrite(RGB_CH_R, rgbDutyFromValue(r, config.rgbActiveLow));
  ledcWrite(RGB_CH_G, rgbDutyFromValue(g, config.rgbActiveLow));
  ledcWrite(RGB_CH_B, rgbDutyFromValue(b, config.rgbActiveLow));

  lastRgbR = r; lastRgbG = g; lastRgbB = b;
  publishRgbState(false);
}

void setupRgb() {
  rgbReady = false;
  if (!config.rgbEnable) {
    Serial.println("RGB disabled in config.");
    return;
  }
  if (!isOutputCapable(config.rgbRPin) || !isOutputCapable(config.rgbGPin) || !isOutputCapable(config.rgbBPin)) {
    Serial.printf("RGB pin invalid (34-39 are input-only). R=%u G=%u B=%u\n", config.rgbRPin, config.rgbGPin, config.rgbBPin);
    return;
  }

  ledcSetup(RGB_CH_R, RGB_FREQ, RGB_RES_BITS);
  ledcSetup(RGB_CH_G, RGB_FREQ, RGB_RES_BITS);
  ledcSetup(RGB_CH_B, RGB_FREQ, RGB_RES_BITS);

  ledcAttachPin(config.rgbRPin, RGB_CH_R);
  ledcAttachPin(config.rgbGPin, RGB_CH_G);
  ledcAttachPin(config.rgbBPin, RGB_CH_B);

  rgbReady = true;
  // Initialize off
  rgbApply(0, 0, 0, true);
}

// ===== Status animations (non-blocking) =====

// Immediate write without updating lastRgb* or publishing
static inline void rgbWriteImmediate(int r, int g, int b) {
  if (!config.rgbEnable || !rgbReady) return;
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  ledcWrite(RGB_CH_R, rgbDutyFromValue(r, config.rgbActiveLow));
  ledcWrite(RGB_CH_G, rgbDutyFromValue(g, config.rgbActiveLow));
  ledcWrite(RGB_CH_B, rgbDutyFromValue(b, config.rgbActiveLow));
}

// Guard to restore steady color after temporary animation
struct RgbAnimGuard {
  int r, g, b;
  bool valid;
  RgbAnimGuard() {
    valid = (config.rgbEnable && rgbReady);
    if (valid) { r = lastRgbR; g = lastRgbG; b = lastRgbB; }
  }
  ~RgbAnimGuard() {
    if (valid) {
      rgbApply(r, g, b, true);
    }
  }
};

// Continuous red blink while connecting to Wi‑Fi
static TaskHandle_t wifiBlinkTaskHandle = nullptr;
static volatile bool wifiBlinking = false;

static void wifiBlinkTask(void *pv) {
  (void)pv;
  RgbAnimGuard guard;
  while (wifiBlinking) {
    rgbWriteImmediate(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(180));
    rgbWriteImmediate(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  rgbWriteImmediate(0, 0, 0);
  wifiBlinkTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

void startWifiConnectingBlink() {
  if (!config.rgbEnable || !rgbReady) return;
  if (wifiBlinkTaskHandle) return;
  wifiBlinking = true;
  xTaskCreatePinnedToCore(wifiBlinkTask, "wifiBlink", 3072, nullptr, 1, &wifiBlinkTaskHandle, tskNO_AFFINITY);
}

void stopWifiConnectingBlink() {
  if (!wifiBlinkTaskHandle) return;
  wifiBlinking = false;
  // small wait for task to exit
  for (int i = 0; i < 10 && wifiBlinkTaskHandle; ++i) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (wifiBlinkTaskHandle) {
    vTaskDelete(wifiBlinkTaskHandle);
    wifiBlinkTaskHandle = nullptr;
  }
}

// One-shot blink (e.g., blue x3 on connect, green pulse on publish)
struct BlinkParams { int r, g, b, times, onMs, offMs; };

static void rgbBlinkTask(void *pv) {
  BlinkParams *p = static_cast<BlinkParams*>(pv);
  RgbAnimGuard guard;
  int t = p->times <= 0 ? 1 : p->times;
  for (int i = 0; i < t; ++i) {
    rgbWriteImmediate(p->r, p->g, p->b);
    vTaskDelay(pdMS_TO_TICKS(p->onMs));
    rgbWriteImmediate(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(p->offMs));
  }
  delete p;
  vTaskDelete(nullptr);
}

void rgbBlinkAsync(int r, int g, int b, int times, int onMs, int offMs) {
  if (!config.rgbEnable || !rgbReady) return;
  BlinkParams *p = new BlinkParams{
    constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255),
    times, max(1, onMs), max(1, offMs)
  };
  xTaskCreatePinnedToCore(rgbBlinkTask, "rgbBlink", 3072, p, 1, nullptr, tskNO_AFFINITY);
}

// ===== End status animations =====

// State snapshot publish
void publishStateSnapshot() {
  if (!mqtt.connected()) return;

  JsonDocument doc;
  doc["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : "0.0.0.0";
  doc["rssi"] = WiFi.isConnected() ? WiFi.RSSI() : 0;
  if (lastDigitalState >= 0) doc["EN_LED_state"] = lastDigitalState;
  doc["pwmDuty"] = lastPwmDuty;
  doc["pwmHz"] = lastPwmHz;

  JsonObject gpio = doc["gpio"].to<JsonObject>();
  gpio["sda"] = config.sdaPin;
  gpio["scl"] = config.sclPin;
  gpio["digital"] = config.digitalPin;
  gpio["digitalPullup"] = config.digitalPullup;
  gpio["pwm"] = config.pwmPin;
  JsonObject rgb = gpio["rgb"].to<JsonObject>();
  rgb["enable"] = config.rgbEnable;
  rgb["r"] = config.rgbRPin;
  rgb["g"] = config.rgbGPin;
  rgb["b"] = config.rgbBPin;
  rgb["activeLow"] = config.rgbActiveLow;

  JsonArray regsArr = doc["registers"].to<JsonArray>();
  for (size_t i = 0; i < registers.size(); i++) {
    JsonObject o = regsArr.add<JsonObject>();
    o["name"] = registers[i].name;
    o["addr"] = registers[i].address;
    o["len"] = registers[i].length;
    o["value"] = lastValues[i] == 0xFFFFFFFF ? 0 : lastValues[i];
  }

  String payload;
  serializeJson(doc, payload);
  String topic = config.mqttBaseTopic + "state";
  mqtt.publish(topic.c_str(), payload.c_str(), false);

  // Also publish RGB state separately
  publishRgbState(true);  // ws only
  publishRgbState(false); // mqtt+ws
}

// Publish IP to MQTT every 10s (non-retained)
void publishIp(bool force = false) {
  if (!mqtt.connected() || config.mqttBaseTopic.isEmpty()) return;
  uint32_t now = millis();
  if (!force && (now - lastIpPublishMs < IP_PUBLISH_MS)) return;

  String topic = config.mqttBaseTopic + "ip";
  String ip = WiFi.isConnected() ? WiFi.localIP().toString() : "0.0.0.0";
  mqtt.publish(topic.c_str(), ip.c_str(), false);
  lastIpPublishMs = now;
  Serial.printf("Published IP to %s: %s\n", topic.c_str(), ip.c_str());
}

// Parse "0xNN" or decimal into uint8_t
bool parseAddrString(const String &s, uint8_t &addrOut) {
  String t = s; t.trim();
  if (t.startsWith("0x") || t.startsWith("0X")) {
    addrOut = (uint8_t)strtoul(t.c_str() + 2, nullptr, 16);
  } else {
    addrOut = (uint8_t)strtoul(t.c_str(), nullptr, 10);
  }
  return true;
}

bool parseAddrVariant(const JsonVariantConst &v, uint8_t &addrOut) {
  if (v.is<const char*>()) {
    return parseAddrString(String(v.as<const char*>()), addrOut);
  } else if (v.is<uint32_t>()) {
    addrOut = (uint8_t)v.as<uint32_t>();
    return true;
  }
  return false;
}

// MQTT message handler
void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  String t(topic);

  // Copy payload
  String body; body.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) body += (char)payload[i];
  String bodyTrim = body; bodyTrim.trim();

  // RGB control topic
  if (t == topicRgbSet) {
    // Accept JSON {"r":0..255,"g":..,"b":..} or {"hex":"#RRGGBB"} or plain "#RRGGBB"
    int r = lastRgbR < 0 ? 0 : lastRgbR, g = lastRgbG < 0 ? 0 : lastRgbG, b = lastRgbB < 0 ? 0 : lastRgbB;
    bool got = false;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (const uint8_t*)body.c_str(), body.length());
    if (!err) {
      if (doc["hex"].is<const char*>()) {
        const char* hx = doc["hex"];
        if (hx && hx[0] == '#') {
          unsigned v = strtoul(hx + 1, nullptr, 16);
          r = (v >> 16) & 0xFF; g = (v >> 8) & 0xFF; b = v & 0xFF; got = true;
        }
      }
      if (doc["color"].is<const char*>()) {
        const char* hx = doc["color"];
        if (hx && hx[0] == '#') {
          unsigned v = strtoul(hx + 1, nullptr, 16);
          r = (v >> 16) & 0xFF; g = (v >> 8) & 0xFF; b = v & 0xFF; got = true;
        }
      }
      if (doc["r"].is<int>() || doc["g"].is<int>() || doc["b"].is<int>()) {
        if (doc["r"].is<int>()) r = doc["r"].as<int>();
        if (doc["g"].is<int>()) g = doc["g"].as<int>();
        if (doc["b"].is<int>()) b = doc["b"].as<int>();
        got = true;
      }
    } else if (bodyTrim.length() == 7 && bodyTrim[0] == '#') {
      unsigned v = strtoul(bodyTrim.c_str() + 1, nullptr, 16);
      r = (v >> 16) & 0xFF; g = (v >> 8) & 0xFF; b = v & 0xFF; got = true;
    }

    if (got) {
      rgbApply(r, g, b, true);
    } else {
      Serial.println("RGB set: invalid payload");
    }
    return;
  }

  // Register write control topics
  uint8_t addr = 0;
  bool addrOk = false;
  uint32_t value = 0;
  bool valueOk = false;

  if (t == topicSetExact) {
    // Expect full JSON {"reg":"0xNN","value":...}
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (const uint8_t*)body.c_str(), body.length());
    if (err) {
      Serial.printf("MQTT set: invalid JSON (%s)\n", err.c_str());
      return;
    }
    addrOk = parseAddrVariant(doc["reg"], addr);
    if (doc["value"].is<uint32_t>()) {
      value = doc["value"].as<uint32_t>();
      valueOk = true;
    } else if (doc["value"].is<int>()) {
      value = (uint32_t)doc["value"].as<int>();
      valueOk = true;
    } else if (doc["value"].is<const char*>()) {
      String s = doc["value"].as<const char*>(); s.trim();
      if (s.startsWith("0x") || s.startsWith("0X")) value = (uint32_t)strtoul(s.c_str() + 2, nullptr, 16);
      else value = (uint32_t)strtoul(s.c_str(), nullptr, 10);
      valueOk = true;
    }
    if (!addrOk || !valueOk) {
      Serial.println("MQTT set: missing/invalid 'reg' or 'value'");
      return;
    }
  } else if (t.startsWith(config.mqttBaseTopic) && t.endsWith("/set")) {
    // Pattern <base><addr>/set
    int baseLen = config.mqttBaseTopic.length();
    int endIdx = t.length() - 4; // strip "/set"
    String addrSeg = t.substring(baseLen, endIdx);
    addrSeg.trim();
    addrOk = parseAddrString(addrSeg, addr);

    // Payload can be JSON {"value":...} or plain number
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (const uint8_t*)body.c_str(), body.length());
    if (!err) {
      if (doc["value"].is<uint32_t>()) {
        value = doc["value"].as<uint32_t>();
        valueOk = true;
      } else if (doc["value"].is<int>()) {
        value = (uint32_t)doc["value"].as<int>();
        valueOk = true;
      } else if (doc["value"].is<const char*>()) {
        String s = doc["value"].as<const char*>(); s.trim();
        if (s.startsWith("0x") || s.startsWith("0X")) value = (uint32_t)strtoul(s.c_str() + 2, nullptr, 16);
        else value = (uint32_t)strtoul(s.c_str(), nullptr, 10);
        valueOk = true;
      }
    }
    if (!valueOk) {
      // Try parse plain numeric payload
      String s = body; s.trim();
      if (s.length()) {
        if (s.startsWith("0x") || s.startsWith("0X")) value = (uint32_t)strtoul(s.c_str() + 2, nullptr, 16);
        else value = (uint32_t)strtoul(s.c_str(), nullptr, 10);
        valueOk = true;
      }
    }
    if (!addrOk || !valueOk) {
      Serial.println("MQTT addr/set: invalid address or value");
      return;
    }
  } else {
    // Not a control topic we care about
    return;
  }

  // Find register definition (for length and name)
  int idx = -1;
  for (size_t i = 0; i < registers.size(); i++) {
    if (registers[i].address == addr) { idx = (int)i; break; }
  }

  bool ok = false;
  const char* uiName = nullptr;

  if (idx >= 0) {
    uint8_t len = registers[idx].length;
    uint32_t mask = (len >= 4) ? 0xFFFFFFFFu : ((1u << (8 * len)) - 1u);
    uint32_t clipped = value & mask;
    ok = writeRegisterValue(registers[idx], clipped);
    if (ok) {
      lastValues[idx] = clipped;
      uiName = registers[idx].name.c_str();
    }
  } else {
    // Unknown register -> attempt 1 byte write
    uint8_t b = (uint8_t)(value & 0xFF);
    ok = writeRegisterRaw(config.i2cAddr, addr, &b, 1);
  }

  if (ok) {
    publishAddressValue(addr, (idx >= 0) ? lastValues[idx] : (uint32_t)(value & 0xFF), uiName);
    Serial.printf("MQTT set OK: 0x%02X <= %lu\n", addr, (unsigned long)value);
  } else {
    Serial.printf("MQTT set FAILED: 0x%02X\n", addr);
  }
}

void setupMqtt() {
  if (config.mqttHost.isEmpty()) {
    Serial.println("MQTT host not set. Skipping.");
    return;
  }
  mqtt.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqtt.setBufferSize(2048);
  mqtt.setCallback(onMqttMessage);
}

void subscribeControlTopics() {
  if (!mqtt.connected()) return;
  bool s1 = mqtt.subscribe(topicSetExact.c_str());
  Serial.printf("Subscribe %s -> %s\n", topicSetExact.c_str(), s1 ? "OK" : "FAIL");
  bool s2 = mqtt.subscribe(topicSetWildcard.c_str());
  Serial.printf("Subscribe %s -> %s\n", topicSetWildcard.c_str(), s2 ? "OK" : "FAIL");
  bool s3 = mqtt.subscribe(topicRgbSet.c_str());
  Serial.printf("Subscribe %s -> %s\n", topicRgbSet.c_str(), s3 ? "OK" : "FAIL");
}

void ensureMqttConnected() {
  if (mqtt.connected()) return;
  if (config.mqttHost.isEmpty()) return;
  String clientId = "radar-" + chipIdHex();
  Serial.printf("Connecting to MQTT %s:%u ...\n", config.mqttHost.c_str(), config.mqttPort);
  bool ok;
  if (config.mqttUser.isEmpty()) {
    ok = mqtt.connect(clientId.c_str());
  } else {
    ok = mqtt.connect(clientId.c_str(), config.mqttUser.c_str(), config.mqttPass.c_str());
  }
  if (ok) {
    Serial.println("MQTT connected.");
    subscribeControlTopics();
    publishStateSnapshot();         // JSON snapshot on connect
    publishAllRegistersByAddress(); // seed register topics on connect
    publishENLEDState(true);        // publish EN_LED_state on connect
    publishPwm(true);               // publish PWM on connect
    publishIp(true);                // publish IP immediately on connect
    publishRgbState(false);         // publish RGB state
  } else {
    Serial.printf("MQTT connect failed rc=%d\n", mqtt.state());
  }
}

// OTA setup/begin after WiFi connects
void setupOTA() {
  if (otaStarted) return;
  String host = "radar-" + chipIdHex();
  ArduinoOTA.setHostname(host.c_str());
  // ArduinoOTA.setPassword("your-ota-password");

  ArduinoOTA.onStart([]() { Serial.println("OTA: Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t last = 255;
    uint8_t pct = (total ? (progress * 100 / total) : 0);
    if (pct != last) { last = pct; Serial.printf("OTA: %u%%\n", pct); }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA ready. Hostname: %s, IP: %s\n", host.c_str(), WiFi.localIP().toString().c_str());
}

void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                          void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client %u connected\n", client->id());
    JsonDocument doc;
    doc["type"] = "hello";
    doc["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    doc["ap"] = WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA;
    if (lastDigitalState >= 0) doc["EN_LED_state"] = lastDigitalState;
    doc["pwmDuty"] = lastPwmDuty;
    doc["pwmHz"] = lastPwmHz;
    doc["rgbR"] = lastRgbR;
    doc["rgbG"] = lastRgbG;
    doc["rgbB"] = lastRgbB;
    String s; serializeJson(doc, s);
    client->text(s.c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client %u disconnected\n", client->id());
  }
}

void setupWebServer() {
  ws.onEvent(handleWebSocketEvent);
  server.addHandler(&ws);

  // Serve UI from LittleFS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/index.html", "text/html");
  });
  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/config.html", "text/html");
  });
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/script.js", "application/javascript");
  });
  server.on("/config.js", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/config.js", "application/javascript");
  });
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/styles.css", "text/css");
  });

  // Expose registers file for editing
  server.on("/registers.txt", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/registers.txt", "text/plain");
  });
  server.on("/registers.txt", HTTP_POST, [](AsyncWebServerRequest *req) {}, nullptr,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
      static File f;
      if (index == 0) {
        f = LittleFS.open("/registers.txt", FILE_WRITE);
      }
      if (f) f.write(data, len);
      if (index + len == total) {
        if (f) f.close();
        loadRegistersFromFS();
        req->send(200, "text/plain", "OK");
      }
    });

  // API to get current registers and values + status
  server.on("/api/registers", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    JsonArray arr = doc["registers"].to<JsonArray>();
    for (size_t i = 0; i < registers.size(); i++) {
      JsonObject o = arr.add<JsonObject>();
      o["name"] = registers[i].name;
      o["address"] = registers[i].address;
      o["length"] = registers[i].length;
      o["value"] = lastValues[i] == 0xFFFFFFFF ? 0 : lastValues[i];
    }
    JsonObject status = doc["status"].to<JsonObject>();
    status["wifiConnected"] = WiFi.isConnected();
    status["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    status["mqttConnected"] = mqtt.connected();
    if (lastDigitalState >= 0) status["EN_LED_state"] = lastDigitalState;
    status["pwmDuty"] = lastPwmDuty;
    status["pwmHz"] = lastPwmHz;
    status["sda"] = config.sdaPin;
    status["scl"] = config.sclPin;
    status["digitalPin"] = config.digitalPin;
    status["digitalPullup"] = config.digitalPullup;
    status["pwmPin"] = config.pwmPin;
    String s; serializeJson(doc, s);
    req->send(200, "application/json", s);
  });

  // API RGB: get/set current color
  server.on("/api/rgb", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    doc["enabled"] = config.rgbEnable;
    doc["activeLow"] = config.rgbActiveLow;
    doc["r"] = lastRgbR < 0 ? 0 : lastRgbR;
    doc["g"] = lastRgbG < 0 ? 0 : lastRgbG;
    doc["b"] = lastRgbB < 0 ? 0 : lastRgbB;
    String s; serializeJson(doc, s);
    req->send(200, "application/json", s);
  });
  server.on("/api/rgb", HTTP_POST, [](AsyncWebServerRequest *req) {}, nullptr,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (err) { req->send(400, "application/json", "{\"error\":\"invalid json\"}"); return; }
      int r = lastRgbR < 0 ? 0 : lastRgbR;
      int g = lastRgbG < 0 ? 0 : lastRgbG;
      int b = lastRgbB < 0 ? 0 : lastRgbB;
      if (doc["color"].is<const char*>()) {
        const char* hx = doc["color"];
        if (hx && hx[0] == '#') {
          unsigned v = strtoul(hx + 1, nullptr, 16);
          r = (v >> 16) & 0xFF; g = (v >> 8) & 0xFF; b = v & 0xFF;
        }
      }
      if (doc["r"].is<int>()) r = doc["r"].as<int>();
      if (doc["g"].is<int>()) g = doc["g"].as<int>();
      if (doc["b"].is<int>()) b = doc["b"].as<int>();
      rgbApply(r, g, b, true);
      req->send(200, "application/json", "{\"ok\":true}");
    });

  // API to get config (hides secrets)
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    doc["ssid"] = config.wifiSsid;
    doc["mqttHost"] = config.mqttHost;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttUser"] = config.mqttUser;
    doc["mqttBaseTopic"] = config.mqttBaseTopic;
    // GPIO
    doc["sdaPin"] = config.sdaPin;
    doc["sclPin"] = config.sclPin;
    doc["digitalPin"] = config.digitalPin;
    doc["digitalPullup"] = config.digitalPullup;
    doc["pwmPin"] = config.pwmPin;
    // I2C address
    doc["i2cAddr"] = config.i2cAddr;
    // RGB
    JsonObject rgb = doc["rgb"].to<JsonObject>();
    rgb["enable"] = config.rgbEnable;
    rgb["rPin"] = config.rgbRPin;
    rgb["gPin"] = config.rgbGPin;
    rgb["bPin"] = config.rgbBPin;
    rgb["activeLow"] = config.rgbActiveLow;
    String s; serializeJson(doc, s);
    req->send(200, "application/json", s);
  });

  // Save config, then reboot to apply
  server.on("/api/save-config", HTTP_POST,
    [](AsyncWebServerRequest *req) {},
    nullptr,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (err) {
        req->send(400, "application/json", "{\"error\":\"invalid json\"}");
        return;
      }
      AppConfig newCfg = config;
      // WiFi/MQTT updates
      if (!doc["ssid"].isNull()) newCfg.wifiSsid = (const char*)doc["ssid"];
      if (!doc["wifipass"].isNull()) {
        const char* w = doc["wifipass"];
        if (w && strlen(w)) newCfg.wifiPass = w; // only update if non-empty
      }
      if (!doc["mqttHost"].isNull()) newCfg.mqttHost = (const char*)doc["mqttHost"];
      if (!doc["mqttPort"].isNull()) newCfg.mqttPort = (uint16_t)doc["mqttPort"].as<uint16_t>();
      if (!doc["mqttUser"].isNull()) newCfg.mqttUser = (const char*)doc["mqttUser"];
      if (!doc["mqttPass"].isNull()) {
        const char* p = doc["mqttPass"];
        if (p && strlen(p)) newCfg.mqttPass = p; // only update if non-empty
      }
      if (!doc["mqttBaseTopic"].isNull()) {
        newCfg.mqttBaseTopic = (const char*)doc["mqttBaseTopic"];
        if (!newCfg.mqttBaseTopic.endsWith("/")) newCfg.mqttBaseTopic += "/";
      }
      // GPIO updates
      if (!doc["sdaPin"].isNull()) newCfg.sdaPin = (uint8_t)doc["sdaPin"].as<uint8_t>();
      if (!doc["sclPin"].isNull()) newCfg.sclPin = (uint8_t)doc["sclPin"].as<uint8_t>();
      if (!doc["digitalPin"].isNull()) newCfg.digitalPin = (uint8_t)doc["digitalPin"].as<uint8_t>();
      if (!doc["digitalPullup"].isNull()) newCfg.digitalPullup = doc["digitalPullup"].as<bool>();
      if (!doc["pwmPin"].isNull()) newCfg.pwmPin = (uint8_t)doc["pwmPin"].as<uint8_t>();
      // I2C address update
      if (!doc["i2cAddr"].isNull()) {
        uint32_t a = doc["i2cAddr"].as<uint32_t>();
        if (a > 127) a = 127;
        newCfg.i2cAddr = (uint8_t)a;
      }
      // RGB updates
      if (!doc["rgb"].isNull()) {
        JsonVariant rgb = doc["rgb"];
        if (!rgb["enable"].isNull()) newCfg.rgbEnable = rgb["enable"].as<bool>();
        if (!rgb["rPin"].isNull()) newCfg.rgbRPin = (uint8_t)rgb["rPin"].as<uint8_t>();
        if (!rgb["gPin"].isNull()) newCfg.rgbGPin = (uint8_t)rgb["gPin"].as<uint8_t>();
        if (!rgb["bPin"].isNull()) newCfg.rgbBPin = (uint8_t)rgb["bPin"].as<uint8_t>();
        if (!rgb["activeLow"].isNull()) newCfg.rgbActiveLow = rgb["activeLow"].as<bool>();
      }

      saveConfig(newCfg);
      String resp = "{\"ok\":true,\"rebooting\":true}";
      req->send(200, "application/json", resp);
      delay(250);
      ESP.restart();
    });

  // Simple health endpoint
  server.on("/api/health", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // Wi‑Fi scanning API
  server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *req) {
    // Preserve current mode so AP mode (if active) is not lost
    wifi_mode_t prev = WiFi.getMode();

    // Ensure STA is enabled for scanning, but keep AP if it was on
    if (prev == WIFI_OFF) {
      WiFi.mode(WIFI_STA);
    } else if (prev == WIFI_AP) {
      WiFi.mode(WIFI_AP_STA);
    }

    // Perform a blocking scan, include hidden networks if supported
    int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    if (n > 0) {
      for (int i = 0; i < n; i++) {
        JsonObject o = arr.add<JsonObject>();
        o["ssid"] = WiFi.SSID(i);
        o["rssi"] = WiFi.RSSI(i);
        o["bssid"] = WiFi.BSSIDstr(i);
        o["channel"] = WiFi.channel(i);
        wifi_auth_mode_t sec = (wifi_auth_mode_t)WiFi.encryptionType(i);
        o["secure"] = (int)sec;
        o["secureStr"] = wifiAuthModeToString(sec);
        o["hidden"] = false; // WiFiClass may not expose per-network hidden flag
      }
    }

    // Free scan results
    WiFi.scanDelete();

    // Restore the previous mode
    if (prev == WIFI_OFF) {
      WiFi.mode(WIFI_OFF);
    } else if (prev == WIFI_AP) {
      WiFi.mode(WIFI_AP);
    } else if (prev == WIFI_STA) {
      WiFi.mode(WIFI_STA);
    } else if (prev == WIFI_AP_STA) {
      WiFi.mode(WIFI_AP_STA);
    }

    String s;
    serializeJson(doc, s);
    req->send(200, "application/json", s);
  });

  server.begin();
  Serial.println("HTTP server started.");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Filesystem
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  // Load config and registers
  loadConfig();
  loadRegistersFromFS();

  // I2C
  Wire.begin(config.sdaPin, config.sclPin);
  Serial.printf("I2C initialized on SDA=%u SCL=%u\n", config.sdaPin, config.sclPin);

  // IO setup
  pinMode(config.digitalPin, config.digitalPullup ? INPUT_PULLUP : INPUT);
  pinMode(config.pwmPin, INPUT);
  lastDigitalState = digitalRead(config.digitalPin) == HIGH ? 1 : 0;

  setupRgb();

  // WiFi / AP
  bool wifiOk = connectWiFi();
  if (!wifiOk) {
    startAP();
  } else {
    setupOTA(); // start OTA once WiFi is up
  }

  // Web + MQTT
  setupWebServer();
  setupMqtt();
}

void loop() {
  // WiFi reconnect watchdog
  static uint32_t lastWifiAttempt = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiAttempt > 10000) { // every 10s
      lastWifiAttempt = millis();
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.reconnect();
      // Fallback: explicitly begin with stored creds if still not connected
      if (WiFi.status() != WL_CONNECTED && !config.wifiSsid.isEmpty()) {
        WiFi.begin(config.wifiSsid.c_str(), config.wifiPass.c_str());
      }
    }
  } else {
    // reset backoff when connected
    lastWifiAttempt = 0;
  }

  // Maintain OTA when WiFi is up; restart OTA if WiFi reconnected
  if (!WiFi.isConnected()) {
    otaStarted = false; // force re-begin next time WiFi returns
  } else if (!otaStarted) {
    setupOTA();
  }
  if (otaStarted) ArduinoOTA.handle();

  // MQTT reconnect and loop
  static uint32_t lastMqttAttempt = 0;
  if (WiFi.isConnected()) {
    mqtt.loop();
    if (!mqtt.connected() && millis() - lastMqttAttempt > 5000) {
      lastMqttAttempt = millis();
      ensureMqttConnected();
    }
  }

  // Poll registers + EN_LED_state
  uint32_t now = millis();
  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;

    // EN_LED_state check
    publishENLEDState(false);

    // Registers
    for (size_t i = 0; i < registers.size(); i++) {
      bool ok;
      uint32_t val = readRegisterValue(registers[i], ok);
      if (!ok) {
        continue;
      }
      if (lastValues[i] != val) {
        lastValues[i] = val;
        broadcastRegisterUpdate(registers[i], val);
      }
    }
  }

  // PWM sampling (blocking up to ~50ms worst case, but only every 500ms)
  if (now - lastPwmSampleMs >= PWM_SAMPLE_INTERVAL_MS) {
    lastPwmSampleMs = now;
    publishPwm(false);
  }

  // Periodic full publish of all registers every 5 seconds
  if (mqtt.connected() && (now - lastAllPublishMs >= PERIODIC_PUBLISH_MS)) {
    lastAllPublishMs = now;
    publishAllRegistersByAddress();
    publishENLEDState(true); // refresh EN_LED_state
    publishPwm(true);        // refresh PWM
    publishRgbState(false);  // refresh RGB
    // Green pulse each time a full publish cycle completes
    rgbBlinkAsync(0, 255, 0, 1, 120, 100);
  }

  // Publish IP every 10 seconds
  if (mqtt.connected()) {
    publishIp(false);
  }
}