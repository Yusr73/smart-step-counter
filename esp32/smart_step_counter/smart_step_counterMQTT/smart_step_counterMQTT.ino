/*
  ESP32 Smart Step Counter (Wi-Fi / MQTT, web dashboard mode)
  
  Added Features:
  - RGB LED status indicators (common-anode)
  - Battery monitoring and publishing
  - All existing MQTT/NTP/day tracking preserved

  MQTT:
    - Publish: smartsteps/steps
    - Subscribe: smartsteps/config

  Commands on smartsteps/config:
    - "reset"       -> stepCount = 0
    - "goal:8000"   -> store stepGoal (EEPROM) + update RAM
    - "sens:1.35"   -> store sensitivity (EEPROM) + update RAM
    - "readbattery" -> force battery reading

  Features:
    - Step detection (simple peak detection on high-pass filtered accel magnitude)
    - stepCount in RAM
    - stepGoal + sensitivity in EEPROM
    - NTP-based new day detection -> reset stepCount
    - Goal reached event only once per day
    - RGB LED signaling for status
    - Sampling & publish rate reduced after reaching goal
    - Battery monitoring and reporting
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

#include <Wire.h>
#include <MMA7660.h>

#include <EEPROM.h>

/* ----------------------------
   WIFI CONFIG
---------------------------- */
const char* ssid     = "TOPNET_28A8";     // ✅ Updated
const char* password = "thjpaep5u3";      // ✅ Updated

/* ----------------------------
   MQTT CONFIG
---------------------------- */
const char* mqtt_server   = "broker.emqx.io";
const int   mqtt_port     = 1883;
const char* mqtt_client_id = "ESP32_SmartSteps";
const char* mqtt_username  = "";
const char* mqtt_password  = "";

static const char* TOPIC_STEPS  = "smartsteps/steps";
static const char* TOPIC_CONFIG = "smartsteps/config";

WiFiClient espClient;
PubSubClient client(espClient);

/* ----------------------------
   RGB LED (COMMON-ANODE)
---------------------------- */
#define LED_Red   16
#define LED_Green 13
#define LED_Blue  12

/* ----------------------------
   EEPROM LAYOUT
---------------------------- */
static const int EEPROM_SIZE = 64;
static const int ADDR_MAGIC       = 0;
static const int ADDR_STEP_GOAL   = 4;
static const int ADDR_SENSITIVITY = 8;
static const uint32_t EEPROM_MAGIC = 0x53544550;

/* ----------------------------
   GLOBAL STATE
---------------------------- */
volatile uint32_t stepCount = 0;
int32_t stepGoal = 6000;
float sensitivity = 1.25f;

bool goalReachedToday = false;
int  currentDayKey = -1;

/* ----------------------------
   POWER / SAMPLING CONTROL
---------------------------- */
uint32_t sampleIntervalMs_beforeGoal = 50;
uint32_t sampleIntervalMs_afterGoal  = 200;

uint32_t publishStepsInterval_beforeGoal = 5000;
uint32_t publishStepsInterval_afterGoal  = 20000;
uint32_t publishBatteryIntervalMs        = 60000;

uint32_t lastSampleMs = 0;
uint32_t lastPublishStepsMs = 0;
uint32_t lastPublishBatteryMs = 0;

/* ----------------------------
   STEP DETECTION
---------------------------- */
float g_est = 9.81f;
float prev_hp = 0.0f;
uint32_t lastStepMs = 0;
uint32_t minStepIntervalMs = 280;

/* ----------------------------
   BATTERY MONITORING
---------------------------- */
const int BAT_ADC_PIN = 34;
const bool BATTERY_ENABLED = true;
const float ADC_REF_V = 3.3f;
const float DIVIDER_RATIO = 2.0f;
const float BAT_V_MIN = 3.30f;
const float BAT_V_MAX = 4.20f;
int batteryPercent = 100;

/* ----------------------------
   ACCELEROMETER
---------------------------- */
MMA7660 accel;

bool initAccelerometer() {
  Serial.println("[ACCEL] Initializing MMA7660...");
  Wire.begin();
  accel.init();

  int8_t x, y, z;
  accel.getXYZ(&x, &y, &z);

  accel.setSampleRate(64);

  Serial.println("[ACCEL] MMA7660 initialized OK.");
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  int8_t x, y, z;
  accel.getXYZ(&x, &y, &z);

  const float scale = (1.5f / 32.0f) * 9.81f;

  ax = (float)x * scale;
  ay = (float)y * scale;
  az = (float)z * scale;

  return true;
}

/* ----------------------------
   RGB LED HELPERS
---------------------------- */
void rgbOff() {
  digitalWrite(LED_Red,   HIGH);
  digitalWrite(LED_Green, HIGH);
  digitalWrite(LED_Blue,  HIGH);
}

void rgbColor(bool r, bool g, bool b) {
  digitalWrite(LED_Red,   r ? LOW : HIGH);
  digitalWrite(LED_Green, g ? LOW : HIGH);
  digitalWrite(LED_Blue,  b ? LOW : HIGH);
}

void rgbBlink(bool r, bool g, bool b, int times, int onMs=120, int offMs=120) {
  for (int i = 0; i < times; i++) {
    rgbColor(r, g, b);
    delay(onMs);
    rgbOff();
    delay(offMs);
  }
}

/* ----------------------------
   STATUS SIGNALS (RGB LED)
---------------------------- */
void signalWiFiOK() {
  Serial.println("[LED] WiFi OK (Cyan blink)");
  rgbBlink(false, true, true, 2, 80, 120);
}

void signalMQTTOK() {
  Serial.println("[LED] MQTT OK (Purple blink)");
  rgbBlink(true, false, true, 3, 70, 90);
}

void signalError() {
  Serial.println("[LED] ERROR (Red blink)");
  rgbBlink(true, false, false, 1, 600, 250);
}

void signalReset() {
  Serial.println("[LED] RESET (Blue blink)");
  rgbBlink(false, false, true, 1, 150, 150);
}

void signalGoal() {
  Serial.println("[LED] GOAL (Green blink)");
  rgbBlink(false, true, false, 5, 90, 90);
}

void signalStepDetected() {
  // Brief orange flash for each step
  rgbBlink(true, true, false, 1, 40, 40);
}

void signalConnectedIdle() {
  // Cyan solid when connected and idle
  rgbColor(false, true, true);
}

void signalAdvertising() {
  // Purple solid when waiting for nothing (WiFi connecting)
  rgbColor(true, false, true);
}

/* ----------------------------
   TIME (NTP)
---------------------------- */
void initTimeNTP() {
  Serial.println("[TIME] Syncing NTP...");
  configTime(3600, 0, "pool.ntp.org", "time.google.com");
  rgbBlink(true, true, true, 3, 100, 100); // White blink during time sync
}

bool timeIsValid() {
  return time(nullptr) > 1700000000;
}

int computeDayKey() {
  if (!timeIsValid()) return -1;
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  return (t.tm_year + 1900) * 1000 + t.tm_yday;
}

void handleNewDayIfNeeded() {
  int dayKey = computeDayKey();
  if (dayKey < 0) return;

  if (currentDayKey == -1) {
    currentDayKey = dayKey;
    Serial.printf("[DAY] Initial day key set: %d\n", dayKey);
    return;
  }

  if (dayKey != currentDayKey) {
    Serial.println("[DAY] New day detected → resetting stepCount");
    stepCount = 0;
    goalReachedToday = false;
    currentDayKey = dayKey;
    signalReset(); // Blue blink for new day reset
  }
}

/* ----------------------------
   EEPROM LOAD/SAVE
---------------------------- */
template <typename T>
void eepromWrite(int addr, const T &value) { EEPROM.put(addr, value); }

template <typename T>
void eepromRead(int addr, T &value) { EEPROM.get(addr, value); }

void loadPersistentConfig() {
  EEPROM.begin(EEPROM_SIZE);

  uint32_t magic = 0;
  eepromRead(ADDR_MAGIC, magic);

  if (magic != EEPROM_MAGIC) {
    Serial.println("[EEPROM] First run → writing defaults");
    magic = EEPROM_MAGIC;
    stepGoal = 6000;
    sensitivity = 1.25f;

    eepromWrite(ADDR_MAGIC, magic);
    eepromWrite(ADDR_STEP_GOAL, stepGoal);
    eepromWrite(ADDR_SENSITIVITY, sensitivity);
    EEPROM.commit();
    return;
  }

  eepromRead(ADDR_STEP_GOAL, stepGoal);
  eepromRead(ADDR_SENSITIVITY, sensitivity);

  Serial.printf("[EEPROM] Loaded goal=%d sensitivity=%.2f\n", stepGoal, sensitivity);
}

void saveStepGoal(int32_t newGoal) {
  stepGoal = newGoal;
  eepromWrite(ADDR_STEP_GOAL, stepGoal);
  EEPROM.commit();
  Serial.printf("[EEPROM] Goal updated → %d\n", stepGoal);
  rgbBlink(true, true, false, 2, 100, 100); // Yellow blink for setting change
}

void saveSensitivity(float newSens) {
  sensitivity = newSens;
  eepromWrite(ADDR_SENSITIVITY, sensitivity);
  EEPROM.commit();
  Serial.printf("[EEPROM] Sensitivity updated → %.2f\n", sensitivity);
  rgbBlink(true, true, false, 2, 100, 100); // Yellow blink for setting change
}

/* ----------------------------
   WIFI + MQTT
---------------------------- */
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to SSID: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  signalAdvertising(); // Purple while connecting
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\n[WiFi] Connected!");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());
  signalWiFiOK();
}

void mqttCallback(char* topic, byte* payload, unsigned int length);

void connectMQTT() {
  Serial.println("[MQTT] Connecting...");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  rgbBlink(false, true, false, 3, 100, 100); // Green blink while connecting MQTT
  
  while (!client.connected()) {
    bool ok = client.connect(mqtt_client_id);
    if (ok) {
      Serial.println("[MQTT] Connected!");
      client.subscribe(TOPIC_CONFIG);
      Serial.printf("[MQTT] Subscribed to %s\n", TOPIC_CONFIG);
      signalMQTTOK();
      signalConnectedIdle(); // Cyan solid when connected and idle
    } else {
      Serial.printf("[MQTT] Failed, rc=%d\n", client.state());
      signalError();
      delay(1200);
    }
  }
}

/* ----------------------------
   BATTERY READING
---------------------------- */
int readBatteryPercent() {
  if (!BATTERY_ENABLED) return -1;
  
  // Take average of 5 readings for stability
  long sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(BAT_ADC_PIN);
    delay(2);
  }
  int raw = sum / 5;
  
  float v = ((float)raw / 4095.0f) * ADC_REF_V * DIVIDER_RATIO;
  v = constrain(v, BAT_V_MIN, BAT_V_MAX);
  
  // Calculate percentage
  batteryPercent = (int)((v - BAT_V_MIN) / (BAT_V_MAX - BAT_V_MIN) * 100.0f);
  batteryPercent = constrain(batteryPercent, 0, 100);
  
  Serial.printf("[BAT] Voltage: %.2fV, Percent: %d%% (raw=%d)\n", v, batteryPercent, raw);
  return batteryPercent;
}

/* ----------------------------
   JSON PUBLISH
---------------------------- */
void publishStepsPayload() {
  StaticJsonDocument<256> doc;

  doc["steps"] = stepCount;
  doc["goal"]  = stepGoal;
  doc["goalReachedToday"] = goalReachedToday;
  doc["battery"] = batteryPercent; // Added battery to JSON

  time_t now = time(nullptr);
  doc["timestamp"] = (uint32_t)now;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  Serial.print("[MQTT] Publishing: ");
  Serial.println(buffer);

  client.publish(TOPIC_STEPS, buffer, n);
  
  // Brief white blink on publish
  rgbBlink(true, true, true, 1, 50, 50);
}

/* ----------------------------
   COMMAND PARSING
---------------------------- */
String trimString(const String& s) {
  int start = 0;
  while (start < (int)s.length() && isspace((unsigned char)s[start])) start++;
  int end = s.length() - 1;
  while (end >= 0 && isspace((unsigned char)s[end])) end--;
  if (end < start) return "";
  return s.substring(start, end + 1);
}

void handleCommand(const String& cmdRaw) {
  Serial.printf("[CMD] Received: %s\n", cmdRaw.c_str());

  String cmd = trimString(cmdRaw);

  if (cmd.equalsIgnoreCase("reset")) {
    Serial.println("[CMD] RESET triggered");
    stepCount = 0;
    goalReachedToday = false;
    signalReset();
    return;
  }

  if (cmd.startsWith("goal:")) {
    int32_t g = cmd.substring(5).toInt();
    if (g > 0) {
      saveStepGoal(g);
    }
    return;
  }

  if (cmd.startsWith("sens:")) {
    float s = cmd.substring(5).toFloat();
    if (s > 0.0f && s <= 20.0f) {
      saveSensitivity(s);
    }
    return;
  }
  
  if (cmd.equalsIgnoreCase("readbattery")) {
    Serial.println("[CMD] Manual battery read requested");
    readBatteryPercent();
    return;
  }

  Serial.println("[CMD] Unknown command");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("[MQTT] Message on %s: %s\n", topic, msg.c_str());

  handleCommand(msg);
}

/* ----------------------------
   STEP DETECTION
---------------------------- */
bool detectStepFromAccel(float ax, float ay, float az, uint32_t nowMs) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);

  const float alpha = 0.90f;
  g_est = alpha * g_est + (1.0f - alpha) * mag;

  float hp = mag - g_est;

  bool crossingUp = (prev_hp <= sensitivity) && (hp > sensitivity);
  bool okSpike = (hp < 50.0f);
  bool okTiming = (nowMs - lastStepMs) >= minStepIntervalMs;

  prev_hp = hp;

  if (crossingUp && okSpike && okTiming) {
    lastStepMs = nowMs;
    return true;
  }
  return false;
}

/* ----------------------------
   GOAL HANDLING
---------------------------- */
void handleGoalLogic() {
  if (!goalReachedToday && stepCount >= (uint32_t)stepGoal) {
    Serial.println("[GOAL] Goal reached!");
    goalReachedToday = true;
    signalGoal();
  }
}

/* ----------------------------
   SETUP
---------------------------- */
void setup() {
  // Initialize RGB LED pins
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);
  rgbOff();
  
  // Initialize battery pin if enabled
  if (BATTERY_ENABLED) {
    pinMode(BAT_ADC_PIN, INPUT);
    analogReadResolution(12);
  }

  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== SMART STEPS ESP32 STARTUP ===");
  
  // Startup sequence - rainbow blink
  rgbBlink(true, false, false, 1, 200, 100); // Red
  rgbBlink(false, true, false, 1, 200, 100); // Green
  rgbBlink(false, false, true, 1, 200, 100); // Blue

  loadPersistentConfig();

  if (!initAccelerometer()) {
    Serial.println("[FATAL] Accelerometer missing");
    while (true) { 
      signalError(); 
      delay(500); 
    }
  }

  connectWiFi();
  initTimeNTP();
  connectMQTT();
  
  // Initial battery reading
  if (BATTERY_ENABLED) {
    readBatteryPercent();
  }

  currentDayKey = computeDayKey();
  Serial.printf("[DAY] Current day key = %d\n", currentDayKey);
}

/* ----------------------------
   LOOP
---------------------------- */
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection → reconnecting");
    signalAdvertising(); // Purple while reconnecting
    connectWiFi();
    signalConnectedIdle(); // Back to cyan
  }

  if (!client.connected()) {
    Serial.println("[MQTT] Lost connection → reconnecting");
    rgbBlink(false, true, false, 3, 100, 100); // Green blink during MQTT reconnect
    connectMQTT();
  }
  client.loop();

  handleNewDayIfNeeded();

  uint32_t nowMs = millis();
  uint32_t sampleInterval = goalReachedToday ? sampleIntervalMs_afterGoal : sampleIntervalMs_beforeGoal;

  if (nowMs - lastSampleMs >= sampleInterval) {
    lastSampleMs = nowMs;

    float ax, ay, az;
    if (readAccel(ax, ay, az)) {
      if (detectStepFromAccel(ax, ay, az, nowMs)) {
        stepCount++;
        signalStepDetected(); // Orange flash for step
        Serial.printf("[STEP] stepCount=%u\n", stepCount);
      }
    }

    handleGoalLogic();
  }

  uint32_t stepsPubInterval = goalReachedToday ? publishStepsInterval_afterGoal : publishStepsInterval_beforeGoal;

  if (nowMs - lastPublishStepsMs >= stepsPubInterval) {
    lastPublishStepsMs = nowMs;
    publishStepsPayload();
  }

  // Publish battery reading at intervals
  if (BATTERY_ENABLED && (nowMs - lastPublishBatteryMs >= publishBatteryIntervalMs)) {
    lastPublishBatteryMs = nowMs;
    readBatteryPercent(); // Update battery reading
    // Battery is included in the regular steps payload, no need for separate publish
  }
  
  // Keep idle LED state
  if (client.connected()) {
    signalConnectedIdle(); // Maintain cyan when idle and connected
  }
}