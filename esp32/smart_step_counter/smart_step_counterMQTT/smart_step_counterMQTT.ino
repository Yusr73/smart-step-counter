/*
  ESP32 Smart Step Counter (OPTIMIZED MQTT Version)
  + ACCEL STREAM PUBLISH (for VM inference)

  What’s new:
  - Publishes raw accelerometer samples at 20Hz to:
      smartsteps/<DEVICE_ID>/accel
    with JSON payload: {"ax":..,"ay":..,"az":..,"t":..}

  Your VM inference service subscribes to smartsteps/+/accel
  and publishes predictions to smartsteps/<DEVICE_ID>/pred
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Wire.h>
#include <MMA7660.h>
#include <EEPROM.h>

// ===== CONFIGURATION =====
const char* ssid = "Haykel4AI";
const char* password = "Galaxy#Planet*";

const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

// Device ID (important for per-device topics)
const char* DEVICE_ID = "esp32_01";

// Existing topics
const char* TOPIC_STEPS  = "smartsteps/steps";
const char* TOPIC_CONFIG = "smartsteps/config";

// New topic for raw accel stream (for VM model)
const char* TOPIC_ACCEL  = "smartsteps/esp32_01/accel"; // keep hard-coded to match DEVICE_ID above
// If you want it truly dynamic, see note at the bottom.

// ===== HARDWARE PINS =====
#define LED_Red   16
#define LED_Green 13
#define LED_Blue  12
#define BAT_ADC_PIN 34

// ===== GLOBAL STATE =====
volatile uint32_t stepCount = 0;
volatile int32_t stepGoal = 6000;
volatile float sensitivity = 1.25f;
volatile bool goalReachedToday = false;
volatile bool newStepsAvailable = false;
volatile int batteryPercent = 100;

// ===== TASK TIMING =====
uint32_t lastStepPublish = 0;
uint32_t lastSensorRead = 0;
uint32_t lastBatteryRead = 0;
uint32_t lastConnectionCheck = 0;
uint32_t lastDayCheck = 0;
uint32_t lastActivityTime = 0;
uint32_t connectionLedOffTime = 0;

// NEW: accel publish timing
uint32_t lastAccelPublish = 0;

// ===== OPTIMIZED INTERVALS =====
const uint32_t SENSOR_INTERVAL = 50;      // 20Hz sampling
const uint32_t PUBLISH_INTERVAL = 2000;   // steps publish every 2 seconds or on change
const uint32_t BATTERY_INTERVAL = 30000;  // 30 seconds
const uint32_t CONNECTION_CHECK = 5000;   // 5 seconds
const uint32_t DAY_CHECK_INTERVAL = 60000; // 1 minute
const uint32_t CONNECTION_LED_DURATION = 3000; // Cyan LED stays on for 3 seconds after connection
const uint32_t INACTIVITY_TIMEOUT = 5000; // Turn off LED after 5 seconds of no steps

// NEW: accel stream publish rate (20Hz)
const uint32_t ACCEL_PUBLISH_INTERVAL = 50;

// ===== OBJECTS =====
WiFiClient espClient;
PubSubClient client(espClient);
MMA7660 accel;

// ===== PRE-ALLOCATED BUFFERS =====
char jsonBuffer[256];              // reuse buffer
StaticJsonDocument<256> jsonDoc;   // slightly larger (steps payload + accel payload)

// ===== OPTIMIZED LED FUNCTIONS =====
void rgbOff() { digitalWrite(LED_Red, HIGH); digitalWrite(LED_Green, HIGH); digitalWrite(LED_Blue, HIGH); }
void rgbColor(bool r, bool g, bool b) {
  digitalWrite(LED_Red, r ? LOW : HIGH);
  digitalWrite(LED_Green, g ? LOW : HIGH);
  digitalWrite(LED_Blue, b ? LOW : HIGH);
}
void rgbBlink(bool r, bool g, bool b, int times = 1, int onMs = 150, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    rgbColor(r, g, b);
    delay(onMs);
    rgbOff();
    if (i < times - 1) delay(offMs);
  }
}
void quickBlink(bool r, bool g, bool b) {
  rgbColor(r, g, b);
  delay(80);
  rgbOff();
}

// ===== OPTIMIZED STEP DETECTION =====
float g_est = 9.81f;
float prev_hp = 0.0f;
uint32_t lastStepMs = 0;

bool detectStep(float ax, float ay, float az, uint32_t nowMs) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);

  // High-pass filter
  g_est = 0.9f * g_est + 0.1f * mag;
  float hp = mag - g_est;

  bool stepDetected = (prev_hp <= sensitivity && hp > sensitivity &&
                       hp < 50.0f && (nowMs - lastStepMs) >= 280);

  prev_hp = hp;

  if (stepDetected) {
    lastStepMs = nowMs;
    lastActivityTime = nowMs;
    stepCount++;
    newStepsAvailable = true;

    // Yellow blink for step
    rgbBlink(true, true, false, 1, 100, 0);

    if (!goalReachedToday && stepCount >= (uint32_t)stepGoal) {
      goalReachedToday = true;
      // Green celebration
      rgbBlink(false, true, false, 3, 200, 150);
    }
    return true;
  }
  return false;
}

// ===== OPTIMIZED JSON PUBLISHING =====
void publishSteps() {
  if (!client.connected()) return;

  jsonDoc.clear();
  jsonDoc["steps"] = stepCount;
  jsonDoc["goal"] = stepGoal;
  jsonDoc["goalReachedToday"] = goalReachedToday;
  jsonDoc["battery"] = batteryPercent;
  jsonDoc["timestamp"] = (uint32_t)time(nullptr);

  size_t len = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
  client.publish(TOPIC_STEPS, jsonBuffer, len);

  // White blink on steps publish
  rgbBlink(true, true, true, 1, 100, 0);
}

// NEW: publish raw accel samples for VM inference
void publishAccel(float ax, float ay, float az, uint32_t nowMs) {
  if (!client.connected()) return;

  jsonDoc.clear();
  jsonDoc["ax"] = ax;
  jsonDoc["ay"] = ay;
  jsonDoc["az"] = az;
  jsonDoc["t"]  = nowMs; // millis timestamp

  size_t len = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
  client.publish(TOPIC_ACCEL, jsonBuffer, len);

  // No LED blink here (too frequent)
}

// ===== FAST BATTERY READING =====
void readBattery() {
  int raw = analogRead(BAT_ADC_PIN);
  float v = ((float)raw / 4095.0f) * 3.3f * 2.0f;
  v = constrain(v, 3.30f, 4.20f);
  batteryPercent = (int)((v - 3.30f) / (4.20f - 3.30f) * 100.0f);
  batteryPercent = constrain(batteryPercent, 0, 100);
}

// ===== FAST ACCELEROMETER READING =====
bool readAccel(float &ax, float &ay, float &az) {
  int8_t x, y, z;
  if (!accel.getXYZ(&x, &y, &z)) return false;

  // scale to m/s^2 (as your code already does)
  static const float SCALE = (1.5f / 32.0f) * 9.81f;
  ax = (float)x * SCALE;
  ay = (float)y * SCALE;
  az = (float)z * SCALE;
  return true;
}

// ===== OPTIMIZED CONNECTION MANAGEMENT =====
void checkConnections() {
  uint32_t now = millis();

  // WiFi check
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiAttempt = 0;
    if (now - lastWifiAttempt > 2000) {
      WiFi.reconnect();
      lastWifiAttempt = now;
      rgbBlink(true, false, true, 2, 150, 150); // Purple
    }
    return;
  }

  // MQTT check
  if (!client.connected()) {
    static uint32_t lastMqttAttempt = 0;
    if (now - lastMqttAttempt > 2000) {
      // Unique client id (avoid collisions on public broker)
      String cid = String("ESP32_SmartSteps_") + DEVICE_ID;

      if (client.connect(cid.c_str())) {
        client.subscribe(TOPIC_CONFIG);
        rgbColor(false, true, true); // Cyan
        connectionLedOffTime = now + CONNECTION_LED_DURATION;
      }
      lastMqttAttempt = now;
    }
  }
}

// ===== MQTT CALLBACK =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;

  String cmd = "";
  for (unsigned int i = 0; i < length && i < 32; i++) {
    cmd += (char)payload[i];
  }

  if (cmd == "reset") {
    stepCount = 0;
    goalReachedToday = false;
    newStepsAvailable = true;
    rgbBlink(false, false, true, 3, 200, 150); // Blue
  }
  else if (cmd.startsWith("goal:")) {
    stepGoal = cmd.substring(5).toInt();
    EEPROM.put(4, stepGoal);
    EEPROM.commit();
    rgbBlink(true, true, false, 2, 200, 150); // Yellow
  }
  else if (cmd.startsWith("sens:")) {
    sensitivity = cmd.substring(5).toFloat();
    EEPROM.put(8, sensitivity);
    EEPROM.commit();
    rgbBlink(true, true, false, 2, 200, 150); // Yellow
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);
  rgbOff();

  Wire.begin();
  accel.init();
  delay(50);

  EEPROM.begin(64);
  EEPROM.get(4, stepGoal);
  EEPROM.get(8, sensitivity);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  espClient.setNoDelay(true); 

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(256);

  configTime(3600, 0, "pool.ntp.org");

  // Startup indicator
  rgbBlink(true, false, false, 1, 300, 100);
  rgbBlink(false, true, false, 1, 300, 100);
  rgbBlink(false, false, true, 1, 300, 100);

  lastActivityTime = millis();
}

// ===== MAIN LOOP =====
void loop() {
  uint32_t now = millis();

  // 1) Check connections
  if (now - lastConnectionCheck >= CONNECTION_CHECK) {
    lastConnectionCheck = now;
    checkConnections();
  }

  // 2) Handle MQTT
  if (client.connected()) {
    client.loop();
  }

  // 3) Read sensor at 20Hz
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    float ax, ay, az;
    if (readAccel(ax, ay, az)) {

      // NEW: publish accel stream at 20Hz (for VM inference)
      if (client.connected() && (now - lastAccelPublish >= ACCEL_PUBLISH_INTERVAL)) {
        lastAccelPublish = now;
        publishAccel(ax, ay, az, now);
      }

      // Existing step detection
      detectStep(ax, ay, az, now);
    }
  }

  // 4) Publish steps (on change or interval)
  if ((newStepsAvailable && client.connected()) ||
      (now - lastStepPublish >= PUBLISH_INTERVAL)) {
    lastStepPublish = now;
    publishSteps();
    newStepsAvailable = false;
  }

  // 5) Battery every 30s
  if (now - lastBatteryRead >= BATTERY_INTERVAL) {
    lastBatteryRead = now;
    readBattery();
  }

  // 6) Day check placeholder
  if (now - lastDayCheck >= DAY_CHECK_INTERVAL) {
    lastDayCheck = now;
    // day change logic here
  }

  // 7) Connection LED timeout
  if (connectionLedOffTime > 0 && now >= connectionLedOffTime) {
    rgbOff();
    connectionLedOffTime = 0;
  }

  // 8) Turn off LED after inactivity
  if (connectionLedOffTime == 0 &&
      client.connected() &&
      WiFi.status() == WL_CONNECTED &&
      (now - lastActivityTime >= INACTIVITY_TIMEOUT)) {
    rgbOff();
  }

  delay(1);
}

/*
  NOTE (optional improvement):
  If you want TOPIC_ACCEL to be built dynamically from DEVICE_ID, do this:

    String accelTopic = String("smartsteps/") + DEVICE_ID + "/accel";
    // then publish to accelTopic.c_str()

  But keeping it as a const char* is fastest + simplest.
*/
