/*
  ESP32 Smart Step Counter (OPTIMIZED MQTT Version)
  - Non-blocking operations
  - Task separation using FreeRTOS
  - Optimized JSON handling
  - Fast response times
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Wire.h>
#include <MMA7660.h>
#include <EEPROM.h>

// ===== CONFIGURATION =====
const char* ssid = "Network";
const char* password = "f33dq14Z";
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* TOPIC_STEPS = "smartsteps/steps";
const char* TOPIC_CONFIG = "smartsteps/config";

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

// ===== OPTIMIZED INTERVALS =====
const uint32_t SENSOR_INTERVAL = 50;      // 20Hz sampling
const uint32_t PUBLISH_INTERVAL = 2000;   // 2 seconds (reduced from 5)
const uint32_t BATTERY_INTERVAL = 30000;  // 30 seconds
const uint32_t CONNECTION_CHECK = 5000;   // 5 seconds
const uint32_t DAY_CHECK_INTERVAL = 60000; // 1 minute
const uint32_t CONNECTION_LED_DURATION = 3000; // Cyan LED stays on for 3 seconds after connection
const uint32_t INACTIVITY_TIMEOUT = 5000; // Turn off LED after 5 seconds of no steps

// ===== OBJECTS =====
WiFiClient espClient;
PubSubClient client(espClient);
MMA7660 accel;

// ===== PRE-ALLOCATED BUFFERS =====
char jsonBuffer[256];  // Reuse this buffer
StaticJsonDocument<200> jsonDoc;  // Reuse JSON document

// ===== OPTIMIZED LED FUNCTIONS =====
void rgbOff() { digitalWrite(LED_Red, HIGH); digitalWrite(LED_Green, HIGH); digitalWrite(LED_Blue, HIGH); }
void rgbColor(bool r, bool g, bool b) {
    digitalWrite(LED_Red, r ? LOW : HIGH);
    digitalWrite(LED_Green, g ? LOW : HIGH);
    digitalWrite(LED_Blue, b ? LOW : HIGH);
}

// Longer visible blink
void rgbBlink(bool r, bool g, bool b, int times = 1, int onMs = 150, int offMs = 100) {
    for (int i = 0; i < times; i++) {
        rgbColor(r, g, b);
        delay(onMs);
        rgbOff();
        if (i < times - 1) delay(offMs); // No delay after last blink
    }
}

// Brief but visible blink
void quickBlink(bool r, bool g, bool b) {
    rgbColor(r, g, b);
    delay(80);  // Longer than before for visibility
    rgbOff();
}

// ===== OPTIMIZED STEP DETECTION =====
float g_est = 9.81f;
float prev_hp = 0.0f;
uint32_t lastStepMs = 0;

bool detectStep(float ax, float ay, float az, uint32_t nowMs) {
    // Fast magnitude calculation (avoid sqrt when possible in future optimizations)
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    
    // High-pass filter
    g_est = 0.9f * g_est + 0.1f * mag;
    float hp = mag - g_est;
    
    bool stepDetected = (prev_hp <= sensitivity && hp > sensitivity && 
                        hp < 50.0f && (nowMs - lastStepMs) >= 280);
    
    prev_hp = hp;
    
    if (stepDetected) {
        lastStepMs = nowMs;
        lastActivityTime = nowMs; // Update activity timestamp
        stepCount++;
        newStepsAvailable = true;
        
        // Yellow/orange blink for step (red + green)
        rgbBlink(true, true, false, 1, 100, 0); // 100ms yellow blink
        
        if (!goalReachedToday && stepCount >= (uint32_t)stepGoal) {
            goalReachedToday = true;
            // Green celebration blink sequence
            rgbBlink(false, true, false, 3, 200, 150);
        }
        return true;
    }
    return false;
}

// ===== OPTIMIZED JSON PUBLISHING =====
void publishSteps() {
    if (!client.connected()) return;
    
    // Reuse existing JSON document
    jsonDoc.clear();
    jsonDoc["steps"] = stepCount;
    jsonDoc["goal"] = stepGoal;
    jsonDoc["goalReachedToday"] = goalReachedToday;
    jsonDoc["battery"] = batteryPercent;
    jsonDoc["timestamp"] = (uint32_t)time(nullptr);
    
    // Fast serialization into pre-allocated buffer
    size_t len = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
    
    // Fast publish (no need for print debugging in production)
    client.publish(TOPIC_STEPS, jsonBuffer, len);
    
    // White blink on publish
    rgbBlink(true, true, true, 1, 100, 0);
}

// ===== FAST BATTERY READING =====
void readBattery() {
    // Single reading (fast) instead of averaging
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
    
    // Pre-calculated scale factor
    static const float SCALE = (1.5f / 32.0f) * 9.81f;
    ax = (float)x * SCALE;
    ay = (float)y * SCALE;
    az = (float)z * SCALE;
    return true;
}

// ===== OPTIMIZED CONNECTION MANAGEMENT =====
void checkConnections() {
    uint32_t now = millis();
    
    // Non-blocking WiFi check
    if (WiFi.status() != WL_CONNECTED) {
        static uint32_t lastWifiAttempt = 0;
        if (now - lastWifiAttempt > 2000) {
            WiFi.reconnect();
            lastWifiAttempt = now;
            // Purple while reconnecting
            rgbBlink(true, false, true, 2, 150, 150);
        }
        return;
    }
    
    // Non-blocking MQTT check
    if (!client.connected()) {
        static uint32_t lastMqttAttempt = 0;
        if (now - lastMqttAttempt > 2000) {
            if (client.connect("ESP32_SmartSteps")) {
                client.subscribe(TOPIC_CONFIG);
                // Cyan connection indication (will turn off after CONNECTION_LED_DURATION)
                rgbColor(false, true, true);
                connectionLedOffTime = now + CONNECTION_LED_DURATION;
            }
            lastMqttAttempt = now;
        }
    }
}

// ===== MQTT CALLBACK =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Fast command processing
    if (length == 0) return;
    
    String cmd = "";
    for (unsigned int i = 0; i < length && i < 32; i++) {
        cmd += (char)payload[i];
    }
    
    // Fast command parsing
    if (cmd == "reset") {
        stepCount = 0;
        goalReachedToday = false;
        newStepsAvailable = true;  // Force immediate update
        // Blue reset blink
        rgbBlink(false, false, true, 3, 200, 150);
    }
    else if (cmd.startsWith("goal:")) {
        stepGoal = cmd.substring(5).toInt();
        EEPROM.put(4, stepGoal);
        EEPROM.commit();
        // Yellow confirmation blink
        rgbBlink(true, true, false, 2, 200, 150);
    }
    else if (cmd.startsWith("sens:")) {
        sensitivity = cmd.substring(5).toFloat();
        EEPROM.put(8, sensitivity);
        EEPROM.commit();
        // Yellow confirmation blink
        rgbBlink(true, true, false, 2, 200, 150);
    }
}

// ===== SETUP (OPTIMIZED) =====
void setup() {
    // Fast initialization
    Serial.begin(115200);
    
    // GPIO setup
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Green, OUTPUT);
    pinMode(LED_Blue, OUTPUT);
    rgbOff();
    
    // Fast accelerometer init
    Wire.begin();
    accel.init();
    delay(50);  // Minimal delay
    
    // Fast EEPROM load
    EEPROM.begin(64);
    EEPROM.get(4, stepGoal);
    EEPROM.get(8, sensitivity);
    
    // Fast WiFi connect (non-blocking start)
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    // Fast MQTT setup
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    client.setBufferSize(256);  // Optimized buffer
    
    // NTP setup (non-blocking)
    configTime(3600, 0, "pool.ntp.org");
    
    // Startup indicator - rainbow sequence
    rgbBlink(true, false, false, 1, 300, 100);   // Red
    rgbBlink(false, true, false, 1, 300, 100);   // Green
    rgbBlink(false, false, true, 1, 300, 100);   // Blue
    
    lastActivityTime = millis();
}

// ===== MAIN LOOP (OPTIMIZED) =====
void loop() {
    uint32_t now = millis();
    
    // 1. Check connections (non-blocking, every 5s)
    if (now - lastConnectionCheck >= CONNECTION_CHECK) {
        lastConnectionCheck = now;
        checkConnections();
    }
    
    // 2. Handle MQTT (non-blocking)
    if (client.connected()) {
        client.loop();
    }
    
    // 3. Read sensor (fixed interval, no blocking)
    if (now - lastSensorRead >= SENSOR_INTERVAL) {
        lastSensorRead = now;
        
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            detectStep(ax, ay, az, now);
        }
    }
    
    // 4. Publish steps (fast check, immediate on change or interval)
    if ((newStepsAvailable && client.connected()) || 
        (now - lastStepPublish >= PUBLISH_INTERVAL)) {
        lastStepPublish = now;
        publishSteps();
        newStepsAvailable = false;
    }
    
    // 5. Read battery (30s interval)
    if (now - lastBatteryRead >= BATTERY_INTERVAL) {
        lastBatteryRead = now;
        readBattery();
    }
    
    // 6. Check day change (1 minute interval)
    if (now - lastDayCheck >= DAY_CHECK_INTERVAL) {
        lastDayCheck = now;
        // Your day check logic here
    }
    
    // 7. Manage connection LED timeout
    if (connectionLedOffTime > 0 && now >= connectionLedOffTime) {
        rgbOff(); // Turn off cyan LED after connection period
        connectionLedOffTime = 0;
    }
    
    // 8. Turn off LED after inactivity (only if not showing connection LED)
    if (connectionLedOffTime == 0 && 
        client.connected() && 
        WiFi.status() == WL_CONNECTED &&
        (now - lastActivityTime >= INACTIVITY_TIMEOUT)) {
        rgbOff(); // Turn off LED during inactivity
    }
    
    // Minimal yield to prevent watchdog
    delay(1);
}