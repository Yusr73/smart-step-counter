/*
  ESP32 Smart Step Counter (BLE Version, full features)

  Replaces WiFi + MQTT with BLE GATT:

  SERVICE: SmartSteps
    UUID: 0000ABCD-0000-1000-8000-00805F9B34FB

  CHARACTERISTICS:
    - Steps JSON Notify (like MQTT payload)
      UUID: 0001ABCD-0000-1000-8000-00805F9B34FB
      Notify payload example:
        {"steps":1234,"goal":6000,"goalReachedToday":false,"timestamp":1700000000}

    - Goal Read/Write
      UUID: 0002ABCD-0000-1000-8000-00805F9B34FB

    - Sensitivity Read/Write
      UUID: 0003ABCD-0000-1000-8000-00805F9B34FB

    - Command Write
      UUID: 0004ABCD-0000-1000-8000-00805F9B34FB
      Commands:
        "reset"
        "goal:8000"
        "sens:1.35"
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <MMA7660.h>
#include <EEPROM.h>
#include <time.h>
#include <math.h>

/* ----------------------------
   LED / STATUS (optional)
---------------------------- */
const int LED_PIN = 2;

/* ----------------------------
   EEPROM LAYOUT
---------------------------- */
static const int EEPROM_SIZE = 64;
static const int ADDR_MAGIC       = 0;
static const int ADDR_STEP_GOAL   = 4;
static const int ADDR_SENSITIVITY = 8;
static const uint32_t EEPROM_MAGIC = 0x53544550; // "STEP"

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
// You can keep these as in MQTT version if you want modulation:
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
   BATTERY (OPTIONAL)
---------------------------- */
const int BAT_ADC_PIN = 34;
const bool BATTERY_ENABLED = false;
const float ADC_REF_V = 3.3f;
const float DIVIDER_RATIO = 2.0f;
const float BAT_V_MIN = 3.30f;
const float BAT_V_MAX = 4.20f;

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
   LED BLINK HELPERS
---------------------------- */
void blink(int times, int onMs=120, int offMs=120) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
}

void signalError() { blink(1, 600, 250); Serial.println("[LED] ERROR"); }
void signalReset() { blink(1, 150, 150); Serial.println("[LED] RESET"); }
void signalGoal()  { blink(5, 90, 90);   Serial.println("[LED] GOAL"); }

/* ----------------------------
   TIME (NTP-like idea)
   NOTE: Without WiFi, NTP won't work.
   Here we show the structure, but in BLE-only mode
   you either:
     - skip time-based daily reset, or
     - set time over BLE from the app.
---------------------------- */

// For now, we keep the structure but without real NTP.
// You can later add BLE-based time sync if needed.
bool timeIsValid() {
  // If you don't have real time, you can disable date logic
  return false;
}

int computeDayKey() {
  if (!timeIsValid()) return -1;
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  return (t.tm_year + 1900) * 1000 + t.tm_yday;
}

void handleNewDayIfNeeded() {
  if (!timeIsValid()) return;
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
}

void saveSensitivity(float newSens) {
  sensitivity = newSens;
  eepromWrite(ADDR_SENSITIVITY, sensitivity);
  EEPROM.commit();
  Serial.printf("[EEPROM] Sensitivity updated → %.2f\n", sensitivity);
}

/* ----------------------------
   STEP DETECTION LOGIC
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
    Serial.printf("[STEP] Detected | hp=%.2f | ax=%.2f ay=%.2f az=%.2f\n",
                  hp, ax, ay, az);
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
   BLE OBJECTS
---------------------------- */
BLECharacteristic* stepsJsonChar = nullptr;
BLECharacteristic* goalChar      = nullptr;
BLECharacteristic* sensChar      = nullptr;
BLECharacteristic* cmdChar       = nullptr;

bool bleClientConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    Serial.println("[BLE] Client disconnected");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();

    if (value.isEmpty()) return;

    String cmd = String(value.c_str());
    Serial.printf("[CMD] Received: %s\n", cmd.c_str());

    cmd.trim();
    if (cmd.equalsIgnoreCase("reset")) {
      Serial.println("[CMD] RESET triggered");
      stepCount = 0;
      goalReachedToday = false;
      signalReset();
      return;
    }

    if (cmd.startsWith("goal:")) {
      int32_t g = cmd.substring(5).toInt();
      saveStepGoal(g);
      if (goalChar) {
        goalChar->setValue(String(stepGoal).c_str());
      }
      return;
    }

    if (cmd.startsWith("sens:")) {
      float s = cmd.substring(5).toFloat();
      saveSensitivity(s);
      if (sensChar) {
        sensChar->setValue(String(sensitivity, 2).c_str());
      }
      return;
    }

    Serial.println("[CMD] Unknown command");
  }
};

/* ----------------------------
   BLE INIT
---------------------------- */
void initBLE() {
  Serial.println("[BLE] Initializing...");

  BLEDevice::init("SmartSteps BLE");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService("0000ABCD-0000-1000-8000-00805F9B34FB");

  stepsJsonChar = service->createCharacteristic(
    "0001ABCD-0000-1000-8000-00805F9B34FB",
    BLECharacteristic::PROPERTY_NOTIFY
  );

  goalChar = service->createCharacteristic(
    "0002ABCD-0000-1000-8000-00805F9B34FB",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  sensChar = service->createCharacteristic(
    "0003ABCD-0000-1000-8000-00805F9B34FB",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  cmdChar = service->createCharacteristic(
    "0004ABCD-0000-1000-8000-00805F9B34FB",
    BLECharacteristic::PROPERTY_WRITE
  );

  cmdChar->setCallbacks(new CommandCallbacks());

  // Initial values for read characteristics
  goalChar->setValue(String(stepGoal).c_str());
  sensChar->setValue(String(sensitivity, 2).c_str());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID("0000ABCD-0000-1000-8000-00805F9B34FB");
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started");
}

/* ----------------------------
   BLE "publish" (JSON notify)
---------------------------- */
void bleNotifyStepsJson() {
  if (!bleClientConnected || !stepsJsonChar) return;

  // We no longer have valid time without WiFi; timestamp can be 0 or millis.
  uint32_t ts = 0; // or millis() if you want relative time

  String json = "{";
  json += "\"steps\":" + String(stepCount) + ",";
  json += "\"goal\":" + String(stepGoal) + ",";
  json += "\"goalReachedToday\":" + String(goalReachedToday ? "true" : "false") + ",";
  json += "\"timestamp\":" + String(ts);
  json += "}";

  Serial.print("[BLE] Notify: ");
  Serial.println(json);

  stepsJsonChar->setValue(json.c_str());
  stepsJsonChar->notify();
}

/* ----------------------------
   BATTERY PUBLISH (optional)
   For now, we can reuse same JSON or add another char if needed.
---------------------------- */
void publishBatteryIfNeeded(uint32_t nowMs) {
  if (!BATTERY_ENABLED) return;

  if (nowMs - lastPublishBatteryMs >= publishBatteryIntervalMs) {
    lastPublishBatteryMs = nowMs;

    // You can measure battery here and include it in JSON (extend JSON or another char)
    // For now, we won't implement full battery to keep it aligned.
  }
}

/* ----------------------------
   SETUP
---------------------------- */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== SMART STEPS ESP32 (BLE) STARTUP ===");

  loadPersistentConfig();

  if (!initAccelerometer()) {
    Serial.println("[FATAL] Accelerometer missing");
    while (true) {
      signalError();
      delay(500);
    }
  }

  // Time / NTP disabled for pure BLE version; you can add BLE time sync later
  currentDayKey = computeDayKey();
  if (currentDayKey != -1) {
    Serial.printf("[DAY] Current day key = %d\n", currentDayKey);
  } else {
    Serial.println("[DAY] Time not valid, day logic disabled");
  }

  initBLE();
}

/* ----------------------------
   LOOP
---------------------------- */
void loop() {
  handleNewDayIfNeeded();

  uint32_t nowMs = millis();
  uint32_t sampleInterval = goalReachedToday ? sampleIntervalMs_afterGoal : sampleIntervalMs_beforeGoal;

  if (nowMs - lastSampleMs >= sampleInterval) {
    lastSampleMs = nowMs;

    float ax, ay, az;
    if (readAccel(ax, ay, az)) {
      if (detectStepFromAccel(ax, ay, az, nowMs)) {
        stepCount++;
        Serial.printf("[STEP] stepCount=%u\n", stepCount);
        handleGoalLogic();
      }
    }
  }

  uint32_t stepsPubInterval = goalReachedToday ? publishStepsInterval_afterGoal : publishStepsInterval_beforeGoal;

  if (nowMs - lastPublishStepsMs >= stepsPubInterval) {
    lastPublishStepsMs = nowMs;
    bleNotifyStepsJson();
  }

  publishBatteryIfNeeded(nowMs);

  // BLE library handles events internally, no client.loop() needed
  delay(5);
}
