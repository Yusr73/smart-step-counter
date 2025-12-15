/*
  ESP32 Smart Step Counter (BLE Version, full features + RGB Status)
  Rewritten with Serial monitor fixes
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 

#include <Wire.h>
#include <MMA7660.h>
#include <EEPROM.h>
#include <time.h>
#include <math.h>

/* ----------------------------
   SERIAL DEBUG MACROS (MOVED TO TOP)
---------------------------- */
// Use these instead of Serial.print() to ensure output
#define SERIAL_BEGIN() do { Serial.begin(115200); delay(100); } while(0)
#define SERIAL_PRINT(x) do { Serial.print(x); Serial.flush(); } while(0)
#define SERIAL_PRINTLN(x) do { Serial.println(x); Serial.flush(); } while(0)
#define SERIAL_PRINTF(...) do { Serial.printf(__VA_ARGS__); Serial.flush(); } while(0)

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
static const uint32_t EEPROM_MAGIC = 0x53544550; // "STEP"

/* ----------------------------
   GLOBAL STATE
---------------------------- */
volatile uint32_t stepCount = 0;
int32_t stepGoal = 6000;
float sensitivity = 1.25f;

bool goalReachedToday = false;
bool bleDeviceConnected = false;  // MOVED HERE: Declared before detectStep() uses it

/* ----------------------------
   POWER / SAMPLING CONTROL
---------------------------- */
uint32_t sampleIntervalMs = 50;

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
   BATTERY 
---------------------------- */
const int BAT_ADC_PIN = 34;
const bool BATTERY_ENABLED = true;  // Changed to true to enable battery
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
  SERIAL_PRINTLN("[ACCEL] Initializing MMA7660...");
  Wire.begin();
  
  // Try I2C scan first
  SERIAL_PRINTLN("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      SERIAL_PRINTF("I2C device at 0x%02X\n", address);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    SERIAL_PRINTLN("[ACCEL] No I2C devices found!");
    return false;
  }
  
  // MMA7660 init doesn't return a value, so we just call it
  accel.init();
  delay(100); // Give time to initialize
  
  int8_t x, y, z;
  if (accel.getXYZ(&x, &y, &z)) {
    SERIAL_PRINTF("[ACCEL] Test read: x=%d, y=%d, z=%d\n", x, y, z);
  } else {
    SERIAL_PRINTLN("[ACCEL] Failed to read accelerometer!");
    return false;
  }
  
  accel.setSampleRate(64);
  SERIAL_PRINTLN("[ACCEL] MMA7660 initialized OK.");
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  int8_t x, y, z;
  if (!accel.getXYZ(&x, &y, &z)) {
    return false;
  }

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
   STATUS SIGNALS
---------------------------- */
void signalError() {
  SERIAL_PRINTLN("[SIGNAL] Error (Red blink)");
  rgbBlink(true, false, false, 1, 600, 250);
}

void signalReset() {
  SERIAL_PRINTLN("[SIGNAL] Reset (Blue blink)");
  rgbBlink(false, false, true, 1, 150, 150);
}

void signalGoal() {
  SERIAL_PRINTLN("[SIGNAL] Goal reached (Green blink)");
  rgbBlink(false, true, false, 5, 90, 90);
}

void signalAdvertising() {
  SERIAL_PRINTLN("[SIGNAL] Advertising (Purple)");
  rgbColor(true, false, true);
}

void signalConnected() {
  SERIAL_PRINTLN("[SIGNAL] Connected (Cyan)");
  rgbColor(false, true, true);
}

void signalStepDetected() {
  // Don't print every step to avoid flooding Serial
  rgbBlink(true, true, false, 1, 40, 40);
}

/* ----------------------------
   EEPROM HELPERS
---------------------------- */
void loadConfigFromEEPROM() {
  SERIAL_PRINTLN("[EEPROM] Loading config...");
  EEPROM.begin(EEPROM_SIZE);
  uint32_t storedMagic;
  EEPROM.get(ADDR_MAGIC, storedMagic);
  if (storedMagic != EEPROM_MAGIC) {
    SERIAL_PRINTLN("[EEPROM] No valid config, using defaults");
    stepGoal = 6000;
    sensitivity = 1.25f;
    EEPROM.put(ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.put(ADDR_STEP_GOAL, stepGoal);
    EEPROM.put(ADDR_SENSITIVITY, sensitivity);
    EEPROM.commit();
  } else {
    EEPROM.get(ADDR_STEP_GOAL, stepGoal);
    EEPROM.get(ADDR_SENSITIVITY, sensitivity);
    SERIAL_PRINTF("[EEPROM] Loaded: Goal=%ld, Sensitivity=%.2f\n", stepGoal, sensitivity);
  }
  EEPROM.end();
}

void saveGoalToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(ADDR_STEP_GOAL, stepGoal);
  EEPROM.commit();
  EEPROM.end();
  SERIAL_PRINTF("[EEPROM] Saved goal: %ld\n", stepGoal);
}

void saveSensitivityToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(ADDR_SENSITIVITY, sensitivity);
  EEPROM.commit();
  EEPROM.end();
  SERIAL_PRINTF("[EEPROM] Saved sensitivity: %.2f\n", sensitivity);
}

/* ----------------------------
   STEP DETECTION LOGIC - MODIFIED
---------------------------- */
bool detectStep(float ax, float ay, float az, uint32_t nowMs) {
  const float mag = sqrtf(ax * ax + ay * ay + az * az);

  const float alpha = 0.9f;
  g_est = alpha * g_est + (1.0f - alpha) * mag;
  const float hp = mag - g_est;

  bool isStep = false;

  if (prev_hp <= sensitivity && hp > sensitivity) {
    if (hp < 50.0f) {
      if (nowMs - lastStepMs >= minStepIntervalMs) {
        isStep = true;
        lastStepMs = nowMs;
        stepCount++;
        
        // Print only every 10 steps to avoid flooding Serial
        if (stepCount % 10 == 0) {
          SERIAL_PRINTF("[STEP] Steps: %lu (HP=%.2f)\n", stepCount, hp);
        }
        
        signalStepDetected();

        // ✅ CRITICAL CHANGE: Publish JSON immediately on step detection BEFORE goal reached
        if (!goalReachedToday && bleDeviceConnected) {
          publishStepsJson(nowMs);
          SERIAL_PRINTF("[STEP] Immediate publish after step: %lu\n", stepCount);
        }

        if (!goalReachedToday && stepCount >= (uint32_t)stepGoal) {
          goalReachedToday = true;
          SERIAL_PRINTLN("[STEP] GOAL REACHED!");
          signalGoal();
          // Also publish immediately when goal is reached
          if (bleDeviceConnected) {
            publishStepsJson(nowMs);
          }
        }
      }
    }
  }

  prev_hp = hp;
  return isStep;
}

/* ----------------------------
   BATTERY READING
---------------------------- */
int readBatteryPercent() {
  if (!BATTERY_ENABLED) return -1;
  
  // Take average of 5 readings
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
  
  SERIAL_PRINTF("[BAT] Voltage: %.2fV, Percent: %d%% (raw=%d)\n", v, batteryPercent, raw);
  return batteryPercent;
}

/* ----------------------------
   BLE GATT DEFINITIONS
---------------------------- */
#define SMARTSTEPS_SERVICE_UUID        "0000abcd-0000-1000-8000-00805f9b34fb"
#define STEPS_JSON_CHAR_UUID           "0001abcd-0000-1000-8000-00805f9b34fb"
#define GOAL_RW_CHAR_UUID              "0002abcd-0000-1000-8000-00805f9b34fb"
#define SENSITIVITY_RW_CHAR_UUID       "0003abcd-0000-1000-8000-00805f9b34fb"
#define COMMAND_W_CHAR_UUID            "0004abcd-0000-1000-8000-00805f9b34fb"
#define BATTERY_CHAR_UUID              "0005abcd-0000-1000-8000-00805f9b34fb"

BLEServer *pServer = nullptr;
BLECharacteristic *pStepsJsonChar = nullptr;
BLECharacteristic *pGoalChar = nullptr;
BLECharacteristic *pSensChar = nullptr;
BLECharacteristic *pCmdChar = nullptr;
BLECharacteristic *pBatteryChar = nullptr;

/* ----------------------------
   BLE CALLBACKS
---------------------------- */
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleDeviceConnected = true;
    SERIAL_PRINTLN("[BLE] Device Connected!");
    signalConnected();
  }
  
  void onDisconnect(BLEServer *pServer) override {
    bleDeviceConnected = false;
    SERIAL_PRINTLN("[BLE] Device Disconnected, restarting advertising");
    pServer->getAdvertising()->start();
    signalAdvertising();
  }
};

class GoalCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    SERIAL_PRINTF("[BLE] Goal write: %s\n", value.c_str());
    
    for (unsigned int i = 0; i < value.length(); i++) {
      if (value[i] < '0' || value[i] > '9') return;
    }

    int newGoal = value.toInt();
    if (newGoal <= 0) return;

    stepGoal = newGoal;
    saveGoalToEEPROM();

    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)stepGoal);
    pGoalChar->setValue(buf);
  }
};

class SensCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    SERIAL_PRINTF("[BLE] Sensitivity write: %s\n", value.c_str());
    
    float f = value.toFloat();
    if (f <= 0.0f || f > 20.0f) return;

    sensitivity = f;
    saveSensitivityToEEPROM();

    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", sensitivity);
    pSensChar->setValue(buf);
  }
};

class CommandCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String cmd = pCharacteristic->getValue();
    if (cmd.length() == 0) return;

    SERIAL_PRINTF("[BLE] Command received: %s\n", cmd.c_str());
    
    if (cmd == "reset") {
      stepCount = 0;
      goalReachedToday = false;
      SERIAL_PRINTLN("[CMD] Steps reset to 0");
      signalReset();
    }
    else if (cmd.startsWith("goal:")) {
      int newGoal = cmd.substring(5).toInt();
      if (newGoal > 0) {
        stepGoal = newGoal;
        saveGoalToEEPROM();
        char buf[16];
        snprintf(buf, sizeof(buf), "%ld", (long)stepGoal);
        pGoalChar->setValue(buf);
        SERIAL_PRINTF("[CMD] Goal changed to: %ld\n", stepGoal);
      }
    }
    else if (cmd.startsWith("sens:")) {
      float s = cmd.substring(5).toFloat();
      if (s > 0.0f && s <= 20.0f) {
        sensitivity = s;
        saveSensitivityToEEPROM();
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f", sensitivity);
        pSensChar->setValue(buf);
        SERIAL_PRINTF("[CMD] Sensitivity changed to: %.2f\n", sensitivity);
      }
    }
    else if (cmd == "readbattery") {
      SERIAL_PRINTLN("[CMD] Manual battery read requested");
      readBatteryPercent();
    }
  }
};

/* ----------------------------
   BLE SETUP
---------------------------- */
void setupBLE() {
  SERIAL_PRINTLN("[BLE] Initializing...");
  
  BLEDevice::init("SmartSteps BLE");
  SERIAL_PRINTLN("[BLE] Device initialized");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  SERIAL_PRINTLN("[BLE] Server created");

  BLEService *pService = pServer->createService(SMARTSTEPS_SERVICE_UUID);
  SERIAL_PRINTLN("[BLE] Service created");

  // Steps characteristic with READ + NOTIFY
  pStepsJsonChar = pService->createCharacteristic(
      STEPS_JSON_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | 
      BLECharacteristic::PROPERTY_READ
  );
  pStepsJsonChar->addDescriptor(new BLE2902());
  SERIAL_PRINTLN("[BLE] Steps characteristic created (NOTIFY+READ)");

  // Goal characteristic
  pGoalChar = pService->createCharacteristic(
      GOAL_RW_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  // Sensitivity characteristic
  pSensChar = pService->createCharacteristic(
      SENSITIVITY_RW_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  // Command characteristic
  pCmdChar = pService->createCharacteristic(
      COMMAND_W_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | 
      BLECharacteristic::PROPERTY_WRITE_NR
  );

  // Battery characteristic
  pBatteryChar = pService->createCharacteristic(
      BATTERY_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | 
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pBatteryChar->addDescriptor(new BLE2902());
  SERIAL_PRINTLN("[BLE] Battery characteristic created");

  pGoalChar->setCallbacks(new GoalCharCallbacks());
  pSensChar->setCallbacks(new SensCharCallbacks());
  pCmdChar->setCallbacks(new CommandCharCallbacks());

  // Set initial values
  char bufG[16], bufS[16], batBuf[16];
  snprintf(bufG, sizeof(bufG), "%ld", (long)stepGoal);
  snprintf(bufS, sizeof(bufS), "%.3f", sensitivity);
  snprintf(batBuf, sizeof(batBuf), "%d%%", batteryPercent);

  pGoalChar->setValue(bufG);
  pSensChar->setValue(bufS);
  pBatteryChar->setValue(batBuf);

  // Initial JSON
  char stepsBuf[256];
  snprintf(stepsBuf, sizeof(stepsBuf), 
           "{\"steps\":%lu,\"goal\":%ld,\"goalReachedToday\":%s,\"battery\":%d,\"timestamp\":%lu}",
           stepCount, stepGoal, 
           goalReachedToday ? "true" : "false",
           batteryPercent,
           (unsigned long)(millis() / 1000UL));
  pStepsJsonChar->setValue(stepsBuf);

  // Empty command characteristic
  pCmdChar->setValue("");

  pService->start();
  SERIAL_PRINTLN("[BLE] Service started");

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SMARTSTEPS_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  SERIAL_PRINTLN("[BLE] Advertising started");
  signalAdvertising();

  // ✅ SAFE PLACE TO SEND FIRST NOTIFY
  pStepsJsonChar->notify();
}


/* ----------------------------
   STEPS JSON PUBLISH (COMPACT + REACHED FLAG)
---------------------------- */
void publishStepsJson(uint32_t nowMs) {
  if (!bleDeviceConnected) {
    return;
  }

  // ✅ FIX: Read battery percentage before including it in JSON
  if (BATTERY_ENABLED) {
    readBatteryPercent();
  }

  static char jsonBuf[64];

  int reached = (stepCount >= stepGoal) ? 1 : 0;

  // ✅ FIX: Use full battery percentage instead of dividing by 10
  snprintf(
    jsonBuf,
    sizeof(jsonBuf),
    "{s:%lu,g:%ld,b:%d,r:%d}",
    (unsigned long)stepCount,
    (long)stepGoal,
    batteryPercent,  // ✅ FIX: Changed from batteryPercent / 10 to batteryPercent
    reached
);

  SERIAL_PRINTF("[BLE] Publishing JSON: %s\n", jsonBuf);

  pStepsJsonChar->setValue((uint8_t *)jsonBuf, strlen(jsonBuf));
  pStepsJsonChar->notify();
}



/* ----------------------------
   BATTERY PUBLISH
---------------------------- */
void publishBattery(uint32_t nowMs) {
  if (!BATTERY_ENABLED) return;
  if (!bleDeviceConnected) return;

  readBatteryPercent();  // Update battery reading
  
  char batBuf[16];
  snprintf(batBuf, sizeof(batBuf), "%d%%", batteryPercent);
  pBatteryChar->setValue(batBuf);
  pBatteryChar->notify();
  
  SERIAL_PRINTF("[BLE] Battery published: %s\n", batBuf);
}

void publishBatteryIfNeeded(uint32_t nowMs) {
  if (nowMs - lastPublishBatteryMs >= publishBatteryIntervalMs) {
    lastPublishBatteryMs = nowMs;
    publishBattery(nowMs);
  }
}

/* ----------------------------
   SETUP
---------------------------- */
void setup() {
  // Initialize Serial with proper delay
  SERIAL_BEGIN();
  delay(3000);  // Critical: Wait for Serial monitor
  
  SERIAL_PRINTLN("\n\n==================================");
  SERIAL_PRINTLN("   Smart Steps BLE - ESP32");
  SERIAL_PRINTLN("==================================");
  SERIAL_PRINTF("Free Heap: %d bytes\n", ESP.getFreeHeap());
  SERIAL_PRINTF("CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
  
  // Initialize LEDs
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);
  rgbOff();
  SERIAL_PRINTLN("LEDs initialized");
  
  // Load configuration
  loadConfigFromEEPROM();
  
  // Initialize battery pin if enabled
  if (BATTERY_ENABLED) {
    pinMode(BAT_ADC_PIN, INPUT);
    analogReadResolution(12);
    SERIAL_PRINTLN("Battery monitoring enabled");
  }
  
  // Initialize accelerometer
  SERIAL_PRINTLN("Initializing accelerometer...");
  if (!initAccelerometer()) {
    SERIAL_PRINTLN("FATAL: Accelerometer failed!");
    signalError();
    while(1) {
      delay(1000);
      SERIAL_PRINTLN("Stuck in error state - check accelerometer wiring");
    }
  }
  
  // Initialize BLE
  SERIAL_PRINTLN("Initializing BLE...");
  setupBLE();
  
  // Initialize timers
  lastSampleMs = millis();
  lastPublishStepsMs = millis();
  lastPublishBatteryMs = millis();
  
  SERIAL_PRINTLN("\n=== SETUP COMPLETE ===");
  SERIAL_PRINTLN("System ready. Waiting for BLE connection...\n");
}

/* ----------------------------
   MAIN LOOP - MODIFIED
---------------------------- */
void loop() {
  static uint32_t lastHeartbeatMs = 0;
  uint32_t nowMs = millis();
  
  // Heartbeat every 10 seconds
  if (nowMs - lastHeartbeatMs >= 10000) {
    lastHeartbeatMs = nowMs;
    SERIAL_PRINTF("[HEARTBEAT] Steps: %lu, Goal: %ld, Reached: %d, Connected: %d, Battery: %d%%, Free Heap: %d\n", 
                  stepCount, stepGoal, goalReachedToday, bleDeviceConnected, batteryPercent, ESP.getFreeHeap());
  }
  
  // Sample accelerometer
  if (nowMs - lastSampleMs >= sampleIntervalMs) {
    lastSampleMs = nowMs;
    float ax, ay, az;
    if (readAccel(ax, ay, az)) {
      detectStep(ax, ay, az, nowMs);
    } else {
      SERIAL_PRINTLN("[ERROR] Failed to read accelerometer!");
    }
  }
  
  // Publish steps via BLE - ONLY when goal is reached (lower rate)
  if (goalReachedToday) {
    // ✅ Only periodic publishing AFTER goal reached
    if (nowMs - lastPublishStepsMs >= publishStepsInterval_afterGoal) {
      lastPublishStepsMs = nowMs;
      publishStepsJson(nowMs);
    }
  }
  
  // Handle battery monitoring
  publishBatteryIfNeeded(nowMs);
  
  // Small delay to prevent watchdog issues
  delay(1);
}