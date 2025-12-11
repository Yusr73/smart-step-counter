#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ----------------------------------------------------
   WIFI CONFIGURATION
   ---------------------------------------------------- */
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";

/* ----------------------------------------------------
   MQTT CONFIGURATION
   ---------------------------------------------------- */
const char* mqtt_server = "YOUR_MQTT_BROKER";
const int   mqtt_port   = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

/* ----------------------------------------------------
   PLACEHOLDER: GLOBAL VARIABLES FOR STEP DETECTION
   ---------------------------------------------------- */
// int steps = 0;
// float x, y, z;
// etc...

/* ----------------------------------------------------
   WIFI CONNECTION
   ---------------------------------------------------- */
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

/* ----------------------------------------------------
   MQTT CONNECTION
   ---------------------------------------------------- */
void connectMQTT() {
  while (!client.connected()) {
    client.connect("ESP32_StepCounter");
    delay(500);
  }
}

/* ----------------------------------------------------
   JSON PUBLISHING (CALL THIS WHEN A STEP IS DETECTED)
   ---------------------------------------------------- */
void publishSteps(int steps) {
  StaticJsonDocument<128> doc;
  doc["steps"] = steps;
  doc["timestamp"] = time(nullptr);

  char buffer[128];
  serializeJson(doc, buffer);

  client.publish("smartdevice/steps", buffer);
}

/* ----------------------------------------------------
   SETUP
   ---------------------------------------------------- */
void setup() {
  Serial.begin(115200);

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();

  /* -----------------------------------------------
     PLACEHOLDER: SENSOR INITIALIZATION
     ----------------------------------------------- */
     // initAccelerometer();
     // calibrateSensor();
}

/* ----------------------------------------------------
   LOOP
   ---------------------------------------------------- */
void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();

  /* -----------------------------------------------
     PLACEHOLDER: STEP DETECTION LOGIC
     ----------------------------------------------- */

     // readAccelerometer();
     // detectStep();
     // if (stepDetected) {
     //     steps++;
     //     publishSteps(steps);  
     // }

  
}
