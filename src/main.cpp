#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <env.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <espMqttClientAsync.h>

espMqttClientAsync mqttClient;
bool reconnectMqtt = false;
uint32_t lastReconnect = 0;
unsigned long lastMsg = 0;

Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PSSWD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  if (!mqttClient.connect()) {
    reconnectMqtt = true;
    lastReconnect = millis();
    Serial.println("Connecting failed.");
  } else {
    reconnectMqtt = false;
  }
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    break;
  default:
    break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.printf("Disconnected from MQTT: %u.\n", static_cast<uint8_t>(reason));

  if (WiFi.isConnected()) {
    reconnectMqtt = true;
    lastReconnect = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  while (!bme.begin(0x76)) {
    Serial.println("bme not find");
    delay(1000); 
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PSSWD);

  connectToWiFi();
}

void loop() {
  static uint32_t currentMillis = millis();

  if (reconnectMqtt && currentMillis - lastReconnect > 5000) {
    connectToMqtt();
  }

  if (mqttClient.connected()) {

    unsigned long now = millis();
    
    if (now - lastMsg > 10000) {

      sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);
      delay(500);

      lastMsg = now;

      float temperature = temp_event.temperature;

      mqttClient.publish("outside/bme280/temperature", 1, false, String(temperature).c_str());

      delay(500);

      float humidity = humidity_event.relative_humidity;
      mqttClient.publish("outside/bme280/humidity", 1, false, String(humidity).c_str());

      delay(500);

      float pressure = pressure_event.pressure * 0.750064;
      mqttClient.publish("outside/bme280/pressure", 1, false, String(pressure).c_str());

      delay(1000);
      
      
    }
  }
}
