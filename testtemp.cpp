#include <WiFi.h>
#include <LoRa.h>
#include "DHT.h"


extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define WIFI_SSID "biniot2021"
#define WIFI_PASSWORD "BINAreTheB35T"

#define MQTT_HOST "iot.devinci.online"
#define MQTT_PORT 1883


//humidity 
#define DHTPIN 14
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

// Temperature MQTT Topic
#define MQTT_PUB_TEMP "qf194182/mcp9700/temperature"
#define MQTT_PUB_LUM "qf194182/LDR/luminosite"
#define MQTT_PUB_DHT "qf194182/DHT/humidity"
#define MQTT_PUB_DHT2 "qf194182/DHT/temperature"

// GPIO where the DS18B20 is connected to
#define oneWireBus 4



OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

float temp;
float temp2;
float lum;
//float dht;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 2000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // Start the DS18B20 sensor
  sensors.begin();
  dht.begin();

  Serial.println(F("DHTxx test!"));
  
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("qf194182", "vG5SgZFWAG");
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New temperature readings
    // Temperature in Celsius degrees
    
    float h = dht.readHumidity();        // read humidity
    float t = dht.readTemperature();  
    float f = dht.readTemperature(true);

    temp2= analogRead(32);
    lum=analogRead(34);
    //dht=analogRead(35);

    temp2=(temp2 - 500)/10;

    if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
}

    // Publish an MQTT message on topic esp32/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp2).c_str());    
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TEMP);    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f \n", temp2);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_LUM, 1, true, String(lum).c_str());    
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_LUM);    Serial.println(packetIdPub2);
    Serial.printf("Message: %.2f \n ", lum);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_DHT, 1, true, String(h).c_str());    
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DHT);    Serial.println(packetIdPub3);
    Serial.printf("Message: %.2f \n ", h);

     uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_DHT2, 1, true, String(t).c_str());    
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_DHT2);    Serial.println(packetIdPub4);
    Serial.printf("Message: %.2f \n ", t);

  }
}