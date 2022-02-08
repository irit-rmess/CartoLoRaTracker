#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "ConfigPortal.h"

#define MQTT_SERVER_STRING_SIZE 256
#define MQTT_PORT_STRING_SIZE 6

char mqtt_server[MQTT_SERVER_STRING_SIZE];
char mqtt_port[MQTT_PORT_STRING_SIZE];

WiFiManager wifiManager;
WiFiManagerParameter wmp_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, MQTT_SERVER_STRING_SIZE);
WiFiManagerParameter wmp_mqtt_port("mqtt_port", "MQTT Port", mqtt_server, MQTT_PORT_STRING_SIZE);

bool configPortalActive = false;


void readSettings()
{
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS, formatting");
    SPIFFS.format();
  }
}

void saveSettings()
{
  strcpy(mqtt_server, wmp_mqtt_server.getValue());
  strcpy(mqtt_port, wmp_mqtt_port.getValue());

  DynamicJsonDocument json(1024);

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(json, Serial);
  serializeJson(json, configFile);

  configFile.close();
}

void saveConfigCallback() {
  configPortalActive = false;
  saveSettings();
}

void ConfigPortalSetup() {
  readSettings();
  wmp_mqtt_server.setValue(mqtt_server, MQTT_SERVER_STRING_SIZE);
  wmp_mqtt_port.setValue(mqtt_port, MQTT_PORT_STRING_SIZE);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&wmp_mqtt_server);
  wifiManager.addParameter(&wmp_mqtt_port);
}

void ConfigPortalProcess() {
  wifiManager.process();
}

void ConfigPortalStart(char *name) {
  wifiManager.startConfigPortal(name);
  configPortalActive = true;
}

void ConfigPortalStop() {
  wifiManager.stopConfigPortal();
  configPortalActive = false;
}

bool ConfigPortalActive() {
  return configPortalActive;
}

char *getMQTTServer() {
  return mqtt_server;
}

uint16_t getMQTTPort() {
  return (uint16_t) strtol(mqtt_port, NULL, 10);
}
