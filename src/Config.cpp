#include "Config.h"

void Config::begin() {
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

char * Config::getMQTTServer() {
  return mqtt_server;
}

char * Config::getMQTTPort() {
  return mqtt_port;
}

void Config::setMQTTServer(char * mqtt_server) {
  strncpy(this->mqtt_server, mqtt_server, MQTT_SERVER_STRING_SIZE);
}

void Config::setMQTTPort(char * mqtt_port) {
  strncpy(this->mqtt_port, mqtt_port, MQTT_PORT_STRING_SIZE);
}

void Config::save() {
  DynamicJsonDocument json(1024);

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(json, configFile);

  configFile.close();
}

Config config;
