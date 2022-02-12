#ifndef CONFIG_H
#define CONFIG_H

#include <SPIFFS.h>
#include <ArduinoJson.h>

#define MQTT_SERVER_STRING_SIZE 256
#define MQTT_PORT_STRING_SIZE 6

class Config {
public:
  void begin();
  void save();
  char * getMQTTServer();
  char * getMQTTPort();
  void setMQTTServer(char * mqtt_server);
  void setMQTTPort(char * mqtt_port);
private:
  char mqtt_server[MQTT_SERVER_STRING_SIZE];
  char mqtt_port[MQTT_PORT_STRING_SIZE];
};

extern Config config;

#endif
