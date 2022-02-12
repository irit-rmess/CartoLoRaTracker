#ifndef CONFIGPORTAL_H
#define CONFIGPORTAL_H

#include <WiFiManager.h>
#include "Config.h"

class ConfigPortal {
public:
  void setup();
  void process();
  void start(char *name);
  void stop();
  bool isActive();
private:
  bool is_active = false;
  WiFiManager wifiManager;
  WiFiManagerParameter wmp_mqtt_server {"mqtt_server", "MQTT Server", "", MQTT_SERVER_STRING_SIZE};
  WiFiManagerParameter wmp_mqtt_port {"mqtt_port", "MQTT Port", "", MQTT_PORT_STRING_SIZE};

  void saveConfig();
  static void WiFiManagerSaveConfigCallback();
};

extern ConfigPortal configPortal;
#endif
