#include "ConfigPortal.h"

void ConfigPortal::WiFiManagerSaveConfigCallback() {
  configPortal.saveConfig();
}

void ConfigPortal::saveConfig() {
  is_active = false;
  config.setMQTTServer((char *) wmp_mqtt_server.getValue());
  config.setMQTTPort((char * ) wmp_mqtt_port.getValue());
  config.save();
}

void ConfigPortal::setup() {
  wmp_mqtt_server.setValue(config.getMQTTServer(), MQTT_SERVER_STRING_SIZE);
  wmp_mqtt_port.setValue(config.getMQTTPort(), MQTT_PORT_STRING_SIZE);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setSaveConfigCallback(ConfigPortal::WiFiManagerSaveConfigCallback);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&wmp_mqtt_server);
  wifiManager.addParameter(&wmp_mqtt_port);
}

void ConfigPortal::process() {
  wifiManager.process();
}

void ConfigPortal::start(char *name) {
  wifiManager.startConfigPortal(name);
  is_active = true;
}

void ConfigPortal::stop() {
  wifiManager.stopConfigPortal();
  is_active = false;
}

bool ConfigPortal::isActive() {
  return is_active;
}

ConfigPortal configPortal;
