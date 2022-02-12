#include <WiFiManager.h>
#include "Config.h"
#include "ConfigPortal.h"


WiFiManager wifiManager;
WiFiManagerParameter wmp_mqtt_server("mqtt_server", "MQTT Server", "", MQTT_SERVER_STRING_SIZE);
WiFiManagerParameter wmp_mqtt_port("mqtt_port", "MQTT Port", "", MQTT_PORT_STRING_SIZE);

bool configPortalActive = false;

void saveConfigCallback() {
  configPortalActive = false;
  config.setMQTTServer((char *) wmp_mqtt_server.getValue());
  config.setMQTTPort((char * ) wmp_mqtt_port.getValue());
  config.save();
}

void ConfigPortalSetup() {
  wmp_mqtt_server.setValue(config.getMQTTServer(), MQTT_SERVER_STRING_SIZE);
  wmp_mqtt_port.setValue(config.getMQTTPort(), MQTT_PORT_STRING_SIZE);

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
