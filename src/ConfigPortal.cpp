#include <WiFiManager.h>
#include "ConfigPortal.h"

WiFiManager wifiManager;

bool configPortalActive = false;

void saveConfigCallback() {
  configPortalActive = false;
}

void ConfigPortalSetup() {
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
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
