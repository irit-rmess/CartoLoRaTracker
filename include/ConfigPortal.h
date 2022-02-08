#ifndef CONFIGPORTAL_H
#define CONFIGPORTAL_H
void ConfigPortalSetup();
void ConfigPortalProcess();
void ConfigPortalStart(char *name);
void ConfigPortalStop();
bool ConfigPortalActive();
char *getMQTTServer();
uint16_t getMQTTPort();
#endif
