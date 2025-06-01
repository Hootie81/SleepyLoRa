#include "config.h"
#include <Preferences.h>
#include <Arduino.h>

GatewayConfig config;
Preferences configPrefs;

void saveConfig() {
    configPrefs.begin("gatewaycfg", false);
    configPrefs.putBytes("cfg", &config, sizeof(config));
    configPrefs.end();
    Serial.println("Config saved to flash.");
}

bool loadConfig() {
    configPrefs.begin("gatewaycfg", true);
    bool found = configPrefs.isKey("cfg");
    if (found) configPrefs.getBytes("cfg", &config, sizeof(config));
    configPrefs.end();
    Serial.println(found ? "Config loaded from flash." : "No config found in flash.");
    return found;
}

void clearConfig() {
    configPrefs.begin("gatewaycfg", false);
    configPrefs.clear();
    configPrefs.end();
    Serial.println("Config cleared from flash.");
}
