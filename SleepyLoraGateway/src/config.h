#pragma once

#include <stdint.h>
#include <Preferences.h>

struct GatewayConfig {
    char wifi_ssid[32];
    char wifi_pass[64];
    char mqtt_host[64];
    uint16_t mqtt_port;
    char mqtt_user[32];
    char mqtt_pass[64];
    char ntp_server[64];
    int32_t ntp_offset;
    uint8_t aes_key[16];
    uint8_t hmacKey[10];
    uint32_t rf_frequency = 915000000; // Default to AU915 (Australia)
};

extern GatewayConfig config;
extern Preferences configPrefs;

void saveConfig();
bool loadConfig();
void clearConfig();
