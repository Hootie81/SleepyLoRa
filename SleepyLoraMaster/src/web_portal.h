#pragma once
#include <ESPAsyncWebServer.h>
#include "BlindMotorController.h"

void setupWebPortal(AsyncWebServer& server, BlindMotorController& blind);
void startConfigAPAndWebserver(BlindMotorController& blind);
void stopConfigAPAndWebserver();

// WiFi config keys
extern String wifi_ssid;
extern String wifi_pass;
extern bool wifi_enable;
extern String wifi_status;
