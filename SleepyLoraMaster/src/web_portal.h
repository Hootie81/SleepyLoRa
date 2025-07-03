#pragma once
#include <ESPAsyncWebServer.h>
#include "BlindMotorController.h"

void setupWebPortal(AsyncWebServer& server, BlindMotorController& blind);
void startConfigAPAndWebserver(BlindMotorController& blind);
void stopConfigAPAndWebserver();
