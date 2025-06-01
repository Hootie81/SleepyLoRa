#pragma once
#include <AsyncMqttClient.h>

void publishGatewayAvailability(bool online);
void publishGatewayStatus();
void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void setupMqttClient();
extern AsyncMqttClient mqttClient;
