// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#include <WiFi.h>
#include "shared.h"
#include "mqtt.h"
#include "blinds.h"
#include "config.h"
#include <AsyncMqttClient.h>
#include <Arduino.h>
#include "Command_Register.h"

// Forward declarations for gateway HA integration
void setupGatewayHA();
void onForgetAllBlinds(const char* topic, const char* payload, unsigned int length);

AsyncMqttClient mqttClient;

void publishGatewayAvailability(bool online) {
    mqttClient.publish("SleepyLoRa/gateway/status/availability", 1, true, online ? "online" : "offline");
}

void publishGatewayStatus() {
    uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t minFreeHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    uint32_t uptimeSec = millis() / 1000;
    char topic[64], payload[32];
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/free_heap");
    snprintf(payload, sizeof(payload), "%u", freeHeap);
    mqttClient.publish(topic, 0, false, payload);
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/min_free_heap");
    snprintf(payload, sizeof(payload), "%u", minFreeHeap);
    mqttClient.publish(topic, 0, false, payload);
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/total_heap");
    snprintf(payload, sizeof(payload), "%u", totalHeap);
    mqttClient.publish(topic, 0, false, payload);
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/uptime");
    snprintf(payload, sizeof(payload), "%u", uptimeSec);
    mqttClient.publish(topic, 0, false, payload);
    // Publish IP address
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/ip");
    String ipStr = WiFi.localIP().toString();
    mqttClient.publish(topic, 0, true, ipStr.c_str());
    // Publish blind count
    snprintf(topic, sizeof(topic), "SleepyLoRa/gateway/status/blind_count");
    snprintf(payload, sizeof(payload), "%u", (unsigned)blindsList.size());
    mqttClient.publish(topic, 0, true, payload);
}

void connectToMqtt() {
    mqttClient.connect();
}

void setupMqttClient() {
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(config.mqtt_host, config.mqtt_port);
    mqttClient.setCredentials(config.mqtt_user, config.mqtt_pass);
}

void onMqttConnect(bool sessionPresent) {
    Serial.println("Connected to MQTT.");
    Serial.printf("MQTT sessionPresent: %s\r\n", sessionPresent ? "true" : "false");
    publishGatewayAvailability(true);
    subscribeToAllBlinds();
    setupGatewayHA(); // Add this to publish HA discovery and status
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print("Disconnected from MQTT. Reason: ");
    Serial.println((int)reason);
    publishGatewayAvailability(false);
    if (WiFi.isConnected()) {
        // Optionally, reconnect logic here
    }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    Serial.print("Message on topic: ");
    Serial.println(topic);
    char msg[64];
    size_t copyLen = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
    strncpy(msg, payload, copyLen);
    msg[copyLen] = '\0';
    uint32_t deviceId = 0;
    uint8_t blindNumber = 0;
    char type[20];
    if (sscanf(topic, "SleepyLoRa/%08X_%hhu/%19s", &deviceId, &blindNumber, type) == 3) {
        if (strcmp(type, "command") == 0) {
            uint8_t set_state = 0xFF;
            if (strcasecmp(msg, "Open") == 0) set_state = 0x01;
            else if (strcasecmp(msg, "Close") == 0) set_state = 0x00;
            else if (strcasecmp(msg, "Stop") == 0) set_state = 0x03;
            if (set_state != 0xFF) {
                gettimeofday(&tv_now, NULL);
                int32_t time = (int32_t)tv_now.tv_sec;
                TXbuffer.push(data::TXpacket{ time, deviceId, BLIND_COMMAND, blindNumber, set_state, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
                Serial.printf("Pushed BLIND_COMMAND to TXbuffer: deviceId=%08X, blindNumber=%u, set_state=0x%02X\r\n", deviceId, blindNumber, set_state);
            }
        } else if (strcmp(type, "set_position") == 0) {
            int setPosition = atoi(msg);
            gettimeofday(&tv_now, NULL);
            int32_t time = (int32_t)tv_now.tv_sec;
            TXbuffer.push(data::TXpacket{ time, deviceId, BLIND_COMMAND, blindNumber, 0x04, (uint8_t)setPosition, 0x00, 0x00, 0x00, 0x00, 0x00 });
            Serial.printf("Pushed BLIND_COMMAND (set position) to TXbuffer: deviceId=%08X, blindNumber=%u, setPosition=%d\r\n", deviceId, blindNumber, setPosition);
        } else if (strcmp(type, "startAP") == 0) {
            // Handle Open Web Portal button press
            gettimeofday(&tv_now, NULL);
            int32_t time = (int32_t)tv_now.tv_sec;
            TXbuffer.push(data::TXpacket{ time, deviceId, UPDATE_FIRMWARE, blindNumber, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
            Serial.printf("Pushed UPDATE_FIRMWARE (open web portal) to TXbuffer: deviceId=%08X, blindNumber=%u\r\n", deviceId, blindNumber);
        }
    }
    // Route forget all button
    onForgetAllBlinds(topic, payload, len);
}
