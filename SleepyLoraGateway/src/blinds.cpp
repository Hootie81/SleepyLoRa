// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#include "blinds.h"
#include <Arduino.h>
#include <vector>
#include <Preferences.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <WiFi.h>
#include <set>

void publishGatewayStatus();
void publishMasterDeviceDiscoveryConfig(uint32_t deviceId);

std::vector<Blind> blindsList;
Preferences blindsPrefs;
extern AsyncMqttClient mqttClient;
Ticker gatewayStatusTicker;

void loadBlindsFromFlash() {
    blindsPrefs.begin("blinds", true);
    size_t count = blindsPrefs.getUInt("count", 0);
    blindsList.clear();
    Serial.print("Loading blinds from flash, count: ");
    Serial.println(count);
    for (size_t i = 0; i < count; ++i) {
        char key[16];
        snprintf(key, sizeof(key), "b%u", i);
        uint64_t val = blindsPrefs.getULong64(key, 0);
        if (val) {
            Blind b;
            b.deviceId = (uint32_t)(val >> 8);
            b.blindNumber = (uint8_t)(val & 0xFF);

            // --- Validation: skip invalid deviceIds ---
            if (b.deviceId == 0 || b.deviceId == 0xFFFFFFFF) {
                Serial.print("Invalid deviceId loaded: ");
                Serial.println(b.deviceId, HEX);
                continue;
            }
            // ------------------------------------------

            blindsList.push_back(b);
            Serial.print("Loaded blind: deviceId=");
            Serial.print(b.deviceId, HEX);
            Serial.print(", blindNumber=");
            Serial.println(b.blindNumber);
        }
    }
    blindsPrefs.end();

    // After loading blinds, publish master device sensors for each unique deviceId
    std::set<uint32_t> uniqueMasters;
    for (const auto& b : blindsList) {
        uniqueMasters.insert(b.deviceId);
    }
    for (auto deviceId : uniqueMasters) {
        publishMasterDeviceDiscoveryConfig(deviceId);
    }
}

void saveBlindsToFlash() {
    blindsPrefs.begin("blinds", false);
    blindsPrefs.putUInt("count", blindsList.size());
    Serial.print("Saving blinds to flash, count: ");
    Serial.println(blindsList.size());
    for (size_t i = 0; i < blindsList.size(); ++i) {
        char key[16];
        snprintf(key, sizeof(key), "b%u", i);
        uint64_t val = ((uint64_t)blindsList[i].deviceId << 8) | blindsList[i].blindNumber;
        blindsPrefs.putULong64(key, val);
        Serial.print("Saved blind: deviceId=");
        Serial.print(blindsList[i].deviceId, HEX);
        Serial.print(", blindNumber=");
        Serial.println(blindsList[i].blindNumber);
    }
    blindsPrefs.end();
}

// Function to clear all blinds from memory and flash
void clearAllBlinds() {
    blindsPrefs.begin("blinds", false);
    size_t count = blindsPrefs.getUInt("count", 0);
    for (size_t i = 0; i < count; ++i) {
        char key[16];
        snprintf(key, sizeof(key), "b%u", i);
        blindsPrefs.remove(key);
    }
    blindsPrefs.putUInt("count", 0);
    blindsPrefs.end();
    blindsList.clear();
    Serial.println("All blinds cleared from flash and memory.");
}

void addBlindIfNew(uint32_t deviceId, uint8_t blindNumber) {
    for (const auto& b : blindsList) {
        if (b.deviceId == deviceId && b.blindNumber == blindNumber) return;
    }
    Blind newBlind = {deviceId, blindNumber};
    blindsList.push_back(newBlind);
    Serial.print("Adding new blind: deviceId=");
    Serial.print(deviceId, HEX);
    Serial.print(", blindNumber=");
    Serial.println(blindNumber);
    saveBlindsToFlash();
    char cmdTopic[64], setPosTopic[64];
    snprintf(cmdTopic, sizeof(cmdTopic), "SleepyLoRa/%08X_%u/command", deviceId, blindNumber);
    snprintf(setPosTopic, sizeof(setPosTopic), "SleepyLoRa/%08X_%u/set_position", deviceId, blindNumber);
    mqttClient.subscribe(cmdTopic, 0);
    Serial.print("Subscribing to topic: ");
    Serial.println(cmdTopic);
    mqttClient.subscribe(setPosTopic, 0);
    Serial.print("Subscribing to topic: ");
    Serial.println(setPosTopic);
    publishHADiscoveryConfig(newBlind);
    publishMasterDeviceDiscoveryConfig(deviceId);
}

// Handler for the forget all button
void onForgetAllBlinds(const char* topic, const char* payload, unsigned int length) {
    if (strcmp(topic, "SleepyLoRa/gateway/forget_all") == 0) {
        clearAllBlinds();
        publishGatewayStatus();
    }
}

// Publish Home Assistant discovery config for the gateway device
void publishGatewayDiscoveryConfig() {
    // Device info
    const char* deviceId = "gateway";
    const char* uniqueId = "SleepyLoRa_gateway";
    const char* deviceName = "SleepyLoRa Gateway";
    // Button discovery
    char buttonTopic[128];
    snprintf(buttonTopic, sizeof(buttonTopic), "homeassistant/button/SleepyLoRa_%s/forget_all/config", deviceId);
    char buttonPayload[512];
    snprintf(buttonPayload, sizeof(buttonPayload),
        "{"
        "\"name\":\"Forget All Blinds\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_forget_all\","  // unique_id
        "\"command_topic\":\"SleepyLoRa/gateway/forget_all\","  // command_topic
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(buttonTopic, 0, true, buttonPayload);
    Serial.print("Published HA discovery for gateway button: ");
    Serial.println(buttonTopic);

    // Status sensor: number of blinds
    char sensorTopic[128];
    snprintf(sensorTopic, sizeof(sensorTopic), "homeassistant/sensor/SleepyLoRa_%s/blind_count/config", deviceId);
    char sensorPayload[512];
    snprintf(sensorPayload, sizeof(sensorPayload),
        "{"
        "\"name\":\"Blind Count\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_blind_count\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/blind_count\","  // state_topic
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(sensorTopic, 0, true, sensorPayload);
    Serial.print("Published HA discovery for gateway sensor: ");
    Serial.println(sensorTopic);

    // IP address sensor
    char ipSensorTopic[128];
    snprintf(ipSensorTopic, sizeof(ipSensorTopic), "homeassistant/sensor/SleepyLoRa_%s/ip/config", deviceId);
    char ipSensorPayload[512];
    snprintf(ipSensorPayload, sizeof(ipSensorPayload),
        "{"
        "\"name\":\"Gateway IP Address\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_ip\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/ip\","  // state_topic
        "\"icon\":\"mdi:ip-network\","  // icon
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(ipSensorTopic, 0, true, ipSensorPayload);
    Serial.print("Published HA discovery for gateway IP sensor: ");
    Serial.println(ipSensorTopic);

    // free_heap sensor
    char freeHeapTopic[128];
    snprintf(freeHeapTopic, sizeof(freeHeapTopic), "homeassistant/sensor/SleepyLoRa_%s/free_heap/config", deviceId);
    char freeHeapPayload[512];
    snprintf(freeHeapPayload, sizeof(freeHeapPayload),
        "{"
        "\"name\":\"Gateway Free Heap\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_free_heap\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/free_heap\","  // state_topic
        "\"unit_of_measurement\":\"bytes\","  // unit
        "\"icon\":\"mdi:memory\","  // icon
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(freeHeapTopic, 0, true, freeHeapPayload);

    // min_free_heap sensor
    char minFreeHeapTopic[128];
    snprintf(minFreeHeapTopic, sizeof(minFreeHeapTopic), "homeassistant/sensor/SleepyLoRa_%s/min_free_heap/config", deviceId);
    char minFreeHeapPayload[512];
    snprintf(minFreeHeapPayload, sizeof(minFreeHeapPayload),
        "{"
        "\"name\":\"Gateway Min Free Heap\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_min_free_heap\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/min_free_heap\","  // state_topic
        "\"unit_of_measurement\":\"bytes\","  // unit
        "\"icon\":\"mdi:memory\","  // icon
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(minFreeHeapTopic, 0, true, minFreeHeapPayload);

    // total_heap sensor
    char totalHeapTopic[128];
    snprintf(totalHeapTopic, sizeof(totalHeapTopic), "homeassistant/sensor/SleepyLoRa_%s/total_heap/config", deviceId);
    char totalHeapPayload[512];
    snprintf(totalHeapPayload, sizeof(totalHeapPayload),
        "{"
        "\"name\":\"Gateway Total Heap\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_total_heap\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/total_heap\","  // state_topic
        "\"unit_of_measurement\":\"bytes\","  // unit
        "\"icon\":\"mdi:memory\","  // icon
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(totalHeapTopic, 0, true, totalHeapPayload);

    // uptime sensor
    char uptimeTopic[128];
    snprintf(uptimeTopic, sizeof(uptimeTopic), "homeassistant/sensor/SleepyLoRa_%s/uptime/config", deviceId);
    char uptimePayload[512];
    snprintf(uptimePayload, sizeof(uptimePayload),
        "{"
        "\"name\":\"Gateway Uptime\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_uptime\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/uptime\","  // state_topic
        "\"unit_of_measurement\":\"s\","  // unit
        "\"icon\":\"mdi:timer-outline\","  // icon
        "\"device_class\":\"duration\","  // device_class
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(uptimeTopic, 0, true, uptimePayload);

    // availability binary_sensor
    char availTopic[128];
    snprintf(availTopic, sizeof(availTopic), "homeassistant/binary_sensor/SleepyLoRa_%s/availability/config", deviceId);
    char availPayload[512];
    snprintf(availPayload, sizeof(availPayload),
        "{"
        "\"name\":\"Gateway Availability\","  // name
        "\"unique_id\":\"SleepyLoRa_%s_availability\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/gateway/status/availability\","  // state_topic
        "\"payload_on\":\"online\","  // payload_on
        "\"payload_off\":\"offline\","  // payload_off
        "\"icon\":\"mdi:lan-connect\","  // icon
        "\"device_class\":\"connectivity\","  // device_class
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Gateway\"}"
        "}",
        deviceId, deviceId, uniqueId, deviceName);
    mqttClient.publish(availTopic, 0, true, availPayload);
}

// Call this in setup after MQTT is connected
void setupGatewayHA() {
    publishGatewayDiscoveryConfig();
    publishGatewayStatus();
    // Subscribe to forget all button
    mqttClient.subscribe("SleepyLoRa/gateway/forget_all", 0);
    Serial.println("Subscribed to gateway forget_all topic");
    // Set up periodic status publishing (every 60s)
    gatewayStatusTicker.attach(60, publishGatewayStatus);
}

void publishHADiscoveryConfig(const Blind& b) {
    char discoveryTopic[128];
    snprintf(discoveryTopic, sizeof(discoveryTopic),
        "homeassistant/cover/SleepyLoRa_%08X_%u/config", b.deviceId, b.blindNumber);

    char name[32];
    snprintf(name, sizeof(name), "Blind %08X_%u", b.deviceId, b.blindNumber);

    char uniqueId[48];
    snprintf(uniqueId, sizeof(uniqueId), "SleepyLoRa_%08X_%u", b.deviceId, b.blindNumber);

    char commandTopic[64], setPosTopic[64], posTopic[64], stateTopic[64];
    snprintf(commandTopic, sizeof(commandTopic), "SleepyLoRa/%08X_%u/command", b.deviceId, b.blindNumber);
    snprintf(setPosTopic, sizeof(setPosTopic), "SleepyLoRa/%08X_%u/set_position", b.deviceId, b.blindNumber);
    snprintf(posTopic, sizeof(posTopic), "SleepyLoRa/%08X_%u/position", b.deviceId, b.blindNumber);
    snprintf(stateTopic, sizeof(stateTopic), "SleepyLoRa/%08X_%u/state", b.deviceId, b.blindNumber);

    char deviceName[64];
    snprintf(deviceName, sizeof(deviceName), "Blind %08X_%u", b.deviceId, b.blindNumber);

    char payload[1024];
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\":\"%s\","  // name
        "\"unique_id\":\"%s\","  // unique_id
        "\"command_topic\":\"%s\","  // command_topic
        "\"set_position_topic\":\"%s\","  // set_position_topic
        "\"position_topic\":\"%s\","  // position_topic
        "\"state_topic\":\"%s\","  // state_topic
        "\"payload_open\":\"open\","  // payload_open
        "\"payload_close\":\"close\","  // payload_close
        "\"payload_stop\":\"stop\","  // payload_stop
        "\"state_open\":\"open\","  // state_open
        "\"state_opening\":\"opening\","  // state_opening
        "\"state_closed\":\"closed\","  // state_closed
        "\"state_closing\":\"closing\","  // state_closing
        "\"state_stopped\":\"stopped\","  // state_stopped
        "\"state_unknown\":\"closed\","  // treat unknown as closed
        "\"optimistic\":false,"
        "\"qos\":0,"
        "\"device_class\":\"blind\","  // add device_class for HA
        "\"availability_topic\":\"SleepyLoRa/%08X_%u/availability\","  // availability
        "\"payload_available\":\"online\","  // payload_available
        "\"payload_not_available\":\"offline\","  // payload_not_available
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Blinds Node\"}"
        "}",
        name, uniqueId, commandTopic, setPosTopic, posTopic, stateTopic, b.deviceId, b.blindNumber, uniqueId, deviceName);

    mqttClient.publish(discoveryTopic, 0, true, payload); // retain = true
    Serial.print("Published Home Assistant discovery config: ");
    Serial.println(discoveryTopic);

    // Last move status sensor (per blind)
    char lastMoveStatusTopic[128];
    snprintf(lastMoveStatusTopic, sizeof(lastMoveStatusTopic), "homeassistant/sensor/SleepyLoRa_%08X_%u_last_move_status/config", b.deviceId, b.blindNumber);
    char lastMoveStatusPayload[512];
    snprintf(lastMoveStatusPayload, sizeof(lastMoveStatusPayload),
        "{"
        "\"name\":\"Blind %08X_%u Last Move Status\","  // name
        "\"unique_id\":\"SleepyLoRa_%08X_%u_last_move_status\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/%08X_%u/last_move_status\","  // state_topic
        "\"availability_topic\":\"SleepyLoRa/%08X_%u/availability\","  // availability
        "\"device_class\":\"enum\","  // device_class
        "\"options\":[\"OK\",\"TIMEOUT\",\"MOVING\",\"UNKNOWN\"],"
        "\"device\":{\"identifiers\":[\"SleepyLoRa_%08X_%u\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Blinds Node\"}"
        "}",
        b.deviceId, b.blindNumber, b.deviceId, b.blindNumber, b.deviceId, b.blindNumber, b.deviceId, b.blindNumber, b.deviceId, b.blindNumber, deviceName);
    mqttClient.publish(lastMoveStatusTopic, 0, true, lastMoveStatusPayload);

    // Update button
    char updateTopic[128];
    char updatePayload[512];
    snprintf(updateTopic, sizeof(updateTopic), "homeassistant/button/SleepyLoRa_%08X_%u/startAP/config", b.deviceId, b.blindNumber);
    snprintf(updatePayload, sizeof(updatePayload),
        "{"
        "\"name\": \"Open Web Portal\","
        "\"command_topic\": \"SleepyLoRa/%08X_%u/startAP\","
        "\"unique_id\": \"SleepyLoRa_%08X_%u_startAP\","    
        "\"device\": {\"identifiers\": [\"SleepyLoRa_%08X_%u\"]}"
        "}",
        b.deviceId, b.blindNumber, b.deviceId, b.blindNumber, b.deviceId, b.blindNumber
    );
    mqttClient.publish(updateTopic, 0, true, updatePayload);

}

// Publish HA discovery config for master device-level sensors (battery, awakeSeconds, wakeCount)
void publishMasterDeviceDiscoveryConfig(uint32_t deviceId) {
    char deviceName[64];
    snprintf(deviceName, sizeof(deviceName), "Blinds Master %08X", deviceId);
    char identifier[32];
    snprintf(identifier, sizeof(identifier), "SleepyLoRa_%08X", deviceId);
    // Battery voltage sensor
    char batteryTopic[128];
    snprintf(batteryTopic, sizeof(batteryTopic), "homeassistant/sensor/SleepyLoRa_%08X_battery/config", deviceId);
    char batteryPayload[512];
    snprintf(batteryPayload, sizeof(batteryPayload),
        "{"
        "\"name\":\"Blinds Battery\","  // name
        "\"unique_id\":\"SleepyLoRa_%08X_battery\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/%08X/status/Battery\","  // state_topic
        "\"unit_of_measurement\":\"mV\","  // unit
        "\"device_class\":\"voltage\","  // device_class
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Blinds Master\"}"
        "}",
        deviceId, deviceId, identifier, deviceName);
    mqttClient.publish(batteryTopic, 0, true, batteryPayload);
    // Awake seconds sensor
    char awakeSecTopic[128];
    snprintf(awakeSecTopic, sizeof(awakeSecTopic), "homeassistant/sensor/SleepyLoRa_%08X_awake_seconds/config", deviceId);
    char awakeSecPayload[512];
    snprintf(awakeSecPayload, sizeof(awakeSecPayload),
        "{"
        "\"name\":\"Blinds Awake Seconds\","  // name
        "\"unique_id\":\"SleepyLoRa_%08X_awake_seconds\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/%08X/status/awakeSeconds\","  // state_topic
        "\"unit_of_measurement\":\"s\","  // unit
        "\"device_class\":\"duration\","  // device_class
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Blinds Master\"}"
        "}",
        deviceId, deviceId, identifier, deviceName);
    mqttClient.publish(awakeSecTopic, 0, true, awakeSecPayload);
    // Wake count sensor
    char wakeCountTopic[128];
    snprintf(wakeCountTopic, sizeof(wakeCountTopic), "homeassistant/sensor/SleepyLoRa_%08X_wake_count/config", deviceId);
    char wakeCountPayload[512];
    snprintf(wakeCountPayload, sizeof(wakeCountPayload),
        "{"
        "\"name\":\"Blinds Wake Count\","  // name
        "\"unique_id\":\"SleepyLoRa_%08X_wake_count\","  // unique_id
        "\"state_topic\":\"SleepyLoRa/%08X/status/wakeCount\","  // state_topic
        "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"SleepyLoRa\",\"model\":\"Blinds Master\"}"
        "}",
        deviceId, deviceId, identifier, deviceName);
    mqttClient.publish(wakeCountTopic, 0, true, wakeCountPayload);
}

void subscribeToAllBlinds() {
    for (const auto& b : blindsList) {
        char cmdTopic[64], setPosTopic[64], updateTopic[128];
        snprintf(cmdTopic, sizeof(cmdTopic), "SleepyLoRa/%08X_%u/command", b.deviceId, b.blindNumber);
        snprintf(setPosTopic, sizeof(setPosTopic), "SleepyLoRa/%08X_%u/set_position", b.deviceId, b.blindNumber);
        snprintf(updateTopic, sizeof(updateTopic), "SleepyLoRa/%08X_%u/startAP", b.deviceId, b.blindNumber);
        mqttClient.subscribe(cmdTopic, 0);
        Serial.print("Subscribing to topic: ");
        Serial.println(cmdTopic);
        mqttClient.subscribe(setPosTopic, 0);
        Serial.print("Subscribing to topic: ");
        Serial.println(setPosTopic);
        mqttClient.subscribe(updateTopic, 0);
        Serial.print("Subscribing to topic: ");
        Serial.println(updateTopic);
        publishHADiscoveryConfig(b);
    }
}
