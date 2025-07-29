// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

// SleepyLoRaSlave blind controller firmware. receives commands via RS485 UART from master controller.
// The master controller is responsible for managing the LoRa communication and sending commands to this slave device.
// This slave device controls a blind motor and reports its position and state back to the master controller.
// The slave device can also be configured via a web portal to change its RS485 address and other settings.
// The device can be updated OTA via the web portal.


#include <Arduino.h>
#include <Ewma.h>
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <Ticker.h>
#include <nvs_flash.h>
#include <AsyncEventSource.h>
#include <algorithm> // For std::min and std::max

#include "Command_Register.h"
#include "BlindMotorController.h"
#include "web_portal.h"


// Debug macros with log levels
#define DEBUG_SERIAL

#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL LOG_LEVEL_DEBUG  // Set default log level here
#endif

#ifdef DEBUG_SERIAL
  #define LOG_ERROR(...)   do { if (DEBUG_LEVEL >= LOG_LEVEL_ERROR) Serial.printf("[ERROR] " __VA_ARGS__); } while(0)
  #define LOG_WARN(...)    do { if (DEBUG_LEVEL >= LOG_LEVEL_WARN)  Serial.printf("[WARN] "  __VA_ARGS__); } while(0)
  #define LOG_INFO(...)    do { if (DEBUG_LEVEL >= LOG_LEVEL_INFO)  Serial.printf("[INFO] "  __VA_ARGS__); } while(0)
  #define LOG_DEBUG(...)   do { if (DEBUG_LEVEL >= LOG_LEVEL_DEBUG) Serial.printf("[DEBUG] " __VA_ARGS__); } while(0)
  #define LOG_DEBUG_RAW(...) do { if (DEBUG_LEVEL >= LOG_LEVEL_DEBUG) Serial.printf(__VA_ARGS__); } while(0)
#else
  #define LOG_ERROR(...)
  #define LOG_WARN(...)
  #define LOG_INFO(...)
  #define LOG_DEBUG(...)
  #define LOG_DEBUG_RAW(...)
#endif

// Pin definitions
#define POSITION_PIN 4
#define IN_A_PIN 10
#define IN_B_PIN 20
#define EN_PIN 7
#define EN_PWM_CHANNEL 0 // PWM channel for enable pin
#define POSITION_REF_PIN 3 // GPIO3: Pot top (reference)

// RS485 direction control pin for slave
#define RS485_DIR_PIN 8
#define RS485_TRANSMIT()  digitalWrite(RS485_DIR_PIN, HIGH)
#define RS485_RECEIVE()   digitalWrite(RS485_DIR_PIN, LOW)

// UART protocol constants
#define UART_START_BYTE 0xAA
#define UART_END_BYTE   0x55
#define UART_BAUDRATE   115200
#define UART_TX_PIN     6
#define UART_RX_PIN     5
uint8_t slave_number = 1; // default

uint8_t rs485_addr = 0x20; // Default address, can be changed via web portal

// Command codes (should match master's Command_Register.h)
#define BLIND_COMMAND 0x03
#define GET_STATUS    0x08
#define UPDATE_SLAVE 0x09


// State definitions (should match master's enum)
#define STATE_CLOSING  0x01
#define STATE_OPENING  0x02
#define STATE_CLOSED   0x03
#define STATE_OPEN     0x04

//dummy definitions for web portal library
uint32_t config_gatewayID = 0;
uint8_t config_aes_key[16] = {0};
uint8_t config_hmacKey[10] = {0};
uint32_t config_rf_frequency = 868100000;
bool configMode = false;
void saveConfigToFlash() {}
bool hexToBytes(const char*, uint8_t*, size_t) { return false; }
void scanUARTSlavesAndPublish() {}
bool sendCommandToSlaveUART(uint8_t, uint8_t, uint8_t*, size_t) { return false; }
uint8_t detected_slaves[1] = {0};
uint8_t detected_slave_count = 0;
uint16_t slave_positions[1] = {0};
bool slave_moving[1] = {false};
uint32_t deviceID = 0;


volatile uint8_t last_command = 0;
volatile uint8_t last_payload[8];
volatile bool command_received = false;

bool configPortalActive = false;
unsigned long configPortalStartTime = 0;

Preferences prefs;

BlindMotorController actuator(
    IN_A_PIN, IN_B_PIN, EN_PIN, EN_PWM_CHANNEL,
    POSITION_PIN, POSITION_REF_PIN
);

uint16_t targetPos;  // position in % x 10
uint8_t blind_state = STATE_UNKNOWN;

uint16_t blind_position = 0;

bool isAddressInUse(uint8_t candidate_addr);

// CRC-16-CCITT
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void sendUARTResponse(uint8_t command, uint8_t *payload) {
    delayMicroseconds(10); 
    RS485_TRANSMIT();
    delayMicroseconds(20); // Allow line to settle
    uint8_t frame[1 + 1 + 1 + 8 + 2 + 1];
    size_t idx = 0;
    frame[idx++] = UART_START_BYTE;
    frame[idx++] = rs485_addr;
    frame[idx++] = command;
    for (int i = 0; i < 8; ++i) frame[idx++] = payload ? payload[i] : 0;
    uint16_t crc = crc16_ccitt(&frame[1], 1 + 1 + 8);
    frame[idx++] = (crc >> 8) & 0xFF;
    frame[idx++] = crc & 0xFF;
    frame[idx++] = UART_END_BYTE;
    LOG_DEBUG("[UART TX] Raw frame: \r\n");
    for (size_t i = 0; i < sizeof(frame); ++i) LOG_DEBUG_RAW("%02X ", frame[i]);
    LOG_DEBUG_RAW("\r\n");
    Serial1.write(frame, sizeof(frame));
    Serial1.flush();
    delayMicroseconds(30); // Allow last byte to leave the bus
    RS485_RECEIVE();
}

void eraseNVSAndReboot() {
    LOG_ERROR("[NVS] Erasing NVS partition and rebooting...\n");
    nvs_flash_erase();
    nvs_flash_init();
    delay(100);
    ESP.restart();
}

bool isAddressInUse(uint8_t candidate_addr) {
    // Send GET_STATUS to candidate_addr
    uint8_t dummy_payload[8] = {0};
    uint8_t frame[1 + 1 + 1 + 8 + 2 + 1];
    size_t idx = 0;
    frame[idx++] = UART_START_BYTE;
    frame[idx++] = candidate_addr;
    frame[idx++] = GET_STATUS;
    for (int i = 0; i < 8; ++i) frame[idx++] = dummy_payload[i];
    uint16_t crc = crc16_ccitt(&frame[1], 1 + 1 + 8);
    frame[idx++] = (crc >> 8) & 0xFF;
    frame[idx++] = crc & 0xFF;
    frame[idx++] = UART_END_BYTE;

    LOG_INFO("[ADDR CHECK] Polling for slave at address 0x%02X...\r\n", candidate_addr);
    LOG_DEBUG("[ADDR CHECK] TX frame: ");
    for ( size_t i = 0; i < sizeof(frame); ++i) LOG_DEBUG_RAW("%02X ", frame[i]);
    LOG_DEBUG_RAW("\r\n");

    RS485_TRANSMIT();
    delayMicroseconds(20);
    Serial1.write(frame, sizeof(frame));
    Serial1.flush();
    delayMicroseconds(100);
    RS485_RECEIVE();

    // Wait for a response for up to 100ms
    unsigned long start = millis();
    size_t r = 0;
    uint8_t resp[1 + 1 + 1 + 8 + 2 + 1];
    bool in_frame = false;

    while (millis() - start < 100) {
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (!in_frame) {
                if (b == UART_START_BYTE) {
                    in_frame = true;
                    r = 0;
                    resp[r++] = b;
                }
            } else {
                resp[r++] = b;
                if (r == sizeof(resp)) {
                    // Got a full frame
                    LOG_INFO("[ADDR CHECK] RX %u bytes from bus\r\n", r);
                    LOG_DEBUG("[ADDR CHECK] RX frame: ");
                    for (size_t i = 0; i < r; ++i) LOG_DEBUG_RAW("%02X ", resp[i]);
                    LOG_DEBUG_RAW("\r\n");
                    // Check if response is from candidate_addr
                    if (resp[0] == UART_START_BYTE && resp[1] == candidate_addr && resp[sizeof(resp)-1] == UART_END_BYTE) {
                        LOG_WARN("[ADDR CHECK] Address 0x%02X is already in use!\r\n", candidate_addr);
                        return true; // Address is in use
                    }
                    in_frame = false;
                    r = 0;
                }
            }
        }
    }
    LOG_INFO("[ADDR CHECK] Address 0x%02X appears free\r\n", candidate_addr);
    return false; // No response, address is free
}

void setup() {
    // Robust NVS init and recovery
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        LOG_ERROR("[NVS] nvs_flash_init() failed (%d), erasing and re-initializing...\n", nvs_ret);
        nvs_flash_erase();
        nvs_ret = nvs_flash_init();
        if (nvs_ret != ESP_OK) {
            LOG_ERROR("[NVS] nvs_flash_init() failed after erase (%d), rebooting...\n", nvs_ret);
            delay(100);
            ESP.restart();
        }
    } else if (nvs_ret != ESP_OK) {
        LOG_ERROR("[NVS] nvs_flash_init() failed (%d), rebooting...\n", nvs_ret);
        delay(100);
        ESP.restart();
    }

    // Robustly open slavecfg namespace, create if missing
    if (!prefs.begin("slavecfg", true)) {
        LOG_WARN("[NVS] Preferences.begin('slavecfg', true) failed, trying RW mode to create namespace...\n");
        if (!prefs.begin("slavecfg", false)) {
            LOG_ERROR("[NVS] Preferences.begin('slavecfg', false) also failed, erasing NVS and rebooting!\n");
            eraseNVSAndReboot();
        }
        // Namespace created, write default value
        prefs.putUChar("slave_number", 1);
       
        slave_number = 1;
        prefs.end();
    } else {
        slave_number = prefs.getUChar("slave_number", 1);
        prefs.end();
    }
    rs485_addr = BASE_ADDR + (slave_number - 1);
    actuator.begin();
    Serial.begin(115200);
    pinMode(RS485_DIR_PIN, OUTPUT);
    RS485_RECEIVE(); // Default to receive mode
    delay(10);
    Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    while (!Serial1) { delay(10); }
}

void loop() {
    static uint8_t buf[1 + 1 + 1 + 8 + 2 + 1];
    static size_t idx = 0;
    static unsigned long last_position_time = 0;
    static int last_sent_raw = -1;
    static uint16_t last_sent_scaled = 0;
    unsigned long now = millis();

    // UART receive
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (idx == 0 && b != UART_START_BYTE) continue;
        buf[idx++] = b;
        if (idx == sizeof(buf)) {
            LOG_DEBUG("[UART RX] Raw frame: \r\n");
            for (size_t i = 0; i < sizeof(buf); ++i) {
                LOG_DEBUG_RAW("%02X ", buf[i]);
            }
            LOG_DEBUG_RAW("\r\n");
            if (buf[0] == UART_START_BYTE && buf[sizeof(buf)-1] == UART_END_BYTE && buf[1] == rs485_addr) {
                uint16_t crc = (buf[11] << 8) | buf[12];
                LOG_DEBUG("[UART RX] CRC received: %04X, calculated: %04X\r\n", crc, crc16_ccitt(&buf[1], 1 + 1 + 8));
                if (crc16_ccitt(&buf[1], 1 + 1 + 8) == crc) {
                    uint8_t command = buf[2];
                    uint8_t *payload = &buf[3];
                    LOG_DEBUG("[UART RX] Command: 0x%02X\r\n", command);
                    LOG_DEBUG("[UART RX] Payload: ");
                    for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", payload[i]);
                    LOG_DEBUG_RAW("\r\n");
                    if (command == GET_STATUS) {
                        uint8_t resp_payload[8] = {0};
                        blind_position = actuator.readPositionPercent();
                        resp_payload[0] = actuator.getCoverState();
                        resp_payload[1] = (blind_position >> 8) & 0xFF;
                        resp_payload[2] = blind_position & 0xFF;
                        resp_payload[7] = actuator.getLastMoveStatus();
                        LOG_DEBUG("[UART TX] GET_STATUS response: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", resp_payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        sendUARTResponse(GET_STATUS, resp_payload);
                        LOG_DEBUG("[UART TX] Sent GET_STATUS frame\r\n");
                    } else if (command == BLIND_COMMAND) {
                        blind_position = actuator.readPositionPercent();
                        LOG_DEBUG("[UART RX] BLIND_COMMAND payload: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        uint8_t set_state = payload[1];
                        LOG_DEBUG("[UART RX] BLIND_COMMAND set_state: 0x%02X\r\n", set_state);
                        uint16_t set_position = (payload[3] << 8) | payload[2];
                        LOG_DEBUG("[UART RX] BLIND_COMMAND set_position: %u\r\n", set_position);
                        switch (set_state) {
                            case 0x00: targetPos = 0; break;
                            case 0x01: targetPos = 1000; break;
                            case 0x03: targetPos = blind_position; actuator.abortMove(); break;
                            case 0x04: targetPos = constrain(set_position, 0, 100) * 10; break;
                            default: break;
                        }
                        LOG_INFO("Target Position: %.1f%% Current Position: %.1f%% \r\n", targetPos / 10.0f, blind_position / 10.0f);
                        actuator.commandMove(targetPos);
                        uint8_t status_payload[8] = {0};
                        status_payload[0] = actuator.getCoverState();
                        status_payload[1] = (blind_position >> 8) & 0xFF;
                        status_payload[2] = blind_position & 0xFF;
                        status_payload[7] = actuator.getLastMoveStatus(); // Add last_move_status to byte 7
                        LOG_DEBUG("[UART TX] BLIND_COMMAND status response: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", status_payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        sendUARTResponse(GET_STATUS, status_payload);
                        LOG_DEBUG("[UART TX] Sent BLIND_COMMAND status frame\r\n");
                    } else if (command == UPDATE_SLAVE) {
                        LOG_DEBUG("[UART TX] UPDATE_SLAVE command received, starting OTA...\r\n");
                        startConfigAPAndWebserver(actuator);
                        configPortalActive = true;
                        configPortalStartTime = millis();
                    } else {
                        LOG_WARN("[UART RX] Unknown command: 0x%02X\r\n", command);
                    }
                } else {
                    LOG_WARN("[UART RX] CRC mismatch, ignoring frame\r\n");
                }
            } else {
                LOG_WARN("[UART RX] Invalid frame or address\r\n");
            }
            idx = 0;
        }
    }

}


