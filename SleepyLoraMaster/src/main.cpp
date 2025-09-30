// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

// SleepyLoRaMaster blind controller firmware. Receives commands via LoRa from the Gateway.
// This Master device controls a blind motor and reports its position and state back to the Gateway.
// The master controller is responsible for managing the LoRa communication and forwarding commands to slave devices via RS485.
// The master controller can also be configured via a web portal to change its settings.
// The device can be updated OTA via the web portal.
// the master will scan for RS485 slaves on startup and store them in flash.
// The master will also periodically check the status of the slaves and update the Gateway with their information.
// The master will also handle any incoming commands from the Gateway and forward them to the appropriate slave device.


#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <rom/rtc.h>
#include <driver/rtc_io.h>
#include <ESP32Time.h>
#include <CryptoAES_CBC.h> /* https://github.com/Obsttube/CryptoAES_CBC/tree/master */
#include <AES.h>           /* https://github.com/Obsttube/CryptoAES_CBC/tree/master */
#include <string.h>        /* https://github.com/Obsttube/CryptoAES_CBC/tree/master */
#include "sha1.h"          // for OTP
#include "TOTP.h"          // for OTP
#include <Ewma.h>          // exponential weighted moving average for position and batt voltage filtering
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h> // I2C master
#include <nvs_flash.h>
#include <AsyncEventSource.h>
#include <algorithm> // For std::min and std::max

#include "LoRa_settings.h"
#include "Device_settings.h"
#include "Command_Register.h"

#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <Update.h>

#include <BlindMotorController.h>
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
  #define LOG_INFO_RAW(...) do { if (DEBUG_LEVEL >= LOG_LEVEL_INFO)  Serial.printf(__VA_ARGS__); } while(0)
  #define LOG_DEBUG(...)   do { if (DEBUG_LEVEL >= LOG_LEVEL_DEBUG) Serial.printf("[DEBUG] " __VA_ARGS__); } while(0)
  #define LOG_DEBUG_RAW(...) do { if (DEBUG_LEVEL >= LOG_LEVEL_DEBUG) Serial.printf(__VA_ARGS__); } while(0)
#else
  #define LOG_ERROR(...)
  #define LOG_WARN(...)
  #define LOG_INFO(...)
  #define LOG_INFO_RAW(...)
  #define LOG_DEBUG(...)
  #define LOG_DEBUG_RAW(...)
#endif



// Set to 1 to stop any output on Serial
#define BATT_SAVE_ON 0

#define CONFIG_BTN_INPUT 40
#define CONFIG_BTN_OUTPUT 42

#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 2000

#define BASE_ADDR 0x20 // Example base address for slaves
#define MAX_SLAVES 12    // Adjust as needed

// UART protocol constants
#define RS485_DIR_PIN 39
#define UART_TX_PIN     46
#define UART_RX_PIN      45
#define RS485_TRANSMIT()  digitalWrite(RS485_DIR_PIN, HIGH)
#define RS485_RECEIVE()   digitalWrite(RS485_DIR_PIN, LOW)
#define UART_START_BYTE 0xAA
#define UART_END_BYTE   0x55
#define UART_BAUDRATE   115200

// dummy values for common library
uint8_t slave_number = 0;
uint8_t rs485_addr = 0;
bool isAddressInUse(unsigned char) { return false; }

#define GET_STATUS 0x08
#define UPDATE_SLAVE 0x09
#define UART_TIMEOUT_MS 90
#define BLIND_COMMAND 0x03

uint8_t slave_miss_count[MAX_SLAVES+1] = {0}; // Track missed polls per slave
#define SLAVE_MISS_THRESHOLD 3                // Number of missed polls before marking as not moving

int cached_raw_position = 0;
uint16_t cached_scaled_position = 0;

#define MOVE_STATUS_OK 0x01
#define MOVE_STATUS_TIMEOUT 0x02
#define MOVE_STATUS_MOVING 0x03
uint8_t last_move_status = MOVE_STATUS_OK;
bool vBAT(bool state);
bool vEXT(bool vext_set_state);
void ensureVBATOn();
void processSlaveStatusResponse(uint8_t blind_number, uint8_t state);
void prepareCommand(uint8_t prep_command, uint8_t blind_number = 0, uint8_t state = 0, uint16_t position = 0, uint16_t real_position = 0, uint8_t last_move_status = 0);
bool loadConfigFromFlash();

bool slave_moving[MAX_SLAVES+1] = {false};

volatile bool configButtonInterruptFlag = false;
volatile bool buttonPressed = false;
volatile unsigned long buttonPressTime = 0;
volatile unsigned long lastInterruptTime = 0;
bool buttonHandled = false;
unsigned long last_slave_poll = 0;
uint8_t last_polled_slave = 1;
unsigned long lastTimeSyncRequest = 0;
bool configPortalActive = false;
unsigned long configPortalStartTime = 0;

Preferences prefs;

// In-memory config
uint32_t config_gatewayID = 0;
uint8_t config_aes_key[16];
uint8_t config_hmacKey[10];
uint32_t config_rf_frequency = 915000000; // Default to AU915 (Australia)
// RTC-persistent list of detected slaves
RTC_DATA_ATTR uint8_t detected_slaves[16] = {0}; // supports up to 15 slaves, 0 = unused
RTC_DATA_ATTR uint8_t detected_slave_count = 0;
unsigned long last_slave_poll_time = 0;
// Provide global access for web_portal.cpp and others
RTC_DATA_ATTR uint16_t slave_positions[16] = {0};

extern uint16_t slave_positions[];
unsigned long vbat_keep_on_until = 0;

bool ledOnWhileAwake = false; // Option to control LED while awake

struct timeval tv_now;
ESP32Time rtc(28800);

/** creating an object of TOTP class */
//TOTP otp = TOTP(config_hmacKey, 10, 1);
TOTP* otp = nullptr;

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

/** creating an object of AES128 class */
AES128 aes128;
byte encrypted[16];  //temp storage
byte decrypted[16];  //temp storage

Ewma posEwma(POS_ALPHA);
Ewma battEwma(BATT_ALPHA);

BlindMotorController actuator(
    IN_A_PIN, IN_B_PIN, EN_PIN, EN_PWM_CHANNEL,
    POSITION_PIN, POSITION_REF_PIN
);

/** LoRa callback events */
static RadioEvents_t RadioEvents;

/** HW configuration structure for the LoRa library */
hw_config hwConfig;

// Event declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);

// flag once full init done to be able to tx
bool fullInit = false;

/** The encoded part of the message */
// order of OTP 3bytes and command prior to 32bit node id, allows 4 extra bytes of payload while still being 16bytes for aes128
struct dataPacket {
  uint8_t OTP[3];
  uint8_t command;
  uint32_t nodeFROMId;
  uint8_t payload[8];
} dataToENC, dataDEC;

/** The Data message will be sent by the node */
struct loraPacket {
  uint32_t nodeToId;
  uint8_t dataEncrypted[16];
} loraTXpacket, prevloraRXpacket, tmploraRXpacket;

namespace data {
typedef struct {
  uint32_t nodeToId;
  uint8_t dataEncrypted[16];
} loraRXpacket;
}

CircularBuffer<data::loraRXpacket, 10> loraRXbuffer;

struct commandPacket {
  uint8_t command;
  uint8_t payload[8];
} command, commandTX;

/** The sensor node ID, created from MAC of ESP32 */
uint32_t deviceID;

/** LoRa error timeout */
time_t loraTimeout;

/** LoRa TX time */
time_t loraTXTime;

/** CAD repeat counter */
uint8_t cadRepeat;

bool commandReady = false;

bool txComplete = true;

int buff_size = 0;

/** The wakeup time */
time_t wakeup;
time_t start_enc;
time_t position_updated;
time_t status_update;
time_t slave_status_update;

time_t motor_start_time;
time_t motor_run_time;

uint16_t targetPos;  // position in % x 10
uint8_t blind_state = STATE_UNKNOWN;

bool vext_state = false;
bool vbat_state = false;

bool slave_reading = false;

RTC_DATA_ATTR uint16_t wakeCount;
RTC_DATA_ATTR time_t awakeTime;
RTC_DATA_ATTR time_t lastSend;
RTC_DATA_ATTR time_t lastTimeSync;  //records the last time the timeSync was performed for rate limiting
RTC_DATA_ATTR int timeSync;  //records if we are currently performing a time sync, and counts number of attempts
RTC_DATA_ATTR uint32_t timeNow;  //saved so it can sleep when waiting for timesync
#ifdef SIMULATOR
RTC_DATA_ATTR int posSIM = 0;
#endif

bool msgInBuffer = false; // Flag to indicate if a message is in the RX buffer when performing a time sync



bool configMode = false;

// Buffer for uploaded config file
String uploadedConfigFile;

/** declare functions */
void setVBATKeepOn(unsigned long duration_ms);
void goToSleep(void);
void radioSetup(void);
void radioInit(void);
uint8_t checkOTP(uint8_t OTP[3]);
void buildPacket(uint32_t toNodeID, time_t OTPtime, commandPacket commandToSend);
void actionCommand(void);
void parsePacket(void);
void performTimeSync(void);
uint16_t checkBAT(void);
void print_reset_reason(RESET_REASON reason);
void startConfigAPAndWebserver();
void stopConfigAPAndWebserver();
void scanUARTSlavesAndPublish();
const char* getMoveStatusString(uint8_t status);
void handleSetClosedLimit(AsyncWebServerRequest *request);
void handleSetOpenLimit(AsyncWebServerRequest *request);
void handleGetPosition(AsyncWebServerRequest *request);
void updateVBATState();
bool isConfigMissing() {
  return !loadConfigFromFlash();
}
bool any_slave_moving() {
  for (uint8_t i = 1; i <= MAX_SLAVES; ++i) {
      if (slave_moving[i]) return true;
  }
  return false;
}
void loadTimingSettingsFromFlash();
void handleAutoCalibrate(AsyncWebServerRequest *request);

// Helper: hex string to bytes
bool hexToBytes(const char* hex, uint8_t* buf, size_t len) {
for (size_t i = 0; i < len; ++i) {
  char tmp[3] = {hex[i*2], hex[i*2+1], 0};
  char* endptr;
  long val = strtol(tmp, &endptr, 16);
  if (*endptr != 0) return false;
  buf[i] = (uint8_t)val;
}
return true;
}

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


// Send UART frame to slave
bool sendUARTCommandToSlave(uint8_t slave_addr, uint8_t command, uint8_t *payload) {
  RS485_TRANSMIT();
  delayMicroseconds(10); // Allow line to settle
  uint8_t frame[1 + 1 + 1 + 8 + 2 + 1];
  size_t idx = 0;
  frame[idx++] = UART_START_BYTE;
  frame[idx++] = slave_addr;
  frame[idx++] = command;
  for (int i = 0; i < 8; ++i) frame[idx++] = payload ? payload[i] : 0;
  uint16_t crc = crc16_ccitt(&frame[1], 1 + 1 + 8); // addr+cmd+payload
  frame[idx++] = (crc >> 8) & 0xFF;
  frame[idx++] = crc & 0xFF;
  frame[idx++] = UART_END_BYTE;
  Serial1.write(frame, sizeof(frame));
  Serial1.flush();
  delayMicroseconds(5); // Allow last byte to leave the bus
  RS485_RECEIVE();
  return true;
}

// Receive UART frame from slave (blocking, with timeout)
bool receiveUARTResponse(uint8_t expected_addr, uint8_t *command, uint8_t *payload, uint32_t timeout_ms = UART_TIMEOUT_MS) {
  uint8_t buf[1 + 1 + 1 + 8 + 2 + 1];
  size_t idx = 0;
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
      if (Serial1.available()) {
          uint8_t b = Serial1.read();
          if (idx == 0 && b != UART_START_BYTE) {
              LOG_DEBUG("[UART RX] Skipping byte 0x%02X (waiting for start byte)\r\n", b);
              continue;
          }
          buf[idx++] = b;
          LOG_DEBUG("[UART RX] Received byte: 0x%02X (idx=%d)\r\n", b, idx-1);
          if (idx == sizeof(buf)) {
              LOG_DEBUG("[UART RX] Full frame received: ");
              for (size_t i = 0; i < sizeof(buf); ++i) LOG_DEBUG_RAW("%02X ", buf[i]);
              LOG_DEBUG_RAW("\r\n");
              if (buf[0] != UART_START_BYTE || buf[13] != UART_END_BYTE) {
                  LOG_ERROR("[UART RX] Invalid frame: start=0x%02X end=0x%02X\r\n", buf[0], buf[13]);
                  return false;
              }
              if (buf[1] != expected_addr) {
                  LOG_ERROR("[UART RX] Address mismatch: got 0x%02X, expected 0x%02X\r\n", buf[1], expected_addr);
                  return false;
              }
              uint16_t crc = (buf[11] << 8) | buf[12];
              uint16_t calc_crc = crc16_ccitt(&buf[1], 1 + 1 + 8);
              if (calc_crc != crc) {
                  LOG_ERROR("[UART RX] CRC mismatch: received=0x%04X calculated=0x%04X\r\n", crc, calc_crc);
                  return false;
              }
              LOG_INFO("[UART RX] Frame valid!\r\n");
              *command = buf[2];
              for (int i = 0; i < 8; ++i) payload[i] = buf[3 + i];
              return true;
          }
      }
  }
  LOG_WARN("[UART RX] Timeout waiting for response\r\n");
  if (idx > 0) {
      LOG_WARN("[UART RX] Partial frame: ");
      for (size_t i = 0; i < idx; ++i) LOG_WARN("%02X ", buf[i]);
      LOG_WARN("\r\n");
  }
  return false;
}

void processSlaveStatusResponse(uint8_t blind_number, uint8_t state) {
  if (blind_number > 0 && blind_number <= MAX_SLAVES) {
      bool was_moving = slave_moving[blind_number];
      if (state == STATE_OPENING || state == STATE_CLOSING) {
          slave_moving[blind_number] = true;
      } else {
          slave_moving[blind_number] = false;
          // If the slave was moving and now stopped, keep vBAT on for disengage_time
          if (was_moving) {
              setVBATKeepOn(2000);
          }
      }
  }
}

void scanUARTSlavesAndPublish() {
  ensureVBATOn();
  LOG_INFO("[UART SCAN] vBAT enabled, starting scan for slaves...\r\n");
  detected_slave_count = 0; // clear on scan
  memset(detected_slaves, 0, sizeof(detected_slaves));
  while (Serial1.available()) Serial1.read();
  for (uint8_t addr = BASE_ADDR; addr < BASE_ADDR + MAX_SLAVES; ++addr) {
      LOG_INFO("[UART SCAN] Probing address 0x%02X... ", addr);
      uint8_t dummy_payload[8] = {0};
      LOG_DEBUG("[UART TX] Sending frame: ");
      uint8_t frame[1 + 1 + 1 + 8 + 2 + 1];
      size_t idx = 0;
      frame[idx++] = UART_START_BYTE;
      frame[idx++] = addr;
      frame[idx++] = GET_STATUS;
      for (int i = 0; i < 8; ++i) frame[idx++] = 0;
      uint16_t crc = crc16_ccitt(&frame[1], 1 + 1 + 8);
      frame[idx++] = (crc >> 8) & 0xFF;
      frame[idx++] = crc & 0xFF;
      frame[idx++] = UART_END_BYTE;
      for (size_t i = 0; i < sizeof(frame); ++i) LOG_DEBUG_RAW("%02X ", frame[i]);
      LOG_DEBUG_RAW("\r\n");
      sendUARTCommandToSlave(addr, GET_STATUS, dummy_payload);
      uint8_t resp_cmd = 0, resp_payload[8];
      LOG_DEBUG("[UART RX] Waiting for response: ");
      bool got_response = receiveUARTResponse(addr, &resp_cmd, resp_payload, UART_TIMEOUT_MS); 
      if (got_response) {
          uint8_t state = resp_payload[0];
          uint16_t position = (resp_payload[1] << 8) | resp_payload[2];
          LOG_INFO("[UART RX] State: 0x%02X, Position: %u\r\n", state, position);
          if (position <= 1000) {
              uint8_t blind_number = addr - BASE_ADDR + 1;
              // Add to detected_slaves if not already present
              bool found = false;
              for (uint8_t i = 0; i < detected_slave_count; ++i) {
                  if (detected_slaves[i] == blind_number) { found = true; break; }
              }
              if (!found && detected_slave_count < MAX_SLAVES) {
                  detected_slaves[detected_slave_count++] = blind_number;
              }
              uint16_t battery = 0;
              // --- Update slave_positions ---
              slave_positions[blind_number] = position;
          }
      } else {
          LOG_WARN("[UART RX] No valid response\r\n");
      }
      delay(30); // Give time for next slave
  }
  LOG_INFO("[UART SCAN] Scan complete.\r\n");
}

// UART send command to slave 
bool sendCommandToSlaveUART(uint8_t blind_number, uint8_t command, uint8_t *payload, size_t payload_len) {
  if (blind_number == 0 || blind_number > MAX_SLAVES) return false;
  uint8_t slave_addr = BASE_ADDR + (blind_number - 1);
  ensureVBATOn();
  uint8_t full_payload[8] = {0};
  if (payload && payload_len > 0) memcpy(full_payload, payload, payload_len > 8 ? 8 : payload_len);
  sendUARTCommandToSlave(slave_addr, command, full_payload);
  // Optionally wait for response if needed
  return true;
}

// UART poll slave for status 
void pollSlaveStatusUART(uint8_t blind_number) {
  uint8_t slave_addr = BASE_ADDR + (blind_number - 1);
  uint8_t dummy_payload[8] = {0}; // Use all zeros for GET_STATUS, matching scan
  while (Serial1.available()) Serial1.read();
  sendUARTCommandToSlave(slave_addr, GET_STATUS, dummy_payload);
  uint8_t resp_cmd = 0, resp_payload[8];

  bool got_response = receiveUARTResponse(slave_addr, &resp_cmd, resp_payload, UART_TIMEOUT_MS);
  if (got_response) {
      uint8_t state = resp_payload[0];
      uint16_t position = (resp_payload[1] << 8) | resp_payload[2];
      uint8_t last_move_status = resp_payload[7]; 
      processSlaveStatusResponse(blind_number, state);
      LOG_INFO("Polled slave %u: state=0x%02X, position=%u, Last move Status=%s \r\n", blind_number, state, position, 
        getMoveStatusString(last_move_status));
      // Send the received data to the gateway
      slave_miss_count[blind_number] = 0; // Reset on success
      // --- Update slave_positions ---
      slave_positions[blind_number] = position;
      prepareCommand(BLIND_STATUS, blind_number, state, position, position, last_move_status);
  } else {
      LOG_WARN("No response from slave %u\r\n", blind_number);
      slave_miss_count[blind_number]++;
      if (slave_miss_count[blind_number] >= SLAVE_MISS_THRESHOLD) {
        slave_moving[blind_number] = false;
      }
      // Debug: print any bytes received (even if not a valid frame)
      if (Serial1.available()) {
          LOG_WARN("[UART RX] Bytes in buffer after poll: ");
          while (Serial1.available()) {
              uint8_t b = Serial1.read();
              LOG_WARN("%02X ", b);
          }
          LOG_WARN("\r\n");
      }
  }
}

bool isConfigButtonPressed() {
  // Button pressed if input is LOW (pulled down by output when button is pressed)
  return digitalRead(CONFIG_BTN_INPUT) == LOW;
}

void IRAM_ATTR onConfigButtonPress() {
    unsigned long now = millis();
    if (now - lastInterruptTime > DEBOUNCE_MS) {
        buttonPressed = true;
        buttonPressTime = now;
        lastInterruptTime = now;
    }
}

void handleConfigButton() {
    static bool wasPressed = false;
    static unsigned long pressStart = 0;
    bool pressed = digitalRead(CONFIG_BTN_INPUT) == LOW;
    unsigned long now = millis();
    if (pressed && !wasPressed) {
        // Button just pressed
        pressStart = now;
        buttonHandled = false;
    }
    if (!pressed && wasPressed && !buttonHandled) {
        // Button just released
        unsigned long pressDuration = now - pressStart;
        if (pressDuration >= LONG_PRESS_MS) {
            // Long press: clear config
            prefs.begin("loracfg", false);
            prefs.clear();
            prefs.end();
            buttonHandled = true;
            LOG_INFO("Config cleared from flash (long press)\r\n");
            // Optionally, give feedback (blink LED, etc.)
        } else if (pressDuration >= DEBOUNCE_MS) {
            // Short press: enter config mode
            configButtonInterruptFlag = true;
            buttonHandled = true;
            LOG_INFO("Short press: enter config mode\r\n");
        }
    }
    wasPressed = pressed;
}



// Save config to flash
void saveConfigToFlash() {
prefs.begin("loracfg", false);
prefs.putUInt("gatewayID", config_gatewayID);
prefs.putBytes("aes_key", config_aes_key, 16);
prefs.putBytes("hmacKey", config_hmacKey, 10);
prefs.putUInt("rf_frequency", config_rf_frequency);
prefs.end();
}
void eraseNVSAndReboot() {
    LOG_ERROR("[NVS] Erasing NVS partition and rebooting...\n");
    nvs_flash_erase();
    nvs_flash_init();
    delay(100);
    ESP.restart();
}


void handleSaveConfig(AsyncWebServerRequest *request) {
  if (request->hasParam("gatewayID", true) && request->hasParam("aes_key", true) && request->hasParam("hmacKey", true)) {
    String gwStr = request->getParam("gatewayID", true)->value();
    String aesStr = request->getParam("aes_key", true)->value();
    String hmacStr = request->getParam("hmacKey", true)->value();
    config_gatewayID = strtoul(gwStr.c_str(), nullptr, 16);
    hexToBytes(aesStr.c_str(), config_aes_key, 16);
    hexToBytes(hmacStr.c_str(), config_hmacKey, 10);
    if (request->hasParam("rf_frequency", true))
      config_rf_frequency = strtoul(request->getParam("rf_frequency", true)->value().c_str(), nullptr, 10);
    saveConfigToFlash();
    request->send(200, "text/html", "<html><body>Config saved!<br><a href='/'>Back</a></body></html>");
    // Optionally stop webserver and go to sleep
    configMode = false;
  } else {
    request->send(400, "text/html", "<html><body>Missing fields.<br><a href='/'>Back</a></body></html>");
  }
}

// Load config from flash, return true if found
bool loadConfigFromFlash() {
prefs.begin("loracfg", true);
bool found = prefs.isKey("gatewayID") && prefs.isKey("aes_key") && prefs.isKey("hmacKey");
if (found) {
  config_gatewayID = prefs.getUInt("gatewayID", 0);
  prefs.getBytes("aes_key", config_aes_key, 16);
  prefs.getBytes("hmacKey", config_hmacKey, 10);
  if (prefs.isKey("rf_frequency"))
    config_rf_frequency = prefs.getUInt("rf_frequency", 868100000);
  else
    config_rf_frequency = 868100000;
}
prefs.end();
return found;
}

const char* getMoveStatusString(uint8_t status) {
  switch (status) {
    case MOVE_STATUS_OK: return "OK";
    case MOVE_STATUS_TIMEOUT: return "TIMEOUT";
    case MOVE_STATUS_MOVING: return "MOVING";
    default: return "UNKNOWN";
  }
}
void handleGetPosition(AsyncWebServerRequest *request) {
    char json[64];
    snprintf(json, sizeof(json),
        "{\"raw\":%d,\"percent\":%.1f}",
        cached_raw_position,
        cached_scaled_position / 10.0f
    );
    request->send(200, "application/json", json);
}

void setup() {
  // Record the wakeup time
  wakeup = millis();
  wakeCount++;

  RESET_REASON cpu0WakeupReason = rtc_get_reset_reason(0);
  RESET_REASON cpu1WakeupReason = rtc_get_reset_reason(1);
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);
  pinMode(CONFIG_BTN_OUTPUT, OUTPUT);
  digitalWrite(CONFIG_BTN_OUTPUT, LOW);
  pinMode(CONFIG_BTN_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CONFIG_BTN_INPUT), onConfigButtonPress, FALLING);

  pinMode(RS485_DIR_PIN, OUTPUT);
  RS485_RECEIVE(); // Default to receive mode

  pinMode(LED_PIN, OUTPUT);
  if (ledOnWhileAwake) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.begin(115200);
  if ((cpu0WakeupReason == POWERON_RESET) || (cpu1WakeupReason == POWERON_RESET)) {
    // make it easier to spot in the logs during testing
    LOG_INFO("############################################\r\n");
    LOG_INFO("#                                          #\r\n");
    LOG_INFO("#             Power On Reset               #\r\n");
    LOG_INFO("#                                          #\r\n");
    LOG_INFO("############################################\r\n");
  }
  LOG_INFO("millis(): %lu\r\n", millis());
  LOG_INFO("CPU0 reset reason: ");
  print_reset_reason(cpu0WakeupReason);
  LOG_INFO("CPU1 reset reason: ");
  print_reset_reason(cpu1WakeupReason);
  gettimeofday(&tv_now, NULL);
  int64_t time = (int64_t)tv_now.tv_sec;
  LOG_INFO("Time at wakeup: %lld\r\n", time);
  pinMode(BATTERY_PIN, OUTPUT);
  digitalWrite(BATTERY_PIN, HIGH);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);  // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  adcAttachPin(BATTERY_ADC_PIN);


  setCpuFrequencyMhz(80);

  // Create node ID
  uint8_t deviceMac[8];
  BoardGetUniqueId(deviceMac);
  deviceID += (uint32_t)deviceMac[2];
  deviceID += (uint32_t)deviceMac[3] << 8;
  deviceID += (uint32_t)deviceMac[4] << 16;
  deviceID += (uint32_t)deviceMac[5] << 24;
  LOG_INFO("Device ID %08X using frequency %.1f MHz\r\r\n", deviceID, (double)(config_rf_frequency / 1000000.0));
  if (
    ((cpu0WakeupReason == POWERON_RESET || cpu1WakeupReason == POWERON_RESET) && digitalRead(CONFIG_BTN_INPUT) == LOW)
    || isConfigMissing()
    || isConfigButtonPressed()
    || configButtonInterruptFlag
    ) {
    // Button held during power-on or config missing: start config portal
    LOG_INFO("AP and Webserver started\r\n");
    startConfigAPAndWebserver(actuator);
    configMode = true;
    unsigned long configStartTime = millis();
    while (configMode) {
      delay(100);
      if (millis() - configStartTime > 300000) { // 5 minutes timeout
        LOG_WARN("Config portal timeout, going to sleep\r\n");
        configMode = false;
      }
    }
    goToSleep();
  }

  aes128.setKey(config_aes_key, 16);
  if (otp) delete otp;
  otp = new TOTP(config_hmacKey, 10, 1);
  // Setup the radio
  radioSetup();

  // Initialize UART
  while (!Serial1) { delay(10); }
  if ((cpu0WakeupReason == POWERON_RESET) || (cpu1WakeupReason == POWERON_RESET)) {
    scanUARTSlavesAndPublish();
  }

  actuator.begin();

  LOG_INFO("Setup finished\r\n");
  LOG_INFO("millis(): %lu\r\n", millis());
}

void loop() {
    Radio.IrqProcess(); // Process radio interrupts first
    handleConfigButton();

    if (commandReady) {
        actionCommand();
    }

    // there is something in the buffer and (if timesync requested wait for additional packet)
    if (!loraRXbuffer.isEmpty() && !(timeSync > 0 && buff_size == loraRXbuffer.size())) {
        parsePacket();
    }
    static BlindMotorState lastState = BlindMotorState::IDLE;
    BlindMotorState currentState = actuator.getState();
    if ((lastState == BlindMotorState::MOVING || lastState == BlindMotorState::DISENGAGING) &&
        currentState == BlindMotorState::IDLE) {
        // Move just finished, send final status update
        actuator.reportStatus();
        prepareCommand(BLIND_STATUS, 0, actuator.getCoverState(), actuator.readPositionPercent(), actuator.readPositionRaw(), actuator.getLastMoveStatus());
    }
    lastState = currentState;

    if (actuator.isMotorRunning()) {
      if (millis() - status_update > MOVING_UPDATE_PERIOD) {
        status_update = millis();
        actuator.reportStatus();
        prepareCommand(BLIND_STATUS, 0, actuator.getCoverState(), actuator.readPositionPercent(), actuator.readPositionPercent(), actuator.getLastMoveStatus());
      }
    }
    if (any_slave_moving()) {
      if (millis() - slave_status_update > MOVING_UPDATE_PERIOD) {
        slave_status_update = millis();
        for (uint8_t i = 0; i < detected_slave_count; ++i) {
          uint8_t blind_number = detected_slaves[i];
          if (slave_moving[blind_number]) {
            pollSlaveStatusUART(blind_number);
          }
        }
      }
    }
    if (configPortalActive && millis() - configPortalStartTime > 10 * 60 * 1000) {
        stopConfigAPAndWebserver(); // Your function to stop the web portal
        configPortalActive = false;
    }
    if (millis() - wakeup > MAX_AWAKE_TIME) {
        LOG_WARN("10min timeout\r\n");
        timeSync = 0;
        lastTimeSync = 0;
        goToSleep();
    }

    // Check for LoRa timeout - tx command didnt finish within 20 seconds
    if ((millis() - loraTimeout) > LORA_TIMEOUT && (!txComplete || timeSync > 0)) {
        LOG_WARN("LoRa loop timeout\r\n");
        // LoRa failed, go back to bed
        timeSync = 0;
        lastTimeSync = 0;
        goToSleep();
    }

    // Perform time sync if its not set already. and send periodic updates
    if (loraRXbuffer.isEmpty() && !commandReady && txComplete && !(timeSync > 0) && millis() > 90) {
        gettimeofday(&tv_now, NULL);
        int32_t time = (int32_t)tv_now.tv_sec;
        if (time < 60000000) {  //its not set, but could be second run
          if(time < 15000000) { //its first run since boot, give a random seed for otp
            tv_now.tv_sec = random(15000001, 30000000);
            settimeofday(&tv_now, NULL);
          }
            if (!(lastTimeSync == 0) && time - lastTimeSync < TIME_SYNC_RATE_LIMIT) {
                //its failed recently
            } else {
                performTimeSync();
                return;
            }
        }

        if (time - lastSend > UPDATE_PERIOD && timeSync == 0) {
            lastSend = time;
            slave_reading = true;
            prepareCommand(DEVICE_STATUS);
            prepareCommand(BLIND_STATUS, 0, actuator.getCoverState(), actuator.readPositionPercent(), actuator.readPositionPercent(), actuator.getLastMoveStatus());

            if (detected_slave_count > 0) ensureVBATOn();
            for (uint8_t i = 0; i < detected_slave_count; ++i) {
              pollSlaveStatusUART(detected_slaves[i]);
            }
            slave_reading = false;
            updateVBATState();
            return;
        }
        if (configPortalActive == true || actuator.isMotorRunning() || any_slave_moving() || slave_reading) {
            // If config portal is active, don't go to sleep
            return;
        }
        if (millis() < vbat_keep_on_until) {
          return;
        }
        // nothing left to do..

        goToSleep();
    }
    updateVBATState();
}

void ensureVBATOn() {
  if (!vbat_state) {
      vBAT(true);
      vEXT(true); // Ensure logic shifter is on for RS485
      delay(VBAT_BOOT_MS);
  } else if (!vext_state) {
      vEXT(true); // If vBAT is already on, still ensure vEXT is on
  }
}

void setVBATKeepOn(unsigned long duration_ms) {
  ensureVBATOn();
  unsigned long new_until = millis() + duration_ms;
  if (new_until > vbat_keep_on_until) vbat_keep_on_until = new_until;
}

void updateVBATState() {
  if (!actuator.isMotorRunning() && !any_slave_moving() && !slave_reading && millis() > vbat_keep_on_until) {
      vBAT(false);
  } 
}
// function to enable the mosfet on the battery, normally used for reading battery voltage, but hacked to power the motor driver board with battery voltage not the regulated power.
bool vBAT(bool state) {
  if (state && !vbat_state) {
    // requested on, but current state is off
    digitalWrite(BATTERY_PIN, LOW);
    vbat_state = true;
    LOG_INFO("vBAT Enabled\r\n");
    delay(10);
    return true;
  }
  if (!state && vbat_state && !actuator.isMotorRunning() && !any_slave_moving() && !slave_reading) {  //make sure motor is not running before turning off
    digitalWrite(BATTERY_PIN, HIGH);
    vbat_state = false;
    LOG_INFO("vBAT Disabled\r\n");
    return false;
  }
  return state;
}

uint16_t checkBAT(void) {
  ensureVBATOn();
  int battRaw = analogReadMilliVolts(BATTERY_ADC_PIN);
  int battAvg = battEwma.filter(battRaw);
  uint16_t battCalc = battAvg * 4.900;  // based on 390k and 100k voltage divider

  LOG_INFO("avg voltage at GPIO 01: %d  Battery voltage: %d\r\n", battRaw, battCalc);
  updateVBATState();
  return battCalc;
}

// function to enable the 2nd voltage regulator, used for position sensor and rs485 module
bool vEXT(bool vext_set_state) {
  if (vext_set_state && !vext_state) {
    // requested on, but current state is off
    digitalWrite(VEXT_PIN, LOW);
    vext_state = true;
    LOG_INFO("vEXT Enabled\r\n");
    delay(500);
    return true;
  }
  if (!vext_set_state && vext_state) {
    digitalWrite(VEXT_PIN, HIGH);
    vext_state = false;
    LOG_INFO("vEXT Disabled\r\n");
    return false;
  }
  return vext_set_state;
}

uint8_t checkOTP(uint8_t OTP[3]) {
  
  gettimeofday(&tv_now, NULL);
  int64_t time = (int64_t)tv_now.tv_sec;
  uint32_t intCode = OTP[0] | (OTP[1] << 8) | (OTP[2] << 16);
  if (timeSync > 0) {  //could be using random time set in timeNow, otherwise otp may be predictable when rtc is 0
    if (intCode == (uint32_t)atoi(otp->getCode(timeNow))) return 0xBB;
  }
  for (uint8_t idx = 0; idx < OTP_RANGE; idx++) {
    if (intCode == (uint32_t)atoi(otp->getCode(time + idx))) return idx + 1;
    if (intCode == (uint32_t)atoi(otp->getCode(time - idx))) return (idx + 1) << 4;
  }
  return 0x00;
}


void buildPacket(uint32_t toNodeID, time_t OTPtime, commandPacket commandToSend) {
  start_enc = millis();

  if (fullInit == false) {
    radioInit();
  }
  // wait for the previous packet to finish transmitting otherwise we will overwrite data
  int ctr = 0;
  while (!txComplete && ctr < 50) {
    LOG_WARN("waiting for TX to complete\r\n");
    ctr++;
    delay(100);
  }

  gettimeofday(&tv_now, NULL);
  int32_t time = (int32_t)tv_now.tv_sec;

  char *newCode = otp->getCode(time); // Get the OTP code for the current time instead of OTPtime
  uint32_t intCode = atoi((char *)newCode);
  LOG_INFO("OTP Time: %ld   OTP code: %u\r\n", time, intCode);
  dataToENC.OTP[2] = intCode >> 16;
  dataToENC.OTP[1] = intCode >> 8;
  dataToENC.OTP[0] = intCode >> 0;

  dataToENC.nodeFROMId = deviceID;

  dataToENC.command = commandToSend.command;

  for (int idx = 0; idx < sizeof(commandToSend.payload); idx++) {
    dataToENC.payload[idx] = commandToSend.payload[idx];
  }

  loraTXpacket.nodeToId = toNodeID;

  //encrypt the data payload
  memcpy(decrypted, &dataToENC, sizeof(decrypted));
  aes128.encryptBlock(encrypted, decrypted);  //cypher->output block, plaintext->input block
  memcpy(&loraTXpacket.dataEncrypted, encrypted, sizeof(encrypted));

  LOG_DEBUG("Data To ENC as   HEX values: ");
  char *printData = (char *)&dataToENC;
  for (int idx = 0; idx < sizeof(dataToENC); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("\r\n");
  LOG_DEBUG("Data encrypted   HEX values: ");
  printData = (char *)&loraTXpacket.dataEncrypted;
  for (int idx = 0; idx < sizeof(loraTXpacket.dataEncrypted); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("\r\n");
  LOG_DEBUG("DataTX package   HEX values: ");
  printData = (char *)&loraTXpacket;
  for (int idx = 0; idx < sizeof(loraTXpacket); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("\r\n");
  LOG_INFO("    Packet build time %ldms\r\r\n", (millis() - start_enc));

  txComplete = false;
  loraTXTime = millis();

  // Start sending
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 150);
  // Counter for repeated CAD in case we have many traffic
  cadRepeat = 0;
  Radio.StartCad();

  // Make sure we detect a timeout during sending
  loraTimeout = millis();
}

void prepareCommand(uint8_t prep_command, uint8_t blind_number, uint8_t state, uint16_t position, uint16_t real_position, uint8_t last_move_status) {
  LOG_INFO(" Preparing command Now.. \r\n");
  gettimeofday(&tv_now, NULL);
  int32_t time = (int32_t)tv_now.tv_sec;

  // Declare all variables at the top to avoid jump to case label errors
  uint16_t battV = 0;
  uint16_t awakeSeconds = 0;
  int posAvg = 0;
  uint16_t posCalc = 0;
  uint32_t timeSet = 0;

  switch (prep_command) {
    case ACK:
      LOG_INFO("ACK\r\n");
      commandTX.command = ACK;
      for (int i = 0; i < 8; ++i) {
        commandTX.payload[i] = random(0,255); // Fast, hardware random on ESP32
      }
      buildPacket(config_gatewayID, time, commandTX);
      break;
    case NAK:
      LOG_INFO("NAK\r\n");
      commandTX.command = NAK;
      for (int i = 0; i < 8; ++i) {
        commandTX.payload[i] = random(0,255); // Fast, hardware random on ESP32
      }
      buildPacket(config_gatewayID, time, commandTX);
      break;
    case DEVICE_STATUS:
      // DEVICE_STATUS payload structure for gateway/HA integration:
      // payload[0,1]: Battery voltage (uint16_t, mV)
      // payload[2,3]: Wake count (uint16_t)
      // payload[4,5]: Awake seconds (uint16_t, seconds)
      // payload[6,7]: Reserved (0xDE, 0xF0)
      commandTX.command = DEVICE_STATUS;
      battV = checkBAT();
      LOG_INFO("Command: DEVICE STATUS  Wakeup Count: %u  Total Awake Time: %lu  Battery Voltage: %umV\r\n", wakeCount, awakeTime, battV);

      commandTX.payload[0] = battV >> 8;
      commandTX.payload[1] = battV;
      commandTX.payload[2] = wakeCount >> 8;
      commandTX.payload[3] = wakeCount;
      awakeSeconds = awakeTime / 1000;
      commandTX.payload[4] = (uint16_t)awakeSeconds >> 8;
      commandTX.payload[5] = (uint16_t)awakeSeconds;
      commandTX.payload[6] = 0xDE;
      commandTX.payload[7] = 0xF0;

      buildPacket(config_gatewayID, time, commandTX);

      break;
    case BLIND_STATUS:
      commandTX.command = BLIND_STATUS;
      commandTX.payload[0] = blind_number;
      commandTX.payload[1] = state;
      commandTX.payload[2] = position >> 8;
      commandTX.payload[3] = position & 0xFF;
      commandTX.payload[4] = real_position >> 8;
      commandTX.payload[5] = real_position & 0xFF;
      commandTX.payload[6] = last_move_status;
      commandTX.payload[7] = 0xFF;
      buildPacket(config_gatewayID, time, commandTX);
      break;
    case BLIND_COMMAND:
      LOG_INFO("BLIND COMMAND\r\n");
      posCalc = 0;
      if (blind_number != 0) {
        bool ok = sendCommandToSlaveUART(blind_number, BLIND_COMMAND, command.payload, sizeof(command.payload));
        LOG_INFO("Sent BLIND_COMMAND to slave %u via UART: %s\r\n", blind_number, ok ? "OK" : "FAIL");
        setVBATKeepOn(8000);
        break;
      }
      memcpy(&blind_command, command.payload, sizeof(command.payload));
      posCalc = actuator.readPositionPercent();
      switch (blind_command.set_state) {
        case 0x00:
          targetPos = 0;
          break;
        case 0x01:
          targetPos = 1000;
          break;
        case 0x03:
          targetPos = posCalc;
          actuator.abortMove();
          break;
        case 0x04:
          targetPos = constrain(blind_command.set_position, 0, 100) * 10;
          break;
        default:
          LOG_WARN("ooops.. something messed up\r\n");
      }
      LOG_INFO("Target Position: %.1f%% Current Position: %.1f%% \r\n", targetPos / 10.0f, posCalc / 10.0f);
        actuator.commandMove(targetPos);
      break;
    case SET_PARAMETER:
      LOG_INFO("SET PARAMETER\r\n");
      memcpy(&blind_parameters, command.payload, sizeof(command.payload));
      
      break;
    case GET_PARAMETER:
      LOG_INFO("GET_PARAMETER\r\n");
      break;
    case SET_LIMIT:
      LOG_INFO("SET LIMIT\r\n");
      break;
    case GET_LIMIT:
      LOG_INFO("GET LIMIT\r\n");
      break;
    case TIME_SYNC_COMMAND:
      timeSet += (uint32_t)command.payload[0];
      timeSet += (uint32_t)command.payload[1] << 8;
      timeSet += (uint32_t)command.payload[2] << 16;
      timeSet += (uint32_t)command.payload[3] << 24;
      LOG_INFO("Time sync data received, time set to: %u\r\n", timeSet);
      tv_now.tv_sec = timeSet;
      settimeofday(&tv_now, NULL);
      lastTimeSync = timeSet;
      lastSend = timeSet - UPDATE_PERIOD;
      break;
    default:
      LOG_WARN("UNKNOWN COMMAND!\r\n");
  }
  return;
}

void actionCommand(void) {
  LOG_INFO(" Actioning command Now.. \r\n");
  // Declare all variables at the top to avoid jump to case label errors
  uint16_t battVRX = 0;
  uint16_t wakeCountRX = 0;
  uint16_t awakeTimeRX = 0;
  int dir = 0;
  int runtime = 0;
  uint16_t calcPos = 0;
  uint8_t blind_number = 0;
  uint32_t timeSet = 0;

  switch (command.command) {
    case NAK:
      LOG_INFO("NAK\r\n");
      performTimeSync();
      break;
    case DEVICE_STATUS:
      prepareCommand(DEVICE_STATUS);
      break;
    case BLIND_STATUS: {
      LOG_INFO("BLIND STATUS poll received\r\n");
      uint8_t requested_blind = command.payload[0];
      if (requested_blind == 0) {
        // Master blind: reply with master status
        prepareCommand(
          BLIND_STATUS,
          0, // blind_number 0 = master
          actuator.getCoverState(),
          actuator.readPositionPercent(),
          actuator.readPositionRaw(),
          actuator.getLastMoveStatus()
        );
      } else if (requested_blind > 0 && requested_blind <= MAX_SLAVES) {
        slave_reading = true;
        ensureVBATOn();
        pollSlaveStatusUART(requested_blind);
        slave_reading = false;
        updateVBATState();
      }
      break;
    }
    case BLIND_COMMAND:
      LOG_INFO("BLIND COMMAND\r\n");
      blind_number = command.payload[0];
      if (blind_number != 0) {
        slave_moving[blind_number] = true;
        bool ok = sendCommandToSlaveUART(blind_number, BLIND_COMMAND, command.payload, sizeof(command.payload));
        LOG_INFO("Sent BLIND_COMMAND to slave %u via UART: %s\r\n", blind_number, ok ? "OK" : "FAIL");
        setVBATKeepOn(8000);
        break;
      }
      memcpy(&blind_command, command.payload, sizeof(command.payload));
      ensureVBATOn();
      setVBATKeepOn(8000);
      calcPos = actuator.readPositionPercent();
      switch (blind_command.set_state) {
        case 0x00:
          targetPos = 0;
          break;
        case 0x01:
          targetPos = 1000;
          break;
        case 0x03:
          targetPos = calcPos;
          actuator.abortMove();
          break;
        case 0x04:
          targetPos = constrain(blind_command.set_position, 0, 100) * 10;
          break;
        default:
          LOG_WARN("ooops.. something messed up\r\n");
      }
      LOG_INFO("Target Position: %.1f%% Current Position: %.1f%% \r\n", targetPos / 10.0f, calcPos / 10.0f);
        actuator.commandMove(targetPos);
      break;
    case SET_PARAMETER:
      LOG_INFO("SET PARAMETER\r\n");
      memcpy(&blind_parameters, command.payload, sizeof(command.payload));
      
      break;
    case GET_PARAMETER:
      LOG_INFO("GET_PARAMETER\r\n");
      break;
    case SET_LIMIT:
      LOG_INFO("SET LIMIT\r\n");
      break;
    case GET_LIMIT:
      LOG_INFO("GET LIMIT\r\n");
      break;
    case UPDATE_FIRMWARE:
      LOG_INFO("Received OTA/Web Portal Open Command\r\n");
      blind_number = command.payload[0];
      if (blind_number != 0) {
        bool ok = sendCommandToSlaveUART(blind_number, UPDATE_SLAVE, command.payload, sizeof(command.payload));
        LOG_INFO("Sent UPDATE_SLAVE to slave %u via UART: %s\r\n", blind_number, ok ? "OK" : "FAIL");
        setVBATKeepOn(5 * 60 * 1000);
        break;
      } else {
        setVBATKeepOn(600000);
        startConfigAPAndWebserver(actuator); // Pass actuator to ensure web portal endpoints are registered
        //actuator.startCalibrationSequence(); // TEMP: Trigger calibration when web portal starts
      }
      configPortalActive = true;
      configPortalStartTime = millis();
      break;
    case TIME_SYNC_COMMAND:
      timeSet += (uint32_t)command.payload[0];
      timeSet += (uint32_t)command.payload[1] << 8;
      timeSet += (uint32_t)command.payload[2] << 16;
      timeSet += (uint32_t)command.payload[3] << 24;
      LOG_INFO("Time sync data received, time set to: %u\r\n", timeSet);
      tv_now.tv_sec = timeSet;
      settimeofday(&tv_now, NULL);
      lastTimeSync = timeSet;
      lastSend = timeSet - UPDATE_PERIOD;
      break;
    default:
      LOG_WARN("UNKNOWN COMMAND!\r\n");
  }

  commandReady = false;
}

void parsePacket(void) {
  // catch the voided packet and return
  if (loraRXbuffer.first().nodeToId != deviceID) {
    loraRXbuffer.shift();
    LOG_WARN("Invalid Packet or not for us\r\n");
    return;
  }

  //decrypt the message
  memcpy(encrypted, loraRXbuffer.first().dataEncrypted, sizeof(encrypted));
  aes128.decryptBlock(decrypted, encrypted);
  memcpy(&dataDEC, decrypted, sizeof(decrypted));

  //verify OTP is correct
  uint8_t OTPresult = checkOTP(dataDEC.OTP);

  LOG_DEBUG("Decoded package HEX values: ");
  char *printData = (char *)&dataDEC;
  for (int idx = 0; idx < sizeof(dataDEC); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("\r\n");
  // print decoded packet
  LOG_DEBUG("OTP: ");
  printData = (char *)&dataDEC.OTP;
  for (int idx = 0; idx < sizeof(dataDEC.OTP); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("   OTP result: %02X   Command: ", OTPresult);
  printData = (char *)&dataDEC.command;
  for (int idx = 0; idx < sizeof(dataDEC.command); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("   Payload: ");
  printData = (char *)&dataDEC.payload;
  for (int idx = 0; idx < sizeof(dataDEC.payload); idx++) {
    LOG_DEBUG_RAW("%02X ", printData[idx]);
  }
  LOG_DEBUG_RAW("\r\n");

  if (OTPresult == 0x00) {  //OTP failed and needs to sync time with server
    LOG_WARN(" OTP Failied verification \r\n");
    prepareCommand(ACK);
    msgInBuffer = true;
    performTimeSync();
    return;

  } else {  //OTP is valid
    if (timeSync > 0) {
      if ((uint8_t)dataDEC.command == TIME_SYNC_COMMAND) {
        timeSync = 0;
      } else {
        // OTP was OK and command was not setting the time, so just ignore the time sync going forward
        LOG_WARN("Was expecting a timeSync but received a valid normal command\r\n");
        timeSync = 0;
      }  // end of command == time sync command
    }    //end of timesync > 0 check

    // at this point a command and payload are available in &dataDEC and OTP was verified OK
    if (dataDEC.nodeFROMId == config_gatewayID) {
      LOG_INFO("Message was from gateway\r\n");
    } else {
      LOG_INFO("Message was from NodeID: ");
      printData = (char *)&dataDEC.nodeFROMId;
      for (int idx = 0; idx < sizeof(dataDEC.nodeFROMId); idx++) {
        LOG_INFO_RAW("%02X ", printData[idx]);
      }
      LOG_INFO_RAW("\r\n");
    }
    
    command.command = dataDEC.command;
    for (int idx = 0; idx < sizeof(dataDEC.payload); idx++) {
      command.payload[idx] = dataDEC.payload[idx];
    }

    commandReady = true;
    loraRXbuffer.shift();
    prepareCommand(ACK);
  }  //end of OTP result == 00
}

void performTimeSync(void) {

  gettimeofday(&tv_now, NULL);
  int32_t time = (int32_t)tv_now.tv_sec;

  if (!(lastTimeSync == 0) && time - lastTimeSync < TIME_SYNC_RATE_LIMIT) {
    if (msgInBuffer){
      msgInBuffer = false;
      loraRXbuffer.shift();
      LOG_WARN(" OTP Failed but time was recently set, bin the message \r\n");
    } else {
      LOG_WARN(" Time sync rate limit exceeded, time set less than 5seconds ago \r\n");
    }
    return;
  }

  if (timeSync <= TIME_SYNC_RETRIES) {
    buff_size = loraRXbuffer.size();  // save the current buffer size
    timeSync++;
    lastTimeSyncRequest = millis();
    LOG_INFO(" Attempting to sync time with gateway, try %d\r\n", timeSync);

    timeNow = (uint32_t)tv_now.tv_sec;
    commandTX.command = TIME_SYNC_COMMAND;
    commandTX.payload[3] = timeNow >> 24;
    commandTX.payload[2] = timeNow >> 16;
    commandTX.payload[1] = timeNow >> 8;
    commandTX.payload[0] = timeNow;
    commandTX.payload[7] = timeNow >> 24;
    commandTX.payload[6] = timeNow >> 16;
    commandTX.payload[5] = timeNow >> 8;
    commandTX.payload[4] = timeNow;

    buildPacket(config_gatewayID, time, commandTX);  //build and send the packet
    return;
  } else {
    //failed to sync time
    timeSync = 0;
    lastTimeSync = time;
    if (msgInBuffer){
      msgInBuffer = false;
      loraRXbuffer.shift();
      LOG_WARN(" OTP Failed to sync 3 times, discarding message \r\n");
    } else {
      LOG_WARN(" Time sync failed 3 times \r\n");
    }
    return;
  }

}  // end of performTimeSync



/*****************************************************************************************************************

Radio Setup, Sleep and Callbacks section

*****************************************************************************************************************/

// setup with re-init routine to be able to rx after sleeping without losing data
void radioSetup(void) {
  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;          // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET;  // LORA RESET
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;      // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;    // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;    // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1;  // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;    // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;    // LORA SPI MOSI
  hwConfig.RADIO_TXEN = RADIO_TXEN;          // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = RADIO_RXEN;          // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = true;       // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = true;             // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;      // Only Insight ISP4520 module uses DIO3 as antenna control

  RESET_REASON cpu0WakeupReason = rtc_get_reset_reason(0);
  RESET_REASON cpu1WakeupReason = rtc_get_reset_reason(1);
  if ((cpu0WakeupReason == DEEPSLEEP_RESET) || (cpu1WakeupReason == DEEPSLEEP_RESET)) {
    // Wake up reason was a DEEPSLEEP_RESET, which means we were woke up by the SX126x
    LOG_INFO("Starting lora_hardware_re_init\r\n");
    lora_hardware_re_init(hwConfig);
  } else {
    // Other wake up reasons mean we need to do a complete initialization of the SX126x
    LOG_INFO("Starting lora_hardware_init\r\n");
    lora_hardware_init(hwConfig);
  }

  // Initialize the Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = OnCadDone;

  if ((cpu0WakeupReason == DEEPSLEEP_RESET) || (cpu1WakeupReason == DEEPSLEEP_RESET)) {
    // Wake up reason was a DEEPSLEEP_RESET, just re-initialize the callbacks
    LOG_INFO("Checking for event after deep sleep wakeup\r\n");
    Radio.ReInit(&RadioEvents);
    Radio.IrqProcessAfterDeepSleep();
  } else {
    radioInit();
  }
}

// full init routine to be able to tx after sleeping
void radioInit(void) {
  LOG_INFO("Fully reinitialize the Radio now\r\n");
  // Something happens when the device goes to deep sleep, and cant transmit after. doing a full init after receiving the packet does the trick
  fullInit = true;
  Radio.Init(&RadioEvents);
  // Use selected frequency from config
  Radio.SetChannel(config_rf_frequency);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.Rx(0);
}

/**
 * Put Radio into low power listen mode and ESP32 into deep-sleep mode
 */

void goToSleep(void) {
  LOG_INFO("Going to sleep...\r\n");
  vEXT(false); // Turn off logic shifter
  // Start waiting for data package
  actuator.prepareForDeepSleep();
  Radio.Standby();
  SX126xSetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

  // Calculate Rx and Sleep periods in terms of 15.625 us
  // Example: desired_rx_time_ms = 30, desired_sleep_time_ms = 60
  const uint32_t desired_rx_time_ms = 90000; 
  const uint32_t desired_sleep_time_ms = 60; 

  uint32_t rx_param = (desired_rx_time_ms * 1000000) / 15625; // More precise calculation
  uint32_t sleep_param = (desired_sleep_time_ms * 1000000) / 15625; // More precise calculation

  LOG_INFO("SetRxDutyCycle with Rx: %lu ms (%lu units), Sleep: %lu ms (%lu units)\r\n", desired_rx_time_ms, rx_param, desired_sleep_time_ms, sleep_param);
  Radio.SetRxDutyCycle(rx_param, sleep_param);

  LOG_INFO("Awake for %ldms\r\r\n", (millis() - wakeup));
  LOG_INFO("Start sleeping\r\n");
  awakeTime += (millis() - wakeup);
  // Make sure the DIO1, RESET and NSS GPIOs are hold on required levels during deep sleep
  rtc_gpio_pulldown_en((gpio_num_t)PIN_LORA_DIO_1);
  rtc_gpio_pullup_en((gpio_num_t)PIN_LORA_RESET);
  rtc_gpio_pullup_en((gpio_num_t)PIN_LORA_NSS);
  // Setup deep sleep with wakeup by external source and periodic wakeup
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_LORA_DIO_1, RISING);

  if (ledOnWhileAwake) {
    digitalWrite(LED_PIN, LOW); // Turn off LED before sleep
  }
  
  LOG_INFO("Entering deep sleep.\r\n");
  Serial.flush(); // Ensure all serial messages are sent
  esp_deep_sleep_start();
}

/**@brief Function to be executed on Radio Rx Done event */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    LOG_INFO("Receive finished\r\n");
    char debugOutput[1024];
    for (int idx = 0; idx < size; idx++) {
      sprintf(&debugOutput[(idx * 2)], "%02X", payload[idx]);
    }
    LOG_INFO("Data: %s\r\r\n", debugOutput);
    if (size == sizeof(tmploraRXpacket)) {  //correct size
      memcpy((void *)&tmploraRXpacket, payload, size);
      if (tmploraRXpacket.nodeToId != deviceID) {
        LOG_WARN("packet dropped, not for us..\r\n");
        return;
      }
      if (timeSync == 0) {  //normally add to the tail, but if waiting on a timeSync add it to the head
        loraRXbuffer.push(data::loraRXpacket{ tmploraRXpacket.nodeToId, tmploraRXpacket.dataEncrypted[0], tmploraRXpacket.dataEncrypted[1], tmploraRXpacket.dataEncrypted[2], tmploraRXpacket.dataEncrypted[3], tmploraRXpacket.dataEncrypted[4], tmploraRXpacket.dataEncrypted[5], tmploraRXpacket.dataEncrypted[6], tmploraRXpacket.dataEncrypted[7], tmploraRXpacket.dataEncrypted[8], tmploraRXpacket.dataEncrypted[9], tmploraRXpacket.dataEncrypted[10], tmploraRXpacket.dataEncrypted[11], tmploraRXpacket.dataEncrypted[12], tmploraRXpacket.dataEncrypted[13], tmploraRXpacket.dataEncrypted[14], tmploraRXpacket.dataEncrypted[15] });
      } else {
        loraRXbuffer.unshift(data::loraRXpacket{ tmploraRXpacket.nodeToId, tmploraRXpacket.dataEncrypted[0], tmploraRXpacket.dataEncrypted[1], tmploraRXpacket.dataEncrypted[2], tmploraRXpacket.dataEncrypted[3], tmploraRXpacket.dataEncrypted[4], tmploraRXpacket.dataEncrypted[5], tmploraRXpacket.dataEncrypted[6], tmploraRXpacket.dataEncrypted[7], tmploraRXpacket.dataEncrypted[8], tmploraRXpacket.dataEncrypted[9], tmploraRXpacket.dataEncrypted[10], tmploraRXpacket.dataEncrypted[11], tmploraRXpacket.dataEncrypted[12], tmploraRXpacket.dataEncrypted[13], tmploraRXpacket.dataEncrypted[14], tmploraRXpacket.dataEncrypted[15] });
      }
    } else {
      LOG_WARN("packet dropped, wrong size.. \r\n");
    }
    // LoRa receive finished, go back to loop
  }

  /**@brief Function to be executed on Radio Rx Timeout event */
  void OnRxTimeout(void) {
    LOG_WARN("Receive timeout - Radio Status: %d\r\n", Radio.GetStatus());
  }
  
  /**@brief Function to be executed on Radio Rx Error event */
  void OnRxError(void) {
    LOG_ERROR("Receive error - Radio Status: %d\r\n", Radio.GetStatus());
  }
  
  /**@brief Function to be executed on Radio Tx Done event */
  void OnTxDone(void) {
    LOG_INFO("Transmit finished - Radio Status: %d\r\n", Radio.GetStatus());
    LOG_INFO("    Packet send time %ldms\r\n", (millis() - loraTXTime));
    txComplete = true;
    Radio.Rx(0);
    LOG_INFO("Set RX mode - Radio Status: %d\r\n", Radio.GetStatus());
  }
  
  /**@brief Function to be executed on Radio Tx Timeout event */
  void OnTxTimeout(void) {
    LOG_ERROR("Transmit timeout - Radio Status: %d\r\n", Radio.GetStatus());
    txComplete = true;
  }
  
  /**@Function for channel activity detection and TX packet*/
  void OnCadDone(bool cadResult) {
    cadRepeat++;
    Radio.Standby();
    if (cadResult) {
      LOG_WARN("CAD returned channel busy %d times\r\r\n", cadRepeat);
      if (cadRepeat < CAD_RETRY) {
        // Retry CAD
        Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 150);
        Radio.StartCad();
      } else {
        LOG_WARN("############# CAD failed too many times.Gave up afer %ldms\r\r\n", (millis() - loraTXTime));
        txComplete = true;
      }
    } else {
      LOG_INFO("CAD returned channel free after %d times\r\r\n", cadRepeat);
      // Send data
      Radio.Send((uint8_t *)&loraTXpacket, sizeof(loraTXpacket));
    }
  }
  
  /* Print the reset reason. */
  void print_reset_reason(RESET_REASON reason) {
    switch (reason) {
      case 1:
        LOG_INFO("POWERON_RESET\r\n");
        break; /**<1, Vbat power on reset*/
      case 3:
        LOG_INFO("SW_RESET\r\n");
        break; /**<3, Software reset digital core*/
      case 4:
        LOG_INFO("OWDT_RESET\r\n");
        break; /**<4, Legacy watch dog reset digital core*/
      case 5:
        LOG_INFO("DEEPSLEEP_RESET\r\n");
        break; /**<5, Deep Sleep reset digital core*/
      case 6:
        LOG_INFO("SDIO_RESET\r\n");
        break; /**<6, Reset by SLC module, reset digital core*/
      case 7:
        LOG_INFO("TG0WDT_SYS_RESET\r\n");
        break; /**<7, Timer Group0 Watch dog reset digital core*/
      case 8:
        LOG_INFO("TG1WDT_SYS_RESET\r\n");
        break; /**<8, Timer Group1 Watch dog reset digital core*/
      case 9:
        LOG_INFO("RTCWDT_SYS_RESET\r\n");
        break; /**<9, RTC Watch dog Reset digital core*/
      case 10:
        LOG_INFO("INTRUSION_RESET\r\n");
        break; /**<10, Instrusion tested to reset CPU*/
      case 11:
        LOG_INFO("TGWDT_CPU_RESET\r\n");
        break; /**<11, Time Group reset CPU*/
      case 12:
        LOG_INFO("SW_CPU_RESET\r\n");
        break; /**<12, Software reset CPU*/
      case 13:
        LOG_INFO("RTCWDT_CPU_RESET\r\n");
        break; /**<13, RTC Watch dog Reset CPU*/
      case 14:
        LOG_INFO("EXT_CPU_RESET\r\n");
        break; /**<14, for APP CPU, reseted by PRO CPU*/
      case 15:
        LOG_INFO("RTCWDT_BROWN_OUT_RESET\r\n");
        break; /**<15, Reset when the vdd voltage is not stable*/
      case 16:
        LOG_INFO("RTCWDT_RTC_RESET\r\n");
        break; /**<16, RTC Watch dog reset digital core and rtc module*/
      default:
        LOG_INFO("NO_MEAN\r\n");
    }
  }



