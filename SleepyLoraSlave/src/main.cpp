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
#define BASE_ADDR 0x20 // base address for slaves
uint8_t rs485_addr = 0x20; // Default address, can be changed via web portal

// Command codes (should match master's Command_Register.h)
#define BLIND_COMMAND 0x03
#define GET_STATUS    0x08
#define UPDATE_FIRMWARE 0x09


// State definitions (should match master's enum)
#define STATE_CLOSING  0x01
#define STATE_OPENING  0x02
#define STATE_CLOSED   0x03
#define STATE_OPEN     0x04

volatile uint8_t last_command = 0;
volatile uint8_t last_payload[8];
volatile bool command_received = false;
AsyncWebServer server(80);
Ticker webserverTimeoutTicker;
bool configPortalActive = false;
unsigned long configPortalStartTime = 0;

#define POS_ALPHA 0.7
#define CLOSED_RAW 5 //19
#define OPEN_RAW 1600 //860
uint8_t blind_state = STATE_CLOSED;
uint16_t blind_position = 0; // 0-1000
uint16_t targetPos;  // position in % x 10
Ewma posEwma(POS_ALPHA);

time_t disengage_time = 1000;
time_t stroke_time = 14000; // time for full stroke
time_t motor_start_time;
time_t motor_run_time;
bool motor_running;
bool motor_engaged;
bool last_motor_dir;
uint8_t last_move_status; // New variable to track last move status

Preferences prefs;

void run_motor(uint8_t dir, int runtime);
void disengage_motor(void);
uint16_t readPosition(void);
int readPositionRaw(void);
void update_motor(void);
void stop_motor(void);
void saveLastMoveStatusToFlash(uint8_t status);
uint8_t loadLastMoveStatusFromFlash();
void stopConfigAPAndWebserver();
void startConfigAPAndWebserver();
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

uint16_t readPosition(void) {
  int posAvg = readPositionRaw();
  long posCalc = map(posAvg, CLOSED_RAW, OPEN_RAW, 0, 1000);  // based on 390k and 100k voltage divider
  uint16_t posConst = constrain(posCalc, 0, 1000);
  LOG_DEBUG("Blind Position: %u\r\n", posConst);
  return posConst;
}

int readPositionRaw(void) {
  int posAvg;

  int posRaw = analogReadMilliVolts(POSITION_PIN);
  posAvg = posEwma.filter(posRaw);

  return posAvg;
}

// set a direction and a max/estimated runtime, motor stop may be called once position reached
void run_motor(uint8_t dir, int runtime) {
  uint16_t calcPos = readPosition();
  LOG_DEBUG("Current Position: %.1f%% Target Position: %.1f%%\r\n", calcPos / 10.0f, targetPos / 10.0f);
  if (dir == 0x01) {
    digitalWrite(IN_A_PIN, HIGH);
    digitalWrite(IN_B_PIN, LOW);
    LOG_DEBUG("Motor Started Retract\r\n");
    if (motor_engaged) {
      blind_state = STATE_CLOSING;
    }
    last_motor_dir = true;
  }
  if (dir == 0x02) {
    digitalWrite(IN_A_PIN, LOW);
    digitalWrite(IN_B_PIN, HIGH);
    LOG_DEBUG("Motor Started Extend\r\n");
    if (motor_engaged) {
      blind_state = STATE_OPENING;
    }
    last_motor_dir = false;
  }
  motor_running = true;
  motor_start_time = millis();
  motor_run_time = runtime;
  LOG_DEBUG("Motor DIR %s\r\n", last_motor_dir ? "CW" : "CCW");
  return;
}

void disengage_motor(void) {
  last_motor_dir = !last_motor_dir;
  LOG_DEBUG("Motor Disengage\r\n");
  motor_engaged = false;
  run_motor(last_motor_dir ? 0x01 : 0x02, disengage_time);
  return;
}

void update_motor(void) {
  if (motor_running) {
    uint16_t calcPos = readPosition();
    if (motor_engaged && (abs(targetPos - calcPos) < 1 || (!last_motor_dir && calcPos > targetPos) || (last_motor_dir && calcPos < targetPos))) {
      LOG_DEBUG("Position achieved\r\n");
      LOG_DEBUG("Current Position: %.1f%% Target Position: %.1f%% Time taken to move: %lu\r\n", calcPos / 10.0f, targetPos / 10.0f, millis() - motor_start_time);
      if (calcPos > 10) {
        blind_state = STATE_OPEN;
      } else {
        blind_state = STATE_CLOSED;
      }
      last_move_status = 0x01; // MOVE_STATUS_OK
      saveLastMoveStatusToFlash(last_move_status);
      disengage_motor();
      return;
    }
    if (millis() - motor_start_time > motor_run_time) {
      if (motor_engaged) {
        LOG_WARN("Timeout before position achieved\r\n");
        if (calcPos > 10) {
          blind_state = STATE_OPEN;
        } else {
          blind_state = STATE_CLOSED;
        }
        last_move_status = 0x02; // MOVE_STATUS_TIMEOUT
        saveLastMoveStatusToFlash(last_move_status);
        disengage_motor();
        return;
      }
      stop_motor();
    }
  }
  return;
}

void stop_motor(void) {
  digitalWrite(IN_A_PIN, LOW);
  digitalWrite(IN_B_PIN, LOW);
  motor_running = false;
  return;
}

void saveLastMoveStatusToFlash(uint8_t status) {
    prefs.begin("blindcfg", false);
    prefs.putUChar("lms", status); // key shortened to fit NVS requirements
    prefs.end();
}

uint8_t loadLastMoveStatusFromFlash() {
    prefs.begin("blindcfg", true);
    uint8_t status = prefs.getUChar("lms", 0); // key shortened to fit NVS requirements
    prefs.end();
    return status;
}

void handleConfigPage(AsyncWebServerRequest *request) {
    char html[4096];
    snprintf(html, sizeof(html),
        "<html><body style='font-family:monospace; background:#f8f9fa; margin:0; padding:0;'>"
        "<div style='max-width:480px;margin:32px auto 0 auto;padding:24px 24px 16px 24px;background:#fff;border-radius:10px;box-shadow:0 2px 12px #0001;'>"
        "<pre style='font-family:monospace; font-size:16px; text-align:center; margin:0 0 12px 0;'>\r\n"
        " ____  _                       _          ____       \r\n"
        "/ ___|| | ___  ___ _ __  _   _| |    ___ |  _ \\ __ _ \r\n"
        "\\___ \\| |/ _ \\/ _ \\ '_ \\| | | | |   / _ \\| |_) / _` |\r\n"
        " ___) | |  __/  __/ |_) | |_| | |__| (_) |  _ < (_| |\r\n"
        "|____/|_|\\___|\\___| .__/ \\__, |_____\\___/|_| \\_\\__,_|\r\n"
        "                  |_|    |___/                       \r\n"
        "</pre>"
        "<h2 style='text-align:center;margin:0 0 18px 0;'>SleepyLora Slave Config</h2>"
        "<form method='POST' action='/save'>"
        "<label>Slave Number:</label><br>"
        "<select name='slave_number' style='width:100%%;padding:6px;margin-bottom:10px;'>"
        "<option value='1'%s>1</option>"
        "<option value='2'%s>2</option>"
        "<option value='3'%s>3</option>"
        "<option value='4'%s>4</option>"
        "<option value='5'%s>5</option>"
        "<option value='6'%s>6</option>"
        "<option value='7'%s>7</option>"
        "<option value='8'%s>8</option>"
        "<option value='9'%s>9</option>"
        "<option value='10'%s>10</option>"
        "<option value='11'%s>11</option>"
        "<option value='12'%s>12</option>"
        "</select><br>"
        "<button type='submit' style='background:#28a745;color:#fff;font-size:1.2em;padding:10px 32px;border:none;border-radius:6px;cursor:pointer;margin-top:10px;width:100%%;'>Save</button>"
        "</form>"
        "<div style='margin-top:18px; text-align:center;'>"
        "<a href='/upload' style='color:#007bff; font-size:1.1em; text-decoration:underline;'>Update Firmware from File</a>"
        "</div>"
        "<form method='POST' action='/close' style='margin-top:18px; text-align:center;'>"
        "<button type='submit' style='background:#dc3545;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;'>Close Web Portal & Sleep</button>"
        "</form>"
        "</div></body></html>",
        (slave_number == 1 ? " selected" : ""),
        (slave_number == 2 ? " selected" : ""),
        (slave_number == 3 ? " selected" : ""),
        (slave_number == 4 ? " selected" : ""),
        (slave_number == 5 ? " selected" : ""),
        (slave_number == 6 ? " selected" : ""),
        (slave_number == 7 ? " selected" : ""),
        (slave_number == 8 ? " selected" : ""),
        (slave_number == 9 ? " selected" : ""),
        (slave_number == 10 ? " selected" : ""),
        (slave_number == 11 ? " selected" : ""),
        (slave_number == 12 ? " selected" : "")
    );
    request->send(200, "text/html", html);
}

void startConfigAPAndWebserver() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("SleepyLoRaSlave", "blind1234");
    server.on("/", HTTP_GET, handleConfigPage);
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
        bool shouldReboot = !Update.hasError();
        LOG_INFO("[OTA] Update finished. Success: %s\n", shouldReboot ? "YES" : "NO");
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
        response->addHeader("Connection", "close");
        request->send(response);
        if (shouldReboot) {
            LOG_INFO("[OTA] Rebooting device...\n");
            delay(100);
            ESP.restart();
        }
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!index) {
            LOG_INFO("[OTA] Starting update: %s\n", filename.c_str());
            Update.begin(UPDATE_SIZE_UNKNOWN);
        }
        if (Update.write(data, len) != len) {
            LOG_ERROR("[OTA] Update write failed at index %u\n", index);
        } else {
            LOG_DEBUG("[OTA] Written %u bytes at index %u\n", len, index);
        }
        if (final) {
            if (Update.end(true)) {
                LOG_INFO("[OTA] Update successful. Total size: %u bytes\n", index + len);
            } else {
                LOG_ERROR("[OTA] Update failed. Error: %s\n", Update.errorString());
            }
        }
    });
    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html",
            "<html><body style='font-family:monospace; background:#f8f9fa; margin:0; padding:0;'>"
            "<div style='max-width:480px;margin:32px auto 0 auto;padding:24px 24px 16px 24px;background:#fff;border-radius:10px;box-shadow:0 2px 12px #0001;'>"
            "<pre style='font-family:monospace; font-size:16px; text-align:center; margin:0 0 12px 0;'>\r\n"
            " ____  _                       _          ____       \r\n"
            "/ ___|| | ___  ___ _ __  _   _| |    ___ |  _ \\ __ _ \r\n"
            "\\___ \\| |/ _ \\/ _ \\ '_ \\| | | | |   / _ \\| |_) / _` |\r\n"
            " ___) | |  __/  __/ |_) | |_| | |__| (_) |  _ < (_| |\r\n"
            "|____/|_|\\___|\\___| .__/ \\__, |_____\\___/|_| \\_\\__,_|\r\n"
            "                  |_|    |___/                       \r\n"
            "</pre>"
            "<h2 style='text-align:center;margin:0 0 18px 0;'>SleepyLora Slave Firmware Update</h2>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update' style='margin-bottom:12px;'>"
            "<br><input type='submit' value='Update' style='background:#007bff;color:#fff;padding:6px 18px;border:none;border-radius:5px;cursor:pointer;'>"
            "</form>"
            "<div style='margin-top:18px; text-align:center;'>"
            "<a href='/' style='color:#007bff; font-size:1.1em; text-decoration:underline;'>Back to Config</a>"
            "</div>"
            "<form method='POST' action='/close' style='margin-top:18px; text-align:center;'>"
            "<button type='submit' style='background:#dc3545;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;'>Close Web Portal & Sleep</button>"
            "</form>"
            "</div></body></html>"
        );
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("slave_number", true)) {
            uint8_t new_slave_number = request->getParam("slave_number", true)->value().toInt();
            if (new_slave_number < 1 || new_slave_number > 12) {
                request->send(400, "text/html", "<html><body>Invalid slave number!<br><a href='/'>Back</a></body></html>");
                return;
            }
            uint8_t candidate_addr = BASE_ADDR + (new_slave_number - 1);
            if (isAddressInUse(candidate_addr)) {
                request->send(400, "text/html", "<html><body>Address already in use by another device!<br><a href='/'>Back</a></body></html>");
                return;
            }
            slave_number = new_slave_number;
            rs485_addr = candidate_addr;
            // Save to flash
            prefs.begin("slavecfg", false);
            prefs.putUChar("slave_number", slave_number);
            prefs.end();
            request->send(200, "text/html", "<html><body>Slave number saved!<br><a href='/'>Back</a></body></html>");
        } else {
            request->send(400, "text/html", "<html><body>Missing slave number.<br><a href='/'>Back</a></body></html>");
        }
    });

    server.on("/close", HTTP_POST, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", "<html><body>Web portal closed. Device will sleep.<br><a href='/'>Back</a></body></html>");
        configPortalActive = false;
        webserverTimeoutTicker.detach();
        // Defer shutdown to allow response to be sent
        static Ticker shutdownTicker;
        shutdownTicker.once_ms(300, [](){
            stopConfigAPAndWebserver();
        });
    });

    server.begin();
    configPortalActive = true;
    configPortalStartTime = millis();
    webserverTimeoutTicker.once_ms(300000, [](){
        stopConfigAPAndWebserver();

    });
}

void stopConfigAPAndWebserver() {
    server.end();
    WiFi.mode(WIFI_OFF);
    configPortalActive = false;
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
    for (size_t i = 0; i < sizeof(frame); ++i) LOG_DEBUG_RAW("%02X ", frame[i]);
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
    pinMode(IN_A_PIN, OUTPUT);
    pinMode(IN_B_PIN, OUTPUT);
    digitalWrite(IN_A_PIN, LOW);
    digitalWrite(IN_B_PIN, LOW);
    prefs.begin("slavecfg", true);
    slave_number = prefs.getUChar("slave_number", 1);
    prefs.end();
    rs485_addr = BASE_ADDR + (slave_number - 1);

    Serial.begin(115200);
    pinMode(RS485_DIR_PIN, OUTPUT);
    RS485_RECEIVE(); // Default to receive mode
    delay(10);
    Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    while (!Serial1) { delay(10); }
    pinMode(8, OUTPUT); // Status LED
    digitalWrite(8, LOW);
    analogSetPinAttenuation(POSITION_PIN, ADC_11db);
    adcAttachPin(POSITION_PIN);
    last_move_status = loadLastMoveStatusFromFlash();
    uint16_t calcPos = readPosition();
    if (calcPos > 10) {
      blind_state = STATE_OPEN;
    } else {
      blind_state = STATE_CLOSED;
    }
}

void loop() {
    static uint8_t buf[1 + 1 + 1 + 8 + 2 + 1];
    static size_t idx = 0;
    static unsigned long last_position_time = 0;
    unsigned long now = millis();
    update_motor();
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
                        resp_payload[0] = blind_state;
                        resp_payload[1] = (blind_position >> 8) & 0xFF;
                        resp_payload[2] = blind_position & 0xFF;
                        resp_payload[7] = last_move_status; // Add last_move_status to byte 7
                        LOG_DEBUG("[UART TX] GET_STATUS response: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", resp_payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        sendUARTResponse(GET_STATUS, resp_payload);
                        LOG_DEBUG("[UART TX] Sent GET_STATUS frame\r\n");
                    } else if (command == BLIND_COMMAND) {
                        LOG_DEBUG("[UART RX] BLIND_COMMAND payload: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        uint8_t set_state = payload[1];
                        LOG_DEBUG("[UART RX] BLIND_COMMAND set_state: 0x%02X\r\n", set_state);
                        uint16_t set_position = (payload[3] << 8) | payload[2];
                        uint16_t calcPos = readPosition();
                        LOG_DEBUG("[UART RX] BLIND_COMMAND set_position: %u\r\n", set_position);
                        switch (set_state) {
                            case 0x00: targetPos = 0; break;
                            case 0x01: targetPos = 1000; break;
                            case 0x03: targetPos = calcPos; stop_motor(); break;
                            case 0x04: targetPos = constrain(set_position, 0, 100) * 10; break;
                            default: break;
                        }
                        if (abs(targetPos - calcPos) > 10) {
                            motor_engaged = true;
                            last_move_status = 0x03; // MOVING
                            saveLastMoveStatusToFlash(last_move_status);
                            run_motor((targetPos < calcPos) ? 0x01 : 0x02, disengage_time + (abs(targetPos - calcPos) * stroke_time / 1000) + 5000);
                        }
                        LOG_DEBUG("[UART TX] BLIND_COMMAND echo: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        sendUARTResponse(BLIND_COMMAND, payload); // Echo for debug
                        LOG_DEBUG("[UART TX] Sent BLIND_COMMAND frame\r\n");
                    } else if (command == UPDATE_FIRMWARE) {
                        LOG_DEBUG("[UART TX] UPDATE_FIRMWARE command received, starting OTA...\r\n");
                        startConfigAPAndWebserver();
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
    if (now - last_position_time > 500) {
        last_position_time = now;
        blind_position = readPosition();
    }
}

