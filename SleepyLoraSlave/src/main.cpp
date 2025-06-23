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
AsyncEventSource events("/events"); // SSE event source
Ticker webserverTimeoutTicker;
bool configPortalActive = false;
unsigned long configPortalStartTime = 0;

#define POS_ALPHA 0.7
uint16_t closed_raw = 1600; // Default CLOSED_RAW
uint16_t open_raw = 5;      // Default OPEN_RAW
uint8_t blind_state = STATE_CLOSED;
uint16_t blind_position = 0; // 0-1000
uint16_t targetPos;  // position in % x 10
Ewma posEwma(POS_ALPHA);

int cached_raw_position = 0;
uint16_t cached_scaled_position = 0;

time_t disengage_time = 1000;
time_t stroke_time = 14000; // time for full stroke
time_t dynamic_pwm_target_stroke_time = 8000; // default 8s, adjust as needed

time_t motor_start_time;
time_t motor_run_time;
bool motor_running;
bool motor_engaged;
bool last_motor_dir;
uint8_t last_move_status; // New variable to track last move status

Preferences prefs;

// PWM control variables and defaults
#define EN_PWM_CHANNEL 0
#define EN_PWM_RESOLUTION 8 // 8 bits (0-255)
uint32_t pwm_freq = 1000; // Default 1kHz
uint8_t pwm_duty = 255;   // Default 100% (255/255)

// ===================== DYNAMIC PWM MODE =====================
bool dynamic_pwm_mode = false; // Experimental: runtime toggle
// Dynamic PWM control variables
uint8_t dynamic_pwm_target = 140; // starting PWM for move
uint8_t dynamic_pwm_engage = 100; // lower power for engage
uint32_t dynamic_pwm_update_interval = 100; // ms
unsigned long dynamic_pwm_last_update = 0;
bool dynamic_pwm_in_engage = false;
unsigned long dynamic_pwm_engage_start = 0;

// Target speed and stall detection
float dynamic_pwm_target_speed = 0.0f; // units: position delta per second
float dynamic_pwm_speed_ewma = 0.0f;
float dynamic_pwm_speed_alpha = 0.3f;
uint16_t dynamic_pwm_last_pos = 0;
unsigned long dynamic_pwm_last_pos_time = 0;
uint8_t dynamic_pwm_current = 100;
uint8_t dynamic_pwm_min = 80;
uint8_t dynamic_pwm_max = 255;
uint8_t dynamic_pwm_stall_count = 0;

// Editable advanced parameters (remove const/define)
uint8_t STALL_PWM_STEP = 20;  // Step to reduce PWM on stall detection
uint8_t SPEED_CORRECTION_STEP = 10; // Step to adjust target speed 
uint8_t STALL_COUNT_THRESHOLD_DEFAULT = 3; // Default stall count threshold
uint8_t STALL_COUNT_THRESHOLD_AFTER_UNJAM1 = 5; // After first unjam attempt
uint8_t STALL_COUNT_THRESHOLD_AFTER_UNJAM2 = 7; // After second unjam attempt
unsigned long UNJAM_DURATION = 500; // ms to reverse motor (first attempt)
unsigned long UNJAM_DURATION_2 = 1000; // ms to reverse motor (second attempt)

// Jam detection and unjam routine variables
uint8_t jam_attempt_count = 0;
const uint8_t JAM_ATTEMPT_LIMIT = 2; // Max unjam attempts
const uint8_t JAM_STALL_CYCLES = 2;  // Number of full stall cycles before jam
uint8_t jam_stall_cycle_count = 0;
bool in_unjam_routine = false;
unsigned long unjam_start_time = 0;
uint8_t unjam_direction = 0;
uint8_t stall_count_threshold = 3; // Active stall count threshold, initialized to default

// Struct to store original move parameters for unjam recovery
struct MoveParams {
    uint8_t direction;
    uint16_t target_position;
    unsigned long runtime;
};
MoveParams original_move = {0, 0, 0};

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

// ===================== Calibration Movement Detection Settings =====================
// These control how movement is detected during calibration:
// - MOVEMENT_THRESHOLD: Minimum change in position (raw units) from the start value to consider as movement.
// - MOVEMENT_CONSECUTIVE_SAMPLES: Number of consecutive samples above threshold required to confirm movement.
// The engage time is taken from the first sample that exceeds the threshold.
#define MOVEMENT_THRESHOLD 10
#define MOVEMENT_CONSECUTIVE_SAMPLES 3
// ===================================================================================

// Calibration state machine definitions

enum CalibState {
    CALIB_IDLE = 0,
    CALIB_START,
    CALIB_MOVE_TO_CLOSED,
    CALIB_WAIT_CLOSED,
    CALIB_FORWARD_STROKE_INIT,
    CALIB_FORWARD_STROKE,
    CALIB_FORWARD_DONE,
    CALIB_REVERSE_STROKE_INIT,
    CALIB_REVERSE_STROKE,
    CALIB_REVERSE_DONE,
    // New states for second cycle
    CALIB_SECOND_FORWARD_STROKE_INIT,
    CALIB_SECOND_FORWARD_STROKE,
    CALIB_SECOND_FORWARD_DONE,
    CALIB_SECOND_REVERSE_STROKE_INIT,
    CALIB_SECOND_REVERSE_STROKE,
    CALIB_SECOND_REVERSE_DONE,
    CALIB_SAVE,
    CALIB_COMPLETE,
    CALIB_ERROR
};

struct Calibration {
    CalibState state = CALIB_IDLE;
    unsigned long start_time = 0;
    unsigned long move_start_time = 0;
    unsigned long move_end_time = 0;
    unsigned long rev_move_start_time = 0;
    unsigned long rev_move_end_time = 0;
    unsigned long engage_time = 0;
    unsigned long forward_stroke_time = 0;
    unsigned long reverse_engage_time = 0;
    unsigned long reverse_stroke_time = 0;
    unsigned long disengage_time_calc =  0; // <-- add this line
    time_t orig_stroke = 0;
    time_t orig_disengage = 0;
    int *pos_samples = nullptr;
    unsigned long *time_samples = nullptr;
    int *rev_pos_samples = nullptr;
    unsigned long *rev_time_samples = nullptr;
    int sample_count = 400;
    int baseline_count = 20;
    int baseline_sum = 0;
    int baseline = 0;
    int noise_floor = 0;
    int move_threshold = 0;
    int move_start_idx = -1;
    int move_end_idx = -1;
    int rev_baseline_sum = 0;
    int rev_baseline = 0;
    int rev_noise_floor = 0;
    int rev_move_threshold = 0;
    int rev_move_start_idx = -1;
    int rev_move_end_idx = -1;
    bool error = false;

    // New fields for simplified calibration
    unsigned long motor_start_time = 0;
    unsigned long movement_detected_time = 0;
    unsigned long target_reached_time = 0;
    unsigned long last_sample_time = 0;
    int samples_taken = 0;
    unsigned long rev_motor_start_time = 0;
    unsigned long rev_movement_detected_time = 0;
    unsigned long rev_target_reached_time = 0;
    unsigned long rev_last_sample_time = 0;
    int rev_samples_taken = 0;

    unsigned long motor_start_time_1 = 0;
    unsigned long movement_detected_time_1 = 0;
    unsigned long target_reached_time_1 = 0;
    unsigned long rev_motor_start_time_1 = 0;
    unsigned long rev_movement_detected_time_1 = 0;
    unsigned long rev_target_reached_time_1 = 0;
        // Second cycle fields
    unsigned long engage_time2 = 0;
    unsigned long forward_stroke_time2 = 0;
    unsigned long reverse_engage_time2 = 0;
    unsigned long reverse_stroke_time2 = 0;
    unsigned long disengage_time_calc2 = 0;
    // Add average PWM fields for each stroke
    uint8_t forward_pwm_avg;
    uint8_t reverse_pwm_avg;
    uint8_t forward_pwm_avg2;
    uint8_t reverse_pwm_avg2;
    
};

Calibration calib;

// Forward declarations for calibration state machine
void processCalibrationStep();
void startCalibration();

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

void loadRawLimitsFromFlash() {
    LOG_INFO("[NVS] Loading raw limits from flash...\n");
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for reading!\n");
        eraseNVSAndReboot();
    }
    bool needSave = false;
    if (!prefs.isKey("closed_raw")) {
        LOG_WARN("[NVS] 'closed_raw' key missing, using default 1600.\n");
        closed_raw = 1600;
        needSave = true;
    } else {
        closed_raw = prefs.getUShort("closed_raw", 1600);
        LOG_INFO("[NVS] Loaded closed_raw: %u\n", closed_raw);
    }
    if (!prefs.isKey("open_raw")) {
        LOG_WARN("[NVS] 'open_raw' key missing, using default 5.\n");
        open_raw = 5;
        needSave = true;
    } else {
        open_raw = prefs.getUShort("open_raw", 5);
        LOG_INFO("[NVS] Loaded open_raw: %u\n", open_raw);
    }
    if (needSave) {
        LOG_INFO("[NVS] Saving default raw limits to flash.\n");
        prefs.putUShort("closed_raw", closed_raw);
        prefs.putUShort("open_raw", open_raw);
    }
    prefs.end();
}

void saveRawLimitsToFlash(uint16_t closed, uint16_t open) {
    LOG_INFO("[NVS] Saving raw limits to flash: closed_raw=%u, open_raw=%u\n", closed, open);
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for writing!\n");
        eraseNVSAndReboot();
    }
    prefs.putUShort("closed_raw", closed);
    prefs.putUShort("open_raw", open);
    prefs.end();
}

void handleSetClosedLimit(AsyncWebServerRequest *request) {
    int raw = readPositionRaw();
    LOG_INFO("[WEB] Set Closed Limit requested. Current raw: %d, Open raw: %d\n", raw, open_raw);
    if (abs(raw - open_raw) < 100) {
        LOG_WARN("[WEB] Closed limit validation failed: difference < 100 or equal.\n");
        request->send(400, "text/html",
            "<html><body>Error: Closed and Open limits must differ by at least 100 raw units and cannot be equal.<br>"
            "<a href='/'>Back</a></body></html>");
        return;
    }
    closed_raw = raw;
    saveRawLimitsToFlash(closed_raw, open_raw);
    LOG_INFO("[WEB] Closed limit set to %d and saved to flash.\n", closed_raw);
    request->send(200, "text/html",
        "<html><body>Closed limit set to current position (" + String(raw) + ").<br><a href='/'>Back</a></body></html>");
}

void handleSetOpenLimit(AsyncWebServerRequest *request) {
    int raw = readPositionRaw();
    LOG_INFO("[WEB] Set Open Limit requested. Current raw: %d, Closed raw: %d\n", raw, closed_raw);
    if (abs(closed_raw - raw) < 100) {
        LOG_WARN("[WEB] Open limit validation failed: difference < 100 or equal.\n");
        request->send(400, "text/html",
            "<html><body>Error: Closed and Open limits must differ by at least 100 raw units and cannot be equal.<br>"
            "<a href='/'>Back</a></body></html>");
        return;
    }
    open_raw = raw;
    saveRawLimitsToFlash(closed_raw, open_raw);
    LOG_INFO("[WEB] Open limit set to %d and saved to flash.\n", open_raw);
    request->send(200, "text/html",
        "<html><body>Open limit set to current position (" + String(raw) + ").<br><a href='/'>Back</a></body></html>");
}

void saveDynamicPWMToFlash(bool mode) {
    LOG_INFO("[NVS] Saving dynamic_pwm_mode to flash: %d\n", mode);
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for writing!\n");
        eraseNVSAndReboot();
    }
    prefs.putUChar("dyn_pwm", mode ? 1 : 0);
    prefs.end();
}

void loadDynamicPWMFromFlash() {
    LOG_INFO("[NVS] Loading dynamic_pwm_mode from flash...\n");
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for reading!\n");
        eraseNVSAndReboot();
    }
    if (!prefs.isKey("dyn_pwm")) {
        LOG_WARN("[NVS] 'dyn_pwm' key missing, using default 0.\n");
        dynamic_pwm_mode = false;
        prefs.putUChar("dyn_pwm", 0);
    } else {
        dynamic_pwm_mode = prefs.getUChar("dyn_pwm", 0) ? true : false;
        LOG_INFO("[NVS] Loaded dynamic_pwm_mode: %d\n", dynamic_pwm_mode);
    }
    prefs.end();
}

uint16_t readPosition(void) {
  int posAvg = readPositionRaw();
  cached_raw_position = posAvg;
  long posCalc = map(posAvg, closed_raw, open_raw, 0, 1000);  // based on 390k and 100k voltage divider
  uint16_t posConst = constrain(posCalc, 0, 1000);
  cached_scaled_position = posConst;
  //LOG_DEBUG("  Blind Position: %u\r\n", posConst);
  return posConst;
}

int readPositionRaw(void) {
    const int samples = 9;
    int readings[samples];
    for (int i = 0; i < samples; ++i) {
        readings[i] = analogReadMilliVolts(POSITION_PIN);
        delayMicroseconds(200); // Small delay for ADC settling
    }
    // Simple insertion sort for small arrays
    for (int i = 1; i < samples; ++i) {
        int key = readings[i];
        int j = i - 1;
        while (j >= 0 && readings[j] > key) {
            readings[j + 1] = readings[j];
            j--;
        }
        readings[j + 1] = key;
    }
    int median = readings[samples / 2];
    int posAvg = posEwma.filter(median);
    // LOG_DEBUG("RAW Position: %u  ", posAvg); // Commented out to prevent serial flooding
    return posAvg;
}

// set a direction and a max/estimated runtime, motor stop may be called once position reached
void run_motor(uint8_t dir, int runtime) {
  uint16_t calcPos = readPosition();
  LOG_DEBUG("Current Position: %.1f%% Target Position: %.1f%%\r\n", calcPos / 10.0f, targetPos / 10.0f);
  if (dir == 0x01) {
    if (closed_raw < open_raw) {
        digitalWrite(IN_A_PIN, HIGH);
        digitalWrite(IN_B_PIN, LOW); 
    } else {
        digitalWrite(IN_A_PIN, LOW);
        digitalWrite(IN_B_PIN, HIGH);
    }
    LOG_DEBUG("Motor Started Retract\r\n");
    if (motor_engaged) {
      blind_state = STATE_CLOSING;
    }
    last_motor_dir = true;
  }
  if (dir == 0x02) {
    if (closed_raw < open_raw) {
        digitalWrite(IN_A_PIN, LOW);
        digitalWrite(IN_B_PIN, HIGH); 
    } else {
        digitalWrite(IN_A_PIN, HIGH);
        digitalWrite(IN_B_PIN, LOW);
    }
    LOG_DEBUG("Motor Started Extend\r\n");
    if (motor_engaged) {
      blind_state = STATE_OPENING;
    }
    last_motor_dir = false;
  }

  if (dynamic_pwm_mode) {
    ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_engage);
    LOG_INFO("[DYN_PWM] Engage phase: PWM set to %u\n", dynamic_pwm_engage);
    dynamic_pwm_in_engage = true;
    dynamic_pwm_engage_start = millis();
    dynamic_pwm_last_update = millis();
    dynamic_pwm_last_pos = calcPos;
    dynamic_pwm_last_pos_time = millis();
    dynamic_pwm_speed_ewma = 0.0f;
    dynamic_pwm_current = dynamic_pwm_target;
    dynamic_pwm_stall_count = 0;
    // Set target speed based on calibration (full stroke time)
    if (stroke_time > 0) {
      dynamic_pwm_target_speed = 1000.0f / (float)dynamic_pwm_target_stroke_time * 1000.0f; // units: pos/second
      LOG_INFO("[DYN_PWM] Target speed set to %.2f pos/sec (stroke_time=%ld ms)\n", dynamic_pwm_target_speed, dynamic_pwm_target_stroke_time);
    } else {
      dynamic_pwm_target_speed = 60.0f; // fallback
    }
  } else {
    ledcWrite(EN_PWM_CHANNEL, pwm_duty);
    dynamic_pwm_in_engage = false;
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

// Forward declaration for timing settings loader
void loadTimingSettingsFromFlash();

// Helper function to robustly detect engage time from samples
int findEngageSample(const int *samples, int count, int baseline, int threshold, int sustain = 3) {
    for (int i = 0; i < count - sustain; ++i) {
        bool sustained = true;
        for (int j = 0; j < sustain; ++j) {
            if (abs(samples[i + j] - baseline) < threshold) {
                sustained = false;
                break;
            }
        }
        if (sustained) return i;
    }
    return -1;
}

// Helper: Find first robust engage sample index
int findEngageSample(int baseline, int *samples, int count, int threshold, int sustain) {
    for (int i = 0; i < count - sustain + 1; ++i) {
        int valid = 0;
        for (int j = 0; j < sustain; ++j) {
            if (abs(samples[i + j] - baseline) > threshold) valid++;
        }
        if (valid >= sustain - 1) return i;
    }
    return -1;
}

// Returns true if the last N samples in the buffer exceed the threshold from baseline
bool isConsecutiveMovementDetected(const int* samples, int samples_taken, int baseline, int threshold, int consecutive) {
    if (samples_taken < consecutive) return false;
    for (int i = samples_taken - consecutive; i < samples_taken; ++i) {
        if (abs(samples[i] - baseline) < threshold) {
            return false;
        }
    }
    return true;
}

void update_motor(void) {
  if (motor_running) {
    if (in_unjam_routine) {
      unsigned long unjam_duration = (jam_attempt_count == 1) ? UNJAM_DURATION_2 : UNJAM_DURATION;
      if (millis() - unjam_start_time > unjam_duration) {
        stop_motor();
        delay(100);
        if (jam_attempt_count < JAM_ATTEMPT_LIMIT) {
          // After first unjam, increase stall threshold
          stall_count_threshold = (jam_attempt_count == 0) ? STALL_COUNT_THRESHOLD_AFTER_UNJAM1 : STALL_COUNT_THRESHOLD_AFTER_UNJAM2;
          LOG_ERROR("[JAM] Retrying original direction after unjam attempt %u, stall threshold now %d\n", jam_attempt_count+1, stall_count_threshold);
          jam_attempt_count++;
          in_unjam_routine = false;
          run_motor(original_move.direction, original_move.runtime);
        } else {
            LOG_ERROR("[JAM] Jam persists after %u unjam attempts. Aborting.\n", JAM_ATTEMPT_LIMIT);
            disengage_motor();
            // Set state based on current position
            uint16_t pos = readPosition();
            if (pos > 10) {
                blind_state = STATE_OPEN;
            } else {
                blind_state = STATE_CLOSED;
            }
            last_move_status = 0x04; // Custom code for JAM_ABORTED
            saveLastMoveStatusToFlash(last_move_status);
            jam_attempt_count = 0;
            jam_stall_cycle_count = 0;
            in_unjam_routine = false;
            stall_count_threshold = STALL_COUNT_THRESHOLD_DEFAULT; // Reset after abort
            if (calib.state != CALIB_IDLE && calib.state != CALIB_COMPLETE && calib.state != CALIB_ERROR) {
                LOG_ERROR("[CAL] Calibration aborted due to jam. Setting CALIB_ERROR state.\n");
                calib.state = CALIB_ERROR;
            }
        }
      }
      return;
    }
    if (dynamic_pwm_mode) {
      unsigned long now = millis();
      uint16_t pos = readPosition();
      // Handle engage phase
      if (dynamic_pwm_in_engage) {
        if ((calib.state != CALIB_IDLE && (now - dynamic_pwm_engage_start >= 4000)) ||
            (calib.state == CALIB_IDLE && (now - dynamic_pwm_engage_start >= disengage_time))) {
          ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
          LOG_INFO("[DYN_PWM] Running phase: PWM set to %u\n", dynamic_pwm_current);
          dynamic_pwm_in_engage = false;
          dynamic_pwm_last_pos = pos;
          dynamic_pwm_last_pos_time = now;
        }
      } else if (now - dynamic_pwm_last_update >= dynamic_pwm_update_interval) {
        // Speed calculation
        float dt = (now - dynamic_pwm_last_pos_time) / 1000.0f;
        float dpos = abs((int)pos - (int)dynamic_pwm_last_pos);
        float speed = (dt > 0.01f) ? dpos / dt : 0.0f;
        dynamic_pwm_speed_ewma = dynamic_pwm_speed_alpha * speed + (1.0f - dynamic_pwm_speed_alpha) * dynamic_pwm_speed_ewma;
        LOG_DEBUG("[DYN_PWM] Speed: %.2f pos/sec, Target: %.2f\n", dynamic_pwm_speed_ewma, dynamic_pwm_target_speed);
        // Stall detection
        if (dynamic_pwm_speed_ewma < dynamic_pwm_target_speed * 0.3f) {
          dynamic_pwm_stall_count++;
          LOG_WARN("[DYN_PWM] Possible stall detected (%u/%d)\n", dynamic_pwm_stall_count, stall_count_threshold);
          if (dynamic_pwm_stall_count >= stall_count_threshold && dynamic_pwm_current < dynamic_pwm_max) {
            dynamic_pwm_current = (uint8_t)std::min((int)dynamic_pwm_current + STALL_PWM_STEP, (int)dynamic_pwm_max);
            ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
            LOG_INFO("[DYN_PWM] Stall: PWM increased to %u\n", dynamic_pwm_current);
            jam_stall_cycle_count++;
            dynamic_pwm_stall_count = 0;
            if (jam_stall_cycle_count >= JAM_STALL_CYCLES) {
              // Jam detected, start unjam routine
              LOG_ERROR("[JAM] Jam detected! Initiating unjam routine.\n");
              // Save original move parameters for recovery
              original_move.direction = last_motor_dir ? 0x01 : 0x02;
              original_move.target_position = targetPos;
              original_move.runtime = motor_run_time - (millis() - motor_start_time);
              stop_motor();
              delay(100);
              // Reverse direction at full power
              in_unjam_routine = true;
              unjam_start_time = millis();
              unjam_direction = (original_move.direction == 0x01) ? 0x02 : 0x01; // reverse direction
              ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_max);
              digitalWrite(IN_A_PIN, unjam_direction == 0x01);
              digitalWrite(IN_B_PIN, unjam_direction == 0x02);
              motor_running = true;
              return;
            }
          }
        } else {
          dynamic_pwm_stall_count = 0;
          jam_stall_cycle_count = 0;
          stall_count_threshold = STALL_COUNT_THRESHOLD_DEFAULT; // Reset after successful movement
          // Adjust PWM to maintain target speed
          if (dynamic_pwm_speed_ewma < dynamic_pwm_target_speed * 0.95f && dynamic_pwm_current < dynamic_pwm_max) {
            dynamic_pwm_current = (uint8_t)std::min((int)dynamic_pwm_current + SPEED_CORRECTION_STEP, (int)dynamic_pwm_max);
            ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
            LOG_INFO("[DYN_PWM] PWM increased to %u\n", dynamic_pwm_current);
          } else if (dynamic_pwm_speed_ewma > dynamic_pwm_target_speed * 1.05f && dynamic_pwm_current > dynamic_pwm_min) {
            dynamic_pwm_current = (uint8_t)std::max((int)dynamic_pwm_current - SPEED_CORRECTION_STEP, (int)dynamic_pwm_min);
            ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
            LOG_INFO("[DYN_PWM] PWM decreased to %u\n", dynamic_pwm_current);
          }
        }
        dynamic_pwm_last_update = now;
        dynamic_pwm_last_pos = pos;
        dynamic_pwm_last_pos_time = now;
      }
    }
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
        if (calib.state != CALIB_IDLE) {
          calib.state = CALIB_ERROR;
        }
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
  ledcWrite(EN_PWM_CHANNEL, 0); // Stop PWM
  motor_running = false;
  return;
}

void saveLastMoveStatusToFlash(uint8_t status) {
    LOG_INFO("[NVS] Saving last_move_status to flash: %u\n", status);
    prefs.begin("blindcfg", false);
    prefs.putUChar("lms", status); // key shortened to fit NVS requirements
    prefs.end();
}

uint8_t loadLastMoveStatusFromFlash() {
    LOG_INFO("[NVS] Loading last_move_status from flash...\n");
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for reading!\n");
        eraseNVSAndReboot();
    }
    uint8_t status = 0;
    if (!prefs.isKey("lms")) {
        LOG_WARN("[NVS] 'lms' key missing, using default 0.\n");
        status = 0;
        prefs.putUChar("lms", status);
    } else {
        status = prefs.getUChar("lms", 0);
        LOG_INFO("[NVS] Loaded last_move_status: %u\n", status);
    }
    prefs.end();
    return status;
}

void handleGetPosition(AsyncWebServerRequest *request) {
    // Always update position before responding
    uint16_t latest_scaled = readPosition();
    int latest_raw = cached_raw_position; // readPosition() updates this
    char json[64];
    snprintf(json, sizeof(json),
        "{\"raw\":%d,\"percent\":%.1f}",
        latest_raw,
        latest_scaled / 10.0f
    );
    request->send(200, "application/json", json);
}

void savePWMSettingsToFlash(uint32_t freq, uint8_t duty) {
    LOG_INFO("[NVS] Saving PWM settings to flash: freq=%u, duty=%u\n", freq, duty);
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for writing!\n");
        eraseNVSAndReboot();
    }
    prefs.putUInt("pwm_freq", freq);
    prefs.putUChar("pwm_duty", duty);
    prefs.end();
}

void loadPWMSettingsFromFlash() {
    LOG_INFO("[NVS] Loading PWM settings from flash...\n");
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for reading!\n");
        eraseNVSAndReboot();
    }
    bool needSave = false;
    if (!prefs.isKey("pwm_freq")) {
        LOG_WARN("[NVS] 'pwm_freq' key missing, using default 1000.\n");
        pwm_freq = 1000;
        needSave = true;
    } else {
        pwm_freq = prefs.getUInt("pwm_freq", 1000);
        LOG_INFO("[NVS] Loaded pwm_freq: %u\n", pwm_freq);
    }
    if (!prefs.isKey("pwm_duty")) {
        LOG_WARN("[NVS] 'pwm_duty' key missing, using default 255.\n");
        pwm_duty = 255;
        needSave = true;
    } else {
        pwm_duty = prefs.getUChar("pwm_duty", 255);
        LOG_INFO("[NVS] Loaded pwm_duty: %u\n", pwm_duty);
    }
    if (needSave) {
        LOG_INFO("[NVS] Saving default PWM settings to flash.\n");
        prefs.putUInt("pwm_freq", pwm_freq);
        prefs.putUChar("pwm_duty", pwm_duty);
    }
    prefs.end();
}

void saveTimingSettingsToFlash(time_t stroke, time_t disengage) {
    LOG_INFO("[NVS] Saving timing settings to flash: stroke_time=%ld, disengage_time=%ld\n", stroke, disengage);
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for writing!\n");
        eraseNVSAndReboot();
    }
    prefs.putULong("stroke_time", stroke);
    prefs.putULong("disengage_time", disengage);
    prefs.end();
}

void loadTimingSettingsFromFlash();
void handleAutoCalibrate(AsyncWebServerRequest *request);

void loadTimingSettingsFromFlash() {
    LOG_INFO("[NVS] Loading timing settings from flash...\n");
    if (!prefs.begin("blindcfg", false)) {
        LOG_ERROR("[NVS] Failed to open 'blindcfg' namespace for reading!\n");
        eraseNVSAndReboot();
    }
    bool needSave = false;
    if (!prefs.isKey("stroke_time")) {
        LOG_WARN("[NVS] 'stroke_time' key missing, using default 14000.\n");
        stroke_time = 14000;
        needSave = true;
    } else {
        stroke_time = prefs.getULong("stroke_time", 14000);
        LOG_INFO("[NVS] Loaded stroke_time: %ld\n", stroke_time);
    }
    if (!prefs.isKey("disengage_time")) {
        LOG_WARN("[NVS] 'disengage_time' key missing, using default 1000.\n");
        disengage_time = 1000;
        needSave = true;
    } else {
        disengage_time = prefs.getULong("disengage_time", 1000);
        LOG_INFO("[NVS] Loaded disengage_time: %ld\n", disengage_time);
    }
    if (needSave) {
        LOG_INFO("[NVS] Saving default timing settings to flash.\n");
        prefs.putULong("stroke_time", stroke_time);
        prefs.putULong("disengage_time", disengage_time);
    }
    prefs.end();
}

void handleSetTiming(AsyncWebServerRequest *request) {
    if (request->hasParam("stroke_time", true) && request->hasParam("disengage_time", true)) {
        time_t stroke = request->getParam("stroke_time", true)->value().toInt();
        time_t disengage = request->getParam("disengage_time", true)->value().toInt();
        if (stroke < 1000 || stroke > 60000 || disengage < 100 || disengage > 10000) {
            request->send(400, "text/html", "<html><body>Invalid timing values.<br><a href='/' >Back</a></body></html>");
            return;
        }
        stroke_time = stroke;
        disengage_time = disengage;
        saveTimingSettingsToFlash(stroke_time, disengage_time);
        request->send(200, "text/html", "<html><body>Timing settings updated.<br><a href='/' >Back</a></body></html>");
    } else {
        request->send(400, "text/html", "<html><body>Missing timing parameters.<br><a href='/' >Back</a></body></html>");
    }
}

void handleConfigPage(AsyncWebServerRequest *request) {
    char *html = (char*)malloc(16384); // Increased buffer for large HTML
    if (!html) {
        request->send(500, "text/html", "Internal Error: Out of Memory");
        return;
    }
    // Debug output for all arguments
    LOG_INFO("[DEBUG] handleConfigPage arguments:\n");
    LOG_INFO("  stroke_time: %ld\n", (long)stroke_time);
    LOG_INFO("  disengage_time: %ld\n", (long)disengage_time);
    LOG_INFO("  pwm_freq: %u\n", pwm_freq);
    LOG_INFO("  pwm_duty: %u\n", pwm_duty);
    LOG_INFO("  dynamic_pwm_mode: %d (%s)\n", dynamic_pwm_mode, dynamic_pwm_mode ? "checked" : "");
    LOG_INFO("  advanced div display: %s\n", (dynamic_pwm_mode ? "block" : "none"));
    LOG_INFO("  dynamic_pwm_min: %u\n", dynamic_pwm_min);
    LOG_INFO("  dynamic_pwm_max: %u\n", dynamic_pwm_max);
    LOG_INFO("  dynamic_pwm_engage: %u\n", dynamic_pwm_engage);
    LOG_INFO("  dynamic_pwm_current: %u\n", dynamic_pwm_current);
    LOG_INFO("  STALL_PWM_STEP: %u\n", STALL_PWM_STEP);
    LOG_INFO("  SPEED_CORRECTION_STEP: %u\n", SPEED_CORRECTION_STEP);
    LOG_INFO("  STALL_COUNT_THRESHOLD_DEFAULT: %u\n", STALL_COUNT_THRESHOLD_DEFAULT);
    LOG_INFO("  STALL_COUNT_THRESHOLD_AFTER_UNJAM1: %u\n", STALL_COUNT_THRESHOLD_AFTER_UNJAM1);
    LOG_INFO("  STALL_COUNT_THRESHOLD_AFTER_UNJAM2: %u\n", STALL_COUNT_THRESHOLD_AFTER_UNJAM2);
    LOG_INFO("  slave_number: %u\n", slave_number);
    int html_len = snprintf(html, 16384,
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
        "<div id='livepos' style='text-align:center;font-size:1.1em;margin-bottom:16px;'>"
        "Raw Position: <span id='rawpos'>...</span>  Blind Position: <span id='blindpos'>...</span>%"
        "</div>"
        "<div id='poserror' style='color:#c00;text-align:center;margin-bottom:8px;'></div>"
        "<form method='POST' action='/set_closed' style='margin-top:10px;'>"
        "<button type='submit' style='background:#ffc107;color:#222;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;'>Set Closed Limit (Current Position)</button>"
        "</form>"
        "<form method='POST' action='/set_open' style='margin-top:10px;'>"
        "<button type='submit' style='background:#17a2b8;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;'>Set Open Limit (Current Position)</button>"
        "</form>"
        "<form method='POST' action='/autocal' style='margin-top:10px;'>"
        "<button type='submit' style='background:#fd7e14;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;'>Auto Calibrate Stroke & Backlash</button>"
        "</form>"
        "<form method='POST' action='/set_timing' style='margin-top:10px;'>"
        "<label>Stroke Time (ms):</label><br>"
        "<input type='number' name='stroke_time' min='1000' max='60000' value='%ld' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label>Disengage Time (ms):</label><br>"
        "<input type='number' name='disengage_time' min='100' max='10000' value='%ld' style='width:100%%;padding:6px;margin-bottom:10px;'><br>"
        "<button type='submit' style='background:#007bff;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;'>Set Timing</button>"
        "</form>"
        "<form method='POST' action='/set_pwm' style='margin-top:10px;'>"
        "<label>PWM Frequency (Hz):</label><br>"
        "<input type='number' name='pwm_freq' min='100' max='20000' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label>PWM Duty Cycle (0-255):</label><br>"
        "<input type='number' name='pwm_duty' min='0' max='255' value='%u' style='width:100%%;padding:6px;margin-bottom:10px;'><br>"
        "<button type='submit' style='background:#6c757d;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;'>Set Motor PWM</button>"
        "</form>"
        "<form method='POST' action='/set_dynamic_pwm' style='margin-top:10px;'>"
        "<label><input type='checkbox' id='dynamic_pwm_mode' name='dynamic_pwm_mode' value='1' %s onchange='toggleAdvanced(this)'> Enable Dynamic PWM (experimental, quieter)</label><br>"
        "<div id='advanced_pwm' style='display:%s;margin-top:12px;padding:12px 8px 8px 8px;background:#f3f3f3;border-radius:8px;'>"
        "<style>"
        ".info-icon { display:inline-block; margin-left:4px; color:#17a2b8; cursor:pointer; font-size:1.1em; vertical-align:middle; }"
        ".tooltip-text { visibility:hidden; background:#222; color:#fff; text-align:left; border-radius:6px; padding:6px 10px; position:absolute; z-index:10; font-size:0.95em; min-width:180px; left:32px; top:50%%; transform:translateY(-50%%); box-shadow:0 2px 8px #0003; }"
        ".info-icon.active + .tooltip-text, .info-icon:hover + .tooltip-text { visibility:visible; }"
        ".tooltip-wrap { position:relative; display:inline-block; }"
        "@media (hover:none) { .info-icon:hover + .tooltip-text { visibility:hidden; } }"
        "</style>"
        "<script>"
        "document.addEventListener('DOMContentLoaded',function(){"
        "  document.querySelectorAll('.info-icon').forEach(function(icon){"
        "    icon.addEventListener('click',function(e){"
        "      e.stopPropagation();"
        "      document.querySelectorAll('.info-icon').forEach(function(i){if(i!==icon)i.classList.remove('active');});"
        "      icon.classList.toggle('active');"
        "    });"
        "  });"
        "  document.body.addEventListener('click',function(){"
        "    document.querySelectorAll('.info-icon').forEach(function(i){i.classList.remove('active');});"
        "  });"
        "});"
        "</script>"
        "<label class='tooltip-wrap'>Dynamic PWM Target Stroke Time (ms): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Stroke time (ms) used for speed targeting in dynamic PWM mode. Does NOT affect move timeout.</span></label><br>"
        "<input type='number' name='dynamic_pwm_target_stroke_time' min='1000' max='60000' value='%ld' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>PWM Min: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Minimum PWM value during dynamic operation. Lower values = quieter, but may stall.</span></label><br>"
        "<input type='number' name='dynamic_pwm_min' min='0' max='255' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>PWM Max: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Maximum PWM value during dynamic operation. Higher values = more power, but louder.</span></label><br>"
        "<input type='number' name='dynamic_pwm_max' min='0' max='255' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>PWM Engage: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>PWM value used during engage phase (initial movement).</span></label><br>"
        "<input type='number' name='dynamic_pwm_engage' min='0' max='255' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>PWM Current: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Current PWM value (runtime, for debugging/tuning).</span></label><br>"
        "<input type='number' name='dynamic_pwm_current' min='0' max='255' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Stall Step Size: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>PWM step size when stall detected. Higher = more aggressive recovery.</span></label><br>"
        "<input type='number' name='stall_pwm_step' min='1' max='100' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Speed Correction Step: <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>PWM step size for normal speed correction. Lower = smoother, higher = faster response.</span></label><br>"
        "<input type='number' name='speed_correction_step' min='1' max='100' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Stall Count (default): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Stall count threshold before recovery is triggered (default).</span></label><br>"
        "<input type='number' name='stall_count_default' min='1' max='20' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Stall Count (after unjam 1): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Stall count threshold after first unjam attempt.</span></label><br>"
        "<input type='number' name='stall_count_ujam1' min='1' max='20' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Stall Count (after unjam 2): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Stall count threshold after second unjam attempt.</span></label><br>"
        "<input type='number' name='stall_count_ujam2' min='1' max='20' value='%u' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Unjam Duration 1 (ms): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Unjam duration (ms, first attempt). How long to reverse motor to clear jam.</span></label><br>"
        "<input type='number' name='unjam_duration_1' min='100' max='5000' value='%lu' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "<label class='tooltip-wrap'>Unjam Duration 2 (ms): <span class='info-icon' tabindex='0'>&#8505;</span><span class='tooltip-text'>Unjam duration (ms, second attempt). Longer for stubborn jams.</span></label><br>"
        "<input type='number' name='unjam_duration_2' min='100' max='5000' value='%lu' style='width:100%%;padding:6px;margin-bottom:8px;'><br>"
        "</div>"
        "<button type='submit' style='background:#20c997;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;width:100%%;margin-top:10px;'>Save PWM Settings</button>"
        "</form>"
        "<script>function toggleAdvanced(cb){document.getElementById('advanced_pwm').style.display=cb.checked?'block':'none';}</script>"
        "<hr style='margin:18px 0; border:0; border-top:2px solid #eee;'>"
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
        "<hr style='margin:18px 0; border:0; border-top:2px solid #eee;'>"
        "<div style='margin-top:18px; text-align:center;'>"
        "<a href='/upload' style='color:#007bff; font-size:1.1em; text-decoration:underline;'>Update Firmware from File</a>"
        "</div>"
        "<hr style='margin:18px 0; border:0; border-top:2px solid #eee;'>"
        "<form method='POST' action='/close' style='margin-top:18px; text-align:center;'>"
        "<button type='submit' style='background:#dc3545;color:#fff;font-size:1.1em;padding:8px 24px;border:none;border-radius:6px;cursor:pointer;'>Close Web Portal & Sleep</button>"
        "</form>"
        "<script>"
        "document.addEventListener('DOMContentLoaded', function() {"
        "  var raw = document.getElementById('rawpos');"
        "  var blind = document.getElementById('blindpos');"
        "  var err = document.getElementById('poserror');"
        "  if (!raw || !blind || !err) return;"
        "  err.textContent = 'SCRIPT RUNNING';"
        "  raw.textContent = 'LOADING...';"
        "  blind.textContent = 'LOADING...';"
        "  var es = new EventSource('/events');"
        "  es.onopen = function() { err.textContent = 'Live updates connected.'; };"
        "  es.onerror = function(e) { err.textContent = 'SSE connection error.'; };"
        "  es.onmessage = function(ev) {"
        "    try {"
        "      var d = JSON.parse(ev.data);"
        "      raw.textContent = d.raw;"
        "      blind.textContent = d.percent.toFixed(1);"
        "      err.textContent = '';"
        "    } catch(e) {"
        "      err.textContent = 'Parse error: ' + e;"
        "    }"
        "  };"
        "});"
        "</script>"
        "</div></body></html>",
        (long)stroke_time, (long)disengage_time, pwm_freq, pwm_duty, dynamic_pwm_mode ? "checked" : "",
        (dynamic_pwm_mode ? "block" : "none"),
        (long)dynamic_pwm_target_stroke_time, dynamic_pwm_min, dynamic_pwm_max, dynamic_pwm_engage, dynamic_pwm_current,
        STALL_PWM_STEP, SPEED_CORRECTION_STEP, STALL_COUNT_THRESHOLD_DEFAULT,
        STALL_COUNT_THRESHOLD_AFTER_UNJAM1, STALL_COUNT_THRESHOLD_AFTER_UNJAM2,
        (unsigned long)UNJAM_DURATION, (unsigned long)UNJAM_DURATION_2,
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
    LOG_INFO("[DEBUG] snprintf returned: %d\n", html_len);
    if (html_len >= 16384) LOG_WARN("[WARN] HTML output truncated!\n");
    request->send(200, "text/html", html);
    free(html);
}

void setupPWM() {
    ledcSetup(EN_PWM_CHANNEL, pwm_freq, EN_PWM_RESOLUTION);
    ledcAttachPin(EN_PIN, EN_PWM_CHANNEL);
    ledcWrite(EN_PWM_CHANNEL, pwm_duty);
}

void updatePWM(uint32_t freq, uint8_t duty) {
    pwm_freq = freq;
    pwm_duty = duty;
    ledcSetup(EN_PWM_CHANNEL, pwm_freq, EN_PWM_RESOLUTION);
    ledcWrite(EN_PWM_CHANNEL, pwm_duty);
    savePWMSettingsToFlash(pwm_freq, pwm_duty);
}

void handleSetPWM(AsyncWebServerRequest *request) {
    if (request->hasParam("pwm_freq", true) && request->hasParam("pwm_duty", true)) {
        uint32_t freq = request->getParam("pwm_freq", true)->value().toInt();
        uint8_t duty = request->getParam("pwm_duty", true)->value().toInt();
        if (freq < 100 || freq > 20000 || duty > 255) {
            request->send(400, "text/html", "<html><body>Invalid PWM values.<br><a href='/' >Back</a></body></html>");
            return;
        }
        updatePWM(freq, duty);
        request->send(200, "text/html", "<html><body>PWM settings updated.<br><a href='/' >Back</a></body></html>");
    } else {
        request->send(400, "text/html", "<html><body>Missing PWM parameters.<br><a href='/' >Back</a></body></html>");
    }
}

// Save/load advanced dynamic PWM parameters to/from NVS
void saveAdvancedPWMSettingsToFlash() {
    if (prefs.begin("blindcfg", false)) {
        prefs.putULong("dyn_pwm_target_stroke_time", dynamic_pwm_target_stroke_time);
        prefs.putUChar("dyn_pwm_min", dynamic_pwm_min);
        prefs.putUChar("dyn_pwm_max", dynamic_pwm_max);
        prefs.putUChar("dyn_pwm_engage", dynamic_pwm_engage);
        prefs.putUChar("dyn_pwm_current", dynamic_pwm_current);
        prefs.putUChar("stall_pwm_step", STALL_PWM_STEP);
        prefs.putUChar("spd_corr_step", SPEED_CORRECTION_STEP);
        prefs.putUChar("stall_cnt_def", STALL_COUNT_THRESHOLD_DEFAULT);
        prefs.putUChar("stall_cnt_uj1", STALL_COUNT_THRESHOLD_AFTER_UNJAM1);
        prefs.putUChar("stall_cnt_uj2", STALL_COUNT_THRESHOLD_AFTER_UNJAM2);
        prefs.putULong("unjam_dur1", UNJAM_DURATION);
        prefs.putULong("unjam_dur2", UNJAM_DURATION_2);
        prefs.end();
    }
}
void loadAdvancedPWMSettingsFromFlash() {
    if (prefs.begin("blindcfg", true)) {
        dynamic_pwm_target_stroke_time = prefs.getULong("dyn_pwm_target_stroke_time", dynamic_pwm_target_stroke_time);
        dynamic_pwm_min = prefs.getUChar("dyn_pwm_min", dynamic_pwm_min);
        dynamic_pwm_max = prefs.getUChar("dyn_pwm_max", dynamic_pwm_max);
        dynamic_pwm_engage = prefs.getUChar("dyn_pwm_engage", dynamic_pwm_engage);
        dynamic_pwm_current = prefs.getUChar("dyn_pwm_current", dynamic_pwm_current);
        STALL_PWM_STEP = prefs.getUChar("stall_pwm_step", STALL_PWM_STEP);
        SPEED_CORRECTION_STEP = prefs.getUChar("spd_corr_step", SPEED_CORRECTION_STEP);
        STALL_COUNT_THRESHOLD_DEFAULT = prefs.getUChar("stall_cnt_def", STALL_COUNT_THRESHOLD_DEFAULT);
        STALL_COUNT_THRESHOLD_AFTER_UNJAM1 = prefs.getUChar("stall_cnt_uj1", STALL_COUNT_THRESHOLD_AFTER_UNJAM1);
        STALL_COUNT_THRESHOLD_AFTER_UNJAM2 = prefs.getUChar("stall_cnt_uj2", STALL_COUNT_THRESHOLD_AFTER_UNJAM2);
        UNJAM_DURATION = prefs.getULong("unjam_dur1", UNJAM_DURATION);
        UNJAM_DURATION_2 = prefs.getULong("unjam_dur2", UNJAM_DURATION_2);
        prefs.end();
    }
}

void handleSetDynamicPWM(AsyncWebServerRequest *request) {
    LOG_INFO("[DEBUG] handleSetDynamicPWM called\n");
    // Parse checkbox
    dynamic_pwm_mode = request->hasParam("dynamic_pwm_mode", true);
    LOG_INFO("[DEBUG] dynamic_pwm_mode: %d\n", dynamic_pwm_mode);
    // Parse advanced fields if present, always check for nullptr and use constrain
    AsyncWebParameter* p;
    p = request->getParam("dynamic_pwm_target_stroke_time", true); if (p) { dynamic_pwm_target_stroke_time = constrain(p->value().toInt(), 1000, 60000); LOG_INFO("[DEBUG] dynamic_pwm_target_stroke_time: %ld\n", (long)dynamic_pwm_target_stroke_time);} else { LOG_INFO("[DEBUG] dynamic_pwm_target_stroke_time missing\n"); }
    p = request->getParam("dynamic_pwm_min", true); if (p) { dynamic_pwm_min = constrain(p->value().toInt(), 0, 255); LOG_INFO("[DEBUG] dynamic_pwm_min: %d\n", dynamic_pwm_min); } else { LOG_INFO("[DEBUG] dynamic_pwm_min missing\n"); }
    p = request->getParam("dynamic_pwm_max", true); if (p) { dynamic_pwm_max = constrain(p->value().toInt(), 0, 255); LOG_INFO("[DEBUG] dynamic_pwm_max: %d\n", dynamic_pwm_max); } else { LOG_INFO("[DEBUG] dynamic_pwm_max missing\n"); }
    p = request->getParam("dynamic_pwm_engage", true); if (p) { dynamic_pwm_engage = constrain(p->value().toInt(), 0, 255); LOG_INFO("[DEBUG] dynamic_pwm_engage: %d\n", dynamic_pwm_engage); } else { LOG_INFO("[DEBUG] dynamic_pwm_engage missing\n"); }
    p = request->getParam("dynamic_pwm_current", true); if (p) { dynamic_pwm_current = constrain(p->value().toInt(), 0, 255); LOG_INFO("[DEBUG] dynamic_pwm_current: %d\n", dynamic_pwm_current); } else { LOG_INFO("[DEBUG] dynamic_pwm_current missing\n"); }
    p = request->getParam("stall_pwm_step", true); if (p) { STALL_PWM_STEP = constrain(p->value().toInt(), 1, 100); LOG_INFO("[DEBUG] stall_pwm_step: %d\n", STALL_PWM_STEP); } else { LOG_INFO("[DEBUG] stall_pwm_step missing\n"); }
    p = request->getParam("speed_correction_step", true); if (p) { SPEED_CORRECTION_STEP = constrain(p->value().toInt(), 1, 100); LOG_INFO("[DEBUG] speed_correction_step: %d\n", SPEED_CORRECTION_STEP); } else { LOG_INFO("[DEBUG] speed_correction_step missing\n"); }
    p = request->getParam("stall_count_default", true); if (p) { STALL_COUNT_THRESHOLD_DEFAULT = constrain(p->value().toInt(), 1, 20); LOG_INFO("[DEBUG] stall_count_default: %d\n", STALL_COUNT_THRESHOLD_DEFAULT); } else { LOG_INFO("[DEBUG] stall_count_default missing\n"); }
    p = request->getParam("stall_count_ujam1", true); if (p) { STALL_COUNT_THRESHOLD_AFTER_UNJAM1 = constrain(p->value().toInt(), 1, 20); LOG_INFO("[DEBUG] stall_count_ujam1: %d\n", STALL_COUNT_THRESHOLD_AFTER_UNJAM1); } else { LOG_INFO("[DEBUG] stall_count_ujam1 missing\n"); }
    p = request->getParam("stall_count_ujam2", true); if (p) { STALL_COUNT_THRESHOLD_AFTER_UNJAM2 = constrain(p->value().toInt(), 1, 20); LOG_INFO("[DEBUG] stall_count_ujam2: %d\n", STALL_COUNT_THRESHOLD_AFTER_UNJAM2); } else { LOG_INFO("[DEBUG] stall_count_ujam2 missing\n"); }
    p = request->getParam("unjam_duration_1", true); if (p) { UNJAM_DURATION = constrain(p->value().toInt(), 100, 5000); LOG_INFO("[DEBUG] unjam_duration_1: %lu\n", UNJAM_DURATION); } else { LOG_INFO("[DEBUG] unjam_duration_1 missing\n"); }
    p = request->getParam("unjam_duration_2", true); if (p) { UNJAM_DURATION_2 = constrain(p->value().toInt(), 100, 5000); LOG_INFO("[DEBUG] unjam_duration_2: %lu\n", UNJAM_DURATION_2); } else { LOG_INFO("[DEBUG] unjam_duration_2 missing\n"); }
    saveDynamicPWMToFlash(dynamic_pwm_mode);
    saveAdvancedPWMSettingsToFlash();
    request->send(200, "text/html", "<html><body>Dynamic PWM and advanced settings saved.<br><a href='/' >Back</a></body></html>");
}

void startConfigAPAndWebserver() {
    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // Reduce WiFi transmit power for testing
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
    server.on("/set_closed", HTTP_POST, handleSetClosedLimit);
    server.on("/set_open", HTTP_POST, handleSetOpenLimit);
    server.on("/position", HTTP_GET, handleGetPosition);
    server.addHandler(&events); // Register SSE endpoint
    server.on("/set_pwm", HTTP_POST, handleSetPWM);
    server.on("/set_timing", HTTP_POST, handleSetTiming);
    server.on("/autocal", HTTP_POST, handleAutoCalibrate);
    server.on("/set_dynamic_pwm", HTTP_POST, handleSetDynamicPWM);
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
    pinMode(IN_A_PIN, OUTPUT);
    pinMode(IN_B_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(IN_A_PIN, LOW);
    digitalWrite(IN_B_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
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
    loadRawLimitsFromFlash();
    loadPWMSettingsFromFlash();
    loadTimingSettingsFromFlash();
    loadDynamicPWMFromFlash();
    loadAdvancedPWMSettingsFromFlash();
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
    setupPWM();
    stall_count_threshold = STALL_COUNT_THRESHOLD_DEFAULT; // Initialize global stall count threshold
}



void loop() {
    static uint8_t buf[1 + 1 + 1 + 8 + 2 + 1];
    static size_t idx = 0;
    static unsigned long last_position_time = 0;
    static int last_sent_raw = -1;
    static uint16_t last_sent_scaled = 0;
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
                        uint8_t status_payload[8] = {0};
                        status_payload[0] = blind_state;
                        status_payload[1] = (blind_position >> 8) & 0xFF;
                        status_payload[2] = blind_position & 0xFF;
                        status_payload[7] = last_move_status; // Add last_move_status to byte 7
                        LOG_DEBUG("[UART TX] BLIND_COMMAND status response: ");
                        for (int i = 0; i < 8; ++i) LOG_DEBUG_RAW("%02X ", status_payload[i]);
                        LOG_DEBUG_RAW("\r\n");
                        sendUARTResponse(GET_STATUS, status_payload);
                        LOG_DEBUG("[UART TX] Sent BLIND_COMMAND status frame\r\n");
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
        // SSE: Only send if changed
        if (cached_raw_position != last_sent_raw || cached_scaled_position != last_sent_scaled) {
            char json[64];
            snprintf(json, sizeof(json), "{\"raw\":%d,\"percent\":%.1f}", cached_raw_position, cached_scaled_position / 10.0f);
            events.send(json, NULL, millis());
            last_sent_raw = cached_raw_position;
            last_sent_scaled = cached_scaled_position;
            Serial.printf("[SSE] Raw: %d  Blind: %.1f%%\r\n", cached_raw_position, cached_scaled_position / 10.0f);
        }
    }
    processCalibrationStep();
}

void startCalibration() {
    if (calib.state != CALIB_IDLE) return;
    calib = Calibration(); // reset
    calib.state = CALIB_START;
    calib.orig_disengage = disengage_time; // Save original disengage_time
    disengage_time = 1; // Always set to 1 for calibration, both modes
    LOG_INFO("[CAL] disengage_time set to 1ms for calibration (all modes)\n");
}

void processCalibrationStep() {
    // Declare these outside the switch to avoid jump errors
    static unsigned long avg_stroke = 0;
    static unsigned long avg_disengage = 0;
    // --- Dynamic PWM calibration additions ---
    static uint16_t forward_pwm_sum = 0, forward_pwm_count = 0;
    static uint16_t reverse_pwm_sum = 0, reverse_pwm_count = 0;
    static uint8_t forward_pwm_samples[50];
    static uint8_t reverse_pwm_samples[50];
    static uint8_t forward_pwm_second_half_samples[50];
    static uint8_t forward_pwm_second_half_count = 0;
    static uint8_t reverse_pwm_second_half_samples[50];
    static uint8_t reverse_pwm_second_half_count = 0;
    static unsigned long last_pwm_second_half_sample_time = 0;
    switch (calib.state) {
        case CALIB_IDLE:
            break;
        case CALIB_START:
            LOG_INFO("[CAL] Starting auto calibration routine...\n");
            calib.orig_stroke = stroke_time;
            stroke_time = 60000;
            calib.state = CALIB_MOVE_TO_CLOSED;
            break;
        case CALIB_MOVE_TO_CLOSED:
            LOG_INFO("[CAL] Moving to 0%% (closed)\n");
            targetPos = 0;
            motor_engaged = true;
            run_motor(0x01, stroke_time);
            calib.start_time = millis();
            calib.state = CALIB_WAIT_CLOSED;
            break;
        case CALIB_WAIT_CLOSED:
            if (readPosition() <= 10 || millis() - calib.start_time > 50000) {
                stop_motor();
                delay(500);
                calib.state = CALIB_FORWARD_STROKE_INIT;
            }
            break;
        case CALIB_FORWARD_STROKE_INIT: {
            LOG_INFO("[CAL] Starting forward stroke (0%% to 100%%)\n");
            if (calib.pos_samples) free(calib.pos_samples);
            if (calib.time_samples) free(calib.time_samples);
            calib.pos_samples = (int*)malloc(50 * sizeof(int)); // 5s at 100ms intervals
            calib.time_samples = (unsigned long*)malloc(50 * sizeof(unsigned long));
            calib.sample_count = 50;
            calib.state = CALIB_FORWARD_STROKE;
            calib.motor_start_time = millis();
            calib.movement_detected_time = 0;
            calib.target_reached_time = 0;
            calib.last_sample_time = millis();
            calib.samples_taken = 0;
            calib.baseline = readPositionRaw();
            calib.move_threshold = MOVEMENT_THRESHOLD; // Use configurable threshold
            targetPos = 1000;
            motor_engaged = true;
            run_motor(0x02, stroke_time);
            forward_pwm_sum = 0; forward_pwm_count = 0;
            memset(forward_pwm_samples, 0, sizeof(forward_pwm_samples));
            forward_pwm_second_half_count = 0;
            last_pwm_second_half_sample_time = 0;
            memset(forward_pwm_second_half_samples, 0, sizeof(forward_pwm_second_half_samples));
            break;
        }
        case CALIB_FORWARD_STROKE: {
            unsigned long now = millis();
            // Take samples every 100ms for first 5s
            if (calib.samples_taken < 50 && now - calib.last_sample_time >= 100) {
                calib.pos_samples[calib.samples_taken] = readPositionRaw();
                calib.time_samples[calib.samples_taken] = now - calib.motor_start_time;
                calib.last_sample_time = now;
                calib.samples_taken++;
            }
            int pos = readPositionRaw();
            if (!calib.movement_detected_time &&
                isConsecutiveMovementDetected(calib.pos_samples, calib.samples_taken, calib.baseline, calib.move_threshold, MOVEMENT_CONSECUTIVE_SAMPLES)) {
                calib.movement_detected_time = now;
                LOG_INFO("[CAL] Forward movement detected at %lu ms (consecutive samples)\n", now - calib.motor_start_time);
                if (dynamic_pwm_mode){
                    ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
                    LOG_INFO("[DYN_PWM] Running phase: PWM set to %u\n", dynamic_pwm_current);
                    dynamic_pwm_in_engage = false;
                    dynamic_pwm_last_pos = pos;
                    dynamic_pwm_last_pos_time = now;
                }
            }

            // --- Record PWM value for second half of stroke in dynamic mode every 100ms ---
            if (dynamic_pwm_mode && (now - calib.motor_start_time) >= (4000)) {
                if ((last_pwm_second_half_sample_time == 0 || now - last_pwm_second_half_sample_time >= 100) && forward_pwm_second_half_count < 50) {
                    forward_pwm_second_half_samples[forward_pwm_second_half_count++] = dynamic_pwm_current;
                    last_pwm_second_half_sample_time = now;
                }
            }
            if (readPosition() > 990) {
                calib.target_reached_time = now;
                stop_motor();
                delay(500);
                calib.state = CALIB_FORWARD_DONE;
            }
            break;
        }
        case CALIB_FORWARD_DONE: {
            // Calculate average PWM for second half of stroke
            if (dynamic_pwm_mode && forward_pwm_second_half_count > 0) {
                uint32_t pwm_sum = 0;
                for (uint8_t i = 0; i < forward_pwm_second_half_count; ++i) {
                    pwm_sum += forward_pwm_second_half_samples[i];
                }
                uint8_t avg_pwm = pwm_sum / forward_pwm_second_half_count;
                LOG_INFO("[CAL] Avg PWM (second half of stroke): %u\n", avg_pwm);
                calib.forward_pwm_avg = avg_pwm;
            } else {
                calib.forward_pwm_avg = 0;
            }
            LOG_INFO("[CAL] Forward stroke samples (up to 50):\n");
            for (int i = 0; i < calib.samples_taken; ++i) {
                LOG_DEBUG("[CAL] t=%lu pos=%d\n", calib.time_samples[i], calib.pos_samples[i]);
                delay(5);
            }
            // Post-process engage time
            {
                int idx = findEngageSample(calib.baseline, calib.pos_samples, calib.samples_taken, MOVEMENT_THRESHOLD, MOVEMENT_CONSECUTIVE_SAMPLES);
                if (idx >= 0) {
                    calib.movement_detected_time = calib.motor_start_time + calib.time_samples[idx];
                    LOG_INFO("[CAL] Forward engage detected at sample %d, t=%lu ms\n", idx, calib.time_samples[idx]);
                } else {
                    calib.movement_detected_time = calib.motor_start_time;
                    LOG_WARN("[CAL] Forward engage not robustly detected, using motor start time.\n");
                }
            }
            stop_motor();
            motor_running = false;
            motor_engaged = false;
            delay(300);
            calib.motor_start_time_1 = calib.motor_start_time;
            calib.movement_detected_time_1 = calib.movement_detected_time;
            calib.target_reached_time_1 = calib.target_reached_time;
            LOG_DEBUG("[CAL] Preparing for reverse stroke: motor stopped, flags reset.\n");
            calib.state = CALIB_REVERSE_STROKE_INIT;
            break;
        }
        case CALIB_REVERSE_STROKE_INIT: {
            LOG_INFO("[CAL] Starting reverse stroke (100%% to 0%%)\n");
            stop_motor();
            delay(200);
            motor_engaged = false;
            last_motor_dir = false;
            if (calib.rev_pos_samples) free(calib.rev_pos_samples);
            if (calib.rev_time_samples) free(calib.rev_time_samples);
            calib.rev_pos_samples = (int*)malloc(50 * sizeof(int));
            calib.rev_time_samples = (unsigned long*)malloc(50 * sizeof(unsigned long));
            calib.rev_samples_taken = 0;
            calib.rev_motor_start_time = millis();
            calib.rev_movement_detected_time = 0;
            calib.rev_target_reached_time = 0;
            calib.rev_last_sample_time = millis();
            calib.rev_baseline = readPositionRaw();
            calib.rev_move_threshold = MOVEMENT_THRESHOLD; // Use configurable threshold
            targetPos = 0;
            motor_engaged = true;
            run_motor(0x01, stroke_time);
            reverse_pwm_sum = 0; reverse_pwm_count = 0;
            memset(reverse_pwm_samples, 0, sizeof(reverse_pwm_samples));
            reverse_pwm_second_half_count = 0;
            last_pwm_second_half_sample_time = 0;
            memset(reverse_pwm_second_half_samples, 0, sizeof(reverse_pwm_second_half_samples));
            calib.state = CALIB_REVERSE_STROKE;
            break;
        }
        case CALIB_REVERSE_STROKE: {
            unsigned long now = millis();
            if (calib.rev_samples_taken < 50 && now - calib.rev_last_sample_time >= 100) {
                calib.rev_pos_samples[calib.rev_samples_taken] = readPositionRaw();
                calib.rev_time_samples[calib.rev_samples_taken] = now - calib.rev_motor_start_time;
                calib.rev_last_sample_time = now;
                calib.rev_samples_taken++;
            }
            int pos = readPositionRaw();
            if (!calib.rev_movement_detected_time &&
                isConsecutiveMovementDetected(calib.rev_pos_samples, calib.rev_samples_taken, calib.rev_baseline, calib.rev_move_threshold, MOVEMENT_CONSECUTIVE_SAMPLES)) {
                calib.rev_movement_detected_time = now;
                LOG_INFO("[CAL] Reverse movement detected at %lu ms (consecutive samples)\n", now - calib.rev_motor_start_time);
                if (dynamic_pwm_mode){
                    ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
                    LOG_INFO("[DYN_PWM] Running phase: PWM set to %u\n", dynamic_pwm_current);
                    dynamic_pwm_in_engage = false;
                    dynamic_pwm_last_pos = pos;
                    dynamic_pwm_last_pos_time = now;
                }
            }
            // --- Record PWM value for second half of stroke in dynamic mode every 100ms ---
            if (dynamic_pwm_mode && (now - calib.rev_motor_start_time) >= (4000)) {
                if ((last_pwm_second_half_sample_time == 0 || now - last_pwm_second_half_sample_time >= 100) && reverse_pwm_second_half_count < 50) {
                    reverse_pwm_second_half_samples[reverse_pwm_second_half_count++] = dynamic_pwm_current;
                    last_pwm_second_half_sample_time = now;
                }
            }
            if (readPosition() < 10) {
                calib.rev_target_reached_time = now;
                stop_motor();
                delay(500);
                calib.state = CALIB_REVERSE_DONE;
            }
            break;
        }
        case CALIB_REVERSE_DONE: {
            // Calculate average PWM for second half of stroke
            if (dynamic_pwm_mode && reverse_pwm_second_half_count > 0) {
                uint32_t pwm_sum = 0;
                for (uint8_t i = 0; i < reverse_pwm_second_half_count; ++i) {
                    pwm_sum += reverse_pwm_second_half_samples[i];
                }
                uint8_t avg_pwm = pwm_sum / reverse_pwm_second_half_count;
                LOG_INFO("[CAL] Avg PWM (second half of stroke): %u\n", avg_pwm);
                calib.reverse_pwm_avg = avg_pwm;
            } else {
                calib.reverse_pwm_avg = 0;
            }
            LOG_INFO("[CAL] Reverse stroke samples (up to 50):\n");
            for (int i = 0; i < calib.rev_samples_taken; ++i) {
                LOG_DEBUG("[CAL] t=%lu pos=%d\n", calib.rev_time_samples[i], calib.rev_pos_samples[i]);
                delay(5);
            }
            // Post-process reverse engage time
            {
                int idx = findEngageSample(calib.rev_baseline, calib.rev_pos_samples, calib.rev_samples_taken, MOVEMENT_THRESHOLD, MOVEMENT_CONSECUTIVE_SAMPLES);
                if (idx >= 0) {
                    calib.rev_movement_detected_time = calib.rev_motor_start_time + calib.rev_time_samples[idx];
                    LOG_INFO("[CAL] Reverse engage detected at sample %d, t=%lu ms\n", idx, calib.rev_time_samples[idx]);
                } else {
                    calib.rev_movement_detected_time = calib.rev_motor_start_time;
                    LOG_WARN("[CAL] Reverse engage not robustly detected, using motor start time.\n");
                }
            }
            calib.rev_motor_start_time_1 = calib.rev_motor_start_time;
            calib.rev_movement_detected_time_1 = calib.rev_movement_detected_time;
            calib.rev_target_reached_time_1 = calib.rev_target_reached_time;
            // Start second forward stroke
            calib.state = CALIB_SECOND_FORWARD_STROKE_INIT;
            break;
        }
        case CALIB_SECOND_FORWARD_STROKE_INIT: {
            LOG_INFO("[CAL] Second forward stroke (0%% to 100%%)\n");
            if (calib.pos_samples) free(calib.pos_samples);
            if (calib.time_samples) free(calib.time_samples);
            calib.pos_samples = (int*)malloc(50 * sizeof(int));
            calib.time_samples = (unsigned long*)malloc(50 * sizeof(unsigned long));
            calib.samples_taken = 0;
            calib.motor_start_time = millis();
            calib.movement_detected_time = 0;
            calib.target_reached_time = 0;
            calib.last_sample_time = millis();
            calib.baseline = readPositionRaw();
            calib.move_threshold = MOVEMENT_THRESHOLD; // Use configurable threshold
            targetPos = 1000;
            motor_engaged = true;
            run_motor(0x02, stroke_time);
            forward_pwm_sum = 0; forward_pwm_count = 0;
            memset(forward_pwm_samples, 0, sizeof(forward_pwm_samples));
            forward_pwm_second_half_count = 0;
            last_pwm_second_half_sample_time = 0;
            memset(forward_pwm_second_half_samples, 0, sizeof(forward_pwm_second_half_samples));
            calib.state = CALIB_SECOND_FORWARD_STROKE;
            break;
        }
        case CALIB_SECOND_FORWARD_STROKE: {
            unsigned long now = millis();
            // Take samples every 100ms for first 5s
            if (calib.samples_taken < 50 && now - calib.last_sample_time >= 100) {
                calib.pos_samples[calib.samples_taken] = readPositionRaw();
                calib.time_samples[calib.samples_taken] = now - calib.motor_start_time;
                calib.last_sample_time = now;
                calib.samples_taken++;
            }
            int pos = readPositionRaw();
            if (!calib.movement_detected_time &&
                isConsecutiveMovementDetected(calib.pos_samples, calib.samples_taken, calib.baseline, calib.move_threshold, MOVEMENT_CONSECUTIVE_SAMPLES)) {
                calib.movement_detected_time = now;
                LOG_INFO("[CAL] 2nd Forward movement detected at %lu ms (consecutive samples)\n", now - calib.motor_start_time);
                if (dynamic_pwm_mode){
                    ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
                    LOG_INFO("[DYN_PWM] Running phase: PWM set to %u\n", dynamic_pwm_current);
                    dynamic_pwm_in_engage = false;
                    dynamic_pwm_last_pos = pos;
                    dynamic_pwm_last_pos_time = now;
                }
            }
            // --- Record PWM value for second half of stroke in dynamic mode every 100ms ---
            if (dynamic_pwm_mode && (now - calib.motor_start_time) >= (4000)) {
                if ((last_pwm_second_half_sample_time == 0 || now - last_pwm_second_half_sample_time >= 100) && forward_pwm_second_half_count < 50) {
                    forward_pwm_second_half_samples[forward_pwm_second_half_count++] = dynamic_pwm_current;
                    last_pwm_second_half_sample_time = now;
                }
            }
            if (readPosition() > 990) {
                calib.target_reached_time = now;
                stop_motor();
                delay(500);
                calib.state = CALIB_SECOND_FORWARD_DONE;
            }
            break;
        }
        case CALIB_SECOND_FORWARD_DONE: {
            LOG_INFO("[CAL] 2nd Forward stroke samples (up to 50):\n");
            // Calculate average PWM for second half of stroke
            if (dynamic_pwm_mode && forward_pwm_second_half_count > 0) {
                uint32_t pwm_sum = 0;
                for (uint8_t i = 0; i < forward_pwm_second_half_count; ++i) {
                    pwm_sum += forward_pwm_second_half_samples[i];
                }
                uint8_t avg_pwm = pwm_sum / forward_pwm_second_half_count;
                LOG_INFO("[CAL] 2nd Avg PWM (second half of stroke): %u\n", avg_pwm);
                calib.forward_pwm_avg2 = avg_pwm;
            } else {
                calib.forward_pwm_avg2 = 0;
            }
            LOG_INFO("[CAL] Forward stroke samples (up to 50):\n");
            for (int i = 0; i < calib.samples_taken; ++i) {
                LOG_DEBUG("[CAL] t=%lu pos=%d\n", calib.time_samples[i], calib.pos_samples[i]);
                delay(5);
            }
            // Post-process engage time for 2nd forward
            {
                int idx = findEngageSample(calib.baseline, calib.pos_samples, calib.samples_taken, MOVEMENT_THRESHOLD, MOVEMENT_CONSECUTIVE_SAMPLES);
                if (idx >= 0) {
                    calib.movement_detected_time = calib.motor_start_time + calib.time_samples[idx];
                    LOG_INFO("[CAL] 2nd Forward engage detected at sample %d, t=%lu ms\n", idx, calib.time_samples[idx]);
                } else {
                    calib.movement_detected_time = calib.motor_start_time;
                    LOG_WARN("[CAL] 2nd Forward engage not robustly detected, using motor start time.\n");
                }
            }
            stop_motor();
            motor_running = false;
            motor_engaged = false;
            delay(300);
            LOG_DEBUG("[CAL] Preparing for 2nd reverse stroke: motor stopped, flags reset.\n");
            calib.state = CALIB_SECOND_REVERSE_STROKE_INIT;
            break;
        }
        case CALIB_SECOND_REVERSE_STROKE_INIT: {
            LOG_INFO("[CAL] Second reverse stroke (100%% to 0%%)\n");
            stop_motor();
            delay(200);
            motor_engaged = false;
            last_motor_dir = false;
            if (calib.rev_pos_samples) free(calib.rev_pos_samples);
            if (calib.rev_time_samples) free(calib.rev_time_samples);
            calib.rev_pos_samples = (int*)malloc(50 * sizeof(int));
            calib.rev_time_samples = (unsigned long*)malloc(50 * sizeof(unsigned long));
            calib.rev_samples_taken = 0;
            calib.rev_motor_start_time = millis();
            calib.rev_movement_detected_time = 0;
            calib.rev_target_reached_time = 0;
            calib.rev_last_sample_time = millis();
            calib.rev_baseline = readPositionRaw();
            calib.rev_move_threshold = MOVEMENT_THRESHOLD; // Use configurable threshold
            targetPos = 0;
            motor_engaged = true;
            run_motor(0x01, stroke_time);
            reverse_pwm_sum = 0; reverse_pwm_count = 0;
            memset(reverse_pwm_samples, 0, sizeof(reverse_pwm_samples));
            reverse_pwm_second_half_count = 0;
            last_pwm_second_half_sample_time = 0;
            memset(reverse_pwm_second_half_samples, 0, sizeof(reverse_pwm_second_half_samples));
            calib.state = CALIB_SECOND_REVERSE_STROKE;
            break;
        }
        case CALIB_SECOND_REVERSE_STROKE: {
            unsigned long now = millis();
            if (calib.rev_samples_taken < 50 && now - calib.rev_last_sample_time >= 100) {
                calib.rev_pos_samples[calib.rev_samples_taken] = readPositionRaw();
                calib.rev_time_samples[calib.rev_samples_taken] = now - calib.rev_motor_start_time;
                calib.rev_last_sample_time = now;
                calib.rev_samples_taken++;
            }
            int pos = readPositionRaw();
            if (!calib.rev_movement_detected_time &&
                isConsecutiveMovementDetected(calib.rev_pos_samples, calib.rev_samples_taken, calib.rev_baseline, calib.rev_move_threshold, MOVEMENT_CONSECUTIVE_SAMPLES)) {
                calib.rev_movement_detected_time = now;
                LOG_INFO("[CAL] 2nd Reverse movement detected at %lu ms (consecutive samples)\n", now - calib.rev_motor_start_time);
                if (dynamic_pwm_mode){
                    ledcWrite(EN_PWM_CHANNEL, dynamic_pwm_current);
                    LOG_INFO("[DYN_PWM] Running phase: PWM set to %u\n", dynamic_pwm_current);
                    dynamic_pwm_in_engage = false;
                    dynamic_pwm_last_pos = pos;
                    dynamic_pwm_last_pos_time = now;
                }
            }
            // --- Record PWM value for second half of stroke in dynamic mode every 100ms ---
            if (dynamic_pwm_mode && (now - calib.rev_motor_start_time) >= (4000)) {
                if ((last_pwm_second_half_sample_time == 0 || now - last_pwm_second_half_sample_time >= 100) && reverse_pwm_second_half_count < 50) {
                    reverse_pwm_second_half_samples[reverse_pwm_second_half_count++] = dynamic_pwm_current;
                    last_pwm_second_half_sample_time = now;
                }
            }
            if (readPosition() < 10) {
                calib.rev_target_reached_time = now;
                stop_motor();
                delay(500);
                calib.state = CALIB_SECOND_REVERSE_DONE;
            }
            break;
        }
        case CALIB_SECOND_REVERSE_DONE: {
            // Calculate average PWM for second half of stroke
            if (dynamic_pwm_mode && reverse_pwm_second_half_count > 0) {
                uint32_t pwm_sum = 0;
                for (uint8_t i = 0; i < reverse_pwm_second_half_count; ++i) {
                    pwm_sum += reverse_pwm_second_half_samples[i];
                }
                uint8_t avg_pwm = pwm_sum / reverse_pwm_second_half_count;
                LOG_INFO("[CAL] Avg PWM (second half of stroke): %u\n", avg_pwm);
                calib.reverse_pwm_avg2 = avg_pwm;
            } else {
                calib.reverse_pwm_avg2 = 0;
            }
            LOG_INFO("[CAL] 2nd Reverse stroke samples (up to 50):\n");
            for (int i = 0; i < calib.rev_samples_taken; ++i) {
                LOG_DEBUG("[CAL] t=%lu pos=%d\n", calib.rev_time_samples[i], calib.rev_pos_samples[i]);
                delay(5);
            }
            // Post-process engage time for 2nd reverse
            {
                int idx = findEngageSample(calib.rev_baseline, calib.rev_pos_samples, calib.rev_samples_taken, MOVEMENT_THRESHOLD, MOVEMENT_CONSECUTIVE_SAMPLES);
                if (idx >= 0) {
                    calib.rev_movement_detected_time = calib.rev_motor_start_time + calib.rev_time_samples[idx];
                    LOG_INFO("[CAL] 2nd Reverse engage detected at sample %d, t=%lu ms\n", idx, calib.rev_time_samples[idx]);
                } else {
                    calib.rev_movement_detected_time = calib.rev_motor_start_time;
                    LOG_WARN("[CAL] 2nd Reverse engage not robustly detected, using motor start time.\n");
                }
            }
            
            calib.state = CALIB_SAVE;
            break;
        }
        case CALIB_SAVE: {
            // First cycle
            calib.engage_time = calib.movement_detected_time_1 - calib.motor_start_time_1;
            calib.forward_stroke_time = calib.target_reached_time_1 - calib.movement_detected_time_1;
            calib.reverse_engage_time = calib.rev_movement_detected_time_1 - calib.rev_motor_start_time_1;
            calib.reverse_stroke_time = calib.rev_target_reached_time_1 - calib.rev_movement_detected_time_1;
            calib.disengage_time_calc = calib.engage_time / 2;
            // Second cycle
            calib.engage_time2 = calib.movement_detected_time - calib.motor_start_time;
            calib.forward_stroke_time2 = calib.target_reached_time - calib.movement_detected_time;
            calib.reverse_engage_time2 = calib.rev_movement_detected_time - calib.rev_motor_start_time;
            calib.reverse_stroke_time2 = calib.rev_target_reached_time - calib.rev_movement_detected_time;
            calib.disengage_time_calc2 = calib.engage_time2 / 2;
            // Average
            avg_stroke = (calib.forward_stroke_time + calib.forward_stroke_time2) / 2;
            avg_disengage = (calib.disengage_time_calc + calib.disengage_time_calc2) / 2;
            LOG_INFO("[CAL] 1st Engage: %lu ms, 1st Stroke: %lu ms\n", calib.engage_time, calib.forward_stroke_time);
            LOG_INFO("[CAL] 2nd Engage: %lu ms, 2nd Stroke: %lu ms\n", calib.engage_time2, calib.forward_stroke_time2);
            LOG_INFO("[CAL] Averaged stroke_time: %lu ms, disengage_time: %lu ms\n", avg_stroke, avg_disengage);
            disengage_time = avg_disengage;
            stroke_time = avg_stroke;
            // --- Store dynamic PWM calibration result ---
            if (dynamic_pwm_mode) {
                uint32_t sum = 0;
                uint8_t n = 0;
                if (calib.forward_pwm_avg > 0) { sum += calib.forward_pwm_avg; n++; }
                if (calib.reverse_pwm_avg > 0) { sum += calib.reverse_pwm_avg; n++; }
                if (calib.forward_pwm_avg2 > 0) { sum += calib.forward_pwm_avg2; n++; }
                if (calib.reverse_pwm_avg2 > 0) { sum += calib.reverse_pwm_avg2; n++; }
                if (n > 0) {
                    dynamic_pwm_target = sum / n;
                    LOG_INFO("[CAL] Set dynamic_pwm_target to %u (avg of 4 strokes)\n", dynamic_pwm_target);
                } else {
                    LOG_WARN("[CAL] No valid PWM samples for dynamic calibration!\n");
                }
            }
            saveTimingSettingsToFlash(stroke_time, disengage_time);
            LOG_INFO("[CAL] Calibration complete. New stroke_time=%lu ms, disengage_time=%lu ms\n", stroke_time, disengage_time);
            // Disengage actuator with new disengage_time
            disengage_motor();
            // Save dynamic_pwm_target to flash if in dynamic mode
            if (dynamic_pwm_mode) {
                if (prefs.begin("blindcfg", false)) {
                    prefs.putUChar("dyn_pwm_target", dynamic_pwm_target);
                    prefs.end();
                    LOG_INFO("[CAL] Saved dynamic_pwm_target=%u to flash\n", dynamic_pwm_target);
                } else {
                    LOG_WARN("[CAL] Could not open NVS to save dynamic_pwm_target\n");
                }
            }
            calib.state = CALIB_COMPLETE;
            break;
        }
        case CALIB_COMPLETE: {
            if (calib.pos_samples) { free(calib.pos_samples); calib.pos_samples = nullptr; }
            if (calib.time_samples) { free(calib.time_samples); calib.time_samples = nullptr; }
            if (calib.rev_pos_samples) { free(calib.rev_pos_samples); calib.rev_pos_samples = nullptr; }
            if (calib.rev_time_samples) { free(calib.rev_time_samples); calib.rev_time_samples = nullptr; }
            LOG_INFO("[CAL] Calibration routine finished.\n");
            // Set blind_state based on final position
            {
                uint16_t final_pos = readPosition();
                if (final_pos > 100) {
                    blind_state = STATE_OPEN;
                    LOG_INFO("[CAL] Final position: OPEN (%.1f%%)\n", final_pos / 10.0f);
                } else {
                    blind_state = STATE_CLOSED;
                    LOG_INFO("[CAL] Final position: CLOSED (%.1f%%)\n", final_pos / 10.0f);
                } 
            }
            calib.state = CALIB_IDLE;
            break;
        }
        case CALIB_ERROR: {
            LOG_ERROR("[CAL] Calibration failed due to error. Restoring disengage_time from flash.\n");
            loadTimingSettingsFromFlash(); // Restore disengage_time and other timing from flash
            calib.state = CALIB_COMPLETE;
            break;
        }
    }
}

void handleAutoCalibrate(AsyncWebServerRequest *request) {
    if (calib.state != CALIB_IDLE) {
        request->send(200, "text/html", "<html><body>Calibration is already running.<br><a href='/' >Back</a></body></html>");
        return;
       }
    startCalibration();
    request->send(200, "text/html", "<html><body>Auto calibration started. This may take up to a minute.<br><a href='/' >Back</a></body></html>");
}