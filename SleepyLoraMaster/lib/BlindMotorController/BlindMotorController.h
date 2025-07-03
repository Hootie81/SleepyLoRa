#pragma once
#include <stdint.h>
#include <Ticker.h>
#include <Arduino.h>
#include <Preferences.h>

// Internal enum for actuator/blind state (mirrors protocol, but self-contained)
enum class CoverState : uint8_t {
    UNKNOWN = 0,
    CLOSING = 1,
    OPENING = 2,
    CLOSED = 3,
    OPEN = 4,
};

// Maximum number of moves to log
#define BLIND_MOVE_LOG_SIZE 30

// State machine states
enum class BlindMotorState {
    IDLE,
    MOVING,
    ENGAGING,
    DISENGAGING,
    UNJAM_ATTEMPT_1,
    UNJAM_ATTEMPT_2,
    FAULT,
    ERROR
};

// Move result codes
enum class BlindMoveResult : uint8_t {
    NONE = 0,
    SUCCESS = 1,
    STALLED = 2,
    JAMMED = 3,
    ABORTED = 4,
    ERROR = 5
};

// Add new enum for actuator direction
enum class ActuatorDirection {
    EXTEND, // always increases raw value
    RETRACT // always decreases raw value
};

struct BlindMoveLogEntry {
    uint32_t timestamp;       // millis() at move start
    uint16_t fromPos;         // raw ADC
    uint16_t toPos;           // raw ADC
    uint16_t fromPercent;     // 0-1000
    uint16_t toPercent;       // 0-1000
    uint32_t duration;        // ms
    BlindMoveResult result;   // result code
    uint32_t engageDetectTime;    // ms taken to detect engagement
    uint32_t disengageTimeUsed;   // ms used for disengage on this move
};

#define MOVE_STATUS_OK      0x01
#define MOVE_STATUS_TIMEOUT 0x02
#define MOVE_STATUS_MOVING  0x03

class BlindMotorController {
public:
    // Constructor: all pins must be provided
    BlindMotorController(
        uint8_t inAPin, uint8_t inBPin, uint8_t enPin, uint8_t pwmChannel,
        uint8_t positionPin, uint8_t vrefPin
    );

    // Call in setup()
    void begin();

    // Main update loop, call from loop()
    void update();

    // Command a move to percent (0-1000 = 0-100.0%)
    void commandMove(uint16_t targetPercent);

    void abortMove();

    void setOpenLimit(uint16_t raw);
    void setClosedLimit(uint16_t raw);

    void loadLimits();

    uint16_t readPositionPercent() const;

    uint16_t readPositionRaw() const;

    void dumpMoveLogToSerial();

    uint8_t getLastMoveStatus() const;

    void reportStatus();
    bool isMotorRunning() const;
    
    uint16_t getOpenLimit() const { return openLimitRaw; }
    uint16_t getClosedLimit() const { return closedLimitRaw; }
    void setEngageTime(uint32_t ms);
    void setDisengageTime(uint32_t ms);
    uint32_t getEngageTime() const { return engageTimeMs; }
    uint32_t getDisengageTime() const { return disengageTimeMs; }

    void runCalibrationSequence();
    void startCalibrationSequence();
    void runCalibrationStep();
    void analyzeCalibrationResults();

    BlindMotorState getState() const { return state; }

    // Set engage detection threshold
    void setEngageDetectThreshold(uint16_t threshold) { engageDetectThreshold = threshold; saveAllSettings(); }
    uint16_t getEngageDetectThreshold() const { return engageDetectThreshold; }
    // Set stall delta threshold
    void setStallDeltaThreshold(uint16_t threshold) { stallDeltaThreshold = threshold; saveAllSettings(); }
    uint16_t getStallDeltaThreshold() const { return stallDeltaThreshold; }
    // Set/get move time
    void setMoveTimeMs(uint32_t ms) { moveTimeMs = ms; saveAllSettings(); }
    uint32_t getMoveTimeMs() const { return moveTimeMs; }
    // Set/get engage PWM
    void setEngagePwm(uint8_t pwm) { engagePwm = pwm; saveAllSettings(); }
    uint8_t getEngagePwm() const { return engagePwm; }
    // Set/get disengage PWM
    void setDisengagePwm(uint8_t pwm) { disengagePwm = pwm; saveAllSettings(); }
    uint8_t getDisengagePwm() const { return disengagePwm; }
    // Set/get minimum PWM
    void setPwmMin(uint8_t pwm) { pwmMin = pwm; saveAllSettings(); }
    uint8_t getPwmMin() const { return pwmMin; }
    // Set/get maximum PWM
    void setPwmMax(uint8_t pwm) { pwmMax = pwm; saveAllSettings(); }
    uint8_t getPwmMax() const { return pwmMax; }
    // Set/get PWM frequency
    void setPwmFrequency(uint32_t freq) {
        // DRV8833: recommended 100Hz to 20kHz
        if (freq < 100) freq = 100;
        if (freq > 20000) freq = 20000;
        pwmFrequency = freq;
        ledcSetup(pwmChannel, pwmFrequency, 8); // Immediately apply new frequency
        saveAllSettings();
    }
    uint32_t getPwmFrequency() const { return pwmFrequency; }
    // Load/save all settings
    void loadAllSettings();
    void saveAllSettings();

    // EWMA filter sampling and access
    void samplePositionEWMA();

    void prepareForDeepSleep();

    CoverState getCoverStateRaw() const;
    // Return the cover state as a uint8_t for protocol/reporting use
    uint8_t getCoverState() const;

    // Return a user-friendly string for the current blind state
    String getStatusString() const {
        switch (state) {
            case BlindMotorState::IDLE: return "Idle";
            case BlindMotorState::MOVING: return "Moving";
            case BlindMotorState::ENGAGING: return "Engaging";
            case BlindMotorState::DISENGAGING: return "Disengaging";
            case BlindMotorState::UNJAM_ATTEMPT_1: return "Unjam 1";
            case BlindMotorState::UNJAM_ATTEMPT_2: return "Unjam 2";
            case BlindMotorState::FAULT: return "Fault";
            case BlindMotorState::ERROR: return "Error";
            default: return "Unknown";
        }
    }
    // Return a user-friendly string for the calibration state
    String getCalibrationStateString() const {
        if (calibRunning) {
            if (calibPhase == CalibrationPhase::ENGAGE_DISENGAGE) return "Engage/Disengage";
            if (calibPhase == CalibrationPhase::COMPLETE) return "Complete";
            return "Running";
        } else {
            return "Idle";
        }
    }

public:
    // --- PID controller parameters (adjustable) ---
    void setKp(float val) { Kp = val; saveAllSettings(); }
    float getKp() const { return Kp; }
    void setKi(float val) { Ki = val; saveAllSettings(); }
    float getKi() const { return Ki; }
    void setKd(float val) { Kd = val; saveAllSettings(); }
    float getKd() const { return Kd; }
    void setErrorDeadband(float val) { errorDeadband = val; saveAllSettings(); }
    float getErrorDeadband() const { return errorDeadband; }
    void setIntegralMin(float val) { integralMin = val; saveAllSettings(); }
    float getIntegralMin() const { return integralMin; }
    void setIntegralMax(float val) { integralMax = val; saveAllSettings(); }
    float getIntegralMax() const { return integralMax; }
    void setDerivMin(float val) { derivMin = val; saveAllSettings(); }
    float getDerivMin() const { return derivMin; }
    void setDerivMax(float val) { derivMax = val; saveAllSettings(); }
    float getDerivMax() const { return derivMax; }
    void setDerivAlpha(float val) { derivAlpha = val; saveAllSettings(); }
    float getDerivAlpha() const { return derivAlpha; }
    void setPwmAlpha(float val) { pwmAlpha = val; saveAllSettings(); }
    float getPwmAlpha() const { return pwmAlpha; }
    // Adjustable stall sample count
    void setStallSampleCount(uint8_t count) { stallSampleCount = count; saveAllSettings(); }
    uint8_t getStallSampleCount() const { return stallSampleCount; }

    // Set/get engage sample count (for engagement detection)
    void setEngageSampleCount(uint8_t count) {
        if (count < 2) count = 2;
        if (count > 20) count = 20;
        engageSampleCount = count;
        saveAllSettings();
    }
    uint8_t getEngageSampleCount() const { return engageSampleCount; }

    uint8_t getPwmChannel() const { return pwmChannel; }

private:
    // Pin assignments
    uint8_t inAPin, inBPin, enPin, pwmChannel, positionPin, vrefPin;

    // Static instance pointer for Ticker callback
    static BlindMotorController* instance;

    // State machine
    BlindMotorState state;
    uint32_t stateEntryTime;
    uint16_t targetPercent;
    uint16_t lastPositionRaw;
    uint16_t lastPositionPercent;
    ActuatorDirection moveDirection; // EXTEND or RETRACT for current move

    // Persistent settings
    uint16_t openLimitRaw, closedLimitRaw;
    uint32_t engageTimeMs, disengageTimeMs;

    // Move log
    BlindMoveLogEntry moveLog[BLIND_MOVE_LOG_SIZE];
    uint8_t moveLogHead;

    // NVS
    Preferences prefs;

    // Ticker for polling position
    Ticker pollTicker;
    Ticker ewmaTicker;
    // Advanced engage detection
    uint16_t engageSamples[20] = {0}; // Now up to 20 samples, default 5
    uint8_t engageSampleIdx = 0;
    uint8_t engageSampleCount = 5; // User-configurable, default 5
    uint16_t engageDetectThreshold = 5; // Default threshold, can be tuned
    bool engageDetected = false;
    void resetEngageDetection();

    // Stall detection filter
    static constexpr uint8_t STALL_WINDOW_SIZE = 8;
    uint16_t stallSamples[STALL_WINDOW_SIZE] = {0};
    uint8_t stallSampleIdx = 0;
    uint32_t stallLastCheckTime = 0;
    uint16_t stallDeltaThreshold = 3; // Default, can be tuned

    // Per-move timing for logging
    uint32_t engageDetectTimeMs = 0;
    uint32_t disengageTimeUsedMs = 0;

    // Time-aware move control
    uint32_t moveStartTime = 0;
    uint32_t moveTimeMs = 2000; // Default move time (ms)
    int16_t moveStartError = 0;
    uint16_t targetRaw = 0; // Raw ADC target for moves
    uint16_t moveStartRaw; // Position at start of move for time-profile controller
    uint16_t moveStartPercent = 0; // Percent at start of move for time-profile controller

    // Non-blocking calibration state
    static constexpr uint8_t CALIBRATION_STEPS = 4;
    const uint16_t calibrationPositions[CALIBRATION_STEPS] = {500, 0, 1000, 500};
    bool calibRunning = false;
    bool calibStepInProgress = false;
    uint8_t calibPosIdx = 0;
    uint32_t calibStepStart = 0;

    uint32_t avgEngageTime = 0, avgDisengageTime = 0;

    // Engage movement diagnostics
    uint16_t engageHistory[50] = {0}; // Increased to 50
    uint8_t engageHistoryIdx = 0;
    uint8_t engageSampleFill = 0; // Track how many samples have been filled since entering ENGAGING

    // EWMA filter state
    volatile uint16_t ewmaRaw = 0;
    volatile bool ewmaInitialized = false;

    // Calibration phase state machine for calibration
    enum class CalibrationPhase {
        ENGAGE_DISENGAGE,
        COMPLETE
    };
    CalibrationPhase calibPhase = CalibrationPhase::ENGAGE_DISENGAGE;

    // --- PID controller parameters (adjustable) ---
    float Kp = 5.0f;
    float Ki = 0.001f;
    float Kd = 0.05f;
    float errorDeadband = 4.0f;
    float integralMin = -100.0f;
    float integralMax = 100.0f;
    float derivMin = -30.0f;
    float derivMax = 30.0f;
    float derivAlpha = 0.12f;
    float pwmAlpha = 0.25f;
    uint8_t stallSampleCount = 8; // Default, adjustable

    // Internal helpers
    void setState(BlindMotorState newState);
    void startMove();
    void stopMotor();
    void engageMotor(ActuatorDirection dir);
    void disengageMotor(ActuatorDirection dir);
    void handleStall();
    void logMove(BlindMoveResult result, uint16_t fromRaw, uint16_t toRaw, uint16_t fromPercent, uint16_t toPercent, uint32_t duration, uint32_t engageDetectTime, uint32_t disengageTimeUsed);

    // ADC helpers
    uint16_t readADC(uint8_t pin);

    // Ratiometric position calculation
    uint16_t calcPositionPercent(uint16_t raw) const;

    // Persistent storage
    void saveLimits();
    void saveTiming();
    void loadAllSettingsInternal();
    void saveAllSettingsInternal();

    // Fault/unjam logic
    uint8_t unjamAttempts;
    void tryUnjam();

    // Logging
    void log(const char *fmt, ...);

    // Proportional controller for PWM
    uint8_t pwmMin = 90; // Minimum PWM to ensure movement
    uint8_t pwmMax = 255; // Maximum PWM

    // PWM for engage/disengage
    uint8_t engagePwm = 255;
    uint8_t disengagePwm = 255;
    // PWM frequency (persistent, adjustable)
    uint32_t pwmFrequency = 1000;
};