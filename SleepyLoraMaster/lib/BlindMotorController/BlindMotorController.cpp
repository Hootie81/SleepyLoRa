// Macro to prepend a timestamp to Serial prints in BlindMotorController
#define BMOTOR_TIMESTAMP Serial.printf("[%lu] ", millis())
#include <Arduino.h>
#include <Preferences.h>
#include "BlindMotorController.h"
#include <Ticker.h>

// PID controller state and constants (add at the top of the file, after includes)
float pidIntegral = 0;
float pidLastError = 0;
float pidLastPos = 0; // For derivative on measurement
// PID gains (tune as needed)
const float Kp = 0.5f;   // Proportional gain (reduced)
const float Ki = 0.001f;  // Integral gain (reduced)
const float Kd = 0.05f;   // Derivative gain (reduced)
const float PID_INTEGRAL_MAX = 500.0f; // Anti-windup clamp
const int PID_DEADBAND = 5; // Deadband for error

// Static instance pointer for Ticker callback
BlindMotorController* BlindMotorController::instance = nullptr;

BlindMotorController::BlindMotorController(
    uint8_t inAPin_, uint8_t inBPin_, uint8_t enPin_, uint8_t pwmChannel_,
    uint8_t positionPin_, uint8_t vrefPin_
)
    : inAPin(inAPin_), inBPin(inBPin_), enPin(enPin_), pwmChannel(pwmChannel_),
      positionPin(positionPin_), vrefPin(vrefPin_),
      state(BlindMotorState::IDLE), stateEntryTime(0), targetPercent(0),
      lastPositionRaw(0), lastPositionPercent(0),
      openLimitRaw(3000), closedLimitRaw(100), engageTimeMs(500), disengageTimeMs(1000),
      moveLogHead(0), unjamAttempts(0), moveDirection(ActuatorDirection::EXTEND),
      pwmFrequency(1000) // <-- initialize here
{}

void BlindMotorController::begin() {
    BMOTOR_TIMESTAMP; Serial.println("[BlindMotor] begin() called");
    pinMode(inAPin, OUTPUT);
    pinMode(inBPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(inAPin, LOW);
    digitalWrite(inBPin, LOW);
    digitalWrite(enPin, LOW);

    // PWM setup (use stored frequency)
    ledcSetup(pwmChannel, pwmFrequency, 8);
    ledcAttachPin(inAPin, pwmChannel);
    ledcWrite(pwmChannel, 0);

    // ADC setup
    analogSetPinAttenuation(positionPin, ADC_11db);
    adcAttachPin(positionPin);
    adcAttachPin(vrefPin);

    // Load persistent settings
    loadLimits();
    prefs.begin("blindcfg", false);
    engageTimeMs = prefs.getUInt("engage_time", 500);
    disengageTimeMs = prefs.getUInt("disengage_time", 1000);
    prefs.end();
    loadAllSettings(); // Load all settings from NVS

    // Set static instance pointer for Ticker callback
    instance = this;
    // Start polling ticker (every 50ms)
    pollTicker.attach_ms(50, []() { if (BlindMotorController::instance) BlindMotorController::instance->update(); });
    // Start a faster ticker for EWMA sampling (e.g., every 10ms)
    ewmaTicker.attach_ms(5, []() { if (BlindMotorController::instance) BlindMotorController::instance->samplePositionEWMA(); });

    resetEngageDetection();
    // Initialize stall detection buffer
    for (uint8_t i = 0; i < STALL_WINDOW_SIZE; ++i) stallSamples[i] = readPositionRaw();
    stallSampleIdx = 0;
    stallLastCheckTime = millis();
}

void BlindMotorController::resetEngageDetection() {
    for (uint8_t i = 0; i < 50; ++i) engageHistory[i] = 0;
    engageHistoryIdx = 0;
    engageSamples[0] = engageSamples[1] = engageSamples[2] = engageSamples[3] = engageSamples[4] = readPositionRaw();
    for (uint8_t i = 5; i < 20; ++i) engageSamples[i] = 0;
    engageSampleIdx = 0;
    engageDetected = false;
    engageSampleFill = 0;
}

void BlindMotorController::update() {

    if (calibRunning) {
        runCalibrationStep();
    }

    uint16_t posRaw = readPositionRaw();
    uint16_t posPercent = calcPositionPercent(posRaw);

    // Update stall detection buffer
    stallSamples[stallSampleIdx] = posRaw;
    stallSampleIdx = (stallSampleIdx + 1) % stallSampleCount;

    switch (state) {
    case BlindMotorState::IDLE:
        // Nothing to do
        break;
    case BlindMotorState::ENGAGING: {
        engageSamples[engageSampleIdx] = posRaw;
        engageSampleIdx = (engageSampleIdx + 1) % 5;
        // Store in engageHistory (circular buffer)
        this->engageHistory[this->engageHistoryIdx] = posRaw;
        this->engageHistoryIdx = (this->engageHistoryIdx + 1) % 50;
        engageSampleFill++; // Now a member, not static
        // Only allow detection after at least engageSampleCount new samples
        bool canDetect = (engageSampleFill >= engageSampleCount);
        bool allInDirection = false;
        if (canDetect) {
            // Check that all (engageSampleCount-1) consecutive deltas are in the correct direction
            allInDirection = true;
            for (int i = 0; i < engageSampleCount - 1; ++i) {
                int idxA = (engageSampleIdx + i) % engageSampleCount;
                int idxB = (engageSampleIdx + i + 1) % engageSampleCount;
                int16_t delta = (int16_t)engageSamples[idxB] - (int16_t)engageSamples[idxA];
                if (moveDirection == ActuatorDirection::EXTEND) { // EXTEND: expect positive movement
                    if (delta < engageDetectThreshold) {
                        allInDirection = false;
                        break;
                    }
                } else { // RETRACT: expect negative movement
                    if (delta > -engageDetectThreshold) {
                        allInDirection = false;
                        break;
                    }
                }
            }
            if (allInDirection) {
                BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][ENGAGING][DEBUG] Last %u samples: ", engageSampleCount);
                for (int i = 0; i < engageSampleCount; ++i) {
                    int idx = (engageSampleIdx + i) % engageSampleCount;
                    Serial.printf("%u ", engageSamples[idx]);
                }
                Serial.println();
            }
        }
        if (allInDirection) {
            engageDetected = true;
            engageDetectTimeMs = millis() - stateEntryTime;
            if (!canDetect) {
                BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][ENGAGING][WARN] Engage detected before 5 new samples!");
            }
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][ENGAGING] Engage detected! engageDetectTimeMs=%lu ms\n", engageDetectTimeMs);
            BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][ENGAGING][HISTORY] Samples this cycle (table):");
            Serial.println("Idx\tValue\tDelta");
            for (uint8_t i = 0; i < engageSampleFill; ++i) {
                uint8_t idx = (this->engageHistoryIdx + 50 - engageSampleFill + i) % 50;
                uint16_t val = this->engageHistory[idx];
                int16_t delta = 0;
                if (i > 0) {
                    uint8_t prevIdx = (this->engageHistoryIdx + 50 - engageSampleFill + i - 1) % 50;
                    delta = (int16_t)val - (int16_t)this->engageHistory[prevIdx];
                }
                Serial.printf("%2u\t%5u\t%+5d\n", i, val, delta);
            }
            setState(BlindMotorState::MOVING);
        }
        // Optionally, fallback to time-based if not detected after a timeout
        else if (canDetect && (millis() - stateEntryTime > engageTimeMs)) {
            engageDetectTimeMs = engageTimeMs;
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][ENGAGING] Timeout fallback. engageDetectTimeMs=%lu ms\n", engageDetectTimeMs);
            BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][ENGAGING][HISTORY] Samples this cycle (table):");
            Serial.println("Idx\tValue\tDelta");
            for (uint8_t i = 0; i < engageSampleFill; ++i) {
                uint8_t idx = (this->engageHistoryIdx + 50 - engageSampleFill + i) % 50;
                uint16_t val = this->engageHistory[idx];
                int16_t delta = 0;
                if (i > 0) {
                    uint8_t prevIdx = (this->engageHistoryIdx + 50 - engageSampleFill + i - 1) % 50;
                    delta = (int16_t)val - (int16_t)this->engageHistory[prevIdx];
                }
                Serial.printf("%2u\t%5u\t%+5d\n", i, val, delta);
            }
            setState(BlindMotorState::MOVING);
        }
        break;
    }
    case BlindMotorState::MOVING: {
        static bool firstMovingDebug = true;
        if (firstMovingDebug) {
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][MOVING] Entered MOVING state. posRaw=%u, targetRaw=%u, moveDirection=%s\n", posRaw, targetRaw, moveDirection == ActuatorDirection::EXTEND ? "EXTEND" : "RETRACT");
            firstMovingDebug = false;
            pidIntegral = 0; // Reset PID state on entry
            pidLastError = 0;
            pidLastPos = posRaw;
        }
        uint16_t posRaw = readPositionRaw();
        int32_t errorRaw = (int32_t)targetRaw - (int32_t)posRaw;
        // Calculate move time based on distance to move
        float percentDelta = fabs((float)targetPercent - (float)moveStartPercent) / 1000.0f; // <-- Use moveStartPercent
        uint32_t moveTimeMsForThisMove = (uint32_t)(moveTimeMs * percentDelta);
        if (moveTimeMsForThisMove < 100) moveTimeMsForThisMove = 100; // Minimum move time

        // Time-based motion profile: adjust PWM so the motor reaches the target at the end of moveTimeMs
        uint32_t now = millis();
        uint32_t elapsed = now - moveStartTime;
        uint32_t moveDuration = moveTimeMsForThisMove; // already calculated above
        float progress = (float)elapsed / (float)moveDuration;
        if (progress > 1.0f) progress = 1.0f;
        float expectedRaw = (float)moveStartRaw + ((float)(targetRaw - moveStartRaw)) * progress;
        // Direction-aware error calculation
        float error;
        if (moveDirection == ActuatorDirection::EXTEND) { // EXTEND
            error = (float)expectedRaw - (float)posRaw;
        } else { // RETRACT
            error = (float)posRaw - (float)expectedRaw;
        }
        // PID control (smoothed and clamped)
        float dt = 0.05f; // 50ms update interval (matches pollTicker)
        // --- Use adjustable PID parameters ---
        float pidOutput = Kp * error + Ki * pidIntegral + Kd * ((error - pidLastError) / dt);
        pidIntegral += error * dt;
        if (pidIntegral > integralMax) pidIntegral = integralMax;
        if (pidIntegral < integralMin) pidIntegral = integralMin;
        float rawDerivative = (error - pidLastError) / dt;
        // Low-pass filter derivative
        static float lastDerivative = 0.0f;
        lastDerivative = derivAlpha * rawDerivative + (1.0f - derivAlpha) * lastDerivative;
        float derivative = lastDerivative;
        if (derivative > derivMax) derivative = derivMax;
        if (derivative < derivMin) derivative = derivMin;
        pidOutput = Kp * error + Ki * pidIntegral + Kd * derivative;
        pidLastError = error;
        // Deadband: if error is small, just use minimum PWM
        int pwm = pwmMin;
        if (fabs(error) > errorDeadband) {
            pwm = pwmMin + (int)pidOutput; // No fabs, only add positive correction
        }
        if (pwm > pwmMax) pwm = pwmMax;
        if (pwm < pwmMin) pwm = pwmMin;
        // --- Smooth the PWM output with EMA ---
        static float filteredPwm = 0.0f;
        if (filteredPwm == 0.0f) filteredPwm = (float)pwm; // Initialize on first run
        else filteredPwm = pwmAlpha * (float)pwm + (1.0f - pwmAlpha) * filteredPwm;
        ledcWrite(pwmChannel, 255 - (int)(filteredPwm + 0.5f));
        // Throttle the [BlindMotor][MOVING] log to print only every 5th call instead of every update.
        static uint16_t movePrintCounter = 0;
        movePrintCounter++;
        if (movePrintCounter >= 10) {
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][MOVING] posRaw=%u, targetRaw=%u, errorRaw=%ld, pwm=%u (inverted=%u)\n", posRaw, targetRaw, errorRaw, pwm, 255 - pwm);
            movePrintCounter = 0;
        }
        // Debug print for control variables
        static uint16_t debugPrintCounter = 0;
        debugPrintCounter++;
        if (debugPrintCounter >= 10) {
            BMOTOR_TIMESTAMP;
            // Direction-aware track status
            const char* trackStatus = "ON_TRACK";
            if (moveDirection == ActuatorDirection::EXTEND) {
                if (error > 5.0f) trackStatus = "BEHIND";
                else if (error < -5.0f) trackStatus = "AHEAD";
            } else {
                if (error < -5.0f) trackStatus = "BEHIND";
                else if (error > 5.0f) trackStatus = "AHEAD";
            }
            Serial.printf("[BlindMotor][CTRLDBG] moveDirection=%s posRaw=%u expectedRaw=%.1f error=%.1f pidOut=%.1f integral=%.1f deriv=%.1f pwm=%d (min=%d max=%d) moveStartRaw=%u targetRaw=%u elapsed=%lu/%lu ms (%.1f%%) [%s]\n",
                moveDirection == ActuatorDirection::EXTEND ? "EXTEND" : "RETRACT", posRaw, expectedRaw, error, pidOutput, pidIntegral, derivative, pwm, pwmMin, pwmMax, moveStartRaw, targetRaw,
                (unsigned long)elapsed, (unsigned long)moveDuration, 100.0f * progress, trackStatus);
            debugPrintCounter = 0;
        }
        // Check if we've reached or passed the target (direction-robust)
        if ((moveDirection == ActuatorDirection::EXTEND && posRaw >= targetRaw) ||
            (moveDirection == ActuatorDirection::RETRACT && posRaw <= targetRaw)) {
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][MOVING] Target reached or passed. posRaw=%u, targetRaw=%u\n", posRaw, targetRaw);
            stopMotor();
            setState(BlindMotorState::DISENGAGING);
            firstMovingDebug = true; // Reset for next time
        }
        // Robust jam detection: require N consecutive samples in the correct direction to reset jam timer
        static uint16_t jamSamples[20] = {0}; // Support up to 20 samples
        static uint8_t jamSampleIdx = 0;
        static uint32_t jamCountdownExpiry = 0;
        static const uint32_t JAM_TIMEOUT_MS = 1000; // Jam triggers if not reset in this time
        // Shift in new sample
        jamSamples[jamSampleIdx] = posRaw;
        jamSampleIdx = (jamSampleIdx + 1) % stallSampleCount;
        bool allInDirection = true;
        for (int i = 0; i < stallSampleCount - 1; ++i) {
            int idxA = (jamSampleIdx + i) % stallSampleCount;
            int idxB = (jamSampleIdx + i + 1) % stallSampleCount;
            int16_t delta = (int16_t)jamSamples[idxB] - (int16_t)jamSamples[idxA];
            if (moveDirection == ActuatorDirection::EXTEND) { // EXTEND: expect positive movement
                if (delta < stallDeltaThreshold) {
                    allInDirection = false;
                    break;
                }
            } else { // RETRACT: expect negative movement
                if (delta > -stallDeltaThreshold) {
                    allInDirection = false;
                    break;
                }
            }
        }
        if (allInDirection) {
            jamCountdownExpiry = millis() + JAM_TIMEOUT_MS;
        }
        if (millis() > jamCountdownExpiry && millis() - stateEntryTime > JAM_TIMEOUT_MS) {
            // Only trigger jam if we are 'behind' the expected position
            bool isBehind = false;
            if (moveDirection == ActuatorDirection::EXTEND) {
                if (error > 5.0f) isBehind = true;
            } else {
                if (error < -5.0f) isBehind = true;
            }
            if (isBehind) {
                BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][MOVING] Jam detected (direction-aware, N consecutive samples not seen, and behind target)");
                stopMotor();
                setState(BlindMotorState::DISENGAGING);
            } else {
                // Not behind, so do not trigger jam
                jamCountdownExpiry = millis() + JAM_TIMEOUT_MS; // Give more time
            }
        }
        break;
    }
    case BlindMotorState::DISENGAGING:
        if (millis() - stateEntryTime > disengageTimeMs) {
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][DISENGAGING] Disengage time elapsed, stopping motor and going IDLE\n");
            disengageTimeUsedMs = disengageTimeMs;
            stopMotor();
            // Log the move here, after disengage time is set
            logMove(BlindMoveResult::SUCCESS, lastPositionRaw, readPositionRaw(), lastPositionPercent, readPositionPercent(), millis() - stateEntryTime, engageDetectTimeMs, disengageTimeUsedMs);
            setState(BlindMotorState::IDLE);
        }
        break;
    case BlindMotorState::UNJAM_ATTEMPT_1:
    case BlindMotorState::UNJAM_ATTEMPT_2:
        // Unjam logic: reverse for disengageTimeMs, then try to resume move
        if (millis() - stateEntryTime > disengageTimeMs) {
            if (unjamAttempts < 2) {
                unjamAttempts++;
                // Resume original move after unjam attempt
                engageMotor(moveDirection);
                resetEngageDetection();
                setState(BlindMotorState::ENGAGING);
            } else {
                setState(BlindMotorState::FAULT);
            }
        }
        break;
    case BlindMotorState::FAULT:
        stopMotor();
        logMove(BlindMoveResult::JAMMED, lastPositionRaw, posRaw, lastPositionPercent, posPercent, millis() - stateEntryTime, engageDetectTimeMs, disengageTimeUsedMs);
        setState(BlindMotorState::DISENGAGING);
        break;
    case BlindMotorState::ERROR:
        stopMotor();
        setState(BlindMotorState::DISENGAGING);
        break;
    }
    lastPositionRaw = posRaw;
    lastPositionPercent = posPercent;
}

void BlindMotorController::commandMove(uint16_t targetPercent_) {
    Serial.printf("[BlindMotor][commandMove] ENTRY: targetPercent_=%u\n", targetPercent_);
    if (targetPercent_ > 1000) targetPercent_ = 1000;
    targetPercent = targetPercent_;
    Serial.printf("[BlindMotor][commandMove] closedLimitRaw=%u, openLimitRaw=%u, targetPercent=%u\n", closedLimitRaw, openLimitRaw, targetPercent);
    int32_t delta = (int32_t)openLimitRaw - (int32_t)closedLimitRaw;
    int32_t scaled = (delta * (int32_t)targetPercent) / 1000;
    targetRaw = closedLimitRaw + scaled;
    Serial.printf("[BlindMotor][commandMove] delta=%ld, scaled=%ld, targetRaw=%u\n", delta, scaled, targetRaw);
    // Determine actuator direction: EXTEND (increase raw) or RETRACT (decrease raw)
    uint16_t currentRaw = readPositionRaw();
    Serial.printf("[BlindMotor][commandMove] currentRaw=%u\n", currentRaw);
    if (targetRaw > currentRaw) {
        moveDirection = ActuatorDirection::EXTEND;
        Serial.println("[BlindMotor][commandMove] moveDirection: EXTEND");
    } else if (targetRaw < currentRaw) {
        moveDirection = ActuatorDirection::RETRACT;
        Serial.println("[BlindMotor][commandMove] moveDirection: RETRACT");
    } else {
        Serial.println("[BlindMotor][commandMove] Already at target, no move needed.");
        return;
    }
    startMove();
    Serial.println("[BlindMotor][commandMove] EXIT");
}

void BlindMotorController::abortMove() {
    stopMotor();
    disengageMotor(moveDirection);
    setState(BlindMotorState::IDLE);
    logMove(BlindMoveResult::ABORTED, lastPositionRaw, readPositionRaw(), lastPositionPercent, readPositionPercent(), millis() - stateEntryTime, engageDetectTimeMs, disengageTimeUsedMs);
}

void BlindMotorController::setOpenLimit(uint16_t raw) {
    openLimitRaw = raw;
    saveLimits();
}
void BlindMotorController::setClosedLimit(uint16_t raw) {
    closedLimitRaw = raw;
    saveLimits();
}
void BlindMotorController::loadLimits() {
    prefs.begin("blindcfg", false);
    openLimitRaw = prefs.getUShort("open_limit", 3000);
    closedLimitRaw = prefs.getUShort("closed_limit", 100);
    prefs.end();
}
void BlindMotorController::saveLimits() {
    prefs.begin("blindcfg", false);
    prefs.putUShort("open_limit", openLimitRaw);
    prefs.putUShort("closed_limit", closedLimitRaw);
    prefs.end();
}
void BlindMotorController::setEngageTime(uint32_t ms) {
    engageTimeMs = ms;
    saveTiming();
}
void BlindMotorController::setDisengageTime(uint32_t ms) {
    disengageTimeMs = ms;
    saveTiming();
}
void BlindMotorController::saveTiming() {
    prefs.begin("blindcfg", false);
    prefs.putUInt("engage_time", engageTimeMs);
    prefs.putUInt("disengage_time", disengageTimeMs);
    prefs.putUInt("move_time_ms", moveTimeMs);
    prefs.end();
}

void BlindMotorController::loadAllSettings() {
    prefs.begin("blindcfg", false);
    // PWM
    engagePwm = prefs.getUChar("engage_pwm", 255);
    disengagePwm = prefs.getUChar("disengage_pwm", 255);
    pwmMin = prefs.getUChar("move_pwm_min", 40);
    pwmMax = prefs.getUChar("move_pwm_max", 255);
    pwmFrequency = prefs.getUInt("pwm_freq", 1000);
    // Config
    engageDetectThreshold = prefs.getUShort("engage_thresh", 10);
    stallDeltaThreshold = prefs.getUShort("stall_thresh", 3);
    moveTimeMs = prefs.getUInt("move_time_ms", 20000);
    // PID/Control
    Kp = prefs.getFloat("pid_kp", 5.0f);
    Ki = prefs.getFloat("pid_ki", 0.001f);
    Kd = prefs.getFloat("pid_kd", 0.05f);
    errorDeadband = prefs.getFloat("pid_deadband", 4.0f);
    integralMin = prefs.getFloat("pid_imin", -100.0f);
    integralMax = prefs.getFloat("pid_imax", 100.0f);
    derivMin = prefs.getFloat("pid_dmin", -30.0f);
    derivMax = prefs.getFloat("pid_dmax", 30.0f);
    derivAlpha = prefs.getFloat("pid_dalpha", 0.12f);
    pwmAlpha = prefs.getFloat("pid_palpha", 0.25f);
    stallSampleCount = prefs.getUChar("stall_samples", 3);
    engageSampleCount = prefs.getUChar("engSampCnt", 5); // New setting for engage sample count (short key)
    prefs.end();
}

void BlindMotorController::saveAllSettings() {
    prefs.begin("blindcfg", false);
    // PWM
    prefs.putUChar("engage_pwm", engagePwm);
    prefs.putUChar("disengage_pwm", disengagePwm);
    prefs.putUChar("move_pwm_min", pwmMin);
    prefs.putUChar("move_pwm_max", pwmMax);
    prefs.putUInt("pwm_freq", pwmFrequency);
    // Config
    prefs.putUShort("engage_thresh", engageDetectThreshold);
    prefs.putUShort("stall_thresh", stallDeltaThreshold);
    prefs.putUInt("move_time_ms", moveTimeMs);
    // PID/Control
    prefs.putFloat("pid_kp", Kp);
    prefs.putFloat("pid_ki", Ki);
    prefs.putFloat("pid_kd", Kd);
    prefs.putFloat("pid_deadband", errorDeadband);
    prefs.putFloat("pid_imin", integralMin);
    prefs.putFloat("pid_imax", integralMax);
    prefs.putFloat("pid_dmin", derivMin);
    prefs.putFloat("pid_dmax", derivMax);
    prefs.putFloat("pid_dalpha", derivAlpha);
    prefs.putFloat("pid_palpha", pwmAlpha);
    prefs.putUChar("stall_samples", stallSampleCount);
    prefs.putUChar("engSampCnt", engageSampleCount); // Save engage sample count (short key)
    prefs.end();
}

uint16_t BlindMotorController::readPositionRaw() const {
    return ewmaRaw;
}
uint16_t BlindMotorController::readPositionPercent() const {
    return calcPositionPercent(readPositionRaw());
}
uint16_t BlindMotorController::calcPositionPercent(uint16_t raw) const {
    if (openLimitRaw == closedLimitRaw) return 0;
    int32_t delta = (int32_t)openLimitRaw - (int32_t)closedLimitRaw;
    int32_t percent = ((int32_t)raw - closedLimitRaw) * 1000 / delta;
    if (delta < 0) percent = ((int32_t)raw - closedLimitRaw) * 1000 / delta; // Handles reversed mapping
    if (percent < 0) percent = 0;
    if (percent > 1000) percent = 1000;
    return (uint16_t)percent;
}
uint16_t BlindMotorController::readADC(uint8_t pin) {
    // Ratiometric: (position / vref) * 4095
    uint32_t pos = analogRead(pin);
    uint32_t vref = analogRead(vrefPin);
    if (vref == 0){
        vref = 1;
        //Serial.printf("[BlindMotor]vref error\n");
    }
    return (uint16_t)((pos * 4095UL) / vref);
}

#define POS_ALPHA 0.4f  // Smoothing factor (0.0-1.0), lower = smoother
volatile uint16_t ewmaRaw = 0;
volatile bool ewmaInitialized = false;

// Call this from a timer interrupt or ticker at a higher rate (e.g., every 10ms)
void BlindMotorController::samplePositionEWMA() {
    uint16_t sample = readADC(positionPin);
    if (!ewmaInitialized) {
        ewmaRaw = sample;
        ewmaInitialized = true;
    } else {
        ewmaRaw = (uint16_t)((1.0f - POS_ALPHA) * ewmaRaw + POS_ALPHA * sample);
    }
}



void BlindMotorController::startMove() {
    unjamAttempts = 0;
    resetEngageDetection();
    setState(BlindMotorState::ENGAGING);
    moveStartTime = millis();
    moveStartError = (int16_t)targetPercent - (int16_t)readPositionPercent();
    moveStartRaw = readPositionRaw();
    moveStartPercent = lastPositionPercent;
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][startMove] moveStartRaw=%u, targetRaw=%u, moveDirection=%s\n", moveStartRaw, targetRaw, moveDirection == ActuatorDirection::EXTEND ? "EXTEND" : "RETRACT");
}

void BlindMotorController::engageMotor(ActuatorDirection dir) {
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][engageMotor] Called. dir=%s, engagePwm=%u\n", dir == ActuatorDirection::EXTEND ? "EXTEND" : "RETRACT", engagePwm);
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][engageMotor] Setting PWM to %u\n", 255 - engagePwm);
    digitalWrite(enPin, HIGH);
    if (dir == ActuatorDirection::EXTEND) {
        // EXTEND: inAPin HIGH, PWM on inBPin
        digitalWrite(inBPin, HIGH);
        digitalWrite(inAPin, LOW);
        ledcDetachPin(inBPin);
        ledcAttachPin(inAPin, pwmChannel);
        ledcWrite(pwmChannel, 255 - engagePwm);
        BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][engageMotor] EXTEND: inBPin HIGH, inAPin LOW, PWM on inAPin");
    } else {
        // RETRACT: inBPin HIGH, PWM on inAPin
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
        ledcDetachPin(inAPin);
        ledcAttachPin(inBPin, pwmChannel);
        ledcWrite(pwmChannel, 255 - engagePwm);
        BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][engageMotor] RETRACT: inAPin HIGH, inBPin LOW, PWM on inBPin");
    }
}

void BlindMotorController::disengageMotor(ActuatorDirection dir) {
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][disengageMotor] Called. dir=%s, disengagePwm=%u\n", dir == ActuatorDirection::EXTEND ? "EXTEND" : "RETRACT", disengagePwm);
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][disengageMotor] Setting PWM to %u\n", 255 - disengagePwm);
    digitalWrite(enPin, HIGH);
    if (dir == ActuatorDirection::RETRACT) {
        // RETRACT: inAPin HIGH, PWM on inBPin (disengage PWM)
        digitalWrite(inBPin, HIGH);
        digitalWrite(inAPin, LOW);
        ledcDetachPin(inBPin);
        ledcAttachPin(inAPin, pwmChannel);
        ledcWrite(pwmChannel, 255 - disengagePwm);
        BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][disengageMotor] RETRACT: inBPin HIGH, inAPin LOW, PWM on inAPin");
    } else {
        // EXTEND: inBPin HIGH, PWM on inAPin (disengage PWM)
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
        ledcDetachPin(inAPin);
        ledcAttachPin(inBPin, pwmChannel);
        ledcWrite(pwmChannel, 255 - disengagePwm);
        BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][disengageMotor] EXTEND: inAPin HIGH, inBPin LOW, PWM on inBPin");
    }
}

void BlindMotorController::stopMotor() {
    ledcWrite(pwmChannel, 0);
    digitalWrite(inAPin, LOW);
    digitalWrite(inBPin, LOW);
    digitalWrite(enPin, LOW);
}

void BlindMotorController::handleStall() {
    // Try unjam
    setState(unjamAttempts == 0 ? BlindMotorState::UNJAM_ATTEMPT_1 : BlindMotorState::UNJAM_ATTEMPT_2);
    tryUnjam();
}

void BlindMotorController::tryUnjam() {
    // Reverse direction briefly
    engageMotor(moveDirection == ActuatorDirection::EXTEND ? ActuatorDirection::RETRACT : ActuatorDirection::EXTEND);
    stateEntryTime = millis();
}

void BlindMotorController::setState(BlindMotorState newState) {
    const char* oldStateStr = "UNKNOWN";
    const char* newStateStr = "UNKNOWN";
    switch (state) {
        case BlindMotorState::IDLE: oldStateStr = "IDLE"; break;
        case BlindMotorState::MOVING: oldStateStr = "MOVING"; break;
        case BlindMotorState::ENGAGING: oldStateStr = "ENGAGING"; break;
        case BlindMotorState::DISENGAGING: oldStateStr = "DISENGAGING"; break;
        case BlindMotorState::UNJAM_ATTEMPT_1: oldStateStr = "UNJAM_ATTEMPT_1"; break;
        case BlindMotorState::UNJAM_ATTEMPT_2: oldStateStr = "UNJAM_ATTEMPT_2"; break;
        case BlindMotorState::FAULT: oldStateStr = "FAULT"; break;
        case BlindMotorState::ERROR: oldStateStr = "ERROR"; break;
    }
    switch (newState) {
        case BlindMotorState::IDLE: newStateStr = "IDLE"; break;
        case BlindMotorState::MOVING: newStateStr = "MOVING"; break;
        case BlindMotorState::ENGAGING: newStateStr = "ENGAGING"; break;
        case BlindMotorState::DISENGAGING: newStateStr = "DISENGAGING"; break;
        case BlindMotorState::UNJAM_ATTEMPT_1: newStateStr = "UNJAM_ATTEMPT_1"; break;
        case BlindMotorState::UNJAM_ATTEMPT_2: newStateStr = "UNJAM_ATTEMPT_2"; break;
        case BlindMotorState::FAULT: newStateStr = "FAULT"; break;
        case BlindMotorState::ERROR: newStateStr = "ERROR"; break;
    }
    Serial.printf("[BlindMotor] State change: %s -> %s\n", oldStateStr, newStateStr);
    // Only call engage/disengage if the state is actually changing
    if (state != newState) {
        state = newState;
        stateEntryTime = millis();
        if (newState == BlindMotorState::ENGAGING) {
            engageMotor(moveDirection);
        } else if (newState == BlindMotorState::DISENGAGING) {
            disengageMotor(moveDirection);
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][DISENGAGING] Entered DISENGAGING state at %lu ms, will disengage for %lu ms\n", stateEntryTime, disengageTimeMs);
        }
    }
}

void BlindMotorController::logMove(BlindMoveResult result, uint16_t fromRaw, uint16_t toRaw, uint16_t fromPercent, uint16_t toPercent, uint32_t duration, uint32_t engageDetectTime, uint32_t disengageTimeUsed) {
    BlindMoveLogEntry& entry = moveLog[moveLogHead];
    entry.timestamp = millis();
    entry.fromPos = fromRaw;
    entry.toPos = toRaw;
    entry.fromPercent = fromPercent;
    entry.toPercent = toPercent;
    entry.duration = duration;
    entry.result = result;
    entry.engageDetectTime = engageDetectTime;
    entry.disengageTimeUsed = disengageTimeUsed;
    moveLogHead = (moveLogHead + 1) % BLIND_MOVE_LOG_SIZE;
}

void BlindMotorController::dumpMoveLogToSerial() {
    BMOTOR_TIMESTAMP; Serial.println("Blind Move Log:");
    for (uint8_t i = 0; i < BLIND_MOVE_LOG_SIZE; ++i) {
        const BlindMoveLogEntry& e = moveLog[(moveLogHead + i) % BLIND_MOVE_LOG_SIZE];
        if (e.duration == 0) continue;
        BMOTOR_TIMESTAMP; Serial.printf("[%lu] %u->%u (%u%%->%u%%) %lums Result:%u EngageDetect:%lums DisengageUsed:%lums\n",
            e.timestamp, e.fromPos, e.toPos, e.fromPercent / 10, e.toPercent / 10, e.duration, (uint8_t)e.result, e.engageDetectTime, e.disengageTimeUsed);
    }
}

uint8_t BlindMotorController::getLastMoveStatus() const {
    if (state == BlindMotorState::MOVING || state == BlindMotorState::ENGAGING || state == BlindMotorState::DISENGAGING)
        return MOVE_STATUS_MOVING;
    // Check last move result in move log
    uint8_t idx = (moveLogHead + BLIND_MOVE_LOG_SIZE - 1) % BLIND_MOVE_LOG_SIZE;
    BlindMoveResult result = moveLog[idx].result;
    if (result == BlindMoveResult::SUCCESS)
        return MOVE_STATUS_OK;
    if (result == BlindMoveResult::JAMMED || result == BlindMoveResult::STALLED || result == BlindMoveResult::ERROR || result == BlindMoveResult::ABORTED)
        return MOVE_STATUS_TIMEOUT;
    return MOVE_STATUS_OK; // Default to OK if unknown
}

void BlindMotorController::reportStatus() {
    const char* stateStr = "UNKNOWN";
    switch (state) {
        case BlindMotorState::IDLE: stateStr = "IDLE"; break;
        case BlindMotorState::MOVING: stateStr = "MOVING"; break;
        case BlindMotorState::ENGAGING: stateStr = "ENGAGING"; break;
        case BlindMotorState::DISENGAGING: stateStr = "DISENGAGING"; break;
        case BlindMotorState::UNJAM_ATTEMPT_1: stateStr = "UNJAM_ATTEMPT_1"; break;
        case BlindMotorState::UNJAM_ATTEMPT_2: stateStr = "UNJAM_ATTEMPT_2"; break;
        case BlindMotorState::FAULT: stateStr = "FAULT"; break;
        case BlindMotorState::ERROR: stateStr = "ERROR"; break;
    }
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor] State: %s, Position: %u (%u%%), Target: %u%%\n",
        stateStr, readPositionRaw(), readPositionPercent() / 10, targetPercent / 10);
}

bool BlindMotorController::isMotorRunning() const {
    return state != BlindMotorState::IDLE;
}

void BlindMotorController::log(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    BMOTOR_TIMESTAMP; Serial.print("[BlindMotor] ");
    Serial.println(buf);
}

void BlindMotorController::startCalibrationSequence() {
    calibPosIdx = 0;
    calibRunning = true;
    calibStepInProgress = false;
    calibPhase = CalibrationPhase::ENGAGE_DISENGAGE;
        // Clear the relevant move log entries for calibration
    for (uint8_t i = 0; i < CALIBRATION_STEPS; ++i) {
        int8_t idx = (int8_t)moveLogHead - CALIBRATION_STEPS + i;
        if (idx < 0) idx += BLIND_MOVE_LOG_SIZE;
        moveLog[idx] = {};
    }
    BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][CALIB] Calibration: started");
}

void BlindMotorController::runCalibrationStep() {
    if (!calibRunning) return;
    if (calibPhase == CalibrationPhase::ENGAGE_DISENGAGE) {
        if (!calibStepInProgress) {
            if (calibPosIdx >= CALIBRATION_STEPS) {
                calibStepInProgress = false;
                calibPhase = CalibrationPhase::COMPLETE;
                BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][CALIB] Calibration: complete");
                analyzeCalibrationResults();
                calibRunning = false;
                return;
            }
            BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][CALIB] Moving to %u%%\n", calibrationPositions[calibPosIdx]/10);
            this->commandMove(calibrationPositions[calibPosIdx]);
            calibStepStart = millis();
            calibStepInProgress = true;
        } else {
            if (this->state == BlindMotorState::IDLE || millis() - calibStepStart > 20000) {
                int8_t idx = (int8_t)moveLogHead - 1;
                if (idx < 0) idx += BLIND_MOVE_LOG_SIZE;
                const BlindMoveLogEntry& entry = moveLog[idx];
                if (entry.engageDetectTime < 200) {
                    disengageTimeMs /= 2;
                    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][CALIB] Disengage time too high (engageDetectTime=%lums), halved to %lums. Restarting calibration...\n", entry.engageDetectTime, disengageTimeMs);
                    saveTiming();
                    calibPosIdx = 0;
                    calibStepInProgress = false;
                    return;
                }
                calibPosIdx++;
                calibStepInProgress = false;
            }
        }
    }
    // Remove PID_TUNING phase
}

void BlindMotorController::analyzeCalibrationResults() {
    // Analyze last X moves in the move log
    uint32_t engageSum = 0, disengageSum = 0;
    uint8_t count = 0;
    bool disengageTooHigh = false;
    for (int8_t i = 0; i < CALIBRATION_STEPS; ++i) {
        int8_t idx = (int8_t)moveLogHead - CALIBRATION_STEPS + i;
        if (idx < 0) idx += BLIND_MOVE_LOG_SIZE;
        const BlindMoveLogEntry& entry = moveLog[idx];
        // Skip entries that are not valid (not completed move)
        if (entry.engageDetectTime == 0 || entry.disengageTimeUsed == 0 || entry.engageDetectTime == engageTimeMs) {
            BMOTOR_TIMESTAMP; Serial.printf("[CALIB][DEBUG] Move %d: SKIPPED (engageDetectTime=%lu, disengageTimeUsed=%lu, engageTimeMs=%lu)\n", i, entry.engageDetectTime, entry.disengageTimeUsed, engageTimeMs);
            continue;
        }
        BMOTOR_TIMESTAMP; Serial.printf("[CALIB][DEBUG] Move %d: engageDetectTime=%lu, disengageTimeUsed=%lu\n", i, entry.engageDetectTime, entry.disengageTimeUsed);
        engageSum += entry.engageDetectTime;
        disengageSum += entry.disengageTimeUsed;
        // Heuristic: if engageDetectTime is very low (e.g. < 4*poll interval), flag as too high
        if (entry.engageDetectTime < 200) disengageTooHigh = true;
        count++;
    }
    if (count == 0) {
        BMOTOR_TIMESTAMP; Serial.println("[BlindMotor][CALIB] No valid calibration moves to analyze! Setting engageTimeMs to 5000ms for safety.");
        engageTimeMs = 5000;
        disengageTimeMs = 5000;
        saveTiming();
        return;
    }
    uint32_t avg = (engageSum + disengageSum) / (2 * count);
    engageTimeMs = avg * 2 + 500;
    disengageTimeMs = avg;
    if (disengageTooHigh) {
        disengageTimeMs /= 2;
        BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][CALIB] Disengage time appeared too high, halved to %lums\n", disengageTimeMs);
    }
    BMOTOR_TIMESTAMP; Serial.printf("[BlindMotor][CALIB] Calibration results: engage=%lums, disengage=%lums (avg of %d valid moves)\n", engageTimeMs, disengageTimeMs, count);
    saveTiming();
}

void BlindMotorController::prepareForDeepSleep() {
    // Detach all tickers to prevent ISR during deep sleep
    pollTicker.detach();
    ewmaTicker.detach();
    // Stop the motor for safety
    stopMotor();
}

CoverState BlindMotorController::getCoverStateRaw() const {
    uint16_t posPercent = readPositionPercent(); // 0-1000
    const uint16_t percentThreshold = 20; // 2% of 1000
    // Determine which limit is physically open/closed
    bool openIsHigh = openLimitRaw > closedLimitRaw;
    bool movingTowardOpen = false, movingTowardClosed = false;
    if (openIsHigh) {
        movingTowardOpen = (moveDirection == ActuatorDirection::EXTEND);
        movingTowardClosed = (moveDirection == ActuatorDirection::RETRACT);
    } else {
        movingTowardOpen = (moveDirection == ActuatorDirection::RETRACT);
        movingTowardClosed = (moveDirection == ActuatorDirection::EXTEND);
    }
    switch (state) {
        case BlindMotorState::IDLE:
            if (posPercent <= percentThreshold)
                return CoverState::CLOSED;
            else 
                return CoverState::OPEN;
        case BlindMotorState::MOVING:
        case BlindMotorState::ENGAGING:
        case BlindMotorState::DISENGAGING:
            if (movingTowardOpen)
                return CoverState::OPENING;
            else if (movingTowardClosed)
                return CoverState::CLOSING;
            else
                return CoverState::UNKNOWN;
        default:
            return CoverState::UNKNOWN;
    }
}

// Return the cover state as a uint8_t for protocol/reporting use
uint8_t BlindMotorController::getCoverState() const {
    return static_cast<uint8_t>(getCoverStateRaw());
}
