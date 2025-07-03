// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#define TIME_TO_SLEEP  60      // Time ESP32 will go to sleep (in seconds).
#define MAX_AWAKE_TIME 600000   // max time the device can be awake
#define UPDATE_PERIOD 60        // how often to send updates
#define OTP_RANGE 5             // max time difference allowed for valid OTP 
#define MOVING_UPDATE_PERIOD 1500

#define POS_ALPHA 0.7
#define BATT_ALPHA 0.7
#define EN_PWM_CHANNEL 0 // PWM channel for enable pin

#define VBAT_BOOT_MS 1500   // Time to allow slaves to boot up before communicating with them

#define BATTERY_PIN 37
#define BATTERY_ADC_PIN 01
#define VEXT_PIN 36
#define LED_PIN 35
#define POSITION_PIN 7
#define POSITION_REF_PIN 15 // GPIO15: Pot top (reference)
#define IN_A_PIN 17 //
#define IN_B_PIN 18 //
#define EN_PIN 4

