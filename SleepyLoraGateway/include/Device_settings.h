// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#define OTP_RANGE 5             // max time difference allowed for valid OTP, must be less than 15 due to return code
#define INTER_MESSAGE_DELAY 180  // minimum time after transmitting in ms

#define GATEWAY

#define LED_PIN 35
#define BATTERY_PIN 37
#define BATTERY_ADC_PIN 01

#define NTP_UPDATE_INTERVAL     600000 //in milliseconds

/************************* Button Settings *************************************/
#define CONFIG_BTN_INPUT        40
#define CONFIG_BTN_OUTPUT       42 // Used to help create a gnd - lazy only needed for prototype - remove later
#define DEBOUNCE_MS             50
#define LONG_PRESS_MS           2000


