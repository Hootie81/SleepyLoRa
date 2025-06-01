// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#pragma once

#include <stdint.h>
#include <vector>
#include <Preferences.h>

struct Blind {
    uint32_t deviceId;
    uint8_t blindNumber;
};

extern std::vector<Blind> blindsList;
extern Preferences blindsPrefs;

void loadBlindsFromFlash();
void saveBlindsToFlash();
void addBlindIfNew(uint32_t deviceId, uint8_t blindNumber);
void publishHADiscoveryConfig(const Blind& b);
void subscribeToAllBlinds();
