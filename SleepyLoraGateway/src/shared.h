// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#pragma once
#include "CircularBuffer.h"
#include <stdint.h>

namespace data {
    typedef struct {
        uint32_t nodeToId;
        uint8_t OTP[3];
        uint8_t command;
        uint32_t nodeFromId;
        uint8_t payload[8];
    } RXpacket;

    typedef struct {
        time_t OTPtime;
        uint32_t nodeToId;
        uint8_t command;
        uint8_t payload[8];
    } TXpacket;
}

extern CircularBuffer<data::TXpacket, 10> TXbuffer;
extern struct timeval tv_now;
