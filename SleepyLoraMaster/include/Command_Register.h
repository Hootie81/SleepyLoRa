// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

#define TIME_SYNC_COMMAND 0xBB
#define ACK             0xAA
#define NAK             0x00
#define DEVICE_STATUS   0x01
#define BLIND_STATUS    0x02
#define BLIND_COMMAND   0x03
#define SET_PARAMETER   0x04
#define GET_PARAMETER   0x05
#define SET_LIMIT       0x06
#define GET_LIMIT       0x07
#define UPDATE_FIRMWARE 0x08

enum coverState{
  STATE_CLOSING=0x01,
  STATE_OPENING=0x02,
  STATE_CLOSED=0x03,
  STATE_OPEN=0x04,
  STATE_UNKNOWN=0xFF
};
// define some stuctures for each command. 8bytes available total. 
// action command will use a choose to copy the bytes to this struct before calling the individual command functions

struct timeSync_t
{
  uint32_t newTime;
  uint32_t oldTime;
} timeSyncPayload;

struct status_t
{
	uint16_t batteryVoltage;
	uint16_t wakeupCount;
	uint16_t awakeTime;
	uint8_t blindCount;
  uint8_t spare;
} device_status;

struct blind_t
{
	uint8_t blindNumber;
	uint8_t state;
	uint16_t position;
  uint16_t real_position;
  uint16_t batteryVoltage;
} blind_status;

struct blind_command_t
{
	uint8_t blindNumber;
	uint8_t set_state; // 0x00 = closed, 0x01 = open, 0x03 = stop, 0x04 = set position
	uint16_t set_position;
  uint32_t spare;
} blind_command;

struct parameter_t
{
	uint8_t blindNumber;
	uint8_t openSpeed;  //dutycycle
	uint8_t closeSpeed;
  uint8_t pwmFrequency; //frequency / 10, gives upto 25.5kHz in 100hz steps. 
  uint8_t openTime; // in seconds, gives upto 4min 15seconds open time.
  uint8_t closeTime;
  uint16_t disengageTime; //in ms / 10, gives 0 to 10.2seconds range
} blind_parameters;

struct limit_t
{
	uint8_t blindNumber;
	uint8_t limit;          //open or close limit for setting
  uint16_t alpha;         //alpha for position averaging
	uint16_t openValue;     //close value when getting
  uint16_t closeValue;    //open value when getting
} blind_limits;
