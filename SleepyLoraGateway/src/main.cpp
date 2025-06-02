// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Chris Huitema

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
#include <set>
#include "LoRa_settings.h"
#include "Device_settings.h"
#include "Command_Register.h"
#include "logger.h"
#include <LittleFS.h>

#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <Adafruit_SleepyDog.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <vector>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <map>

#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"

#define MQTT_DEBUG
#define INITIAL_VALUE 254

#include "config.h"
#include "blinds.h"

// Forward declaration for master device discovery config
void publishMasterDeviceDiscoveryConfig(uint32_t deviceId);

#include "mqtt.h"
#include "webserver.h"

#define CONFIG_BUTTON_PIN 0 // This is likely a leftover and not used, will be replaced by Device_settings.h defines

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

/** creating an object of AES128 class */
AES128 aes128;
byte encrypted[16];  //temp storage
byte decrypted[16];  //temp storage

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


/** The Data message will be sent by the node */
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

}  //end namespace

struct loraPacket {
  uint32_t nodeToId;
  uint8_t dataEncrypted[16];
} loraTXpacket, tmploraRXpacket;

struct dataPacket {
  uint8_t OTP[3];
  uint8_t command;
  uint32_t nodeFromId;
  uint8_t payload[8];
} dataToENC, dataDEC;

struct PendingTx {
    unsigned long lastSendTime = 0;
    uint8_t retryCount = 0;
    bool awaitingAck = false;
    uint32_t awaitingNodeId = 0;
} pendingTx;

CircularBuffer<data::RXpacket, 10> RXbuffer;
CircularBuffer<data::TXpacket, 10> TXbuffer;

/** The node ID, created from MAC of ESP32 */
uint32_t deviceID = 0;

/** LoRa error timeout */
time_t loraTimeout = 0;

/** LoRa TX time */
time_t loraTXTime = 0;

/** CAD repeat counter */
uint8_t cadRepeat = 0;

bool txComplete = true;

/** The wakeup time */
time_t wakeup = 0;
time_t start_enc = 0;
time_t lastTX = 0;

RTC_DATA_ATTR uint16_t wakeCount;
RTC_DATA_ATTR time_t awakeTime;
RTC_DATA_ATTR time_t lastSend;

int countdownMS = 0;

// Variables for config button handling
volatile bool configButtonPressed = false;
volatile unsigned long buttonPressTime = 0;
volatile unsigned long lastInterruptTime = 0;
bool buttonHandled = false;
// End of variables for config button handling

struct timeval tv_now;

bool shouldClearConfig = false;
bool shouldStartConfig = false;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Use defaults, update after config loaded
TOTP* otp = nullptr;
extern Ticker gatewayStatusTicker;

// Track last seen time for each blind
std::map<std::pair<uint32_t, uint8_t>, unsigned long> blindLastSeen;
Ticker blindAvailabilityTicker;

// Track published discovery config for master devices
std::set<uint32_t> publishedMasterDiscovery;

/** declare functions */
void radioSetup(void);
uint8_t checkOTP(uint8_t OTP[3]);
void sendPacket(void);
void decodePacket(void);
void connectToWifi();
void onWifiEvent(WiFiEvent_t event);
void checkBlindAvailability();
void handleConfigButton(); // Forward declaration
void IRAM_ATTR onConfigButtonPress(); // Forward declaration for ISR

Ticker logFlushTicker;

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);

  // Setup config button pins
  pinMode(CONFIG_BTN_OUTPUT, OUTPUT);
  digitalWrite(CONFIG_BTN_OUTPUT, LOW); // Set output low to act as ground for button
  pinMode(CONFIG_BTN_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CONFIG_BTN_INPUT), onConfigButtonPress, FALLING);


  Serial.println("############################################");
  Serial.println("#                                          #");
  Serial.println("#            Gateway Bootup                #");
  Serial.println("#                                          #");
  Serial.println("############################################");
  Serial.println();
  countdownMS = Watchdog.enable(60000); // Default watchdog
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  Serial.println();
  if (!LittleFS.begin()) {
      Serial.println("LittleFS mount failed! Formatting...");
      LittleFS.format();
      if (!LittleFS.begin()) {
          Serial.println("LittleFS mount failed after format! Logging disabled.");
          // Optionally: set a flag to disable logging
      }
  }
  // Create node ID
  uint8_t deviceMac[8];
  BoardGetUniqueId(deviceMac);
  deviceID = 0;
  deviceID += (uint32_t)deviceMac[2];
  deviceID += (uint32_t)deviceMac[3] << 8;
  deviceID += (uint32_t)deviceMac[4] << 16;
  deviceID += (uint32_t)deviceMac[5] << 24;

  checkConfigButton();
  bool configLoaded = loadConfig();
  if (configLoaded) {
      timeClient.setPoolServerName(config.ntp_server);
      timeClient.setTimeOffset(config.ntp_offset);
      if (otp) delete otp;
      otp = new TOTP(config.hmacKey, 10, 1);
  }
  if (!configLoaded) {
      // NVS not found, extend watchdog to 15 minutes for configuration
      countdownMS = Watchdog.enable(900000); 
      Serial.print("NVS not found. Extended watchdog to ");
      Serial.print(countdownMS, DEC);
      Serial.println(" milliseconds for configuration.");
      WiFi.disconnect(true);
      startConfigPortalAP();
      setupWebServer();
      while (true) {
          checkConfigButton();
          if (shouldClearConfig) {
              clearConfig();
              ESP.restart();
          }
          delay(100);
      }
  }
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  connectToWifi();
  setupWebServer();
  setupMqttClient();
  loadBlindsFromFlash();
  mqttClient.setKeepAlive(60); // 60 seconds
  Watchdog.reset();

  Serial.println();

  aes128.setKey(config.aes_key, 16);

  Serial.printf("Device ID %08X using frequency %.1f MHz\r\n", deviceID, (double)(config.rf_frequency / 1000000.0));

  // Setup the radio
  radioSetup();
  digitalWrite(LED_PIN, LOW);
  lastTX = millis();

  gatewayStatusTicker.attach(30, publishGatewayStatus);
  blindAvailabilityTicker.attach_ms(60000, checkBlindAvailability); // check every minute
  logFlushTicker.attach(900, [](){ Logger.flush(); }); // flush log every 15 minutes
  Serial.println("Setup finished");
  Logger.begin();
  Logger.log("INFO", "BOOT", "Gateway boot, DeviceID=%08X, Freq=%.1fMHz", deviceID, (double)(config.rf_frequency / 1000000.0));
}

void loop() {
  Watchdog.reset();
  checkConfigButton(); // Initial check before loop
  if (shouldStartConfig) {
      WiFi.disconnect(true);
      startConfigPortalAP();
      // Web server is already running, so portal is available on AP too
      while (true) {
          checkConfigButton();
          if (shouldClearConfig) {
              clearConfig();
              ESP.restart();
          }
          delay(100);
      }
  }
  if (shouldClearConfig) {
      clearConfig();
      ESP.restart();
  }

  handleConfigButton(); // Handle button state changes in the main loop

  static unsigned long lastMqttCheck = 0;
  static unsigned long mqttDisconnectedSince = 0;
  if (millis() - lastMqttCheck > 5000) { // check every 5 seconds
    lastMqttCheck = millis();
    if (!mqttClient.connected()) {
      if (mqttDisconnectedSince == 0) mqttDisconnectedSince = millis();
      if ((millis() - mqttDisconnectedSince) > 60000) { // 1 minute
        Serial.println("WARNING: MQTT has been disconnected for over 1 minute!");
        Logger.log("WARN", "MQTT", "Disconnected >1min");
      }
      connectToMqtt();
    } else {
      mqttDisconnectedSince = 0;
    }
  }

  static time_t lastNtp = 0;
  if (WiFi.isConnected()) {
    if (timeClient.update()) {
      time_t timeSet = timeClient.getEpochTime();
      if (abs((long)(timeSet - lastNtp)) > 1) {
        Logger.log("INFO", "NTP", "Time synced: %ld (%s)", timeSet, timeClient.getFormattedTime().c_str());
        lastNtp = timeSet;
      }
      tv_now.tv_sec = timeSet;
      settimeofday(&tv_now, NULL);
      
      Serial.print("NTP time synced, time set to: ");
      Serial.print(timeSet);
      Serial.print(" : ");
      Serial.println(timeClient.getFormattedTime());
    }
  }
  if (pendingTx.awaitingAck && !TXbuffer.isEmpty()) {
    if (millis() - pendingTx.lastSendTime > 750) { // timeout
        if (pendingTx.retryCount < 3) {
            Serial.println("No ACK, resending...");
            Logger.log("WARN", "LORA_ACK", "No ACK, resending, node=0x%08X, retry=%u", pendingTx.awaitingNodeId, pendingTx.retryCount+1);
            sendPacket(); // Resend the same packet
            pendingTx.lastSendTime = millis();
            pendingTx.retryCount++;
        } else {
            Serial.println("No ACK after 3 tries, discarding message.");
            Logger.log("ERROR", "LORA_ACK", "No ACK after 3 tries, discarding, node=0x%08X", pendingTx.awaitingNodeId);
            TXbuffer.shift();
            pendingTx.awaitingAck = false;
            pendingTx.retryCount = 0;
            pendingTx.awaitingNodeId = 0;
        }
    }
  }
  if (!RXbuffer.isEmpty()) {
    digitalWrite(LED_PIN, HIGH);
    decodePacket();
    digitalWrite(LED_PIN, LOW);
  }
  if (!TXbuffer.isEmpty() && !pendingTx.awaitingAck && (millis() - lastTX > INTER_MESSAGE_DELAY)) {
    digitalWrite(LED_PIN, HIGH);
    sendPacket();
    digitalWrite(LED_PIN, LOW);
  }
}

void connectToWifi() { WiFi.begin(config.wifi_ssid, config.wifi_pass); }

void onWifiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Logger.log("INFO", "WIFI", "Connected, IP=%s", WiFi.localIP().toString().c_str());
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    connectToMqtt();
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Logger.log("WARN", "WIFI", "Disconnected");
    gatewayStatusTicker.detach();
    // Optionally, reconnect WiFi here
  }
}

void sendPacket() {
  if (!txComplete && !pendingTx.awaitingAck) {
    return;
  }

  start_enc = micros();
  gettimeofday(&tv_now, NULL);
  int32_t time = (int32_t)tv_now.tv_sec;

  char *newCode = otp->getCode(TXbuffer.first().OTPtime);
  uint32_t intCode = atoi((char *)newCode);

  dataToENC.OTP[2] = intCode >> 16;
  dataToENC.OTP[1] = intCode >> 8;
  dataToENC.OTP[0] = intCode >> 0;

  dataToENC.command = TXbuffer.first().command;

  dataToENC.nodeFromId = deviceID;

  for (int idx = 0; idx < sizeof(TXbuffer.first().payload); idx++) {
    dataToENC.payload[idx] = TXbuffer.first().payload[idx];
  }

  loraTXpacket.nodeToId = TXbuffer.first().nodeToId;
  //encrypt the data payload
  memcpy(decrypted, &dataToENC, sizeof(decrypted));
  aes128.encryptBlock(encrypted, decrypted);  //cypher->output block, plaintext->input block
  memcpy(&loraTXpacket.dataEncrypted, encrypted, sizeof(encrypted));

  //temp disable encryption
  //memcpy(&loraTXpacket.dataEncrypted, &dataToENC, sizeof(dataToENC));

  //Serial.printf("Message to NodeID: %08X ",TXbuffer.first().nodeToId);
  Serial.print("Message to NodeID: ");
  auto tx = TXbuffer.first();
  char *printToID = (char *)&tx.nodeToId;
  for (int idx = 0; idx < sizeof(tx.nodeToId); idx++) {
      Serial.printf("%02X ", printToID[idx]);
  }
  Serial.println();
  //Serial.print(TXbuffer.first().nodeToId);
  Serial.print("OTP Time: ");
  Serial.print(TXbuffer.first().OTPtime);
  Serial.print("  OTP code: ");
  Serial.print(intCode);
  Serial.print("  Time now: ");
  Serial.print(time);

  Serial.print("Data To ENC as   HEX values: ");
  char *printData = (char *)&dataToENC;
  for (int idx = 0; idx < sizeof(dataToENC); idx++) {
    Serial.printf("%02X ", printData[idx]);
  }
  //Serial.println();
  //Serial.print("Data encrypted   HEX values: ");
  //printData = (char *)&loraTXpacket.dataEncrypted;
  //for (int idx = 0; idx < sizeof(loraTXpacket.dataEncrypted); idx++) {
  //  Serial.printf("%02X ", printData[idx]);
  //}
  //Serial.println();
  //Serial.print("DataTX package   HEX values: ");
  //printData = (char *)&loraTXpacket;
  //for (int idx = 0; idx < sizeof(loraTXpacket); idx++) {
  //  Serial.printf("%02X ", printData[idx]);
  //}
  Serial.println();
  Serial.printf("  Packet build time %ldus\r\n", (micros() - start_enc));

  txComplete = false;
  loraTXTime = micros();
  if (!pendingTx.awaitingAck) {
    pendingTx.retryCount = 0;
  }
  pendingTx.lastSendTime = millis();
  pendingTx.awaitingAck = true;
  pendingTx.awaitingNodeId = TXbuffer.first().nodeToId;
  // Start sending
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 150);
  // Counter for repeated CAD in case we have many traffic
  cadRepeat = 0;
  Radio.StartCad();
  // Make sure we detect a timeout during sending
  loraTimeout = millis();
  Logger.log("INFO", "LORA_TX", "to=0x%08X, cad=%u, enc=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", loraTXpacket.nodeToId, cadRepeat, encrypted[0], encrypted[1], encrypted[2], encrypted[3], encrypted[4], encrypted[5], encrypted[6], encrypted[7], encrypted[8], encrypted[9], encrypted[10], encrypted[11], encrypted[12], encrypted[13], encrypted[14], encrypted[15]);
}

// ISR for config button press
void IRAM_ATTR onConfigButtonPress() {
    unsigned long now = millis();
    if (now - lastInterruptTime > DEBOUNCE_MS) { // Debounce
        configButtonPressed = true; // Set flag for main loop to handle
        buttonPressTime = now;      // Record time of press
        lastInterruptTime = now;    // Update last interrupt time
    }
}

// Handle config button logic (short/long press)
void handleConfigButton() {
    if (configButtonPressed) { // Check if ISR flagged a press
        configButtonPressed = false; // Reset flag

        // Wait for button release or long press timeout
        unsigned long pressDuration = 0;
        while (digitalRead(CONFIG_BTN_INPUT) == LOW) {
            delay(10); // Small delay to avoid busy-waiting
            pressDuration = millis() - buttonPressTime;
            if (pressDuration >= LONG_PRESS_MS) {
                break; // Exit if long press detected
            }
        }

        if (pressDuration >= LONG_PRESS_MS) {
            Serial.println("Long press detected. Clearing config and restarting.");
            shouldClearConfig = true; // Signal to clear config
        } else if (pressDuration >= DEBOUNCE_MS) { // Check for valid short press
            Serial.println("Short press detected. Entering config mode.");
            shouldStartConfig = true; // Signal to start config mode
        }
        // Reset button state for next press
        buttonHandled = true; // Mark as handled to avoid re-triggering immediately
    } else {
        // If button is released and was previously handled, reset handled flag
        if (digitalRead(CONFIG_BTN_INPUT) == HIGH && buttonHandled) {
            buttonHandled = false;
        }
    }
}


uint8_t checkOTP(uint8_t OTP[3]) {
  gettimeofday(&tv_now, NULL);
  int64_t time = (int64_t)tv_now.tv_sec;
  char *newCode = otp->getCode(time);
  uint32_t intCode = 0;
  intCode += (uint32_t)OTP[0];
  intCode += (uint32_t)OTP[1] << 8;
  intCode += (uint32_t)OTP[2] << 16;

  for (uint8_t idx = 0; idx < OTP_RANGE; idx++) {
    newCode = otp->getCode(time + idx);
    if (intCode == atoi((char *)newCode)) {
      return idx + 1;
    }
    newCode = otp->getCode(time - idx);
    if (intCode == atoi((char *)newCode)) {
      return (idx + 1) << 4;
    }
  }
  //no codes matched
  return 0x00;
}
uint8_t oldTimeOTP(uint8_t OTP[3], time_t oldTime) {
  uint32_t intCode = 0;
  intCode += (uint32_t)OTP[0];
  intCode += (uint32_t)OTP[1] << 8;
  intCode += (uint32_t)OTP[2] << 16;

  char *newCode = otp->getCode(oldTime);
  if (intCode == atoi((char *)newCode)) {
    return 0xBB;
  }

  return 0x00;
}


void decodePacket(void) {
  gettimeofday(&tv_now, NULL);
  int32_t time = (int32_t)tv_now.tv_sec;

  //verify OTP is correct
  uint8_t OTPresult = checkOTP(RXbuffer.first().OTP);

  Serial.print("   OTP result: ");
  Serial.printf("%02X ", OTPresult);

  if ((uint8_t)RXbuffer.first().command == ACK) {
    Serial.print("ACK received from: ");
    auto rx = RXbuffer.first();
    char *printData = (char *)&rx.nodeFromId;
    for (int idx = 0; idx < sizeof(rx.nodeFromId); idx++) {
        Serial.printf("%02X ", printData[idx]);
    }
    Serial.println();

    if (pendingTx.awaitingAck && RXbuffer.first().nodeFromId == pendingTx.awaitingNodeId) {
      TXbuffer.shift();
      pendingTx.awaitingAck = false;
      pendingTx.retryCount = 0;
      pendingTx.awaitingNodeId = 0;
    }
    RXbuffer.shift();  //we have acted on the command, remove it from buffer
    return;
  }

  if (OTPresult == 0x00) {
    Serial.print(" OTP Failied verification ");
    if ((uint8_t)RXbuffer.first().command != TIME_SYNC_COMMAND) {  // only a time sync request is allowed on failed OTP
      // send a NAK, the device should request timesync then resend whatever..
      Serial.print(" Not a time sync request, Sending Nak ");
      TXbuffer.push(data::TXpacket{ time,
                                    RXbuffer.first().nodeFromId,
                                    NAK,
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255),
                                    (uint8_t)random(0, 255) });
      RXbuffer.shift();  //we have acted on the command, remove it from buffer.
      return;
    }
    //retry OTP validation with old time
    Serial.print(" Is a time sync request, verifying devices current time used to generate OTP: ");
    time_t oldTime = 0;
    oldTime += (uint32_t)RXbuffer.first().payload[4];
    oldTime += (uint32_t)RXbuffer.first().payload[5] << 8;
    oldTime += (uint32_t)RXbuffer.first().payload[6] << 16;
    oldTime += (uint32_t)RXbuffer.first().payload[7] << 24;
    Serial.print(oldTime);
    OTPresult = oldTimeOTP(RXbuffer.first().OTP, oldTime);
    Serial.print(" OTP result: ");
    Serial.printf("%02X ", OTPresult);
  }
  //OTP is valid
  if ((uint8_t)RXbuffer.first().command == TIME_SYNC_COMMAND) {
    //time sync request
    Serial.println("Time Sync Requested");
    time_t timeSet = 0;
    timeSet += (uint32_t)RXbuffer.first().payload[0];
    timeSet += (uint32_t)RXbuffer.first().payload[1] << 8;
    timeSet += (uint32_t)RXbuffer.first().payload[2] << 16;
    timeSet += (uint32_t)RXbuffer.first().payload[3] << 24;

    uint8_t currentTime[4];
    currentTime[0] = time >> 24;
    currentTime[1] = time >> 16;
    currentTime[2] = time >> 8;
    currentTime[3] = time;

    TXbuffer.push(data::TXpacket{ timeSet,
                                  RXbuffer.first().nodeFromId,
                                  TIME_SYNC_COMMAND,
                                  (uint8_t)time,
                                  (uint8_t)(time >> 8),
                                  (uint8_t)(time >> 16),
                                  (uint8_t)(time >> 24),
                                  RXbuffer.first().payload[4],
                                  RXbuffer.first().payload[5],
                                  RXbuffer.first().payload[6],
                                  RXbuffer.first().payload[7] });
    RXbuffer.shift();  //we have acted on the command, remove it from buffer
    return;
  }


  if ((uint8_t)RXbuffer.first().command == NAK) {
    Serial.println("NAK received from: ");
    Serial.println(RXbuffer.first().nodeFromId);
    RXbuffer.shift();  //we have acted on the command, remove it from buffer
    return;
  }

  if ((uint8_t)RXbuffer.first().command == DEVICE_STATUS) {
    // Publish discovery config for master device sensors if not already done
    if (publishedMasterDiscovery.find(RXbuffer.first().nodeFromId) == publishedMasterDiscovery.end()) {
        publishMasterDeviceDiscoveryConfig(RXbuffer.first().nodeFromId);
        publishedMasterDiscovery.insert(RXbuffer.first().nodeFromId);
    }
    Serial.print("Recieved: DEVICE STATUS");
    uint16_t battVRX = 0;
    battVRX += (uint16_t)RXbuffer.first().payload[0] << 8;
    battVRX += (uint16_t)RXbuffer.first().payload[1];
    Serial.print("  Battery Voltage: ");
    Serial.print(battVRX);
    Serial.print("mV  Wakeup Count: ");
    uint16_t wakeCountRX = 0;
    wakeCountRX += (uint16_t)RXbuffer.first().payload[2] << 8;
    wakeCountRX += (uint16_t)RXbuffer.first().payload[3];
    Serial.print(wakeCountRX);
    Serial.print("  Total Awake Time: ");
    uint16_t awakeTimeRX = 0;
    awakeTimeRX += (uint16_t)RXbuffer.first().payload[4] << 8;
    awakeTimeRX += (uint16_t)RXbuffer.first().payload[5];
    Serial.println(awakeTimeRX);
    char topic[64];
    char payload[16];
    // Battery voltage
    snprintf(topic, sizeof(topic), "SleepyLoRa/%08X/status/Battery", RXbuffer.first().nodeFromId);
    snprintf(payload, sizeof(payload), "%u", battVRX);
    mqttClient.publish(topic, 0, false, payload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", topic, payload);
    // Wake count
    snprintf(topic, sizeof(topic), "SleepyLoRa/%08X/status/wakeCount", RXbuffer.first().nodeFromId);
    snprintf(payload, sizeof(payload), "%u", (unsigned int)wakeCountRX);
    mqttClient.publish(topic, 0, false, payload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", topic, payload);
    // Awake seconds
    snprintf(topic, sizeof(topic), "SleepyLoRa/%08X/status/awakeSeconds", RXbuffer.first().nodeFromId);
    snprintf(payload, sizeof(payload), "%u", awakeTimeRX);
    mqttClient.publish(topic, 0, false, payload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", topic, payload);

    RXbuffer.shift();  //we have acted on the command, remove it from buffer
    return;
  }

  if ((uint8_t)RXbuffer.first().command == BLIND_STATUS) {
    Serial.print("Recieved: BLIND STATUS");
    uint8_t blind_number = RXbuffer.first().payload[0];
    uint8_t blind_state = RXbuffer.first().payload[1];
    uint16_t posCalc = 0;
    posCalc += (uint16_t)RXbuffer.first().payload[2] << 8;
    posCalc += (uint16_t)RXbuffer.first().payload[3];
    int posAvg = 0;
    posAvg += (uint16_t)RXbuffer.first().payload[4] << 8;
    posAvg += (uint16_t)RXbuffer.first().payload[5];
    uint8_t moveStatus = RXbuffer.first().payload[6];
    uint8_t moveStatusExt = RXbuffer.first().payload[7]; // should be 0xFF
    Serial.print("   Blind Number: ");
    Serial.print(blind_number);
    const char* statePayload = "unknown";
    Serial.print(" Blind Staus: ");
    switch (blind_state) {
      case STATE_CLOSING:
        Serial.print("Closing");
        statePayload = "closing";
        break;
      case STATE_OPENING:
        Serial.print("Opening");
        statePayload = "opening";
        break;
      case STATE_CLOSED:
        Serial.print("Closed");
        statePayload = "closed";
        break;
      case STATE_OPEN:
        Serial.print("Open");
        statePayload = "open";
        break;
      default:
        Serial.print("Unknown");
        statePayload = "unknown";
    }
    Serial.print("  Calculated Position: ");
    Serial.print(posCalc / 10.0f);
    Serial.print("%  Raw Position: ");
    Serial.print(posAvg / 10.0f);
    Serial.print("%  Last Move Status: ");
    switch (moveStatus) {
      case 0x01:
        Serial.print("OK");
        break;
      case 0x02:
        Serial.print("TIMEOUT");
        break;
      default:
        Serial.printf("Unknown (0x%02X)", moveStatus);
    }
    Serial.println();

    addBlindIfNew(RXbuffer.first().nodeFromId, blind_number);

    // Track last seen time for this blind
    blindLastSeen[std::make_pair(RXbuffer.first().nodeFromId, blind_number)] = millis();

    // State topic
    char stateTopic[64];
    snprintf(stateTopic, sizeof(stateTopic), "SleepyLoRa/%08X_%u/state", RXbuffer.first().nodeFromId, blind_number);

    // Availability topic
    char availTopic[80];
    snprintf(availTopic, sizeof(availTopic), "SleepyLoRa/%08X_%u/availability", RXbuffer.first().nodeFromId, blind_number);
    mqttClient.publish(availTopic, 0, true, "online");
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=true, payload=online", availTopic);

    mqttClient.publish(stateTopic, 0, false, statePayload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", stateTopic, statePayload);

    // Position topic
    char posTopic[64];
    snprintf(posTopic, sizeof(posTopic), "SleepyLoRa/%08X_%u/position", RXbuffer.first().nodeFromId, blind_number);

    char posPayload[16];
    snprintf(posPayload, sizeof(posPayload), "%u", posCalc / 10);

    mqttClient.publish(posTopic, 0, false, posPayload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", posTopic, posPayload);

    // Last move status attribute topic
    char statusTopic[64];
    snprintf(statusTopic, sizeof(statusTopic), "SleepyLoRa/%08X_%u/last_move_status", RXbuffer.first().nodeFromId, blind_number);
    char statusPayload[8];
    switch (moveStatus) {
      case 0x01:
        strcpy(statusPayload, "OK");
        break;
      case 0x02:
        strcpy(statusPayload, "TIMEOUT");
        break;
      case 0x03:
        strcpy(statusPayload, "MOVING");
        break;
      default:
        strcpy(statusPayload, "UNKNOWN");
    }
    mqttClient.publish(statusTopic, 0, false, statusPayload);
    Logger.log("INFO", "MQTT_PUB", "Publish: topic=%s, qos=0, retain=false, payload=%s", statusTopic, statusPayload);

    RXbuffer.shift();  //we have acted on the command, remove it from buffer
    return;
  }
  /*
    case BLIND_COMMAND:
      Serial.println("BLIND COMMAND");
      break;
    case SET_PARAMETER:
      Serial.println("SET PARAMETER");
      break;
    case GET_PARAMETER:
      Serial.println("GET PARAMETER");
      break;
    case SET_LIMIT:
      Serial.println("SET LIMIT");
      break;
    case GET_LIMIT:
      Serial.println("GET LIMIT");
      break;
    default:
      Serial.println("UNKNOWN COMMAND!");
*/

}

void checkBlindAvailability() {
    unsigned long now = millis();
    for (const auto& entry : blindLastSeen) {
        uint32_t nodeId = entry.first.first;
        uint8_t blindNum = entry.first.second;
        unsigned long lastSeen = entry.second;
        if (now - lastSeen > 600000UL) { // 10 minutes
            char availTopic[80];
            snprintf(availTopic, sizeof(availTopic), "SleepyLoRa/%08X_%u/availability", nodeId, blindNum);
            mqttClient.publish(availTopic, 0, true, "offline");
        }
    }
}

/*****************************************************************************************************************

Radio Setup, Sleep and Callbacks section

*****************************************************************************************************************/

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
  hwConfig.USE_DIO2_ANT_SWITCH = true;       // uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = true;             // DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;      // Only Insight ISP4520 module uses DIO3 as antenna control

  lora_hardware_init(hwConfig);

  // Initialize the Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = OnCadDone;

  Radio.Init(&RadioEvents);
  // Use selected frequency from config
  Radio.SetChannel(config.rf_frequency);
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


/**@brief Function to be executed on Radio Rx Done event */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {

  memcpy(&tmploraRXpacket, payload, size);

  String rxHex;
  for (int idx = 0; idx < size; idx++) {
    char buf[3];
    sprintf(buf, "%02X", payload[idx]);
    rxHex += buf;
  }
  Logger.log("INFO", "LORA_RX", "from=0x%08X, size=%u, enc=%s", tmploraRXpacket.nodeToId, size, rxHex.c_str());

  if (size != sizeof(tmploraRXpacket)) {
    Serial.println("Packet dropped, wrong size.. ");
    Logger.log("WARN", "LORA_RX", "Packet dropped, wrong size: %u", size);
    return;
  }
  if (tmploraRXpacket.nodeToId != deviceID) {
    Serial.println("Packet dropped,  not for us");
    Logger.log("INFO", "LORA_RX", "Packet dropped, not for us: 0x%08X", tmploraRXpacket.nodeToId);
    return;
  }
  //decrypt the message
  memcpy(encrypted, tmploraRXpacket.dataEncrypted, sizeof(encrypted));
  aes128.decryptBlock(decrypted, encrypted);
  memcpy(&dataDEC, decrypted, sizeof(decrypted));

  // print decoded packet
  Serial.print("Message from NodeID: ");
  char *printData = (char *)&dataDEC.nodeFromId;
  for (int idx = 0; idx < sizeof(dataDEC.nodeFromId); idx++) {
    Serial.printf("%02X ", printData[idx]);
  }
  Serial.print("  OTP: ");
  printData = (char *)&dataDEC.OTP;
  for (int idx = 0; idx < sizeof(dataDEC.OTP); idx++) {
    Serial.printf("%02X ", printData[idx]);
  }

  Serial.print("  Command: ");
  printData = (char *)&dataDEC.command;
  for (int idx = 0; idx < sizeof(dataDEC.command); idx++) {
    Serial.printf("%02X ", printData[idx]);
  }
  Serial.print("  Payload: ");
  printData = (char *)&dataDEC.payload;
  for (int idx = 0; idx < sizeof(dataDEC.payload); idx++) {
    Serial.printf("%02X ", printData[idx]);
  }
  Serial.println();
if (!RXbuffer.isFull()) {
  RXbuffer.push(data::RXpacket{ tmploraRXpacket.nodeToId,
                                dataDEC.OTP[0],
                                dataDEC.OTP[1],
                                dataDEC.OTP[2],
                                dataDEC.command,
                                dataDEC.nodeFromId,
                                dataDEC.payload[0],
                                dataDEC.payload[1],
                                dataDEC.payload[2],
                                dataDEC.payload[3],
                                dataDEC.payload[4],
                                dataDEC.payload[5],
                                dataDEC.payload[6],
                                dataDEC.payload[7] });
} else {
    Logger.log("WARN", "LORA_RX", "RX Buffer full, dropping packet!");
}
  // LoRa receive finished, go back to loop
}

/**@brief Function to be executed on Radio Rx Timeout event */
void OnRxTimeout(void) {
  Serial.print("   Receive timeout - Radio Status: ");
  Serial.println(Radio.GetStatus());
}

/**@brief Function to be executed on Radio Rx Error event */
void OnRxError(void) {
  Serial.print("   Receive error - Radio Status: ");
  Serial.println(Radio.GetStatus());
}

/**@brief Function to be executed on Radio Tx Done event */
void OnTxDone(void) {
  Serial.print("   Transmit finished - Radio Status: ");
  Serial.print(Radio.GetStatus());
  Serial.printf("    Packet send time %ldus", (micros() - loraTXTime));
  lastTX = millis();
  //TXbuffer.shift();
  txComplete = true;
  Radio.Rx(0);
  Serial.print("  Set RX mode - Radio Status: ");
  Serial.println(Radio.GetStatus());
}

/**@brief Function to be executed on Radio Tx Timeout event */
void OnTxTimeout(void) {
  Serial.print("   Transmit timeout - Radio Status: ");
  Serial.println(Radio.GetStatus());
  lastTX = millis();
  txComplete = true;
}

/**@Function for channel activity detection and TX packet*/
void OnCadDone(bool cadResult) {
  cadRepeat++;
  Radio.Standby();
  if (cadResult) {
    Serial.printf("CAD returned channel busy %d times\r\n", cadRepeat);
    if (cadRepeat < CAD_RETRY) {
      // Retry CAD
      Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 150);
      Radio.StartCad();
    } else {
      Serial.printf("CAD failed too many times.Gave up afer %ldus\r\n", (micros() - loraTXTime));
      txComplete = true;
    }
  } else {
    Radio.Send((uint8_t *)&loraTXpacket, sizeof(loraTXpacket));
  }
}

