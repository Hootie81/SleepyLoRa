#include "web_portal.h"
#include "web_templates.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <ESPAsyncWebServer.h>
#include "Command_Register.h" // <-- Add this include for command macros

// Extern config variables from main.cpp
extern uint32_t config_gatewayID;
extern uint8_t config_aes_key[16];
extern uint8_t config_hmacKey[10];
extern uint32_t config_rf_frequency;
extern void saveConfigToFlash();
extern bool configMode;
extern bool configPortalActive;

// Externs for slave/command functions/constants from main.cpp
extern void scanUARTSlavesAndPublish();
extern bool sendCommandToSlaveUART(uint8_t blind_number, uint8_t command, uint8_t *payload, size_t payload_len);

extern Preferences prefs;
// Webserver/AP config
AsyncWebServer server(80);
Ticker webserverTimeoutTicker;
Ticker webPortalPositionTicker;

// Helper to render the slave list HTML
String renderSlaveList();

void onLoadFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

extern bool hexToBytes(const char*, uint8_t*, size_t);

void startConfigAPAndWebserver(BlindMotorController& blind) {
  Serial.println("[DEBUG] Starting Config AP and Webserver...");
  WiFi.mode(WIFI_AP);
  bool apResult = WiFi.softAP("SleepyLoRaMaster", "blind1234");
  Serial.printf("[DEBUG] softAP result: %s\n", apResult ? "success" : "fail");
  Serial.print("[DEBUG] AP IP: ");
  Serial.println(WiFi.softAPIP());
  setupWebPortal(server, blind);
  server.begin();
  Serial.println("[DEBUG] Webserver started on port 80");
}

void stopConfigAPAndWebserver() {
  Serial.println("[DEBUG] Stopping Webserver and AP...");
  server.end();
  WiFi.mode(WIFI_OFF);
  configMode = false;
  Serial.println("[DEBUG] Webserver stopped, WiFi off, configMode=false");
}

const char* BLIND_ASCII_ART =
" ____  _                       _          ____       \r\n"
"/ ___|| | ___  ___ _ __  _   _| |    ___ |  _ \\ __ _ \r\n"
"\\___ \\| |/ _ \\/ _ \\ '_ \\| | | | |   / _ \\| |_) / _` |\r\n"
" ___) | |  __/  __/ |_) | |_| | |__| (_) |  _ < (_| |\r\n"
"|____/|_|\\___|\\___| .__/ \\__, |_____\\___/|_| \\_\\__,_|\r\n"
"                  |_|    |___/                       \r\n";

static BlindMotorController* blindPtr = nullptr;
static int wsClientCount = 0;

String processor(const String& var) {
    Serial.print("[DEBUG] processor called for var: ");
    Serial.println(var);
    if (var == "ASCII_ART") return String(BLIND_ASCII_ART);
    if (!blindPtr) {
        Serial.println("[DEBUG] blindPtr is null!");
        return String();
    }
    if (var == "RAW_POS") return String(blindPtr->readPositionRaw());
    if (var == "BLIND_POS") return String(blindPtr->readPositionPercent());
    if (var == "ENGAGE_PWM") return String(blindPtr->getEngagePwm());
    if (var == "DISENGAGE_PWM") return String(blindPtr->getDisengagePwm());
    if (var == "PWM_MIN") return String(blindPtr->getPwmMin());
    if (var == "PWM_MAX") return String(blindPtr->getPwmMax());
    if (var == "MOVE_TIME") return String(blindPtr->getMoveTimeMs());
    if (var == "ENGAGE_TIME") return String(blindPtr->getEngageTime());
    if (var == "DISENGAGE_TIME") return String(blindPtr->getDisengageTime());
    if (var == "CLOSED_LIMIT") return String(blindPtr->getClosedLimit());
    if (var == "OPEN_LIMIT") return String(blindPtr->getOpenLimit());
    if (var == "GATEWAY_ID") {
        char buf[16];
        snprintf(buf, sizeof(buf), "%08lx", (unsigned long)config_gatewayID); // HEX, zero-padded
        return String(buf);
    }
    if (var == "AES_KEY") {
        char aesKeyHex[33] = {0};
        for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config_aes_key[i]);
        return String(aesKeyHex);
    }
    if (var == "HMAC_KEY") {
        char hmacKeyHex[21] = {0};
        for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config_hmacKey[i]);
        return String(hmacKeyHex);
    }
    // Add more as needed for keys, gateway, etc.
    Serial.println("[DEBUG] processor: unknown var");
    return String();
}

AsyncWebSocket ws("/ws");

static void notifyAllClients() {
    if (!blindPtr) return;
    DynamicJsonDocument doc(384); // Increased size for slave data
    doc["blindPos"] = blindPtr->readPositionPercent();
    doc["rawPos"] = blindPtr->readPositionRaw();
    doc["status"] = blindPtr->getStatusString();
    doc["calib"] = blindPtr->getCalibrationStateString();

    // --- Add slave positions ---
    extern uint8_t detected_slaves[];
    extern uint8_t detected_slave_count;
    extern uint16_t slave_positions[];
    JsonArray slaves = doc.createNestedArray("slaves");
    for (uint8_t i = 0; i < detected_slave_count; ++i) {
        uint8_t blind_number = detected_slaves[i];
        JsonObject s = slaves.createNestedObject();
        s["id"] = blind_number;
        s["pos"] = slave_positions ? (slave_positions[blind_number] / 10) : 0; // scale 0-1000 to 0-100
    }

    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
}

// Manual template rendering for ESP32 (since processor is not called)
String renderTemplate(const char* tpl) {
    String html(tpl);
    html.replace("{{ASCII_ART}}", BLIND_ASCII_ART);
    html.replace("{{RAW_POS}}", String(blindPtr ? blindPtr->readPositionRaw() : 0));
    html.replace("{{BLIND_POS}}", String(blindPtr ? blindPtr->readPositionPercent() : 0));
    html.replace("{{ENGAGE_PWM}}", String(blindPtr ? blindPtr->getEngagePwm() : 0));
    html.replace("{{DISENGAGE_PWM}}", String(blindPtr ? blindPtr->getDisengagePwm() : 0));
    html.replace("{{PWM_MIN}}", String(blindPtr ? blindPtr->getPwmMin() : 0));
    html.replace("{{PWM_MAX}}", String(blindPtr ? blindPtr->getPwmMax() : 0));
    html.replace("{{MOVE_TIME}}", String(blindPtr ? blindPtr->getMoveTimeMs() : 0));
    html.replace("{{ENGAGE_TIME}}", String(blindPtr ? blindPtr->getEngageTime() : 0));
    html.replace("{{DISENGAGE_TIME}}", String(blindPtr ? blindPtr->getDisengageTime() : 0));
    html.replace("{{CLOSED_LIMIT}}", String(blindPtr ? blindPtr->getClosedLimit() : 0));
    html.replace("{{OPEN_LIMIT}}", String(blindPtr ? blindPtr->getOpenLimit() : 0));
    char buf[16];
    snprintf(buf, sizeof(buf), "%08lx", (unsigned long)config_gatewayID); // HEX, zero-padded
    html.replace("{{GATEWAY_ID}}", buf);
    char aesKeyHex[33] = {0};
    for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config_aes_key[i]);
    html.replace("{{AES_KEY}}", aesKeyHex);
    char hmacKeyHex[21] = {0};
    for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config_hmacKey[i]);
    html.replace("{{HMAC_KEY}}", hmacKeyHex);
    html.replace("{{BLIND_STATUS}}", String(blindPtr ? blindPtr->getStatusString() : "Unknown"));
    html.replace("{{CALIBRATION_STATE}}", String(blindPtr ? blindPtr->getCalibrationStateString() : "Unknown"));
    html.replace("{{PID_KP}}", String(blindPtr ? blindPtr->getKp() : 0));
    html.replace("{{PID_KI}}", String(blindPtr ? blindPtr->getKi() : 0, 4));
    html.replace("{{PID_KD}}", String(blindPtr ? blindPtr->getKd() : 0, 3));
    html.replace("{{PID_DEADBAND}}", String(blindPtr ? blindPtr->getErrorDeadband() : 0));
    html.replace("{{PID_IMIN}}", String(blindPtr ? blindPtr->getIntegralMin() : 0, 0));
    html.replace("{{PID_IMAX}}", String(blindPtr ? blindPtr->getIntegralMax() : 0, 0));
    html.replace("{{PID_DMIN}}", String(blindPtr ? blindPtr->getDerivMin() : 0, 0));
    html.replace("{{PID_DMAX}}", String(blindPtr ? blindPtr->getDerivMax() : 0, 0));
    html.replace("{{PID_DALPHA}}", String(blindPtr ? blindPtr->getDerivAlpha() : 0));
    html.replace("{{PID_PALPHA}}", String(blindPtr ? blindPtr->getPwmAlpha() : 0));
    html.replace("{{ENGAGE_THRESH}}", String(blindPtr ? blindPtr->getEngageDetectThreshold() : 0));
    html.replace("{{STALL_THRESH}}", String(blindPtr ? blindPtr->getStallDeltaThreshold() : 0));
    html.replace("{{STALL_SAMPLES}}", String(blindPtr ? blindPtr->getStallSampleCount() : 0));
    html.replace("{{PWM_FREQ}}", String(blindPtr ? blindPtr->getPwmFrequency() : 1000));
    html.replace("{{SLAVE_LIST}}", renderSlaveList());
    html.replace("{{ENGAGE_SAMPLES}}", String(blindPtr ? blindPtr->getEngageSampleCount() : 0));
    html.replace("{{ENGAGE_SAMPLE_COUNT}}", String(blindPtr ? blindPtr->getEngageSampleCount() : 0));
    return html;
}



// Implementation at the end of the file (or after renderTemplate)
String renderSlaveList() {
    extern uint8_t detected_slaves[];
    extern uint8_t detected_slave_count;
    extern uint16_t slave_positions[]; // Optional: if you track positions
    String out;
    if (detected_slave_count == 0) {
        out += "<div style='color:#888;'>No slaves detected. Run scan to find devices.</div>";
    } else {
        for (uint8_t i = 0; i < detected_slave_count; ++i) {
            uint8_t blind_number = detected_slaves[i];
            out += "<div style='margin-bottom:10px; border-bottom:1px solid #ddd; padding-bottom:8px;'>";
            out += "<b>Slave #" + String(blind_number) + "</b> ";
            out += "<form method='POST' action='/open_slave_portal' style='display:inline;margin-left:8px;'><input type='hidden' name='slave' value='" + String(blind_number) + "'><button type='submit'>Open Web Portal</button></form> ";
            // --- WebSocket slider with status indicator ---
            out += "<input type='range' min='0' max='100' value='";
            out += (slave_positions ? String(slave_positions[blind_number]) : "0");
            out += "' id='slaveSlider_" + String(blind_number) + "' style='width:120px;vertical-align:middle;' oninput='this.nextElementSibling.value=this.value'>";
            out += "<output style='margin-left:6px; width:3ch; display:inline-block; text-align:right;' id='slaveOutput_" + String(blind_number) + "'>";
            out += (slave_positions ? String(slave_positions[blind_number]) : "0");
            out += "</output>% ";
            // Status indicator (spinner/green check)
            out += "<span id='slaveStatus_" + String(blind_number) + "' style='margin-left:8px;'></span>";
            // Fallback POST form (hidden, for non-WS)
            out += "<form method='POST' action='/set_slave_position' style='display:none;' id='slaveForm_" + String(blind_number) + "'>";
            out += "<input type='hidden' name='slave' value='" + String(blind_number) + "'>";
            out += "<input type='hidden' name='position' id='slaveHiddenPos_" + String(blind_number) + "' value='" + (slave_positions ? String(slave_positions[blind_number]) : "0") + "'>";
            out += "</form>";
            out += "</div>";
        }
    }
    return out;
}

// Upload handler for /loadfile
static String uploadedConfigFile;
// New upload handler for /loadfile
void onLoadFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    uploadedConfigFile = "";
  }
  for (size_t i = 0; i < len; i++) {
    uploadedConfigFile += (char)data[i];
  }
  if (final) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, uploadedConfigFile);
    if (err) {
      request->send(400, "text/html", "<html><body>Invalid JSON.<br><a href='/'>Back</a></body></html>");
      return;
    }
    if (!(doc["gatewayID"].is<uint32_t>() || doc["gatewayID"].is<const char*>()) || !doc["aes_key"].is<const char*>() || !doc["hmacKey"].is<const char*>()) {
      request->send(400, "text/html", "<html><body>Missing fields in JSON.<br><a href='/'>Back</a></body></html>");
      return;
    }
    uint32_t gwid = 0;
    if (doc["gatewayID"].is<uint32_t>()) {
      gwid = doc["gatewayID"].as<uint32_t>();
    } else if (doc["gatewayID"].is<const char*>()) {
      const char* gwStr = doc["gatewayID"].as<const char*>();
      gwid = strtoul(gwStr, nullptr, 16);
    }
    const char* aesStr = doc["aes_key"];
    const char* hmacStr = doc["hmacKey"];
    uint32_t rf_freq = doc["rf_frequency"].is<uint32_t>() ? doc["rf_frequency"].as<uint32_t>() : 868100000;
    if (strlen(aesStr) != 32 || strlen(hmacStr) != 20) {
      request->send(400, "text/html", "<html><body>Key lengths invalid.<br><a href='/'>Back</a></body></html>");
      return;
    }
    uint8_t aesBuf[16];
    uint8_t hmacBuf[10];
    if (!hexToBytes(aesStr, aesBuf, 16) || !hexToBytes(hmacStr, hmacBuf, 10)) {
      request->send(400, "text/html", "<html><body>Key format invalid.<br><a href='/'>Back</a></body></html>");
      return;
    }
    config_gatewayID = gwid;
    memcpy(config_aes_key, aesBuf, 16);
    memcpy(config_hmacKey, hmacBuf, 10);
    config_rf_frequency = rf_freq;
    saveConfigToFlash();
    request->send(200, "text/html", "<html><body>Config loaded from file!<br><a href='/'>Back</a></body></html>");
    configMode = false;
  }
}


void setupWebPortal(AsyncWebServer& server, BlindMotorController& blind) {
    Serial.println("[DEBUG] Entered setupWebPortal");
    Serial.printf("[DEBUG] blindPtr address before assignment: %p\n", (void*)blindPtr);
    blindPtr = &blind;
    Serial.printf("[DEBUG] blindPtr address after assignment: %p\n", (void*)blindPtr);
    Serial.println("[DEBUG] Registering main page handler with manual template rendering...");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.println("[DEBUG] / handler called");
        Serial.printf("[DEBUG] MAIN_PAGE_TEMPLATE address: %p\n", (const void*)MAIN_PAGE_TEMPLATE);
        Serial.println("[DEBUG] About to call renderTemplate...");
        String html = renderTemplate(MAIN_PAGE_TEMPLATE);
        request->send(200, "text/html", html);
        Serial.println("[DEBUG] send call returned");
    });
    server.on("/set_closed", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("manual_closed", true) && request->hasParam("closed_limit", true)) {
            int val = request->getParam("closed_limit", true)->value().toInt();
            blindPtr->setClosedLimit(val);
            Serial.printf("[DEBUG] Manually set closed limit to %d\n", val);
        } else {
            blindPtr->setClosedLimit(blindPtr->readPositionRaw());
            Serial.println("[DEBUG] Set closed limit to current position");
        }
        request->redirect("/");
    });
    server.on("/set_open", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("manual_open", true) && request->hasParam("open_limit", true)) {
            int val = request->getParam("open_limit", true)->value().toInt();
            blindPtr->setOpenLimit(val);
            Serial.printf("[DEBUG] Manually set open limit to %d\n", val);
        } else {
            blindPtr->setOpenLimit(blindPtr->readPositionRaw());
            Serial.println("[DEBUG] Set open limit to current position");
        }
        request->redirect("/");
    });
    server.on("/set_engage_pwm", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("engage_pwm", true)) {
            uint8_t val = request->getParam("engage_pwm", true)->value().toInt();
            blindPtr->setEngagePwm(val);
        }
        request->redirect("/");
    });
    server.on("/set_disengage_pwm", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("disengage_pwm", true)) {
            uint8_t val = request->getParam("disengage_pwm", true)->value().toInt();
            blindPtr->setDisengagePwm(val);
        }
        request->redirect("/");
    });
    server.on("/set_pwm_min", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("pwm_min", true)) {
            uint8_t minVal = request->getParam("pwm_min", true)->value().toInt();
            uint8_t maxVal = blindPtr->getPwmMax();
            if (maxVal >= minVal) {
                blindPtr->setPwmMin(minVal);
            } else {
                // Invalid: do not apply, maybe add a flash message in future
                request->redirect("/?error=pwmmin");
                return;
            }
        }
        request->redirect("/");
    });
    server.on("/set_pwm_max", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("pwm_max", true)) {
            uint8_t maxVal = request->getParam("pwm_max", true)->value().toInt();
            uint8_t minVal = blindPtr->getPwmMin();
            if (maxVal >= minVal) {
                blindPtr->setPwmMax(maxVal);
            } else {
                // Invalid: do not apply, maybe add a flash message in future
                request->redirect("/?error=pwmmax");
                return;
            }
        }
        request->redirect("/");
    });
    server.on("/set_move_time", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("move_time", true)) {
            int val = request->getParam("move_time", true)->value().toInt();
            blindPtr->setMoveTimeMs(val);
        }
        request->redirect("/");
    });
    server.on("/set_engage_time", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("engage_time", true)) {
            int val = request->getParam("engage_time", true)->value().toInt();
            blindPtr->setEngageTime(val);
        }
        request->redirect("/");
    });
    server.on("/set_disengage_time", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("disengage_time", true)) {
            int val = request->getParam("disengage_time", true)->value().toInt();
            blindPtr->setDisengageTime(val);
        }
        request->redirect("/");
    });
    server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *request){
        blindPtr->startCalibrationSequence();
        request->redirect("/");
    });
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
        bool shouldReboot = !Update.hasError();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
        response->addHeader("Connection", "close");
        request->send(response);
        if (shouldReboot) {
            delay(100);
            ESP.restart();
        }
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!index) {
            Serial.printf("Update Start: %s\n", filename.c_str());
            Update.begin(UPDATE_SIZE_UNKNOWN);
        }
        if (Update.write(data, len) != len) {
            Serial.println("Update Write Failed");
        }
        if (final) {
            if (Update.end(true)) {
                Serial.println("Update Success");
            } else {
                Serial.println("Update Failed");
            }
        }
    });
    server.on("/loadfile", HTTP_POST, [](AsyncWebServerRequest *request) {
        // This will be called after upload is complete if no upload handler is set, so just do nothing here
    }, onLoadFileUpload);
    server.on("/download_keys", HTTP_GET, [](AsyncWebServerRequest *request){
        char aesKeyHex[33] = {0};
        char hmacKeyHex[21] = {0};
        for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config_aes_key[i]);
        for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config_hmacKey[i]);
        String json = String("{\"gatewayID\":\"") + String(config_gatewayID, HEX) + "\"," +
            "\"aes_key\":\"" + aesKeyHex + "\"," +
            "\"hmacKey\":\"" + hmacKeyHex + "\"," +
            "\"rf_frequency\":" + String(config_rf_frequency) + "}";
        request->send(200, "application/json", json);
    });
    server.on("/close", HTTP_POST, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", "<html><body>Web portal closed. Device will sleep.<br><a href='/'>Back</a></body></html>");
        // Stop the webserver and go to sleep after a short delay to allow the response to be sent
        configPortalActive = false;
        webserverTimeoutTicker.detach();
        // Use a timer to delay shutdown, e.g. 500ms
        static Ticker shutdownTicker;
        shutdownTicker.once_ms(500, []() {
            stopConfigAPAndWebserver();
        });
    });
    server.on("/set_pid", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("Kp", true)) blindPtr->setKp(request->getParam("Kp", true)->value().toFloat());
        if (request->hasParam("Ki", true)) blindPtr->setKi(request->getParam("Ki", true)->value().toFloat());
        if (request->hasParam("Kd", true)) blindPtr->setKd(request->getParam("Kd", true)->value().toFloat());
        if (request->hasParam("Deadband", true)) blindPtr->setErrorDeadband(request->getParam("Deadband", true)->value().toFloat());
        if (request->hasParam("IntegralMin", true)) blindPtr->setIntegralMin(request->getParam("IntegralMin", true)->value().toFloat());
        if (request->hasParam("IntegralMax", true)) blindPtr->setIntegralMax(request->getParam("IntegralMax", true)->value().toFloat());
        if (request->hasParam("DerivMin", true)) blindPtr->setDerivMin(request->getParam("DerivMin", true)->value().toFloat());
        if (request->hasParam("DerivMax", true)) blindPtr->setDerivMax(request->getParam("DerivMax", true)->value().toFloat());
        if (request->hasParam("DerivAlpha", true)) blindPtr->setDerivAlpha(request->getParam("DerivAlpha", true)->value().toFloat());
        if (request->hasParam("PwmAlpha", true)) blindPtr->setPwmAlpha(request->getParam("PwmAlpha", true)->value().toFloat());
        request->redirect("/");
    });
    server.on("/set_stall", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("EngageThresh", true)) blindPtr->setEngageDetectThreshold(request->getParam("EngageThresh", true)->value().toInt());
        if (request->hasParam("EngageSampleCount", true)) blindPtr->setEngageSampleCount(request->getParam("EngageSampleCount", true)->value().toInt());
        if (request->hasParam("StallThresh", true)) blindPtr->setStallDeltaThreshold(request->getParam("StallThresh", true)->value().toInt());
        if (request->hasParam("StallSamples", true)) blindPtr->setStallSampleCount(request->getParam("StallSamples", true)->value().toInt());
        request->redirect("/");
    });
    server.on("/set_pwm_freq", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("pwm_freq", true)) {
            uint32_t val = request->getParam("pwm_freq", true)->value().toInt();
            if (val >= 100 && val <= 40000) { // Increased max to 40,000 Hz
                blindPtr->setPwmFrequency(val);
                // Re-apply frequency immediately
                ledcSetup(blindPtr->getPwmChannel(), val, 8);
            }
        }
        request->redirect("/");
    });
    server.on("/scan_slaves", HTTP_POST, [](AsyncWebServerRequest *request){
        scanUARTSlavesAndPublish();
        request->redirect("/");
    });
    server.on("/open_slave_portal", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("slave", true)) {
            uint8_t blind_number = request->getParam("slave", true)->value().toInt();
            // Call your function to open the web portal for this slave
            // e.g., sendCommandToSlaveUART(blind_number, UPDATE_SLAVE, ...)
            uint8_t dummy[8] = {0};
            sendCommandToSlaveUART(blind_number, UPDATE_SLAVE, dummy, sizeof(dummy));
        }
        request->redirect("/");
    });
    server.on("/set_slave_position", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("slave", true) && request->hasParam("position", true)) {
            uint8_t blind_number = request->getParam("slave", true)->value().toInt();
            uint8_t pos = request->getParam("position", true)->value().toInt();
            blind_command_t cmd = {};
            cmd.blindNumber = blind_number;
            cmd.set_state = 0x04; // set position
            cmd.set_position = pos; // send 0-100 directly
            uint8_t payload[8] = {0};
            memcpy(payload, &cmd, sizeof(cmd));
            sendCommandToSlaveUART(blind_number, BLIND_COMMAND, payload, sizeof(payload));
            // Set slave_moving to true to increase polling rate
            extern bool slave_moving[];
            slave_moving[blind_number] = true;
        }
        request->redirect("/");
    });
    server.on("/set_all_pwm", HTTP_POST, [](AsyncWebServerRequest *request){
        uint8_t minVal = blindPtr->getPwmMin();
        uint8_t maxVal = blindPtr->getPwmMax();
        if (request->hasParam("pwm_min", true)) {
            minVal = request->getParam("pwm_min", true)->value().toInt();
        }
        if (request->hasParam("pwm_max", true)) {
            maxVal = request->getParam("pwm_max", true)->value().toInt();
        }
        if (maxVal >= minVal) {
            if (request->hasParam("engage_pwm", true)) {
                uint8_t val = request->getParam("engage_pwm", true)->value().toInt();
                blindPtr->setEngagePwm(val);
            }
            if (request->hasParam("disengage_pwm", true)) {
                uint8_t val = request->getParam("disengage_pwm", true)->value().toInt();
                blindPtr->setDisengagePwm(val);
            }
            blindPtr->setPwmMin(minVal);
            blindPtr->setPwmMax(maxVal);
            if (request->hasParam("pwm_freq", true)) {
                uint32_t val = request->getParam("pwm_freq", true)->value().toInt();
                if (val >= 100 && val <= 40000) { // Increased max to 40,000 Hz
                    blindPtr->setPwmFrequency(val);
                    ledcSetup(blindPtr->getPwmChannel(), val, 8);
                }
            }
            request->redirect("/");
        } else {
            // Invalid: do not apply, maybe add a flash message in future
            request->redirect("/?error=pwmall");
        }
    });
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        Serial.printf("[DEBUG] WebSocket event: %d\n", type);
        if (type == WS_EVT_CONNECT) {
            wsClientCount++;
            configPortalActive = true;
            notifyAllClients();
        } else if (type == WS_EVT_DISCONNECT) {
            wsClientCount--;
            if (wsClientCount <= 0) {
                wsClientCount = 0;
                configPortalActive = false;
            }
        } else if (type == WS_EVT_DATA) {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                String msg = String((char*)data).substring(0, len);
                Serial.printf("[DEBUG] WS RX: %s\n", msg.c_str());
                DynamicJsonDocument doc(128);
                DeserializationError err = deserializeJson(doc, msg);
                if (!err) {
                    // --- Support both old and new frontend message formats ---
                    if (doc.containsKey("setBlindPos") && doc["setBlindPos"].is<int>()) {
                        int pos = doc["setBlindPos"];
                        Serial.printf("[DEBUG] Main slider setBlindPos: %d\n", pos);
                        blindPtr->commandMove(pos);
                        notifyAllClients();
                    } else if (doc.containsKey("setSlavePos") && doc.containsKey("slave") && doc["setSlavePos"].is<int>() && doc["slave"].is<int>()) {
                        uint8_t slave = doc["slave"];
                        uint8_t pos = doc["setSlavePos"];
                        Serial.printf("[DEBUG] Slave slider setBlindPos: slave=%u pos=%u\n", slave, pos);
                        blind_command_t cmd = {};
                        cmd.blindNumber = slave;
                        cmd.set_state = 0x04; // set position
                        cmd.set_position = pos; // send 0-100 directly
                        Serial.printf("[DEBUG] Sending to slave %u: set_position=%u (raw UI pos=%u)\n", slave, cmd.set_position, pos);
                        uint8_t payload[8] = {0};
                        memcpy(payload, &cmd, sizeof(cmd));
                        sendCommandToSlaveUART(slave, BLIND_COMMAND, payload, sizeof(payload));
                        // Set slave_moving to true to increase polling rate
                        extern bool slave_moving[];
                        slave_moving[slave] = true;
                    } else if (doc.containsKey("type") && doc["type"] == "move" && doc.containsKey("pos")) {
                        int pos = doc["pos"];
                        Serial.printf("[DEBUG] Main slider WS type=move: %d\n", pos);
                        blindPtr->commandMove(pos * 10); // Scale 0-100 to 0-1000
                        notifyAllClients();
                    } else if (doc.containsKey("type") && doc["type"] == "slave_move" && doc.containsKey("id") && doc.containsKey("pos")) {
                        uint8_t slave = doc["id"];
                        uint8_t pos = doc["pos"];
                        Serial.printf("[DEBUG] Slave slider WS type=slave_move: slave=%u pos=%u\n", slave, pos);
                        blind_command_t cmd = {};
                        cmd.blindNumber = slave;
                        cmd.set_state = 0x04; // set position
                        cmd.set_position = pos; // send 0-100 directly
                        Serial.printf("[DEBUG] Sending to slave %u: set_position=%u (raw UI pos=%u)\n", slave, cmd.set_position, pos);
                        uint8_t payload[8] = {0};
                        memcpy(payload, &cmd, sizeof(cmd));
                        sendCommandToSlaveUART(slave, BLIND_COMMAND, payload, sizeof(payload));
                        // Set slave_moving to true to increase polling rate
                        extern bool slave_moving[];
                        slave_moving[slave] = true;
                    }
                }
            }
        }
    });
    server.addHandler(&ws);
    Serial.println("[DEBUG] Web portal endpoints and websocket handler added.");
    webPortalPositionTicker.attach_ms(300, notifyAllClients); // 300ms for fast UI updates
    server.on("/help", HTTP_GET, [](AsyncWebServerRequest *request){
        String helpHtml = R"rawliteral(
<html><head><title>SleepyLora Blinds Help</title><style>
body { font-family: monospace; background: #f8f9fa; margin: 0; padding: 0; }
.help-container { max-width: 800px; margin: 32px auto; background: #fff; border-radius: 8px; box-shadow: 0 2px 8px #ccc; padding: 32px; }
h1 { text-align: center; }
h2 { margin-top: 28px; }
dt { font-weight: bold; margin-top: 12px; }
dd { margin-left: 18px; margin-bottom: 10px; }
pre.ascii-art { text-align: center; font-size: 1.1em; margin-bottom: 18px; color: #444; }
</style></head><body>
<div class='help-container'>
<pre class='ascii-art'>{{ASCII_ART}}</pre>
<h1>SleepyLora Blinds Settings Help</h1>
<dl>
<dt>Closed Limit / Open Limit</dt><dd>The raw ADC value representing the fully closed or fully open position of the blind. Used to calibrate the range of movement. Set by moving the blind to the desired position and pressing "Set Current Position" or entering a value manually. This allows for the actuator to be used on the left or right of the blinds.</dd>
<dt>Move Time (ms)</dt><dd>The time (in milliseconds) the blind should take to move from fully closed to fully open (or vice versa). Used for time-based moves allowing syncronisation of blinds and also allowing the blind to run at minimum pwm to reduce noise.</dd>
<dt>Auto Calibrate</dt><dd>Performs a calibration sequence where the actuator is cycled to determine the optimal disengage times for the current PWM settings</dd>
<dt>Engage Time (ms) </dt><dd>The time (in ms) to that the controller will sample for engagement, typically Disengage time x 2 + 500ms.</dd>
<dt>Disengage Time (ms)</dt><dd>The time (in ms) to apply power to the motor to disengage the mechanism after a move allowing manual adjustment of the blinds</dd>
<dt>Engage Threshold</dt><dd>The minimum change in position (ADC units) between samples required to detect that the mechanism has engaged. Review the serial logs to determine the appropriate value. After the engage has been detected or a timout occurs the list of positions will be dispayed, you should see a step change after engagement occurs</dd>
<dt>Engage Sample Count</dt><dd>The number of consecutive samples used to detect engagement. Higher values make detection less sensitive to noise but will be harder to trigger.</dd>
<dt>Stall Threshold</dt><dd>The minimum change in position (ADC units) between samples required to detect that the motor is not stalled.</dd>
<dt>Stall Sample Count</dt><dd>The number of consecutive samples used to detect a stall. Higher values make detection more robust but slower to detect a stall.</dd>
<dt>Engage PWM / Disengage PWM</dt><dd>The PWM value (0-255) applied to the motor during engage/disengage phases. adjusting this value will require a change of disengage time.</dd>
<dt>Min PWM / Max PWM</dt><dd>The minimum and maximum PWM values (0-255) used during normal movement. Min PWM ensures the motor starts moving; Max PWM limits the maximum speed. setting min to the same as max will bypass the PID controller and run the motor at the desires fixed PWM</dd>
<dt>PWM Frequency (Hz)</dt><dd>The frequency of the PWM signal sent to the motor driver. Typical range: 100-40,000 Hz.</dd>
<dt>PID Controller Settings</dt><dd><ul><li><b>Kp (Proportional Gain):</b> Controls how strongly the controller reacts to the current error (difference between target and actual position).</li><li><b>Ki (Integral Gain):</b> Controls how strongly the controller reacts to the accumulation of past errors (helps eliminate steady-state error).</li><li><b>Kd (Derivative Gain):</b> Controls how strongly the controller reacts to the rate of change of the error (helps dampen oscillations).</li><li><b>Deadband:</b> The error range within which the controller will not react (prevents unnecessary movement for small errors).</li><li><b>Integral Min/Max:</b> Limits for the integral term to prevent windup.</li><li><b>Deriv Min/Max:</b> Limits for the derivative term to prevent excessive correction.</li><li><b>Deriv Alpha:</b> Smoothing factor for the derivative term (0-1, higher = less smoothing).</li><li><b>PWM Alpha:</b> Smoothing factor for the PWM output (0-1, higher = less smoothing).</li></ul></dd>
<dt>Slave Devices</dt><dd><ul><li><b>Scan for Slaves:</b> Searches for connected slave devices on the RS485 bus.</li><li><b>Open Web Portal:</b> Sends a command to the slave to open its web portal.</li><li><b>Slave Position:</b> Shows and allows control of the position of each slave blind.</li></ul></dd>
<dt>Firmware Update (OTA)</dt><dd>Allows uploading a new firmware binary to update the device.</dd>
<dt>Device Keys & Gateway Configuration</dt><dd><ul><li><b>Gateway ID:</b> Unique identifier for the gateway device (hexadecimal) only messages from this device will be accepted.</li><li><b>AES Key:</b> 16-byte (32 hex digits) encryption key for secure communication.</li><li><b>HMAC Key:</b> 10-byte (20 hex digits) key for message authentication.</li><li><b>RF Frequency:</b> The radio frequency (in Hz) used for LoRa communication.</li></ul></dd>
</dl>
<div style='text-align:center;margin-top:32px;'><a href='/' style='font-size:1.1em;color:#007bff;'>Back to Main Page</a></div>
</div></body></html>
)rawliteral";
        // Insert ASCII art and replace any template variables
        helpHtml.replace("{{ASCII_ART}}", String(BLIND_ASCII_ART));
        request->send(200, "text/html", helpHtml);
    });
}
