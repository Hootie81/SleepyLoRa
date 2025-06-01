#include "webserver.h"
#include "config.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>

extern GatewayConfig config;
extern uint32_t deviceID;
extern bool shouldClearConfig;
extern bool shouldStartConfig;

AsyncWebServer server(80);

void startConfigPortalAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("SleepyLoRaGateway", "blind1234");
    Serial.println("AP mode started. Connect to the device's AP and open the web page.");
}

void setupWebServer() {
    char aesKeyHex[33] = {0};
    for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config.aes_key[i]);
    char hmacKeyHex[21] = {0};
    for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config.hmacKey[i]);
    server.on("/", HTTP_GET, [aesKeyHex, hmacKeyHex](AsyncWebServerRequest *request){
        char deviceIdHex[9];
        snprintf(deviceIdHex, sizeof(deviceIdHex), "%08X", deviceID);
        String html = "<html><body style='font-family:monospace; background:#f8f9fa; margin:0; padding:0;'>"
            "<div style='max-width:480px;margin:32px auto 0 auto;padding:24px 24px 16px 24px;background:#fff;border-radius:10px;box-shadow:0 2px 12px #0001;'>"
            "<pre style='font-family:monospace; font-size:16px; text-align:center; margin:0 0 12px 0;'>\n"
            " ____  _                       _          ____          \n"
            "/ ___|| | ___  ___ _ __  _   _| |    ___ |  _ \\ __ _    \n"
            "\\___ \\| |/ _ \\/ _ \\ '_ \\| | | | |   / _ \\| |_) / _` |   \n"
            " ___) | |  __/  __/ |_) | |_| | |__| (_) |  _ < (_| |   \n"
            "|____/|_|\\___|\\___| .__/ \\__, |_____\\___/|_| \\_\\__,_|   \n"
            "                  |_|    |___/                          \n"
            "</pre>"
            "<h2 style='text-align:center;margin:0 0 18px 0;'>SleepyLora Gateway Config</h2>"
            "<form id='config-form' method='POST' action='/save' autocomplete='off'>"
            "<label>WiFi SSID:</label><br>"
            "<input name='wifi_ssid' id='wifi_ssid' value='" + String(config.wifi_ssid) + "'>"
            "<button type='button' onclick='scanWifi()'>Scan WiFi</button>"
            "<div id='wifi-list' style='margin:10px 0;'></div>"
            "<br>"
            "<label>WiFi Password:</label><br>"
            "<input name='wifi_pass' type='password' value='" + String(config.wifi_pass) + "'><br><br>"
            "<hr style='margin:18px 0;'>"
            "<label>MQTT Host:</label><br>"
            "<input name='mqtt_host' value='" + String(config.mqtt_host) + "'><br><br>"
            "<label>MQTT Port:</label><br>"
            "<input name='mqtt_port' value='" + String(config.mqtt_port) + "'><br><br>"
            "<label>MQTT User:</label><br>"
            "<input name='mqtt_user' value='" + String(config.mqtt_user) + "'><br><br>"
            "<label>MQTT Password:</label><br>"
            "<input name='mqtt_pass' type='password' value='" + String(config.mqtt_pass) + "'><br><br>"
            "<hr style='margin:18px 0;'>"
            "<label>NTP Server:</label><br>"
            "<input name='ntp_server' value='" + String(config.ntp_server) + "'><br><br>"
            "<label>NTP Offset (seconds):</label><br>"
            "<input name='ntp_offset' value='" + String(config.ntp_offset) + "'><br><br>"
            "<hr style='margin:18px 0;'>"
            "<label>AES Key (hex, 16 bytes):</label><br>"
            "<input name='aes_key' value='" + String(aesKeyHex) + "' maxlength='32' size='34'><br><br>"
            "<label>HMAC Key (hex, 10 bytes):</label><br>"
            "<input name='hmacKey' value='" + String(hmacKeyHex) + "' maxlength='20' size='22'><br><br>"
            "<button type='button' id='generate-btn' style='margin-bottom:20px;'>Generate Keys</button><br>"
            "<button type='button' id='download-btn' style='margin-bottom:20px;'>Download Keys</button><br>"
            "<input type='file' id='keyfile' style='display:none' accept='.json'>"
            "<button type='button' id='loadfile-btn' style='margin-bottom:20px;'>Load from File</button><br><br>"
            "<hr style='margin:18px 0;'>"
            "<label>LoRa Region/Frequency:</label><br>"
            "<select name='rf_frequency' id='rf_frequency'>"
            "<option value='868100000'" + String(config.rf_frequency == 868100000 ? " selected" : "") + ">EU868 (868.1 MHz)</option>"
            "<option value='915000000'" + String(config.rf_frequency == 915000000 ? " selected" : "") + ">US915/AU915 (915 MHz)</option>"
            "<option value='923200000'" + String(config.rf_frequency == 923200000 ? " selected" : "") + ">AS923 (923.2 MHz)</option>"
            "<option value='920900000'" + String(config.rf_frequency == 920900000 ? " selected" : "") + ">KR920 (920.9 MHz)</option>"
            "<option value='865062500'" + String(config.rf_frequency == 865062500 ? " selected" : "") + ">IN865 (865.0625 MHz)</option>"
            "<option value='433175000'" + String(config.rf_frequency == 433175000 ? " selected" : "") + ">EU433 (433.175 MHz)</option>"
            "<option value='470300000'" + String(config.rf_frequency == 470300000 ? " selected" : "") + ">CN470 (470.3 MHz)</option>"
            "</select><br><br>"
            "<div id='reboot-warning' style='display:none; color:#b00020; font-weight:bold; margin-bottom:10px;'>Keys have been updated. <br>Reboot for the changes to take effect.</div>"
            "<button id='reboot-btn' type='button' style='background:#dc3545;color:#fff;font-size:1.1em;padding:8px 28px;border:none;border-radius:6px;cursor:pointer;display:none;margin-bottom:16px;' onclick=\"rebootDevice()\">Reboot</button>"
            "<button type='submit' id='save-btn' style='background:#28a745;color:#fff;font-size:1.2em;padding:10px 32px;border:none;border-radius:6px;cursor:pointer;margin-top:10px;'>Save</button>"
            "</form>"
            "</div>"
            "<script>\n"
            "document.addEventListener('DOMContentLoaded', function() {\n"
            "  // Generate Keys\n"
            "  document.getElementById('generate-btn').onclick = function() {\n"
            "    fetch('/generate_keys').then(r => r.json()).then(j => {\n"
            "      document.querySelector('[name=aes_key]').value = j.aes_key;\n"
            "      document.querySelector('[name=hmacKey]').value = j.hmacKey;\n"
            "      showRebootWarning();\n"
            "    });\n"
            "  };\n"
            "  // Download Keys\n"
            "  document.getElementById('download-btn').onclick = function() {\n"
            "    const form = document.getElementById('config-form');\n"
            "    const formData = new FormData(form);\n"
            "    const params = new URLSearchParams();\n"
            "    for (const pair of formData.entries()) {\n"
            "      params.append(pair[0], pair[1]);\n"
            "    }\n"
            "    fetch('/save', {\n"
            "      method: 'POST',\n"
            "      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },\n"
            "      body: params.toString()\n"
            "    }).then(() => {\n"
            "      showRebootWarning();\n"
            "      fetch('/download_keys').then(r => r.json()).then(j => {\n"
            "        const blob = new Blob([JSON.stringify(j, null, 2)], {type: 'application/json'});\n"
            "        const url = URL.createObjectURL(blob);\n"
            "        const a = document.createElement('a');\n"
            "        a.href = url;\n"
            "        a.download = 'gateway_keys.json';\n"
            "        document.body.appendChild(a);\n"
            "        a.click();\n"
            "        document.body.removeChild(a);\n"
            "        URL.revokeObjectURL(url);\n"
            "      });\n"
            "    });\n"
            "  };\n"
            "  // Load from File\n"
            "  document.getElementById('loadfile-btn').onclick = function() {\n"
            "    document.getElementById('keyfile').click();\n"
            "  };\n"
            "  document.getElementById('keyfile').onchange = function(event) {\n"
            "    const file = event.target.files[0];\n"
            "    if (!file) return;\n"
            "    const reader = new FileReader();\n"
            "    reader.onload = function(e) {\n"
            "      try {\n"
            "        const data = JSON.parse(e.target.result);\n"
            "        fetch('/upload_keys', {\n"
            "          method: 'POST',\n"
            "          headers: {'Content-Type': 'application/json'},\n"
            "          body: JSON.stringify(data)\n"
            "        }).then(r => r.json()).then(j => {\n"
            "          if (j.success) {\n"
            "            document.querySelector('[name=aes_key]').value = data.aes_key;\n"
            "            document.querySelector('[name=hmacKey]').value = data.hmacKey;\n"
            "            alert('Keys loaded!');\n"
            "            showRebootWarning();\n"
            "          } else {\n"
            "            alert('Invalid file or error loading keys.');\n"
            "          }\n"
            "        });\n"
            "      } catch (err) {\n"
            "        alert('Invalid JSON file.');\n"
            "      }\n"
            "    };\n"
            "    reader.readAsText(file);\n"
            "  };\n"
            "  // Save (AJAX)\n"
            "  document.getElementById('config-form').addEventListener('submit', function(e) {\n"
            "    e.preventDefault();\n"
            "    const form = e.target;\n"
            "    const formData = new FormData(form);\n"
            "    const params = new URLSearchParams();\n"
            "    for (const pair of formData.entries()) {\n"
            "      params.append(pair[0], pair[1]);\n"
            "    }\n"
            "    fetch('/save', {\n"
            "      method: 'POST',\n"
            "      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },\n"
            "      body: params.toString()\n"
            "    }).then(() => {\n"
            "      showRebootWarning();\n"
            "      alert('Config saved!');\n"
            "    });\n"
            "  });\n"
            "});\n"
            "function scanWifi() {\n"
            "  const btn = event.target;\n"
            "  btn.disabled = true;\n"
            "  btn.textContent = 'Scanning...';\n"
            "  fetch('/scan').then(r => r.json()).then(list => {\n"
            "    let html = '';\n"
            "    if (list.length === 0) html = '<i>No networks found</i>';\n"
            "    else html = list.map(x => `<a href='#' onclick=\"document.getElementById('wifi_ssid').value='${x.ssid}';return false;\">${x.ssid}</a>`).join('<br>');\n"
            "    document.getElementById('wifi-list').innerHTML = html;\n"
            "    btn.disabled = false;\n"
            "    btn.textContent = 'Scan WiFi';\n"
            "  }).catch(() => {\n"
            "    document.getElementById('wifi-list').innerHTML = '<i>Error scanning</i>';\n"
            "    btn.disabled = false;\n"
            "    btn.textContent = 'Scan WiFi';\n"
            "  });\n"
            "}\n"
            "function showRebootWarning() {\n"
            "  document.getElementById('reboot-warning').style.display = 'block';\n"
            "  document.getElementById('reboot-btn').style.display = 'inline-block';\n"
            "}\n"
            "function rebootDevice() {\n"
            "  fetch('/reboot', {method: 'POST'}).then(() => {\n"
            "    document.getElementById('reboot-btn').disabled = true;\n"
            "    document.getElementById('reboot-btn').textContent = 'Rebooting...';\n"
            "    setTimeout(function() { location.reload(); }, 15000);\n"
            "  });\n"
            "}\n"
            "</script>"
            "</body></html>";
        request->send(200, "text/html", html);
    });
    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("wifi_ssid", true)) strncpy(config.wifi_ssid, request->getParam("wifi_ssid", true)->value().c_str(), sizeof(config.wifi_ssid));
        if (request->hasParam("wifi_pass", true)) strncpy(config.wifi_pass, request->getParam("wifi_pass", true)->value().c_str(), sizeof(config.wifi_pass));
        if (request->hasParam("mqtt_host", true)) strncpy(config.mqtt_host, request->getParam("mqtt_host", true)->value().c_str(), sizeof(config.mqtt_host));
        if (request->hasParam("mqtt_port", true)) config.mqtt_port = atoi(request->getParam("mqtt_port", true)->value().c_str());
        if (request->hasParam("mqtt_user", true)) strncpy(config.mqtt_user, request->getParam("mqtt_user", true)->value().c_str(), sizeof(config.mqtt_user));
        if (request->hasParam("mqtt_pass", true)) strncpy(config.mqtt_pass, request->getParam("mqtt_pass", true)->value().c_str(), sizeof(config.mqtt_pass));
        if (request->hasParam("ntp_server", true)) strncpy(config.ntp_server, request->getParam("ntp_server", true)->value().c_str(), sizeof(config.ntp_server));
        if (request->hasParam("ntp_offset", true)) config.ntp_offset = atoi(request->getParam("ntp_offset", true)->value().c_str());
        if (request->hasParam("aes_key", true)) {
            String aesStr = request->getParam("aes_key", true)->value();
            for (int i = 0; i < 16 && i*2+1 < aesStr.length(); ++i) config.aes_key[i] = strtoul(aesStr.substring(i*2, i*2+2).c_str(), nullptr, 16);
        }
        if (request->hasParam("hmacKey", true)) {
            String hmacStr = request->getParam("hmacKey", true)->value();
            for (int i = 0; i < 10 && i*2+1 < hmacStr.length(); ++i) config.hmacKey[i] = strtoul(hmacStr.substring(i*2, i*2+2).c_str(), nullptr, 16);
        }
        if (request->hasParam("rf_frequency", true)) config.rf_frequency = atol(request->getParam("rf_frequency", true)->value().c_str());
        saveConfig();
        request->send(200, "text/html", "<html><body><h2>Saved!</h2></body></html>");
    });
    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "[";
        int n = WiFi.scanComplete();
        if(n == -2){
            WiFi.scanNetworks(true);
        } else if(n){
            for (int i = 0; i < n; ++i){
                if(i) json += ",";
                json += "{\"ssid\":\"" + WiFi.SSID(i) + "\"}";
            }
            WiFi.scanDelete();
            if(WiFi.scanComplete() == -2){
                WiFi.scanNetworks(true);
            }
        }
        json += "]";
        request->send(200, "application/json", json);
    });
    server.on("/generate_keys", HTTP_GET, [](AsyncWebServerRequest *request){
        for (int i = 0; i < 16; ++i) config.aes_key[i] = (uint8_t)random(0, 256);
        for (int i = 0; i < 10; ++i) config.hmacKey[i] = (uint8_t)random(0, 256);
        char aesKeyHex[33] = {0};
        char hmacKeyHex[21] = {0};
        for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config.aes_key[i]);
        for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config.hmacKey[i]);
        String json = String("{\"aes_key\":\"") + aesKeyHex + "\",\"hmacKey\":\"" + hmacKeyHex + "\"}";
        request->send(200, "application/json", json);
    });
    server.on("/download_keys", HTTP_GET, [](AsyncWebServerRequest *request){
        char aesKeyHex[33] = {0};
        char hmacKeyHex[21] = {0};
        for (int i = 0; i < 16; ++i) sprintf(&aesKeyHex[i*2], "%02X", config.aes_key[i]);
        for (int i = 0; i < 10; ++i) sprintf(&hmacKeyHex[i*2], "%02X", config.hmacKey[i]);
        String json = String("{\"gatewayID\":\"") + String(deviceID, HEX) + "\"," +
            "\"aes_key\":\"" + aesKeyHex + "\"," +
            "\"hmacKey\":\"" + hmacKeyHex + "\"," +
            "\"rf_frequency\":" + String(config.rf_frequency) + "}";
        request->send(200, "application/json", json);
    });
    server.on("/upload_keys", HTTP_POST, [](AsyncWebServerRequest *request){
        // Handler is now empty; actual processing is in onRequestBody
        request->send(200, "application/json", "{\"success\":true}");
    });
    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (request->url() == "/upload_keys" && request->method() == HTTP_POST) {
            String body = String((const char*)data, len);
            Serial.println("/upload_keys received body:");
            Serial.println(body);
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, body);
            if (err) {
                Serial.print("JSON parse error: ");
                Serial.println(err.c_str());
                request->send(400, "application/json", String("{\"success\":false,\"error\":\"Invalid JSON: ") + err.c_str() + "}" );
                return;
            }
            if (!doc["aes_key"].is<const char*>() || !doc["hmacKey"].is<const char*>()) {
                Serial.println("Missing required fields (aes_key, hmacKey)");
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Missing fields\"}");
                return;
            }
            const char* aesStr = doc["aes_key"];
            const char* hmacStr = doc["hmacKey"];
            if (strlen(aesStr) != 32 || strlen(hmacStr) != 20) {
                Serial.println("Key length error");
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Key length error\"}");
                return;
            }
            for (int i = 0; i < 16; ++i) config.aes_key[i] = strtoul(String(aesStr).substring(i*2, i*2+2).c_str(), nullptr, 16);
            for (int i = 0; i < 10; ++i) config.hmacKey[i] = strtoul(String(hmacStr).substring(i*2, i*2+2).c_str(), nullptr, 16);
            if (doc["rf_frequency"].is<uint32_t>()) {
                config.rf_frequency = doc["rf_frequency"].as<uint32_t>();
            }
            saveConfig();
            Serial.println("Keys and frequency updated from file.");
            request->send(200, "application/json", "{\"success\":true}");
        }
    });
    server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", "<html><body><h2>Rebooting...</h2></body></html>");
        delay(1000);
        ESP.restart();
    });
    server.begin();
    Serial.println("Web server started. Portal available on local network IP.");
}

void checkConfigButton() {
    static unsigned long pressStart = 0;
    static bool wasPressed = false;
    pinMode(0, INPUT_PULLUP);
    bool pressed = digitalRead(0) == LOW;
    if (pressed && !wasPressed) {
        pressStart = millis();
    } else if (!pressed && wasPressed) {
        unsigned long pressDuration = millis() - pressStart;
        if (pressDuration > 5000) {
            shouldClearConfig = true;
        } else if (pressDuration > 50) {
            shouldStartConfig = true;
        }
    }
    wasPressed = pressed;
}

