// SPDX-License-Identifier: MIT
// Logging system implementation for SleepyLoraGateway
#include "logger.h"
#include <stdarg.h>

GatewayLogger Logger;

void GatewayLogger::begin() {
    LittleFS.begin();
    removeOldLogs();
    lastFlushDay = (time(nullptr) / 86400);
}

void GatewayLogger::log(const char* level, const char* event, const char* fmt, ...) {
    LogEntry& entry = buffer[bufHead];
    entry.timestamp = time(nullptr);
    strncpy(entry.level, level, sizeof(entry.level) - 1);
    entry.level[sizeof(entry.level) - 1] = 0;
    strncpy(entry.event, event, sizeof(entry.event) - 1);
    entry.event[sizeof(entry.event) - 1] = 0;
    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.message, sizeof(entry.message), fmt, args);
    va_end(args);
    bufHead = (bufHead + 1) % LOG_BUFFER_SIZE;
    if (bufCount < LOG_BUFFER_SIZE) bufCount++;
    flushIfNeeded();
}

void GatewayLogger::flushIfNeeded() {
    time_t now = time(nullptr);
    time_t day = now / 86400;
    if (day != lastFlushDay) {
        flush();
        rotateIfNeeded();
        lastFlushDay = day;
    } else {
        File f = LittleFS.open(currentLogFileName(), "a");
        if (f && f.size() > LOG_FILE_MAX_SIZE) {
            flush();
            rotateIfNeeded();
        }
        if (f) f.close();
    }
}

void GatewayLogger::flush() {
    if (bufCount == 0) return;
    File f = LittleFS.open(currentLogFileName(), "a");
    if (!f) return;
    for (uint8_t i = 0; i < bufCount; ++i) {
        uint8_t idx = (bufHead + LOG_BUFFER_SIZE - bufCount + i) % LOG_BUFFER_SIZE;
        LogEntry& e = buffer[idx];
        char line[LOG_ENTRY_MAXLEN + 64];
        struct tm* tm = localtime(&e.timestamp);
        snprintf(line, sizeof(line), "%04d-%02d-%02dT%02d:%02d:%02d,%s,%s,%s\n",
            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec,
            e.level, e.event, e.message);
        f.print(line);
    }
    f.close();
    bufCount = 0;
}

void GatewayLogger::rotateIfNeeded() {
    removeOldLogs();
}

String GatewayLogger::currentLogFileName() {
    time_t now = time(nullptr);
    struct tm* tm = localtime(&now);
    char fname[32];
    snprintf(fname, sizeof(fname), "/log_%04d%02d%02d.txt", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
    return String(fname);
}

void GatewayLogger::setRetentionDays(uint8_t days) {
    retentionDays = days;
}

uint8_t GatewayLogger::getRetentionDays() const {
    return retentionDays;
}

void GatewayLogger::clearLogs() {
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String name = file.name();
        if (name.startsWith("/log_")) LittleFS.remove(name);
        file = root.openNextFile();
    }
}

void GatewayLogger::listLogs(String& out) {
    File root = LittleFS.open("/");
    if (!root) {
        Serial.println("[Logger] LittleFS.open('/') failed in listLogs");
        return;
    }
    File file = root.openNextFile();
    Serial.println("[Logger] Listing files in LittleFS root:");
    while (file) {
        String name = file.name();
        Serial.print("[Logger] Found file: ");
        Serial.println(name);
        if (name.startsWith("/log_")) {
            out += name;
            out += "\n";
        } else if (name.startsWith("log_")) { // handle files without leading slash
            out += "/";
            out += name;
            out += "\n";
        }
        file = root.openNextFile();
    }
    Serial.print("[Logger] Final out string: ");
    Serial.println(out);
}

bool GatewayLogger::getLogFile(const String& filename, File& file) {
    file = LittleFS.open(filename, "r");
    return file;
}

void GatewayLogger::removeOldLogs() {
    time_t now = time(nullptr);
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String name = file.name();
        if (name.startsWith("/log_")) {
            int y, m, d;
            if (sscanf(name.c_str(), "/log_%4d%2d%2d.txt", &y, &m, &d) == 3) {
                struct tm tm = {0};
                tm.tm_year = y - 1900;
                tm.tm_mon = m - 1;
                tm.tm_mday = d;
                time_t fileTime = mktime(&tm);
                if ((now - fileTime) > (retentionDays * 86400L)) {
                    LittleFS.remove(name);
                }
            }
        }
        file = root.openNextFile();
    }
}
