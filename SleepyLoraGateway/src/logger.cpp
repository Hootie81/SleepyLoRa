// SPDX-License-Identifier: MIT
// Logging system implementation for SleepyLoraGateway
#include "logger.h"
#include <stdarg.h>

GatewayLogger Logger;

void GatewayLogger::formatFS() {
    Serial.print("[Logger] Formatting LittleFS...\r\n");
    LittleFS.end();
    bool ok = LittleFS.format();
    Serial.printf("[Logger] LittleFS.format() returned: %s\r\n", ok ? "OK" : "FAIL");
    ok = LittleFS.begin();
    Serial.printf("[Logger] LittleFS.begin() after format: %s\r\n", ok ? "OK" : "FAIL");
}

void GatewayLogger::begin() {
    if (!LittleFS.begin()) {
        Serial.print("[Logger] LittleFS.begin() failed in begin(), attempting format...\r\n");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.print("[Logger] LittleFS.begin() failed after format! Logging disabled.\r\n");
            return;
        }
    }
    removeOldLogs();
    // Scan for existing log chunks for today
    time_t now = time(nullptr);
    struct tm* tm = localtime(&now);
    char prefix[24];
    snprintf(prefix, sizeof(prefix), "/log_%04d%02d%02d", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
    logChunk = 0;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String name = file.name();
        if (name.startsWith(prefix)) {
            // Check for chunked files: /log_YYYYMMDD.txt, /log_YYYYMMDD_1.txt, etc.
            if (name == String(prefix) + ".txt") {
                if (logChunk < 1) logChunk = 0;
            } else if (name.startsWith(String(prefix) + "_")) {
                int chunkNum = name.substring(String(prefix).length() + 1, name.length() - 4).toInt();
                if (chunkNum > logChunk) logChunk = chunkNum;
            }
        }
        file = root.openNextFile();
    }
    lastFlushDay = (now / 86400);
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

void GatewayLogger::deleteOldestLogFile() {
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    String oldestName;
    time_t oldestTime = LONG_MAX;
    while (file) {
        String name = file.name();
        if (name.startsWith("/log_")) {
            int y, m, d;
            if (sscanf(name.c_str(), "/log_%4d%2d%2d", &y, &m, &d) == 3) {
                struct tm tm = {0};
                tm.tm_year = y - 1900;
                tm.tm_mon = m - 1;
                tm.tm_mday = d;
                time_t fileTime = mktime(&tm);
                if (fileTime < oldestTime) {
                    oldestTime = fileTime;
                    oldestName = name;
                }
            }
        }
        file = root.openNextFile();
    }
    if (oldestName.length() > 0) {
        Serial.printf("[Logger] Free space low, deleting oldest log file: %s\r\n", oldestName.c_str());
        LittleFS.remove(oldestName);
    } else {
        Serial.print("[Logger] Free space low, but no log files found to delete!\r\n");
    }
}

void GatewayLogger::flushIfNeeded() {
    // Free space protection: delete oldest log if >90% full
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    if (total > 0 && used > (total * 9 / 10)) {
        Serial.printf("[Logger] LittleFS used %u/%u bytes (>90%%), deleting oldest log file.\r\n", (unsigned)used, (unsigned)total);
        deleteOldestLogFile();
    }
    time_t now = time(nullptr);
    time_t day = now / 86400;
    if (day != lastFlushDay) {
        logChunk = 0; // Reset chunk at midnight
        // Scan for existing chunks for new day
        struct tm* tm = localtime(&now);
        char prefix[24];
        snprintf(prefix, sizeof(prefix), "/log_%04d%02d%02d", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file) {
            String name = file.name();
            if (name.startsWith(prefix)) {
                if (name == String(prefix) + ".txt") {
                    if (logChunk < 1) logChunk = 0;
                } else if (name.startsWith(String(prefix) + "_")) {
                    int chunkNum = name.substring(String(prefix).length() + 1, name.length() - 4).toInt();
                    if (chunkNum > logChunk) logChunk = chunkNum;
                }
            }
            file = root.openNextFile();
        }
        flush();
        rotateIfNeeded();
        lastFlushDay = day;
    } else {
        File f = LittleFS.open(currentLogFileName(), "a");
        if (f && f.size() > LOG_FILE_MAX_SIZE) {
            flush();
            logChunk++;
            Serial.printf("[Logger] Log file exceeded %u bytes, rotating to chunk %u\r\n", LOG_FILE_MAX_SIZE, logChunk);
            flush(); // flush to new chunk file
            rotateIfNeeded();
        }
        if (f) f.close();
    }
}

void GatewayLogger::flush() {
    // Free space protection: delete oldest log if >90% full
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    if (total > 0 && used > (total * 9 / 10)) {
        Serial.printf("[Logger] LittleFS used %u/%u bytes (>90%%), deleting oldest log file.\r\n", (unsigned)used, (unsigned)total);
        deleteOldestLogFile();
    }
    if (bufCount == 0) {
        Serial.print("[Logger] flush: No entries to flush.\r\n");
        return;
    }
    String fname = currentLogFileName();
    if (!LittleFS.begin()) {
        Serial.print("[Logger] flush: LittleFS not mounted, attempting to mount...\r\n");
        if (!LittleFS.begin()) {
            Serial.print("[Logger] flush: LittleFS mount failed, aborting flush.\r\n");
            return;
        }
    }
    File f = LittleFS.open(fname, "a");
    if (!f) {
        Serial.printf("[Logger] flush: Failed to open %s for append!\r\n", fname.c_str());
        return;
    }
    size_t sizeBefore = f.size();
    Serial.printf("[Logger] flush: Opened %s for append, file size before: %u\r\n", fname.c_str(), (unsigned)sizeBefore);
    size_t totalWritten = 0;
    for (uint8_t i = 0; i < bufCount; ++i) {
        uint8_t idx = (bufHead + LOG_BUFFER_SIZE - bufCount + i) % LOG_BUFFER_SIZE;
        LogEntry& e = buffer[idx];
        char line[LOG_ENTRY_MAXLEN + 64];
        struct tm* tm = localtime(&e.timestamp);
        snprintf(line, sizeof(line), "%04d-%02d-%02dT%02d:%02d:%02d,%s,%s,%s\r\n",
            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec,
            e.level, e.event, e.message);
        size_t written = f.print(line);
        totalWritten += written;
        Serial.printf("[Logger] flush: Wrote %u bytes to %s\r\n", (unsigned)written, fname.c_str());
    }
    f.flush();
    size_t sizeAfter = f.size();
    Serial.printf("[Logger] flush: File size after: %u\r\n", (unsigned)sizeAfter);
    f.close();
    Serial.printf("[Logger] flush: Closed %s\r\n", fname.c_str());
    if (totalWritten > 0 && sizeAfter == sizeBefore) {
        Serial.printf("[Logger] WARNING: File size did not grow after flush! Possible corruption in %s. Rotating file.\r\n", fname.c_str());
        // Try to rename the file
        String corruptName = fname.substring(0, fname.length() - 4) + "_corrupt.txt";
        if (LittleFS.rename(fname, corruptName)) {
            Serial.printf("[Logger] Renamed corrupt file to %s\r\n", corruptName.c_str());
        } else {
            Serial.printf("[Logger] Failed to rename corrupt file %s\r\n", fname.c_str());
        }
        // Next flush will create a new file with the original name
    }
    bufCount = 0;
}

void GatewayLogger::rotateIfNeeded() {
    removeOldLogs();
}

String GatewayLogger::currentLogFileName() {
    time_t now = time(nullptr);
    struct tm* tm = localtime(&now);
    char fname[40];
    if (logChunk == 0) {
        snprintf(fname, sizeof(fname), "/log_%04d%02d%02d.txt", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
    } else {
        snprintf(fname, sizeof(fname), "/log_%04d%02d%02d_%u.txt", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, logChunk);
    }
    return String(fname);
}

void GatewayLogger::setRetentionDays(uint8_t days) {
    retentionDays = days;
}

uint8_t GatewayLogger::getRetentionDays() const {
    return retentionDays;
}

void GatewayLogger::clearLogs() {
    if (!LittleFS.begin()) {
        Serial.print("[Logger] clearLogs: LittleFS not mounted, attempting to mount...\r\n");
        if (!LittleFS.begin()) {
            Serial.print("[Logger] clearLogs: LittleFS mount failed, aborting clearLogs.\r\n");
            return;
        }
    }
    Serial.print("[Logger] clearLogs: Collecting log file names...\r\n");
    String logFiles[32];
    int logFileCount = 0;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file && logFileCount < 32) {
        String name = file.name();
        Serial.printf("[Logger] clearLogs: Found file: %s\r\n", name.c_str());
        if (name.startsWith("/log_") || name.startsWith("log_")) {
            // Always prepend / for deletion
            String delName = name.startsWith("/") ? name : ("/" + name);
            logFiles[logFileCount++] = delName;
        }
        file = root.openNextFile();
    }
    Serial.printf("[Logger] clearLogs: Will attempt to delete %d files.\r\n", logFileCount);
    for (int i = 0; i < logFileCount; ++i) {
        bool removed = LittleFS.remove(logFiles[i]);
        Serial.printf("[Logger] clearLogs: Remove %s: %s\r\n", logFiles[i].c_str(), removed ? "OK" : "FAIL");
    }
}

void GatewayLogger::listLogs(String& out) {
    File root = LittleFS.open("/");
    if (!root) {
        Serial.print("[Logger] LittleFS.open('/') failed in listLogs\r\n");
        return;
    }
    File file = root.openNextFile();
    Serial.print("[Logger] Listing files in LittleFS root:\r\n");
    while (file) {
        String name = file.name();
        Serial.printf("[Logger] Found file: %s\r\n", name.c_str());
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
    Serial.print(out);
    Serial.print("\r\n");
}

bool GatewayLogger::getLogFile(const String& filename, File& file) {
    if (!LittleFS.begin()) {
        Serial.print("[Logger] getLogFile: LittleFS not mounted, attempting to mount...\r\n");
        if (!LittleFS.begin()) {
            Serial.print("[Logger] getLogFile: LittleFS mount failed, aborting.\r\n");
            return false;
        }
    }
    file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.printf("[Logger] getLogFile: Failed to open %s for read!\r\n", filename.c_str());
        return false;
    }
    Serial.printf("[Logger] getLogFile: Opened %s for read, size: %u\r\n", filename.c_str(), file.size());
    return true;
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
