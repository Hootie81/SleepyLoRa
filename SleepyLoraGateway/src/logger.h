// SPDX-License-Identifier: MIT
// Logging system for SleepyLoraGateway
// Buffers log entries in RAM, flushes to LittleFS file at midnight or when file size limit is reached.
// Log files are rotated daily and retained for a user-configurable number of days (default 7).

#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <sys/time.h>

#define LOG_BUFFER_SIZE 32  // Number of log entries in RAM buffer
#define LOG_ENTRY_MAXLEN 192
#define LOG_FILE_MAX_SIZE (512 * 1024) // 512KB per file
#define LOG_RETENTION_DEFAULT 7 // days

struct LogEntry {
    struct timeval tv; // microsecond timestamp
    char level[8];
    char event[16];
    char message[LOG_ENTRY_MAXLEN];
};

class GatewayLogger {
public:
    void begin();
    void log(const char* level, const char* event, const char* fmt, ...);
    void flushIfNeeded();
    void flush(); // Moved to public
    void rotateIfNeeded();
    void setRetentionDays(uint8_t days);
    uint8_t getRetentionDays() const;
    void clearLogs();
    void listLogs(String& out);
    bool getLogFile(const String& filename, File& file);
    void removeOldLogs();
    String currentLogFileName(); // Make this public for debug
    void formatFS();
private:
    LogEntry buffer[LOG_BUFFER_SIZE];
    uint8_t bufHead = 0, bufCount = 0;
    uint8_t retentionDays = LOG_RETENTION_DEFAULT;
    time_t lastFlushDay = 0;
    uint8_t logChunk = 0; // For chunked log rotation
    void deleteOldestLogFile();
};

extern GatewayLogger Logger;
