/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "error_handler.h"
#include "ui.h"
#include "settings.h"
#include "motor.h"
#include "globals.h"

extern UserInterface ui;

ErrorHandler errorHandler;

ErrorHandler::ErrorHandler() {
    _criticalError = false;
    _criticalCode = ERR_NONE;
    _criticalMessage[0] = 0;
    _sessionId = 0;
}

void ErrorHandler::begin() {
    // Filesystem initialization is handled by Settings before ErrorHandler starts.
    if (_sessionId == 0) {
        _sessionId = rp2040.hwrand32();
        if (_sessionId == 0) _sessionId = micros() ^ (uint32_t)(uintptr_t)this;
    }
}

void ErrorHandler::logEvent(ErrorCode code, const char* message) {
    if (_sessionId == 0) {
        _sessionId = rp2040.hwrand32();
        if (_sessionId == 0) _sessionId = micros() ^ (uint32_t)(uintptr_t)this;
    }
    // logEvent is for informational boot/runtime entries. It does not trigger UI alerts or motor actions.
    if (SERIAL_MONITOR_ENABLE) {
        Serial.print("ERROR ");
        Serial.print(code);
        Serial.print(": ");
        Serial.println(message);
    }

    logToFile(code, message);
}

void ErrorHandler::report(ErrorCode code, const char* message, bool critical) {
    if (critical) {
        // Stop outputs before doing file/UI work so safety action is not delayed by flash or display operations.
        _criticalError = true;
        _criticalCode = code;
        snprintf(_criticalMessage, sizeof(_criticalMessage), "%s", message ? message : "Critical fault");
        motor.emergencyStop();
    }
    
    // 1. Log to Serial Console and persistent file
    logEvent(code, message);
    
    // 2. Display visual alert on UI
    if (settings.get().errorDisplayEnabled) {
        // Critical errors stay longer (10s) vs warnings (3s) -> Now Configurable
        uint32_t duration = settings.get().errorDisplayDuration * 1000;
        if (critical && duration < 10000) duration = 10000; // Force min 10s for critical
        ui.showError(message, duration, critical);
    }
}

void ErrorHandler::logToFile(ErrorCode code, const char* message) {
    // Safe Mode is a read-only recovery boot. Keep serial diagnostics available without changing flash contents.
    if (safeModeActive) return;

    // Keep the log bounded. A small log is enough for bench diagnostics and avoids filling the LittleFS partition after repeated warnings.
    File f = LittleFS.open("/error.log", "r");
    if (f) {
        size_t size = f.size();
        f.close();
        
        // Rotate if > 10KB
        if (size > 10240) {
            LittleFS.remove("/error.bak");
            LittleFS.rename("/error.log", "/error.bak");
        }
    }

    f = LittleFS.open("/error.log", "a");
    if (f) {
        char session[9];
        snprintf(session, sizeof(session), "%08lx", (unsigned long)_sessionId);
        f.print(session);
        f.print(",");
        f.print(hal.getMillis());
        f.print(",");
        f.print(code);
        f.print(",");
        f.println(message);
        f.close();
    }
}

bool ErrorHandler::clearLogs() {
    if (safeModeActive) return false;
    bool currentRemoved = !LittleFS.exists("/error.log") || LittleFS.remove("/error.log");
    bool backupRemoved = !LittleFS.exists("/error.bak") || LittleFS.remove("/error.bak");
    return currentRemoved && backupRemoved;
}

void ErrorHandler::dumpLog(Stream& out) {
    // Serial command path: stream bytes directly rather than building a String.
    File f = LittleFS.open("/error.log", "r");
    if (!f) {
        out.println("No log file.");
        return;
    }
    
    while (f.available()) {
        out.write(f.read());
    }
    f.close();
}

void ErrorHandler::getLogLines(std::vector<String>& lines, int maxLines) {
    // UI/web path: return a capped number of lines so callers can render quickly.
    File f = LittleFS.open("/error.log", "r");
    if (!f) return;
    
    // Read from the beginning; log rotation keeps the file small.
    while (f.available() && (int)lines.size() < maxLines) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            lines.push_back(line);
        }
    }
    f.close();
}
