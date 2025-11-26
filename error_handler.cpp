/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "error_handler.h"
#include "error_handler.h"
#include "ui.h"
#include "settings.h" // Added missing include

extern UserInterface ui;

ErrorHandler errorHandler;

ErrorHandler::ErrorHandler() {
    _criticalError = false;
}

void ErrorHandler::begin() {
    // Filesystem initialization is handled by Settings class
}

void ErrorHandler::report(ErrorCode code, const char* message, bool critical) {
    if (critical) _criticalError = true;
    
    // 1. Log to Serial Console for debugging
    Serial.print("ERROR ");
    Serial.print(code);
    Serial.print(": ");
    Serial.println(message);
    
    // 2. Append to persistent log file
    logToFile(code, message);
    
    // 3. Display visual alert on UI
    if (settings.get().errorDisplayEnabled) {
        // Critical errors stay longer (10s) vs warnings (3s) -> Now Configurable
        uint32_t duration = settings.get().errorDisplayDuration * 1000;
        if (critical && duration < 10000) duration = 10000; // Force min 10s for critical
        ui.showError(message, duration);
    }
    
    // Note: If critical, the system should ideally stop the motor.
    // This is currently handled by the UI or main loop checking `hasCriticalError()`
    // or by the caller of `report()`.
}

void ErrorHandler::logToFile(ErrorCode code, const char* message) {
    // Check file size first
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
        f.print(hal.getMillis());
        f.print(",");
        f.print(code);
        f.print(",");
        f.println(message);
        f.close();
    }
}

void ErrorHandler::clearLogs() {
    LittleFS.remove("/error.log");
}

void ErrorHandler::dumpLog(Stream& out) {
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
    File f = LittleFS.open("/error.log", "r");
    if (!f) return;
    
    // Read lines from the beginning of the file
    // TODO: For large logs, implementing a tail reader would be more efficient
    while (f.available() && lines.size() < maxLines) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            lines.push_back(line);
        }
    }
    f.close();
}
