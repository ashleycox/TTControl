/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <vector>
#include "hal.h"

enum ErrorCode {
    ERR_NONE = 0,
    ERR_SYSTEM_FREEZE = 1,
    ERR_MOTOR_STALL = 2,
    ERR_SETTINGS_CORRUPT = 3,
    ERR_I2C_FAILURE = 4,
    ERR_OUT_OF_MEMORY = 5
};

/**
 * @brief Centralized Error Handling and Logging.
 * 
 * Capabilities:
 * - Logs errors to Serial console
 * - Appends errors to persistent file (/error.log)
 * - Triggers UI alerts
 * - Tracks critical system state
 */
class ErrorHandler {
public:
    ErrorHandler();
    
    void begin();
    
    // Report an error occurrence
    void report(ErrorCode code, const char* message, bool critical = false);
    
    // Clear all persistent logs
    void clearLogs();
    
    // Stream entire log to output (e.g. Serial)
    void dumpLog(Stream& out);
    
    // Retrieve log lines for UI display
    void getLogLines(std::vector<String>& lines, int maxLines = 50);
    
    // Check if a critical error has occurred since boot
    bool hasCriticalError() { return _criticalError; }
    
private:
    bool _criticalError;
    
    void logToFile(ErrorCode code, const char* message);
};

extern ErrorHandler errorHandler;

#endif // ERROR_HANDLER_H
