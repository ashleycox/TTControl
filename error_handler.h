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
    // Codes are persisted in /error.log. Add new codes at the end so older logs
    // remain readable.
    ERR_NONE = 0,
    ERR_SYSTEM_FREEZE = 1,
    ERR_MOTOR_STALL = 2,
    ERR_SETTINGS_CORRUPT = 3,
    ERR_I2C_FAILURE = 4,
    ERR_OUT_OF_MEMORY = 5,
    ERR_AMP_THERMAL = 6,
    ERR_SPEED_FEEDBACK = 7,
    ERR_RESET_CAUSE = 8,
    ERR_WAVEFORM_HEALTH = 9,
    ERR_SETTINGS_ROLLBACK = 10
};

/**
 * @brief Centralized Error Handling and Logging.
 * 
 * Logs events/errors to Serial and LittleFS, raises OLED alerts for report(),
 * and latches critical state. Critical reports force MotorController through the
 * same emergency-stop path used by user commands.
 */
class ErrorHandler {
public:
    ErrorHandler();
    
    void begin();
    
    // Report an error occurrence
    void report(ErrorCode code, const char* message, bool critical = false);

    // Record an event without triggering UI display or motor actions
    void logEvent(ErrorCode code, const char* message);
    
    // Clear all persistent logs
    void clearLogs();
    
    // Stream entire log to output (e.g. Serial)
    void dumpLog(Stream& out);
    
    // Retrieve log lines for UI display
    void getLogLines(std::vector<String>& lines, int maxLines = 50);
    
    // Check if a critical error has occurred since boot
    bool hasCriticalError() { return _criticalError; }
    
private:
    // Latched until reboot. The UI and diagnostics use this to show that a
    // critical fault occurred even if outputs have already been shut down.
    bool _criticalError;
    
    void logToFile(ErrorCode code, const char* message);
};

extern ErrorHandler errorHandler;

#endif // ERROR_HANDLER_H
