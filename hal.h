/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef HAL_H
#define HAL_H

#include <Arduino.h>
#include "config.h"

enum ResetCause {
    RESET_CAUSE_UNKNOWN,
    RESET_CAUSE_POWER_ON,
    RESET_CAUSE_RUN_PIN,
    RESET_CAUSE_SOFT,
    RESET_CAUSE_WATCHDOG,
    RESET_CAUSE_DEBUG,
    RESET_CAUSE_GLITCH,
    RESET_CAUSE_BROWNOUT
};

/**
 * @brief Hardware Abstraction Layer.
 * 
 * Centralizes non-waveform hardware interactions so most modules do not call
 * Arduino or RP2040 APIs directly. WaveformGenerator still uses low-level
 * PWM/DMA APIs because those paths are timing critical.
 */
class HardwareAbstraction {
public:
    HardwareAbstraction();
    
    void begin();
    
    // --- GPIO Control ---
    void setPinMode(int pin, int mode);
    void digitalWrite(int pin, int value);
    int digitalRead(int pin);
    
    /*
     * --- PWM Control ---
     * Generic Arduino PWM wrappers. The DDS output code configures PWM directly
     * in waveform.cpp and does not rely on these helpers.
     */
    void analogWrite(int pin, int value);
    void setPWMFreq(int freq);
    void setPWMRange(int range);
    
    // --- Watchdog Timer ---
    void watchdogEnable(int timeoutMs);
    void watchdogFeed();
    void watchdogReboot(); // Triggers immediate system reset
    ResetCause getResetCause();
    const char* resetCauseName(ResetCause cause);
    
    // --- Timing ---
    uint32_t getMicros();
    uint32_t getMillis();
    void delayMs(uint32_t ms);
    
    /*
     * --- Semantic Hardware Control ---
     * These functions perform pin mapping only. Polarity, staggered sequencing,
     * and safety timing stay in MotorController.
     */
    void setMuteRelay(int index, bool active);
    void setStandbyRelay(bool active);
    
private:
    // Prevent watchdog updates before watchdog_enable() has actually run.
    bool _watchdogEnabled;
};

extern HardwareAbstraction hal;

#endif // HAL_H
