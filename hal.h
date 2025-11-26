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

/**
 * @brief Hardware Abstraction Layer.
 * 
 * Centralizes all direct hardware interactions to improve portability and testability.
 * Wraps Arduino and RP2040-specific APIs (GPIO, PWM, Watchdog, Timing).
 */
class HardwareAbstraction {
public:
    HardwareAbstraction();
    
    void begin();
    
    // --- GPIO Control ---
    void setPinMode(int pin, int mode);
    void digitalWrite(int pin, int value);
    int digitalRead(int pin);
    
    // --- PWM Control ---
    void analogWrite(int pin, int value);
    void setPWMFreq(int freq);
    void setPWMRange(int range);
    
    // --- Watchdog Timer ---
    void watchdogEnable(int timeoutMs);
    void watchdogFeed();
    void watchdogReboot(); // Triggers immediate system reset
    
    // --- Timing ---
    uint32_t getMicros();
    uint32_t getMillis();
    void delayMs(uint32_t ms);
    
    // --- Semantic Hardware Control ---
    void setMuteRelay(int index, bool active); // Direct pin control (logic handled in MotorController)
    void setStandbyRelay(bool active);
    
private:
    bool _watchdogEnabled;
};

extern HardwareAbstraction hal;

#endif // HAL_H
