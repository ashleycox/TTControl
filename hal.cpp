/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "hal.h"

extern "C" {
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
}

HardwareAbstraction hal;

HardwareAbstraction::HardwareAbstraction() {
    _watchdogEnabled = false;
}

void HardwareAbstraction::begin() {
    // Initialize any global hardware states here if needed
}

void HardwareAbstraction::setPinMode(int pin, int mode) {
    pinMode(pin, mode);
}

void HardwareAbstraction::digitalWrite(int pin, int value) {
    ::digitalWrite(pin, (PinStatus)value);
}

int HardwareAbstraction::digitalRead(int pin) {
    return ::digitalRead(pin);
}

void HardwareAbstraction::analogWrite(int pin, int value) {
    ::analogWrite(pin, value);
}

void HardwareAbstraction::setPWMFreq(int freq) {
    ::analogWriteFreq(freq);
}

void HardwareAbstraction::setPWMRange(int range) {
    ::analogWriteRange(range);
}

void HardwareAbstraction::watchdogEnable(int timeoutMs) {
    // RP2040 Watchdog Max timeout is approx 8.3 seconds
    if (timeoutMs > 8300) timeoutMs = 8300;
    
    // Enable watchdog with pause on debug support
    watchdog_enable(timeoutMs, 1); 
    _watchdogEnabled = true;
}

void HardwareAbstraction::watchdogFeed() {
    if (_watchdogEnabled) {
        watchdog_update();
    }
}

void HardwareAbstraction::watchdogReboot() {
    // Force immediate reboot
    watchdog_reboot(0, 0, 0);
}

ResetCause HardwareAbstraction::getResetCause() {
    switch (rp2040.getResetReason()) {
        case RP2040::PWRON_RESET: return RESET_CAUSE_POWER_ON;
        case RP2040::RUN_PIN_RESET: return RESET_CAUSE_RUN_PIN;
        case RP2040::SOFT_RESET: return RESET_CAUSE_SOFT;
        case RP2040::WDT_RESET: return RESET_CAUSE_WATCHDOG;
        case RP2040::DEBUG_RESET: return RESET_CAUSE_DEBUG;
        case RP2040::GLITCH_RESET: return RESET_CAUSE_GLITCH;
        case RP2040::BROWNOUT_RESET: return RESET_CAUSE_BROWNOUT;
        case RP2040::UNKNOWN_RESET:
        default: return RESET_CAUSE_UNKNOWN;
    }
}

const char* HardwareAbstraction::resetCauseName(ResetCause cause) {
    switch (cause) {
        case RESET_CAUSE_POWER_ON: return "power-on";
        case RESET_CAUSE_RUN_PIN: return "run-pin";
        case RESET_CAUSE_SOFT: return "soft-reset";
        case RESET_CAUSE_WATCHDOG: return "watchdog";
        case RESET_CAUSE_DEBUG: return "debug";
        case RESET_CAUSE_GLITCH: return "glitch";
        case RESET_CAUSE_BROWNOUT: return "brownout";
        case RESET_CAUSE_UNKNOWN:
        default: return "unknown";
    }
}

uint32_t HardwareAbstraction::getMicros() {
    return micros();
}

uint32_t HardwareAbstraction::getMillis() {
    return millis();
}

void HardwareAbstraction::delayMs(uint32_t ms) {
    delay(ms);
}

void HardwareAbstraction::setMuteRelay(int index, bool active) {
    // Helper to map index to physical pin
    // Note: Polarity and timing are handled by MotorController
    int pin = -1;
    switch(index) {
        case 0: pin = PIN_MUTE_PHASE_A; break;
        case 1: pin = PIN_MUTE_PHASE_B; break;
        case 2: pin = PIN_MUTE_PHASE_C; break;
        case 3: pin = PIN_MUTE_PHASE_D; break;
    }
    
    if (pin != -1) {
        ::digitalWrite(pin, active ? HIGH : LOW);
    }
}

void HardwareAbstraction::setStandbyRelay(bool active) {
    ::digitalWrite(PIN_RELAY_STANDBY, active ? HIGH : LOW);
}
