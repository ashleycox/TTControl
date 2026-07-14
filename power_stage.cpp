/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "power_stage.h"
#include "error_handler.h"
#include "waveform.h"
#include "globals.h"
#include "hal.h"
#include "settings.h"
#include "motor.h"
#include <string.h>

extern "C" {
#include <pico/stdlib.h>
}

PowerStage powerStage;
static PowerStage* powerStageInstance = nullptr;

PowerStage::PowerStage() {
    _enabled = false;
    _enablePending = false;
    _faultLatched = false;
    _faultReportPending = false;
    _enableRequestMs = 0;
    _enableRequestBufferCount = 0;
    _state = POWER_STAGE_DISABLED;
    _faultOriginState = POWER_STAGE_DISABLED;
    _phaseEnableMask = 0;
    _stateDeadlineMs = 0;
    memset(&_metrics, 0, sizeof(_metrics));
    memset(&_faultSnapshot, 0, sizeof(_faultSnapshot));
    powerStageInstance = this;
}

void PowerStage::begin() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    // Establish the hardware-off level before configuring any fault input or allowing Core 1 to start PWM.
#if POWER_STAGE_SHARED_ENABLE
    hal.setPinMode(PIN_POWER_STAGE_ENABLE, OUTPUT);
    writeEnable(false);
#endif
#if POWER_STAGE_PHASE_ENABLES
    hal.setPinMode(PIN_POWER_STAGE_PHASE_ENABLE_A, OUTPUT);
    hal.setPinMode(PIN_POWER_STAGE_PHASE_ENABLE_B, OUTPUT);
    hal.setPinMode(PIN_POWER_STAGE_PHASE_ENABLE_C, OUTPUT);
    writePhaseEnables(false);
#endif
#if POWER_STAGE_SLEEP_ENABLE
    hal.setPinMode(PIN_POWER_STAGE_SLEEP, OUTPUT);
    writeSleep(false);
#endif
#if POWER_STAGE_RESET_ENABLE
    hal.setPinMode(PIN_POWER_STAGE_RESET, OUTPUT);
    writeReset(true);
#endif
#if POWER_STAGE_FAULT_ENABLE
#if POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN
    hal.setPinMode(PIN_POWER_STAGE_ENABLE, OUTPUT);
    hal.digitalWrite(PIN_POWER_STAGE_ENABLE, LOW);
#else
    hal.setPinMode(PIN_POWER_STAGE_FAULT, INPUT_PULLUP);
#endif
    if (faultInputActive()) {
        _faultLatched = true;
        _faultReportPending = true;
    }
    attachInterrupt(digitalPinToInterrupt(POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN ? PIN_POWER_STAGE_ENABLE : PIN_POWER_STAGE_FAULT), PowerStage::faultInterruptHandler,
        POWER_STAGE_FAULT_ACTIVE_LOW ? FALLING : RISING);
#endif
    _enabled = false;
    _state = _faultLatched ? POWER_STAGE_FAULT_LATCHED : POWER_STAGE_DISABLED;
#else
    _enabled = false;
#endif
}

void PowerStage::update() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    if (_faultReportPending) {
        _faultReportPending = false;
        captureFaultSnapshot();
        char message[160];
        snprintf(message, sizeof(message), "Driver fault stage=%u motor=%u speed=%u freq=%.2fHz buffer=%lu clips=%lu/%lu/%lu/%lu",
            (unsigned)_faultSnapshot.originState, (unsigned)_faultSnapshot.motorState,
            (unsigned)_faultSnapshot.speed, _faultSnapshot.frequencyHz,
            (unsigned long)_faultSnapshot.bufferFillCount,
            (unsigned long)_faultSnapshot.clippingCount[0], (unsigned long)_faultSnapshot.clippingCount[1],
            (unsigned long)_faultSnapshot.clippingCount[2], (unsigned long)_faultSnapshot.clippingCount[3]);
        errorHandler.report(ERR_POWER_STAGE_FAULT, message, true);
        return;
    }

    if (!_enablePending || _faultLatched || faultInputActive()) return;
    uint32_t now = hal.getMillis();
    if (_state == POWER_STAGE_RESET_ASSERTED && (int32_t)(now - _stateDeadlineMs) >= 0) {
        writeReset(false);
        enterWakeSequence();
    } else if (_state == POWER_STAGE_WAKING && (int32_t)(now - _stateDeadlineMs) >= 0) {
        _state = POWER_STAGE_WAITING_NEUTRAL;
    } else if (_state == POWER_STAGE_WAITING_NEUTRAL &&
               waveform.getBufferFillCount() - _enableRequestBufferCount >= POWER_STAGE_NEUTRAL_BUFFER_COUNT) {
        writePhaseEnables(true);
        _state = POWER_STAGE_PHASE_ENABLING;
        _stateDeadlineMs = now + POWER_STAGE_PHASE_ENABLE_DELAY_MS;
    } else if (_state == POWER_STAGE_PHASE_ENABLING && (int32_t)(now - _stateDeadlineMs) >= 0) {
        writeEnable(true);
        if (faultInputActive()) {
            disableFromInterrupt();
            return;
        }
        _enabled = true;
        _enablePending = false;
        _state = POWER_STAGE_READY;
        _metrics.successfulEnables++;
    }
#endif
}

bool PowerStage::requestEnable() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    if (_faultLatched || faultInputActive()) {
        disable();
        return false;
    }
    _metrics.enableAttempts++;
    writeEnable(false);
    writePhaseEnables(false);
    _enabled = false;
    _enablePending = true;
    _enableRequestMs = hal.getMillis();
    _enableRequestBufferCount = waveform.getBufferFillCount();
#if POWER_STAGE_RESET_ENABLE
    writeReset(true);
    _state = POWER_STAGE_RESET_ASSERTED;
    _stateDeadlineMs = _enableRequestMs + POWER_STAGE_RESET_PULSE_MS;
#else
    enterWakeSequence();
#endif
    return true;
#else
    _enabled = true;
    return true;
#endif
}

void PowerStage::notifyRunning() {
    if (_enabled && (_state == POWER_STAGE_READY || _state == POWER_STAGE_PHASE_ENABLING)) _state = POWER_STAGE_RUNNING;
}

void PowerStage::notifyStopping() {
    if (_enabled) _state = POWER_STAGE_STOPPING;
}

void PowerStage::refreshPhaseEnables() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_PHASE_ENABLES
    // Reductions are safe immediately. Additions wait for the next start so neutral-buffer confirmation precedes hardware enable.
    if (!_faultLatched && (_enabled || _state == POWER_STAGE_PHASE_ENABLING)) {
        uint8_t desiredMask = configuredPhaseEnableMask();
        if ((desiredMask & ~_phaseEnableMask) == 0) writePhaseEnables(true);
    }
#endif
}

void PowerStage::disable() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    // Global disable leads the shutdown sequence; per-phase disable is a second independent barrier where available.
    _enabled = false;
    _enablePending = false;
    if (!_faultLatched) _state = POWER_STAGE_DISABLED;
    writeEnable(false);
    writePhaseEnables(false);
    writeSleep(false);
    writeReset(true);
#else
    _enabled = false;
    _state = POWER_STAGE_DISABLED;
#endif
}

bool PowerStage::isEnabled() const {
    return _enabled;
}

bool PowerStage::isEnablePending() const {
    return _enablePending;
}

bool PowerStage::hasFault() const {
    return _faultLatched || faultInputActive();
}

bool PowerStage::faultInputActive() const {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_FAULT_ENABLE
#if POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN
    // While disabled the controller itself holds the shared line low. It only
    // represents a driver fault after writeEnable(true) releases the line.
    if (_state != POWER_STAGE_PHASE_ENABLING && _state != POWER_STAGE_READY &&
        _state != POWER_STAGE_RUNNING && _state != POWER_STAGE_STOPPING) return false;
#endif
    int faultPin = POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN ? PIN_POWER_STAGE_ENABLE : PIN_POWER_STAGE_FAULT;
    int level = hal.digitalRead(faultPin);
    return POWER_STAGE_FAULT_ACTIVE_LOW ? level == LOW : level == HIGH;
#else
    return false;
#endif
}

const char* PowerStage::backendName() const {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    return "DRV8313 3-PWM bridge";
#else
    return "Linear PWM amplifier";
#endif
}

PowerStageState PowerStage::state() const { return _state; }

const char* PowerStage::stateName() const {
    static const char* const names[] = {"Disabled", "Reset", "Waking", "Neutral", "Phase enable", "Ready", "Running", "Stopping", "Fault"};
    uint8_t value = (uint8_t)_state;
    return value < sizeof(names) / sizeof(names[0]) ? names[value] : "Unknown";
}

PowerStageMetrics PowerStage::metrics() const { return _metrics; }
PowerStageFaultSnapshot PowerStage::faultSnapshot() const { return _faultSnapshot; }

void PowerStage::writeEnable(bool active) {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SHARED_ENABLE
#if POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN
    if (active) {
        gpio_pull_up(PIN_POWER_STAGE_ENABLE);
        gpio_set_dir(PIN_POWER_STAGE_ENABLE, GPIO_IN);
    } else {
        // Set the output latch before taking ownership of the shared line. This
        // path is also called by the fault ISR, so use the direct GPIO API.
        gpio_put(PIN_POWER_STAGE_ENABLE, false);
        gpio_set_dir(PIN_POWER_STAGE_ENABLE, GPIO_OUT);
    }
#else
    gpio_put(PIN_POWER_STAGE_ENABLE, active == (POWER_STAGE_ENABLE_ACTIVE_HIGH != 0));
#endif
#else
    (void)active;
#endif
}

void PowerStage::writeSleep(bool active) {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SLEEP_ENABLE
    gpio_put(PIN_POWER_STAGE_SLEEP, active == (POWER_STAGE_SLEEP_ACTIVE_HIGH != 0));
#else
    (void)active;
#endif
}

void PowerStage::writeReset(bool asserted) {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_RESET_ENABLE
    gpio_put(PIN_POWER_STAGE_RESET, asserted == (POWER_STAGE_RESET_ACTIVE_HIGH != 0));
#else
    (void)asserted;
#endif
}

void PowerStage::enterWakeSequence() {
    writeSleep(true);
    _state = POWER_STAGE_WAKING;
    _stateDeadlineMs = hal.getMillis() + POWER_STAGE_WAKE_DELAY_MS;
}

void PowerStage::writePhaseEnables(bool active) {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_PHASE_ENABLES
    // The false path is used by the fault ISR and deliberately avoids reading settings.
    uint8_t mask = active ? configuredPhaseEnableMask() : 0;
    bool activeHigh = POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH != 0;
    gpio_put(PIN_POWER_STAGE_PHASE_ENABLE_A, ((mask & 0x01u) != 0) == activeHigh);
    gpio_put(PIN_POWER_STAGE_PHASE_ENABLE_B, ((mask & 0x02u) != 0) == activeHigh);
    gpio_put(PIN_POWER_STAGE_PHASE_ENABLE_C, ((mask & 0x04u) != 0) == activeHigh);
    _phaseEnableMask = mask;
#else
    (void)active;
#endif
}

uint8_t PowerStage::configuredPhaseEnableMask() const {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_PHASE_ENABLES
    uint8_t activeOutputs = settings.get().phaseMode;
    if (activeOutputs < PHASE_1) return 0;
    if (activeOutputs > PHASE_3) activeOutputs = PHASE_3;
    if (activeOutputs == PHASE_1) return 0x01u;
    if (activeOutputs == PHASE_2) return 0x03u;
    return 0x07u;
#else
    return 0;
#endif
}

void PowerStage::disableFromInterrupt() {
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
#if POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN
    if (!_enabled && !_enablePending) return; // Controller deliberately asserted the shared line.
#endif
    _faultOriginState = _state;
    _enabled = false;
    _enablePending = false;
    _faultLatched = true;
    _state = POWER_STAGE_FAULT_LATCHED;
    writeEnable(false);
    writePhaseEnables(false);
    writeSleep(false);
    writeReset(true);
    _faultReportPending = true;
    _metrics.faultCount++;
    if (_faultOriginState == POWER_STAGE_RUNNING || _faultOriginState == POWER_STAGE_STOPPING) _metrics.runningFaultCount++;
    else _metrics.wakeFaultCount++;
#endif
}

void PowerStage::captureFaultSnapshot() {
    _faultSnapshot.valid = true;
    _faultSnapshot.timestampMs = hal.getMillis();
    _faultSnapshot.originState = (uint8_t)_faultOriginState;
    _faultSnapshot.motorState = (uint8_t)motor.getState();
    _faultSnapshot.speed = (uint8_t)motor.getSpeed();
    _faultSnapshot.frequencyHz = motor.getCurrentFrequency();
    _faultSnapshot.bufferFillCount = waveform.getBufferFillCount();
    SpeedSettings& tune = settings.getCurrentSpeedSettings();
    for (int channel = 0; channel < 4; channel++) {
        _faultSnapshot.clippingCount[channel] = waveform.getClippingCount(channel);
        _faultSnapshot.phaseOffset[channel] = tune.phaseOffset[channel];
        _faultSnapshot.channelGain[channel] = tune.channelAmplitude[channel];
    }
}

void PowerStage::faultInterruptHandler() {
    if (powerStageInstance) powerStageInstance->disableFromInterrupt();
}
