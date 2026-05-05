/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "amp_monitor.h"
#include "error_handler.h"
#include "hal.h"
#include "motor.h"

extern MotorController motor;

AmplifierMonitor ampMonitor;

AmplifierMonitor::AmplifierMonitor() {
    _lastSampleMs = 0;
    _temperatureC = 0.0;
    _thermalOk = true;
    _warned = false;
    _shutdown = false;
}

void AmplifierMonitor::begin() {
#if AMP_MONITOR_ENABLE
    hal.setPinMode(PIN_AMP_THERM_OK, INPUT_PULLDOWN);
    hal.setPinMode(PIN_AMP_TEMP, INPUT);
#endif
}

void AmplifierMonitor::update() {
#if AMP_MONITOR_ENABLE
    uint32_t now = hal.getMillis();
    if (now - _lastSampleMs < 500) return;
    _lastSampleMs = now;

    if (_shutdown) {
        motor.emergencyStop();
        return;
    }

    _thermalOk = hal.digitalRead(PIN_AMP_THERM_OK) == HIGH;
    _temperatureC = readTmp36C();

    if (!_thermalOk) {
        shutdownOutputs("Amp thermal cutout");
        return;
    }

    if (_temperatureC >= AMP_TEMP_SHUTDOWN_C) {
        shutdownOutputs("Amp over temperature");
        return;
    }

    if (!_warned && _temperatureC >= AMP_TEMP_WARN_C) {
        _warned = true;
        errorHandler.report(ERR_AMP_THERMAL, "Amp temperature high", false);
    }

    if (_temperatureC < (AMP_TEMP_WARN_C - 5.0f)) {
        _warned = false;
    }
#endif
}

float AmplifierMonitor::readTmp36C() {
    int raw = analogRead(PIN_AMP_TEMP);
    float voltage = (raw * 3.3f) / 1023.0f;
    return (voltage - 0.5f) * 100.0f;
}

void AmplifierMonitor::shutdownOutputs(const char* message) {
    if (_shutdown) return;
    _shutdown = true;

    motor.emergencyStop();
    errorHandler.report(ERR_AMP_THERMAL, message, true);
}
