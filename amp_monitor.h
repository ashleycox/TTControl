/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef AMP_MONITOR_H
#define AMP_MONITOR_H

#include <Arduino.h>
#include "config.h"

// Polls the amplifier protection inputs from Core 0. It intentionally reports
// thermal faults through MotorController rather than touching waveform hardware
// directly, so output shutdown follows the same emergency path as other faults.
class AmplifierMonitor {
public:
    AmplifierMonitor();

    void begin();
    void update();

    float getTemperatureC() const { return _temperatureC; }
    bool isThermalOk() const { return _thermalOk; }
    bool isShutdown() const { return _shutdown; }

private:
    // Sampling is throttled because temperature and thermal OK do not require
    // per-loop reads, and analog reads are slow enough to matter in Core 0.
    uint32_t _lastSampleMs;
    float _temperatureC;
    bool _thermalOk;
    bool _warned;
    bool _shutdown;

    float readTmp36C();
    void shutdownOutputs(const char* message);
};

extern AmplifierMonitor ampMonitor;

#endif // AMP_MONITOR_H
