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

class AmplifierMonitor {
public:
    AmplifierMonitor();

    void begin();
    void update();

    float getTemperatureC() const { return _temperatureC; }
    bool isThermalOk() const { return _thermalOk; }
    bool isShutdown() const { return _shutdown; }

private:
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
