/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <Arduino.h>

struct SystemMetricsSnapshot {
    float core0LoadPercent;
    float core1LoadPercent;

    uint32_t heapTotalBytes;
    uint32_t heapUsedBytes;
    uint32_t heapFreeBytes;
    uint32_t psramTotalBytes;
    uint32_t psramUsedBytes;
    uint32_t psramFreeBytes;

    uint32_t flashTotalBytes;
    uint32_t sketchUsedBytes;
    uint32_t sketchCapacityBytes;
    uint32_t filesystemUsedBytes;
    uint32_t filesystemTotalBytes;
    uint32_t filesystemCapacityBytes;
    bool filesystemMounted;
};

class SystemMonitor {
public:
    SystemMonitor();

    void begin();
    void beginCore0Loop();
    void endCore0Loop();
    void update();
    void recordCore1WorkMicros(uint32_t durationUs);

    SystemMetricsSnapshot snapshot() const;

private:
    uint32_t _core0LoopStartUs;
    uint32_t _windowStartUs;
    uint32_t _core0WindowBusyUs;
    volatile uint32_t _core1WindowBusyUs;
    SystemMetricsSnapshot _snapshot;

    void refreshMemoryAndFlash();
};

extern SystemMonitor systemMonitor;

#endif // SYSTEM_MONITOR_H
