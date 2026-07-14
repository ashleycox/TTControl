/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#include <Arduino.h>
#include "config.h"

enum PowerStageState : uint8_t {
    POWER_STAGE_DISABLED = 0,
    POWER_STAGE_RESET_ASSERTED,
    POWER_STAGE_WAKING,
    POWER_STAGE_WAITING_NEUTRAL,
    POWER_STAGE_PHASE_ENABLING,
    POWER_STAGE_READY,
    POWER_STAGE_RUNNING,
    POWER_STAGE_STOPPING,
    POWER_STAGE_FAULT_LATCHED
};

struct PowerStageMetrics {
    uint32_t enableAttempts;
    uint32_t successfulEnables;
    uint32_t faultCount;
    uint32_t wakeFaultCount;
    uint32_t runningFaultCount;
};

struct PowerStageFaultSnapshot {
    bool valid;
    uint32_t timestampMs;
    uint8_t originState;
    uint8_t motorState;
    uint8_t speed;
    float frequencyHz;
    uint32_t bufferFillCount;
    uint32_t clippingCount[4];
    float phaseOffset[4];
    uint8_t channelGain[4];
};

/**
 * Owns the electrical contract between the DDS PWM outputs and the selected
 * power stage. Bridge builds keep the driver disabled until neutral PWM has
 * crossed the DMA boundary; fault ISR work is restricted to immediate GPIO
 * shutdown and an atomic notification for Core 0.
 */
class PowerStage {
public:
    PowerStage();

    void begin();
    void update();
    bool requestEnable();
    void notifyRunning();
    void notifyStopping();
    void refreshPhaseEnables();
    void disable();

    bool isEnabled() const;
    bool isEnablePending() const;
    bool hasFault() const;
    bool faultInputActive() const;
    const char* backendName() const;
    PowerStageState state() const;
    const char* stateName() const;
    PowerStageMetrics metrics() const;
    PowerStageFaultSnapshot faultSnapshot() const;

    static void faultInterruptHandler();

private:
    volatile bool _enabled;
    volatile bool _enablePending;
    volatile bool _faultLatched;
    volatile bool _faultReportPending;
    uint32_t _enableRequestMs;
    uint32_t _enableRequestBufferCount;
    volatile PowerStageState _state;
    volatile PowerStageState _faultOriginState;
    volatile uint8_t _phaseEnableMask;
    uint32_t _stateDeadlineMs;
    PowerStageMetrics _metrics;
    PowerStageFaultSnapshot _faultSnapshot;

    void writeEnable(bool active);
    void writePhaseEnables(bool active);
    uint8_t configuredPhaseEnableMask() const;
    void writeSleep(bool active);
    void writeReset(bool asserted);
    void enterWakeSequence();
    void captureFaultSnapshot();
    void disableFromInterrupt();
};

extern PowerStage powerStage;

#endif // POWER_STAGE_H
