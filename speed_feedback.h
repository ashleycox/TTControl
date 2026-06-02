/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef SPEED_FEEDBACK_H
#define SPEED_FEEDBACK_H

#include <Arduino.h>
#include "types.h"

enum SpeedFeedbackDirection : int8_t {
    SPEED_FEEDBACK_DIR_UNKNOWN = 0,
    SPEED_FEEDBACK_DIR_FORWARD = 1,
    SPEED_FEEDBACK_DIR_REVERSE = -1
};

struct SpeedFeedbackStatus {
    bool configured;
    bool signalValid;
    bool locked;
    bool pinAHigh;
    bool pinBHigh;
    float targetRpm;
    float measuredRpm;
    float filteredRpm;
    float rpmError;
    int8_t direction;
    int8_t rawDirection;
    int32_t count;
    int32_t countDelta;
    uint32_t invalidTransitions;
    uint32_t debouncedTransitions;
    uint32_t lastPulseAgeMs;
};

struct SpeedFeedbackSetupStatus {
    bool active;
    bool pinAHigh;
    bool pinBHigh;
    uint32_t elapsedMs;
    int32_t countDelta;
    uint32_t invalidDelta;
    uint32_t debouncedDelta;
    int8_t rawDirection;
    int8_t correctedDirection;
    uint16_t suggestedCountsPerRev;
    bool suggestedReverseDirection;
    uint8_t suggestedSensorMode;
    uint8_t suggestedQuadratureMode;
};

class SpeedFeedback {
public:
    SpeedFeedback();

    void begin();
    void configure();
    void reset();
    void update(float targetRpm);
    void beginSetupCapture();
    void cancelSetupCapture();

    SpeedFeedbackStatus getStatus();
    SpeedFeedbackSetupStatus getSetupStatus();

private:
    static SpeedFeedback* _instance;
    static void isrHandler();
    void handleInterrupt();

    void resetCounters();
    void resetMeasurements();
    bool acceptsPulseEdge(bool previousA, bool currentA) const;
    int8_t quadratureDelta(uint8_t previousState, uint8_t currentState) const;
    bool shouldCountQuadratureStep(uint8_t previousState, uint8_t currentState) const;

    volatile bool _configured;
    volatile uint8_t _sensorMode;
    volatile uint8_t _pulseEdge;
    volatile uint8_t _quadratureMode;
    volatile bool _reverseDirection;
    volatile uint16_t _debounceUs;
    volatile int32_t _count;
    volatile uint32_t _lastPulseUs;
    volatile uint32_t _lastAcceptedEdgeUs;
    volatile uint32_t _invalidTransitions;
    volatile uint32_t _debouncedTransitions;
    volatile bool _lastAState;
    volatile bool _lastBState;
    volatile uint8_t _lastQuadState;
    volatile int8_t _lastDirection;
    volatile int8_t _lastRawDirection;

    uint16_t _countsPerRev;
    uint16_t _timeoutMs;
    uint16_t _updateIntervalMs;
    uint16_t _lockTimeMs;
    float _filterAlpha;
    float _lockToleranceRpm;

    int32_t _lastSampleCount;
    uint32_t _lastSampleMs;
    uint32_t _lockCandidateStartMs;
    float _targetRpm;
    float _measuredRpm;
    float _filteredRpm;
    float _rpmError;
    int32_t _lastCountDelta;
    bool _signalValid;
    bool _locked;

    volatile bool _setupActive;
    int32_t _setupStartCount;
    uint32_t _setupStartInvalidTransitions;
    uint32_t _setupStartDebouncedTransitions;
    uint32_t _setupStartMs;
};

extern SpeedFeedback speedFeedback;

#endif // SPEED_FEEDBACK_H
