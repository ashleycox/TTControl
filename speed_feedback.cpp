/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "speed_feedback.h"
#include "settings.h"
#include "hal.h"
#include "globals.h"

SpeedFeedback speedFeedback;
SpeedFeedback* SpeedFeedback::_instance = nullptr;

SpeedFeedback::SpeedFeedback() {
    _instance = this;
    _configured = false;
    _sensorMode = CLOSED_LOOP_SENSOR_PULSE;
    _pulseEdge = CLOSED_LOOP_EDGE_RISING;
    _quadratureMode = CLOSED_LOOP_QUAD_X4;
    _reverseDirection = false;
    _debounceUs = 0;
    _count = 0;
    _lastPulseUs = 0;
    _lastAcceptedEdgeUs = 0;
    _invalidTransitions = 0;
    _lastAState = false;
    _lastBState = false;
    _lastQuadState = 0;
    _lastDirection = SPEED_FEEDBACK_DIR_UNKNOWN;

    _countsPerRev = 1;
    _timeoutMs = 3000;
    _updateIntervalMs = 100;
    _lockTimeMs = 1000;
    _filterAlpha = 0.25f;
    _lockToleranceRpm = 0.05f;

    _lastSampleCount = 0;
    _lastSampleMs = 0;
    _lockCandidateStartMs = 0;
    _targetRpm = 0.0f;
    _measuredRpm = 0.0f;
    _filteredRpm = 0.0f;
    _rpmError = 0.0f;
    _lastCountDelta = 0;
    _signalValid = false;
    _locked = false;
}

void SpeedFeedback::begin() {
#if CLOSED_LOOP_SPEED_ENABLE
    hal.setPinMode(PIN_SPEED_SENSOR_A, INPUT_PULLUP);
    hal.setPinMode(PIN_SPEED_SENSOR_B, INPUT_PULLUP);
    configure();
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_SENSOR_A), SpeedFeedback::isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_SENSOR_B), SpeedFeedback::isrHandler, CHANGE);
#else
    _configured = false;
#endif
}

void SpeedFeedback::configure() {
#if CLOSED_LOOP_SPEED_ENABLE
    bool a = hal.digitalRead(PIN_SPEED_SENSOR_A) == HIGH;
    bool b = hal.digitalRead(PIN_SPEED_SENSOR_B) == HIGH;
    GlobalSettings& g = settings.get();

    noInterrupts();
    _configured = g.closedLoopEnabled;
    _sensorMode = g.closedLoopSensorMode;
    _pulseEdge = g.closedLoopPulseEdge;
    _quadratureMode = g.closedLoopQuadratureMode;
    _reverseDirection = g.closedLoopReverseDirection;
    _debounceUs = g.closedLoopDebounceUs;
    _lastAState = a;
    _lastBState = b;
    _lastQuadState = ((uint8_t)a << 1) | (uint8_t)b;
    interrupts();

    _countsPerRev = g.closedLoopCountsPerRev;
    _timeoutMs = g.closedLoopTimeoutMs;
    _updateIntervalMs = g.closedLoopUpdateIntervalMs;
    _lockTimeMs = g.closedLoopLockTimeMs;
    _filterAlpha = g.closedLoopFilterAlpha;
    _lockToleranceRpm = g.closedLoopLockToleranceRpm;
    reset();
#else
    _configured = false;
    reset();
#endif
}

void SpeedFeedback::reset() {
    resetCounters();
    resetMeasurements();
}

void SpeedFeedback::resetCounters() {
    noInterrupts();
    _count = 0;
    _lastPulseUs = 0;
    _lastAcceptedEdgeUs = 0;
    _invalidTransitions = 0;
    _lastDirection = SPEED_FEEDBACK_DIR_UNKNOWN;
    interrupts();
}

void SpeedFeedback::resetMeasurements() {
    _lastSampleCount = 0;
    _lastSampleMs = hal.getMillis();
    _lockCandidateStartMs = 0;
    _targetRpm = 0.0f;
    _measuredRpm = 0.0f;
    _filteredRpm = 0.0f;
    _rpmError = 0.0f;
    _lastCountDelta = 0;
    _signalValid = false;
    _locked = false;
}

void SpeedFeedback::update(float targetRpm) {
    _targetRpm = targetRpm;

    int32_t count;
    uint32_t lastPulseUs;
    noInterrupts();
    count = _count;
    lastPulseUs = _lastPulseUs;
    interrupts();

    uint32_t nowMs = hal.getMillis();
    uint32_t nowUs = hal.getMicros();
    uint32_t pulseAgeMs = lastPulseUs == 0 ? UINT32_MAX : (nowUs - lastPulseUs) / 1000UL;
    bool signalValid = _configured && lastPulseUs != 0 && pulseAgeMs <= _timeoutMs;

    if (_lastSampleMs == 0) {
        _lastSampleMs = nowMs;
        _lastSampleCount = count;
        _signalValid = signalValid;
        return;
    }

    uint32_t elapsedMs = nowMs - _lastSampleMs;
    if (elapsedMs < _updateIntervalMs) {
        _signalValid = signalValid;
        _rpmError = _targetRpm - _filteredRpm;
        return;
    }

    int32_t delta = count - _lastSampleCount;
    _lastCountDelta = delta;
    _lastSampleCount = count;
    _lastSampleMs = nowMs;
    _signalValid = signalValid;

    if (!signalValid || _countsPerRev == 0 || elapsedMs == 0) {
        _measuredRpm = 0.0f;
        _filteredRpm = 0.0f;
        _rpmError = _targetRpm;
        _locked = false;
        _lockCandidateStartMs = 0;
        return;
    }

    float revolutions = (float)abs(delta) / (float)_countsPerRev;
    _measuredRpm = revolutions * (60000.0f / (float)elapsedMs);
    if (_filteredRpm <= 0.0f) {
        _filteredRpm = _measuredRpm;
    } else {
        _filteredRpm += (_measuredRpm - _filteredRpm) * _filterAlpha;
    }
    _rpmError = _targetRpm - _filteredRpm;

    if (abs(_rpmError) <= _lockToleranceRpm) {
        if (_lockCandidateStartMs == 0) {
            _lockCandidateStartMs = nowMs;
        }
        _locked = (nowMs - _lockCandidateStartMs) >= _lockTimeMs;
    } else {
        _locked = false;
        _lockCandidateStartMs = 0;
    }
}

SpeedFeedbackStatus SpeedFeedback::getStatus() {
    SpeedFeedbackStatus status;
    uint32_t nowUs = hal.getMicros();

    noInterrupts();
    status.configured = _configured;
    status.count = _count;
    status.invalidTransitions = _invalidTransitions;
    status.direction = _lastDirection;
    uint32_t lastPulseUs = _lastPulseUs;
    interrupts();

    status.signalValid = _signalValid;
    status.locked = _locked;
    status.targetRpm = _targetRpm;
    status.measuredRpm = _measuredRpm;
    status.filteredRpm = _filteredRpm;
    status.rpmError = _rpmError;
    status.countDelta = _lastCountDelta;
    status.lastPulseAgeMs = lastPulseUs == 0 ? UINT32_MAX : (nowUs - lastPulseUs) / 1000UL;
    return status;
}

void SpeedFeedback::isrHandler() {
    if (_instance) _instance->handleInterrupt();
}

void SpeedFeedback::handleInterrupt() {
#if CLOSED_LOOP_SPEED_ENABLE
    if (!_configured) return;

    uint32_t nowUs = micros();
    if (_debounceUs > 0 && (nowUs - _lastAcceptedEdgeUs) < _debounceUs) {
        return;
    }

    bool a = digitalRead(PIN_SPEED_SENSOR_A) == HIGH;
    bool b = digitalRead(PIN_SPEED_SENSOR_B) == HIGH;

    if (_sensorMode == CLOSED_LOOP_SENSOR_PULSE) {
        bool previousA = _lastAState;
        _lastAState = a;
        if (!acceptsPulseEdge(previousA, a)) return;

        _lastAcceptedEdgeUs = nowUs;
        _lastPulseUs = nowUs;
        _lastDirection = SPEED_FEEDBACK_DIR_FORWARD;
        _count++;
        return;
    }

    uint8_t previousState = _lastQuadState;
    uint8_t currentState = ((uint8_t)a << 1) | (uint8_t)b;
    if (currentState == previousState) return;

    int8_t delta = quadratureDelta(previousState, currentState);
    if (delta == 0) {
        _invalidTransitions++;
        _lastQuadState = currentState;
        _lastAState = a;
        _lastBState = b;
        return;
    }

    if (!shouldCountQuadratureStep(previousState, currentState)) {
        _lastQuadState = currentState;
        _lastAState = a;
        _lastBState = b;
        return;
    }

    if (_reverseDirection) delta = -delta;
    _count += delta;
    _lastDirection = delta >= 0 ? SPEED_FEEDBACK_DIR_FORWARD : SPEED_FEEDBACK_DIR_REVERSE;
    _lastPulseUs = nowUs;
    _lastAcceptedEdgeUs = nowUs;
    _lastQuadState = currentState;
    _lastAState = a;
    _lastBState = b;
#endif
}

bool SpeedFeedback::acceptsPulseEdge(bool previousA, bool currentA) const {
    if (previousA == currentA) return false;
    if (_pulseEdge == CLOSED_LOOP_EDGE_CHANGE) return true;
    if (_pulseEdge == CLOSED_LOOP_EDGE_RISING) return !previousA && currentA;
    return previousA && !currentA;
}

int8_t SpeedFeedback::quadratureDelta(uint8_t previousState, uint8_t currentState) const {
    static const int8_t lookup[16] = {
        0, 1, -1, 0,
        -1, 0, 0, 1,
        1, 0, 0, -1,
        0, -1, 1, 0
    };
    return lookup[((previousState & 0x03) << 2) | (currentState & 0x03)];
}

bool SpeedFeedback::shouldCountQuadratureStep(uint8_t previousState, uint8_t currentState) const {
    if (_quadratureMode == CLOSED_LOOP_QUAD_X4) return true;

    bool previousA = previousState & 0x02;
    bool currentA = currentState & 0x02;
    if (previousA == currentA) return false;

    if (_quadratureMode == CLOSED_LOOP_QUAD_X2) return true;
    return !previousA && currentA;
}
