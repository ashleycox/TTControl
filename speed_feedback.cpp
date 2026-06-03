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
    _debouncedTransitions = 0;
    _acceptedTransitions = 0;
    _intervalSamples = 0;
    _intervalJitterSamples = 0;
    _lastIntervalUs = 0;
    _minIntervalUs = 0;
    _maxIntervalUs = 0;
    _previousIntervalUs = 0;
    _intervalSumUs = 0;
    _intervalJitterSumUs = 0;
    _lastAState = false;
    _lastBState = false;
    _lastQuadState = 0;
    _lastDirection = SPEED_FEEDBACK_DIR_UNKNOWN;
    _lastRawDirection = SPEED_FEEDBACK_DIR_UNKNOWN;

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
    _sampleSequence = 0;
    _signalValid = false;
    _locked = false;
    _setupActive = false;
    _setupStartCount = 0;
    _setupStartInvalidTransitions = 0;
    _setupStartDebouncedTransitions = 0;
    _setupStartMs = 0;
}

void SpeedFeedback::begin() {
#if CLOSED_LOOP_SPEED_ENABLE
    // Both pins are attached for quadrature. Pulse mode still uses pin A but pin B is sampled for diagnostics and setup visibility.
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
    // Snapshot pin state before enabling counting so the first interrupt has a valid previous state.
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
    _filterAlpha = g.closedLoopFilterAlpha;
    ClosedLoopSpeedTuning& t = settings.getCurrentClosedLoopTuning();
    _lockTimeMs = t.lockTimeMs;
    _lockToleranceRpm = t.lockToleranceRpm;
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
    _debouncedTransitions = 0;
    _acceptedTransitions = 0;
    _intervalSamples = 0;
    _intervalJitterSamples = 0;
    _lastIntervalUs = 0;
    _minIntervalUs = 0;
    _maxIntervalUs = 0;
    _previousIntervalUs = 0;
    _intervalSumUs = 0;
    _intervalJitterSumUs = 0;
    _lastDirection = SPEED_FEEDBACK_DIR_UNKNOWN;
    _lastRawDirection = SPEED_FEEDBACK_DIR_UNKNOWN;
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
    _sampleSequence = 0;
    _signalValid = false;
    _locked = false;
}

void SpeedFeedback::beginSetupCapture() {
    // Start with a clean counter window so the suggested counts/rev is the count delta for the user's manual revolution.
    reset();
    noInterrupts();
    _setupStartCount = _count;
    _setupStartInvalidTransitions = _invalidTransitions;
    _setupStartDebouncedTransitions = _debouncedTransitions;
    interrupts();
    _setupStartMs = hal.getMillis();
    _setupActive = true;
}

void SpeedFeedback::cancelSetupCapture() {
    _setupActive = false;
}

void SpeedFeedback::update(float targetRpm) {
    // This method runs on Core 0. It samples ISR counters at the configured interval and turns count delta over elapsed time into RPM.
    _targetRpm = targetRpm;
    ClosedLoopSpeedTuning& tuning = settings.getCurrentClosedLoopTuning();
    _lockTimeMs = tuning.lockTimeMs;
    _lockToleranceRpm = tuning.lockToleranceRpm;

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
    _sampleSequence++;
    _signalValid = signalValid;

    if (!signalValid || _countsPerRev == 0 || elapsedMs == 0) {
        // Drop stale speed estimates immediately when the signal is lost so closed-loop code cannot keep correcting from old RPM data.
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
        // Exponential smoothing is intentionally simple and bounded by settings.
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
    uint32_t acceptedTransitions;
    uint32_t intervalSamples;
    uint32_t intervalJitterSamples;
    uint32_t lastIntervalUs;
    uint32_t minIntervalUs;
    uint32_t maxIntervalUs;
    uint64_t intervalSumUs;
    uint64_t intervalJitterSumUs;

    // Copy ISR-owned fields in one critical section, then derive percentages and ages after interrupts are restored.
    noInterrupts();
    status.configured = _configured;
    status.count = _count;
    status.invalidTransitions = _invalidTransitions;
    status.debouncedTransitions = _debouncedTransitions;
    acceptedTransitions = _acceptedTransitions;
    intervalSamples = _intervalSamples;
    intervalJitterSamples = _intervalJitterSamples;
    lastIntervalUs = _lastIntervalUs;
    minIntervalUs = _minIntervalUs;
    maxIntervalUs = _maxIntervalUs;
    intervalSumUs = _intervalSumUs;
    intervalJitterSumUs = _intervalJitterSumUs;
    status.direction = _lastDirection;
    status.rawDirection = _lastRawDirection;
    status.pinAHigh = _lastAState;
    status.pinBHigh = _lastBState;
    uint32_t lastPulseUs = _lastPulseUs;
    interrupts();

    status.signalValid = _signalValid;
    status.locked = _locked;
    status.targetRpm = _targetRpm;
    status.measuredRpm = _measuredRpm;
    status.filteredRpm = _filteredRpm;
    status.rpmError = _rpmError;
    status.countDelta = _lastCountDelta;
    status.acceptedTransitions = acceptedTransitions;
    status.totalTransitions = acceptedTransitions + status.invalidTransitions + status.debouncedTransitions;
    status.intervalSamples = intervalSamples;
    status.lastIntervalUs = lastIntervalUs;
    status.minIntervalUs = minIntervalUs;
    status.maxIntervalUs = maxIntervalUs;
    status.averageIntervalUs = intervalSamples > 0 ? (uint32_t)(intervalSumUs / intervalSamples) : 0;
    status.averageJitterUs = intervalJitterSamples > 0 ? (uint32_t)(intervalJitterSumUs / intervalJitterSamples) : 0;
    status.averageJitterPercent = status.averageIntervalUs > 0 ?
        ((float)status.averageJitterUs * 100.0f) / (float)status.averageIntervalUs : 0.0f;
    status.invalidTransitionPercent = status.totalTransitions > 0 ?
        ((float)status.invalidTransitions * 100.0f) / (float)status.totalTransitions : 0.0f;
    status.debouncedTransitionPercent = status.totalTransitions > 0 ?
        ((float)status.debouncedTransitions * 100.0f) / (float)status.totalTransitions : 0.0f;
    status.lastPulseAgeMs = lastPulseUs == 0 ? UINT32_MAX : (nowUs - lastPulseUs) / 1000UL;
    status.sampleTimeMs = _lastSampleMs;
    status.sampleSequence = _sampleSequence;
    return status;
}

SpeedFeedbackSetupStatus SpeedFeedback::getSetupStatus() {
    SpeedFeedbackSetupStatus status;
    int32_t count;
    uint32_t invalidTransitions;
    uint32_t debouncedTransitions;

    noInterrupts();
    count = _count;
    invalidTransitions = _invalidTransitions;
    debouncedTransitions = _debouncedTransitions;
    status.pinAHigh = _lastAState;
    status.pinBHigh = _lastBState;
    status.rawDirection = _lastRawDirection;
    status.correctedDirection = _lastDirection;
    status.suggestedSensorMode = _sensorMode;
    status.suggestedQuadratureMode = _quadratureMode;
    interrupts();

    status.active = _setupActive;
    status.elapsedMs = _setupActive ? hal.getMillis() - _setupStartMs : 0;
    status.countDelta = count - _setupStartCount;
    status.invalidDelta = invalidTransitions - _setupStartInvalidTransitions;
    status.debouncedDelta = debouncedTransitions - _setupStartDebouncedTransitions;
    int32_t absDelta = abs(status.countDelta);
    /*
     * A single manual revolution should be small enough to fit uint16_t and
     * large enough to infer counts/rev. Larger values are treated as invalid
     * capture noise rather than applying a bad setting.
     */
    status.suggestedCountsPerRev = absDelta > 0 && absDelta <= 20000 ? (uint16_t)absDelta : 0;
    status.suggestedReverseDirection = status.rawDirection == SPEED_FEEDBACK_DIR_REVERSE;
    return status;
}

void SpeedFeedback::isrHandler() {
    if (_instance) _instance->handleInterrupt();
}

void SpeedFeedback::handleInterrupt() {
#if CLOSED_LOOP_SPEED_ENABLE
    if (!_configured && !_setupActive) return;

    // Debounce is based on accepted edges, not all pin changes, so rejected chatter does not keep extending the debounce window forever.
    uint32_t nowUs = micros();
    if (_debounceUs > 0 && (nowUs - _lastAcceptedEdgeUs) < _debounceUs) {
        _debouncedTransitions++;
        return;
    }

    bool a = digitalRead(PIN_SPEED_SENSOR_A) == HIGH;
    bool b = digitalRead(PIN_SPEED_SENSOR_B) == HIGH;

    if (_sensorMode == CLOSED_LOOP_SENSOR_PULSE) {
        bool previousA = _lastAState;
        _lastAState = a;
        _lastBState = b;
        if (!acceptsPulseEdge(previousA, a)) return;

        _lastRawDirection = SPEED_FEEDBACK_DIR_FORWARD;
        _lastDirection = SPEED_FEEDBACK_DIR_FORWARD;
        recordAcceptedTransition(nowUs);
        _count++;
        return;
    }

    uint8_t previousState = _lastQuadState;
    uint8_t currentState = ((uint8_t)a << 1) | (uint8_t)b;
    if (currentState == previousState) return;

    int8_t delta = quadratureDelta(previousState, currentState);
    if (delta == 0) {
        // A two-bit jump means the encoder skipped an intermediate state or the input is noisy. Track it for diagnostics and resynchronize.
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

    _lastRawDirection = delta >= 0 ? SPEED_FEEDBACK_DIR_FORWARD : SPEED_FEEDBACK_DIR_REVERSE;
    int8_t correctedDelta = _reverseDirection ? -delta : delta;
    _count += correctedDelta;
    _lastDirection = correctedDelta >= 0 ? SPEED_FEEDBACK_DIR_FORWARD : SPEED_FEEDBACK_DIR_REVERSE;
    recordAcceptedTransition(nowUs);
    _lastQuadState = currentState;
    _lastAState = a;
    _lastBState = b;
#endif
}

void SpeedFeedback::recordAcceptedTransition(uint32_t nowUs) {
#if CLOSED_LOOP_SPEED_ENABLE
    if (_lastPulseUs != 0) {
        uint32_t intervalUs = nowUs - _lastPulseUs;
        _lastIntervalUs = intervalUs;
        if (_minIntervalUs == 0 || intervalUs < _minIntervalUs) _minIntervalUs = intervalUs;
        if (intervalUs > _maxIntervalUs) _maxIntervalUs = intervalUs;
        _intervalSumUs += intervalUs;
        _intervalSamples++;

        if (_previousIntervalUs != 0) {
            uint32_t jitterUs = intervalUs > _previousIntervalUs ?
                intervalUs - _previousIntervalUs : _previousIntervalUs - intervalUs;
            _intervalJitterSumUs += jitterUs;
            _intervalJitterSamples++;
        }
        _previousIntervalUs = intervalUs;
    }

    _acceptedTransitions++;
    _lastPulseUs = nowUs;
    _lastAcceptedEdgeUs = nowUs;
#else
    (void)nowUs;
#endif
}

bool SpeedFeedback::acceptsPulseEdge(bool previousA, bool currentA) const {
    // Pulse tachometer mode can count rising, falling, or both edges depending on the sensor and magnet/slot geometry.
    if (previousA == currentA) return false;
    if (_pulseEdge == CLOSED_LOOP_EDGE_CHANGE) return true;
    if (_pulseEdge == CLOSED_LOOP_EDGE_RISING) return !previousA && currentA;
    return previousA && !currentA;
}

int8_t SpeedFeedback::quadratureDelta(uint8_t previousState, uint8_t currentState) const {
    // Standard Gray-code transition table. Zero means no movement or an invalid two-bit transition.
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

    // X2 and X1 are down-sampled from the same ISR stream by only counting A transitions or A rising transitions.
    bool previousA = previousState & 0x02;
    bool currentA = currentState & 0x02;
    if (previousA == currentA) return false;

    if (_quadratureMode == CLOSED_LOOP_QUAD_X2) return true;
    return !previousA && currentA;
}
