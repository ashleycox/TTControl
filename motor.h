/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "globals.h"

struct SpeedFeedbackStatus;

enum ClosedLoopTuneStep : uint8_t {
    CLOSED_LOOP_TUNE_IDLE = 0,
    CLOSED_LOOP_TUNE_SENSOR,
    CLOSED_LOOP_TUNE_MONITOR,
    CLOSED_LOOP_TUNE_KP,
    CLOSED_LOOP_TUNE_KI,
    CLOSED_LOOP_TUNE_LIMITS,
    CLOSED_LOOP_TUNE_VERIFY
};

struct ClosedLoopMetrics {
    uint32_t sampleCount;
    uint32_t validSamples;
    uint32_t lockedSamples;
    uint32_t validMs;
    uint32_t lockedMs;
    uint32_t saturatedMs;
    uint32_t dropoutEvents;
    uint32_t saturationEvents;
    uint32_t directionFaultEvents;
    uint32_t plausibilityEvents;
    uint32_t lockTimeoutEvents;
    uint32_t ampRecoveryEvents;
    uint32_t errorSignChanges;
    float averageErrorRpm;
    float averageAbsErrorRpm;
    float peakAbsErrorRpm;
    float lastErrorRpm;
};

struct ClosedLoopTuningStatus {
    bool active;
    uint8_t step;
    const char* stepName;
    char instruction[96];
    char recommendation[120];
    ClosedLoopMetrics metrics;
};

/**
 * @brief Manages the high-level state of the motor.
 * 
 * Handles state transitions (Standby, Stopped, Starting, Running, Stopping),
 * speed selection, pitch control, and relay management.
 * It coordinates with the WaveformGenerator to produce the correct output.
 */
class MotorController {
public:
    MotorController();
    
    // Initialize hardware pins and load settings
    void begin();
    
    // Main update loop (call frequently)
    void update();
    
    // --- State Control ---
    void start();
    void stop();
    void emergencyStop();
    void toggleStartStop();
    void toggleStandby();
    
    // --- Speed Control ---
    void setSpeed(SpeedMode speed);
    void cycleSpeed();
    void adjustSpeed(int delta);
    
    // --- Pitch Control ---
    void setPitch(float percent);
    void adjustPitchFreq(float deltaHz);
    void resetPitch();
    void togglePitchRange();
    int getPitchRange() { return _pitchRange; }
    
    // --- Accessors ---
    bool isRunning() { return _state == STATE_RUNNING || _state == STATE_STARTING; }
    bool isStandby() { return _state == STATE_STANDBY; }
    bool isSweepingMode() { return _isSweepingMode; }
    bool isSpeedRamping() { return _isSpeedRamping; }
    bool isRelayTestMode() { return _relayTestMode; }
    bool isClosedLoopActive() { return _closedLoopActive; }
    bool isClosedLoopAmpRecoveryActive() { return _closedLoopAmpRecoveryActive; }
    bool isClosedLoopSaturated() { return _closedLoopSaturationStart != 0; }
    MotorState getState() { return _state; }
    SpeedMode getSpeed() { return _currentSpeedMode; }
    float getCurrentFrequency() { return _currentFreq; }
    float getPitchPercent() { return currentPitchPercent; }
    float getClosedLoopTargetRpm() { return _closedLoopTargetRpm; }
    float getClosedLoopRequestedTargetRpm() { return _closedLoopRequestedTargetRpm; }
    float getClosedLoopRampTargetRpm() { return _closedLoopRampTargetRpm; }
    float getClosedLoopCorrectionHz() { return _closedLoopCorrectionHz; }
    ClosedLoopMetrics getClosedLoopMetrics() { return _closedLoopMetrics; }
    ClosedLoopTuningStatus getClosedLoopTuningStatus();
    float getMotionProgress();
    void resetClosedLoop();
    void beginClosedLoopTuning();
    void advanceClosedLoopTuning();
    void cancelClosedLoopTuning();
    
    // --- Relay Control ---
    void setRelays(bool active);
    bool beginRelayTest();
    void setRelayTestStage(uint8_t stage);
    void endRelayTest();
    uint8_t getRelayTestStage() { return _relayTestStage; }
    uint8_t getRelayTestStageCount();
    
    // Apply current settings to waveform generator
    void applySettings();

    // --- Diagnostic Modes ---
    void startSymmetricSweep(float minSep, float maxSep, float speed);
    void stopSymmetricSweep();

private:
    MotorState _state;
    SpeedMode _currentSpeedMode;
    
    float _currentFreq;
    float _targetFreq;
    float _currentAmp;
    float _targetAmp;
    
    int _pitchRange; // Range in percent (e.g., 10%)
    
    uint32_t _stateStartTime;
    uint32_t _lastUpdate;
    
    // Soft Start/Stop state
    float _startDuration;
    
    // Startup Kick
    bool _isKicking;
    uint32_t _kickEndTime;
    
    // Amplitude Reduction
    uint32_t _ampReductionStartTime;
    bool _isReducedAmp;
    
    // Braking
    uint32_t _brakePulseLastToggle;
    bool _brakePulseState;
    
    // Relay Control
    bool _relaysActive;
    bool _relayActivationPending;
    uint32_t _relayStageTime;
    int _relayStage; // 0=All Off, 1=A On, 2=B On...
    bool _relayTestMode;
    uint8_t _relayTestStage;
    
    // Power On Delay
    bool _powerOnDelayActive;
    uint32_t _powerOnTime;
    
    // Ramping State
    bool _isSpeedRamping;
    float _rampStartFreq;
    float _rampTargetFreq;
    uint32_t _rampStartTime;
    float _rampDuration;
    
    // Kick Ramp
    bool _isKickRamping;
    float _kickRampStartFreq;
    uint32_t _kickRampStartTime;
    float _kickRampDuration;

    // Closed-loop speed correction
    bool _closedLoopActive;
    float _closedLoopTargetRpm;
    float _closedLoopRequestedTargetRpm;
    float _closedLoopRampTargetRpm;
    float _closedLoopCorrectionHz;
    float _closedLoopIntegralHz;
    float _closedLoopLastErrorRpm;
    uint32_t _closedLoopLastUpdate;
    uint32_t _closedLoopTargetLastUpdate;
    uint32_t _closedLoopEngageTime;
    bool _closedLoopDirectionFaultLatched;
    bool _closedLoopDropoutLatched;
    uint32_t _closedLoopSaturationStart;
    bool _closedLoopSaturationLatched;
    uint32_t _closedLoopLockWaitStart;
    bool _closedLoopLockTimeoutLatched;
    bool _closedLoopPlausibilityLatched;
    uint32_t _closedLoopAmpOutOfLockStart;
    bool _closedLoopAmpRecoveryActive;
    bool _closedLoopAmpRecoveryLatched;
    float _rampStartRpm;
    float _rampTargetRpm;
    ClosedLoopMetrics _closedLoopMetrics;
    float _closedLoopErrorSumRpm;
    float _closedLoopAbsErrorSumRpm;
    uint32_t _closedLoopMetricsLastSampleSequence;
    uint32_t _closedLoopMetricsLastSampleMs;
    int8_t _closedLoopLastErrorSign;
    bool _closedLoopMetricsLastSignalValid;
    bool _closedLoopMetricsWasSaturated;
    uint8_t _closedLoopTuneStep;
    
    void updateState();
    float calculateSoftStartAmp(float elapsed, float duration);
    void handleBraking(uint32_t now);
    float calculateClosedLoopTargetRpm() const;
    float calculateClosedLoopTargetRpmForSpeed(SpeedMode speed) const;
    float updateClosedLoopTarget(uint32_t now, float requestedRpm);
    float applyClosedLoopRampCorrection(uint32_t now, float openLoopFreq, float rampTargetRpm);
    bool closedLoopControlAllowed() const;
    bool closedLoopSafetyAllowsCorrection(uint32_t now, const SpeedFeedbackStatus& feedback);
    void resetClosedLoopPidState();
    void reportClosedLoopAction(const char* message, uint8_t action, bool& latch);
    void reportClosedLoopAction(const char* message, uint8_t action, bool& latch, const SpeedFeedbackStatus* feedback);
    void resetClosedLoopMetrics();
    void recordClosedLoopMetrics(uint32_t now, const SpeedFeedbackStatus& feedback);
    const char* closedLoopTuneStepName(uint8_t step) const;
    const char* closedLoopTuneInstruction(uint8_t step) const;
    void buildClosedLoopRecommendation(char* out, size_t outSize);
    void updateClosedLoopAmpRecovery(uint32_t now, const SpeedFeedbackStatus& feedback);
    float applyClosedLoopCorrection(uint32_t now, float openLoopFreq);
    void scheduleClosedLoopEngage(uint32_t now);
    void resetClosedLoopControl(bool resetFeedback);
    float clampToCurrentSpeedRange(float freq) const;
    void setStandbyRelay(bool active);
    void writeRelayOutput(int pin, bool active);
    void clearMotionState();
    void forceDriveOutputsOff();
    
    // Diagnostic Sweep State
    bool _isSweepingMode;
    bool _wasRunningBeforeSweep;
    float _sweepMinSeparation;
    float _sweepMaxSeparation;
    float _sweepSpeed;
    
    // Deferred Settings Save
    bool _settingsDirty;
    uint32_t _lastSettingsChange;
};

#endif // MOTOR_H
