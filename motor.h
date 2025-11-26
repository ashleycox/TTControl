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
    SpeedMode getSpeed() { return _currentSpeedMode; }
    float getCurrentFrequency() { return _currentFreq; }
    float getPitchPercent() { return currentPitchPercent; }
    
    // --- Relay Control ---
    void setRelays(bool active);
    
    // Apply current settings to waveform generator
    void applySettings();

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
    uint32_t _relayStageTime;
    int _relayStage; // 0=All Off, 1=A On, 2=B On...
    
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
    
    void updateState();
    float calculateSoftStartAmp(float elapsed, float duration);
    void handleBraking(uint32_t now);
    
    // Deferred Settings Save
    bool _settingsDirty;
    uint32_t _lastSettingsChange;
};

#endif // MOTOR_H
