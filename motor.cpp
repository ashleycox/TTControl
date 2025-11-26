/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "motor.h"
#include "settings.h"
#include "waveform.h"
#include "hal.h"

MotorController::MotorController() {
    _state = STATE_STANDBY;
    
    // Check if we should auto-boot into a specific state
    if (settings.get().autoBoot) {
        _state = STATE_STOPPED;
        // Check if we should automatically start the motor
        if (settings.get().autoStart) {
            start();
        }
    }
    
    // Initialize default values
    _currentSpeedMode = SPEED_33;
    _currentFreq = 50.0;
    _targetFreq = 50.0;
    _currentAmp = 0.0;
    _targetAmp = 0.0;
    _pitchRange = 10;
    _isSpeedRamping = false;
    _isKickRamping = false;
    _powerOnDelayActive = true;
    _isKickRamping = false;
    _powerOnDelayActive = true;
    _powerOnTime = hal.getMillis();
    
    _settingsDirty = false;
    _lastSettingsChange = 0;
}

void MotorController::begin() {
    // Configure hardware pins via HAL
    hal.setPinMode(PIN_RELAY_STANDBY, OUTPUT);
    hal.setPinMode(PIN_MUTE_PHASE_A, OUTPUT);
    hal.setPinMode(PIN_MUTE_PHASE_B, OUTPUT);
    hal.setPinMode(PIN_MUTE_PHASE_C, OUTPUT);
    hal.setPinMode(PIN_MUTE_PHASE_D, OUTPUT);
    
    // Initialize relays to OFF state
    _relaysActive = false;
    _relayStage = 0;
    setRelays(false);
    
    // Load initial speed settings
    if (settings.get().bootSpeed <= 2) {
        _currentSpeedMode = (SpeedMode)settings.get().bootSpeed;
    } else {
        _currentSpeedMode = settings.get().currentSpeed;
    }
    applySettings();
    
    // Handle auto-start if configured
    if (settings.get().autoStart) {
        start();
    }
}

void MotorController::update() {
    uint32_t now = hal.getMillis();
    
    // --- Main State Machine ---
    switch (_state) {
        case STATE_STANDBY:
            // System is in low-power/standby mode. Waiting for user input.
            break;
            
        case STATE_STOPPED:
            // Motor is powered but not rotating. Waiting for start command.
            break;
            
        case STATE_STARTING:
            // Motor is accelerating. Handles Startup Kick and Soft Start.
            
            // 1. Startup Kick Logic (High torque start)
            if (_isKicking) {
                if (now >= _kickEndTime) {
                    _isKicking = false;
                    
                    // Transition from Kick frequency to Target frequency
                    SpeedSettings& s = settings.getCurrentSpeedSettings();
                    if (s.startupKickRampDuration > 0) {
                        // Ramp down frequency smoothly
                        _kickRampDuration = s.startupKickRampDuration * 1000.0; 
                        _kickRampStartTime = now;
                        _kickRampStartFreq = waveform.getFrequency();
                        _isKickRamping = true;
                    } else {
                        // Jump immediately to target
                        waveform.setFrequency(_targetFreq);
                    }
                }
            }
            
            // 2. Kick Ramp Logic
            if (_isKickRamping) {
                float elapsed = now - _kickRampStartTime;
                if (elapsed >= _kickRampDuration) {
                    _isKickRamping = false;
                    waveform.setFrequency(_targetFreq);
                } else {
                    float t = elapsed / _kickRampDuration;
                    float currentF = _kickRampStartFreq - ((_kickRampStartFreq - _targetFreq) * t);
                    waveform.setFrequency(currentF);
                }
            } else if (!_isKicking) {
                // Ensure we are exactly at target frequency if not kicking/ramping
                if (waveform.getFrequency() != _targetFreq) waveform.setFrequency(_targetFreq);
            }
            
            // 3. Amplitude Soft Start Logic
            {
                float duration = settings.getCurrentSpeedSettings().softStartDuration * 1000.0;
                float elapsed = now - _stateStartTime;
                
                if (elapsed >= duration) {
                    // Soft start complete, transition to RUNNING
                    _state = STATE_RUNNING;
                    _currentAmp = _targetAmp;
                    _ampReductionStartTime = now; // Start timer for amplitude reduction
                } else {
                    _currentAmp = calculateSoftStartAmp(elapsed, duration);
                }
                
                // Apply Frequency Dependent Amplitude (FDA) Scaling
                // Linearly interpolate between FDA% (at 0Hz) and Target Amp (at Target Freq)
                if (settings.get().freqDependentAmplitude > 0) {
                    float fdaRatio = (float)settings.get().freqDependentAmplitude / 100.0;
                    float fdaAmp = _targetAmp * fdaRatio;
                    
                    // Current ratio of frequency to target frequency
                    float freqRatio = 0.0;
                    if (_targetFreq > 0.1) {
                         freqRatio = waveform.getFrequency() / _targetFreq;
                         if (freqRatio > 1.0) freqRatio = 1.0;
                         if (freqRatio < 0.0) freqRatio = 0.0;
                    }
                    
                    // Scale amplitude: Start at FDA, ramp to 100% of calculated amp
                    // Actually, FDA usually implies V/f constant or similar.
                    // Here we want: Amp = FDA_Amp + (Target_Amp - FDA_Amp) * (Freq / TargetFreq)
                    // But we must also respect the Soft Start ramp which is currently dictating _currentAmp.
                    // The user requirement says: "scales the output amplitude with the frequency during all modes of operation"
                    // So we should probably apply this scaling to the FINAL output amplitude, based on current frequency.
                    
                    // Let's recalculate based on strict V/f from FDA baseline
                    float baseAmp = fdaAmp + ((_targetAmp - fdaAmp) * freqRatio);
                    
                    // However, we also have Soft Start and Reduced Amplitude modes.
                    // Soft Start is a time-based ramp of amplitude.
                    // FDA is a frequency-based ramp of amplitude.
                    // If we are Soft Starting, we are ramping Amp from 0 to Target.
                    // If we are Frequency Ramping (Startup Kick), we are ramping Freq.
                    
                    // If we simply apply FDA scaling to the _targetAmp, then Soft Start might conflict.
                    // Let's apply FDA as a ceiling or modifier?
                    // "allow the current and hence torque to be equalised between speeds, and during start-up when a frequency ramp is used"
                    
                    // Interpretation: The amplitude should be a function of frequency.
                    // Amp(f) = FDA_Amp + (Max_Amp - FDA_Amp) * (f / f_max)
                    // But "Max_Amp" here is the target amplitude for the current speed.
                    
                    // So, if we are at 50% frequency, we should be at roughly 50% amplitude (if FDA=0).
                    // If FDA=10 (10% boost at 0Hz), then at 50% freq we are at 10 + (90 * 0.5) = 55%.
                    
                    // We should apply this scaling to the _currentAmp derived from other states?
                    // No, FDA usually overrides other amplitude logic when frequency is the dominant variable.
                    // But Soft Start is time-based.
                    
                    // If we are in STATE_STARTING with a Kick Ramp (Frequency Ramp), FDA is crucial.
                    // If we are in STATE_STARTING with Soft Start (Fixed Freq, Amp Ramp), FDA is constant (since Freq is constant).
                    
                    // Let's apply FDA scaling as a factor on the *intended* amplitude, or just replace it?
                    // "scales the output amplitude with the frequency"
                    
                    // Let's define the "Frequency Scale Factor":
                    // Factor = FDA_Percent + (1.0 - FDA_Percent) * (CurrentFreq / TargetFreq)
                    
                    float scaleFactor = fdaRatio + ((1.0 - fdaRatio) * freqRatio);
                    
                    // Apply this factor to the current amplitude state
                    _currentAmp = _currentAmp * scaleFactor;
                }

                waveform.setAmplitude(_currentAmp);
            }
            break;
            
        case STATE_RUNNING:
            // Motor is running at target speed. Handles Pitch and Reduced Amplitude.
            
            // 1. Pitch Control / Frequency Update
            {
                float baseFreq = settings.getCurrentSpeedSettings().frequency;
                float pitchMod = baseFreq * (currentPitchPercent / 100.0);
                _targetFreq = baseFreq + pitchMod;
                
                if (_currentFreq != _targetFreq) {
                    _currentFreq = _targetFreq;
                    waveform.setFrequency(_currentFreq);
                    currentFrequency = _currentFreq; // Update global for UI
                }
                
                // 2. Reduced Amplitude (Power Saving / Noise Reduction)
                if (!_isReducedAmp) {
                    uint32_t delaySec = settings.getCurrentSpeedSettings().amplitudeDelay; 
                    uint32_t delayMs = delaySec * 1000;
                    
                    if (now - _ampReductionStartTime >= delayMs) {
                        _isReducedAmp = true;
                        float reducePercent = (float)settings.getCurrentSpeedSettings().reducedAmplitude / 100.0;
                        _currentAmp = _targetAmp * reducePercent;
                        waveform.setAmplitude(_currentAmp);
                    }
                }
                
                // 3. Speed Switching Ramp (Smooth transition between speeds)
                if (_isSpeedRamping) {
                    float elapsed = now - _rampStartTime;
                    if (elapsed >= _rampDuration) {
                        _isSpeedRamping = false;
                        _currentFreq = _rampTargetFreq;
                        waveform.setFrequency(_currentFreq);
                    } else {
                        float t = elapsed / _rampDuration;
                        float currentF = _rampStartFreq + ((_rampTargetFreq - _rampStartFreq) * t);
                        waveform.setFrequency(currentF);
                        _currentFreq = currentF;
                    }
                }
                
                // 4. Update Runtime Counter
                settings.updateRuntime();
            }
            break;
            
        case STATE_STOPPING:
            // Motor is decelerating. Handles Braking logic.
            handleBraking(now);
            break;
    }
    
    // Update global state for UI access
    currentMotorState = _state;
    
    // --- Relay Staggering Logic ---
    // Prevents current spikes by turning on relays sequentially
    if (_relaysActive && _relayStage < 4) {
        if (now - _relayStageTime > 100) { // 100ms stagger delay
            _relayStageTime = now;
            _relayStage++;
            bool activeHigh = settings.get().relayActiveHigh;
            int pin = -1;
            if (_relayStage == 1) pin = PIN_MUTE_PHASE_A;
            else if (_relayStage == 2) pin = PIN_MUTE_PHASE_B;
            else if (_relayStage == 3) pin = PIN_MUTE_PHASE_C;
            else if (_relayStage == 4) pin = PIN_MUTE_PHASE_D;
            
            if (pin != -1) hal.digitalWrite(pin, activeHigh ? HIGH : LOW);
        }
    }
    
    // --- Deferred Settings Save ---
    if (_settingsDirty && (now - _lastSettingsChange > 2000)) {
        settings.save();
        _settingsDirty = false;
    }
}

void MotorController::start() {
    if (_state == STATE_RUNNING || _state == STATE_STARTING) return;
    
    _state = STATE_STARTING;
    _stateStartTime = hal.getMillis();
    
    applySettings();
    _targetAmp = (float)settings.get().maxAmplitude / 100.0;
    _currentAmp = 0.0;
    _isReducedAmp = false;
    
    // Initialize Startup Kick if configured
    SpeedSettings& s = settings.getCurrentSpeedSettings();
    if (s.startupKick > 1) {
        _isKicking = true;
        _kickEndTime = hal.getMillis() + (s.startupKickDuration * 1000);
        waveform.setFrequency(_targetFreq * s.startupKick);
    } else {
        _isKicking = false;
        waveform.setFrequency(_targetFreq);
    }
    
    // Unmute relays if linked to start/stop
    if (settings.get().muteRelayLinkStartStop) {
        setRelays(true); 
    }
    
    waveform.setEnabled(true);
    waveform.setAmplitude(0.0);
}

void MotorController::stop() {
    if (_state == STATE_STOPPED || _state == STATE_STANDBY) return;
    
    _state = STATE_STOPPING;
    _stateStartTime = hal.getMillis();
    
    // Configure Braking Mode
    if (settings.get().brakeMode == BRAKE_PULSE) {
        _brakePulseState = true; 
        _brakePulseLastToggle = hal.getMillis();
        // Reverse frequency for braking torque
        waveform.setFrequency(-_targetFreq);
        waveform.setAmplitude(_targetAmp); 
    } else if (settings.get().brakeMode == BRAKE_RAMP) {
        waveform.setFrequency(settings.get().brakeStartFreq);
    }
    
    if (settings.get().pitchResetOnStop) {
        resetPitch();
    }
}

void MotorController::handleBraking(uint32_t now) {
    float duration = settings.get().brakeDuration * 1000.0;
    float elapsed = now - _stateStartTime;
    
    // Check if braking is complete
    if (elapsed >= duration) {
        _state = STATE_STOPPED;
        _currentAmp = 0.0;
        waveform.setEnabled(false);
        
        if (settings.get().muteRelayLinkStartStop) {
            setRelays(false); // Mute
        }
        
        // Reset frequency to positive
        waveform.setFrequency(abs(_targetFreq));
        return;
    }
    
    // Handle specific braking modes
    if (settings.get().brakeMode == BRAKE_RAMP) {
        // Linearly ramp frequency down
        float startF = settings.get().brakeStartFreq;
        float stopF = settings.get().brakeStopFreq;
        float currentF = startF - ((startF - stopF) * (elapsed / duration));
        waveform.setFrequency(currentF);
        
        // Ramp amplitude down
        _currentAmp = _targetAmp * (1.0 - (elapsed / duration));
        waveform.setAmplitude(_currentAmp);
    }
    else if (settings.get().brakeMode == BRAKE_PULSE) {
        // Pulse the reverse torque on/off
        float gap = settings.get().brakePulseGap * 1000.0;
        if (now - _brakePulseLastToggle >= gap) {
            _brakePulseLastToggle = now;
            _brakePulseState = !_brakePulseState;
            if (_brakePulseState) {
                waveform.setAmplitude(_targetAmp);
            } else {
                waveform.setAmplitude(0.0);
            }
        }
    }
    else {
        // Default: Simple amplitude ramp down
        _currentAmp = _targetAmp * (1.0 - (elapsed / duration));
        waveform.setAmplitude(_currentAmp);
    }
}

float MotorController::calculateSoftStartAmp(float elapsed, float duration) {
    float t = elapsed / duration;
    if (t > 1.0) t = 1.0;
    
    if (settings.get().rampType == RAMP_SCURVE) {
        // Sine S-Curve: 0.5 * (1 - cos(PI * t))
        // Maps 0.0-1.0 to 0.0-1.0 with ease-in/ease-out
        return _targetAmp * (0.5 * (1.0 - cos(PI * t)));
    } else {
        // Linear
        return _targetAmp * t;
    }
    return _targetAmp * t; // Should be unreachable but satisfies compiler
}

void MotorController::toggleStartStop() {
    if (isRunning()) stop();
    else start();
}

void MotorController::toggleStandby() {
    if (_state == STATE_STANDBY) {
        // Waking up
        _state = STATE_STOPPED;
        
        // If linked to standby, unmute. 
        // BUT if also linked to Start/Stop, we should stay muted until Start.
        if (settings.get().muteRelayLinkStandby && !settings.get().muteRelayLinkStartStop) {
             setRelays(true); 
        } else {
             setRelays(false);
        }
    } else {
        // Going to sleep
        stop();
        _state = STATE_STANDBY;
        
        // If linked to standby, mute.
        if (settings.get().muteRelayLinkStandby) {
            setRelays(false);
        }
        
        // Reset Session Runtime
        settings.resetSessionRuntime();
        
        // Save Total Runtime (Silent)
        settings.save(false);
    }
    currentMotorState = _state;
}

void MotorController::cycleSpeed() {
    int s = (int)_currentSpeedMode + 1;
    if (s > SPEED_78) s = SPEED_33;
    
    // Skip 78 RPM if disabled in settings
    if (s == SPEED_78 && !settings.get().enable78rpm) {
        s = SPEED_33;
    }
    
    setSpeed((SpeedMode)s);
}

void MotorController::adjustSpeed(int delta) {
    int s = (int)_currentSpeedMode + delta;
    
    // Clamp to valid range
    if (s < SPEED_33) s = SPEED_33;
    if (s > SPEED_78) s = SPEED_78;
    
    // Check 78 RPM limit
    if (s == SPEED_78 && !settings.get().enable78rpm) {
        s = SPEED_45;
    }
    
    setSpeed((SpeedMode)s);
}

void MotorController::setSpeed(SpeedMode mode) {
    if (_currentSpeedMode == mode) return;
    
    _currentSpeedMode = mode;
    applySettings();
    
    // Calculate new target frequency including pitch
    float baseFreq = settings.getCurrentSpeedSettings().frequency;
    float pitchMod = baseFreq * (currentPitchPercent / 100.0);
    float newTarget = baseFreq + pitchMod;
    
    if (_state == STATE_RUNNING) {
        if (settings.get().smoothSwitching) {
            // Initiate smooth frequency ramp
            _isSpeedRamping = true;
            _rampStartFreq = waveform.getFrequency();
            _rampTargetFreq = newTarget;
            _rampStartTime = hal.getMillis();
            _rampDuration = settings.get().switchRampDuration * 1000.0;
        } else {
            // Instant switch
            _targetFreq = newTarget;
            _currentFreq = _targetFreq;
            waveform.setFrequency(_currentFreq);
        }
    } else {
        _targetFreq = newTarget;
    }
    
    // Persist new speed selection
    settings.get().currentSpeed = mode;
    // Defer save to avoid blocking
    _settingsDirty = true;
    _lastSettingsChange = hal.getMillis();
}

void MotorController::setPitch(float percent) {
    currentPitchPercent = percent;
}

void MotorController::resetPitch() {
    currentPitchPercent = 0.0;
}

void MotorController::togglePitchRange() {
    _pitchRange += 10;
    if (_pitchRange > 50) _pitchRange = 10;
}

void MotorController::adjustPitchFreq(float deltaHz) {
    // Calculate current pitch in Hz
    float baseFreq = settings.getCurrentSpeedSettings().frequency;
    float currentPitchHz = baseFreq * (currentPitchPercent / 100.0);
    float newPitchHz = currentPitchHz + deltaHz;
    
    // Limit pitch to configured range
    float maxPitchHz = baseFreq * (_pitchRange / 100.0);
    if (newPitchHz > maxPitchHz) newPitchHz = maxPitchHz;
    if (newPitchHz < -maxPitchHz) newPitchHz = -maxPitchHz;
    
    // Convert back to percentage
    currentPitchPercent = (newPitchHz / baseFreq) * 100.0;
}

void MotorController::applySettings() {
    SpeedSettings& s = settings.getCurrentSpeedSettings();
    
    _targetFreq = s.frequency;
    _currentFreq = _targetFreq;
    currentFrequency = _currentFreq;
    
    waveform.updateSettings(_currentFreq, s);
}

void MotorController::setRelays(bool active) {
    bool activeHigh = settings.get().relayActiveHigh;
    
    // Safety: Enforce Power On Delay
    if (_powerOnDelayActive) {
        uint32_t delayMs = settings.get().powerOnRelayDelay * 1000;
        if (hal.getMillis() - _powerOnTime < delayMs) { 
            active = false; // Force mute
        } else {
            _powerOnDelayActive = false;
        }
    }
    
    if (active) {
        // Start Staggered Unmute Sequence
        _relaysActive = true;
        _relayStage = 0;
        _relayStageTime = hal.getMillis();
        // Pins are updated in update() loop
    } else {
        // Immediate Mute (All Off)
        _relaysActive = false;
        _relayStage = 0;
        hal.digitalWrite(PIN_MUTE_PHASE_A, activeHigh ? LOW : HIGH);
        hal.digitalWrite(PIN_MUTE_PHASE_B, activeHigh ? LOW : HIGH);
        hal.digitalWrite(PIN_MUTE_PHASE_C, activeHigh ? LOW : HIGH);
        hal.digitalWrite(PIN_MUTE_PHASE_D, activeHigh ? LOW : HIGH);
    }
    
    // Handle Standby Relay Linking
    if (settings.get().muteRelayLinkStandby) {
        if (active) hal.digitalWrite(PIN_RELAY_STANDBY, activeHigh ? HIGH : LOW);
    }
}
