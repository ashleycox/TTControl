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
#include "speed_feedback.h"
#include "error_handler.h"

static float clampOutputFrequency(float freq) {
    if (freq > MAX_OUTPUT_FREQUENCY_HZ) return MAX_OUTPUT_FREQUENCY_HZ;
    if (freq < -MAX_OUTPUT_FREQUENCY_HZ) return -MAX_OUTPUT_FREQUENCY_HZ;
    return freq;
}

static float clampToSpeedSettings(float freq, const SpeedSettings& s) {
    if (freq > s.maxFrequency) return s.maxFrequency;
    if (freq < s.minFrequency) return s.minFrequency;
    return clampOutputFrequency(freq);
}

MotorController::MotorController() {
    _state = ENABLE_STANDBY ? STATE_STANDBY : STATE_STOPPED;
    _currentSpeedMode = SPEED_33;
    _currentFreq = 50.0;
    _targetFreq = 50.0;
    _currentAmp = 0.0;
    _targetAmp = 0.0;
    _pitchRange = 10;
    _stateStartTime = 0;
    _lastUpdate = 0;
    _startDuration = 0.0;
    _isKicking = false;
    _kickEndTime = 0;
    _ampReductionStartTime = 0;
    _isReducedAmp = false;
    _brakePulseLastToggle = 0;
    _brakePulseState = false;
    _relaysActive = false;
    _relayActivationPending = false;
    _relayStageTime = 0;
    _relayStage = 0;
    _relayTestMode = false;
    _relayTestStage = 0;
    _isSpeedRamping = false;
    _rampStartFreq = 0.0;
    _rampTargetFreq = 0.0;
    _rampStartTime = 0;
    _rampDuration = 0.0;
    _isKickRamping = false;
    _kickRampStartFreq = 0.0;
    _kickRampStartTime = 0;
    _kickRampDuration = 0.0;
    _closedLoopActive = false;
    _closedLoopTargetRpm = 0.0;
    _closedLoopRequestedTargetRpm = 0.0;
    _closedLoopRampTargetRpm = 0.0;
    _closedLoopCorrectionHz = 0.0;
    _closedLoopIntegralHz = 0.0;
    _closedLoopLastErrorRpm = 0.0;
    _closedLoopLastUpdate = 0;
    _closedLoopTargetLastUpdate = 0;
    _closedLoopEngageTime = 0;
    _closedLoopDirectionFaultLatched = false;
    _closedLoopDropoutLatched = false;
    _closedLoopSaturationStart = 0;
    _closedLoopSaturationLatched = false;
    _closedLoopLockWaitStart = 0;
    _closedLoopLockTimeoutLatched = false;
    _closedLoopPlausibilityLatched = false;
    _closedLoopAmpOutOfLockStart = 0;
    _closedLoopAmpRecoveryActive = false;
    _closedLoopAmpRecoveryLatched = false;
    _rampStartRpm = 0.0;
    _rampTargetRpm = 0.0;
    _powerOnDelayActive = true;
    _powerOnTime = 0;
    _isSweepingMode = false;
    _wasRunningBeforeSweep = false;
    _sweepMinSeparation = 0.0;
    _sweepMaxSeparation = 0.0;
    _sweepSpeed = 0.0;
    _settingsDirty = false;
    _lastSettingsChange = 0;
}

void MotorController::begin() {
    // Configure hardware pins via HAL
    hal.setPinMode(PIN_RELAY_STANDBY, OUTPUT);

    if (ENABLE_MUTE_RELAYS) {
        if (ENABLE_DPDT_RELAYS) {
            hal.setPinMode(PIN_RELAY_DPDT_1, OUTPUT);
            hal.setPinMode(PIN_RELAY_DPDT_2, OUTPUT);
        } else {
            hal.setPinMode(PIN_MUTE_PHASE_A, OUTPUT);
            hal.setPinMode(PIN_MUTE_PHASE_B, OUTPUT);
            hal.setPinMode(PIN_MUTE_PHASE_C, OUTPUT);
#if ENABLE_4_CHANNEL_SUPPORT
            hal.setPinMode(PIN_MUTE_PHASE_D, OUTPUT);
#endif
        }
    }

    // Initialize relays to OFF state
    _relaysActive = false;
    _relayStage = 0;
    _powerOnTime = hal.getMillis();
    setRelays(false);
    speedFeedback.begin();

    _state = (ENABLE_STANDBY && !settings.get().autoBoot) ? STATE_STANDBY : STATE_STOPPED;

    // Load initial speed settings
    if (settings.get().bootSpeed <= 2) {
        _currentSpeedMode = (SpeedMode)settings.get().bootSpeed;
    } else {
        _currentSpeedMode = settings.get().currentSpeed;
    }
    if (_currentSpeedMode == SPEED_78 && !settings.get().enable78rpm) {
        _currentSpeedMode = SPEED_33;
    }
    settings.get().currentSpeed = _currentSpeedMode;
    applySettings();
    setStandbyRelay(_state != STATE_STANDBY);
    currentMotorState = _state;

    // Handle auto-start only when boot is allowed to bypass standby.
    if (_state == STATE_STOPPED && settings.get().autoStart) {
        start();
    }
}

void MotorController::startSymmetricSweep(float minSep, float maxSep, float speed) {
    if (_relayTestMode) return;
    if (settings.get().phaseMode == PHASE_4) return; // Invalid for 4-phase twin motors

    _wasRunningBeforeSweep = isRunning();
    if (!isRunning()) {
        start();
    }

    _isSweepingMode = true;
    _sweepMinSeparation = minSep;
    _sweepMaxSeparation = maxSep;
    _sweepSpeed = speed;
}

void MotorController::stopSymmetricSweep() {
    _isSweepingMode = false;

    if (!_wasRunningBeforeSweep) {
        stop();
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
                    speedFeedback.reset();
                    scheduleClosedLoopEngage(now);
                } else {
                    _currentAmp = calculateSoftStartAmp(elapsed, duration);
                }

                // Apply Frequency Dependent Amplitude (FDA) Scaling
                // Linearly interpolate between FDA% (at 0Hz) and Target Amp (at Target Freq)
                if (settings.get().freqDependentAmplitude > 0) {
                    float fdaRatio = (float)settings.get().freqDependentAmplitude / 100.0;
                    // 3-Point V/f Curve Interpolation
                    float currentF = waveform.getFrequency();

                    // User coordinates
                    float fLow = settings.get().vfLowFreq;
                    float vLow = (float)settings.get().vfLowBoost / 100.0;
                    float fMid = settings.get().vfMidFreq;
                    float vMid = (float)settings.get().vfMidBoost / 100.0;
                    float fHigh = _targetFreq;
                    float vHigh = 1.0; // Target freq implies 100% of calculated soft-start target

                    float scaleFactor = 1.0;

                    // Prevent divide-by-zero or malformed curves
                    if (fLow >= fMid) fMid = fLow + 0.1;
                    if (fMid >= fHigh) fHigh = fMid + 0.1;

                    if (currentF <= fLow) {
                        // Point 1 - Flat line up to fLow (or linear ramp from 0 to vLow)
                        // Most motors need instant boost at 0Hz to break friction
                        scaleFactor = vLow;
                    } else if (currentF > fLow && currentF <= fMid) {
                        // Segment 1: Low to Mid
                        float segmentProgress = (currentF - fLow) / (fMid - fLow);
                        scaleFactor = vLow + ((vMid - vLow) * segmentProgress);
                    } else {
                        // Segment 2: Mid to High
                        float segmentProgress = (currentF - fMid) / (fHigh - fMid);
                        if (segmentProgress > 1.0) segmentProgress = 1.0;
                        scaleFactor = vMid + ((vHigh - vMid) * segmentProgress);
                    }

                    // The FDA master percentage can act as an overall multiplier/mix for the curve
                    // If FDA = 100%, we use the full calculated curve. If FDA = 50%, we blend it halfway towards 1.0.
                    float blendFDA = fdaRatio * scaleFactor + (1.0 - fdaRatio);

                    // Apply this factor to the current amplitude state
                    _currentAmp = _currentAmp * blendFDA;
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
                _targetFreq = clampToCurrentSpeedRange(baseFreq + pitchMod);

#if CLOSED_LOOP_SPEED_ENABLE
                float requestedTargetRpm = calculateClosedLoopTargetRpm();
#endif

                // 2. Speed Switching Ramp (Smooth transition between speeds)
                if (_isSpeedRamping) {
                    float elapsed = now - _rampStartTime;
                    if (elapsed >= _rampDuration) {
                        _isSpeedRamping = false;
                        _closedLoopRampTargetRpm = 0.0f;
                        _currentFreq = _rampTargetFreq;
                        waveform.setFrequency(_currentFreq);
                        currentFrequency = _currentFreq;
                        speedFeedback.reset();
                        scheduleClosedLoopEngage(now);
                    } else {
                        float t = _rampDuration > 0.0f ? elapsed / _rampDuration : 1.0f;
                        float openLoopFreq = _rampStartFreq + ((_rampTargetFreq - _rampStartFreq) * t);
                        float commandedFreq = openLoopFreq;
#if CLOSED_LOOP_SPEED_ENABLE
                        float rampTargetRpm = _rampStartRpm + ((_rampTargetRpm - _rampStartRpm) * t);
                        _closedLoopRampTargetRpm = rampTargetRpm;
                        _closedLoopTargetRpm = updateClosedLoopTarget(now, rampTargetRpm);
                        speedFeedback.update(_closedLoopTargetRpm);
                        if (!_isSweepingMode && settings.get().closedLoopRampMode == CLOSED_LOOP_RAMP_TRACK) {
                            commandedFreq = applyClosedLoopRampCorrection(now, openLoopFreq, _closedLoopTargetRpm);
                            if (_state != STATE_RUNNING) break;
                        } else {
                            _closedLoopActive = false;
                            _closedLoopCorrectionHz = 0.0f;
                            resetClosedLoopPidState();
                        }
#endif
                        _currentFreq = commandedFreq;
                        waveform.setFrequency(_currentFreq);
                        currentFrequency = _currentFreq;
                    }
                } else {
                    float commandedFreq = _targetFreq;
#if CLOSED_LOOP_SPEED_ENABLE
                    _closedLoopRampTargetRpm = 0.0f;
                    _closedLoopTargetRpm = updateClosedLoopTarget(now, requestedTargetRpm);
                    speedFeedback.update(_closedLoopTargetRpm);
                    if (!_isSweepingMode) {
                        commandedFreq = applyClosedLoopCorrection(now, _targetFreq);
                        if (_state != STATE_RUNNING) break;
                    } else {
                        _closedLoopActive = false;
                        resetClosedLoopPidState();
                    }
#endif
                    if (_currentFreq != commandedFreq) {
                        _currentFreq = commandedFreq;
                        waveform.setFrequency(_currentFreq);
                        currentFrequency = _currentFreq; // Update global for UI
                    }
                }

                // 3. Reduced Amplitude (Power Saving / Noise Reduction)
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

#if CLOSED_LOOP_SPEED_ENABLE
                updateClosedLoopAmpRecovery(now, speedFeedback.getStatus());
#endif

                // 4. Update Runtime Counter
                settings.updateRuntime();

                // 5. Diagnostic Resonance Sweep
                if (_isSweepingMode) {
                    float timeSec = now / 1000.0;
                    float range = _sweepMaxSeparation - _sweepMinSeparation;
                    if (range > 0 && _sweepSpeed > 0) {
                        float period = (range * 2.0) / _sweepSpeed;
                        float modTime = fmod(timeSec, period);
                        float currentSep = 0;

                        if (modTime < period / 2.0) {
                            // Rising
                            currentSep = _sweepMinSeparation + (modTime * _sweepSpeed);
                        } else {
                            // Falling
                            currentSep = _sweepMaxSeparation - ((modTime - period / 2.0) * _sweepSpeed);
                        }

                        SpeedSettings& s = settings.getCurrentSpeedSettings();
                        if (settings.get().phaseMode == 2) {
                            s.phaseOffset[1] = currentSep;
                        } else if (settings.get().phaseMode == 3) {
                            s.phaseOffset[1] = currentSep;
                            s.phaseOffset[2] = currentSep * 2.0;
                        }

                        waveform.updateSettings(_targetFreq, s);
                    }
                }
            }
            break;

        case STATE_STOPPING:
            // Motor is decelerating. Handles Braking logic.
            handleBraking(now);
            break;
    }

    // Update global state for UI access
    currentMotorState = _state;

    if (!_relayTestMode && _relayActivationPending) {
        uint32_t delayMs = settings.get().powerOnRelayDelay * 1000;
        if (now - _powerOnTime >= delayMs) {
            _powerOnDelayActive = false;
            _relayActivationPending = false;
            setRelays(true);
        }
    }

    // --- Relay Staggering Logic ---
    // Prevents current spikes by turning on relays sequentially
    if (!_relayTestMode && ENABLE_MUTE_RELAYS && _relaysActive) {
        bool activeHigh = settings.get().relayActiveHigh;

        if (ENABLE_DPDT_RELAYS) {
            // DPDT Logic: 2 stages
            if (_relayStage < 2) {
                if (now - _relayStageTime > 100) {
                    _relayStageTime = now;
                    _relayStage++;

                    int pin = -1;
                    int phaseMode = settings.get().phaseMode;

                    if (_relayStage == 1) {
                        // DPDT 1: Always used (Phase A/B or 1/2)
                        pin = PIN_RELAY_DPDT_1;
                    } else if (_relayStage == 2) {
                        // DPDT 2: Only used for three or more phase modes
                        if (phaseMode >= 3) {
                            pin = PIN_RELAY_DPDT_2;
                        }
                    }

                    if (pin != -1) hal.digitalWrite(pin, activeHigh ? HIGH : LOW);
                }
            }
        } else {
            // SPST Logic: one stage per enabled output phase
            if (_relayStage < MAX_ACTIVE_PHASE_OUTPUTS) {
                if (now - _relayStageTime > 100) { // 100ms stagger delay
                    _relayStageTime = now;
                    _relayStage++;

                    int pin = -1;
                    int phaseMode = settings.get().phaseMode;

                    // Only switch relays required for current phase mode
                    if (_relayStage == 1) pin = PIN_MUTE_PHASE_A;
                    else if (_relayStage == 2 && phaseMode >= 2) pin = PIN_MUTE_PHASE_B;
                    else if (_relayStage == 3 && phaseMode >= 3) pin = PIN_MUTE_PHASE_C;
#if ENABLE_4_CHANNEL_SUPPORT
                    else if (_relayStage == 4 && phaseMode >= 4) pin = PIN_MUTE_PHASE_D;
#endif

                    if (pin != -1) hal.digitalWrite(pin, activeHigh ? HIGH : LOW);
                }
            }
        }
    }

    // --- Deferred Settings Save ---
    if (_settingsDirty && (now - _lastSettingsChange > 2000)) {
        settings.save();
        _settingsDirty = false;
    }
}

void MotorController::start() {
    if (_relayTestMode) return;
    if (_state == STATE_RUNNING || _state == STATE_STARTING) return;

    if (_state == STATE_STANDBY) {
        _state = STATE_STOPPED;
        setStandbyRelay(true);

        if (settings.get().muteRelayLinkStandby && !settings.get().muteRelayLinkStartStop) {
            setRelays(true);
        } else {
            setRelays(false);
        }
    }

    _state = STATE_STARTING;
    _stateStartTime = hal.getMillis();
    settings.syncRuntimeClock();
    resetClosedLoopControl(true);

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
    if (_relayTestMode) return;
    if (_state == STATE_STOPPED || _state == STATE_STANDBY) return;

    _state = STATE_STOPPING;
    _stateStartTime = hal.getMillis();
    resetClosedLoopControl(false);

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
    else if (settings.get().brakeMode == BRAKE_SOFT_STOP) {
        // Active Coasting: Gently bring frequency down to the configured cutoff point while maintaining driving torque
        float startF = abs(_targetFreq);
        float stopF = settings.get().softStopCutoff;

        // If we're already below the cutoff, or duration is 0, just stop instantly like BRAKE_OFF
        if (startF <= stopF || duration <= 0) {
            _currentAmp = 0.0;
            waveform.setAmplitude(0.0);
        } else {
            // Ramp frequency down
            float currentF = startF - ((startF - stopF) * (elapsed / duration));
            waveform.setFrequency(currentF);
            // Maintain full intended amplitude throughout the coast to ensure load tracks frequency
            waveform.setAmplitude(_targetAmp);
        }
    }
    else {
        // Default (BRAKE_OFF): Simple amplitude ramp down (or instant cut if duration is 0)
        if (duration <= 0) {
            _currentAmp = 0.0;
        } else {
            _currentAmp = _targetAmp * (1.0 - (elapsed / duration));
        }
        waveform.setAmplitude(_currentAmp);
    }
}

float MotorController::calculateSoftStartAmp(float elapsed, float duration) {
    float t = elapsed / duration;
    if (t > 1.0) t = 1.0;

    if (settings.get().rampType == RAMP_SCURVE) {
        // Sine S-Curve: 0.5 * (1 - cos(PI * t))
        return _targetAmp * (0.5 * (1.0 - cos(PI * t)));
    } else {
        uint8_t curve = settings.get().softStartCurve;
        if (curve == 1) {
            // Logarithmic (Base 10 mapping 0-1 to 0-1)
            return _targetAmp * log10(1.0 + 9.0 * t);
        } else if (curve == 2) {
            // Exponential (Base 10 mapping 0-1 to 0-1)
            return _targetAmp * (pow(10.0, t) - 1.0) / 9.0;
        } else {
            // Linear
            return _targetAmp * t;
        }
    }
}

void MotorController::toggleStartStop() {
    if (isRunning()) stop();
    else start();
}

void MotorController::toggleStandby() {
    if (_relayTestMode) return;
    if (!ENABLE_STANDBY) return;

    if (_state == STATE_STANDBY) {
        // Waking up
        _state = STATE_STOPPED;
        setStandbyRelay(true);

        // If linked to standby, unmute.
        // BUT if also linked to Start/Stop, we should stay muted until Start.
        if (settings.get().muteRelayLinkStandby && !settings.get().muteRelayLinkStartStop) {
             setRelays(true);
        } else {
             setRelays(false);
        }

        if (settings.get().autoStart) {
            start();
        }
    } else {
        // Going to sleep
        clearMotionState();
        resetPitch();
        resetClosedLoopControl(true);
        _state = STATE_STANDBY;
        forceDriveOutputsOff();
        setStandbyRelay(false);

        // Reset Session Runtime
        settings.resetSessionRuntime();

        // Save Total Runtime (Silent)
        settings.save(false);
    }
    currentMotorState = _state;
}

void MotorController::emergencyStop() {
    if (_relayTestMode) {
        setRelayTestStage(0);
        _relayTestMode = false;
        _relayTestStage = 0;
    }

    _state = ENABLE_STANDBY ? STATE_STANDBY : STATE_STOPPED;
    currentMotorState = _state;

    clearMotionState();
    resetClosedLoopControl(true);
    forceDriveOutputsOff();
    setStandbyRelay(_state != STATE_STANDBY);
    settings.resetSessionRuntime();
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
    if (mode < SPEED_33 || mode > SPEED_78) mode = SPEED_33;
    if (mode == SPEED_78 && !settings.get().enable78rpm) mode = SPEED_45;
    if (_currentSpeedMode == mode) return;

    SpeedMode previousSpeedMode = _currentSpeedMode;
    float previousTargetRpm = calculateClosedLoopTargetRpmForSpeed(previousSpeedMode);

    _currentSpeedMode = mode;
    settings.get().currentSpeed = mode;
    SpeedSettings& s = settings.getCurrentSpeedSettings();

    // Calculate new target frequency including pitch
    float baseFreq = s.frequency;
    float pitchMod = baseFreq * (currentPitchPercent / 100.0);
    float newTarget = clampOutputFrequency(baseFreq + pitchMod);

    if (_state == STATE_RUNNING) {
        if (settings.get().smoothSwitching) {
            // Initiate smooth frequency ramp
            _isSpeedRamping = true;
            _rampStartFreq = waveform.getFrequency();
            _rampTargetFreq = newTarget;
            _rampStartTime = hal.getMillis();
            _rampDuration = settings.get().switchRampDuration * 1000.0;
            _rampStartRpm = previousTargetRpm;
            _rampTargetRpm = calculateClosedLoopTargetRpmForSpeed(mode);
            _targetFreq = newTarget;
            resetClosedLoopControl(false);
            waveform.updateSettings(_rampStartFreq, s);
        } else {
            // Instant switch
            _isSpeedRamping = false;
            _targetFreq = newTarget;
            _currentFreq = _targetFreq;
            currentFrequency = _currentFreq;
            scheduleClosedLoopEngage(hal.getMillis());
            waveform.updateSettings(_currentFreq, s);
        }
    } else {
        _targetFreq = newTarget;
        _currentFreq = _targetFreq;
        currentFrequency = _currentFreq;
        resetClosedLoopControl(false);
        waveform.updateSettings(_currentFreq, s);
    }

    // Defer save to avoid blocking
    _settingsDirty = true;
    _lastSettingsChange = hal.getMillis();
}

void MotorController::setPitch(float percent) {
    if (percent > _pitchRange) percent = _pitchRange;
    if (percent < -_pitchRange) percent = -_pitchRange;
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

    _targetFreq = clampOutputFrequency(s.frequency);
    _currentFreq = _targetFreq;
    currentFrequency = _currentFreq;

    speedFeedback.configure();
    if (_state == STATE_RUNNING) {
        scheduleClosedLoopEngage(hal.getMillis());
    } else {
        resetClosedLoopControl(false);
    }
    waveform.updateSettings(_currentFreq, s);
}

void MotorController::resetClosedLoop() {
    resetClosedLoopControl(true);
}

float MotorController::calculateClosedLoopTargetRpm() const {
    return calculateClosedLoopTargetRpmForSpeed(_currentSpeedMode);
}

float MotorController::calculateClosedLoopTargetRpmForSpeed(SpeedMode speed) const {
    uint8_t index = (uint8_t)speed;
    if (index > SPEED_78) index = SPEED_33;

    float targetRpm = settings.get().closedLoopTargetRpm[index];
    targetRpm *= 1.0f + (currentPitchPercent / 100.0f);
    if (targetRpm < 0.0f) targetRpm = 0.0f;
    return targetRpm;
}

float MotorController::clampToCurrentSpeedRange(float freq) const {
    SpeedSettings& s = settings.getCurrentSpeedSettings();
    return clampToSpeedSettings(clampOutputFrequency(freq), s);
}

void MotorController::scheduleClosedLoopEngage(uint32_t now) {
    _closedLoopActive = false;
    _closedLoopCorrectionHz = 0.0f;
    resetClosedLoopPidState();
    _closedLoopTargetLastUpdate = 0;
    _closedLoopEngageTime = now + settings.get().closedLoopEngageDelayMs;
    _closedLoopDirectionFaultLatched = false;
    _closedLoopDropoutLatched = false;
    _closedLoopSaturationStart = 0;
    _closedLoopSaturationLatched = false;
    _closedLoopLockWaitStart = 0;
    _closedLoopLockTimeoutLatched = false;
    _closedLoopPlausibilityLatched = false;
    _closedLoopAmpOutOfLockStart = 0;
    _closedLoopAmpRecoveryActive = false;
    _closedLoopAmpRecoveryLatched = false;
}

void MotorController::resetClosedLoopControl(bool resetFeedback) {
    _closedLoopActive = false;
    _closedLoopTargetRpm = 0.0f;
    _closedLoopRequestedTargetRpm = 0.0f;
    _closedLoopRampTargetRpm = 0.0f;
    _closedLoopCorrectionHz = 0.0f;
    resetClosedLoopPidState();
    _closedLoopTargetLastUpdate = 0;
    _closedLoopEngageTime = 0;
    _closedLoopDirectionFaultLatched = false;
    _closedLoopDropoutLatched = false;
    _closedLoopSaturationStart = 0;
    _closedLoopSaturationLatched = false;
    _closedLoopLockWaitStart = 0;
    _closedLoopLockTimeoutLatched = false;
    _closedLoopPlausibilityLatched = false;
    _closedLoopAmpOutOfLockStart = 0;
    _closedLoopAmpRecoveryActive = false;
    _closedLoopAmpRecoveryLatched = false;
    if (resetFeedback) {
        speedFeedback.reset();
    }
}

float MotorController::updateClosedLoopTarget(uint32_t now, float requestedRpm) {
#if CLOSED_LOOP_SPEED_ENABLE
    GlobalSettings& g = settings.get();
    _closedLoopRequestedTargetRpm = requestedRpm;

    if (_closedLoopTargetLastUpdate == 0 || _closedLoopTargetRpm <= 0.0f || g.closedLoopPitchSlewRpmPerSec <= 0.0f) {
        if (g.closedLoopPitchResetThresholdRpm > 0.0f &&
            fabs(requestedRpm - _closedLoopTargetRpm) >= g.closedLoopPitchResetThresholdRpm) {
            resetClosedLoopPidState();
        }
        _closedLoopTargetRpm = requestedRpm;
        _closedLoopTargetLastUpdate = now;
        return _closedLoopTargetRpm;
    }

    uint32_t elapsedMs = now - _closedLoopTargetLastUpdate;
    float delta = requestedRpm - _closedLoopTargetRpm;
    if (g.closedLoopPitchResetThresholdRpm > 0.0f &&
        fabs(delta) >= g.closedLoopPitchResetThresholdRpm) {
        resetClosedLoopPidState();
    }

    float maxStep = g.closedLoopPitchSlewRpmPerSec * (elapsedMs / 1000.0f);
    if (maxStep <= 0.0f || fabs(delta) <= maxStep) {
        _closedLoopTargetRpm = requestedRpm;
    } else if (delta > 0.0f) {
        _closedLoopTargetRpm += maxStep;
    } else {
        _closedLoopTargetRpm -= maxStep;
    }

    _closedLoopTargetLastUpdate = now;
    return _closedLoopTargetRpm;
#else
    (void)now;
    _closedLoopRequestedTargetRpm = requestedRpm;
    _closedLoopTargetRpm = requestedRpm;
    return _closedLoopTargetRpm;
#endif
}

bool MotorController::closedLoopControlAllowed() const {
#if CLOSED_LOOP_SPEED_ENABLE
    return settings.get().closedLoopEnabled &&
           settings.get().closedLoopControlMode == CLOSED_LOOP_CONTROL_CORRECT;
#else
    return false;
#endif
}

void MotorController::resetClosedLoopPidState() {
    _closedLoopIntegralHz = 0.0f;
    _closedLoopLastErrorRpm = 0.0f;
    _closedLoopLastUpdate = 0;
    _closedLoopSaturationStart = 0;
}

void MotorController::reportClosedLoopAction(const char* message, uint8_t action, bool& latch) {
#if CLOSED_LOOP_SPEED_ENABLE
    if (action == CLOSED_LOOP_FAULT_IGNORE || latch) return;

    latch = true;
    errorHandler.report(ERR_SPEED_FEEDBACK, message, action == CLOSED_LOOP_FAULT_STOP);
#else
    (void)message;
    (void)action;
    (void)latch;
#endif
}

bool MotorController::closedLoopSafetyAllowsCorrection(uint32_t now, const SpeedFeedbackStatus& feedback) {
#if CLOSED_LOOP_SPEED_ENABLE
    GlobalSettings& g = settings.get();

    if (feedback.signalValid) {
        bool implausible = false;
        if (g.closedLoopPlausibilityMinRpm > 0.0f && feedback.filteredRpm < g.closedLoopPlausibilityMinRpm) {
            implausible = true;
        }
        if (g.closedLoopPlausibilityMaxRpm > 0.0f && feedback.filteredRpm > g.closedLoopPlausibilityMaxRpm) {
            implausible = true;
        }

        if (implausible) {
            reportClosedLoopAction("Speed feedback implausible", g.closedLoopPlausibilityAction, _closedLoopPlausibilityLatched);
            if (g.closedLoopPlausibilityAction == CLOSED_LOOP_FAULT_STOP) return false;
        } else {
            _closedLoopPlausibilityLatched = false;
        }
    }

    if (g.closedLoopLockTimeoutMs > 0 && feedback.signalValid) {
        if (feedback.locked) {
            _closedLoopLockWaitStart = 0;
            _closedLoopLockTimeoutLatched = false;
        } else {
            if (_closedLoopLockWaitStart == 0) {
                _closedLoopLockWaitStart = now;
            } else if (now - _closedLoopLockWaitStart >= g.closedLoopLockTimeoutMs) {
                reportClosedLoopAction("Speed feedback lock timeout", g.closedLoopLockTimeoutAction, _closedLoopLockTimeoutLatched);
                if (g.closedLoopLockTimeoutAction == CLOSED_LOOP_FAULT_STOP) return false;
            }
        }
    }

    return true;
#else
    (void)now;
    (void)feedback;
    return true;
#endif
}

float MotorController::applyClosedLoopRampCorrection(uint32_t now, float openLoopFreq, float rampTargetRpm) {
#if CLOSED_LOOP_SPEED_ENABLE
    GlobalSettings& g = settings.get();
    if (!closedLoopControlAllowed() || g.closedLoopRampMode != CLOSED_LOOP_RAMP_TRACK) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        resetClosedLoopPidState();
        return openLoopFreq;
    }

    SpeedFeedbackStatus feedback = speedFeedback.getStatus();
    if (!feedback.signalValid) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        resetClosedLoopPidState();
        return openLoopFreq;
    }

    if (!closedLoopSafetyAllowsCorrection(now, feedback)) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        return openLoopFreq;
    }

    float errorRpm = rampTargetRpm - feedback.filteredRpm;
    if (fabs(errorRpm) < g.closedLoopDeadbandRpm) errorRpm = 0.0f;

    float requestedCorrection = g.closedLoopRampKp * errorRpm;
    if (requestedCorrection > g.closedLoopRampCorrectionLimitHz) {
        requestedCorrection = g.closedLoopRampCorrectionLimitHz;
    } else if (requestedCorrection < -g.closedLoopRampCorrectionLimitHz) {
        requestedCorrection = -g.closedLoopRampCorrectionLimitHz;
    }

    _closedLoopCorrectionHz = requestedCorrection;
    _closedLoopActive = true;
    return clampToCurrentSpeedRange(openLoopFreq + _closedLoopCorrectionHz);
#else
    (void)now;
    (void)rampTargetRpm;
    return openLoopFreq;
#endif
}

float MotorController::applyClosedLoopCorrection(uint32_t now, float openLoopFreq) {
#if CLOSED_LOOP_SPEED_ENABLE
    GlobalSettings& g = settings.get();
    if (!g.closedLoopEnabled) {
        resetClosedLoopControl(false);
        return openLoopFreq;
    }

    if (!closedLoopControlAllowed()) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        resetClosedLoopPidState();
        return openLoopFreq;
    }

    if (now < _closedLoopEngageTime) {
        _closedLoopActive = false;
        return openLoopFreq;
    }

    SpeedFeedbackStatus feedback = speedFeedback.getStatus();
    if (g.closedLoopRequireSignalBeforeEngage && _closedLoopLastUpdate == 0 && !feedback.signalValid) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        resetClosedLoopPidState();
        return openLoopFreq;
    }

    if (!feedback.signalValid) {
        _closedLoopActive = false;
        resetClosedLoopPidState();

        if (g.closedLoopDropoutAction == CLOSED_LOOP_DROPOUT_STOP) {
            if (!_closedLoopDropoutLatched) {
                _closedLoopDropoutLatched = true;
                errorHandler.report(ERR_SPEED_FEEDBACK, "Speed feedback lost", true);
            }
            return openLoopFreq;
        }

        if (g.closedLoopDropoutAction == CLOSED_LOOP_DROPOUT_HOLD) {
            return clampToCurrentSpeedRange(openLoopFreq + _closedLoopCorrectionHz);
        }

        _closedLoopCorrectionHz = 0.0f;
        return openLoopFreq;
    }
    _closedLoopDropoutLatched = false;

    if (g.closedLoopRequireNearTargetBeforeEngage &&
        _closedLoopLastUpdate == 0 &&
        fabs(feedback.rpmError) > g.closedLoopEngageToleranceRpm) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        resetClosedLoopPidState();
        return openLoopFreq;
    }

    if (!closedLoopSafetyAllowsCorrection(now, feedback)) {
        _closedLoopActive = false;
        _closedLoopCorrectionHz = 0.0f;
        return openLoopFreq;
    }

    if (g.closedLoopSensorMode == CLOSED_LOOP_SENSOR_QUADRATURE &&
        feedback.direction == SPEED_FEEDBACK_DIR_REVERSE &&
        g.closedLoopDirectionFaultAction != CLOSED_LOOP_FAULT_IGNORE) {
        if (!_closedLoopDirectionFaultLatched) {
            _closedLoopDirectionFaultLatched = true;
            bool critical = g.closedLoopDirectionFaultAction == CLOSED_LOOP_FAULT_STOP;
            errorHandler.report(ERR_SPEED_FEEDBACK, "Speed feedback reversed", critical);
        }
        if (g.closedLoopDirectionFaultAction == CLOSED_LOOP_FAULT_STOP) {
            return openLoopFreq;
        }
    } else if (feedback.direction != SPEED_FEEDBACK_DIR_REVERSE) {
        _closedLoopDirectionFaultLatched = false;
    }

    uint32_t elapsedMs = _closedLoopLastUpdate == 0 ? 0 : now - _closedLoopLastUpdate;
    if (_closedLoopLastUpdate == 0) {
        _closedLoopLastUpdate = now;
        _closedLoopLastErrorRpm = feedback.rpmError;
        _closedLoopActive = true;
        return clampToCurrentSpeedRange(openLoopFreq + _closedLoopCorrectionHz);
    }
    if (elapsedMs < g.closedLoopUpdateIntervalMs) {
        _closedLoopActive = true;
        return clampToCurrentSpeedRange(openLoopFreq + _closedLoopCorrectionHz);
    }

    float dt = elapsedMs / 1000.0f;
    float errorRpm = feedback.rpmError;
    if (fabs(errorRpm) < g.closedLoopDeadbandRpm) errorRpm = 0.0f;

    float proportionalHz = g.closedLoopKp * errorRpm;
    if (g.closedLoopKi > 0.0f && g.closedLoopIntegralLimitHz > 0.0f) {
        _closedLoopIntegralHz += g.closedLoopKi * errorRpm * dt;
        if (_closedLoopIntegralHz > g.closedLoopIntegralLimitHz) {
            _closedLoopIntegralHz = g.closedLoopIntegralLimitHz;
        } else if (_closedLoopIntegralHz < -g.closedLoopIntegralLimitHz) {
            _closedLoopIntegralHz = -g.closedLoopIntegralLimitHz;
        }
    } else {
        _closedLoopIntegralHz = 0.0f;
    }

    float derivativeHz = 0.0f;
    if (dt > 0.0f && g.closedLoopKd > 0.0f) {
        derivativeHz = g.closedLoopKd * ((errorRpm - _closedLoopLastErrorRpm) / dt);
    }

    float requestedCorrection = proportionalHz + _closedLoopIntegralHz + derivativeHz;
    if (requestedCorrection > g.closedLoopCorrectionLimitHz) {
        requestedCorrection = g.closedLoopCorrectionLimitHz;
    } else if (requestedCorrection < -g.closedLoopCorrectionLimitHz) {
        requestedCorrection = -g.closedLoopCorrectionLimitHz;
    }

    if (g.closedLoopCorrectionLimitHz > 0.0f &&
        fabs(requestedCorrection) >= (g.closedLoopCorrectionLimitHz - 0.001f)) {
        if (_closedLoopSaturationStart == 0) {
            _closedLoopSaturationStart = now;
        } else if (g.closedLoopSaturationTimeMs > 0 &&
                   now - _closedLoopSaturationStart >= g.closedLoopSaturationTimeMs) {
            reportClosedLoopAction("Speed correction saturated", g.closedLoopSaturationAction, _closedLoopSaturationLatched);
            if (g.closedLoopSaturationAction == CLOSED_LOOP_FAULT_STOP) {
                _closedLoopActive = false;
                return openLoopFreq;
            }
        }
    } else {
        _closedLoopSaturationStart = 0;
        _closedLoopSaturationLatched = false;
    }

    if (g.closedLoopSlewLimitHzPerSec > 0.0f) {
        float maxStep = g.closedLoopSlewLimitHzPerSec * dt;
        float delta = requestedCorrection - _closedLoopCorrectionHz;
        if (delta > maxStep) {
            requestedCorrection = _closedLoopCorrectionHz + maxStep;
        } else if (delta < -maxStep) {
            requestedCorrection = _closedLoopCorrectionHz - maxStep;
        }
    }

    _closedLoopCorrectionHz = requestedCorrection;
    _closedLoopLastErrorRpm = errorRpm;
    _closedLoopLastUpdate = now;
    _closedLoopActive = true;
    return clampToCurrentSpeedRange(openLoopFreq + _closedLoopCorrectionHz);
#else
    (void)now;
    return openLoopFreq;
#endif
}

void MotorController::updateClosedLoopAmpRecovery(uint32_t now, const SpeedFeedbackStatus& feedback) {
#if CLOSED_LOOP_SPEED_ENABLE
    GlobalSettings& g = settings.get();
    if (g.closedLoopAmpRecoveryMode == CLOSED_LOOP_AMP_RECOVERY_OFF || !_isReducedAmp) {
        _closedLoopAmpOutOfLockStart = 0;
        _closedLoopAmpRecoveryActive = false;
        _closedLoopAmpRecoveryLatched = false;
        return;
    }

    if (!feedback.signalValid || feedback.locked || fabs(feedback.rpmError) <= g.closedLoopLockToleranceRpm) {
        _closedLoopAmpOutOfLockStart = 0;
        _closedLoopAmpRecoveryActive = false;
        _closedLoopAmpRecoveryLatched = false;
        return;
    }

    if (_closedLoopAmpOutOfLockStart == 0) {
        _closedLoopAmpOutOfLockStart = now;
        return;
    }

    if (now - _closedLoopAmpOutOfLockStart < g.closedLoopAmpRecoveryDelayMs) return;

    if (g.closedLoopAmpRecoveryMode == CLOSED_LOOP_AMP_RECOVERY_RESTORE) {
        if (!_closedLoopAmpRecoveryActive) {
            _closedLoopAmpRecoveryActive = true;
            _isReducedAmp = false;
            _currentAmp = _targetAmp;
            waveform.setAmplitude(_currentAmp);
            _ampReductionStartTime = now;
        }
        reportClosedLoopAction("Speed unlocked, full amplitude restored", CLOSED_LOOP_FAULT_WARN, _closedLoopAmpRecoveryLatched);
    } else if (g.closedLoopAmpRecoveryMode == CLOSED_LOOP_AMP_RECOVERY_WARN) {
        reportClosedLoopAction("Speed unlocked after amplitude reduction", CLOSED_LOOP_FAULT_WARN, _closedLoopAmpRecoveryLatched);
    }
#else
    (void)now;
    (void)feedback;
#endif
}

void MotorController::clearMotionState() {
    _isKicking = false;
    _isKickRamping = false;
    _isSpeedRamping = false;
    _isSweepingMode = false;
    _isReducedAmp = false;
    _brakePulseState = false;
    resetClosedLoopControl(false);
    _targetAmp = 0.0;
    _currentAmp = 0.0;
}

void MotorController::forceDriveOutputsOff() {
    _relayActivationPending = false;
    _relaysActive = false;
    _relayStage = 0;
    setRelays(false);
    waveform.setAmplitude(0.0);
    waveform.setEnabled(false);
}

void MotorController::setRelays(bool active) {
    if (!ENABLE_MUTE_RELAYS) return;
    if (_relayTestMode) return;

    bool activeHigh = settings.get().relayActiveHigh;
    bool requestedActive = active;

    // Safety: Enforce Power On Delay
    if (active && _powerOnDelayActive) {
        uint32_t delayMs = settings.get().powerOnRelayDelay * 1000;
        if (hal.getMillis() - _powerOnTime < delayMs) {
            _relayActivationPending = true;
            active = false; // Force mute
        } else {
            _powerOnDelayActive = false;
        }
    }
    if (!requestedActive) {
        _relayActivationPending = false;
    }

    if (active) {
        // Start Staggered Unmute Sequence
        _relayActivationPending = false;
        _relaysActive = true;
        _relayStage = 0;
        _relayStageTime = hal.getMillis();
        // Pins are updated in update() loop
    } else {
        // Immediate Mute (All Off)
        _relaysActive = false;
        _relayStage = 0;

        if (ENABLE_DPDT_RELAYS) {
             hal.digitalWrite(PIN_RELAY_DPDT_1, activeHigh ? LOW : HIGH);
             hal.digitalWrite(PIN_RELAY_DPDT_2, activeHigh ? LOW : HIGH);
        } else {
            hal.digitalWrite(PIN_MUTE_PHASE_A, activeHigh ? LOW : HIGH);
            hal.digitalWrite(PIN_MUTE_PHASE_B, activeHigh ? LOW : HIGH);
            hal.digitalWrite(PIN_MUTE_PHASE_C, activeHigh ? LOW : HIGH);
#if ENABLE_4_CHANNEL_SUPPORT
            hal.digitalWrite(PIN_MUTE_PHASE_D, activeHigh ? LOW : HIGH);
#endif
        }
    }
}

void MotorController::setStandbyRelay(bool active) {
    if (!ENABLE_STANDBY) {
        active = true;
    }

    bool activeHigh = settings.get().relayActiveHigh;
    hal.digitalWrite(PIN_RELAY_STANDBY, active ? (activeHigh ? HIGH : LOW) : (activeHigh ? LOW : HIGH));
}

void MotorController::writeRelayOutput(int pin, bool active) {
    bool activeHigh = settings.get().relayActiveHigh;
    hal.digitalWrite(pin, active ? (activeHigh ? HIGH : LOW) : (activeHigh ? LOW : HIGH));
}

uint8_t MotorController::getRelayTestStageCount() {
    uint8_t count = 1; // All off
    if (ENABLE_STANDBY) count++;
    if (ENABLE_MUTE_RELAYS) {
        count += ENABLE_DPDT_RELAYS ? 2 : MAX_ACTIVE_PHASE_OUTPUTS;
    }
    return count;
}

bool MotorController::beginRelayTest() {
    if (_state == STATE_STARTING || _state == STATE_RUNNING || _state == STATE_STOPPING) {
        return false;
    }

    clearMotionState();
    forceDriveOutputsOff();
    _relayTestMode = true;
    _state = STATE_STOPPED;
    currentMotorState = _state;

    setRelayTestStage(0);
    return true;
}

void MotorController::setRelayTestStage(uint8_t stage) {
    uint8_t count = getRelayTestStageCount();
    if (count == 0) return;
    if (stage >= count) stage = count - 1;
    _relayTestStage = stage;

    writeRelayOutput(PIN_RELAY_STANDBY, false);

    if (ENABLE_MUTE_RELAYS) {
        if (ENABLE_DPDT_RELAYS) {
            writeRelayOutput(PIN_RELAY_DPDT_1, false);
            writeRelayOutput(PIN_RELAY_DPDT_2, false);
        } else {
            writeRelayOutput(PIN_MUTE_PHASE_A, false);
            writeRelayOutput(PIN_MUTE_PHASE_B, false);
            writeRelayOutput(PIN_MUTE_PHASE_C, false);
#if ENABLE_4_CHANNEL_SUPPORT
            writeRelayOutput(PIN_MUTE_PHASE_D, false);
#endif
        }
    }

    if (stage == 0) return;

    uint8_t relayStage = stage;
    if (ENABLE_STANDBY) {
        if (relayStage == 1) {
            writeRelayOutput(PIN_RELAY_STANDBY, true);
            return;
        }
        relayStage--;
    }

    if (!ENABLE_MUTE_RELAYS) return;

    if (ENABLE_DPDT_RELAYS) {
        if (relayStage == 1) writeRelayOutput(PIN_RELAY_DPDT_1, true);
        else if (relayStage == 2) writeRelayOutput(PIN_RELAY_DPDT_2, true);
    } else {
        if (relayStage == 1) writeRelayOutput(PIN_MUTE_PHASE_A, true);
        else if (relayStage == 2) writeRelayOutput(PIN_MUTE_PHASE_B, true);
        else if (relayStage == 3) writeRelayOutput(PIN_MUTE_PHASE_C, true);
#if ENABLE_4_CHANNEL_SUPPORT
        else if (relayStage == 4) writeRelayOutput(PIN_MUTE_PHASE_D, true);
#endif
    }
}

void MotorController::endRelayTest() {
    if (!_relayTestMode) return;

    setRelayTestStage(0);
    _relayTestMode = false;
    _relayTestStage = 0;

    setStandbyRelay(_state != STATE_STANDBY);
    if (_state == STATE_STOPPED && settings.get().muteRelayLinkStandby && !settings.get().muteRelayLinkStartStop) {
        setRelays(true);
    } else {
        setRelays(false);
    }
}

float MotorController::getMotionProgress() {
    uint32_t now = hal.getMillis();

    if (_state == STATE_STARTING) {
        float duration = settings.getCurrentSpeedSettings().softStartDuration * 1000.0;
        if (duration <= 0.0) return 1.0;
        float progress = (float)(now - _stateStartTime) / duration;
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;
        return progress;
    }

    if (_state == STATE_STOPPING) {
        float duration = settings.get().brakeDuration * 1000.0;
        if (duration <= 0.0) return 1.0;
        float progress = (float)(now - _stateStartTime) / duration;
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;
        return progress;
    }

    if (_isSpeedRamping) {
        if (_rampDuration <= 0.0) return 1.0;
        float progress = (float)(now - _rampStartTime) / _rampDuration;
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;
        return progress;
    }

    return 0.0;
}
