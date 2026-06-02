/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "config.h"

// --- Enumerations ---
// These values are persisted, exported, and exposed through serial/web APIs.
// Avoid reordering existing entries; add new values at the end or migrate stored
// data explicitly.

enum MotorState {
    STATE_STANDBY,  // Low power, relays off
    STATE_STOPPED,  // Powered, relays mute, no rotation
    STATE_STARTING, // Accelerating (Kick/Soft Start)
    STATE_RUNNING,  // Stable rotation
    STATE_STOPPING  // Decelerating (Braking)
};

enum SpeedMode {
    SPEED_33 = 0,
    SPEED_45 = 1,
    SPEED_78 = 2
};

// Phase mode is one-based because it represents the number of active phase
// outputs rather than an array index.
enum PhaseMode {
    PHASE_1 = 1,
    PHASE_2 = 2,
    PHASE_3 = 3,
    PHASE_4 = 4
};

enum FilterType {
    FILTER_NONE,
    FILTER_IIR, // Infinite Impulse Response (Low Pass)
    FILTER_FIR  // Finite Impulse Response (Convolution)
};

enum FirProfile {
    FIR_GENTLE,
    FIR_MEDIUM,
    FIR_AGGRESSIVE
};

enum BrakeMode : int {
    BRAKE_OFF,
    BRAKE_PULSE, // Pulsed reverse torque
    BRAKE_RAMP,   // Linear frequency ramp down
    BRAKE_SOFT_STOP // Active coasting down to a cutoff frequency
};

enum RampType {
    RAMP_LINEAR = 0,
    RAMP_SCURVE = 1
};

enum ScreensaverMode {
    SAVER_BOUNCE = 0,
    SAVER_MATRIX = 1,
    SAVER_LISSAJOUS = 2
};

enum ClosedLoopSensorMode {
    CLOSED_LOOP_SENSOR_PULSE = 0,
    CLOSED_LOOP_SENSOR_QUADRATURE = 1
};

enum ClosedLoopControlMode {
    CLOSED_LOOP_CONTROL_MONITOR = 0,
    CLOSED_LOOP_CONTROL_CORRECT = 1
};

enum ClosedLoopPulseEdge {
    CLOSED_LOOP_EDGE_RISING = 0,
    CLOSED_LOOP_EDGE_FALLING = 1,
    CLOSED_LOOP_EDGE_CHANGE = 2
};

enum ClosedLoopQuadratureMode {
    CLOSED_LOOP_QUAD_X1 = 0,
    CLOSED_LOOP_QUAD_X2 = 1,
    CLOSED_LOOP_QUAD_X4 = 2
};

enum ClosedLoopFaultAction {
    CLOSED_LOOP_FAULT_IGNORE = 0,
    CLOSED_LOOP_FAULT_WARN = 1,
    CLOSED_LOOP_FAULT_STOP = 2
};

enum ClosedLoopDropoutAction {
    CLOSED_LOOP_DROPOUT_OPEN_LOOP = 0,
    CLOSED_LOOP_DROPOUT_HOLD = 1,
    CLOSED_LOOP_DROPOUT_STOP = 2
};

enum ClosedLoopRampMode {
    CLOSED_LOOP_RAMP_DISABLED = 0,
    CLOSED_LOOP_RAMP_TRACK = 1
};

enum ClosedLoopAmpRecoveryMode {
    CLOSED_LOOP_AMP_RECOVERY_OFF = 0,
    CLOSED_LOOP_AMP_RECOVERY_WARN = 1,
    CLOSED_LOOP_AMP_RECOVERY_RESTORE = 2
};

enum ClosedLoopPitchTargetMode {
    CLOSED_LOOP_PITCH_TARGET_FIXED = 0,
    CLOSED_LOOP_PITCH_TARGET_FOLLOW = 1
};

// --- Data Structures ---
// The settings structs below are written directly to LittleFS. Field order,
// types, and padding are part of the storage contract.

// Settings specific to a single speed (33, 45, or 78 RPM). GlobalSettings owns
// three copies so each speed can have its own frequency, phase, and startup tune.
struct SpeedSettings {
    // Nominal DDS output and user-adjustable bounds for pitch changes.
    float frequency;
    float minFrequency;
    float maxFrequency;
    
    // Phase Offsets (Degrees)
    // Index 0 is Phase 1 (Reference, usually 0)
    float phaseOffset[4];
    
    // Motor Control
    float softStartDuration; // Seconds
    uint8_t reducedAmplitude; // 10-100%
    uint8_t amplitudeDelay; // Seconds
    uint8_t startupKick; // Multiplier (1-4)
    uint8_t startupKickDuration; // Seconds
    float startupKickRampDuration; // Seconds
    
    // Digital Filters
    uint8_t filterType; // 0=None, 1=IIR, 2=FIR
    float iirAlpha;
    uint8_t firProfile; // 0=Gentle, 1=Medium, 2=Aggressive
};

// PID-like closed-loop parameters are duplicated per speed. That lets a deck
// use gentler correction at one RPM and tighter correction at another.
struct ClosedLoopSpeedTuning {
    float deadbandRpm;
    float lockToleranceRpm;
    float kp;
    float ki;
    float kd;
    float integralLimitHz;
    float correctionLimitHz;
    float slewLimitHzPerSec;
    float rampKp;
    float rampCorrectionLimitHz;
    uint16_t lockTimeMs;
};

// Top-level persisted settings. Keep new fields grouped by feature and update
// settings.cpp, menus, serial registry, web JSON, and schema migration together.
struct GlobalSettings {
    // Version is checked before using binary contents from LittleFS.
    uint32_t schemaVersion;
    
    // Phase Configuration
    uint8_t phaseMode; // 1, 2, 3, 4
    
    // Motor Global
    uint8_t maxAmplitude; // 0-100%
    uint8_t softStartCurve; // 0=Linear, 1=Log, 2=Exp
    bool smoothSwitching;
    uint8_t switchRampDuration; // Seconds
    
    // Braking
    // These are output-driving values; validation keeps them within motor-safe
    // limits before MotorController applies them.
    uint8_t brakeMode; // 0=Off, 1=Pulse, 2=Ramp, 3=SoftStop
    float brakeDuration;
    float brakePulseGap;
    float brakeStartFreq;
    float brakeStopFreq;
    float softStopCutoff; // Hz limit for active coasting
    
    // Relays
    bool relayActiveHigh;
    bool muteRelayLinkStandby;
    bool muteRelayLinkStartStop;
    uint8_t powerOnRelayDelay;
    
    // Display
    // Delay fields are menu indices or minutes depending on the label; see
    // settings.cpp validation/defaults and menu_data.cpp labels.
    uint8_t displayBrightness;
    uint8_t displaySleepDelay; // Index
    bool screensaverEnabled;
    uint8_t autoDimDelay; // Minutes
    bool showRuntime;
    bool errorDisplayEnabled;
    uint8_t errorDisplayDuration;
    
    // System
    uint8_t autoStandbyDelay; // Minutes
    bool autoStart;
    bool autoBoot;
    bool pitchResetOnStop;
    
    // Presets
    SpeedSettings speeds[3]; // 33, 45, 78
    char presetNames[5][17]; // 5 Slots, 16 chars + null
    
    // Runtime Stats
    uint32_t totalRuntime; // Seconds
    
    // New Settings (v2)
    bool reverseEncoder;
    float pitchStepSize; // 0.01 - 1.0
    uint8_t rampType; // 0=Linear, 1=S-Curve
    uint8_t screensaverMode; // 0=Bounce, 1=Matrix, 2=Lissajous
    
    bool enable78rpm;
    uint8_t freqDependentAmplitude; // 0-100% (FDA master toggle/multiplier)
    
    // 3-Point V/f Curve Definitions
    float vfLowFreq; // Hz
    uint8_t vfLowBoost; // 0-100%
    float vfMidFreq; // Hz
    uint8_t vfMidBoost; // 0-100%
    
    uint8_t bootSpeed; // 0=33, 1=45, 2=78, 3=Last Used
    
    // Current State Persistence
    // Stored separately from bootSpeed so "Last Used" can be honored on the next
    // boot without changing the user's boot mode preference.
    SpeedMode currentSpeed;

    // Amplifier Monitor
    float ampTempWarnC;
    float ampTempShutdownC;

    // Dashboard resource pages
    bool showCpuDashboard;
    bool showMemoryDashboard;
    bool showFlashDashboard;

    // Closed-loop speed feedback
    // Closed-loop can run as monitor-only diagnostics or actively trim waveform
    // frequency. Feature flags decide whether pins and ISR code are compiled in.
    bool closedLoopEnabled;
    uint8_t closedLoopControlMode; // 0=Monitor only, 1=Correct speed
    uint8_t closedLoopSensorMode; // 0=Pulse tach, 1=Quadrature
    float closedLoopTargetRpm[3]; // 33, 45, 78
    uint16_t closedLoopCountsPerRev;
    uint8_t closedLoopPulseEdge; // 0=Rising, 1=Falling, 2=Change
    uint8_t closedLoopQuadratureMode; // 0=x1, 1=x2, 2=x4
    bool closedLoopReverseDirection;
    uint8_t closedLoopDirectionFaultAction; // 0=Ignore, 1=Warn, 2=Stop
    uint16_t closedLoopDebounceUs;
    uint16_t closedLoopTimeoutMs;
    uint16_t closedLoopEngageDelayMs;
    uint16_t closedLoopUpdateIntervalMs;
    float closedLoopFilterAlpha;
    float closedLoopDeadbandRpm;
    float closedLoopLockToleranceRpm;
    uint16_t closedLoopLockTimeMs;
    float closedLoopKp;
    float closedLoopKi;
    float closedLoopKd;
    float closedLoopIntegralLimitHz;
    float closedLoopCorrectionLimitHz;
    float closedLoopSlewLimitHzPerSec;
    uint8_t closedLoopDropoutAction; // 0=Open loop, 1=Hold, 2=Stop
    bool closedLoopRequireSignalBeforeEngage;
    bool closedLoopRequireNearTargetBeforeEngage;
    float closedLoopEngageToleranceRpm;
    uint8_t closedLoopRampMode; // 0=Disable correction during ramps, 1=Track ramp lightly
    float closedLoopRampKp;
    float closedLoopRampCorrectionLimitHz;
    float closedLoopPitchSlewRpmPerSec;
    float closedLoopPitchResetThresholdRpm;
    uint16_t closedLoopSaturationTimeMs;
    uint8_t closedLoopSaturationAction; // 0=Ignore, 1=Warn, 2=Stop
    float closedLoopPlausibilityMinRpm;
    float closedLoopPlausibilityMaxRpm;
    uint8_t closedLoopPlausibilityAction; // 0=Ignore, 1=Warn, 2=Stop
    uint16_t closedLoopLockTimeoutMs;
    uint8_t closedLoopLockTimeoutAction; // 0=Ignore, 1=Warn, 2=Stop
    uint8_t closedLoopAmpRecoveryMode; // 0=Off, 1=Warn, 2=Restore full amplitude
    uint16_t closedLoopAmpRecoveryDelayMs;
    ClosedLoopSpeedTuning closedLoopTuning[3];
    uint8_t closedLoopPitchTargetMode; // 0=Fixed preset target, 1=Follow effective pitch ratio
};

// These assertions catch accidental storage-layout changes during compilation.
static_assert(sizeof(SpeedSettings) == SPEED_SETTINGS_STORAGE_SIZE, "Update SPEED_SETTINGS_STORAGE_SIZE when SpeedSettings changes.");
static_assert(sizeof(ClosedLoopSpeedTuning) == CLOSED_LOOP_TUNING_STORAGE_SIZE, "Update CLOSED_LOOP_TUNING_STORAGE_SIZE when ClosedLoopSpeedTuning changes.");
static_assert(sizeof(GlobalSettings) == GLOBAL_SETTINGS_STORAGE_SIZE, "Update GLOBAL_SETTINGS_STORAGE_SIZE and storage handling when GlobalSettings changes.");

#endif // TYPES_H
