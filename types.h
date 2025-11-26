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

// --- Enumerations ---

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
    BRAKE_RAMP   // Linear frequency ramp down
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

// --- Data Structures ---

// Settings specific to a single speed (e.g. 33 RPM)
struct SpeedSettings {
    float frequency;
    float minFrequency;
    float maxFrequency;
    
    // Phase Offsets (Degrees)
    // Index 0 is Phase 1 (Reference, usually 0)
    float phaseOffset[4];
    
    // Motor Control
    float softStartDuration; // Seconds
    uint8_t reducedAmplitude; // 50-100%
    uint8_t amplitudeDelay; // Seconds
    uint8_t startupKick; // Multiplier (1-4)
    uint8_t startupKickDuration; // Seconds
    float startupKickRampDuration; // Seconds
    
    // Digital Filters
    uint8_t filterType; // 0=None, 1=IIR, 2=FIR
    float iirAlpha;
    uint8_t firProfile; // 0=Gentle, 1=Medium, 2=Aggressive
};

struct GlobalSettings {
    uint32_t schemaVersion;
    
    // Phase Configuration
    uint8_t phaseMode; // 1, 2, 3, 4
    
    // Motor Global
    uint8_t maxAmplitude; // 0-100%
    uint8_t softStartCurve; // 0=Linear, 1=Log, 2=Exp
    bool smoothSwitching;
    uint8_t switchRampDuration; // Seconds
    
    // Braking
    uint8_t brakeMode; // 0=Off, 1=Pulse, 2=Ramp
    float brakeDuration;
    float brakePulseGap;
    float brakeStartFreq;
    float brakeStopFreq;
    
    // Relays
    bool relayActiveHigh;
    bool muteRelayLinkStandby;
    bool muteRelayLinkStartStop;
    uint8_t powerOnRelayDelay;
    
    // Display
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
    uint8_t freqDependentAmplitude; // 0-100% (FDA)
    uint8_t bootSpeed; // 0=33, 1=45, 2=78, 3=Last Used
    
    // Current State Persistence
    SpeedMode currentSpeed;
};

#endif // TYPES_H
