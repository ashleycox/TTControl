/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "settings.h"
#include "error_handler.h"
#include "globals.h"
#include <ArduinoJson.h>
#include <math.h>

namespace {
/*
 * Settings and presets are stored as a small checked header plus the binary
 * GlobalSettings payload. The header lets future schemas migrate without
 * accepting random flash contents as valid settings.
 *
 * The four-byte packing matches the explicit current-schema storage contract
 * in types.h and keeps legacy migration records identical on RP2350 ARM and
 * RISC-V builds.
 */
#pragma pack(push, 4)
struct SettingsFileHeader {
    uint32_t magic;
    uint16_t formatVersion;
    uint16_t headerSize;
    uint32_t schemaVersion;
    uint32_t payloadSize;
    uint32_t crc32;
};
static_assert(sizeof(SettingsFileHeader) == 20, "SettingsFileHeader storage layout changed.");

static const char* SETTINGS_KNOWN_GOOD_FILE = "/settings_good.bin";
static const char* SETTINGS_BOOT_MARKER_FILE = "/settings_boot.bin";
static const uint32_t SETTINGS_BOOT_MARKER_MAGIC = 0x54544253UL; // "TTBS"
static const uint8_t SETTINGS_BOOT_MARKER_VERSION = 1;
static const uint8_t SETTINGS_BOOT_NONE = 0;
static const uint8_t SETTINGS_BOOT_PENDING = 1;
static const uint8_t SETTINGS_BOOT_BOOTING = 2;

float finiteOr(float value, float fallback) {
    return isfinite(value) ? value : fallback;
}

/*
 * Boot marker state machine:
 * PENDING is written after a protected save, BOOTING is written on the next
 * startup, and NONE is restored only after waveform generation proves healthy.
 */
struct SettingsBootMarker {
    uint32_t magic;
    uint8_t version;
    uint8_t state;
    uint16_t reserved;
    uint32_t crc32;
};
static_assert(sizeof(SettingsBootMarker) == 12, "SettingsBootMarker storage layout changed.");

// Legacy layouts preserve exact field order and padding for schema migration. Do not edit these structs unless you are correcting an older schema definition.
struct SpeedSettingsV9 {
    float frequency;
    float minFrequency;
    float maxFrequency;
    float phaseOffset[4];
    float softStartDuration;
    uint8_t reducedAmplitude;
    uint8_t amplitudeDelay;
    uint8_t startupKick;
    uint8_t startupKickDuration;
    float startupKickRampDuration;
    uint8_t filterType;
    float iirAlpha;
    uint8_t firProfile;
};

static_assert(sizeof(SpeedSettingsV9) == 52, "SpeedSettingsV9 must match schema 9 storage size.");

struct GlobalSettingsV5 {
    uint32_t schemaVersion;
    uint8_t phaseMode;
    uint8_t maxAmplitude;
    uint8_t softStartCurve;
    bool smoothSwitching;
    uint8_t switchRampDuration;
    uint8_t brakeMode;
    float brakeDuration;
    float brakePulseGap;
    float brakeStartFreq;
    float brakeStopFreq;
    float softStopCutoff;
    bool relayActiveHigh;
    bool muteRelayLinkStandby;
    bool muteRelayLinkStartStop;
    uint8_t powerOnRelayDelay;
    uint8_t displayBrightness;
    uint8_t displaySleepDelay;
    bool screensaverEnabled;
    uint8_t autoDimDelay;
    bool showRuntime;
    bool errorDisplayEnabled;
    uint8_t errorDisplayDuration;
    uint8_t autoStandbyDelay;
    bool autoStart;
    bool autoBoot;
    bool pitchResetOnStop;
    SpeedSettingsV9 speeds[3];
    char presetNames[5][17];
    uint32_t totalRuntime;
    bool reverseEncoder;
    float pitchStepSize;
    uint8_t rampType;
    uint8_t screensaverMode;
    bool enable78rpm;
    uint8_t freqDependentAmplitude;
    float vfLowFreq;
    uint8_t vfLowBoost;
    float vfMidFreq;
    uint8_t vfMidBoost;
    uint8_t bootSpeed;
    uint8_t currentSpeed;
    float ampTempWarnC;
    float ampTempShutdownC;
    bool showCpuDashboard;
    bool showMemoryDashboard;
    bool showFlashDashboard;
};

static_assert(sizeof(GlobalSettingsV5) == 336, "GlobalSettingsV5 must match schema 5 storage size.");

struct GlobalSettingsV6 {
    uint32_t schemaVersion;
    uint8_t phaseMode;
    uint8_t maxAmplitude;
    uint8_t softStartCurve;
    bool smoothSwitching;
    uint8_t switchRampDuration;
    uint8_t brakeMode;
    float brakeDuration;
    float brakePulseGap;
    float brakeStartFreq;
    float brakeStopFreq;
    float softStopCutoff;
    bool relayActiveHigh;
    bool muteRelayLinkStandby;
    bool muteRelayLinkStartStop;
    uint8_t powerOnRelayDelay;
    uint8_t displayBrightness;
    uint8_t displaySleepDelay;
    bool screensaverEnabled;
    uint8_t autoDimDelay;
    bool showRuntime;
    bool errorDisplayEnabled;
    uint8_t errorDisplayDuration;
    uint8_t autoStandbyDelay;
    bool autoStart;
    bool autoBoot;
    bool pitchResetOnStop;
    SpeedSettingsV9 speeds[3];
    char presetNames[5][17];
    uint32_t totalRuntime;
    bool reverseEncoder;
    float pitchStepSize;
    uint8_t rampType;
    uint8_t screensaverMode;
    bool enable78rpm;
    uint8_t freqDependentAmplitude;
    float vfLowFreq;
    uint8_t vfLowBoost;
    float vfMidFreq;
    uint8_t vfMidBoost;
    uint8_t bootSpeed;
    uint8_t currentSpeed;
    float ampTempWarnC;
    float ampTempShutdownC;
    bool showCpuDashboard;
    bool showMemoryDashboard;
    bool showFlashDashboard;
    bool closedLoopEnabled;
    uint8_t closedLoopSensorMode;
    float closedLoopTargetRpm[3];
    uint16_t closedLoopCountsPerRev;
    uint8_t closedLoopPulseEdge;
    uint8_t closedLoopQuadratureMode;
    bool closedLoopReverseDirection;
    uint8_t closedLoopDirectionFaultAction;
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
    uint8_t closedLoopDropoutAction;
};

static_assert(sizeof(GlobalSettingsV6) == 412, "GlobalSettingsV6 must match schema 6 storage size.");

struct GlobalSettingsV7 {
    uint32_t schemaVersion;
    uint8_t phaseMode;
    uint8_t maxAmplitude;
    uint8_t softStartCurve;
    bool smoothSwitching;
    uint8_t switchRampDuration;
    uint8_t brakeMode;
    float brakeDuration;
    float brakePulseGap;
    float brakeStartFreq;
    float brakeStopFreq;
    float softStopCutoff;
    bool relayActiveHigh;
    bool muteRelayLinkStandby;
    bool muteRelayLinkStartStop;
    uint8_t powerOnRelayDelay;
    uint8_t displayBrightness;
    uint8_t displaySleepDelay;
    bool screensaverEnabled;
    uint8_t autoDimDelay;
    bool showRuntime;
    bool errorDisplayEnabled;
    uint8_t errorDisplayDuration;
    uint8_t autoStandbyDelay;
    bool autoStart;
    bool autoBoot;
    bool pitchResetOnStop;
    SpeedSettingsV9 speeds[3];
    char presetNames[5][17];
    uint32_t totalRuntime;
    bool reverseEncoder;
    float pitchStepSize;
    uint8_t rampType;
    uint8_t screensaverMode;
    bool enable78rpm;
    uint8_t freqDependentAmplitude;
    float vfLowFreq;
    uint8_t vfLowBoost;
    float vfMidFreq;
    uint8_t vfMidBoost;
    uint8_t bootSpeed;
    uint8_t currentSpeed;
    float ampTempWarnC;
    float ampTempShutdownC;
    bool showCpuDashboard;
    bool showMemoryDashboard;
    bool showFlashDashboard;
    bool closedLoopEnabled;
    uint8_t closedLoopControlMode;
    uint8_t closedLoopSensorMode;
    float closedLoopTargetRpm[3];
    uint16_t closedLoopCountsPerRev;
    uint8_t closedLoopPulseEdge;
    uint8_t closedLoopQuadratureMode;
    bool closedLoopReverseDirection;
    uint8_t closedLoopDirectionFaultAction;
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
    uint8_t closedLoopDropoutAction;
    bool closedLoopRequireSignalBeforeEngage;
    bool closedLoopRequireNearTargetBeforeEngage;
    float closedLoopEngageToleranceRpm;
    uint8_t closedLoopRampMode;
    float closedLoopRampKp;
    float closedLoopRampCorrectionLimitHz;
    float closedLoopPitchSlewRpmPerSec;
    float closedLoopPitchResetThresholdRpm;
    uint16_t closedLoopSaturationTimeMs;
    uint8_t closedLoopSaturationAction;
    float closedLoopPlausibilityMinRpm;
    float closedLoopPlausibilityMaxRpm;
    uint8_t closedLoopPlausibilityAction;
    uint16_t closedLoopLockTimeoutMs;
    uint8_t closedLoopLockTimeoutAction;
    uint8_t closedLoopAmpRecoveryMode;
    uint16_t closedLoopAmpRecoveryDelayMs;
};

static_assert(sizeof(GlobalSettingsV7) == 456, "GlobalSettingsV7 must match schema 7 storage size.");

struct GlobalSettingsV8 {
    GlobalSettingsV7 base;
    ClosedLoopSpeedTuning closedLoopTuning[3];
};

static_assert(sizeof(GlobalSettingsV8) == 588, "GlobalSettingsV8 must match schema 8 storage size.");

struct GlobalSettingsV9 {
    GlobalSettingsV8 base;
    uint8_t closedLoopPitchTargetMode;
};

static_assert(sizeof(GlobalSettingsV9) == 592, "GlobalSettingsV9 must match schema 9 storage size.");

// Schema 10 is byte-for-byte the prefix of schema 11. Its former direction byte is retained as a reserved byte in the new layout.
struct GlobalSettingsV10 {
    uint8_t bytes[608];
};

static_assert(sizeof(GlobalSettingsV10) == 608, "GlobalSettingsV10 must match schema 10 storage size.");

struct GlobalSettingsV11 {
    uint8_t bytes[616];
};

static_assert(sizeof(GlobalSettingsV11) == 616, "GlobalSettingsV11 must match schema 11 storage size.");
#pragma pack(pop)

void copySpeedFromV9(const SpeedSettingsV9& source, SpeedSettings& target) {
    target.frequency = source.frequency;
    target.minFrequency = source.minFrequency;
    target.maxFrequency = source.maxFrequency;
    for (int i = 0; i < 4; i++) {
        target.phaseOffset[i] = source.phaseOffset[i];
        target.channelAmplitude[i] = 100;
    }
    target.softStartDuration = source.softStartDuration;
    target.reducedAmplitude = source.reducedAmplitude;
    target.amplitudeDelay = source.amplitudeDelay;
    target.startupKick = source.startupKick;
    target.startupKickDuration = source.startupKickDuration;
    target.startupKickRampDuration = source.startupKickRampDuration;
    target.filterType = source.filterType;
    target.iirAlpha = source.iirAlpha;
    target.firProfile = source.firProfile;
}

void setOutputArchitectureMigrationDefaults(GlobalSettings& target) {
    // Older schemas stored raw phase tunes only. Custom topology preserves those tunes exactly instead of imposing a new nominal map.
    target.motorTopology = MOTOR_TOPOLOGY_CUSTOM;
    target.outputConfigReserved = 0;
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    target.activeBrakingAllowed = false;
#else
    target.activeBrakingAllowed = true;
#endif
}

void setOutputTuningDefaults(GlobalSettings& target) {
    target.phaseSlewDegreesPerSecond = 180.0f;
    target.gainSlewPercentPerSecond = 50.0f;
}

void copyGlobalClosedLoopTuningToSpeed(const GlobalSettings& source, ClosedLoopSpeedTuning& target) {
    // Schema 6/7 stored a single global tuning block. Newer schemas keep one tuning block per speed, so migration copies the global values to all three.
    target.deadbandRpm = source.closedLoopDeadbandRpm;
    target.lockToleranceRpm = source.closedLoopLockToleranceRpm;
    target.kp = source.closedLoopKp;
    target.ki = source.closedLoopKi;
    target.kd = source.closedLoopKd;
    target.integralLimitHz = source.closedLoopIntegralLimitHz;
    target.correctionLimitHz = source.closedLoopCorrectionLimitHz;
    target.slewLimitHzPerSec = source.closedLoopSlewLimitHzPerSec;
    target.rampKp = source.closedLoopRampKp;
    target.rampCorrectionLimitHz = source.closedLoopRampCorrectionLimitHz;
    target.lockTimeMs = source.closedLoopLockTimeMs;
}

void copyGlobalClosedLoopTuningToSpeeds(GlobalSettings& data) {
    for (int i = 0; i < 3; i++) {
        copyGlobalClosedLoopTuningToSpeed(data, data.closedLoopTuning[i]);
    }
}

void setClosedLoopAdvancedDefaults(GlobalSettings& data) {
    // Defaults for fields added after the initial closed-loop feature. Keeping these separate makes migrations easier to read.
    data.closedLoopControlMode = CLOSED_LOOP_CONTROL_CORRECT;
    data.closedLoopRequireSignalBeforeEngage = true;
    data.closedLoopRequireNearTargetBeforeEngage = false;
    data.closedLoopEngageToleranceRpm = 2.0f;
    data.closedLoopRampMode = CLOSED_LOOP_RAMP_DISABLED;
    data.closedLoopRampKp = 0.02f;
    data.closedLoopRampCorrectionLimitHz = 1.0f;
    data.closedLoopPitchSlewRpmPerSec = 0.0f;
    data.closedLoopPitchResetThresholdRpm = 0.25f;
    data.closedLoopPitchTargetMode = CLOSED_LOOP_PITCH_TARGET_FOLLOW;
    data.closedLoopSaturationTimeMs = 5000;
    data.closedLoopSaturationAction = CLOSED_LOOP_FAULT_WARN;
    data.closedLoopPlausibilityMinRpm = 1.0f;
    data.closedLoopPlausibilityMaxRpm = 120.0f;
    data.closedLoopPlausibilityAction = CLOSED_LOOP_FAULT_WARN;
    data.closedLoopLockTimeoutMs = 30000;
    data.closedLoopLockTimeoutAction = CLOSED_LOOP_FAULT_WARN;
    data.closedLoopAmpRecoveryMode = CLOSED_LOOP_AMP_RECOVERY_OFF;
    data.closedLoopAmpRecoveryDelayMs = 2000;
}

void setClosedLoopDefaults(GlobalSettings& data) {
    // Conservative defaults: feature disabled, correction mode prepared, and sensor assumptions safe for a single-pulse tachometer.
    data.closedLoopEnabled = false;
    data.closedLoopControlMode = CLOSED_LOOP_CONTROL_CORRECT;
    data.closedLoopSensorMode = CLOSED_LOOP_SENSOR_PULSE;
    data.closedLoopTargetRpm[SPEED_33] = 33.3333f;
    data.closedLoopTargetRpm[SPEED_45] = 45.0f;
    data.closedLoopTargetRpm[SPEED_78] = 78.0f;
    data.closedLoopCountsPerRev = 1;
    data.closedLoopPulseEdge = CLOSED_LOOP_EDGE_RISING;
    data.closedLoopQuadratureMode = CLOSED_LOOP_QUAD_X4;
    data.closedLoopReverseDirection = false;
    data.closedLoopDirectionFaultAction = CLOSED_LOOP_FAULT_WARN;
    data.closedLoopDebounceUs = 100;
    data.closedLoopTimeoutMs = 3000;
    data.closedLoopEngageDelayMs = 2000;
    data.closedLoopUpdateIntervalMs = 100;
    data.closedLoopFilterAlpha = 0.25f;
    data.closedLoopDeadbandRpm = 0.02f;
    data.closedLoopLockToleranceRpm = 0.05f;
    data.closedLoopLockTimeMs = 3000;
    data.closedLoopKp = 0.05f;
    data.closedLoopKi = 0.01f;
    data.closedLoopKd = 0.0f;
    data.closedLoopIntegralLimitHz = 1.0f;
    data.closedLoopCorrectionLimitHz = 3.0f;
    data.closedLoopSlewLimitHzPerSec = 0.5f;
    data.closedLoopDropoutAction = CLOSED_LOOP_DROPOUT_OPEN_LOOP;
    setClosedLoopAdvancedDefaults(data);
    copyGlobalClosedLoopTuningToSpeeds(data);
}

void applyMotorPreset(const GlobalSettings& source, GlobalSettings& target) {
    // Presets are motor tunes, not whole-device snapshots. Preserve runtime, names, display/UI choices, relay wiring, boot policy, and the live speed selection.
    target.phaseMode = source.phaseMode;
    target.motorTopology = source.motorTopology;
    target.phaseSlewDegreesPerSecond = source.phaseSlewDegreesPerSecond;
    target.gainSlewPercentPerSecond = source.gainSlewPercentPerSecond;
    target.maxAmplitude = source.maxAmplitude;
    target.softStartCurve = source.softStartCurve;
    target.smoothSwitching = source.smoothSwitching;
    target.switchRampDuration = source.switchRampDuration;
    target.brakeMode = source.brakeMode;
    target.brakeDuration = source.brakeDuration;
    target.brakePulseGap = source.brakePulseGap;
    target.brakeStartFreq = source.brakeStartFreq;
    target.brakeStopFreq = source.brakeStopFreq;
    target.softStopCutoff = source.softStopCutoff;
    target.rampType = source.rampType;
    target.vfBlend = source.vfBlend;
    target.vfLowFreq = source.vfLowFreq;
    target.vfLowLevel = source.vfLowLevel;
    target.vfMidFreq = source.vfMidFreq;
    target.vfMidLevel = source.vfMidLevel;
    target.vfBaseFreq = source.vfBaseFreq;

    for (int i = 0; i < 3; i++) {
        target.speeds[i] = source.speeds[i];
        target.closedLoopTargetRpm[i] = source.closedLoopTargetRpm[i];
        target.closedLoopTuning[i] = source.closedLoopTuning[i];
    }

    target.closedLoopEnabled = source.closedLoopEnabled;
    target.closedLoopControlMode = source.closedLoopControlMode;
    target.closedLoopSensorMode = source.closedLoopSensorMode;
    target.closedLoopCountsPerRev = source.closedLoopCountsPerRev;
    target.closedLoopPulseEdge = source.closedLoopPulseEdge;
    target.closedLoopQuadratureMode = source.closedLoopQuadratureMode;
    target.closedLoopReverseDirection = source.closedLoopReverseDirection;
    target.closedLoopDirectionFaultAction = source.closedLoopDirectionFaultAction;
    target.closedLoopDebounceUs = source.closedLoopDebounceUs;
    target.closedLoopTimeoutMs = source.closedLoopTimeoutMs;
    target.closedLoopEngageDelayMs = source.closedLoopEngageDelayMs;
    target.closedLoopUpdateIntervalMs = source.closedLoopUpdateIntervalMs;
    target.closedLoopFilterAlpha = source.closedLoopFilterAlpha;
    target.closedLoopDeadbandRpm = source.closedLoopDeadbandRpm;
    target.closedLoopLockToleranceRpm = source.closedLoopLockToleranceRpm;
    target.closedLoopLockTimeMs = source.closedLoopLockTimeMs;
    target.closedLoopKp = source.closedLoopKp;
    target.closedLoopKi = source.closedLoopKi;
    target.closedLoopKd = source.closedLoopKd;
    target.closedLoopIntegralLimitHz = source.closedLoopIntegralLimitHz;
    target.closedLoopCorrectionLimitHz = source.closedLoopCorrectionLimitHz;
    target.closedLoopSlewLimitHzPerSec = source.closedLoopSlewLimitHzPerSec;
    target.closedLoopDropoutAction = source.closedLoopDropoutAction;
    target.closedLoopRequireSignalBeforeEngage = source.closedLoopRequireSignalBeforeEngage;
    target.closedLoopRequireNearTargetBeforeEngage = source.closedLoopRequireNearTargetBeforeEngage;
    target.closedLoopEngageToleranceRpm = source.closedLoopEngageToleranceRpm;
    target.closedLoopRampMode = source.closedLoopRampMode;
    target.closedLoopRampKp = source.closedLoopRampKp;
    target.closedLoopRampCorrectionLimitHz = source.closedLoopRampCorrectionLimitHz;
    target.closedLoopPitchSlewRpmPerSec = source.closedLoopPitchSlewRpmPerSec;
    target.closedLoopPitchResetThresholdRpm = source.closedLoopPitchResetThresholdRpm;
    target.closedLoopPitchTargetMode = source.closedLoopPitchTargetMode;
    target.closedLoopSaturationTimeMs = source.closedLoopSaturationTimeMs;
    target.closedLoopSaturationAction = source.closedLoopSaturationAction;
    target.closedLoopPlausibilityMinRpm = source.closedLoopPlausibilityMinRpm;
    target.closedLoopPlausibilityMaxRpm = source.closedLoopPlausibilityMaxRpm;
    target.closedLoopPlausibilityAction = source.closedLoopPlausibilityAction;
    target.closedLoopLockTimeoutMs = source.closedLoopLockTimeoutMs;
    target.closedLoopLockTimeoutAction = source.closedLoopLockTimeoutAction;
    target.closedLoopAmpRecoveryMode = source.closedLoopAmpRecoveryMode;
    target.closedLoopAmpRecoveryDelayMs = source.closedLoopAmpRecoveryDelayMs;
}

void copyFromV5(const GlobalSettingsV5& source, GlobalSettings& target) {
    target.schemaVersion = SETTINGS_SCHEMA_VERSION;
    target.phaseMode = source.phaseMode;
    target.maxAmplitude = source.maxAmplitude;
    target.softStartCurve = source.softStartCurve;
    target.smoothSwitching = source.smoothSwitching;
    target.switchRampDuration = source.switchRampDuration;
    target.brakeMode = source.brakeMode;
    target.brakeDuration = source.brakeDuration;
    target.brakePulseGap = source.brakePulseGap;
    target.brakeStartFreq = source.brakeStartFreq;
    target.brakeStopFreq = source.brakeStopFreq;
    target.softStopCutoff = source.softStopCutoff;
    target.relayActiveHigh = source.relayActiveHigh;
    target.muteRelayLinkStandby = source.muteRelayLinkStandby;
    target.muteRelayLinkStartStop = source.muteRelayLinkStartStop;
    target.powerOnRelayDelay = source.powerOnRelayDelay;
    target.displayBrightness = source.displayBrightness;
    target.displaySleepDelay = source.displaySleepDelay;
    target.screensaverEnabled = source.screensaverEnabled;
    target.autoDimDelay = source.autoDimDelay;
    target.showRuntime = source.showRuntime;
    target.errorDisplayEnabled = source.errorDisplayEnabled;
    target.errorDisplayDuration = source.errorDisplayDuration;
    target.autoStandbyDelay = source.autoStandbyDelay;
    target.autoStart = source.autoStart;
    target.autoBoot = source.autoBoot;
    target.pitchResetOnStop = source.pitchResetOnStop;
    for (int i = 0; i < 3; i++) {
        copySpeedFromV9(source.speeds[i], target.speeds[i]);
    }
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        strncpy(target.presetNames[i], source.presetNames[i], 16);
        target.presetNames[i][16] = 0;
    }
    target.totalRuntime = source.totalRuntime;
    target.reverseEncoder = source.reverseEncoder;
    target.pitchStepSize = source.pitchStepSize;
    target.rampType = source.rampType;
    target.screensaverMode = source.screensaverMode;
    target.enable78rpm = source.enable78rpm;
    target.vfBlend = source.freqDependentAmplitude;
    target.vfLowFreq = source.vfLowFreq;
    target.vfLowLevel = source.vfLowBoost;
    target.vfMidFreq = source.vfMidFreq;
    target.vfMidLevel = source.vfMidBoost;
    target.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
    target.bootSpeed = source.bootSpeed;
    target.currentSpeed = source.currentSpeed;
    target.ampTempWarnC = source.ampTempWarnC;
    target.ampTempShutdownC = source.ampTempShutdownC;
    target.showCpuDashboard = source.showCpuDashboard;
    target.showMemoryDashboard = source.showMemoryDashboard;
    target.showFlashDashboard = source.showFlashDashboard;
    setClosedLoopDefaults(target);
    setOutputArchitectureMigrationDefaults(target);
    setOutputTuningDefaults(target);
}

void copyFromV6(const GlobalSettingsV6& source, GlobalSettings& target) {
    target.schemaVersion = SETTINGS_SCHEMA_VERSION;
    target.phaseMode = source.phaseMode;
    target.maxAmplitude = source.maxAmplitude;
    target.softStartCurve = source.softStartCurve;
    target.smoothSwitching = source.smoothSwitching;
    target.switchRampDuration = source.switchRampDuration;
    target.brakeMode = source.brakeMode;
    target.brakeDuration = source.brakeDuration;
    target.brakePulseGap = source.brakePulseGap;
    target.brakeStartFreq = source.brakeStartFreq;
    target.brakeStopFreq = source.brakeStopFreq;
    target.softStopCutoff = source.softStopCutoff;
    target.relayActiveHigh = source.relayActiveHigh;
    target.muteRelayLinkStandby = source.muteRelayLinkStandby;
    target.muteRelayLinkStartStop = source.muteRelayLinkStartStop;
    target.powerOnRelayDelay = source.powerOnRelayDelay;
    target.displayBrightness = source.displayBrightness;
    target.displaySleepDelay = source.displaySleepDelay;
    target.screensaverEnabled = source.screensaverEnabled;
    target.autoDimDelay = source.autoDimDelay;
    target.showRuntime = source.showRuntime;
    target.errorDisplayEnabled = source.errorDisplayEnabled;
    target.errorDisplayDuration = source.errorDisplayDuration;
    target.autoStandbyDelay = source.autoStandbyDelay;
    target.autoStart = source.autoStart;
    target.autoBoot = source.autoBoot;
    target.pitchResetOnStop = source.pitchResetOnStop;
    for (int i = 0; i < 3; i++) {
        copySpeedFromV9(source.speeds[i], target.speeds[i]);
    }
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        strncpy(target.presetNames[i], source.presetNames[i], 16);
        target.presetNames[i][16] = 0;
    }
    target.totalRuntime = source.totalRuntime;
    target.reverseEncoder = source.reverseEncoder;
    target.pitchStepSize = source.pitchStepSize;
    target.rampType = source.rampType;
    target.screensaverMode = source.screensaverMode;
    target.enable78rpm = source.enable78rpm;
    target.vfBlend = source.freqDependentAmplitude;
    target.vfLowFreq = source.vfLowFreq;
    target.vfLowLevel = source.vfLowBoost;
    target.vfMidFreq = source.vfMidFreq;
    target.vfMidLevel = source.vfMidBoost;
    target.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
    target.bootSpeed = source.bootSpeed;
    target.currentSpeed = source.currentSpeed;
    target.ampTempWarnC = source.ampTempWarnC;
    target.ampTempShutdownC = source.ampTempShutdownC;
    target.showCpuDashboard = source.showCpuDashboard;
    target.showMemoryDashboard = source.showMemoryDashboard;
    target.showFlashDashboard = source.showFlashDashboard;
    target.closedLoopEnabled = source.closedLoopEnabled;
    target.closedLoopSensorMode = source.closedLoopSensorMode;
    for (int i = 0; i < 3; i++) {
        target.closedLoopTargetRpm[i] = source.closedLoopTargetRpm[i];
    }
    target.closedLoopCountsPerRev = source.closedLoopCountsPerRev;
    target.closedLoopPulseEdge = source.closedLoopPulseEdge;
    target.closedLoopQuadratureMode = source.closedLoopQuadratureMode;
    target.closedLoopReverseDirection = source.closedLoopReverseDirection;
    target.closedLoopDirectionFaultAction = source.closedLoopDirectionFaultAction;
    target.closedLoopDebounceUs = source.closedLoopDebounceUs;
    target.closedLoopTimeoutMs = source.closedLoopTimeoutMs;
    target.closedLoopEngageDelayMs = source.closedLoopEngageDelayMs;
    target.closedLoopUpdateIntervalMs = source.closedLoopUpdateIntervalMs;
    target.closedLoopFilterAlpha = source.closedLoopFilterAlpha;
    target.closedLoopDeadbandRpm = source.closedLoopDeadbandRpm;
    target.closedLoopLockToleranceRpm = source.closedLoopLockToleranceRpm;
    target.closedLoopLockTimeMs = source.closedLoopLockTimeMs;
    target.closedLoopKp = source.closedLoopKp;
    target.closedLoopKi = source.closedLoopKi;
    target.closedLoopKd = source.closedLoopKd;
    target.closedLoopIntegralLimitHz = source.closedLoopIntegralLimitHz;
    target.closedLoopCorrectionLimitHz = source.closedLoopCorrectionLimitHz;
    target.closedLoopSlewLimitHzPerSec = source.closedLoopSlewLimitHzPerSec;
    target.closedLoopDropoutAction = source.closedLoopDropoutAction;
    setClosedLoopAdvancedDefaults(target);
    copyGlobalClosedLoopTuningToSpeeds(target);
    setOutputArchitectureMigrationDefaults(target);
    setOutputTuningDefaults(target);
}

void copyFromV7(const GlobalSettingsV7& source, GlobalSettings& target) {
    target.schemaVersion = SETTINGS_SCHEMA_VERSION;
    target.phaseMode = source.phaseMode;
    target.maxAmplitude = source.maxAmplitude;
    target.softStartCurve = source.softStartCurve;
    target.smoothSwitching = source.smoothSwitching;
    target.switchRampDuration = source.switchRampDuration;
    target.brakeMode = source.brakeMode;
    target.brakeDuration = source.brakeDuration;
    target.brakePulseGap = source.brakePulseGap;
    target.brakeStartFreq = source.brakeStartFreq;
    target.brakeStopFreq = source.brakeStopFreq;
    target.softStopCutoff = source.softStopCutoff;
    target.relayActiveHigh = source.relayActiveHigh;
    target.muteRelayLinkStandby = source.muteRelayLinkStandby;
    target.muteRelayLinkStartStop = source.muteRelayLinkStartStop;
    target.powerOnRelayDelay = source.powerOnRelayDelay;
    target.displayBrightness = source.displayBrightness;
    target.displaySleepDelay = source.displaySleepDelay;
    target.screensaverEnabled = source.screensaverEnabled;
    target.autoDimDelay = source.autoDimDelay;
    target.showRuntime = source.showRuntime;
    target.errorDisplayEnabled = source.errorDisplayEnabled;
    target.errorDisplayDuration = source.errorDisplayDuration;
    target.autoStandbyDelay = source.autoStandbyDelay;
    target.autoStart = source.autoStart;
    target.autoBoot = source.autoBoot;
    target.pitchResetOnStop = source.pitchResetOnStop;
    for (int i = 0; i < 3; i++) {
        copySpeedFromV9(source.speeds[i], target.speeds[i]);
    }
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        strncpy(target.presetNames[i], source.presetNames[i], 16);
        target.presetNames[i][16] = 0;
    }
    target.totalRuntime = source.totalRuntime;
    target.reverseEncoder = source.reverseEncoder;
    target.pitchStepSize = source.pitchStepSize;
    target.rampType = source.rampType;
    target.screensaverMode = source.screensaverMode;
    target.enable78rpm = source.enable78rpm;
    target.vfBlend = source.freqDependentAmplitude;
    target.vfLowFreq = source.vfLowFreq;
    target.vfLowLevel = source.vfLowBoost;
    target.vfMidFreq = source.vfMidFreq;
    target.vfMidLevel = source.vfMidBoost;
    target.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
    target.bootSpeed = source.bootSpeed;
    target.currentSpeed = source.currentSpeed;
    target.ampTempWarnC = source.ampTempWarnC;
    target.ampTempShutdownC = source.ampTempShutdownC;
    target.showCpuDashboard = source.showCpuDashboard;
    target.showMemoryDashboard = source.showMemoryDashboard;
    target.showFlashDashboard = source.showFlashDashboard;
    target.closedLoopEnabled = source.closedLoopEnabled;
    target.closedLoopControlMode = source.closedLoopControlMode;
    target.closedLoopSensorMode = source.closedLoopSensorMode;
    for (int i = 0; i < 3; i++) {
        target.closedLoopTargetRpm[i] = source.closedLoopTargetRpm[i];
    }
    target.closedLoopCountsPerRev = source.closedLoopCountsPerRev;
    target.closedLoopPulseEdge = source.closedLoopPulseEdge;
    target.closedLoopQuadratureMode = source.closedLoopQuadratureMode;
    target.closedLoopReverseDirection = source.closedLoopReverseDirection;
    target.closedLoopDirectionFaultAction = source.closedLoopDirectionFaultAction;
    target.closedLoopDebounceUs = source.closedLoopDebounceUs;
    target.closedLoopTimeoutMs = source.closedLoopTimeoutMs;
    target.closedLoopEngageDelayMs = source.closedLoopEngageDelayMs;
    target.closedLoopUpdateIntervalMs = source.closedLoopUpdateIntervalMs;
    target.closedLoopFilterAlpha = source.closedLoopFilterAlpha;
    target.closedLoopDeadbandRpm = source.closedLoopDeadbandRpm;
    target.closedLoopLockToleranceRpm = source.closedLoopLockToleranceRpm;
    target.closedLoopLockTimeMs = source.closedLoopLockTimeMs;
    target.closedLoopKp = source.closedLoopKp;
    target.closedLoopKi = source.closedLoopKi;
    target.closedLoopKd = source.closedLoopKd;
    target.closedLoopIntegralLimitHz = source.closedLoopIntegralLimitHz;
    target.closedLoopCorrectionLimitHz = source.closedLoopCorrectionLimitHz;
    target.closedLoopSlewLimitHzPerSec = source.closedLoopSlewLimitHzPerSec;
    target.closedLoopDropoutAction = source.closedLoopDropoutAction;
    target.closedLoopRequireSignalBeforeEngage = source.closedLoopRequireSignalBeforeEngage;
    target.closedLoopRequireNearTargetBeforeEngage = source.closedLoopRequireNearTargetBeforeEngage;
    target.closedLoopEngageToleranceRpm = source.closedLoopEngageToleranceRpm;
    target.closedLoopRampMode = source.closedLoopRampMode;
    target.closedLoopRampKp = source.closedLoopRampKp;
    target.closedLoopRampCorrectionLimitHz = source.closedLoopRampCorrectionLimitHz;
    target.closedLoopPitchSlewRpmPerSec = source.closedLoopPitchSlewRpmPerSec;
    target.closedLoopPitchResetThresholdRpm = source.closedLoopPitchResetThresholdRpm;
    target.closedLoopSaturationTimeMs = source.closedLoopSaturationTimeMs;
    target.closedLoopSaturationAction = source.closedLoopSaturationAction;
    target.closedLoopPlausibilityMinRpm = source.closedLoopPlausibilityMinRpm;
    target.closedLoopPlausibilityMaxRpm = source.closedLoopPlausibilityMaxRpm;
    target.closedLoopPlausibilityAction = source.closedLoopPlausibilityAction;
    target.closedLoopLockTimeoutMs = source.closedLoopLockTimeoutMs;
    target.closedLoopLockTimeoutAction = source.closedLoopLockTimeoutAction;
    target.closedLoopAmpRecoveryMode = source.closedLoopAmpRecoveryMode;
    target.closedLoopAmpRecoveryDelayMs = source.closedLoopAmpRecoveryDelayMs;
    copyGlobalClosedLoopTuningToSpeeds(target);
    setOutputArchitectureMigrationDefaults(target);
    setOutputTuningDefaults(target);
}

void copyFromV8(const GlobalSettingsV8& source, GlobalSettings& target) {
    copyFromV7(source.base, target);
    for (int i = 0; i < 3; i++) {
        target.closedLoopTuning[i] = source.closedLoopTuning[i];
    }
    target.closedLoopPitchTargetMode = CLOSED_LOOP_PITCH_TARGET_FOLLOW;
}

void copyFromV9(const GlobalSettingsV9& source, GlobalSettings& target) {
    copyFromV8(source.base, target);
    target.closedLoopPitchTargetMode = source.closedLoopPitchTargetMode;
}

void copyFromV10(const GlobalSettingsV10& source, GlobalSettings& target) {
    memset(&target, 0, sizeof(target));
    memcpy(&target, source.bytes, sizeof(source.bytes));
    target.schemaVersion = SETTINGS_SCHEMA_VERSION;
    target.outputConfigReserved = 0;
    setOutputTuningDefaults(target);
    target.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
}

void copyFromV11(const GlobalSettingsV11& source, GlobalSettings& target) {
    memset(&target, 0, sizeof(target));
    memcpy(&target, source.bytes, sizeof(source.bytes));
    target.schemaVersion = SETTINGS_SCHEMA_VERSION;
    target.outputConfigReserved = 0;
    target.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
}

uint32_t settingsCrc32(const uint8_t* data, size_t length) {
    // Standard reflected CRC-32 used only for flash payload integrity. It is not a security primitive; it just rejects partial writes and corrupted blobs.
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

bool makeSidecarPath(const char* path, const char* suffix, char* out, size_t outSize) {
    int written = snprintf(out, outSize, "%s%s", path, suffix);
    return written > 0 && written < (int)outSize;
}

bool readSettingsBlob(const char* path, uint32_t magic, GlobalSettings& target, bool* migrated = nullptr) {
    if (migrated) *migrated = false;

    File f = LittleFS.open(path, "r");
    if (!f) return false;

    if (f.size() < sizeof(SettingsFileHeader)) {
        f.close();
        return false;
    }

    SettingsFileHeader header;
    if (f.read((uint8_t*)&header, sizeof(header)) != sizeof(header)) {
        f.close();
        return false;
    }

    if (header.magic != magic ||
        header.formatVersion != SETTINGS_FILE_FORMAT_VERSION ||
        header.headerSize != sizeof(SettingsFileHeader) ||
        f.size() != (size_t)header.headerSize + header.payloadSize) {
        f.close();
        return false;
    }

    // Fast path for current schema: read the payload, verify CRC, then copy.
    if (header.schemaVersion == SETTINGS_SCHEMA_VERSION &&
        header.payloadSize == GLOBAL_SETTINGS_STORAGE_SIZE) {
        GlobalSettings candidate;
        memset(&candidate, 0, sizeof(candidate));
        if (f.read((uint8_t*)&candidate, GLOBAL_SETTINGS_STORAGE_SIZE) != GLOBAL_SETTINGS_STORAGE_SIZE) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&candidate, GLOBAL_SETTINGS_STORAGE_SIZE);
        if (crc != header.crc32) return false;

        target = candidate;
        return true;
    }

    // Older schemas are migrated only after their exact payload size and CRC match, avoiding partial or ambiguous upgrades.
    if (header.schemaVersion == 5 && header.payloadSize == sizeof(GlobalSettingsV5)) {
        GlobalSettingsV5 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&legacy, sizeof(legacy));
        if (crc != header.crc32) return false;

        copyFromV5(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 6 && header.payloadSize == sizeof(GlobalSettingsV6)) {
        GlobalSettingsV6 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&legacy, sizeof(legacy));
        if (crc != header.crc32) return false;

        copyFromV6(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 7 && header.payloadSize == sizeof(GlobalSettingsV7)) {
        GlobalSettingsV7 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&legacy, sizeof(legacy));
        if (crc != header.crc32) return false;

        copyFromV7(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 8 && header.payloadSize == sizeof(GlobalSettingsV8)) {
        GlobalSettingsV8 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&legacy, sizeof(legacy));
        if (crc != header.crc32) return false;

        copyFromV8(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 9 && header.payloadSize == sizeof(GlobalSettingsV9)) {
        GlobalSettingsV9 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();

        uint32_t crc = settingsCrc32((const uint8_t*)&legacy, sizeof(legacy));
        if (crc != header.crc32) return false;

        copyFromV9(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 10 && header.payloadSize == sizeof(GlobalSettingsV10)) {
        GlobalSettingsV10 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();
        if (settingsCrc32((const uint8_t*)&legacy, sizeof(legacy)) != header.crc32) return false;
        copyFromV10(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    if (header.schemaVersion == 11 && header.payloadSize == sizeof(GlobalSettingsV11)) {
        GlobalSettingsV11 legacy;
        if (f.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) {
            f.close();
            return false;
        }
        f.close();
        if (settingsCrc32((const uint8_t*)&legacy, sizeof(legacy)) != header.crc32) return false;
        copyFromV11(legacy, target);
        if (migrated) *migrated = true;
        return true;
    }

    f.close();
    return false;
}

bool loadSettingsBlob(const char* path, uint32_t magic, GlobalSettings& target, bool* migrated = nullptr) {
    if (readSettingsBlob(path, magic, target, migrated)) return true;

    // A failed primary read falls back to the previous backup created by the atomic writer below.
    char backupPath[40];
    if (makeSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) {
        return readSettingsBlob(backupPath, magic, target, migrated);
    }
    return false;
}

bool writeSettingsBlob(const char* path, uint32_t magic, const GlobalSettings& source) {
    char tmpPath[40];
    char backupPath[40];
    if (!makeSidecarPath(path, ".tmp", tmpPath, sizeof(tmpPath))) return false;
    if (!makeSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) return false;

    // Write to a temp file first, then rename the current file to .bak, then promote the temp file. This avoids leaving no readable copy after reset.
    LittleFS.remove(tmpPath);

    File f = LittleFS.open(tmpPath, "w");
    if (!f) return false;

    SettingsFileHeader header;
    header.magic = magic;
    header.formatVersion = SETTINGS_FILE_FORMAT_VERSION;
    header.headerSize = sizeof(SettingsFileHeader);
    header.schemaVersion = SETTINGS_SCHEMA_VERSION;
    header.payloadSize = GLOBAL_SETTINGS_STORAGE_SIZE;
    header.crc32 = settingsCrc32((const uint8_t*)&source, GLOBAL_SETTINGS_STORAGE_SIZE);

    bool ok = f.write((const uint8_t*)&header, sizeof(header)) == sizeof(header);
    ok = ok && f.write((const uint8_t*)&source, GLOBAL_SETTINGS_STORAGE_SIZE) == GLOBAL_SETTINGS_STORAGE_SIZE;
    f.close();

    if (!ok) {
        LittleFS.remove(tmpPath);
        return false;
    }

    LittleFS.remove(backupPath);
    bool hadOriginal = LittleFS.exists(path);
    if (hadOriginal && !LittleFS.rename(path, backupPath)) {
        LittleFS.remove(tmpPath);
        return false;
    }

    if (!LittleFS.rename(tmpPath, path)) {
        if (hadOriginal) LittleFS.rename(backupPath, path);
        LittleFS.remove(tmpPath);
        return false;
    }

    if (hadOriginal) LittleFS.remove(backupPath);
    return true;
}

uint8_t readBootMarkerState() {
    File f = LittleFS.open(SETTINGS_BOOT_MARKER_FILE, "r");
    if (!f) return SETTINGS_BOOT_NONE;

    SettingsBootMarker marker;
    bool ok = f.read((uint8_t*)&marker, sizeof(marker)) == sizeof(marker);
    f.close();

    if (!ok ||
        marker.magic != SETTINGS_BOOT_MARKER_MAGIC ||
        marker.version != SETTINGS_BOOT_MARKER_VERSION ||
        marker.state > SETTINGS_BOOT_BOOTING) {
        LittleFS.remove(SETTINGS_BOOT_MARKER_FILE);
        return SETTINGS_BOOT_NONE;
    }

    // CRC is calculated with crc32 cleared so random marker files are ignored.
    uint32_t storedCrc = marker.crc32;
    marker.crc32 = 0;
    if (settingsCrc32((const uint8_t*)&marker, sizeof(marker)) != storedCrc) {
        LittleFS.remove(SETTINGS_BOOT_MARKER_FILE);
        return SETTINGS_BOOT_NONE;
    }

    return marker.state;
}

bool writeBootMarkerState(uint8_t state) {
    if (state == SETTINGS_BOOT_NONE) {
        LittleFS.remove(SETTINGS_BOOT_MARKER_FILE);
        return true;
    }

    SettingsBootMarker marker;
    memset(&marker, 0, sizeof(marker));
    marker.magic = SETTINGS_BOOT_MARKER_MAGIC;
    marker.version = SETTINGS_BOOT_MARKER_VERSION;
    marker.state = state;
    marker.crc32 = settingsCrc32((const uint8_t*)&marker, sizeof(marker));

    File f = LittleFS.open(SETTINGS_BOOT_MARKER_FILE, "w");
    if (!f) return false;
    bool ok = f.write((const uint8_t*)&marker, sizeof(marker)) == sizeof(marker);
    f.close();
    if (!ok) LittleFS.remove(SETTINGS_BOOT_MARKER_FILE);
    return ok;
}
}

// --- Helper Functions for Preset Management ---

bool Settings::loadFromSlot(uint8_t slot, GlobalSettings& target) {
    if (safeModeActive) return false;
    // Slot indexes are zero-based internally even though user-facing labels are one-based in serial/web text.
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    return loadSettingsBlob(path, PRESET_FILE_MAGIC, target);
}

bool Settings::saveToSlot(uint8_t slot, const GlobalSettings& source) {
    if (safeModeActive) return false;
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    return writeSettingsBlob(path, PRESET_FILE_MAGIC, source);
}

bool Settings::renamePreset(uint8_t slot, const char* name) {
    if (safeModeActive) return false;
    if (slot >= MAX_PRESET_SLOTS || !name) return false;
    char previous[17];
    memcpy(previous, _data.presetNames[slot], sizeof(previous));
    // Names are fixed width in GlobalSettings; truncate rather than resizing the persisted struct.
    strncpy(_data.presetNames[slot], name, 16);
    _data.presetNames[slot][16] = 0; // Ensure null termination
    if (save()) return true;
    memcpy(_data.presetNames[slot], previous, sizeof(previous));
    return false;
}

void Settings::resetSessionRuntime() {
    _sessionRuntime = 0;
    _lastRuntimeUpdate = millis();
}

bool Settings::resetTotalRuntime() {
    if (safeModeActive) return false;
    uint32_t previous = _data.totalRuntime;
    _data.totalRuntime = 0;
    if (save()) return true;
    _data.totalRuntime = previous;
    return false;
}

Settings::Settings() {
    _sessionRuntime = 0;
    _lastRuntimeUpdate = 0;
    _rollbackApplied = false;
    _bootCandidateActive = false;
}

void Settings::begin() {
    // Check Hardware Safe Mode Flag before any formatting or writes.
    extern bool safeModeActive;
    if (safeModeActive) {
        if (SERIAL_MONITOR_ENABLE) Serial.println("HARDWARE SAFE MODE ENGAGED. Bypassing Flash Load.");
        if (!LittleFS.begin()) {
            if (SERIAL_MONITOR_ENABLE) Serial.println("LittleFS unavailable in Safe Mode. Leaving flash untouched.");
        }

        setDefaults(); // Load baseline safe settings to RAM only.

        // Name the preset list to make Safe Mode visible in menus and status.
        strncpy(_data.presetNames[0], "SAFE MODE", 16);
        _data.presetNames[0][16] = 0;
        for(int i=1; i<5; i++) {
            strncpy(_data.presetNames[i], "LOCKED", 16);
            _data.presetNames[i][16] = 0;
        }

        _lastRuntimeUpdate = millis();
        return;
    }

    // Mount LittleFS only after Safe Mode has had a chance to bypass normal flash load. Do not format on a single mount failure; a transient flash issue
    // should not erase settings, presets, logs, and network configuration.
    if (!LittleFS.begin()) {
        if (SERIAL_MONITOR_ENABLE) Serial.println("LittleFS Mount Failed. Using RAM defaults and preserving flash.");
        setDefaults();
        _lastRuntimeUpdate = millis();
        return;
    }

    handlePendingRollback();
    load(); // Normal operation

    _lastRuntimeUpdate = millis();
}

void Settings::handlePendingRollback() {
    // If a protected settings save reached BOOTING but never got confirmed, the previous known-good file is restored before normal load.
    uint8_t markerState = readBootMarkerState();
    if (markerState == SETTINGS_BOOT_BOOTING) {
        GlobalSettings knownGood;
        if (loadSettingsBlob(SETTINGS_KNOWN_GOOD_FILE, SETTINGS_FILE_MAGIC, knownGood)) {
            if (writeSettingsBlob(_filename, SETTINGS_FILE_MAGIC, knownGood)) {
                _rollbackApplied = true;
                errorHandler.logEvent(ERR_SETTINGS_ROLLBACK, "Unconfirmed settings boot rolled back to known-good settings");
            } else {
                errorHandler.logEvent(ERR_SETTINGS_ROLLBACK, "Rollback requested but settings restore failed");
            }
        } else {
            errorHandler.logEvent(ERR_SETTINGS_ROLLBACK, "Rollback requested but no known-good settings were available");
        }
        writeBootMarkerState(SETTINGS_BOOT_NONE);
        return;
    }

    if (markerState == SETTINGS_BOOT_PENDING) {
        // First boot after a protected save: mark it as BOOTING. Core 0 will clear this only after Core 1 has filled a waveform buffer.
        _bootCandidateActive = true;
        writeBootMarkerState(SETTINGS_BOOT_BOOTING);
    }
}

void Settings::load() {
    if (safeModeActive) {
        setDefaults();
        _lastRuntimeUpdate = millis();
        return;
    }
    GlobalSettings loaded;
    bool migrated = false;
    if (loadSettingsBlob(_filename, SETTINGS_FILE_MAGIC, loaded, &migrated)) {
        _data = loaded;
        if (SERIAL_MONITOR_ENABLE) Serial.println("Settings loaded.");
        validate();
        if (migrated) {
            // Save migrated data back in the current schema so future boots take the fast path.
            if (SERIAL_MONITOR_ENABLE) Serial.println("Settings migrated.");
            save(false);
        }
        return;
    }

    if (SERIAL_MONITOR_ENABLE) Serial.println("Settings not found or invalid. Using defaults.");
    resetDefaults();
}

bool Settings::save(bool verbose, bool rollbackProtected) {
    if (safeModeActive) {
        if (verbose) Serial.println("Safe Mode is read-only; settings were not saved.");
        return false;
    }
    if (rollbackProtected) {
        // Preserve a known-good copy before writing the candidate settings file.
        GlobalSettings knownGood;
        bool knownGoodSaved;
        if (loadSettingsBlob(_filename, SETTINGS_FILE_MAGIC, knownGood)) {
            knownGoodSaved = writeSettingsBlob(SETTINGS_KNOWN_GOOD_FILE, SETTINGS_FILE_MAGIC, knownGood);
        } else {
            knownGoodSaved = writeSettingsBlob(SETTINGS_KNOWN_GOOD_FILE, SETTINGS_FILE_MAGIC, _data);
        }
        if (!knownGoodSaved) {
            if (verbose) Serial.println("Failed to preserve known-good settings.");
            return false;
        }
    }

    if (writeSettingsBlob(_filename, SETTINGS_FILE_MAGIC, _data)) {
        if (rollbackProtected) {
            _bootCandidateActive = true;
            if (!writeBootMarkerState(SETTINGS_BOOT_PENDING)) {
                if (verbose) Serial.println("Settings saved without rollback marker.");
                return false;
            }
        }
        if (verbose) Serial.println("Settings saved.");
        return true;
    } else {
        if (verbose) Serial.println("Failed to save settings.");
        return false;
    }
}

void Settings::markBootSuccessful() {
    if (safeModeActive) return;
    // Called after Core 1 fills a waveform buffer. At that point the current settings have survived enough of boot to become the new known-good copy.
    uint8_t markerState = readBootMarkerState();
    if (markerState == SETTINGS_BOOT_NONE && !_bootCandidateActive) return;

    if (writeSettingsBlob(SETTINGS_KNOWN_GOOD_FILE, SETTINGS_FILE_MAGIC, _data)) {
        writeBootMarkerState(SETTINGS_BOOT_NONE);
        _bootCandidateActive = false;
        if (SERIAL_MONITOR_ENABLE) Serial.println("Settings boot confirmed.");
    }
}

bool Settings::resetDefaults() {
    if (safeModeActive) {
        setDefaults();
        return false;
    }
    setDefaults();
    writeBootMarkerState(SETTINGS_BOOT_NONE);
    return save();
}

bool Settings::factoryReset() {
    if (safeModeActive) return false;
    if (currentMotorState == STATE_STARTING || currentMotorState == STATE_RUNNING || currentMotorState == STATE_STOPPING) {
        return false;
    }
    // Format filesystem to clear all settings, presets, logs, and boot markers.
    if (!LittleFS.format()) return false;
    _rollbackApplied = false;
    _bootCandidateActive = false;
    return resetDefaults();
}

SpeedSettings& Settings::getCurrentSpeedSettings() {
    // currentSpeed is validated on load and after imports before this is used.
    return _data.speeds[_data.currentSpeed];
}

ClosedLoopSpeedTuning& Settings::getCurrentClosedLoopTuning() {
    return getClosedLoopTuning((SpeedMode)_data.currentSpeed);
}

ClosedLoopSpeedTuning& Settings::getClosedLoopTuning(SpeedMode speed) {
    uint8_t index = (uint8_t)speed;
    if (index > SPEED_78) index = SPEED_33;
    return _data.closedLoopTuning[index];
}

void Settings::normalize() {
    validate();
}

void Settings::validate() {
    // Current storage is strict: any schema mismatch that was not handled by a migration path resets to defaults rather than guessing field layout.
    if (_data.schemaVersion != SETTINGS_SCHEMA_VERSION) {
        if (SERIAL_MONITOR_ENABLE) Serial.println("Schema mismatch. Resetting defaults.");
        resetDefaults();
    }

    _data.brakeDuration = finiteOr(_data.brakeDuration, 2.0f);
    _data.brakePulseGap = finiteOr(_data.brakePulseGap, 0.5f);
    _data.brakeStartFreq = finiteOr(_data.brakeStartFreq, 50.0f);
    _data.brakeStopFreq = finiteOr(_data.brakeStopFreq, 0.0f);
    _data.softStopCutoff = finiteOr(_data.softStopCutoff, 5.0f);
    _data.pitchStepSize = finiteOr(_data.pitchStepSize, 0.1f);
    _data.vfLowFreq = finiteOr(_data.vfLowFreq, 5.0f);
    _data.vfMidFreq = finiteOr(_data.vfMidFreq, 25.0f);
    _data.vfBaseFreq = finiteOr(_data.vfBaseFreq, DEFAULT_VF_BASE_FREQUENCY_HZ);
    _data.ampTempWarnC = finiteOr(_data.ampTempWarnC, AMP_TEMP_WARN_C);
    _data.ampTempShutdownC = finiteOr(_data.ampTempShutdownC, AMP_TEMP_SHUTDOWN_C);
    const float defaultClosedLoopTargetRpm[3] = {33.3333f, 45.0f, 78.0f};
    for (uint8_t i = 0; i < 3; i++) {
        _data.closedLoopTargetRpm[i] = finiteOr(_data.closedLoopTargetRpm[i], defaultClosedLoopTargetRpm[i]);
    }
    _data.closedLoopFilterAlpha = finiteOr(_data.closedLoopFilterAlpha, 0.25f);
    _data.closedLoopDeadbandRpm = finiteOr(_data.closedLoopDeadbandRpm, 0.02f);
    _data.closedLoopLockToleranceRpm = finiteOr(_data.closedLoopLockToleranceRpm, 0.05f);
    _data.closedLoopKp = finiteOr(_data.closedLoopKp, 0.05f);
    _data.closedLoopKi = finiteOr(_data.closedLoopKi, 0.01f);
    _data.closedLoopKd = finiteOr(_data.closedLoopKd, 0.0f);
    _data.closedLoopIntegralLimitHz = finiteOr(_data.closedLoopIntegralLimitHz, 1.0f);
    _data.closedLoopCorrectionLimitHz = finiteOr(_data.closedLoopCorrectionLimitHz, 3.0f);
    _data.closedLoopSlewLimitHzPerSec = finiteOr(_data.closedLoopSlewLimitHzPerSec, 0.5f);
    _data.closedLoopEngageToleranceRpm = finiteOr(_data.closedLoopEngageToleranceRpm, 2.0f);
    _data.closedLoopRampKp = finiteOr(_data.closedLoopRampKp, 0.02f);
    _data.closedLoopRampCorrectionLimitHz = finiteOr(_data.closedLoopRampCorrectionLimitHz, 1.0f);
    _data.closedLoopPitchSlewRpmPerSec = finiteOr(_data.closedLoopPitchSlewRpmPerSec, 0.0f);
    _data.closedLoopPitchResetThresholdRpm = finiteOr(_data.closedLoopPitchResetThresholdRpm, 0.25f);
    _data.closedLoopPlausibilityMinRpm = finiteOr(_data.closedLoopPlausibilityMinRpm, 1.0f);
    _data.closedLoopPlausibilityMaxRpm = finiteOr(_data.closedLoopPlausibilityMaxRpm, 120.0f);

    // Enforce global ranges before per-speed ranges so dependent calculations see sane values.
    if (_data.phaseMode < PHASE_1 || _data.phaseMode > MAX_PHASE_MODE) _data.phaseMode = DEFAULT_PHASE_MODE;
    if (_data.motorTopology > MOTOR_TOPOLOGY_THREE_PHASE) _data.motorTopology = MOTOR_TOPOLOGY_CUSTOM;
    _data.outputConfigReserved = 0;
    _data.phaseSlewDegreesPerSecond = finiteOr(_data.phaseSlewDegreesPerSecond, 180.0f);
    _data.gainSlewPercentPerSecond = finiteOr(_data.gainSlewPercentPerSecond, 50.0f);
    if (_data.phaseSlewDegreesPerSecond < 0.0f) _data.phaseSlewDegreesPerSecond = 0.0f;
    if (_data.phaseSlewDegreesPerSecond > 3600.0f) _data.phaseSlewDegreesPerSecond = 3600.0f;
    if (_data.gainSlewPercentPerSecond < 0.0f) _data.gainSlewPercentPerSecond = 0.0f;
    if (_data.gainSlewPercentPerSecond > 1000.0f) _data.gainSlewPercentPerSecond = 1000.0f;
    if (_data.currentSpeed < SPEED_33 || _data.currentSpeed > SPEED_78) _data.currentSpeed = SPEED_33;
    if (_data.maxAmplitude > 100) _data.maxAmplitude = 100;
    if (_data.softStartCurve > 2) _data.softStartCurve = 0;
    if (_data.switchRampDuration < 1) _data.switchRampDuration = 1;
    if (_data.switchRampDuration > 5) _data.switchRampDuration = 5;
    if (_data.brakeMode > BRAKE_SOFT_STOP) _data.brakeMode = BRAKE_OFF;
    if (_data.brakeDuration < 0.0) _data.brakeDuration = 0.0;
    if (_data.brakeDuration > 10.0) _data.brakeDuration = 10.0;
    if (_data.brakePulseGap < 0.1) _data.brakePulseGap = 0.1;
    if (_data.brakePulseGap > 2.0) _data.brakePulseGap = 2.0;
    if (_data.brakeStartFreq < 10.0f) _data.brakeStartFreq = 10.0f;
    if (_data.brakeStartFreq > 200.0f) _data.brakeStartFreq = 200.0f;
    if (_data.brakeStopFreq < 0.0f) _data.brakeStopFreq = 0.0f;
    if (_data.brakeStopFreq > 50.0f) _data.brakeStopFreq = 50.0f;
    if (_data.softStopCutoff < 0.0) _data.softStopCutoff = 0.0;
    if (_data.softStopCutoff > 50.0) _data.softStopCutoff = 50.0;
    if (_data.powerOnRelayDelay > 10) _data.powerOnRelayDelay = 10;
    if (_data.autoDimDelay > 60) _data.autoDimDelay = 60;
    if (_data.autoStandbyDelay > 60) _data.autoStandbyDelay = 60;
    if (_data.displaySleepDelay > 6) _data.displaySleepDelay = 0;
    if (_data.errorDisplayDuration < 1) _data.errorDisplayDuration = 1;
    if (_data.errorDisplayDuration > 60) _data.errorDisplayDuration = 60;
    if (_data.pitchStepSize < 0.01) _data.pitchStepSize = 0.01;
    if (_data.pitchStepSize > 1.0) _data.pitchStepSize = 1.0;
    if (_data.rampType > RAMP_SCURVE) _data.rampType = RAMP_LINEAR;
    if (_data.screensaverMode > SAVER_LISSAJOUS) _data.screensaverMode = SAVER_BOUNCE;
    if (_data.vfBlend > 100) _data.vfBlend = 100;
    if (_data.vfLowLevel > 100) _data.vfLowLevel = 100;
    if (_data.vfMidLevel > 100) _data.vfMidLevel = 100;
    if (_data.vfLowFreq < 0.0) _data.vfLowFreq = 0.0;
    if (_data.vfLowFreq > 50.0) _data.vfLowFreq = 50.0;
    if (_data.vfMidFreq < 0.0) _data.vfMidFreq = 0.0;
    if (_data.vfMidFreq > 100.0) _data.vfMidFreq = 100.0;
    if (_data.vfMidFreq <= _data.vfLowFreq) _data.vfMidFreq = _data.vfLowFreq + 0.1f;
    if (_data.vfBaseFreq < MIN_OUTPUT_FREQUENCY_HZ) _data.vfBaseFreq = MIN_OUTPUT_FREQUENCY_HZ;
    if (_data.vfBaseFreq > MAX_OUTPUT_FREQUENCY_HZ) _data.vfBaseFreq = MAX_OUTPUT_FREQUENCY_HZ;
    if (_data.vfBaseFreq <= _data.vfMidFreq) _data.vfBaseFreq = _data.vfMidFreq + 0.1f;
    if (_data.vfMidLevel < _data.vfLowLevel) _data.vfMidLevel = _data.vfLowLevel;
    if (_data.bootSpeed > 3) _data.bootSpeed = 3;
    if (_data.ampTempShutdownC < (AMP_TEMP_MIN_C + AMP_TEMP_MIN_SHUTDOWN_MARGIN_C)) {
        _data.ampTempShutdownC = AMP_TEMP_MIN_C + AMP_TEMP_MIN_SHUTDOWN_MARGIN_C;
    }
    if (_data.ampTempShutdownC > AMP_TEMP_MAX_C) _data.ampTempShutdownC = AMP_TEMP_MAX_C;
    if (_data.ampTempWarnC < AMP_TEMP_MIN_C) _data.ampTempWarnC = AMP_TEMP_MIN_C;
    if (_data.ampTempWarnC > (AMP_TEMP_MAX_C - AMP_TEMP_MIN_SHUTDOWN_MARGIN_C)) {
        _data.ampTempWarnC = AMP_TEMP_MAX_C - AMP_TEMP_MIN_SHUTDOWN_MARGIN_C;
    }
    if (_data.ampTempWarnC > (_data.ampTempShutdownC - AMP_TEMP_MIN_SHUTDOWN_MARGIN_C)) {
        _data.ampTempWarnC = _data.ampTempShutdownC - AMP_TEMP_MIN_SHUTDOWN_MARGIN_C;
    }
    if (_data.closedLoopControlMode > CLOSED_LOOP_CONTROL_CORRECT) {
        _data.closedLoopControlMode = CLOSED_LOOP_CONTROL_CORRECT;
    }
    if (_data.closedLoopSensorMode > CLOSED_LOOP_SENSOR_QUADRATURE) {
        _data.closedLoopSensorMode = CLOSED_LOOP_SENSOR_PULSE;
    }
    for (uint8_t i = 0; i < 3; i++) {
        if (_data.closedLoopTargetRpm[i] < 1.0f) _data.closedLoopTargetRpm[i] = 1.0f;
        if (_data.closedLoopTargetRpm[i] > 120.0f) _data.closedLoopTargetRpm[i] = 120.0f;
    }
    if (_data.closedLoopCountsPerRev < 1) _data.closedLoopCountsPerRev = 1;
    if (_data.closedLoopCountsPerRev > 20000) _data.closedLoopCountsPerRev = 20000;
    if (_data.closedLoopPulseEdge > CLOSED_LOOP_EDGE_CHANGE) {
        _data.closedLoopPulseEdge = CLOSED_LOOP_EDGE_RISING;
    }
    if (_data.closedLoopQuadratureMode > CLOSED_LOOP_QUAD_X4) {
        _data.closedLoopQuadratureMode = CLOSED_LOOP_QUAD_X4;
    }
    if (_data.closedLoopDirectionFaultAction > CLOSED_LOOP_FAULT_STOP) {
        _data.closedLoopDirectionFaultAction = CLOSED_LOOP_FAULT_WARN;
    }
    if (_data.closedLoopDebounceUs > 50000) _data.closedLoopDebounceUs = 50000;
    if (_data.closedLoopTimeoutMs < 100) _data.closedLoopTimeoutMs = 100;
    if (_data.closedLoopTimeoutMs > 10000) _data.closedLoopTimeoutMs = 10000;
    if (_data.closedLoopEngageDelayMs > 30000) _data.closedLoopEngageDelayMs = 30000;
    if (_data.closedLoopUpdateIntervalMs < 20) _data.closedLoopUpdateIntervalMs = 20;
    if (_data.closedLoopUpdateIntervalMs > 1000) _data.closedLoopUpdateIntervalMs = 1000;
    if (_data.closedLoopFilterAlpha < 0.01f) _data.closedLoopFilterAlpha = 0.01f;
    if (_data.closedLoopFilterAlpha > 1.0f) _data.closedLoopFilterAlpha = 1.0f;
    if (_data.closedLoopDeadbandRpm < 0.0f) _data.closedLoopDeadbandRpm = 0.0f;
    if (_data.closedLoopDeadbandRpm > 5.0f) _data.closedLoopDeadbandRpm = 5.0f;
    if (_data.closedLoopLockToleranceRpm < 0.01f) _data.closedLoopLockToleranceRpm = 0.01f;
    if (_data.closedLoopLockToleranceRpm > 5.0f) _data.closedLoopLockToleranceRpm = 5.0f;
    if (_data.closedLoopLockTimeMs > 30000) _data.closedLoopLockTimeMs = 30000;
    if (_data.closedLoopKp < 0.0f) _data.closedLoopKp = 0.0f;
    if (_data.closedLoopKp > 20.0f) _data.closedLoopKp = 20.0f;
    if (_data.closedLoopKi < 0.0f) _data.closedLoopKi = 0.0f;
    if (_data.closedLoopKi > 20.0f) _data.closedLoopKi = 20.0f;
    if (_data.closedLoopKd < 0.0f) _data.closedLoopKd = 0.0f;
    if (_data.closedLoopKd > 20.0f) _data.closedLoopKd = 20.0f;
    if (_data.closedLoopIntegralLimitHz < 0.0f) _data.closedLoopIntegralLimitHz = 0.0f;
    if (_data.closedLoopIntegralLimitHz > 50.0f) _data.closedLoopIntegralLimitHz = 50.0f;
    if (_data.closedLoopCorrectionLimitHz < 0.0f) _data.closedLoopCorrectionLimitHz = 0.0f;
    if (_data.closedLoopCorrectionLimitHz > 100.0f) _data.closedLoopCorrectionLimitHz = 100.0f;
    if (_data.closedLoopSlewLimitHzPerSec < 0.0f) _data.closedLoopSlewLimitHzPerSec = 0.0f;
    if (_data.closedLoopSlewLimitHzPerSec > 100.0f) _data.closedLoopSlewLimitHzPerSec = 100.0f;
    if (_data.closedLoopDropoutAction > CLOSED_LOOP_DROPOUT_STOP) {
        _data.closedLoopDropoutAction = CLOSED_LOOP_DROPOUT_OPEN_LOOP;
    }
    if (_data.closedLoopEngageToleranceRpm < 0.01f) _data.closedLoopEngageToleranceRpm = 0.01f;
    if (_data.closedLoopEngageToleranceRpm > 20.0f) _data.closedLoopEngageToleranceRpm = 20.0f;
    if (_data.closedLoopRampMode > CLOSED_LOOP_RAMP_TRACK) {
        _data.closedLoopRampMode = CLOSED_LOOP_RAMP_DISABLED;
    }
    if (_data.closedLoopRampKp < 0.0f) _data.closedLoopRampKp = 0.0f;
    if (_data.closedLoopRampKp > 5.0f) _data.closedLoopRampKp = 5.0f;
    if (_data.closedLoopRampCorrectionLimitHz < 0.0f) _data.closedLoopRampCorrectionLimitHz = 0.0f;
    if (_data.closedLoopRampCorrectionLimitHz > 20.0f) _data.closedLoopRampCorrectionLimitHz = 20.0f;
    if (_data.closedLoopPitchSlewRpmPerSec < 0.0f) _data.closedLoopPitchSlewRpmPerSec = 0.0f;
    if (_data.closedLoopPitchSlewRpmPerSec > 200.0f) _data.closedLoopPitchSlewRpmPerSec = 200.0f;
    if (_data.closedLoopPitchResetThresholdRpm < 0.0f) _data.closedLoopPitchResetThresholdRpm = 0.0f;
    if (_data.closedLoopPitchResetThresholdRpm > 20.0f) _data.closedLoopPitchResetThresholdRpm = 20.0f;
    if (_data.closedLoopPitchTargetMode > CLOSED_LOOP_PITCH_TARGET_FOLLOW) {
        _data.closedLoopPitchTargetMode = CLOSED_LOOP_PITCH_TARGET_FOLLOW;
    }
    if (_data.closedLoopSaturationTimeMs > 60000) _data.closedLoopSaturationTimeMs = 60000;
    if (_data.closedLoopSaturationAction > CLOSED_LOOP_FAULT_STOP) {
        _data.closedLoopSaturationAction = CLOSED_LOOP_FAULT_WARN;
    }
    if (_data.closedLoopPlausibilityMinRpm < 0.0f) _data.closedLoopPlausibilityMinRpm = 0.0f;
    if (_data.closedLoopPlausibilityMinRpm > 120.0f) _data.closedLoopPlausibilityMinRpm = 120.0f;
    if (_data.closedLoopPlausibilityMaxRpm < 1.0f) _data.closedLoopPlausibilityMaxRpm = 1.0f;
    if (_data.closedLoopPlausibilityMaxRpm > 200.0f) _data.closedLoopPlausibilityMaxRpm = 200.0f;
    if (_data.closedLoopPlausibilityMinRpm > _data.closedLoopPlausibilityMaxRpm) {
        float temp = _data.closedLoopPlausibilityMinRpm;
        _data.closedLoopPlausibilityMinRpm = _data.closedLoopPlausibilityMaxRpm;
        _data.closedLoopPlausibilityMaxRpm = temp;
    }
    if (_data.closedLoopPlausibilityAction > CLOSED_LOOP_FAULT_STOP) {
        _data.closedLoopPlausibilityAction = CLOSED_LOOP_FAULT_WARN;
    }
    if (_data.closedLoopLockTimeoutMs > 60000) _data.closedLoopLockTimeoutMs = 60000;
    if (_data.closedLoopLockTimeoutAction > CLOSED_LOOP_FAULT_STOP) {
        _data.closedLoopLockTimeoutAction = CLOSED_LOOP_FAULT_WARN;
    }
    if (_data.closedLoopAmpRecoveryMode > CLOSED_LOOP_AMP_RECOVERY_RESTORE) {
        _data.closedLoopAmpRecoveryMode = CLOSED_LOOP_AMP_RECOVERY_OFF;
    }
    if (_data.closedLoopAmpRecoveryDelayMs > 30000) _data.closedLoopAmpRecoveryDelayMs = 30000;

    for (uint8_t i = 0; i < 3; i++) {
        ClosedLoopSpeedTuning& t = _data.closedLoopTuning[i];
        t.deadbandRpm = finiteOr(t.deadbandRpm, 0.02f);
        t.lockToleranceRpm = finiteOr(t.lockToleranceRpm, 0.05f);
        t.kp = finiteOr(t.kp, 0.05f);
        t.ki = finiteOr(t.ki, 0.01f);
        t.kd = finiteOr(t.kd, 0.0f);
        t.integralLimitHz = finiteOr(t.integralLimitHz, 1.0f);
        t.correctionLimitHz = finiteOr(t.correctionLimitHz, 3.0f);
        t.slewLimitHzPerSec = finiteOr(t.slewLimitHzPerSec, 0.5f);
        t.rampKp = finiteOr(t.rampKp, 0.02f);
        t.rampCorrectionLimitHz = finiteOr(t.rampCorrectionLimitHz, 1.0f);
        if (t.deadbandRpm < 0.0f) t.deadbandRpm = 0.0f;
        if (t.deadbandRpm > 5.0f) t.deadbandRpm = 5.0f;
        if (t.lockToleranceRpm < 0.01f) t.lockToleranceRpm = 0.01f;
        if (t.lockToleranceRpm > 5.0f) t.lockToleranceRpm = 5.0f;
        if (t.lockTimeMs > 30000) t.lockTimeMs = 30000;
        if (t.kp < 0.0f) t.kp = 0.0f;
        if (t.kp > 20.0f) t.kp = 20.0f;
        if (t.ki < 0.0f) t.ki = 0.0f;
        if (t.ki > 20.0f) t.ki = 20.0f;
        if (t.kd < 0.0f) t.kd = 0.0f;
        if (t.kd > 20.0f) t.kd = 20.0f;
        if (t.integralLimitHz < 0.0f) t.integralLimitHz = 0.0f;
        if (t.integralLimitHz > 50.0f) t.integralLimitHz = 50.0f;
        if (t.correctionLimitHz < 0.0f) t.correctionLimitHz = 0.0f;
        if (t.correctionLimitHz > 100.0f) t.correctionLimitHz = 100.0f;
        if (t.slewLimitHzPerSec < 0.0f) t.slewLimitHzPerSec = 0.0f;
        if (t.slewLimitHzPerSec > 100.0f) t.slewLimitHzPerSec = 100.0f;
        if (t.rampKp < 0.0f) t.rampKp = 0.0f;
        if (t.rampKp > 5.0f) t.rampKp = 5.0f;
        if (t.rampCorrectionLimitHz < 0.0f) t.rampCorrectionLimitHz = 0.0f;
        if (t.rampCorrectionLimitHz > 20.0f) t.rampCorrectionLimitHz = 20.0f;
    }

    // Validate per-speed settings. The min/max/frequency order matters: first clamp each value to hardware limits, then fix relationships between them.
    const float defaultSpeedFrequency[3] = {25.07f, 33.85f, 58.66f};
    const float defaultSpeedMinFrequency[3] = {20.0f, 30.0f, 50.0f};
    const float defaultSpeedMaxFrequency[3] = {30.0f, 40.0f, 70.0f};
    const float defaultSoftStartDuration[3] = {1.0f, 1.0f, 1.5f};
    const float defaultPhaseOffset[4] = {0.0f, 120.0f, 240.0f, 270.0f};
    for(int i=0; i<3; i++) {
        _data.speeds[i].frequency = finiteOr(_data.speeds[i].frequency, defaultSpeedFrequency[i]);
        _data.speeds[i].minFrequency = finiteOr(_data.speeds[i].minFrequency, defaultSpeedMinFrequency[i]);
        _data.speeds[i].maxFrequency = finiteOr(_data.speeds[i].maxFrequency, defaultSpeedMaxFrequency[i]);
        _data.speeds[i].softStartDuration = finiteOr(_data.speeds[i].softStartDuration, defaultSoftStartDuration[i]);
        _data.speeds[i].startupKickRampDuration = finiteOr(_data.speeds[i].startupKickRampDuration, 1.0f);
        _data.speeds[i].iirAlpha = finiteOr(_data.speeds[i].iirAlpha, 0.5f);
        for (int p = 0; p < 4; p++) {
            _data.speeds[i].phaseOffset[p] = finiteOr(_data.speeds[i].phaseOffset[p], defaultPhaseOffset[p]);
            if (_data.speeds[i].channelAmplitude[p] < 50) _data.speeds[i].channelAmplitude[p] = 50;
            if (_data.speeds[i].channelAmplitude[p] > 150) _data.speeds[i].channelAmplitude[p] = 150;
        }

        if (_data.speeds[i].minFrequency < MIN_OUTPUT_FREQUENCY_HZ) _data.speeds[i].minFrequency = MIN_OUTPUT_FREQUENCY_HZ;
        if (_data.speeds[i].minFrequency > MAX_OUTPUT_FREQUENCY_HZ) _data.speeds[i].minFrequency = MAX_OUTPUT_FREQUENCY_HZ;
        if (_data.speeds[i].maxFrequency < MIN_OUTPUT_FREQUENCY_HZ) _data.speeds[i].maxFrequency = MIN_OUTPUT_FREQUENCY_HZ;
        if (_data.speeds[i].maxFrequency > MAX_OUTPUT_FREQUENCY_HZ) _data.speeds[i].maxFrequency = MAX_OUTPUT_FREQUENCY_HZ;

        if (_data.speeds[i].minFrequency > _data.speeds[i].maxFrequency) {
            float temp = _data.speeds[i].minFrequency;
            _data.speeds[i].minFrequency = _data.speeds[i].maxFrequency;
            _data.speeds[i].maxFrequency = temp;
        }
        if (_data.speeds[i].frequency < MIN_OUTPUT_FREQUENCY_HZ) _data.speeds[i].frequency = MIN_OUTPUT_FREQUENCY_HZ;
        if (_data.speeds[i].frequency > MAX_OUTPUT_FREQUENCY_HZ) _data.speeds[i].frequency = MAX_OUTPUT_FREQUENCY_HZ;
        if (_data.speeds[i].frequency < _data.speeds[i].minFrequency) _data.speeds[i].frequency = _data.speeds[i].minFrequency;
        if (_data.speeds[i].frequency > _data.speeds[i].maxFrequency) _data.speeds[i].frequency = _data.speeds[i].maxFrequency;

        if (_data.speeds[i].softStartDuration < 0.0) _data.speeds[i].softStartDuration = 0.0;
        if (_data.speeds[i].softStartDuration > 10.0) _data.speeds[i].softStartDuration = 10.0;
        if (_data.speeds[i].reducedAmplitude < 10) _data.speeds[i].reducedAmplitude = 10;
        if (_data.speeds[i].reducedAmplitude > 100) _data.speeds[i].reducedAmplitude = 100;
        if (_data.speeds[i].amplitudeDelay > 60) _data.speeds[i].amplitudeDelay = 60;
        if (_data.speeds[i].startupKick < 1) _data.speeds[i].startupKick = 1;
        if (_data.speeds[i].startupKick > 4) _data.speeds[i].startupKick = 4;
        if (_data.speeds[i].startupKickDuration > 15) _data.speeds[i].startupKickDuration = 15;
        if (_data.speeds[i].startupKickRampDuration < 0.0) _data.speeds[i].startupKickRampDuration = 0.0;
        if (_data.speeds[i].startupKickRampDuration > 15.0) _data.speeds[i].startupKickRampDuration = 15.0;
        if (_data.speeds[i].filterType > FILTER_FIR) _data.speeds[i].filterType = FILTER_NONE;
        if (_data.speeds[i].iirAlpha < 0.01) _data.speeds[i].iirAlpha = 0.01;
        if (_data.speeds[i].iirAlpha > 0.99) _data.speeds[i].iirAlpha = 0.99;
        if (_data.speeds[i].firProfile > FIR_AGGRESSIVE) _data.speeds[i].firProfile = FIR_GENTLE;

        // Normalize phase offsets to one full cycle so DDS phase conversion does not depend on callers wrapping values before saving.
        for(int p=0; p<4; p++) {
            while(_data.speeds[i].phaseOffset[p] >= 360.0) _data.speeds[i].phaseOffset[p] -= 360.0;
            while(_data.speeds[i].phaseOffset[p] < 0.0) _data.speeds[i].phaseOffset[p] += 360.0;
        }
    }
}

void Settings::setDefaults() {
    // Clear every field and padding byte before assigning semantic defaults so
    // factory reset and Safe Mode cannot retain bytes from a previous payload.
    memset(&_data, 0, sizeof(_data));
    _data.schemaVersion = SETTINGS_SCHEMA_VERSION;
    // Initialize preset names in the live settings directory.
    for(int i=0; i<5; i++) {
        snprintf(_data.presetNames[i], 17, "Preset %d", i+1);
    }

    _data.phaseMode = (PhaseMode)DEFAULT_PHASE_MODE;
    _data.motorTopology = DEFAULT_MOTOR_TOPOLOGY;
    _data.outputConfigReserved = 0;
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
    _data.activeBrakingAllowed = false;
#else
    _data.activeBrakingAllowed = true;
#endif
    _data.maxAmplitude = 68;
    _data.softStartCurve = 0; // Linear
    _data.smoothSwitching = true;
    _data.switchRampDuration = 2;

    _data.brakeMode = BRAKE_RAMP;
    _data.brakeDuration = 2.0;
    _data.brakePulseGap = 0.5;
    _data.brakeStartFreq = 50.0;
    _data.brakeStopFreq = 0.0;
    _data.softStopCutoff = 5.0;

    _data.relayActiveHigh = true;
    _data.muteRelayLinkStandby = true;
    _data.muteRelayLinkStartStop = true;
    _data.powerOnRelayDelay = 2;

    _data.autoStandbyDelay = 0;
    _data.autoDimDelay = 0;
    _data.autoStart = false;
    _data.autoBoot = false;
    _data.displaySleepDelay = 0; // Minutes
    _data.screensaverEnabled = true;

    _data.errorDisplayEnabled = true;
    _data.errorDisplayDuration = 10;

    _data.showRuntime = true;
    _data.showCpuDashboard = true;
    _data.showMemoryDashboard = true;
    _data.showFlashDashboard = true;
    _data.pitchResetOnStop = true;
    _data.currentSpeed = (SpeedMode)DEFAULT_SPEED_INDEX;

    // --- 33.3 RPM Defaults (primary 12-pole motor, 7.52 belt ratio) ---
    _data.speeds[SPEED_33].frequency = 25.07;
    _data.speeds[SPEED_33].minFrequency = 20.0;
    _data.speeds[SPEED_33].maxFrequency = 30.0;
    _data.speeds[SPEED_33].softStartDuration = 1.0;
    _data.speeds[SPEED_33].reducedAmplitude = 35;
    _data.speeds[SPEED_33].amplitudeDelay = 5;
    _data.speeds[SPEED_33].startupKick = 1;
    _data.speeds[SPEED_33].startupKickDuration = 1;
    _data.speeds[SPEED_33].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_33].filterType = FILTER_NONE;
    _data.speeds[SPEED_33].iirAlpha = 0.5;
    _data.speeds[SPEED_33].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_33].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_33].phaseOffset[1] = 120.0;
    _data.speeds[SPEED_33].phaseOffset[2] = 240.0;
    _data.speeds[SPEED_33].phaseOffset[3] = 270.0;

    // --- 45 RPM Defaults (primary 12-pole motor, 7.52 belt ratio) ---
    _data.speeds[SPEED_45].frequency = 33.85;
    _data.speeds[SPEED_45].minFrequency = 30.0;
    _data.speeds[SPEED_45].maxFrequency = 40.0;
    _data.speeds[SPEED_45].softStartDuration = 1.0;
    _data.speeds[SPEED_45].reducedAmplitude = 35;
    _data.speeds[SPEED_45].amplitudeDelay = 5;
    _data.speeds[SPEED_45].startupKick = 1;
    _data.speeds[SPEED_45].startupKickDuration = 1;
    _data.speeds[SPEED_45].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_45].filterType = FILTER_NONE;
    _data.speeds[SPEED_45].iirAlpha = 0.5;
    _data.speeds[SPEED_45].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_45].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_45].phaseOffset[1] = 120.0;
    _data.speeds[SPEED_45].phaseOffset[2] = 240.0;
    _data.speeds[SPEED_45].phaseOffset[3] = 270.0;

    // --- 78 RPM Defaults (primary 12-pole motor, 7.52 belt ratio) ---
    _data.speeds[SPEED_78].frequency = 58.66;
    _data.speeds[SPEED_78].minFrequency = 50.0;
    _data.speeds[SPEED_78].maxFrequency = 70.0;
    _data.speeds[SPEED_78].softStartDuration = 1.5;
    _data.speeds[SPEED_78].reducedAmplitude = 35;
    _data.speeds[SPEED_78].amplitudeDelay = 5;
    _data.speeds[SPEED_78].startupKick = 1;
    _data.speeds[SPEED_78].startupKickDuration = 1;
    _data.speeds[SPEED_78].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_78].filterType = FILTER_NONE;
    _data.speeds[SPEED_78].iirAlpha = 0.5;
    _data.speeds[SPEED_78].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_78].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_78].phaseOffset[1] = 120.0;
    _data.speeds[SPEED_78].phaseOffset[2] = 240.0;
    _data.speeds[SPEED_78].phaseOffset[3] = 270.0;

    for (int speed = 0; speed < 3; speed++) {
        for (int channel = 0; channel < 4; channel++) {
            _data.speeds[speed].channelAmplitude[channel] = 100;
        }
    }

    _data.enable78rpm = true;
    _data.totalRuntime = 0;

    _data.displayBrightness = 255; // Max brightness
    _data.reverseEncoder = false;
    _data.pitchStepSize = 0.1;
    _data.rampType = 1; // Default to S-Curve
    _data.screensaverMode = 0; // Default to Bounce
    _data.vfBlend = 0; // Disabled by default
    _data.vfLowFreq = 5.0;
    _data.vfLowLevel = 100;
    _data.vfMidFreq = 25.0;
    _data.vfMidLevel = 100;
    _data.vfBaseFreq = DEFAULT_VF_BASE_FREQUENCY_HZ;
    _data.bootSpeed = 3; // Default to Last Used
    _data.ampTempWarnC = AMP_TEMP_WARN_C;
    _data.ampTempShutdownC = AMP_TEMP_SHUTDOWN_C;
    setClosedLoopDefaults(_data);
    setOutputTuningDefaults(_data);
}

bool Settings::loadPreset(uint8_t slot) {
    if (safeModeActive) return false;
    if (slot >= MAX_PRESET_SLOTS) return false;

    // Loading a preset replaces the active global settings in RAM. The caller is responsible for applying the new motor/waveform settings.
    GlobalSettings temp;
    if (loadFromSlot(slot, temp)) {
        applyMotorPreset(temp, _data);
        validate();
        return true;
    }
    return false;
}

bool Settings::savePreset(uint8_t slot) {
    if (safeModeActive) return false;
    if (slot >= MAX_PRESET_SLOTS) return false;
    // The compatible blob remains a GlobalSettings payload; loadPreset() deliberately applies only its motor-tuning subset.
    return saveToSlot(slot, _data);
}

bool Settings::resetPreset(uint8_t slot) {
    if (safeModeActive) return false;
    if (slot >= MAX_PRESET_SLOTS) return false;

    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    if (LittleFS.exists(path) && !LittleFS.remove(path)) return false;
    char sidecar[40];
    if (makeSidecarPath(path, ".bak", sidecar, sizeof(sidecar))) LittleFS.remove(sidecar);
    if (makeSidecarPath(path, ".tmp", sidecar, sizeof(sidecar))) LittleFS.remove(sidecar);
    // Reset name to default in the preset directory.
    snprintf(_data.presetNames[slot], 17, "Preset %d", slot + 1);
    return save();
}

const char* Settings::getPresetName(uint8_t slot) {
    if (slot >= MAX_PRESET_SLOTS) return "Invalid";
    // Ensure null termination in case a legacy or imported blob filled all bytes.
    _data.presetNames[slot][16] = 0;
    return _data.presetNames[slot];
}

bool Settings::saveToSlot(uint8_t slot) {
    // Legacy wrapper
    return savePreset(slot);
}

void Settings::loadFromSlot(uint8_t slot) {
    // Legacy wrapper
    loadPreset(slot);
}

// --- Serialization ---
bool Settings::exportPresetToJSON(uint8_t slot, String& outStr) {
    if (slot >= MAX_PRESET_SLOTS) return false;

    GlobalSettings target;
    // Export describes the requested stored preset; an empty or invalid slot
    // must not silently export the unrelated live configuration.
    if (!loadFromSlot(slot, target)) return false;

    // ArduinoJson 7 allocates JsonDocument dynamically. The short keys keep the preset export compact enough for serial and web workflows.
    JsonDocument doc;

    // Global motor parameters use compact keys so preset export remains practical over Serial Monitor.
    doc["pm"] = target.phaseMode;
    doc["top"] = target.motorTopology;
    doc["phSlew"] = target.phaseSlewDegreesPerSecond;
    doc["gainSlew"] = target.gainSlewPercentPerSecond;
    doc["maxAmp"] = target.maxAmplitude;
    doc["ssCurve"] = target.softStartCurve;
    doc["vfBlend"] = target.vfBlend;
    doc["vfLF"] = target.vfLowFreq;
    doc["vfLL"] = target.vfLowLevel;
    doc["vfMF"] = target.vfMidFreq;
    doc["vfML"] = target.vfMidLevel;
    doc["vfBase"] = target.vfBaseFreq;
    doc["clEn"] = target.closedLoopEnabled;
    doc["clCtrl"] = target.closedLoopControlMode;
    doc["clMd"] = target.closedLoopSensorMode;
    JsonArray clTargets = doc["clT"].to<JsonArray>();
    for (int i = 0; i < 3; i++) clTargets.add(target.closedLoopTargetRpm[i]);
    doc["clCpr"] = target.closedLoopCountsPerRev;
    doc["clEdge"] = target.closedLoopPulseEdge;
    doc["clQuad"] = target.closedLoopQuadratureMode;
    doc["clRev"] = target.closedLoopReverseDirection;
    doc["clDir"] = target.closedLoopDirectionFaultAction;
    doc["clDb"] = target.closedLoopDebounceUs;
    doc["clTo"] = target.closedLoopTimeoutMs;
    doc["clEng"] = target.closedLoopEngageDelayMs;
    doc["clUpd"] = target.closedLoopUpdateIntervalMs;
    doc["clAlpha"] = target.closedLoopFilterAlpha;
    doc["clDbnd"] = target.closedLoopTuning[SPEED_33].deadbandRpm;
    doc["clLock"] = target.closedLoopTuning[SPEED_33].lockToleranceRpm;
    doc["clLockMs"] = target.closedLoopTuning[SPEED_33].lockTimeMs;
    doc["clKp"] = target.closedLoopTuning[SPEED_33].kp;
    doc["clKi"] = target.closedLoopTuning[SPEED_33].ki;
    doc["clKd"] = target.closedLoopTuning[SPEED_33].kd;
    doc["clILim"] = target.closedLoopTuning[SPEED_33].integralLimitHz;
    doc["clCLim"] = target.closedLoopTuning[SPEED_33].correctionLimitHz;
    doc["clSlew"] = target.closedLoopTuning[SPEED_33].slewLimitHzPerSec;
    doc["clDrop"] = target.closedLoopDropoutAction;
    doc["clReqSig"] = target.closedLoopRequireSignalBeforeEngage;
    doc["clReqNear"] = target.closedLoopRequireNearTargetBeforeEngage;
    doc["clEngTol"] = target.closedLoopEngageToleranceRpm;
    doc["clRamp"] = target.closedLoopRampMode;
    doc["clRampKp"] = target.closedLoopTuning[SPEED_33].rampKp;
    doc["clRampLim"] = target.closedLoopTuning[SPEED_33].rampCorrectionLimitHz;
    doc["clPitchMode"] = target.closedLoopPitchTargetMode;
    doc["clPitchSlew"] = target.closedLoopPitchSlewRpmPerSec;
    doc["clPitchReset"] = target.closedLoopPitchResetThresholdRpm;
    doc["clSatMs"] = target.closedLoopSaturationTimeMs;
    doc["clSatAct"] = target.closedLoopSaturationAction;
    doc["clMinRpm"] = target.closedLoopPlausibilityMinRpm;
    doc["clMaxRpm"] = target.closedLoopPlausibilityMaxRpm;
    doc["clPlausAct"] = target.closedLoopPlausibilityAction;
    doc["clLockTo"] = target.closedLoopLockTimeoutMs;
    doc["clLockAct"] = target.closedLoopLockTimeoutAction;
    doc["clAmpRec"] = target.closedLoopAmpRecoveryMode;
    doc["clAmpRecMs"] = target.closedLoopAmpRecoveryDelayMs;
    JsonArray clTune = doc["clTune"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
        JsonObject tune = clTune.add<JsonObject>();
        tune["db"] = target.closedLoopTuning[i].deadbandRpm;
        tune["lock"] = target.closedLoopTuning[i].lockToleranceRpm;
        tune["lockMs"] = target.closedLoopTuning[i].lockTimeMs;
        tune["kp"] = target.closedLoopTuning[i].kp;
        tune["ki"] = target.closedLoopTuning[i].ki;
        tune["kd"] = target.closedLoopTuning[i].kd;
        tune["iLim"] = target.closedLoopTuning[i].integralLimitHz;
        tune["cLim"] = target.closedLoopTuning[i].correctionLimitHz;
        tune["slew"] = target.closedLoopTuning[i].slewLimitHzPerSec;
        tune["rKp"] = target.closedLoopTuning[i].rampKp;
        tune["rLim"] = target.closedLoopTuning[i].rampCorrectionLimitHz;
    }

    // Braking
    doc["brkMd"] = target.brakeMode;
    doc["brkDur"] = target.brakeDuration;
    doc["brkPG"] = target.brakePulseGap;
    doc["brkSF"] = target.brakeStartFreq;
    doc["brkStF"] = target.brakeStopFreq;
    doc["brkCut"] = target.softStopCutoff;

    // Speeds array carries all per-speed waveform/startup/filter settings.
    JsonArray speeds = doc["speeds"].to<JsonArray>();
    for (int i=0; i<3; i++) {
        JsonObject spd = speeds.add<JsonObject>();
        spd["f"] = target.speeds[i].frequency;
        spd["minF"] = target.speeds[i].minFrequency;
        spd["maxF"] = target.speeds[i].maxFrequency;

        JsonArray ph = spd["ph"].to<JsonArray>();
        for (int p=0; p<4; p++) ph.add(target.speeds[i].phaseOffset[p]);

        JsonArray gains = spd["gain"].to<JsonArray>();
        for (int p=0; p<4; p++) gains.add(target.speeds[i].channelAmplitude[p]);

        spd["ssD"] = target.speeds[i].softStartDuration;
        spd["rAmp"] = target.speeds[i].reducedAmplitude;
        spd["aDly"] = target.speeds[i].amplitudeDelay;
        spd["kick"] = target.speeds[i].startupKick;
        spd["kDur"] = target.speeds[i].startupKickDuration;
        spd["kRmp"] = target.speeds[i].startupKickRampDuration;

        spd["fTyp"] = target.speeds[i].filterType;
        spd["iir"] = target.speeds[i].iirAlpha;
        spd["fir"] = target.speeds[i].firProfile;
    }

    serializeJson(doc, outStr);
    return true;
}

bool Settings::importPresetFromJSON(uint8_t slot, const String& jsonStr) {
    if (slot >= MAX_PRESET_SLOTS) return false;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);

    if (error) {
        if (SERIAL_MONITOR_ENABLE) {
            Serial.print("JSON Error: ");
            Serial.println(error.c_str());
        }
        return false;
    }

    GlobalSettings target;
    // Base imports on the existing slot, or current config if the slot is empty, so omitted keys keep their previous values.
    if (!loadFromSlot(slot, target)) {
        target = _data;
    }

    // Every imported field is optional. validate() below clamps unsafe or out-of-range values before the slot is saved.
    if (doc["pm"].is<uint8_t>()) target.phaseMode = doc["pm"].as<uint8_t>();
    if (doc["top"].is<uint8_t>()) target.motorTopology = doc["top"].as<uint8_t>();
    if (doc["phSlew"].is<float>()) target.phaseSlewDegreesPerSecond = doc["phSlew"].as<float>();
    if (doc["gainSlew"].is<float>()) target.gainSlewPercentPerSecond = doc["gainSlew"].as<float>();
    if (doc["maxAmp"].is<uint8_t>()) target.maxAmplitude = doc["maxAmp"].as<uint8_t>();
    if (doc["ssCurve"].is<uint8_t>()) target.softStartCurve = doc["ssCurve"].as<uint8_t>();
    if (doc["vfBlend"].is<uint8_t>()) target.vfBlend = doc["vfBlend"].as<uint8_t>();
    if (doc["vfLF"].is<float>()) target.vfLowFreq = doc["vfLF"].as<float>();
    if (doc["vfLL"].is<uint8_t>()) target.vfLowLevel = doc["vfLL"].as<uint8_t>();
    if (doc["vfMF"].is<float>()) target.vfMidFreq = doc["vfMF"].as<float>();
    if (doc["vfML"].is<uint8_t>()) target.vfMidLevel = doc["vfML"].as<uint8_t>();
    if (doc["vfBase"].is<float>()) target.vfBaseFreq = doc["vfBase"].as<float>();
    if (doc["clEn"].is<bool>()) target.closedLoopEnabled = doc["clEn"].as<bool>();
    if (doc["clCtrl"].is<uint8_t>()) target.closedLoopControlMode = doc["clCtrl"].as<uint8_t>();
    if (doc["clMd"].is<uint8_t>()) target.closedLoopSensorMode = doc["clMd"].as<uint8_t>();
    JsonArray clTargets = doc["clT"].as<JsonArray>();
    if (!clTargets.isNull()) {
        for (size_t i = 0; i < 3 && i < clTargets.size(); i++) {
            if (clTargets[i].is<float>()) target.closedLoopTargetRpm[i] = clTargets[i].as<float>();
        }
    }
    if (doc["clCpr"].is<uint16_t>()) target.closedLoopCountsPerRev = doc["clCpr"].as<uint16_t>();
    if (doc["clEdge"].is<uint8_t>()) target.closedLoopPulseEdge = doc["clEdge"].as<uint8_t>();
    if (doc["clQuad"].is<uint8_t>()) target.closedLoopQuadratureMode = doc["clQuad"].as<uint8_t>();
    if (doc["clRev"].is<bool>()) target.closedLoopReverseDirection = doc["clRev"].as<bool>();
    if (doc["clDir"].is<uint8_t>()) target.closedLoopDirectionFaultAction = doc["clDir"].as<uint8_t>();
    if (doc["clDb"].is<uint16_t>()) target.closedLoopDebounceUs = doc["clDb"].as<uint16_t>();
    if (doc["clTo"].is<uint16_t>()) target.closedLoopTimeoutMs = doc["clTo"].as<uint16_t>();
    if (doc["clEng"].is<uint16_t>()) target.closedLoopEngageDelayMs = doc["clEng"].as<uint16_t>();
    if (doc["clUpd"].is<uint16_t>()) target.closedLoopUpdateIntervalMs = doc["clUpd"].as<uint16_t>();
    if (doc["clAlpha"].is<float>()) target.closedLoopFilterAlpha = doc["clAlpha"].as<float>();
    bool oldClosedLoopTuningImported = false;
    if (doc["clDbnd"].is<float>()) { target.closedLoopDeadbandRpm = doc["clDbnd"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clLock"].is<float>()) { target.closedLoopLockToleranceRpm = doc["clLock"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clLockMs"].is<uint16_t>()) { target.closedLoopLockTimeMs = doc["clLockMs"].as<uint16_t>(); oldClosedLoopTuningImported = true; }
    if (doc["clKp"].is<float>()) { target.closedLoopKp = doc["clKp"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clKi"].is<float>()) { target.closedLoopKi = doc["clKi"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clKd"].is<float>()) { target.closedLoopKd = doc["clKd"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clILim"].is<float>()) { target.closedLoopIntegralLimitHz = doc["clILim"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clCLim"].is<float>()) { target.closedLoopCorrectionLimitHz = doc["clCLim"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clSlew"].is<float>()) { target.closedLoopSlewLimitHzPerSec = doc["clSlew"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clDrop"].is<uint8_t>()) target.closedLoopDropoutAction = doc["clDrop"].as<uint8_t>();
    if (doc["clReqSig"].is<bool>()) target.closedLoopRequireSignalBeforeEngage = doc["clReqSig"].as<bool>();
    if (doc["clReqNear"].is<bool>()) target.closedLoopRequireNearTargetBeforeEngage = doc["clReqNear"].as<bool>();
    if (doc["clEngTol"].is<float>()) target.closedLoopEngageToleranceRpm = doc["clEngTol"].as<float>();
    if (doc["clRamp"].is<uint8_t>()) target.closedLoopRampMode = doc["clRamp"].as<uint8_t>();
    if (doc["clRampKp"].is<float>()) { target.closedLoopRampKp = doc["clRampKp"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clRampLim"].is<float>()) { target.closedLoopRampCorrectionLimitHz = doc["clRampLim"].as<float>(); oldClosedLoopTuningImported = true; }
    if (doc["clPitchMode"].is<uint8_t>()) target.closedLoopPitchTargetMode = doc["clPitchMode"].as<uint8_t>();
    if (doc["clPitchSlew"].is<float>()) target.closedLoopPitchSlewRpmPerSec = doc["clPitchSlew"].as<float>();
    if (doc["clPitchReset"].is<float>()) target.closedLoopPitchResetThresholdRpm = doc["clPitchReset"].as<float>();
    if (doc["clSatMs"].is<uint16_t>()) target.closedLoopSaturationTimeMs = doc["clSatMs"].as<uint16_t>();
    if (doc["clSatAct"].is<uint8_t>()) target.closedLoopSaturationAction = doc["clSatAct"].as<uint8_t>();
    if (doc["clMinRpm"].is<float>()) target.closedLoopPlausibilityMinRpm = doc["clMinRpm"].as<float>();
    if (doc["clMaxRpm"].is<float>()) target.closedLoopPlausibilityMaxRpm = doc["clMaxRpm"].as<float>();
    if (doc["clPlausAct"].is<uint8_t>()) target.closedLoopPlausibilityAction = doc["clPlausAct"].as<uint8_t>();
    if (doc["clLockTo"].is<uint16_t>()) target.closedLoopLockTimeoutMs = doc["clLockTo"].as<uint16_t>();
    if (doc["clLockAct"].is<uint8_t>()) target.closedLoopLockTimeoutAction = doc["clLockAct"].as<uint8_t>();
    if (doc["clAmpRec"].is<uint8_t>()) target.closedLoopAmpRecoveryMode = doc["clAmpRec"].as<uint8_t>();
    if (doc["clAmpRecMs"].is<uint16_t>()) target.closedLoopAmpRecoveryDelayMs = doc["clAmpRecMs"].as<uint16_t>();
    JsonArray clTune = doc["clTune"].as<JsonArray>();
    if (!clTune.isNull()) {
        // New preset format stores closed-loop tuning per speed.
        for (size_t i = 0; i < 3 && i < clTune.size(); i++) {
            JsonObject tune = clTune[i].as<JsonObject>();
            if (tune.isNull()) continue;
            ClosedLoopSpeedTuning& t = target.closedLoopTuning[i];
            if (tune["db"].is<float>()) t.deadbandRpm = tune["db"].as<float>();
            if (tune["lock"].is<float>()) t.lockToleranceRpm = tune["lock"].as<float>();
            if (tune["lockMs"].is<uint16_t>()) t.lockTimeMs = tune["lockMs"].as<uint16_t>();
            if (tune["kp"].is<float>()) t.kp = tune["kp"].as<float>();
            if (tune["ki"].is<float>()) t.ki = tune["ki"].as<float>();
            if (tune["kd"].is<float>()) t.kd = tune["kd"].as<float>();
            if (tune["iLim"].is<float>()) t.integralLimitHz = tune["iLim"].as<float>();
            if (tune["cLim"].is<float>()) t.correctionLimitHz = tune["cLim"].as<float>();
            if (tune["slew"].is<float>()) t.slewLimitHzPerSec = tune["slew"].as<float>();
            if (tune["rKp"].is<float>()) t.rampKp = tune["rKp"].as<float>();
            if (tune["rLim"].is<float>()) t.rampCorrectionLimitHz = tune["rLim"].as<float>();
        }
    } else if (oldClosedLoopTuningImported) {
        // Older preset exports stored one closed-loop tuning block. Apply it to all speeds so imports preserve old behavior.
        copyGlobalClosedLoopTuningToSpeeds(target);
    }

    if (doc["brkMd"].is<uint8_t>()) target.brakeMode = doc["brkMd"].as<uint8_t>();
    if (doc["brkDur"].is<float>()) target.brakeDuration = doc["brkDur"].as<float>();
    if (doc["brkPG"].is<float>()) target.brakePulseGap = doc["brkPG"].as<float>();
    if (doc["brkSF"].is<float>()) target.brakeStartFreq = doc["brkSF"].as<float>();
    if (doc["brkStF"].is<float>()) target.brakeStopFreq = doc["brkStF"].as<float>();
    if (doc["brkCut"].is<float>()) target.softStopCutoff = doc["brkCut"].as<float>();

    // Speeds array is optional; partial imports can update only selected speeds.
    JsonArray speeds = doc["speeds"].as<JsonArray>();
    if (!speeds.isNull()) {
        for (size_t i=0; i<3 && i<speeds.size(); i++) {
            JsonObject spd = speeds[i].as<JsonObject>();
            if (spd.isNull()) continue;

            if (spd["f"].is<float>()) target.speeds[i].frequency = spd["f"].as<float>();
            if (spd["minF"].is<float>()) target.speeds[i].minFrequency = spd["minF"].as<float>();
            if (spd["maxF"].is<float>()) target.speeds[i].maxFrequency = spd["maxF"].as<float>();

            JsonArray ph = spd["ph"].as<JsonArray>();
            if (!ph.isNull()) {
                for (size_t p=0; p<4 && p<ph.size(); p++) {
                    if (ph[p].is<float>()) target.speeds[i].phaseOffset[p] = ph[p].as<float>();
                }
            }

            JsonArray gains = spd["gain"].as<JsonArray>();
            if (!gains.isNull()) {
                for (size_t p=0; p<4 && p<gains.size(); p++) {
                    if (gains[p].is<uint8_t>()) target.speeds[i].channelAmplitude[p] = gains[p].as<uint8_t>();
                }
            }

            if (spd["ssD"].is<float>()) target.speeds[i].softStartDuration = spd["ssD"].as<float>();
            if (spd["rAmp"].is<uint8_t>()) target.speeds[i].reducedAmplitude = spd["rAmp"].as<uint8_t>();
            if (spd["aDly"].is<uint8_t>()) target.speeds[i].amplitudeDelay = spd["aDly"].as<uint8_t>();
            if (spd["kick"].is<uint8_t>()) target.speeds[i].startupKick = spd["kick"].as<uint8_t>();
            if (spd["kDur"].is<uint8_t>()) target.speeds[i].startupKickDuration = spd["kDur"].as<uint8_t>();
            if (spd["kRmp"].is<float>()) target.speeds[i].startupKickRampDuration = spd["kRmp"].as<float>();

            if (spd["fTyp"].is<uint8_t>()) target.speeds[i].filterType = spd["fTyp"].as<uint8_t>();
            if (spd["iir"].is<float>()) target.speeds[i].iirAlpha = spd["iir"].as<float>();
            if (spd["fir"].is<uint8_t>()) target.speeds[i].firProfile = spd["fir"].as<uint8_t>();
        }
    }

    // validate() operates on _data, so temporarily swap the candidate into the live struct, copy the normalized result, then restore the active settings.
    GlobalSettings liveSettings = _data;
    _data = target;
    validate();
    target = _data;
    _data = liveSettings;

    // Save to the slot file without changing active runtime settings.
    return saveToSlot(slot, target);
}

void Settings::updateRuntime() {
    // Runtime accumulation is coarse by design to avoid frequent writes. Total runtime is persisted by normal settings saves.
    uint32_t now = millis();
    if (now - _lastRuntimeUpdate >= 1000) {
        uint32_t seconds = (now - _lastRuntimeUpdate) / 1000;
        _lastRuntimeUpdate = now;
        _sessionRuntime += seconds;
        _data.totalRuntime += seconds;
    }
}

void Settings::syncRuntimeClock() {
    _lastRuntimeUpdate = millis();
}

uint32_t Settings::getSessionRuntime() {
    return _sessionRuntime;
}

uint32_t Settings::getTotalRuntime() {
    return _data.totalRuntime;
}
