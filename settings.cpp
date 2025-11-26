/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "settings.h"

// --- Helper Functions for Preset Management ---

// Loads a preset from a specific slot file into a target struct
// Returns true if successful, false if file not found or read error
bool Settings::loadFromSlot(uint8_t slot, GlobalSettings& target) {
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    if (LittleFS.exists(path)) {
        File f = LittleFS.open(path, "r");
        if (f) {
            if (f.read((uint8_t*)&target, sizeof(GlobalSettings)) == sizeof(GlobalSettings)) {
                f.close();
                return true;
            }
            f.close();
        }
    }
    return false;
}

// Saves a specific settings struct to a preset slot file
void Settings::saveToSlot(uint8_t slot, const GlobalSettings& source) {
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    File f = LittleFS.open(path, "w");
    if (f) {
        f.write((uint8_t*)&source, sizeof(GlobalSettings));
        f.close();
    }
}

void Settings::renamePreset(uint8_t slot, const char* name) {
    if (slot >= MAX_PRESET_SLOTS) return;
    strncpy(_data.presetNames[slot], name, 16);
    _data.presetNames[slot][16] = 0; // Ensure null termination
    save();
}

void Settings::duplicatePreset(uint8_t src, uint8_t dest) {
    if (src >= MAX_PRESET_SLOTS || dest >= MAX_PRESET_SLOTS) return;
    GlobalSettings temp;
    if (loadFromSlot(src, temp)) {
        saveToSlot(dest, temp);
    }
}

void Settings::resetSessionRuntime() {
    _sessionRuntime = 0;
}

void Settings::resetTotalRuntime() {
    _data.totalRuntime = 0;
    save();
}

Settings::Settings() {
    _sessionRuntime = 0;
}

void Settings::begin() {
    // Mount the filesystem
    if (!LittleFS.begin()) {
        Serial.println("LittleFS Mount Failed. Formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS Mount Failed again. Critical Error.");
            return;
        }
    }
    load();
}

void Settings::load() {
    if (LittleFS.exists(_filename)) {
        File f = LittleFS.open(_filename, "r");
        if (f) {
            uint32_t version;
            if (f.read((uint8_t*)&version, sizeof(version)) == sizeof(version)) {
                // Rewind to start
                f.seek(0);
                
                if (version < SETTINGS_SCHEMA_VERSION) {
                    Serial.print("Migrating settings from v");
                    Serial.print(version);
                    Serial.print(" to v");
                    Serial.println(SETTINGS_SCHEMA_VERSION);
                    
                    if (migrate(version, f)) {
                        Serial.println("Migration successful.");
                        f.close();
                        return;
                    } else {
                        Serial.println("Migration failed. Resetting defaults.");
                    }
                } else if (version > SETTINGS_SCHEMA_VERSION) {
                    Serial.println("Newer schema version detected. Resetting defaults.");
                } else {
                    // Current version
                    if (f.read((uint8_t*)&_data, sizeof(GlobalSettings)) == sizeof(GlobalSettings)) {
                        Serial.println("Settings loaded.");
                        validate();
                        f.close();
                        return;
                    }
                }
            }
            f.close();
        }
    }
    
    Serial.println("Settings not found or invalid. Using defaults.");
    resetDefaults();
}

// --- Legacy Structures for Migration ---

// V2: Before FDA and Boot Speed
struct GlobalSettingsV2 {
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
    SpeedSettings speeds[3];
    char presetNames[5][17];
    uint32_t totalRuntime;
    bool reverseEncoder;
    float pitchStepSize;
    uint8_t rampType;
    uint8_t screensaverMode;
    bool enable78rpm;
    // NO FDA
    // NO Boot Speed
    SpeedMode currentSpeed;
};

// V3: With FDA, Before Boot Speed
struct GlobalSettingsV3 {
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
    SpeedSettings speeds[3];
    char presetNames[5][17];
    uint32_t totalRuntime;
    bool reverseEncoder;
    float pitchStepSize;
    uint8_t rampType;
    uint8_t screensaverMode;
    bool enable78rpm;
    uint8_t freqDependentAmplitude;
    // NO Boot Speed
    SpeedMode currentSpeed;
};

bool Settings::migrate(uint32_t oldVersion, File& f) {
    // Initialize current defaults first to ensure new fields have safe values
    GlobalSettings newSettings;
    // We can't use setDefaults() directly on a local var easily without refactoring, 
    // but we can use the main _data and then copy back.
    // Better: Load legacy into temp, then copy fields to _data (which is already defaulted if we call setDefaults first)
    
    setDefaults(); // Initialize _data with defaults (including new fields)
    
    if (oldVersion == 2) {
        GlobalSettingsV2 v2;
        if (f.read((uint8_t*)&v2, sizeof(GlobalSettingsV2)) != sizeof(GlobalSettingsV2)) return false;
        
        // Copy common fields
        _data.phaseMode = v2.phaseMode;
        _data.maxAmplitude = v2.maxAmplitude;
        _data.softStartCurve = v2.softStartCurve;
        _data.smoothSwitching = v2.smoothSwitching;
        _data.switchRampDuration = v2.switchRampDuration;
        _data.brakeMode = v2.brakeMode;
        _data.brakeDuration = v2.brakeDuration;
        _data.brakePulseGap = v2.brakePulseGap;
        _data.brakeStartFreq = v2.brakeStartFreq;
        _data.brakeStopFreq = v2.brakeStopFreq;
        _data.relayActiveHigh = v2.relayActiveHigh;
        _data.muteRelayLinkStandby = v2.muteRelayLinkStandby;
        _data.muteRelayLinkStartStop = v2.muteRelayLinkStartStop;
        _data.powerOnRelayDelay = v2.powerOnRelayDelay;
        _data.displayBrightness = v2.displayBrightness;
        _data.displaySleepDelay = v2.displaySleepDelay;
        _data.screensaverEnabled = v2.screensaverEnabled;
        _data.autoDimDelay = v2.autoDimDelay;
        _data.showRuntime = v2.showRuntime;
        _data.errorDisplayEnabled = v2.errorDisplayEnabled;
        _data.errorDisplayDuration = v2.errorDisplayDuration;
        _data.autoStandbyDelay = v2.autoStandbyDelay;
        _data.autoStart = v2.autoStart;
        _data.autoBoot = v2.autoBoot;
        _data.pitchResetOnStop = v2.pitchResetOnStop;
        memcpy(_data.speeds, v2.speeds, sizeof(v2.speeds));
        memcpy(_data.presetNames, v2.presetNames, sizeof(v2.presetNames));
        _data.totalRuntime = v2.totalRuntime;
        _data.reverseEncoder = v2.reverseEncoder;
        _data.pitchStepSize = v2.pitchStepSize;
        _data.rampType = v2.rampType;
        _data.screensaverMode = v2.screensaverMode;
        _data.enable78rpm = v2.enable78rpm;
        _data.currentSpeed = v2.currentSpeed;
        
        // New fields for V3/V4 are already set by setDefaults()
        // freqDependentAmplitude = 0
        // bootSpeed = 3
        
        save();
        return true;
    }
    else if (oldVersion == 3) {
        GlobalSettingsV3 v3;
        if (f.read((uint8_t*)&v3, sizeof(GlobalSettingsV3)) != sizeof(GlobalSettingsV3)) return false;
        
        // Copy common fields (V3 is V2 + FDA)
        _data.phaseMode = v3.phaseMode;
        _data.maxAmplitude = v3.maxAmplitude;
        _data.softStartCurve = v3.softStartCurve;
        _data.smoothSwitching = v3.smoothSwitching;
        _data.switchRampDuration = v3.switchRampDuration;
        _data.brakeMode = v3.brakeMode;
        _data.brakeDuration = v3.brakeDuration;
        _data.brakePulseGap = v3.brakePulseGap;
        _data.brakeStartFreq = v3.brakeStartFreq;
        _data.brakeStopFreq = v3.brakeStopFreq;
        _data.relayActiveHigh = v3.relayActiveHigh;
        _data.muteRelayLinkStandby = v3.muteRelayLinkStandby;
        _data.muteRelayLinkStartStop = v3.muteRelayLinkStartStop;
        _data.powerOnRelayDelay = v3.powerOnRelayDelay;
        _data.displayBrightness = v3.displayBrightness;
        _data.displaySleepDelay = v3.displaySleepDelay;
        _data.screensaverEnabled = v3.screensaverEnabled;
        _data.autoDimDelay = v3.autoDimDelay;
        _data.showRuntime = v3.showRuntime;
        _data.errorDisplayEnabled = v3.errorDisplayEnabled;
        _data.errorDisplayDuration = v3.errorDisplayDuration;
        _data.autoStandbyDelay = v3.autoStandbyDelay;
        _data.autoStart = v3.autoStart;
        _data.autoBoot = v3.autoBoot;
        _data.pitchResetOnStop = v3.pitchResetOnStop;
        memcpy(_data.speeds, v3.speeds, sizeof(v3.speeds));
        memcpy(_data.presetNames, v3.presetNames, sizeof(v3.presetNames));
        _data.totalRuntime = v3.totalRuntime;
        _data.reverseEncoder = v3.reverseEncoder;
        _data.pitchStepSize = v3.pitchStepSize;
        _data.rampType = v3.rampType;
        _data.screensaverMode = v3.screensaverMode;
        _data.enable78rpm = v3.enable78rpm;
        _data.freqDependentAmplitude = v3.freqDependentAmplitude;
        _data.currentSpeed = v3.currentSpeed;
        
        // New fields for V4 are already set by setDefaults()
        // bootSpeed = 3
        
        save();
        return true;
    }
    
    return false;
}

void Settings::save(bool verbose) {
    File f = LittleFS.open(_filename, "w");
    if (f) {
        f.write((uint8_t*)&_data, sizeof(GlobalSettings));
        f.close();
        if (verbose) Serial.println("Settings saved.");
    } else {
        if (verbose) Serial.println("Failed to save settings.");
    }
}

void Settings::resetDefaults() {
    setDefaults();
    save();
}

void Settings::factoryReset() {
    // Format filesystem to clear all presets and logs
    LittleFS.format();
    resetDefaults();
}

SpeedSettings& Settings::getCurrentSpeedSettings() {
    return _data.speeds[_data.currentSpeed];
}

void Settings::validate() {
    // Check schema version to handle data migration if needed
    if (_data.schemaVersion != SETTINGS_SCHEMA_VERSION) {
        Serial.println("Schema mismatch. Resetting defaults.");
        // In the future, migration logic would go here
        resetDefaults();
    }
    
    // Enforce valid ranges
    // Enforce valid ranges
    if (_data.currentSpeed > SPEED_78) _data.currentSpeed = SPEED_33;
    if (_data.maxAmplitude > 100) _data.maxAmplitude = 100;
    
    // Validate Per-Speed Settings
    for(int i=0; i<3; i++) {
        if (_data.speeds[i].minFrequency > _data.speeds[i].maxFrequency) {
            float temp = _data.speeds[i].minFrequency;
            _data.speeds[i].minFrequency = _data.speeds[i].maxFrequency;
            _data.speeds[i].maxFrequency = temp;
        }
        if (_data.speeds[i].frequency < _data.speeds[i].minFrequency) _data.speeds[i].frequency = _data.speeds[i].minFrequency;
        if (_data.speeds[i].frequency > _data.speeds[i].maxFrequency) _data.speeds[i].frequency = _data.speeds[i].maxFrequency;
        
        if (_data.speeds[i].softStartDuration < 0.0) _data.speeds[i].softStartDuration = 0.0;
        
        // Normalize Phase Offsets (0-360)
        for(int p=0; p<4; p++) {
            while(_data.speeds[i].phaseOffset[p] >= 360.0) _data.speeds[i].phaseOffset[p] -= 360.0;
            while(_data.speeds[i].phaseOffset[p] < 0.0) _data.speeds[i].phaseOffset[p] += 360.0;
        }
    }
}

void Settings::setDefaults() {
    _data.schemaVersion = SETTINGS_SCHEMA_VERSION;
    // Initialize Preset Names
    for(int i=0; i<5; i++) {
        snprintf(_data.presetNames[i], 17, "Preset %d", i+1);
    }
    
    _data.phaseMode = (PhaseMode)DEFAULT_PHASE_MODE;
    _data.maxAmplitude = 100;
    _data.softStartCurve = 0; // Linear
    _data.smoothSwitching = true;
    _data.switchRampDuration = 2;
    
    _data.brakeMode = BRAKE_RAMP;
    _data.brakeDuration = 2.0;
    _data.brakePulseGap = 0.5;
    _data.brakeStartFreq = 50.0;
    _data.brakeStopFreq = 0.0;
    
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
    _data.pitchResetOnStop = true;
    _data.currentSpeed = (SpeedMode)DEFAULT_SPEED_INDEX;
    
    // --- 33.3 RPM Defaults (50Hz base) ---
    _data.speeds[SPEED_33].frequency = 50.0;
    _data.speeds[SPEED_33].minFrequency = 40.0;
    _data.speeds[SPEED_33].maxFrequency = 60.0;
    _data.speeds[SPEED_33].softStartDuration = 1.0;
    _data.speeds[SPEED_33].reducedAmplitude = 80;
    _data.speeds[SPEED_33].amplitudeDelay = 5;
    _data.speeds[SPEED_33].startupKick = 1;
    _data.speeds[SPEED_33].startupKickDuration = 1;
    _data.speeds[SPEED_33].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_33].filterType = FILTER_NONE;
    _data.speeds[SPEED_33].iirAlpha = 0.5;
    _data.speeds[SPEED_33].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_33].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_33].phaseOffset[1] = 90.0;
    _data.speeds[SPEED_33].phaseOffset[2] = 120.0;
    _data.speeds[SPEED_33].phaseOffset[3] = 240.0;

    // --- 45 RPM Defaults (67.5Hz base) ---
    _data.speeds[SPEED_45].frequency = 67.5;
    _data.speeds[SPEED_45].minFrequency = 57.5;
    _data.speeds[SPEED_45].maxFrequency = 77.5;
    _data.speeds[SPEED_45].softStartDuration = 1.0;
    _data.speeds[SPEED_45].reducedAmplitude = 80;
    _data.speeds[SPEED_45].amplitudeDelay = 5;
    _data.speeds[SPEED_45].startupKick = 1;
    _data.speeds[SPEED_45].startupKickDuration = 1;
    _data.speeds[SPEED_45].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_45].filterType = FILTER_NONE;
    _data.speeds[SPEED_45].iirAlpha = 0.5;
    _data.speeds[SPEED_45].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_45].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_45].phaseOffset[1] = 90.0;
    _data.speeds[SPEED_45].phaseOffset[2] = 120.0;
    _data.speeds[SPEED_45].phaseOffset[3] = 240.0;

    // --- 78 RPM Defaults (113.5Hz base) ---
    _data.speeds[SPEED_78].frequency = 113.5;
    _data.speeds[SPEED_78].minFrequency = 100.0;
    _data.speeds[SPEED_78].maxFrequency = 130.0;
    _data.speeds[SPEED_78].softStartDuration = 1.5;
    _data.speeds[SPEED_78].reducedAmplitude = 90;
    _data.speeds[SPEED_78].amplitudeDelay = 5;
    _data.speeds[SPEED_78].startupKick = 1;
    _data.speeds[SPEED_78].startupKickDuration = 1;
    _data.speeds[SPEED_78].startupKickRampDuration = 1.0;
    _data.speeds[SPEED_78].filterType = FILTER_NONE;
    _data.speeds[SPEED_78].iirAlpha = 0.5;
    _data.speeds[SPEED_78].firProfile = FIR_MEDIUM;
    _data.speeds[SPEED_78].phaseOffset[0] = 0.0;
    _data.speeds[SPEED_78].phaseOffset[1] = 90.0;
    _data.speeds[SPEED_78].phaseOffset[2] = 120.0;
    _data.speeds[SPEED_78].phaseOffset[3] = 240.0;
    
    _data.enable78rpm = true;
    _data.totalRuntime = 0;
    
    _data.displayBrightness = 255; // Max brightness
    _data.reverseEncoder = false;
    _data.pitchStepSize = 0.1;
    _data.rampType = 1; // Default to S-Curve
    _data.screensaverMode = 0; // Default to Bounce
    _data.freqDependentAmplitude = 0; // Default 0% (Disabled)
    _data.bootSpeed = 3; // Default to Last Used
 // Default 0.1%
}

bool Settings::loadPreset(uint8_t slot) {
    if (slot >= MAX_PRESET_SLOTS) return false;
    
    GlobalSettings temp;
    if (loadFromSlot(slot, temp)) {
        _data = temp;
        validate();
        return true;
    }
    return false;
}

void Settings::savePreset(uint8_t slot) {
    if (slot >= MAX_PRESET_SLOTS) return;
    saveToSlot(slot, _data);
}

void Settings::resetPreset(uint8_t slot) {
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    if (LittleFS.exists(path)) {
        LittleFS.remove(path);
    }
    // Reset name to default
    snprintf(_data.presetNames[slot], 17, "Preset %d", slot + 1);
    save();
}

const char* Settings::getPresetName(uint8_t slot) {
    if (slot >= MAX_PRESET_SLOTS) return "Invalid";
    // Ensure null termination just in case
    _data.presetNames[slot][16] = 0;
    return _data.presetNames[slot];
}

void Settings::saveToSlot(uint8_t slot) {
    // Legacy wrapper
    savePreset(slot);
}

void Settings::loadFromSlot(uint8_t slot) {
    // Legacy wrapper
    loadPreset(slot);
}

void Settings::updateRuntime() {
    uint32_t now = millis();
    if (now - _lastRuntimeUpdate >= 1000) {
        uint32_t seconds = (now - _lastRuntimeUpdate) / 1000;
        _lastRuntimeUpdate = now;
        _sessionRuntime += seconds;
        _data.totalRuntime += seconds;
    }
}

uint32_t Settings::getSessionRuntime() {
    return _sessionRuntime;
}

uint32_t Settings::getTotalRuntime() {
    return _data.totalRuntime;
}
