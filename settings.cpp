/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "settings.h"
#include <ArduinoJson.h>

namespace {
struct SettingsFileHeader {
    uint32_t magic;
    uint16_t formatVersion;
    uint16_t headerSize;
    uint32_t schemaVersion;
    uint32_t payloadSize;
    uint32_t crc32;
};

uint32_t settingsCrc32(const uint8_t* data, size_t length) {
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

bool readSettingsBlob(const char* path, uint32_t magic, GlobalSettings& target) {
    File f = LittleFS.open(path, "r");
    if (!f) return false;

    const size_t expectedSize = sizeof(SettingsFileHeader) + sizeof(GlobalSettings);
    if (f.size() != expectedSize) {
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
        header.schemaVersion != SETTINGS_SCHEMA_VERSION ||
        header.payloadSize != sizeof(GlobalSettings)) {
        f.close();
        return false;
    }

    GlobalSettings candidate;
    if (f.read((uint8_t*)&candidate, sizeof(candidate)) != sizeof(candidate)) {
        f.close();
        return false;
    }
    f.close();

    uint32_t crc = settingsCrc32((const uint8_t*)&candidate, sizeof(candidate));
    if (crc != header.crc32) return false;

    target = candidate;
    return true;
}

bool loadSettingsBlob(const char* path, uint32_t magic, GlobalSettings& target) {
    if (readSettingsBlob(path, magic, target)) return true;

    char backupPath[40];
    if (makeSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) {
        return readSettingsBlob(backupPath, magic, target);
    }
    return false;
}

bool writeSettingsBlob(const char* path, uint32_t magic, const GlobalSettings& source) {
    char tmpPath[40];
    char backupPath[40];
    if (!makeSidecarPath(path, ".tmp", tmpPath, sizeof(tmpPath))) return false;
    if (!makeSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) return false;

    LittleFS.remove(tmpPath);

    File f = LittleFS.open(tmpPath, "w");
    if (!f) return false;

    SettingsFileHeader header;
    header.magic = magic;
    header.formatVersion = SETTINGS_FILE_FORMAT_VERSION;
    header.headerSize = sizeof(SettingsFileHeader);
    header.schemaVersion = SETTINGS_SCHEMA_VERSION;
    header.payloadSize = sizeof(GlobalSettings);
    header.crc32 = settingsCrc32((const uint8_t*)&source, sizeof(source));

    bool ok = f.write((const uint8_t*)&header, sizeof(header)) == sizeof(header);
    ok = ok && f.write((const uint8_t*)&source, sizeof(source)) == sizeof(source);
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
}

// --- Helper Functions for Preset Management ---

// Loads a preset from a specific slot file into a target struct
// Returns true if successful, false if file not found or read error
bool Settings::loadFromSlot(uint8_t slot, GlobalSettings& target) {
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    return loadSettingsBlob(path, PRESET_FILE_MAGIC, target);
}

// Saves a specific settings struct to a preset slot file
void Settings::saveToSlot(uint8_t slot, const GlobalSettings& source) {
    char path[32];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    writeSettingsBlob(path, PRESET_FILE_MAGIC, source);
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
    _lastRuntimeUpdate = millis();
}

void Settings::resetTotalRuntime() {
    _data.totalRuntime = 0;
    save();
}

Settings::Settings() {
    _sessionRuntime = 0;
    _lastRuntimeUpdate = 0;
}

void Settings::begin() {
    // Check Hardware Safe Mode Flag before any formatting or writes.
    extern bool safeModeActive;
    if (safeModeActive) {
        Serial.println("HARDWARE SAFE MODE ENGAGED. Bypassing Flash Load.");
        if (!LittleFS.begin()) {
            Serial.println("LittleFS unavailable in Safe Mode. Leaving flash untouched.");
        }

        setDefaults(); // Load baseline safe settings to RAM only

        // Name the preset to make it obvious
        strncpy(_data.presetNames[0], "SAFE MODE", 16);
        _data.presetNames[0][16] = 0;
        for(int i=1; i<5; i++) {
            strncpy(_data.presetNames[i], "LOCKED", 16);
            _data.presetNames[i][16] = 0;
        }

        _lastRuntimeUpdate = millis();
        return;
    }

    // Mount the filesystem
    if (!LittleFS.begin()) {
        Serial.println("LittleFS Mount Failed. Formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS Mount Failed again. Critical Error.");
            setDefaults();
            return;
        }
    }

    load(); // Normal operation

    _lastRuntimeUpdate = millis();
}

void Settings::load() {
    GlobalSettings loaded;
    if (loadSettingsBlob(_filename, SETTINGS_FILE_MAGIC, loaded)) {
        _data = loaded;
        Serial.println("Settings loaded.");
        validate();
        return;
    }

    Serial.println("Settings not found or invalid. Using defaults.");
    resetDefaults();
}

void Settings::save(bool verbose) {
    if (writeSettingsBlob(_filename, SETTINGS_FILE_MAGIC, _data)) {
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

void Settings::normalize() {
    validate();
}

void Settings::validate() {
    // Current first-release storage is strict: mismatches reset to defaults.
    if (_data.schemaVersion != SETTINGS_SCHEMA_VERSION) {
        Serial.println("Schema mismatch. Resetting defaults.");
        resetDefaults();
    }

    // Enforce valid ranges
    if (_data.phaseMode < PHASE_1 || _data.phaseMode > MAX_PHASE_MODE) _data.phaseMode = DEFAULT_PHASE_MODE;
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
    if (_data.freqDependentAmplitude > 100) _data.freqDependentAmplitude = 100;
    if (_data.vfLowBoost > 100) _data.vfLowBoost = 100;
    if (_data.vfMidBoost > 100) _data.vfMidBoost = 100;
    if (_data.vfLowFreq < 0.0) _data.vfLowFreq = 0.0;
    if (_data.vfLowFreq > 50.0) _data.vfLowFreq = 50.0;
    if (_data.vfMidFreq < 0.0) _data.vfMidFreq = 0.0;
    if (_data.vfMidFreq > 100.0) _data.vfMidFreq = 100.0;
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

    // Validate Per-Speed Settings
    for(int i=0; i<3; i++) {
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

    _data.enable78rpm = true;
    _data.totalRuntime = 0;

    _data.displayBrightness = 255; // Max brightness
    _data.reverseEncoder = false;
    _data.pitchStepSize = 0.1;
    _data.rampType = 1; // Default to S-Curve
    _data.screensaverMode = 0; // Default to Bounce
    _data.freqDependentAmplitude = 0; // Default 0% (Disabled)
    _data.vfLowFreq = 5.0;
    _data.vfLowBoost = 100;
    _data.vfMidFreq = 25.0;
    _data.vfMidBoost = 100;
    _data.bootSpeed = 3; // Default to Last Used
    _data.ampTempWarnC = AMP_TEMP_WARN_C;
    _data.ampTempShutdownC = AMP_TEMP_SHUTDOWN_C;
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

// --- Serialization ---
bool Settings::exportPresetToJSON(uint8_t slot, String& outStr) {
    if (slot >= MAX_PRESET_SLOTS) return false;

    GlobalSettings target;
    // Load existing settings from slot. If fail, fall back to current active data.
    if (!loadFromSlot(slot, target)) {
        target = _data;
    }

    // Create a JSON document (adjust size based on fields, approx 1024 bytes is safe for this)
    JsonDocument doc;

    // Global Motor Parameters
    doc["pm"] = target.phaseMode;
    doc["maxAmp"] = target.maxAmplitude;
    doc["ssCurve"] = target.softStartCurve;
    doc["fda"] = target.freqDependentAmplitude;
    doc["vfLF"] = target.vfLowFreq;
    doc["vfLB"] = target.vfLowBoost;
    doc["vfMF"] = target.vfMidFreq;
    doc["vfMB"] = target.vfMidBoost;

    // Braking
    doc["brkMd"] = target.brakeMode;
    doc["brkDur"] = target.brakeDuration;
    doc["brkPG"] = target.brakePulseGap;
    doc["brkSF"] = target.brakeStartFreq;
    doc["brkStF"] = target.brakeStopFreq;
    doc["brkCut"] = target.softStopCutoff;

    // Speeds array
    JsonArray speeds = doc["speeds"].to<JsonArray>();
    for (int i=0; i<3; i++) {
        JsonObject spd = speeds.add<JsonObject>();
        spd["f"] = target.speeds[i].frequency;
        spd["minF"] = target.speeds[i].minFrequency;
        spd["maxF"] = target.speeds[i].maxFrequency;

        JsonArray ph = spd["ph"].to<JsonArray>();
        for (int p=0; p<4; p++) ph.add(target.speeds[i].phaseOffset[p]);

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
        Serial.print("JSON Error: ");
        Serial.println(error.c_str());
        return false;
    }

    GlobalSettings target;
    // Base it off of existing slot, or current config so we don't wipe screensaver/boot settings
    if (!loadFromSlot(slot, target)) {
        target = _data;
    }

    // Global Motor Parameters
    if (doc["pm"].is<uint8_t>()) target.phaseMode = doc["pm"].as<uint8_t>();
    if (doc["maxAmp"].is<uint8_t>()) target.maxAmplitude = doc["maxAmp"].as<uint8_t>();
    if (doc["ssCurve"].is<uint8_t>()) target.softStartCurve = doc["ssCurve"].as<uint8_t>();
    if (doc["fda"].is<uint8_t>()) target.freqDependentAmplitude = doc["fda"].as<uint8_t>();
    if (doc["vfLF"].is<float>()) target.vfLowFreq = doc["vfLF"].as<float>();
    if (doc["vfLB"].is<uint8_t>()) target.vfLowBoost = doc["vfLB"].as<uint8_t>();
    if (doc["vfMF"].is<float>()) target.vfMidFreq = doc["vfMF"].as<float>();
    if (doc["vfMB"].is<uint8_t>()) target.vfMidBoost = doc["vfMB"].as<uint8_t>();

    if (doc["brkMd"].is<uint8_t>()) target.brakeMode = doc["brkMd"].as<uint8_t>();
    if (doc["brkDur"].is<float>()) target.brakeDuration = doc["brkDur"].as<float>();
    if (doc["brkPG"].is<float>()) target.brakePulseGap = doc["brkPG"].as<float>();
    if (doc["brkSF"].is<float>()) target.brakeStartFreq = doc["brkSF"].as<float>();
    if (doc["brkStF"].is<float>()) target.brakeStopFreq = doc["brkStF"].as<float>();
    if (doc["brkCut"].is<float>()) target.softStopCutoff = doc["brkCut"].as<float>();

    // Speeds array
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

    GlobalSettings liveSettings = _data;
    _data = target;
    validate();
    target = _data;
    _data = liveSettings;

    // Save to slot file, don't change RAM actively
    saveToSlot(slot, target);
    return true;
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

void Settings::syncRuntimeClock() {
    _lastRuntimeUpdate = millis();
}

uint32_t Settings::getSessionRuntime() {
    return _sessionRuntime;
}

uint32_t Settings::getTotalRuntime() {
    return _data.totalRuntime;
}
