/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <LittleFS.h>
#include "types.h"
#include "config.h"

/**
 * @brief Manages persistent configuration using LittleFS.
 * 
 * Owns the binary storage contract for GlobalSettings, preset slots, runtime
 * counters, schema migration, and JSON preset import/export. The in-memory
 * struct is always normalized before being used by motor or UI code.
 */
class Settings {
public:
    Settings();
    
    void begin();
    void load();
    /*
     * rollbackProtected saves the current file as known-good and marks the next
     * boot as a candidate; setup clears the marker only after waveform buffers
     * are successfully running.
     */
    void save(bool verbose = false, bool rollbackProtected = false);
    void resetDefaults();
    void factoryReset();
    void markBootSuccessful();
    bool rollbackWasApplied() const { return _rollbackApplied; }
    
    // Direct mutable access is used throughout the firmware. Call normalize() after batch edits from serial/web/menu code before applying settings.
    GlobalSettings& get() { return _data; }
    void normalize();
    
    // Helper to get current speed settings
    SpeedSettings& getCurrentSpeedSettings();
    ClosedLoopSpeedTuning& getCurrentClosedLoopTuning();
    ClosedLoopSpeedTuning& getClosedLoopTuning(SpeedMode speed);

    // --- Preset Management ---
    void savePreset(uint8_t slot);
    bool loadPreset(uint8_t slot);
    void resetPreset(uint8_t slot);
    void renamePreset(uint8_t slot, const char* name);
    void duplicatePreset(uint8_t src, uint8_t dest);
    const char* getPresetName(uint8_t slot);
    
    // --- Serialization ---
    bool exportPresetToJSON(uint8_t slot, String& outStr);
    bool importPresetFromJSON(uint8_t slot, const String& jsonStr);
    
    // --- Runtime Tracking ---
    void updateRuntime();
    void syncRuntimeClock();
    uint32_t getSessionRuntime();
    uint32_t getTotalRuntime();
    void resetSessionRuntime();
    void resetTotalRuntime();

private:
    // Current live settings. This is written directly as a binary payload, so field changes must be coordinated with types.h/config.h migrations.
    GlobalSettings _data;
    const char* _filename = "/settings.bin";

    // Runtime counters are updated once per second while MotorController reports a running state.
    uint32_t _lastRuntimeUpdate;
    uint32_t _sessionRuntime;

    // Rollback state guards against a saved setting that lets the firmware boot but fails before waveform generation becomes healthy.
    bool _rollbackApplied;
    bool _bootCandidateActive;
    
    void validate();
    void setDefaults();
    void handlePendingRollback();
    
    // Internal helpers. Preset slots use the same blob format as global settings but a different file magic so the two cannot be accidentally interchanged.
    void saveToSlot(uint8_t slot); // Legacy wrapper
    void loadFromSlot(uint8_t slot); // Legacy wrapper
    bool loadFromSlot(uint8_t slot, GlobalSettings& target);
    void saveToSlot(uint8_t slot, const GlobalSettings& source);
};

#endif // SETTINGS_H
