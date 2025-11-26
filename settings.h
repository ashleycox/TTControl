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
 * Handles:
 * - Loading/Saving global settings
 * - Preset management (Slots 1-5)
 * - Factory reset
 * - Runtime tracking
 */
class Settings {
public:
    Settings();
    
    void begin();
    void load();
    void save(bool verbose = false);
    void resetDefaults();
    void factoryReset();
    
    // Accessor for the global settings struct
    GlobalSettings& get() { return _data; }
    
    // Helper to get current speed settings
    SpeedSettings& getCurrentSpeedSettings();

    // --- Preset Management ---
    void savePreset(uint8_t slot);
    bool loadPreset(uint8_t slot);
    void resetPreset(uint8_t slot);
    void renamePreset(uint8_t slot, const char* name);
    void duplicatePreset(uint8_t src, uint8_t dest);
    const char* getPresetName(uint8_t slot);
    
    // --- Runtime Tracking ---
    void updateRuntime();
    uint32_t getSessionRuntime();
    uint32_t getTotalRuntime();
    void resetSessionRuntime();
    void resetTotalRuntime();

private:
    GlobalSettings _data;
    const char* _filename = "/settings.bin";
    uint32_t _lastRuntimeUpdate;
    uint32_t _sessionRuntime;
    
    void validate();
    void setDefaults();
    bool migrate(uint32_t oldVersion, File& f);
    
    // Internal helpers
    void saveToSlot(uint8_t slot); // Legacy wrapper
    void loadFromSlot(uint8_t slot); // Legacy wrapper
    bool loadFromSlot(uint8_t slot, GlobalSettings& target);
    void saveToSlot(uint8_t slot, const GlobalSettings& source);
};

#endif // SETTINGS_H
