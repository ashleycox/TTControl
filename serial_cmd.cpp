/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "serial_cmd.h"
#include "ui.h"
#include "error_handler.h"
#include "hal.h"

extern UserInterface ui;

// --- CLI Registry ---
struct SettingItem {
    String name;
    std::function<String()> get;
    std::function<void(String)> set;
};

std::vector<SettingItem> registry;
bool cliInitialized = false;

static const char* speedName(SpeedMode speed) {
    if (speed == SPEED_33) return "33 RPM";
    if (speed == SPEED_45) return "45 RPM";
    return "78 RPM";
}

static const char* motorStateName() {
    if (motor.isRelayTestMode()) return "RELAY TEST";
    switch (motor.getState()) {
        case STATE_STANDBY: return "STANDBY";
        case STATE_STOPPED: return "STOPPED";
        case STATE_STARTING: return "STARTING";
        case STATE_RUNNING: return motor.isSpeedRamping() ? "RAMPING" : "RUNNING";
        case STATE_STOPPING: return "BRAKING";
    }
    return "UNKNOWN";
}

static const char* brakeModeName(uint8_t mode) {
    switch (mode) {
        case BRAKE_OFF: return "Off";
        case BRAKE_PULSE: return "Pulse";
        case BRAKE_RAMP: return "Ramp";
        case BRAKE_SOFT_STOP: return "Soft Stop";
    }
    return "Invalid";
}

static const char* filterName(uint8_t filter) {
    switch (filter) {
        case FILTER_NONE: return "None";
        case FILTER_IIR: return "IIR";
        case FILTER_FIR: return "FIR";
    }
    return "Invalid";
}

static const char* rampTypeName(uint8_t ramp) {
    return ramp == RAMP_SCURVE ? "S-Curve" : "Linear";
}

static void printPresetList();
static void printSettingsDump();
static void handlePresetCommand(const String& input);
static void handleRelayTestCommand(const String& input);

static int clampInt(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

void initCLI() {
    if (cliInitialized) return;
    
    // --- Global Settings ---
    registry.push_back({ "brightness", 
        []() { return String(settings.get().displayBrightness); },
        [](String v) { settings.get().displayBrightness = (uint8_t)clampInt(v.toInt(), 0, 255); }
    });
    
    registry.push_back({ "ramp", 
        []() { return String(settings.get().rampType); },
        [](String v) { settings.get().rampType = (uint8_t)clampInt(v.toInt(), 0, 1); }
    });
    
    registry.push_back({ "pitch_step", 
        []() { return String(settings.get().pitchStepSize); },
        [](String v) { settings.get().pitchStepSize = clampFloat(v.toFloat(), 0.01, 1.0); }
    });
    
    registry.push_back({ "rev_enc", 
        []() { return String(settings.get().reverseEncoder); },
        [](String v) { settings.get().reverseEncoder = (v == "1" || v == "true"); }
    });
    
    registry.push_back({ "saver_mode", 
        []() { return String(settings.get().screensaverMode); },
        [](String v) { settings.get().screensaverMode = (uint8_t)clampInt(v.toInt(), 0, 2); }
    });

    registry.push_back({ "phase_mode",
        []() { return String(settings.get().phaseMode); },
        [](String v) {
            settings.get().phaseMode = (uint8_t)clampInt(v.toInt(), 1, 4);
            motor.applySettings();
        }
    });

    registry.push_back({ "max_amp",
        []() { return String(settings.get().maxAmplitude); },
        [](String v) { settings.get().maxAmplitude = (uint8_t)clampInt(v.toInt(), 0, 100); }
    });

    registry.push_back({ "smooth_switch",
        []() { return String(settings.get().smoothSwitching); },
        [](String v) { settings.get().smoothSwitching = (v == "1" || v == "true"); }
    });

    registry.push_back({ "switch_ramp",
        []() { return String(settings.get().switchRampDuration); },
        [](String v) { settings.get().switchRampDuration = (uint8_t)clampInt(v.toInt(), 1, 5); }
    });

    registry.push_back({ "brake_mode",
        []() { return String(settings.get().brakeMode); },
        [](String v) { settings.get().brakeMode = (uint8_t)clampInt(v.toInt(), 0, 3); }
    });

    registry.push_back({ "brake_duration",
        []() { return String(settings.get().brakeDuration); },
        [](String v) { settings.get().brakeDuration = clampFloat(v.toFloat(), 0.0, 10.0); }
    });

    registry.push_back({ "brake_pulse_gap",
        []() { return String(settings.get().brakePulseGap); },
        [](String v) { settings.get().brakePulseGap = clampFloat(v.toFloat(), 0.1, 2.0); }
    });

    registry.push_back({ "brake_start_freq",
        []() { return String(settings.get().brakeStartFreq); },
        [](String v) { settings.get().brakeStartFreq = clampFloat(v.toFloat(), 10.0, 200.0); }
    });

    registry.push_back({ "brake_stop_freq",
        []() { return String(settings.get().brakeStopFreq); },
        [](String v) { settings.get().brakeStopFreq = clampFloat(v.toFloat(), 0.0, 50.0); }
    });

    registry.push_back({ "brake_cutoff",
        []() { return String(settings.get().softStopCutoff); },
        [](String v) { settings.get().softStopCutoff = clampFloat(v.toFloat(), 0.0, 50.0); }
    });

    registry.push_back({ "relay_active_high",
        []() { return String(settings.get().relayActiveHigh); },
        [](String v) { settings.get().relayActiveHigh = (v == "1" || v == "true"); }
    });

    registry.push_back({ "relay_delay",
        []() { return String(settings.get().powerOnRelayDelay); },
        [](String v) { settings.get().powerOnRelayDelay = (uint8_t)clampInt(v.toInt(), 0, 10); }
    });
    
    // --- Current Speed Settings ---
    // Note: These access the *currently active* speed settings
    
    registry.push_back({ "freq", 
        []() { return String(settings.getCurrentSpeedSettings().frequency); },
        [](String v) {
            SpeedSettings& speed = settings.getCurrentSpeedSettings();
            speed.frequency = clampFloat(v.toFloat(), speed.minFrequency, speed.maxFrequency);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase1", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[0]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[0] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase2", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[1]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[1] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase3", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[2]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[2] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase4", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[3]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[3] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "soft_start", 
        []() { return String(settings.getCurrentSpeedSettings().softStartDuration); },
        [](String v) { settings.getCurrentSpeedSettings().softStartDuration = clampFloat(v.toFloat(), 0.0, 10.0); }
    });
    
    registry.push_back({ "kick", 
        []() { return String(settings.getCurrentSpeedSettings().startupKick); },
        [](String v) { settings.getCurrentSpeedSettings().startupKick = (uint8_t)clampInt(v.toInt(), 1, 4); }
    });
    
    registry.push_back({ "kick_dur", 
        []() { return String(settings.getCurrentSpeedSettings().startupKickDuration); },
        [](String v) { settings.getCurrentSpeedSettings().startupKickDuration = (uint8_t)clampInt(v.toInt(), 0, 15); }
    });

    registry.push_back({ "filter",
        []() { return String(settings.getCurrentSpeedSettings().filterType); },
        [](String v) {
            settings.getCurrentSpeedSettings().filterType = (uint8_t)clampInt(v.toInt(), 0, 2);
            motor.applySettings();
        }
    });

    registry.push_back({ "reduced_amp",
        []() { return String(settings.getCurrentSpeedSettings().reducedAmplitude); },
        [](String v) { settings.getCurrentSpeedSettings().reducedAmplitude = (uint8_t)clampInt(v.toInt(), 50, 100); }
    });

    registry.push_back({ "amp_delay",
        []() { return String(settings.getCurrentSpeedSettings().amplitudeDelay); },
        [](String v) { settings.getCurrentSpeedSettings().amplitudeDelay = (uint8_t)clampInt(v.toInt(), 0, 60); }
    });
    
    // --- Live Motor State ---
    registry.push_back({ "pitch", 
        []() { return String(motor.getPitchPercent()); },
        [](String v) { motor.setPitch(v.toFloat()); }
    });
    
    cliInitialized = true;
}

void handleSerialCommands() {
    if (!cliInitialized) initCLI();
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) return;
        
        // --- Standard Commands ---
        if (input == "start") {
            if (motor.isRelayTestMode()) {
                Serial.println("Exit relay test before starting.");
            } else {
                motor.start();
                Serial.println("Motor Started");
            }
        }
        else if (input == "stop") {
            if (motor.isRelayTestMode()) {
                Serial.println("Relay test is active. Use 'relay test off'.");
            } else {
                motor.stop();
                Serial.println("Motor Stopped");
            }
        }
        else if (input.startsWith("speed ")) {
            int s = input.substring(6).toInt();
            if (s >= 0 && s <= 2) {
                motor.setSpeed((SpeedMode)s);
                Serial.print("Speed set to ");
                Serial.println(speedName(motor.getSpeed()));
            } else {
                Serial.println("Invalid speed index (0-2)");
            }
        }
        else if (input == "s") {
            motor.cycleSpeed();
            Serial.println("Speed Cycled");
        }
        else if (input == "status" || input == "i") {
            printStatus();
        }
        else if (input == "t") {
            motor.toggleStandby();
            Serial.println("Standby Toggled");
        }
        else if (input == "p") {
            motor.resetPitch();
            Serial.println("Pitch Reset");
        }
        else if (input == "f") {
            Serial.println("Factory Resetting...");
            settings.factoryReset();
            settings.load();
            motor.endRelayTest();
            motor.applySettings();
            Serial.println("Factory reset complete.");
        }
        else if (input == "help") {
            printHelp();
        }
        else if (input == "save") {
            settings.save(true);
        }
        else if (input == "reboot") {
            Serial.println("Rebooting...");
            Serial.flush();
            hal.watchdogReboot();
        }
        else if (input == "dump settings") {
            printSettingsDump();
        }
        else if (input.startsWith("preset ")) {
            handlePresetCommand(input);
        }
        else if (input.startsWith("relay test")) {
            handleRelayTestCommand(input);
        }
        else if (input == "brake test start") {
            if (motor.isRelayTestMode()) {
                Serial.println("Exit relay test before starting.");
            } else {
                if (motor.isStandby()) motor.toggleStandby();
                motor.start();
                Serial.println("Brake test motor start requested.");
            }
        }
        else if (input == "brake test stop") {
            if (motor.isRunning()) {
                motor.stop();
                Serial.println("Brake stop requested.");
            } else {
                Serial.println("Motor is not running.");
            }
        }
        else if (input == "error dump") {
            errorHandler.dumpLog(Serial);
        }
        else if (input == "error clear") {
            errorHandler.clearLogs();
            Serial.println("Error Log Cleared");
        }
        
        // --- Registry Commands ---
        else if (input == "list") {
            Serial.println("--- Settings List ---");
            for (const auto& item : registry) {
                Serial.print(item.name);
                Serial.print(" = ");
                Serial.println(item.get());
            }
            Serial.println("---------------------");
        }
        else if (input.startsWith("set ")) {
            int firstSpace = input.indexOf(' ');
            int secondSpace = input.indexOf(' ', firstSpace + 1);
            
            if (secondSpace > 0) {
                String key = input.substring(firstSpace + 1, secondSpace);
                String valStr = input.substring(secondSpace + 1);
                
                bool found = false;
                for (const auto& item : registry) {
                    if (item.name == key) {
                        item.set(valStr);
                        Serial.print("Set "); Serial.print(key); Serial.print(" = "); Serial.println(valStr);
                        found = true;
                        break;
                    }
                }
                if (!found) Serial.println("Unknown setting key");
            } else {
                Serial.println("Usage: set <key> <value>");
            }
        }
        else if (input.startsWith("get ")) {
            String key = input.substring(4);
            bool found = false;
            for (const auto& item : registry) {
                if (item.name == key) {
                    Serial.println(item.get());
                    found = true;
                    break;
                }
            }
            if (!found) Serial.println("Unknown setting key");
        }
        
        // --- Preset Import/Export ---
        else if (input.startsWith("export preset ")) {
            int slot = input.substring(14).toInt() - 1; // 1-based to 0-based
            if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
                String out;
                if (settings.exportPresetToJSON(slot, out)) {
                    Serial.println(out);
                } else {
                    Serial.println("Error exporting preset.");
                }
            } else {
                Serial.println("Invalid preset slot (1-5)");
            }
        }
        else if (input.startsWith("import preset ")) {
            int firstSpace = input.indexOf(' ', 14);
            if (firstSpace > 14) {
                int slot = input.substring(14, firstSpace).toInt() - 1;
                String jsonStr = input.substring(firstSpace + 1);
                
                if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
                    if (settings.importPresetFromJSON(slot, jsonStr)) {
                        Serial.println("Preset imported successfully.");
                    } else {
                        Serial.println("Failed to import preset.");
                    }
                } else {
                    Serial.println("Invalid preset slot (1-5)");
                }
            } else {
                Serial.println("Usage: import preset <1-5> <json_string>");
            }
        }
        
        // --- UI Injection ---
        else if (input == "j") ui.injectInput(-1, false);
        else if (input == "l") ui.injectInput(1, false);
        else if (input == "k") ui.injectInput(0, true);
        else if (input == "m") ui.enterMenu();
        
        else {
            Serial.println("Unknown command. Type 'help' for list.");
        }
    }
}

void printStatus() {
    Serial.println("--- TT Control Status ---");
    Serial.print("State: ");
    Serial.println(motorStateName());
    
    Serial.print("Speed Mode: ");
    Serial.println(speedName(motor.getSpeed()));
    
    Serial.print("Frequency: ");
    Serial.print(motor.getCurrentFrequency());
    Serial.println(" Hz");
    
    Serial.print("Pitch: ");
    Serial.print(currentPitchPercent);
    Serial.println("%");

    Serial.print("Brake: ");
    Serial.println(brakeModeName(settings.get().brakeMode));
    
    Serial.println("-------------------------");
}

void printHelp() {
    if (!cliInitialized) initCLI();
    
    Serial.println("Available Commands:");
    Serial.println("start, stop, t (standby)");
    Serial.println("speed <0-2>, s (cycle)");
    Serial.println("status, p (reset pitch)");
    Serial.println("list - List all settings");
    Serial.println("set <key> <val> - Set setting");
    Serial.println("get <key> - Get setting");
    Serial.println("save, reboot, dump settings");
    Serial.println("preset list|load <1-5>|save <1-5>");
    Serial.println("export preset <1-5> - Dump JSON");
    Serial.println("import preset <1-5> <json> - Load JSON");
    Serial.println("brake test start|stop");
    Serial.println("relay test <0-N|off>");
    Serial.println("error dump, error clear");
    Serial.println("f - Factory Reset");
}

static void printPresetList() {
    Serial.println("--- Presets ---");
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(settings.getPresetName(i));
    }
}

static void printSettingsDump() {
    GlobalSettings& g = settings.get();

    Serial.println("--- Settings Dump ---");
    Serial.print("Schema: "); Serial.println(g.schemaVersion);
    Serial.print("Phase Mode: "); Serial.println(g.phaseMode);
    Serial.print("Max Amp: "); Serial.print(g.maxAmplitude); Serial.println("%");
    Serial.print("Ramp Type: "); Serial.println(rampTypeName(g.rampType));
    Serial.print("Smooth Switch: "); Serial.println(g.smoothSwitching ? "ON" : "OFF");
    Serial.print("Brake Mode: "); Serial.println(brakeModeName(g.brakeMode));
    Serial.print("Brake Duration: "); Serial.print(g.brakeDuration); Serial.println("s");
    Serial.print("Relay Active High: "); Serial.println(g.relayActiveHigh ? "YES" : "NO");
    Serial.print("Boot Speed: "); Serial.println(g.bootSpeed);
    Serial.print("Runtime: "); Serial.print(settings.getTotalRuntime()); Serial.println("s");

    for (int i = 0; i < 3; i++) {
        SpeedSettings& s = g.speeds[i];
        Serial.print("Speed ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(speedName((SpeedMode)i));
        Serial.print(": ");
        Serial.print(s.frequency);
        Serial.print("Hz, ");
        Serial.print(s.reducedAmplitude);
        Serial.print("% amp, filter ");
        Serial.println(filterName(s.filterType));
    }
    Serial.println("---------------------");
}

static void handlePresetCommand(const String& input) {
    if (input == "preset list") {
        printPresetList();
        return;
    }

    if (input.startsWith("preset load ")) {
        int slot = input.substring(12).toInt() - 1;
        if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
            if (settings.loadPreset(slot)) {
                motor.applySettings();
                Serial.println("Preset loaded. Use 'save' to make it the boot settings.");
            } else {
                Serial.println("Preset slot is empty.");
            }
        } else {
            Serial.println("Invalid preset slot (1-5).");
        }
        return;
    }

    if (input.startsWith("preset save ")) {
        int slot = input.substring(12).toInt() - 1;
        if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
            settings.savePreset(slot);
            Serial.println("Preset saved.");
        } else {
            Serial.println("Invalid preset slot (1-5).");
        }
        return;
    }

    Serial.println("Usage: preset list|load <1-5>|save <1-5>");
}

static void handleRelayTestCommand(const String& input) {
    String arg = "";
    if (input.length() > 10) {
        arg = input.substring(10);
        arg.trim();
    }

    if (arg == "off") {
        motor.endRelayTest();
        Serial.println("Relay test off.");
        return;
    }

    int stage = -1;
    bool numeric = arg.length() > 0;
    for (size_t i = 0; i < arg.length(); i++) {
        char c = arg.charAt(i);
        if (c < '0' || c > '9') {
            numeric = false;
            break;
        }
    }

    if (numeric) {
        stage = arg.toInt();
    }

    if (stage < 0 || stage >= motor.getRelayTestStageCount()) {
        Serial.print("Usage: relay test <0-");
        Serial.print(motor.getRelayTestStageCount() - 1);
        Serial.println("|off>");
        return;
    }

    if (!motor.isRelayTestMode() && !motor.beginRelayTest()) {
        Serial.println("Stop motor before relay test.");
        return;
    }

    motor.setRelayTestStage((uint8_t)stage);
    Serial.print("Relay test stage ");
    Serial.println(stage);
}
