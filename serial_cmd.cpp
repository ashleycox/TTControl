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

extern UserInterface ui;

#include "serial_cmd.h"
#include "ui.h"
#include "error_handler.h"

extern UserInterface ui;

// --- CLI Registry ---
struct SettingItem {
    String name;
    std::function<String()> get;
    std::function<void(String)> set;
};

std::vector<SettingItem> registry;
bool cliInitialized = false;

void initCLI() {
    if (cliInitialized) return;
    
    // --- Global Settings ---
    registry.push_back({ "brightness", 
        []() { return String(settings.get().displayBrightness); },
        [](String v) { settings.get().displayBrightness = v.toInt(); }
    });
    
    registry.push_back({ "ramp", 
        []() { return String(settings.get().rampType); },
        [](String v) { settings.get().rampType = v.toInt(); }
    });
    
    registry.push_back({ "pitch_step", 
        []() { return String(settings.get().pitchStepSize); },
        [](String v) { settings.get().pitchStepSize = v.toFloat(); }
    });
    
    registry.push_back({ "rev_enc", 
        []() { return String(settings.get().reverseEncoder); },
        [](String v) { settings.get().reverseEncoder = (v == "1" || v == "true"); }
    });
    
    registry.push_back({ "saver_mode", 
        []() { return String(settings.get().screensaverMode); },
        [](String v) { settings.get().screensaverMode = v.toInt(); }
    });
    
    // --- Current Speed Settings ---
    // Note: These access the *currently active* speed settings
    
    registry.push_back({ "freq", 
        []() { return String(settings.getCurrentSpeedSettings().frequency); },
        [](String v) { 
            settings.getCurrentSpeedSettings().frequency = v.toFloat(); 
            motor.applySettings(); 
        }
    });
    
    registry.push_back({ "phase1", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[0]); },
        [](String v) { 
            settings.getCurrentSpeedSettings().phaseOffset[0] = v.toFloat(); 
            motor.applySettings(); 
        }
    });
    
    registry.push_back({ "phase2", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[1]); },
        [](String v) { 
            settings.getCurrentSpeedSettings().phaseOffset[1] = v.toFloat(); 
            motor.applySettings(); 
        }
    });
    
    registry.push_back({ "phase3", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[2]); },
        [](String v) { 
            settings.getCurrentSpeedSettings().phaseOffset[2] = v.toFloat(); 
            motor.applySettings(); 
        }
    });
    
    registry.push_back({ "phase4", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[3]); },
        [](String v) { 
            settings.getCurrentSpeedSettings().phaseOffset[3] = v.toFloat(); 
            motor.applySettings(); 
        }
    });
    
    registry.push_back({ "soft_start", 
        []() { return String(settings.getCurrentSpeedSettings().softStartDuration); },
        [](String v) { settings.getCurrentSpeedSettings().softStartDuration = v.toFloat(); }
    });
    
    registry.push_back({ "kick", 
        []() { return String(settings.getCurrentSpeedSettings().startupKick); },
        [](String v) { settings.getCurrentSpeedSettings().startupKick = v.toInt(); }
    });
    
    registry.push_back({ "kick_dur", 
        []() { return String(settings.getCurrentSpeedSettings().startupKickDuration); },
        [](String v) { settings.getCurrentSpeedSettings().startupKickDuration = v.toInt(); }
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
            motor.start();
            Serial.println("Motor Started");
        }
        else if (input == "stop") {
            motor.stop();
            Serial.println("Motor Stopped");
        }
        else if (input.startsWith("speed ")) {
            int s = input.substring(6).toInt();
            if (s >= 0 && s <= 2) {
                motor.setSpeed((SpeedMode)s);
                Serial.print("Speed set to index ");
                Serial.println(s);
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
        }
        else if (input == "help") {
            printHelp();
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
    if (motor.isRunning()) Serial.println("RUNNING");
    else if (motor.isStandby()) Serial.println("STANDBY");
    else Serial.println("STOPPED");
    
    Serial.print("Speed Mode: ");
    SpeedMode s = motor.getSpeed();
    if (s == SPEED_33) Serial.println("33 RPM");
    else if (s == SPEED_45) Serial.println("45 RPM");
    else Serial.println("78 RPM");
    
    Serial.print("Frequency: ");
    Serial.print(motor.getCurrentFrequency());
    Serial.println(" Hz");
    
    Serial.print("Pitch: ");
    Serial.print(currentPitchPercent);
    Serial.println("%");
    
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
    Serial.println("error dump, error clear");
    Serial.println("f - Factory Reset");
}
