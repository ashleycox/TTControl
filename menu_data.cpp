/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "menu_data.h"
#include "ui.h"
#include "motor.h"
#include "error_handler.h"
#include "settings.h" // Added missing include
#include <vector>

extern UserInterface ui;
extern MotorController motor;

// Shadow State
// We use a shadow copy of settings for editing, allowing the user to "Save" or "Cancel" changes.
char speedLabelBuffer[32];

void updateSpeedLabel() {
    if (menuShadowSpeedIndex == 0) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 33");
    else if (menuShadowSpeedIndex == 1) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 45");
    else snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 78");
}

void initMenuState() {
    menuShadowSpeedIndex = (int)motor.getSpeed();
    menuShadowSettings = settings.get().speeds[menuShadowSpeedIndex];
    updateSpeedLabel();
}

void actionNextSpeed() {
    // Save current shadow state back to the temporary array
    settings.get().speeds[menuShadowSpeedIndex] = menuShadowSettings;
    
    // Cycle to next speed index
    menuShadowSpeedIndex++;
    if (menuShadowSpeedIndex > 2) menuShadowSpeedIndex = 0;
    
    // Skip 78 RPM if disabled
    if (menuShadowSpeedIndex == 2 && !settings.get().enable78rpm) {
        menuShadowSpeedIndex = 0;
    }
    
    // Load new speed settings into shadow
    menuShadowSettings = settings.get().speeds[menuShadowSpeedIndex];
    updateSpeedLabel();
}


void actionSaveExit() {
    // Commit all changes to persistent storage
    settings.save();
    ui.exitMenu();
}

void actionCancelExit() {
    // Discard changes by reloading from persistent storage
    settings.load();
    ui.exitMenu();
}

void actionFactoryReset() {
    ui.showConfirm("Factory Reset?", [](){
        settings.factoryReset();
        settings.load();
        ui.exitMenu();
    });
}

// --- Error Log Actions ---

void actionClearLog() {
    errorHandler.clearLogs();
    ui.back(); // Return to refresh the menu
    ui.showError("Log Cleared", 2000);
}

void actionEnterErrorLog() {
    // Create page on demand if needed (or reuse global)
    if (!pageErrorLog) pageErrorLog = new MenuPage("Error Log");
    
    // Clear previous items (important for dynamic content)
    pageErrorLog->clear();
    
    // Add Clear Action
    pageErrorLog->addItem(new MenuAction("Clear Log", actionClearLog));
    
    // Fetch and add log lines
    std::vector<String> lines;
    errorHandler.getLogLines(lines);
    
    if (lines.empty()) {
        pageErrorLog->addItem(new MenuInfo("No Errors"));
    } else {
        for (const auto& line : lines) {
            // Use MenuDynamicInfo to manage string memory
            pageErrorLog->addItem(new MenuDynamicInfo(line));
        }
    }
    
    pageErrorLog->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageErrorLog);
}

// --- Preset Actions ---

// Helper to build the actions for a specific slot
void buildPresetSlotMenu(int slot) {
    static int currentSlot = 0;
    currentSlot = slot;
    static MenuPage* pageSlot = nullptr;
    
    char title[32];
    snprintf(title, sizeof(title), "Slot %d: %s", slot + 1, settings.getPresetName(slot));
    
    if (pageSlot) delete pageSlot; // Rebuild
    pageSlot = new MenuPage(title);
    
    // Load Action
    pageSlot->addItem(new MenuAction("Load", [](){
        if (settings.loadPreset(currentSlot)) {
            ui.showMessage("Loaded!", 2000);
            ui.exitMenu(); // Exit to apply
        } else {
            ui.showError("Empty Slot", 2000);
        }
    }));
    
    // Save Action
    pageSlot->addItem(new MenuAction("Save", [](){
        ui.showConfirm("Overwrite?", [](){
            settings.savePreset(currentSlot);
            ui.showMessage("Saved!", 2000);
            ui.back();
        });
    }));
    
    // Rename Action
    // We need a persistent buffer for the name editing
    static char nameBuffer[17];
    strncpy(nameBuffer, settings.getPresetName(slot), 16);
    nameBuffer[16] = 0;
    
    pageSlot->addItem(new MenuText("Rename", nameBuffer, 16));
    
    // Save Name Action (Explicit or auto-save on back?)
    // MenuText saves to buffer on exit. We need to commit it to settings.
    pageSlot->addItem(new MenuAction("Apply Name", [](){
        settings.renamePreset(currentSlot, nameBuffer);
        ui.showMessage("Renamed!", 1000);
    }));
    
    // Reset Action
    pageSlot->addItem(new MenuAction("Clear", [](){
        ui.showConfirm("Clear Slot?", [](){
            settings.resetPreset(currentSlot);
            ui.showMessage("Cleared!", 2000);
            ui.back();
        });
    }));
    
    pageSlot->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageSlot);
}

void actionEnterPresets() {
    // Populate the Presets page dynamically
    pagePresets->clear();
    
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        // Create a dynamic label for the slot
        char label[32];
        snprintf(label, sizeof(label), "%d: %s", i + 1, settings.getPresetName(i));
        
        // Use lambda capture to bind the slot index
        pagePresets->addItem(new MenuAction(label, [i](){ buildPresetSlotMenu(i); }));
    }
    
    pagePresets->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pagePresets);
}

// --- Menu Builder ---


void buildMenuSystem() {
    // --- Speed Tuning Page (Per-Speed) ---
    pageSpeedTuning = new MenuPage("Speed Tuning");
    pageSpeedTuning->addItem(new MenuFloat("Frequency", &menuShadowSettings.frequency, 0.1, 10.0, 3000.0));
    pageSpeedTuning->addItem(new MenuFloat("Min Freq", &menuShadowSettings.minFrequency, 0.1, 10.0, 3000.0));
    pageSpeedTuning->addItem(new MenuFloat("Max Freq", &menuShadowSettings.maxFrequency, 0.1, 10.0, 3000.0));
    pageSpeedTuning->addItem(new MenuInt("Filt Type", (int*)&menuShadowSettings.filterType, 0, 2));
    pageSpeedTuning->addItem(new MenuFloat("IIR Alpha", &menuShadowSettings.iirAlpha, 0.01, 0.01, 0.99));
    pageSpeedTuning->addItem(new MenuInt("FIR Prof", (int*)&menuShadowSettings.firProfile, 0, 2));
    pageSpeedTuning->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Phase Page (Mixed) ---
    pagePhase = new MenuPage("Phase Control");
    pagePhase->addItem(new MenuInt("Mode (Glb)", (int*)&settings.get().phaseMode, 1, 4));
    pagePhase->addItem(new MenuFloat("Ph 2 Offs", &menuShadowSettings.phaseOffset[1], 0.1, -360.0, 360.0));
    pagePhase->addItem(new MenuFloat("Ph 3 Offs", &menuShadowSettings.phaseOffset[2], 0.1, -360.0, 360.0));
    pagePhase->addItem(new MenuFloat("Ph 4 Offs", &menuShadowSettings.phaseOffset[3], 0.1, -360.0, 360.0));
    pagePhase->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Motor Page (Mixed) ---
    pageMotor = new MenuPage("Motor Control");
    // Per-Speed
    pageMotor->addItem(new MenuFloat("Soft Start", &menuShadowSettings.softStartDuration, 0.1, 0.0, 10.0));
    pageMotor->addItem(new MenuInt("Red. Amp %", (int*)&menuShadowSettings.reducedAmplitude, 50, 100));
    pageMotor->addItem(new MenuInt("Amp Delay", (int*)&menuShadowSettings.amplitudeDelay, 0, 60));
    pageMotor->addItem(new MenuInt("Kick Mult", (int*)&menuShadowSettings.startupKick, 1, 4));
    pageMotor->addItem(new MenuInt("Kick Dur", (int*)&menuShadowSettings.startupKickDuration, 0, 15));
    pageMotor->addItem(new MenuFloat("Kick Ramp", &menuShadowSettings.startupKickRampDuration, 0.1, 0.0, 15.0));
    // Global
    pageMotor->addItem(new MenuInt("FDA % (0=Off)", (int*)&settings.get().freqDependentAmplitude, 0, 100));
    pageMotor->addItem(new MenuInt("Max Amp %", (int*)&settings.get().maxAmplitude, 0, 100));
    pageMotor->addItem(new MenuInt("SS Curve", (int*)&settings.get().softStartCurve, 0, 2));
    pageMotor->addItem(new MenuBool("Smooth Sw", &settings.get().smoothSwitching));
    pageMotor->addItem(new MenuInt("Sw Ramp", (int*)&settings.get().switchRampDuration, 1, 5));
    pageMotor->addItem(new MenuInt("Brake Mode", (int*)&settings.get().brakeMode, 0, 2));
    pageMotor->addItem(new MenuFloat("Brake Dur", &settings.get().brakeDuration, 0.1, 0.0, 10.0));
    pageMotor->addItem(new MenuFloat("Brk Pulse", &settings.get().brakePulseGap, 0.1, 0.1, 2.0));
    pageMotor->addItem(new MenuFloat("Brk StartF", &settings.get().brakeStartFreq, 1.0, 10.0, 200.0));
    pageMotor->addItem(new MenuFloat("Brk StopF", &settings.get().brakeStopFreq, 1.0, 0.0, 50.0));
    pageMotor->addItem(new MenuBool("Auto Start", &settings.get().autoStart));
    pageMotor->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Power Page (Global) ---
    pagePower = new MenuPage("Power Control");
    
    if (ENABLE_MUTE_RELAYS) {
        pagePower->addItem(new MenuBool("Rly: ActHi", &settings.get().relayActiveHigh));
        if (ENABLE_STANDBY) {
            pagePower->addItem(new MenuBool("Rly: Stby", &settings.get().muteRelayLinkStandby));
        }
        pagePower->addItem(new MenuBool("Rly: S/S", &settings.get().muteRelayLinkStartStop));
        pagePower->addItem(new MenuInt("Rly: Delay", (int*)&settings.get().powerOnRelayDelay, 0, 10));
    }
    
    if (ENABLE_STANDBY) {
        pagePower->addItem(new MenuInt("Auto Stby", (int*)&settings.get().autoStandbyDelay, 0, 60));
    }
    
    pagePower->addItem(new MenuBool("Auto Boot", &settings.get().autoBoot));
    pagePower->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Display Page (Global) ---
    pageDisplay = new MenuPage("Display");
    pageDisplay->addItem(new MenuInt("Brightness", (int*)&settings.get().displayBrightness, 0, 255));
    pageDisplay->addItem(new MenuInt("Sleep Dly", (int*)&settings.get().displaySleepDelay, 0, 6));
    pageDisplay->addItem(new MenuBool("Scrn Saver", &settings.get().screensaverEnabled));
    pageDisplay->addItem(new MenuInt("Saver Mode", (int*)&settings.get().screensaverMode, 0, 2)); // 0=Bounce, 1=Matrix, 2=Lissajous
    pageDisplay->addItem(new MenuInt("Auto Dim", (int*)&settings.get().autoDimDelay, 0, 60));
    pageDisplay->addItem(new MenuBool("Show Runtime", &settings.get().showRuntime));
    pageDisplay->addItem(new MenuBool("Err Display", &settings.get().errorDisplayEnabled));
    pageDisplay->addItem(new MenuInt("Err Dur", (int*)&settings.get().errorDisplayDuration, 1, 60));
    pageDisplay->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- System Page (Global) ---
    pageSystem = new MenuPage("System");
    pageSystem->addItem(new MenuInfo("Ver: " FIRMWARE_VERSION));
    pageSystem->addItem(new MenuBool("Rev Encoder", &settings.get().reverseEncoder));
    pageSystem->addItem(new MenuFloat("Pitch Step", &settings.get().pitchStepSize, 0.01, 0.01, 1.0));
    pageSystem->addItem(new MenuBool("Pitch Reset", &settings.get().pitchResetOnStop));
    pageSystem->addItem(new MenuBool("Enable 78", &settings.get().enable78rpm));
    pageSystem->addItem(new MenuAction("Error Log", actionEnterErrorLog));
    pageSystem->addItem(new MenuAction("Reset Runtime", [](){
        ui.showConfirm("Reset Runtime?", [](){
            settings.resetTotalRuntime();
            ui.showMessage("Runtime Reset", 2000);
            ui.back();
        });
    }));
    pageSystem->addItem(new MenuInt("Boot Speed", (int*)&settings.get().bootSpeed, 0, 3)); // 0=33, 1=45, 2=78, 3=Last
    pageSystem->addItem(new MenuAction("Fact Reset", actionFactoryReset));
    pageSystem->addItem(new MenuAction("Back", [](){ ui.back(); }));
    
    // --- Presets Page ---
    pagePresets = new MenuPage("Presets");
    // Items are populated dynamically in actionEnterPresets
    pagePresets->addItem(new MenuAction("Back", [](){ ui.back(); }));
    
    // --- Main Menu ---
    pageMain = new MenuPage("Main Menu");
    // The first item toggles which speed we are editing in the submenus
    pageMain->addItem(new MenuAction(speedLabelBuffer, actionNextSpeed)); 
    
    pageMain->addItem(new MenuNav("Speed Tuning", pageSpeedTuning));
    pageMain->addItem(new MenuNav("Phase", pagePhase));
    pageMain->addItem(new MenuNav("Motor", pageMotor));
    pageMain->addItem(new MenuNav("Power", pagePower));
    pageMain->addItem(new MenuNav("Display", pageDisplay));
    pageMain->addItem(new MenuNav("System", pageSystem));
    pageMain->addItem(new MenuAction("Presets", actionEnterPresets)); 
    
    pageMain->addItem(new MenuAction("Save & Exit", actionSaveExit));
    pageMain->addItem(new MenuAction("Cancel", actionCancelExit));
}
