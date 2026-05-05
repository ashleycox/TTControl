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
#include "settings.h"
#include "network_manager.h"
#include <vector>

extern UserInterface ui;
extern MotorController motor;

// Shadow State
// We use a shadow copy of settings for editing, allowing the user to "Save" or "Cancel" changes.
char speedLabelBuffer[32];
MenuItem* menuSpeedSelector = nullptr;
MenuPage* pageNetwork = nullptr;

// --- Sweep Diagnostic ---
float sweepMinSep = 80.0;
float sweepMaxSep = 100.0;
float sweepSpeed = 1.0;
MenuPage* pageSweep = nullptr;

static const char* const phaseModeLabels[] = {"-", "1P", "2P", "3P", "4P"};
static const char* const filterLabels[] = {"None", "IIR", "FIR"};
static const char* const firLabels[] = {"Gentle", "Medium", "Agg"};
static const char* const softStartCurveLabels[] = {"Linear", "Log", "Exp"};
static const char* const rampTypeLabels[] = {"Linear", "S-Curve"};
static const char* const brakeModeLabels[] = {"Off", "Pulse", "Ramp", "Soft"};
static const char* const saverModeLabels[] = {"Bounce", "Matrix", "Liss"};
static const char* const sleepDelayLabels[] = {"Off", "10s", "20s", "30s", "1m", "5m", "10m"};
static const char* const bootSpeedLabels[] = {"33", "45", "78", "Last"};
static const char* const networkModeLabels[] = {"Setup AP", "Station", "STA+AP"};
static const char* const webHomeLabels[] = {"Dash", "Control", "Settings", "Cal", "Network", "Presets", "Bench", "Diag", "Errors"};

#if ENABLE_STANDBY && ENABLE_MUTE_RELAYS && ENABLE_DPDT_RELAYS
static const char* const relayTestLabels[] = {"All Off", "Standby", "DPDT 1", "DPDT 2"};
#elif ENABLE_STANDBY && ENABLE_MUTE_RELAYS
#if ENABLE_4_CHANNEL_SUPPORT
static const char* const relayTestLabels[] = {"All Off", "Standby", "Mute A", "Mute B", "Mute C", "Mute D"};
#else
static const char* const relayTestLabels[] = {"All Off", "Standby", "Mute A", "Mute B", "Mute C"};
#endif
#elif ENABLE_MUTE_RELAYS && ENABLE_DPDT_RELAYS
static const char* const relayTestLabels[] = {"All Off", "DPDT 1", "DPDT 2"};
#elif ENABLE_MUTE_RELAYS
#if ENABLE_4_CHANNEL_SUPPORT
static const char* const relayTestLabels[] = {"All Off", "Mute A", "Mute B", "Mute C", "Mute D"};
#else
static const char* const relayTestLabels[] = {"All Off", "Mute A", "Mute B", "Mute C"};
#endif
#elif ENABLE_STANDBY
static const char* const relayTestLabels[] = {"All Off", "Standby"};
#else
static const char* const relayTestLabels[] = {"All Off"};
#endif
static const size_t relayTestLabelCount = sizeof(relayTestLabels) / sizeof(relayTestLabels[0]);
static uint8_t relayTestStage = 0;

void updateSpeedLabel() {
    if (menuShadowSpeedIndex == 0) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 33");
    else if (menuShadowSpeedIndex == 1) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 45");
    else snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 78");

    if (menuSpeedSelector) {
        menuSpeedSelector->setLabel(speedLabelBuffer);
    }
}

void initMenuState() {
    menuShadowSpeedIndex = (int)motor.getSpeed();
    menuShadowSettings = settings.get().speeds[menuShadowSpeedIndex];
    updateSpeedLabel();
}

void actionNextSpeed() {
    // Save current shadow state back to the temporary array
    commitMenuShadowSettings();

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


void commitMenuShadowSettings() {
    if (menuShadowSpeedIndex >= 0 && menuShadowSpeedIndex < 3) {
        settings.get().speeds[menuShadowSpeedIndex] = menuShadowSettings;
    }
}

void saveMenuChangesAndExit() {
    commitMenuShadowSettings();
    motor.endRelayTest();
    settings.normalize();
    motor.applySettings();
    settings.save();
    ui.exitMenu();
}

void cancelMenuChangesAndExit() {
    settings.load();
    motor.endRelayTest();
    motor.applySettings();
    ui.exitMenu();
}

void actionSaveExit() {
    saveMenuChangesAndExit();
}

void actionCancelExit() {
    cancelMenuChangesAndExit();
}

void actionFactoryReset() {
    ui.showConfirm("Factory Reset?", [](){
        settings.factoryReset();
        settings.load();
        motor.endRelayTest();
        motor.applySettings();
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
            motor.applySettings();
            settings.save();
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

void actionEnterSweep() {
    if (settings.get().phaseMode == 4) {
        ui.showError("N/A 4-Phase", 2000);
        return;
    }

    if (!pageSweep) pageSweep = new MenuPage("Symmetric Sweep");
    pageSweep->clear();
    pageSweep->addItem(new MenuFloat("Min Sep", &sweepMinSep, 1.0, 0.0, 180.0));
    pageSweep->addItem(new MenuFloat("Max Sep", &sweepMaxSep, 1.0, 0.0, 180.0));
    pageSweep->addItem(new MenuFloat("Speed/s", &sweepSpeed, 0.1, 0.1, 10.0));
    pageSweep->addItem(new MenuAction("Start Sweep", [](){
        motor.startSymmetricSweep(sweepMinSep, sweepMaxSep, sweepSpeed);
        // UI intercepts draw and input now
    }));
    pageSweep->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageSweep);
}

void actionEnterBrakeTune() {
    if (!pageBrakeTune) pageBrakeTune = new MenuPage("Brake Tune");
    pageBrakeTune->clear();

    pageBrakeTune->addItem(new MenuInfo("Tune, Test, Save"));
    pageBrakeTune->addItem(new MenuByte("Mode", &settings.get().brakeMode, 0, 3, brakeModeLabels, 4));

    MenuItem* brakeDuration = new MenuFloat("Duration", &settings.get().brakeDuration, 0.1, 0.0, 10.0);
    brakeDuration->setVisibleWhen([](){ return settings.get().brakeMode != BRAKE_OFF; });
    pageBrakeTune->addItem(brakeDuration);

    MenuItem* brakePulse = new MenuFloat("Pulse Gap", &settings.get().brakePulseGap, 0.1, 0.1, 2.0);
    brakePulse->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_PULSE; });
    pageBrakeTune->addItem(brakePulse);

    MenuItem* brakeStart = new MenuFloat("Start Hz", &settings.get().brakeStartFreq, 1.0, 10.0, 200.0);
    brakeStart->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageBrakeTune->addItem(brakeStart);

    MenuItem* brakeStop = new MenuFloat("Stop Hz", &settings.get().brakeStopFreq, 1.0, 0.0, 50.0);
    brakeStop->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageBrakeTune->addItem(brakeStop);

    MenuItem* brakeCutoff = new MenuFloat("Cutoff Hz", &settings.get().softStopCutoff, 1.0, 0.0, 50.0);
    brakeCutoff->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_SOFT_STOP; });
    pageBrakeTune->addItem(brakeCutoff);

    pageBrakeTune->addItem(new MenuAction("Start Motor", [](){
        if (motor.isStandby()) motor.toggleStandby();
        motor.start();
        ui.showMessage("Motor Starting", 1000);
    }));

    pageBrakeTune->addItem(new MenuAction("Brake Stop", [](){
        if (motor.isRunning()) {
            motor.stop();
            ui.showMessage("Brake Test", 1000);
        } else {
            ui.showError("Not Running", 1000);
        }
    }));

    pageBrakeTune->addItem(new MenuAction("Save Brake", [](){
        settings.save();
        ui.showMessage("Brake Saved", 1000);
    }));

    pageBrakeTune->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageBrakeTune);
}

void actionEnterRelayTest() {
    if (!motor.beginRelayTest()) {
        ui.showError("Stop Motor First", 2000);
        return;
    }

    relayTestStage = 0;
    if (!pageRelayTest) pageRelayTest = new MenuPage("Relay Test");
    pageRelayTest->clear();

    pageRelayTest->addItem(new MenuInfo("Waveform Off"));
    uint8_t maxStage = motor.getRelayTestStageCount();
    if (maxStage > 0) maxStage--;
    pageRelayTest->addItem(new MenuByte("Output", &relayTestStage, 0, maxStage,
        relayTestLabels, relayTestLabelCount,
        [](uint8_t stage) { motor.setRelayTestStage(stage); }));

    pageRelayTest->addItem(new MenuAction("All Off", [](){
        relayTestStage = 0;
        motor.setRelayTestStage(0);
    }));

    pageRelayTest->addItem(new MenuAction("Exit Test", [](){
        motor.endRelayTest();
        ui.back();
    }));

    ui.navigateTo(pageRelayTest);
}

void actionEnterNetwork() {
    if (!networkManager.isAvailable()) {
        ui.showError("No Wi-Fi", 2000);
        return;
    }

    if (!pageNetwork) pageNetwork = new MenuPage("Network");
    pageNetwork->clear();

    NetworkConfig& cfg = networkManager.getConfig();
    char status[32];
    snprintf(status, sizeof(status), "Status: %s", networkManager.statusText());
    pageNetwork->addItem(new MenuDynamicInfo(String(status)));

    String ip = networkManager.ipText();
    if (ip.length() == 0) ip = "No IP";
    pageNetwork->addItem(new MenuDynamicInfo(String("Web: ") + ip));

    pageNetwork->addItem(new MenuBool("Wi-Fi", &cfg.enabled));
    pageNetwork->addItem(new MenuByte("Mode", &cfg.mode, NETWORK_MODE_AP, NETWORK_MODE_STA_AP, networkModeLabels, 3));
    pageNetwork->addItem(new MenuText("Host", cfg.hostname, NETWORK_HOSTNAME_MAX));
    pageNetwork->addItem(new MenuText("SSID", cfg.ssid, NETWORK_SSID_MAX));
    pageNetwork->addItem(new MenuText("Pass", cfg.password, NETWORK_PASSWORD_MAX));
    pageNetwork->addItem(new MenuBool("DHCP", &cfg.dhcp));
    pageNetwork->addItem(new MenuBool("AP Fallback", &cfg.apFallback));
    pageNetwork->addItem(new MenuText("AP SSID", cfg.apSsid, NETWORK_SSID_MAX));
    pageNetwork->addItem(new MenuText("AP Pass", cfg.apPassword, NETWORK_PASSWORD_MAX));
    pageNetwork->addItem(new MenuByte("AP Channel", &cfg.apChannel, 1, 13));
    pageNetwork->addItem(new MenuBool("ReadOnly", &cfg.readOnlyMode));
    pageNetwork->addItem(new MenuText("Web PIN", cfg.webPin, NETWORK_WEB_PIN_MAX));
    pageNetwork->addItem(new MenuByte("Web Home", &cfg.webHomePage, 0, WEB_HOME_PAGE_COUNT - 1, webHomeLabels, WEB_HOME_PAGE_COUNT));
    pageNetwork->addItem(new MenuAction("Reset PIN", [](){
        NetworkConfig& cfg = networkManager.getConfig();
        strncpy(cfg.webPin, NETWORK_DEFAULT_WEB_PIN, NETWORK_WEB_PIN_MAX);
        cfg.webPin[NETWORK_WEB_PIN_MAX] = 0;
        ui.showMessage("PIN Reset", 1200);
    }));

    pageNetwork->addItem(new MenuAction("Apply", [](){
        networkManager.save();
        networkManager.restart();
        ui.showMessage("Network Applied", 1500);
    }));
    pageNetwork->addItem(new MenuAction("Setup AP", [](){
        NetworkConfig& cfg = networkManager.getConfig();
        cfg.enabled = true;
        cfg.mode = NETWORK_MODE_AP;
        networkManager.save();
        networkManager.restart();
        ui.showMessage("Setup AP On", 1500);
    }));
    pageNetwork->addItem(new MenuAction("Refresh", actionEnterNetwork));
    pageNetwork->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageNetwork);
}

// --- Menu Builder ---


void buildMenuSystem() {
    // --- Speed Tuning Page (Per-Speed) ---
    pageSpeedTuning = new MenuPage("Speed Tuning");
    pageSpeedTuning->addItem(new MenuFloat("Frequency", &menuShadowSettings.frequency, 0.1, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ));
    pageSpeedTuning->addItem(new MenuFloat("Min Freq", &menuShadowSettings.minFrequency, 0.1, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ));
    pageSpeedTuning->addItem(new MenuFloat("Max Freq", &menuShadowSettings.maxFrequency, 0.1, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ));
    pageSpeedTuning->addItem(new MenuByte("Filt Type", &menuShadowSettings.filterType, 0, 2, filterLabels, 3));
    MenuItem* iirAlpha = new MenuFloat("IIR Alpha", &menuShadowSettings.iirAlpha, 0.01, 0.01, 0.99);
    iirAlpha->setVisibleWhen([](){ return menuShadowSettings.filterType == FILTER_IIR; });
    pageSpeedTuning->addItem(iirAlpha);
    MenuItem* firProfile = new MenuByte("FIR Prof", &menuShadowSettings.firProfile, 0, 2, firLabels, 3);
    firProfile->setVisibleWhen([](){ return menuShadowSettings.filterType == FILTER_FIR; });
    pageSpeedTuning->addItem(firProfile);
    pageSpeedTuning->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Phase Page (Mixed) ---
    pagePhase = new MenuPage("Phase Control");
    pagePhase->addItem(new MenuByte("Mode (Glb)", &settings.get().phaseMode, 1, MAX_PHASE_MODE, phaseModeLabels, MAX_PHASE_MODE + 1));
    MenuItem* phase2 = new MenuFloat("Ph 2 Offs", &menuShadowSettings.phaseOffset[1], 0.1, -360.0, 360.0);
    phase2->setVisibleWhen([](){ return settings.get().phaseMode >= PHASE_2; });
    pagePhase->addItem(phase2);
    MenuItem* phase3 = new MenuFloat("Ph 3 Offs", &menuShadowSettings.phaseOffset[2], 0.1, -360.0, 360.0);
    phase3->setVisibleWhen([](){ return settings.get().phaseMode >= PHASE_3; });
    pagePhase->addItem(phase3);
    MenuItem* phase4 = new MenuFloat("Ph 4 Offs", &menuShadowSettings.phaseOffset[3], 0.1, -360.0, 360.0);
    phase4->setVisibleWhen([](){ return ENABLE_4_CHANNEL_SUPPORT && settings.get().phaseMode >= PHASE_4; });
    pagePhase->addItem(phase4);
    MenuItem* sweepDiag = new MenuAction("Sweep Diag.", actionEnterSweep);
    sweepDiag->setVisibleWhen([](){ return settings.get().phaseMode == PHASE_2 || settings.get().phaseMode == PHASE_3; });
    pagePhase->addItem(sweepDiag);
    pagePhase->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Motor Page (Mixed) ---
    pageMotor = new MenuPage("Motor Control");
    // Per-Speed
    pageMotor->addItem(new MenuFloat("Soft Start", &menuShadowSettings.softStartDuration, 0.1, 0.0, 10.0));
    pageMotor->addItem(new MenuByte("Red. Amp %", &menuShadowSettings.reducedAmplitude, 10, 100));
    pageMotor->addItem(new MenuByte("Amp Delay", &menuShadowSettings.amplitudeDelay, 0, 60));
    pageMotor->addItem(new MenuByte("Kick Mult", &menuShadowSettings.startupKick, 1, 4));
    MenuItem* kickDuration = new MenuByte("Kick Dur", &menuShadowSettings.startupKickDuration, 0, 15);
    kickDuration->setVisibleWhen([](){ return menuShadowSettings.startupKick > 1; });
    pageMotor->addItem(kickDuration);
    MenuItem* kickRamp = new MenuFloat("Kick Ramp", &menuShadowSettings.startupKickRampDuration, 0.1, 0.0, 15.0);
    kickRamp->setVisibleWhen([](){ return menuShadowSettings.startupKick > 1; });
    pageMotor->addItem(kickRamp);
    // Global
    pageMotor->addItem(new MenuByte("V/f Blend%", &settings.get().freqDependentAmplitude, 0, 100));
    MenuItem* vfLowHz = new MenuFloat("V/f LowHz", &settings.get().vfLowFreq, 1.0, 0.0, 50.0);
    vfLowHz->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotor->addItem(vfLowHz);
    MenuItem* vfLowBoost = new MenuByte("V/f Low%", &settings.get().vfLowBoost, 0, 100);
    vfLowBoost->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotor->addItem(vfLowBoost);
    MenuItem* vfMidHz = new MenuFloat("V/f MidHz", &settings.get().vfMidFreq, 1.0, 0.0, 100.0);
    vfMidHz->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotor->addItem(vfMidHz);
    MenuItem* vfMidBoost = new MenuByte("V/f Mid%", &settings.get().vfMidBoost, 0, 100);
    vfMidBoost->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotor->addItem(vfMidBoost);
    pageMotor->addItem(new MenuByte("Max Amp %", &settings.get().maxAmplitude, 0, 100));
    pageMotor->addItem(new MenuByte("Ramp Type", &settings.get().rampType, 0, 1, rampTypeLabels, 2));
    MenuItem* softStartCurve = new MenuByte("SS Curve", &settings.get().softStartCurve, 0, 2, softStartCurveLabels, 3);
    softStartCurve->setVisibleWhen([](){ return settings.get().rampType == RAMP_LINEAR; });
    pageMotor->addItem(softStartCurve);
    pageMotor->addItem(new MenuBool("Smooth Sw", &settings.get().smoothSwitching));
    MenuItem* switchRamp = new MenuByte("Sw Ramp", &settings.get().switchRampDuration, 1, 5);
    switchRamp->setVisibleWhen([](){ return settings.get().smoothSwitching; });
    pageMotor->addItem(switchRamp);
    pageMotor->addItem(new MenuByte("Brake Mode", &settings.get().brakeMode, 0, 3, brakeModeLabels, 4));
    MenuItem* brakeDuration = new MenuFloat("Brake Dur", &settings.get().brakeDuration, 0.1, 0.0, 10.0);
    brakeDuration->setVisibleWhen([](){ return settings.get().brakeMode != BRAKE_OFF; });
    pageMotor->addItem(brakeDuration);
    MenuItem* brakePulse = new MenuFloat("Brk Pulse", &settings.get().brakePulseGap, 0.1, 0.1, 2.0);
    brakePulse->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_PULSE; });
    pageMotor->addItem(brakePulse);
    MenuItem* brakeStart = new MenuFloat("Brk StartF", &settings.get().brakeStartFreq, 1.0, 10.0, 200.0);
    brakeStart->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageMotor->addItem(brakeStart);
    MenuItem* brakeStop = new MenuFloat("Brk StopF", &settings.get().brakeStopFreq, 1.0, 0.0, 50.0);
    brakeStop->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageMotor->addItem(brakeStop);
    MenuItem* brakeCutoff = new MenuFloat("Brk Cutoff", &settings.get().softStopCutoff, 1.0, 0.0, 50.0);
    brakeCutoff->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_SOFT_STOP; });
    pageMotor->addItem(brakeCutoff);
    pageMotor->addItem(new MenuBool("Auto Start", &settings.get().autoStart));
    pageMotor->addItem(new MenuAction("Brake Tune", actionEnterBrakeTune));
    pageMotor->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Power Page (Global) ---
    pagePower = new MenuPage("Power Control");

    if (ENABLE_MUTE_RELAYS) {
        pagePower->addItem(new MenuBool("Rly: ActHi", &settings.get().relayActiveHigh));
        if (ENABLE_STANDBY) {
            pagePower->addItem(new MenuBool("Rly: Stby", &settings.get().muteRelayLinkStandby));
        }
        pagePower->addItem(new MenuBool("Rly: S/S", &settings.get().muteRelayLinkStartStop));
        pagePower->addItem(new MenuByte("Rly: Delay", &settings.get().powerOnRelayDelay, 0, 10));
    }

    if (ENABLE_STANDBY) {
        pagePower->addItem(new MenuByte("Auto Stby", &settings.get().autoStandbyDelay, 0, 60));
    }

    pagePower->addItem(new MenuBool("Auto Boot", &settings.get().autoBoot));
    if (ENABLE_STANDBY || ENABLE_MUTE_RELAYS) {
        pagePower->addItem(new MenuAction("Relay Test", actionEnterRelayTest));
    }
    pagePower->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Display Page (Global) ---
    pageDisplay = new MenuPage("Display");
    pageDisplay->addItem(new MenuByte("Brightness", &settings.get().displayBrightness, 0, 255));
    pageDisplay->addItem(new MenuByte("Sleep Dly", &settings.get().displaySleepDelay, 0, 6, sleepDelayLabels, 7));
    pageDisplay->addItem(new MenuBool("Scrn Saver", &settings.get().screensaverEnabled));
    MenuItem* saverMode = new MenuByte("Saver Mode", &settings.get().screensaverMode, 0, 2, saverModeLabels, 3);
    saverMode->setVisibleWhen([](){ return settings.get().screensaverEnabled; });
    pageDisplay->addItem(saverMode);
    pageDisplay->addItem(new MenuByte("Auto Dim", &settings.get().autoDimDelay, 0, 60));
    pageDisplay->addItem(new MenuBool("Show Runtime", &settings.get().showRuntime));
    pageDisplay->addItem(new MenuBool("Err Display", &settings.get().errorDisplayEnabled));
    MenuItem* errorDuration = new MenuByte("Err Dur", &settings.get().errorDisplayDuration, 1, 60);
    errorDuration->setVisibleWhen([](){ return settings.get().errorDisplayEnabled; });
    pageDisplay->addItem(errorDuration);
    pageDisplay->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- System Page (Global) ---
    pageSystem = new MenuPage("System");
    pageSystem->addItem(new MenuInfo("Ver: " FIRMWARE_VERSION));
    pageSystem->addItem(new MenuBool("Rev Encoder", &settings.get().reverseEncoder));
    pageSystem->addItem(new MenuFloat("Pitch Step", &settings.get().pitchStepSize, 0.01, 0.01, 1.0));
    pageSystem->addItem(new MenuBool("Pitch Reset", &settings.get().pitchResetOnStop));
    pageSystem->addItem(new MenuBool("Enable 78", &settings.get().enable78rpm));
#if AMP_MONITOR_ENABLE
    pageSystem->addItem(new MenuFloat("Amp Warn C", &settings.get().ampTempWarnC, 1.0, AMP_TEMP_MIN_C, AMP_TEMP_MAX_C));
    pageSystem->addItem(new MenuFloat("Amp Shut C", &settings.get().ampTempShutdownC, 1.0, AMP_TEMP_MIN_C, AMP_TEMP_MAX_C));
#endif
    pageSystem->addItem(new MenuAction("Error Log", actionEnterErrorLog));
    pageSystem->addItem(new MenuAction("Reset Runtime", [](){
        ui.showConfirm("Reset Runtime?", [](){
            settings.resetTotalRuntime();
            ui.showMessage("Runtime Reset", 2000);
            ui.back();
        });
    }));
    pageSystem->addItem(new MenuByte("Boot Speed", &settings.get().bootSpeed, 0, 3, bootSpeedLabels, 4));
    pageSystem->addItem(new MenuAction("Fact Reset", actionFactoryReset));
    pageSystem->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Presets Page ---
    pagePresets = new MenuPage("Presets");
    // Items are populated dynamically in actionEnterPresets
    pagePresets->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Main Menu ---
    pageMain = new MenuPage("Main Menu");

    extern bool safeModeActive;
    if (safeModeActive) {
        pageMain->addItem(new MenuAction("Exit Safe Mode", [](){
            hal.watchdogReboot();
        }));
    }

    // The first item toggles which speed we are editing in the submenus
    menuSpeedSelector = new MenuAction(speedLabelBuffer, actionNextSpeed);
    pageMain->addItem(menuSpeedSelector);

    pageMain->addItem(new MenuNav("Speed Tuning", pageSpeedTuning));
    pageMain->addItem(new MenuNav("Phase", pagePhase));
    pageMain->addItem(new MenuNav("Motor", pageMotor));
    pageMain->addItem(new MenuNav("Power", pagePower));
    pageMain->addItem(new MenuNav("Display", pageDisplay));
    pageMain->addItem(new MenuNav("System", pageSystem));
    if (networkManager.isAvailable()) {
        pageMain->addItem(new MenuAction("Network", actionEnterNetwork));
    }
    pageMain->addItem(new MenuAction("Presets", actionEnterPresets));

    pageMain->addItem(new MenuAction("Save & Exit", actionSaveExit));
    pageMain->addItem(new MenuAction("Cancel", actionCancelExit));
}
