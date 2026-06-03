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
#include "speed_feedback.h"
#include <vector>

extern UserInterface ui;
extern MotorController motor;

// Per-speed shadow editing. Speed pages edit menuShadowSettings; Save writes it into settings, Cancel reloads persisted settings.
char speedLabelBuffer[32];
MenuItem* menuSpeedSelector = nullptr;
MenuPage* pageNetwork = nullptr;
MenuPage* pageClosedLoop = nullptr;
static MenuPage* pageMotorStartup = nullptr;
static MenuPage* pageMotorAmplitude = nullptr;
static MenuPage* pageMotorRamping = nullptr;
static MenuPage* pageMotorBraking = nullptr;
static MenuPage* pageNetworkStation = nullptr;
static MenuPage* pageNetworkIpPower = nullptr;
static MenuPage* pageNetworkAccessPoint = nullptr;
static MenuPage* pageNetworkWeb = nullptr;
static char unlockPinBuffer[NETWORK_WEB_PIN_MAX + 1] = "";
static char securityPinBuffer[NETWORK_WEB_PIN_MAX + 1] = "";
#if CLOSED_LOOP_SPEED_ENABLE
static MenuPage* pageClosedLoopControl = nullptr;
static MenuPage* pageClosedLoopSensor = nullptr;
static MenuPage* pageClosedLoopEngage = nullptr;
static MenuPage* pageClosedLoopTuning = nullptr;
static MenuPage* pageClosedLoopPid = nullptr;
static MenuPage* pageClosedLoopRamp = nullptr;
static MenuPage* pageClosedLoopPitch = nullptr;
static MenuPage* pageClosedLoopSafety = nullptr;
static MenuPage* pageClosedLoopTools = nullptr;
static MenuPage* pageClosedLoopActions = nullptr;
static MenuPage* pageClosedLoopSetup = nullptr;
static MenuPage* pageClosedLoopTune = nullptr;
#endif

/*
 * --- Sweep Diagnostic ---
 * These are temporary diagnostic parameters, not persisted settings.
 */
float sweepMinSep = 80.0;
float sweepMaxSep = 100.0;
float sweepSpeed = 1.0;
MenuPage* pageSweep = nullptr;

static const char* const phaseModeLabels[] = {"-", "1P", "2P", "3P", "4P"};
// OLED menu labels are deliberately short enough for a 128x64 display.
static const char* const filterLabels[] = {"None", "IIR", "FIR"};
static const char* const firLabels[] = {"Gentle", "Medium", "Agg"};
static const char* const softStartCurveLabels[] = {"Linear", "Log", "Exp"};
static const char* const rampTypeLabels[] = {"Linear", "S-Curve"};
static const char* const brakeModeLabels[] = {"Off", "Pulse", "Ramp", "Soft"};
static const char* const saverModeLabels[] = {"Bounce", "Matrix", "Liss"};
static const char* const sleepDelayLabels[] = {"Off", "10s", "20s", "30s", "1m", "5m", "10m"};
static const char* const bootSpeedLabels[] = {"33", "45", "78", "Last"};
static const char* const networkModeLabels[] = {"Setup AP", "Station", "STA+AP"};
static const char* const networkStandbyLabels[] = {"Network", "Eco"};
static const char* const webHomeLabels[] = {"Dash", "Control", "Settings", "Cal", "Network", "Presets", "Bench", "Diag", "Errors"};
#if CLOSED_LOOP_SPEED_ENABLE
static const char* const closedLoopControlLabels[] = {"Monitor", "Correct"};
static const char* const closedLoopSensorLabels[] = {"Pulse", "Quad"};
static const char* const closedLoopEdgeLabels[] = {"Rise", "Fall", "Change"};
static const char* const closedLoopQuadLabels[] = {"x1", "x2", "x4"};
static const char* const closedLoopFaultLabels[] = {"Ignore", "Warn", "Stop"};
static const char* const closedLoopDropLabels[] = {"Open", "Hold", "Stop"};
static const char* const closedLoopRampLabels[] = {"Off", "Track"};
static const char* const closedLoopAmpRecoveryLabels[] = {"Off", "Warn", "Restore"};
static const char* const closedLoopPitchTargetLabels[] = {"Fixed", "Follow"};
#endif

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

static MenuPage* rebuildMenuPage(MenuPage*& page, const char* title) {
    // Dynamic pages are rebuilt on entry so status text and conditional items are fresh without leaking old MenuItems.
    if (!page) page = new MenuPage(title);
    page->clear();
    return page;
}

static void addBackItem(MenuPage* page) {
    page->addItem(new MenuAction("Back", [](){ ui.back(); }));
}

static void addNavItem(MenuPage* page,
                       const char* label,
                       MenuPage* target,
                       MenuItem::VisibilityCallback visibleWhen = nullptr) {
    MenuItem* item = new MenuNav(label, target);
    if (visibleWhen) item->setVisibleWhen(visibleWhen);
    page->addItem(item);
}

#if CLOSED_LOOP_SPEED_ENABLE
static void addClosedLoopItem(MenuPage* page, MenuItem* item) {
    // Closed-loop subitems remain hidden until the feature is enabled in settings.
    item->setVisibleWhen([](){ return settings.get().closedLoopEnabled; });
    page->addItem(item);
}
#endif

void updateSpeedLabel() {
    // Main menu's first item doubles as the "which speed am I editing" selector.
    if (menuShadowSpeedIndex == 0) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 33");
    else if (menuShadowSpeedIndex == 1) snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 45");
    else snprintf(speedLabelBuffer, sizeof(speedLabelBuffer), "Edit Speed: 78");

    if (menuSpeedSelector) {
        menuSpeedSelector->setLabel(speedLabelBuffer);
    }
}

void initMenuState() {
    // Called when the menu opens so edits begin from the currently selected motor speed.
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
    // Commit only the speed currently represented by menuShadowSettings.
    if (menuShadowSpeedIndex >= 0 && menuShadowSpeedIndex < 3) {
        settings.get().speeds[menuShadowSpeedIndex] = menuShadowSettings;
    }
}

void saveMenuChangesAndExit() {
    // Normalize and protected-save because these changes can affect hardware startup on the next boot.
    commitMenuShadowSettings();
    motor.endRelayTest();
    settings.normalize();
    motor.applySettings();
    settings.save(false, true);
    ui.exitMenu();
}

void cancelMenuChangesAndExit() {
    // Reload persisted settings to discard shadow edits and any live global menu edits made during this menu session.
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
    // Confirmation keeps the destructive filesystem format behind a second click.
    ui.showConfirm("Factory Reset?", [](){
        settings.factoryReset();
        settings.load();
        motor.endRelayTest();
        motor.applySettings();
        ui.exitMenu();
    });
}

void actionUnlockDevice() {
    // Unlock page uses a separate buffer so failed attempts never overwrite the saved PIN.
    if (networkManager.unlockDevice(unlockPinBuffer)) {
        memset(unlockPinBuffer, 0, sizeof(unlockPinBuffer));
        ui.showMessage("Unlocked", 1000);
        ui.exitMenu();
    } else {
        memset(unlockPinBuffer, 0, sizeof(unlockPinBuffer));
        ui.showMessage("Wrong PIN", 1500);
    }
}

static void saveSecurityPin() {
    if (!networkManager.setWebPin(securityPinBuffer)) {
        ui.showMessage("PIN 4-8 chars", 1500);
        return;
    }
    networkManager.save();
    ui.showMessage("PIN Saved", 1000);
}

static void actionEnterSecurity() {
    // Rebuild this page on every entry so Lock/State lines reflect current state.
    if (!pageSecurity) pageSecurity = new MenuPage("UI Lock");
    pageSecurity->clear();

    const NetworkConfig& cfg = networkManager.getConfig();
    strncpy(securityPinBuffer, cfg.webPin, NETWORK_WEB_PIN_MAX);
    securityPinBuffer[NETWORK_WEB_PIN_MAX] = 0;

    pageSecurity->addItem(new MenuInfo(networkManager.isDeviceLockEnabled() ? "Lock: ON" : "Lock: OFF"));
    pageSecurity->addItem(new MenuInfo(networkManager.isDeviceLocked() ? "State: LOCKED" : "State: OPEN"));
    pageSecurity->addItem(new MenuText("PIN", securityPinBuffer, NETWORK_WEB_PIN_MAX));
    pageSecurity->addItem(new MenuAction("Save PIN", saveSecurityPin));

    MenuAction* enableLock = new MenuAction("Enable Lock", [](){
        networkManager.setDeviceLockEnabled(true);
        networkManager.save();
        ui.showMessage("Lock Enabled", 1000);
        actionEnterSecurity();
    });
    enableLock->setVisibleWhen([](){ return !networkManager.isDeviceLockEnabled(); });
    pageSecurity->addItem(enableLock);

    MenuAction* disableLock = new MenuAction("Disable Lock", [](){
        networkManager.setDeviceLockEnabled(false);
        networkManager.save();
        ui.showMessage("Lock Disabled", 1000);
        actionEnterSecurity();
    });
    disableLock->setVisibleWhen([](){ return networkManager.isDeviceLockEnabled(); });
    pageSecurity->addItem(disableLock);

    MenuAction* lockNow = new MenuAction("Lock Now", [](){
        networkManager.lockDevice();
        ui.showMessage("Locked", 1000);
        ui.exitMenu();
    });
    lockNow->setVisibleWhen([](){ return networkManager.isDeviceLockEnabled(); });
    pageSecurity->addItem(lockNow);

    pageSecurity->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageSecurity);
}

// --- Error Log Actions ---

void actionClearLog() {
    errorHandler.clearLogs();
    // Return to the parent so re-entering the log page rebuilds it empty.
    ui.back();
    ui.showError("Log Cleared", 2000);
}

void actionEnterErrorLog() {
    // Create page on demand and rebuild it from the current log contents.
    if (!pageErrorLog) pageErrorLog = new MenuPage("Error Log");

    pageErrorLog->clear();

    // Add Clear action first so it is reachable even when the log is long.
    pageErrorLog->addItem(new MenuAction("Clear Log", actionClearLog));

    std::vector<String> lines;
    errorHandler.getLogLines(lines);

    if (lines.empty()) {
        pageErrorLog->addItem(new MenuInfo("No Errors"));
    } else {
        for (const auto& line : lines) {
            // MenuDynamicInfo owns a copy of each generated log line.
            pageErrorLog->addItem(new MenuDynamicInfo(line));
        }
    }

    pageErrorLog->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageErrorLog);
}

// --- Preset Actions ---

void buildPresetSlotMenu(int slot) {
    // Rebuild one reusable slot page. Lambdas read currentSlot so the callback always targets the slot displayed in this page.
    static int currentSlot = 0;
    currentSlot = slot;
    static MenuPage* pageSlot = nullptr;

    char title[32];
    snprintf(title, sizeof(title), "Slot %d: %s", slot + 1, settings.getPresetName(slot));

    if (pageSlot) delete pageSlot; // Rebuild
    pageSlot = new MenuPage(title);

    // Loading a preset replaces active settings and protected-saves it as boot settings so the selected preset survives restart.
    pageSlot->addItem(new MenuAction("Load", [](){
        if (settings.loadPreset(currentSlot)) {
            motor.applySettings();
            settings.save(false, true);
            ui.showMessage("Loaded!", 2000);
            ui.exitMenu(); // Exit to apply
        } else {
            ui.showError("Empty Slot", 2000);
        }
    }));

    // Save writes the active settings snapshot to the chosen slot.
    pageSlot->addItem(new MenuAction("Save", [](){
        ui.showConfirm("Overwrite?", [](){
            settings.savePreset(currentSlot);
            ui.showMessage("Saved!", 2000);
            ui.back();
        });
    }));

    // Persistent buffer because MenuText edits in place and the page lives until rebuilt for another slot.
    static char nameBuffer[17];
    strncpy(nameBuffer, settings.getPresetName(slot), 16);
    nameBuffer[16] = 0;

    pageSlot->addItem(new MenuText("Rename", nameBuffer, 16));

    // MenuText only updates nameBuffer; Apply Name commits it to settings.
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
    // Populate the Presets page dynamically so slot names are current.
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
    // Sweep is a live diagnostic. MotorController restores original phase offsets when sweep exits/stops.
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
    // Brake Tune edits global brake settings live so the user can start/stop the motor repeatedly, then explicitly save the final tune.
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
        settings.save(false, true);
        ui.showMessage("Brake Saved", 1000);
    }));

    pageBrakeTune->addItem(new MenuAction("Back", [](){ ui.back(); }));
    ui.navigateTo(pageBrakeTune);
}

void actionEnterRelayTest() {
    // Relay test is refused while the motor is moving and disables waveform output before any relay stage is selected.
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
    // Network pages edit NetworkConfig directly and apply/restart only when the user chooses Apply.
    if (!networkManager.isAvailable()) {
        ui.showError("No Wi-Fi", 2000);
        return;
    }

    rebuildMenuPage(pageNetwork, "Network");
    rebuildMenuPage(pageNetworkStation, "Wi-Fi Station");
    rebuildMenuPage(pageNetworkIpPower, "Net IP/Power");
    rebuildMenuPage(pageNetworkAccessPoint, "Setup AP");
    rebuildMenuPage(pageNetworkWeb, "Web Access");

    NetworkConfig& cfg = networkManager.getConfig();
    char status[32];
    snprintf(status, sizeof(status), "Status: %s", networkManager.statusText());
    pageNetwork->addItem(new MenuDynamicInfo(String(status)));

    String ip = networkManager.ipText();
    if (ip.length() == 0) ip = "No IP";
    pageNetwork->addItem(new MenuDynamicInfo(String("Web: ") + ip));

    pageNetwork->addItem(new MenuBool("Wi-Fi", &cfg.enabled));
    pageNetwork->addItem(new MenuByte("Mode", &cfg.mode, NETWORK_MODE_AP, NETWORK_MODE_STA_AP, networkModeLabels, 3));
    addNavItem(pageNetwork, "Station", pageNetworkStation);
    addNavItem(pageNetwork, "IP/Power", pageNetworkIpPower);
    addNavItem(pageNetwork, "Setup AP", pageNetworkAccessPoint);
    addNavItem(pageNetwork, "Web Access", pageNetworkWeb);

    pageNetworkStation->addItem(new MenuText("SSID", cfg.ssid, NETWORK_SSID_MAX));
    pageNetworkStation->addItem(new MenuBool("Hidden SSID", &cfg.hiddenSsid));
    pageNetworkStation->addItem(new MenuText("Pass", cfg.password, NETWORK_PASSWORD_MAX));
    addBackItem(pageNetworkStation);

    pageNetworkIpPower->addItem(new MenuText("Host", cfg.hostname, NETWORK_HOSTNAME_MAX));
    pageNetworkIpPower->addItem(new MenuBool("DHCP", &cfg.dhcp));
    pageNetworkIpPower->addItem(new MenuBool("AP Fallback", &cfg.apFallback));
    pageNetworkIpPower->addItem(new MenuByte("Standby", &cfg.standbyMode, NETWORK_STANDBY_NETWORK, NETWORK_STANDBY_ECO, networkStandbyLabels, 2));
    addBackItem(pageNetworkIpPower);

    pageNetworkAccessPoint->addItem(new MenuText("AP SSID", cfg.apSsid, NETWORK_SSID_MAX));
    pageNetworkAccessPoint->addItem(new MenuText("AP Pass", cfg.apPassword, NETWORK_PASSWORD_MAX));
    pageNetworkAccessPoint->addItem(new MenuByte("AP Channel", &cfg.apChannel, 1, 13));
    addBackItem(pageNetworkAccessPoint);

    pageNetworkWeb->addItem(new MenuBool("ReadOnly", &cfg.readOnlyMode));
    pageNetworkWeb->addItem(new MenuText("Web PIN", cfg.webPin, NETWORK_WEB_PIN_MAX));
    pageNetworkWeb->addItem(new MenuByte("Web Home", &cfg.webHomePage, 0, WEB_HOME_PAGE_COUNT - 1, webHomeLabels, WEB_HOME_PAGE_COUNT));
    pageNetworkWeb->addItem(new MenuAction("Reset PIN", [](){
        NetworkConfig& cfg = networkManager.getConfig();
        strncpy(cfg.webPin, NETWORK_DEFAULT_WEB_PIN, NETWORK_WEB_PIN_MAX);
        cfg.webPin[NETWORK_WEB_PIN_MAX] = 0;
        ui.showMessage("PIN Reset", 1200);
    }));
    addBackItem(pageNetworkWeb);

    pageNetwork->addItem(new MenuAction("Apply", [](){
        networkManager.save();
        networkManager.restart();
        ui.showMessage("Network Applied", 1500);
    }));
    pageNetwork->addItem(new MenuAction("Force AP", [](){
        NetworkConfig& cfg = networkManager.getConfig();
        cfg.enabled = true;
        cfg.mode = NETWORK_MODE_AP;
        networkManager.save();
        networkManager.restart();
        ui.showMessage("Setup AP On", 1500);
    }));
    pageNetwork->addItem(new MenuAction("Refresh", actionEnterNetwork));
    addBackItem(pageNetwork);
    ui.navigateTo(pageNetwork);
}

#if CLOSED_LOOP_SPEED_ENABLE
void actionApplyClosedLoopSettings() {
    // Apply validates tuning and reconfigures the feedback sensor/controller.
    settings.normalize();
    motor.applySettings();
    ui.showMessage("CL Applied", 1000);
}

void actionClosedLoopStatus() {
    SpeedFeedbackStatus status = speedFeedback.getStatus();
    char msg[32];
    if (!settings.get().closedLoopEnabled) {
        snprintf(msg, sizeof(msg), "Closed Loop Off");
    } else if (!status.signalValid) {
        snprintf(msg, sizeof(msg), "No Sensor Signal");
    } else {
        snprintf(msg, sizeof(msg), "%.2f RPM %s", status.filteredRpm, status.locked ? "LOCK" : "RUN");
    }
    ui.showMessage(msg, 2000);
}

void actionClosedLoopSetupStart() {
    speedFeedback.beginSetupCapture();
    ui.showMessage("Rotate 1 Rev", 1500);
}

void actionClosedLoopSetupStatus() {
    SpeedFeedbackSetupStatus setup = speedFeedback.getSetupStatus();
    char msg[32];
    if (!setup.active) {
        snprintf(msg, sizeof(msg), "Setup Idle");
    } else if (setup.suggestedCountsPerRev > 0) {
        snprintf(msg, sizeof(msg), "CPR %u %s", setup.suggestedCountsPerRev,
            setup.correctedDirection == SPEED_FEEDBACK_DIR_REVERSE ? "REV" : "FWD");
    } else {
        snprintf(msg, sizeof(msg), "Count %ld", (long)setup.countDelta);
    }
    ui.showMessage(msg, 2000);
}

void actionClosedLoopSetupApply() {
    // Setup apply uses the captured one-revolution count as counts/rev.
    SpeedFeedbackSetupStatus setup = speedFeedback.getSetupStatus();
    if (!setup.active || setup.suggestedCountsPerRev == 0) {
        ui.showMessage("No Setup Data", 1500);
        return;
    }

    settings.get().closedLoopCountsPerRev = setup.suggestedCountsPerRev;
    if (settings.get().closedLoopSensorMode == CLOSED_LOOP_SENSOR_QUADRATURE) {
        settings.get().closedLoopReverseDirection = setup.suggestedReverseDirection;
    }
    speedFeedback.cancelSetupCapture();
    actionApplyClosedLoopSettings();
    ui.showMessage("Setup Applied", 1500);
}

void actionClosedLoopSetupStop() {
    speedFeedback.cancelSetupCapture();
    ui.showMessage("Setup Stopped", 1200);
}

void actionClosedLoopTuneStart() {
    motor.beginClosedLoopTuning();
    ClosedLoopTuningStatus tuning = motor.getClosedLoopTuningStatus();
    ui.showMessage(tuning.stepName, 1500);
}

void actionClosedLoopTuneNext() {
    motor.advanceClosedLoopTuning();
    ClosedLoopTuningStatus tuning = motor.getClosedLoopTuningStatus();
    ui.showMessage(tuning.stepName, 1500);
}

void actionClosedLoopTuneStatus() {
    ClosedLoopTuningStatus tuning = motor.getClosedLoopTuningStatus();
    ui.showMessage(tuning.recommendation, 2500);
}

void actionClosedLoopTuneApply() {
    char msg[120];
    motor.applyClosedLoopTuningSuggestion(msg, sizeof(msg));
    ui.showMessage(msg, 2500);
}

void actionClosedLoopTuneStop() {
    motor.cancelClosedLoopTuning();
    ui.showMessage("Tune Stopped", 1200);
}

void actionEnterClosedLoop() {
    // Closed-loop pages are rebuilt because the currently edited speed selects which per-speed tuning block is shown.
    rebuildMenuPage(pageClosedLoop, "Closed Loop");
    rebuildMenuPage(pageClosedLoopControl, "CL Control");
    rebuildMenuPage(pageClosedLoopSensor, "CL Sensor");
    rebuildMenuPage(pageClosedLoopEngage, "CL Engage");
    rebuildMenuPage(pageClosedLoopTuning, "CL Tuning");
    rebuildMenuPage(pageClosedLoopPid, "CL PID");
    rebuildMenuPage(pageClosedLoopRamp, "CL Ramp");
    rebuildMenuPage(pageClosedLoopPitch, "CL Pitch");
    rebuildMenuPage(pageClosedLoopSafety, "CL Safety");
    rebuildMenuPage(pageClosedLoopTools, "CL Tools");
    rebuildMenuPage(pageClosedLoopActions, "CL Actions");
    rebuildMenuPage(pageClosedLoopSetup, "CL Setup");
    rebuildMenuPage(pageClosedLoopTune, "CL Tune");

    uint8_t speedIndex = menuShadowSpeedIndex >= 0 && menuShadowSpeedIndex <= 2 ? (uint8_t)menuShadowSpeedIndex : SPEED_33;
    ClosedLoopSpeedTuning& tuning = settings.get().closedLoopTuning[speedIndex];
    pageClosedLoop->addItem(new MenuDynamicInfo(String("Target: ") + (speedIndex == SPEED_33 ? "33" : speedIndex == SPEED_45 ? "45" : "78")));
    pageClosedLoop->addItem(new MenuBool("Enable", &settings.get().closedLoopEnabled));
    addNavItem(pageClosedLoop, "Control", pageClosedLoopControl, [](){ return settings.get().closedLoopEnabled; });
    addNavItem(pageClosedLoop, "Sensor", pageClosedLoopSensor, [](){ return settings.get().closedLoopEnabled; });
    addNavItem(pageClosedLoop, "Engage", pageClosedLoopEngage, [](){ return settings.get().closedLoopEnabled; });
    addNavItem(pageClosedLoop, "Tuning", pageClosedLoopTuning, [](){ return settings.get().closedLoopEnabled; });
    addNavItem(pageClosedLoop, "Safety", pageClosedLoopSafety, [](){ return settings.get().closedLoopEnabled; });
    addNavItem(pageClosedLoop, "Tools", pageClosedLoopTools, [](){ return settings.get().closedLoopEnabled; });
    pageClosedLoop->addItem(new MenuAction("Apply", actionApplyClosedLoopSettings));
    addBackItem(pageClosedLoop);

    addNavItem(pageClosedLoopTuning, "PID", pageClosedLoopPid);
    addNavItem(pageClosedLoopTuning, "Ramp", pageClosedLoopRamp);
    addNavItem(pageClosedLoopTuning, "Pitch", pageClosedLoopPitch);
    addBackItem(pageClosedLoopTuning);

    addNavItem(pageClosedLoopTools, "Actions", pageClosedLoopActions);
    addNavItem(pageClosedLoopTools, "Setup", pageClosedLoopSetup);
    addNavItem(pageClosedLoopTools, "Tune", pageClosedLoopTune);
    addBackItem(pageClosedLoopTools);

    MenuItem* controlMode = new MenuByte("Control", &settings.get().closedLoopControlMode,
        CLOSED_LOOP_CONTROL_MONITOR, CLOSED_LOOP_CONTROL_CORRECT, closedLoopControlLabels, 2);
    addClosedLoopItem(pageClosedLoopControl, controlMode);

    MenuItem* sensorMode = new MenuByte("Sensor", &settings.get().closedLoopSensorMode,
        CLOSED_LOOP_SENSOR_PULSE, CLOSED_LOOP_SENSOR_QUADRATURE, closedLoopSensorLabels, 2);
    addClosedLoopItem(pageClosedLoopSensor, sensorMode);

    MenuItem* targetRpm = new MenuFloat("Target RPM", &settings.get().closedLoopTargetRpm[speedIndex], 0.01, 1.0, 120.0);
    addClosedLoopItem(pageClosedLoopControl, targetRpm);

    MenuItem* counts = new MenuUInt16("Counts/Rev", &settings.get().closedLoopCountsPerRev, 1, 1, 20000);
    addClosedLoopItem(pageClosedLoopSensor, counts);

    MenuItem* edge = new MenuByte("Pulse Edge", &settings.get().closedLoopPulseEdge,
        CLOSED_LOOP_EDGE_RISING, CLOSED_LOOP_EDGE_CHANGE, closedLoopEdgeLabels, 3);
    edge->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopSensorMode == CLOSED_LOOP_SENSOR_PULSE;
    });
    pageClosedLoopSensor->addItem(edge);

    MenuItem* quadMode = new MenuByte("Quad Decode", &settings.get().closedLoopQuadratureMode,
        CLOSED_LOOP_QUAD_X1, CLOSED_LOOP_QUAD_X4, closedLoopQuadLabels, 3);
    quadMode->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopSensorMode == CLOSED_LOOP_SENSOR_QUADRATURE;
    });
    pageClosedLoopSensor->addItem(quadMode);

    MenuItem* reverse = new MenuBool("Reverse Dir", &settings.get().closedLoopReverseDirection);
    reverse->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopSensorMode == CLOSED_LOOP_SENSOR_QUADRATURE;
    });
    pageClosedLoopSensor->addItem(reverse);

    MenuItem* dirFault = new MenuByte("Dir Fault", &settings.get().closedLoopDirectionFaultAction,
        CLOSED_LOOP_FAULT_IGNORE, CLOSED_LOOP_FAULT_STOP, closedLoopFaultLabels, 3);
    dirFault->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopSensorMode == CLOSED_LOOP_SENSOR_QUADRATURE;
    });
    pageClosedLoopSensor->addItem(dirFault);

    MenuItem* debounce = new MenuUInt16("Debounce us", &settings.get().closedLoopDebounceUs, 10, 0, 50000);
    addClosedLoopItem(pageClosedLoopSensor, debounce);

    MenuItem* timeout = new MenuUInt16("Timeout ms", &settings.get().closedLoopTimeoutMs, 100, 100, 10000);
    addClosedLoopItem(pageClosedLoopSensor, timeout);

    MenuItem* engage = new MenuUInt16("Engage ms", &settings.get().closedLoopEngageDelayMs, 100, 0, 30000);
    addClosedLoopItem(pageClosedLoopEngage, engage);

    MenuItem* requireSignal = new MenuBool("Req Signal", &settings.get().closedLoopRequireSignalBeforeEngage);
    addClosedLoopItem(pageClosedLoopEngage, requireSignal);

    MenuItem* requireNear = new MenuBool("Req Near", &settings.get().closedLoopRequireNearTargetBeforeEngage);
    addClosedLoopItem(pageClosedLoopEngage, requireNear);

    MenuItem* engageTol = new MenuFloat("Eng Tol", &settings.get().closedLoopEngageToleranceRpm, 0.01, 0.01, 20.0);
    engageTol->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopRequireNearTargetBeforeEngage;
    });
    pageClosedLoopEngage->addItem(engageTol);

    MenuItem* update = new MenuUInt16("Update ms", &settings.get().closedLoopUpdateIntervalMs, 10, 20, 1000);
    addClosedLoopItem(pageClosedLoopControl, update);

    MenuItem* alpha = new MenuFloat("Filter A", &settings.get().closedLoopFilterAlpha, 0.01, 0.01, 1.0);
    addClosedLoopItem(pageClosedLoopControl, alpha);

    MenuItem* deadband = new MenuFloat("Dead RPM", &tuning.deadbandRpm, 0.01, 0.0, 5.0);
    addClosedLoopItem(pageClosedLoopPid, deadband);

    MenuItem* lockTol = new MenuFloat("Lock Tol", &tuning.lockToleranceRpm, 0.01, 0.01, 5.0);
    addClosedLoopItem(pageClosedLoopPid, lockTol);

    MenuItem* lockMs = new MenuUInt16("Lock ms", &tuning.lockTimeMs, 100, 0, 30000);
    addClosedLoopItem(pageClosedLoopPid, lockMs);

    MenuItem* kp = new MenuFloat("Kp", &tuning.kp, 0.01, 0.0, 20.0);
    addClosedLoopItem(pageClosedLoopPid, kp);

    MenuItem* ki = new MenuFloat("Ki", &tuning.ki, 0.01, 0.0, 20.0);
    addClosedLoopItem(pageClosedLoopPid, ki);

    MenuItem* kd = new MenuFloat("Kd", &tuning.kd, 0.01, 0.0, 20.0);
    addClosedLoopItem(pageClosedLoopPid, kd);

    MenuItem* iLimit = new MenuFloat("I Lim Hz", &tuning.integralLimitHz, 0.1, 0.0, 50.0);
    addClosedLoopItem(pageClosedLoopPid, iLimit);

    MenuItem* correction = new MenuFloat("Corr Hz", &tuning.correctionLimitHz, 0.1, 0.0, 100.0);
    addClosedLoopItem(pageClosedLoopPid, correction);

    MenuItem* slew = new MenuFloat("Slew Hz/s", &tuning.slewLimitHzPerSec, 0.1, 0.0, 100.0);
    addClosedLoopItem(pageClosedLoopPid, slew);

    MenuItem* dropout = new MenuByte("Dropout", &settings.get().closedLoopDropoutAction,
        CLOSED_LOOP_DROPOUT_OPEN_LOOP, CLOSED_LOOP_DROPOUT_STOP, closedLoopDropLabels, 3);
    addClosedLoopItem(pageClosedLoopSafety, dropout);

    MenuItem* rampMode = new MenuByte("Ramp CL", &settings.get().closedLoopRampMode,
        CLOSED_LOOP_RAMP_DISABLED, CLOSED_LOOP_RAMP_TRACK, closedLoopRampLabels, 2);
    addClosedLoopItem(pageClosedLoopRamp, rampMode);

    MenuItem* rampKp = new MenuFloat("Ramp Kp", &tuning.rampKp, 0.01, 0.0, 5.0);
    rampKp->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopRampMode == CLOSED_LOOP_RAMP_TRACK;
    });
    pageClosedLoopRamp->addItem(rampKp);

    MenuItem* rampLimit = new MenuFloat("Ramp Lim", &tuning.rampCorrectionLimitHz, 0.1, 0.0, 20.0);
    rampLimit->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopRampMode == CLOSED_LOOP_RAMP_TRACK;
    });
    pageClosedLoopRamp->addItem(rampLimit);

    MenuItem* pitchMode = new MenuByte("Pitch Mode", &settings.get().closedLoopPitchTargetMode,
        CLOSED_LOOP_PITCH_TARGET_FIXED, CLOSED_LOOP_PITCH_TARGET_FOLLOW, closedLoopPitchTargetLabels, 2);
    addClosedLoopItem(pageClosedLoopPitch, pitchMode);

    MenuItem* pitchSlew = new MenuFloat("Pitch Slew", &settings.get().closedLoopPitchSlewRpmPerSec, 0.1, 0.0, 200.0);
    addClosedLoopItem(pageClosedLoopPitch, pitchSlew);

    MenuItem* pitchReset = new MenuFloat("Pitch Reset", &settings.get().closedLoopPitchResetThresholdRpm, 0.1, 0.0, 20.0);
    addClosedLoopItem(pageClosedLoopPitch, pitchReset);

    MenuItem* satMs = new MenuUInt16("Sat ms", &settings.get().closedLoopSaturationTimeMs, 100, 0, 60000);
    addClosedLoopItem(pageClosedLoopSafety, satMs);

    MenuItem* satAction = new MenuByte("Sat Act", &settings.get().closedLoopSaturationAction,
        CLOSED_LOOP_FAULT_IGNORE, CLOSED_LOOP_FAULT_STOP, closedLoopFaultLabels, 3);
    satAction->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopSaturationTimeMs > 0;
    });
    pageClosedLoopSafety->addItem(satAction);

    MenuItem* minRpm = new MenuFloat("Min RPM", &settings.get().closedLoopPlausibilityMinRpm, 1.0, 0.0, 120.0);
    addClosedLoopItem(pageClosedLoopSafety, minRpm);

    MenuItem* maxRpm = new MenuFloat("Max RPM", &settings.get().closedLoopPlausibilityMaxRpm, 1.0, 1.0, 200.0);
    addClosedLoopItem(pageClosedLoopSafety, maxRpm);

    MenuItem* plausAction = new MenuByte("Plaus Act", &settings.get().closedLoopPlausibilityAction,
        CLOSED_LOOP_FAULT_IGNORE, CLOSED_LOOP_FAULT_STOP, closedLoopFaultLabels, 3);
    addClosedLoopItem(pageClosedLoopSafety, plausAction);

    MenuItem* lockTimeout = new MenuUInt16("Lock To", &settings.get().closedLoopLockTimeoutMs, 100, 0, 60000);
    addClosedLoopItem(pageClosedLoopSafety, lockTimeout);

    MenuItem* lockAction = new MenuByte("Lock Act", &settings.get().closedLoopLockTimeoutAction,
        CLOSED_LOOP_FAULT_IGNORE, CLOSED_LOOP_FAULT_STOP, closedLoopFaultLabels, 3);
    lockAction->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopLockTimeoutMs > 0;
    });
    pageClosedLoopSafety->addItem(lockAction);

    MenuItem* ampRecovery = new MenuByte("Amp Rec", &settings.get().closedLoopAmpRecoveryMode,
        CLOSED_LOOP_AMP_RECOVERY_OFF, CLOSED_LOOP_AMP_RECOVERY_RESTORE, closedLoopAmpRecoveryLabels, 3);
    addClosedLoopItem(pageClosedLoopSafety, ampRecovery);

    MenuItem* ampRecoveryDelay = new MenuUInt16("Amp Rec ms", &settings.get().closedLoopAmpRecoveryDelayMs, 100, 0, 30000);
    ampRecoveryDelay->setVisibleWhen([](){
        return settings.get().closedLoopEnabled &&
               settings.get().closedLoopAmpRecoveryMode != CLOSED_LOOP_AMP_RECOVERY_OFF;
    });
    pageClosedLoopSafety->addItem(ampRecoveryDelay);

    pageClosedLoopActions->addItem(new MenuAction("Apply", actionApplyClosedLoopSettings));
    pageClosedLoopActions->addItem(new MenuAction("Reset PID", [](){
        motor.resetClosedLoop();
        ui.showMessage("PID Reset", 1000);
    }));
    pageClosedLoopActions->addItem(new MenuAction("Sensor Test", actionClosedLoopStatus));

    pageClosedLoopSetup->addItem(new MenuAction("Setup Start", actionClosedLoopSetupStart));
    pageClosedLoopSetup->addItem(new MenuAction("Setup Stat", actionClosedLoopSetupStatus));
    pageClosedLoopSetup->addItem(new MenuAction("Setup Apply", actionClosedLoopSetupApply));
    pageClosedLoopSetup->addItem(new MenuAction("Setup Stop", actionClosedLoopSetupStop));

    pageClosedLoopTune->addItem(new MenuAction("Tune Start", actionClosedLoopTuneStart));
    pageClosedLoopTune->addItem(new MenuAction("Tune Next", actionClosedLoopTuneNext));
    pageClosedLoopTune->addItem(new MenuAction("Tune Stat", actionClosedLoopTuneStatus));
    pageClosedLoopTune->addItem(new MenuAction("Tune Apply", actionClosedLoopTuneApply));
    pageClosedLoopTune->addItem(new MenuAction("Tune Stop", actionClosedLoopTuneStop));

    addBackItem(pageClosedLoopControl);
    addBackItem(pageClosedLoopSensor);
    addBackItem(pageClosedLoopEngage);
    addBackItem(pageClosedLoopPid);
    addBackItem(pageClosedLoopRamp);
    addBackItem(pageClosedLoopPitch);
    addBackItem(pageClosedLoopSafety);
    addBackItem(pageClosedLoopActions);
    addBackItem(pageClosedLoopSetup);
    addBackItem(pageClosedLoopTune);
    ui.navigateTo(pageClosedLoop);
}
#endif

// --- Menu Builder ---


void buildMenuSystem() {
    // Static pages are allocated once. Dynamic pages are built when entered.

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

    /*
     * --- Phase Page (Mixed) ---
     * Phase mode is global; phase offsets are per-speed shadow values.
     */
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

    /*
     * --- Motor Pages (Mixed) ---
     * Startup/amplitude per-speed fields use the shadow copy; global ramp/brake
     * fields update settings directly.
     */
    pageMotor = new MenuPage("Motor Control");
    pageMotorStartup = new MenuPage("Motor Start");
    pageMotorAmplitude = new MenuPage("Motor Amp");
    pageMotorRamping = new MenuPage("Motor Ramp");
    pageMotorBraking = new MenuPage("Motor Brake");

    addNavItem(pageMotor, "Startup", pageMotorStartup);
    addNavItem(pageMotor, "Amplitude", pageMotorAmplitude);
    addNavItem(pageMotor, "Ramping", pageMotorRamping);
    addNavItem(pageMotor, "Braking", pageMotorBraking);
    addBackItem(pageMotor);

    pageMotorStartup->addItem(new MenuFloat("Soft Start", &menuShadowSettings.softStartDuration, 0.1, 0.0, 10.0));
    pageMotorStartup->addItem(new MenuByte("Kick Mult", &menuShadowSettings.startupKick, 1, 4));
    MenuItem* kickDuration = new MenuByte("Kick Dur", &menuShadowSettings.startupKickDuration, 0, 15);
    kickDuration->setVisibleWhen([](){ return menuShadowSettings.startupKick > 1; });
    pageMotorStartup->addItem(kickDuration);
    MenuItem* kickRamp = new MenuFloat("Kick Ramp", &menuShadowSettings.startupKickRampDuration, 0.1, 0.0, 15.0);
    kickRamp->setVisibleWhen([](){ return menuShadowSettings.startupKick > 1; });
    pageMotorStartup->addItem(kickRamp);
    addBackItem(pageMotorStartup);

    pageMotorAmplitude->addItem(new MenuByte("Red. Amp %", &menuShadowSettings.reducedAmplitude, 10, 100));
    pageMotorAmplitude->addItem(new MenuByte("Amp Delay", &menuShadowSettings.amplitudeDelay, 0, 60));
    pageMotorAmplitude->addItem(new MenuByte("V/f Blend%", &settings.get().freqDependentAmplitude, 0, 100));
    MenuItem* vfLowHz = new MenuFloat("V/f LowHz", &settings.get().vfLowFreq, 1.0, 0.0, 50.0);
    vfLowHz->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotorAmplitude->addItem(vfLowHz);
    MenuItem* vfLowBoost = new MenuByte("V/f Low%", &settings.get().vfLowBoost, 0, 100);
    vfLowBoost->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotorAmplitude->addItem(vfLowBoost);
    MenuItem* vfMidHz = new MenuFloat("V/f MidHz", &settings.get().vfMidFreq, 1.0, 0.0, 100.0);
    vfMidHz->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotorAmplitude->addItem(vfMidHz);
    MenuItem* vfMidBoost = new MenuByte("V/f Mid%", &settings.get().vfMidBoost, 0, 100);
    vfMidBoost->setVisibleWhen([](){ return settings.get().freqDependentAmplitude > 0; });
    pageMotorAmplitude->addItem(vfMidBoost);
    pageMotorAmplitude->addItem(new MenuByte("Max Amp %", &settings.get().maxAmplitude, 0, 100));
    addBackItem(pageMotorAmplitude);

    pageMotorRamping->addItem(new MenuByte("Ramp Type", &settings.get().rampType, 0, 1, rampTypeLabels, 2));
    MenuItem* softStartCurve = new MenuByte("SS Curve", &settings.get().softStartCurve, 0, 2, softStartCurveLabels, 3);
    softStartCurve->setVisibleWhen([](){ return settings.get().rampType == RAMP_LINEAR; });
    pageMotorRamping->addItem(softStartCurve);
    pageMotorRamping->addItem(new MenuBool("Smooth Sw", &settings.get().smoothSwitching));
    MenuItem* switchRamp = new MenuByte("Sw Ramp", &settings.get().switchRampDuration, 1, 5);
    switchRamp->setVisibleWhen([](){ return settings.get().smoothSwitching; });
    pageMotorRamping->addItem(switchRamp);
    pageMotorRamping->addItem(new MenuBool("Auto Start", &settings.get().autoStart));
    addBackItem(pageMotorRamping);

    pageMotorBraking->addItem(new MenuByte("Brake Mode", &settings.get().brakeMode, 0, 3, brakeModeLabels, 4));
    MenuItem* brakeDuration = new MenuFloat("Brake Dur", &settings.get().brakeDuration, 0.1, 0.0, 10.0);
    brakeDuration->setVisibleWhen([](){ return settings.get().brakeMode != BRAKE_OFF; });
    pageMotorBraking->addItem(brakeDuration);
    MenuItem* brakePulse = new MenuFloat("Brk Pulse", &settings.get().brakePulseGap, 0.1, 0.1, 2.0);
    brakePulse->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_PULSE; });
    pageMotorBraking->addItem(brakePulse);
    MenuItem* brakeStart = new MenuFloat("Brk StartF", &settings.get().brakeStartFreq, 1.0, 10.0, 200.0);
    brakeStart->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageMotorBraking->addItem(brakeStart);
    MenuItem* brakeStop = new MenuFloat("Brk StopF", &settings.get().brakeStopFreq, 1.0, 0.0, 50.0);
    brakeStop->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_RAMP; });
    pageMotorBraking->addItem(brakeStop);
    MenuItem* brakeCutoff = new MenuFloat("Brk Cutoff", &settings.get().softStopCutoff, 1.0, 0.0, 50.0);
    brakeCutoff->setVisibleWhen([](){ return settings.get().brakeMode == BRAKE_SOFT_STOP; });
    pageMotorBraking->addItem(brakeCutoff);
    pageMotorBraking->addItem(new MenuAction("Brake Tune", actionEnterBrakeTune));
    addBackItem(pageMotorBraking);

    /*
     * --- Power Page (Global) ---
     * Relay items are only added for hardware compiled into this build.
     */
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
    pageDisplay->addItem(new MenuBool("Show CPU", &settings.get().showCpuDashboard));
    pageDisplay->addItem(new MenuBool("Show Memory", &settings.get().showMemoryDashboard));
    pageDisplay->addItem(new MenuBool("Show Flash", &settings.get().showFlashDashboard));
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
    pageSystem->addItem(new MenuAction("UI Lock", actionEnterSecurity));
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
    // Items are populated dynamically in actionEnterPresets.
    pagePresets->addItem(new MenuAction("Back", [](){ ui.back(); }));

    // --- Main Menu ---
    pageMain = new MenuPage("Main Menu");

    extern bool safeModeActive;
    if (safeModeActive) {
        pageMain->addItem(new MenuAction("Exit Safe Mode", [](){
            hal.watchdogReboot();
        }));
    }

    // The first item toggles which speed we are editing in the submenus.
    menuSpeedSelector = new MenuAction(speedLabelBuffer, actionNextSpeed);
    pageMain->addItem(menuSpeedSelector);

    pageMain->addItem(new MenuNav("Speed Tuning", pageSpeedTuning));
    pageMain->addItem(new MenuNav("Phase", pagePhase));
    pageMain->addItem(new MenuNav("Motor", pageMotor));
    pageMain->addItem(new MenuNav("Power", pagePower));
    pageMain->addItem(new MenuNav("Display", pageDisplay));
    pageMain->addItem(new MenuNav("System", pageSystem));
#if CLOSED_LOOP_SPEED_ENABLE
    pageMain->addItem(new MenuAction("Closed Loop", actionEnterClosedLoop));
#endif
    if (networkManager.isAvailable()) {
        pageMain->addItem(new MenuAction("Network", actionEnterNetwork));
    }
    pageMain->addItem(new MenuAction("Presets", actionEnterPresets));

    pageMain->addItem(new MenuAction("Save & Exit", actionSaveExit));
    pageMain->addItem(new MenuAction("Cancel", actionCancelExit));

    /*
     * --- Locked UI Entry Page ---
     * This page is shown instead of the main menu when the device lock is active.
     */
    pageUnlock = new MenuPage("UI Locked");
    pageUnlock->addItem(new MenuInfo("Enter PIN"));
    pageUnlock->addItem(new MenuText("PIN", unlockPinBuffer, NETWORK_WEB_PIN_MAX));
    pageUnlock->addItem(new MenuAction("Unlock", actionUnlockDevice));
    pageUnlock->addItem(new MenuAction("Back", [](){ ui.exitMenu(); }));
}
