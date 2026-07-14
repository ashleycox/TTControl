/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include <Arduino.h>

#include "config.h"
#include "display.h"
#include "types.h"
#include "globals.h"
#include "settings.h"
#include "waveform.h"
#include "motor.h"
#include "ui.h"
#include "serial_cmd.h"
#include "hal.h"
#include "error_handler.h"
#include "amp_monitor.h"
#include "network_manager.h"
#include "web_interface.h"
#include "system_monitor.h"
#include "power_stage.h"

/*
 * --- Global Objects ---
 * These are constructed once and referenced from the modules through globals.h.
 * Keeping ownership here makes the Arduino sketch the composition root for the
 * firmware while the modules stay focused on behavior.
 */
Settings settings;
WaveformGenerator waveform;
MotorController motor;
UserInterface ui;

/*
 * --- Cross-core State ---
 * Core 0 owns motor decisions; Core 1 reads these values while generating the
 * waveform. Volatile prevents stale reads across cores, but callers should still
 * prefer the module APIs so updates remain clamped and sequenced.
 */
volatile MotorState currentMotorState = STATE_STANDBY;
volatile float currentFrequency = 50.0;
volatile float currentPitchPercent = 0.0;

// Core 1 refreshes this each waveform loop. Core 0 uses it both for watchdog gating and for detecting a waveform task that has stopped servicing buffers.
volatile uint32_t core1HeartbeatMs = 0;

// Fault reporting is one-shot so a stalled waveform path does not spam the persistent error log while the watchdog is already being withheld.
static bool waveformHealthFaultReported = false;
static bool settingsBootConfirmed = false;
static const uint32_t WAVEFORM_CORE_STALL_MS = 1000;
static const uint32_t WAVEFORM_BUFFER_STALL_MS = 250;

static void checkWaveformHealth(uint32_t now) {
    if (waveformHealthFaultReported) return;

    // A missing heartbeat means Core 1 has stopped running its loop entirely.
    if (core1HeartbeatMs != 0 && now - core1HeartbeatMs > WAVEFORM_CORE_STALL_MS) {
        waveformHealthFaultReported = true;
        errorHandler.report(ERR_WAVEFORM_HEALTH, "Waveform core heartbeat stalled", true);
        return;
    }

    // A fresh heartbeat with an old fill timestamp points to the buffer service path rather than the core scheduler itself.
    uint32_t lastFillMs = waveform.getLastBufferFillMs();
    if (lastFillMs != 0 && now - lastFillMs > WAVEFORM_BUFFER_STALL_MS) {
        waveformHealthFaultReported = true;
        errorHandler.report(ERR_WAVEFORM_HEALTH, "Waveform DMA buffer service stalled", true);
    }
}

/*
 * --- Core 0: UI & Control Logic ---
 * Handles user interaction, display, motor state machine, and serial comms.
 */
void setup() {
    // Initialize Hardware Abstraction Layer
    hal.begin();
    // Assert the bridge interlock before filesystem, display, UI, or motor initialization can delay hardware-safe GPIO levels.
    powerStage.begin();
    ResetCause resetCause = hal.getResetCause();

    /*
     * Check for Hardware Safe Mode Boot
     * If the encoder button is held during power-on/reset, enter Safe Mode
     */
    hal.setPinMode(PIN_ENC_MAIN_SW, INPUT_PULLUP);
    if (hal.digitalRead(PIN_ENC_MAIN_SW) == LOW) {
        safeModeActive = true;
    }

    // Serial is intentionally optional so production builds can disable the monitor without changing command or UI code.
    if (SERIAL_MONITOR_ENABLE) {
        Serial.begin(115200);
        Serial.println("TT Control Booting...");
    }

    // Settings must come before modules that read configuration. Safe Mode is already latched so Settings can decide whether to bypass flash contents.
    settings.begin();
    errorHandler.begin();
    char resetMsg[48];
    snprintf(resetMsg, sizeof(resetMsg), "Reset cause: %s", hal.resetCauseName(resetCause));
    errorHandler.logEvent(ERR_RESET_CAUSE, resetMsg);
    systemMonitor.begin();

    // Display failure is non-fatal: the logical canvas, serial CLI, web UI, and safe hardware defaults remain available.
    if (!displayManager.begin()) {
        if (SERIAL_MONITOR_ENABLE) {
            Serial.print(F("Display initialization failed: "));
            Serial.println(displayManager.driverName());
        }
    } else if (SERIAL_MONITOR_ENABLE) {
        Serial.print(F("Display: "));
        Serial.print(displayManager.driverName());
        Serial.print(F(" via "));
        Serial.print(displayManager.transportName());
        Serial.print(F(" / "));
        Serial.print(displayManager.wiringProfileName());
        Serial.print(F(" ("));
        Serial.print(displayManager.physicalWidth());
        Serial.print('x');
        Serial.print(displayManager.physicalHeight());
        Serial.println(')');
    }
    displayManager.setBrightness(settings.get().displayBrightness);

    // UI and motor startup are separated: UI sets up displays and inputs, then MotorController applies relay and waveform-safe defaults.
    ui.begin();

    motor.begin();

    // Safety and connectivity services are polled from loop(); begin() only performs lightweight setup so watchdog enable remains near the end.
    ampMonitor.begin();
    networkManager.begin();
    webInterface.begin();
    
    if (SERIAL_MONITOR_ENABLE) Serial.println("Core 0 Setup Complete");
    
    /*
     * Enable Watchdog Timer (2000ms timeout)
     * Must be fed in both loop() and loop1()
     */
    hal.watchdogEnable(2000);
    
    // Signal Core 1 to proceed with its setup
    systemInitialized = true;
}

void loop() {
    systemMonitor.beginCore0Loop();

    // Keep each subsystem non-blocking. This loop owns all user-facing work and must leave regular time for motor state transitions and network polling.
    ui.update();
    powerStage.update();
    motor.update();
    ampMonitor.update();
    
    if (SERIAL_MONITOR_ENABLE) {
        handleSerialCommands();
    }

    networkManager.update();
    webInterface.update();
    systemMonitor.update();
    ui.render();
    systemMonitor.endCore0Loop();
    
    // Feed the watchdog only after waveform generation has proved it is alive. If Core 1 or DMA service stalls, the watchdog reset is the safest outcome.
    uint32_t now = hal.getMillis();
    checkWaveformHealth(now);
    uint32_t core1Age = now - core1HeartbeatMs;
    if (core1HeartbeatMs != 0 && core1Age < 1000) {
        /*
         * The boot marker is delayed until a waveform buffer has actually been
         * filled, so fallback logic can distinguish a real boot from a crash
         * during early initialization.
         */
        if (!safeModeActive && !settingsBootConfirmed && waveform.getLastBufferFillMs() != 0) {
            settings.markBootSuccessful();
            settingsBootConfirmed = true;
        }
        hal.watchdogFeed();
    }
}

/*
 * --- Core 1: Waveform Generation ---
 * Dedicated to high-precision signal generation (DDS)
 */
void setup1() {
    // Wait for Core 0 to initialize shared resources (Settings, etc.)
    while (!systemInitialized) {
        delay(1);
    }
    
    // Core 1 owns timing-sensitive PWM/DMA state. No display, filesystem, or serial command work should be added below this point.
    waveform.begin();
    core1HeartbeatMs = hal.getMillis();
    
    if (SERIAL_MONITOR_ENABLE) Serial.println("Core 1 Setup Complete");
}

void loop1() {
    // High-priority waveform loop. WaveformGenerator decides whether a DMA buffer needs service; this wrapper only records liveness for Core 0.
    waveform.update();
    core1HeartbeatMs = hal.getMillis();
}
