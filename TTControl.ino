/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "types.h"
#include "globals.h"
#include "settings.h"
#include "waveform.h"
#include "motor.h"
#include "ui.h"
#include "serial_cmd.h"
#include "hal.h"
#include "error_handler.h"

// --- Global Objects ---
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

Settings settings;
WaveformGenerator waveform;
MotorController motor;
UserInterface ui;

// Shared State
volatile MotorState currentMotorState = STATE_STANDBY;
volatile float currentFrequency = 50.0;
volatile float currentPitchPercent = 0.0;

// --- Core 0: UI & Control Logic ---
// Handles user interaction, display, motor state machine, and serial comms.
void setup() {
    // Initialize Hardware Abstraction Layer
    hal.begin();

    // Initialize Serial for debugging
    if (SERIAL_MONITOR_ENABLE) {
        Serial.begin(115200);
        Serial.println("TT Control Booting...");
    }

    // Initialize Settings (Mounts LittleFS and loads config)
    settings.begin();

    // Initialize Display (I2C)
    Wire.setSDA(PIN_I2C0_SDA);
    Wire.setSCL(PIN_I2C0_SCL);
    Wire.begin();
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        // TODO: Blink LED to indicate fatal error
    }
    display.clearDisplay();
    display.display();

    // Initialize User Interface
    ui.begin();

    // Initialize Motor Control Logic (State Machine)
    motor.begin();
    
    Serial.println("Core 0 Setup Complete");
    
    // Enable Watchdog Timer (2000ms timeout)
    // Must be fed in both loop() and loop1()
    hal.watchdogEnable(2000);
    
    // Signal Core 1 to proceed with its setup
    systemInitialized = true;
}

void loop() {
    // 1. Update User Interface (Poll inputs, draw screen)
    ui.update();
    
    // 2. Update Motor Logic (State transitions, speed ramping)
    motor.update();
    
    // 3. Handle Serial Commands (Debug/Control)
    if (SERIAL_MONITOR_ENABLE) {
        handleSerialCommands();
    }
    
    // 4. Feed Watchdog
    hal.watchdogFeed();
}

// --- Core 1: Waveform Generation ---
// Dedicated to high-precision signal generation (DDS)
void setup1() {
    // Wait for Core 0 to initialize shared resources (Settings, etc.)
    while (!systemInitialized) {
        delay(1);
    }
    
    // Initialize Waveform Generator (PWM, LUTs)
    waveform.begin();
    
    Serial.println("Core 1 Setup Complete");
}

void loop1() {
    // High priority waveform generation loop
    // Generates samples at 20kHz (50us interval)
    waveform.update();
}
