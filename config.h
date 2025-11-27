/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef CONFIG_H
#define CONFIG_H

// --- System Information ---
#define FIRMWARE_VERSION "v1.0.0"
#ifndef BUILD_DATE
#define BUILD_DATE __DATE__ " " __TIME__
#endif

// --- Flash Memory Configuration ---
// 8MB LittleFS partition recommended for Pico+2 (16MB total flash)
// Ensure you select the correct partition scheme in Arduino IDE
#define LITTLEFS_FS_SIZE (8 * 1024 * 1024) 

// --- Core Hardware Settings ---
#define OLED_I2C_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// --- Waveform Generation ---
#define LUT_MAX_SIZE 16384 // Size of Sine Lookup Table

// --- Preset Management ---
#define MAX_PRESET_SLOTS 5

// --- Serial Debugging ---
#define SERIAL_MONITOR_ENABLE true
#define DUPLICATE_DISPLAY_TO_SERIAL false

// --- Optional Features (Compile-time flags) ---
// Set to true to enable specific hardware features
#define PITCH_CONTROL_ENABLE false
#define STANDBY_BUTTON_ENABLE false
#define SPEED_BUTTON_ENABLE false
#define START_STOP_BUTTON_ENABLE false

// --- Feature Flags ---
#define ENABLE_STANDBY true         // Set to false to disable all standby functionality
#define ENABLE_MUTE_RELAYS true     // Set to false to disable muting relays entirely
#define ENABLE_DPDT_RELAYS false    // Set to true to use 2x DPDT relays instead of 4x SPST

// --- Pin Assignments (RP2040 / RP2053) ---

// PWM Outputs (Waveform Generation)
#define PIN_PWM_PHASE_A 0
#define PIN_PWM_PHASE_B 1
#define PIN_PWM_PHASE_C 2
#define PIN_PWM_PHASE_D 3

// I2C Interface (Display)
#define PIN_I2C0_SDA 4
#define PIN_I2C0_SCL 5

// Main Encoder
#define PIN_ENC_MAIN_CLK 10
#define PIN_ENC_MAIN_DT 11
#define PIN_ENC_MAIN_SW 12

// Optional Pins (only used if enabled above)
#define PIN_ENC_PITCH_CLK 13
#define PIN_ENC_PITCH_DT 14
#define PIN_ENC_PITCH_SW 15

#define PIN_RELAY_STANDBY 16
#define PIN_MUTE_PHASE_A 17
#define PIN_MUTE_PHASE_B 18
#define PIN_MUTE_PHASE_C 19
#define PIN_MUTE_PHASE_D 20

// DPDT Relay Pins (only used if ENABLE_DPDT_RELAYS is true)
// Defaults to reusing Phase A and Phase B mute pins
#define PIN_RELAY_DPDT_1 PIN_MUTE_PHASE_A
#define PIN_RELAY_DPDT_2 PIN_MUTE_PHASE_B

#define PIN_BTN_STANDBY 21
#define PIN_BTN_SPEED 22
#define PIN_BTN_START_STOP 23

// --- UI Strings ---
#define STANDBY_MESSAGE "TT Control Standby"
#define WELCOME_MESSAGE "Welcome to TT Control"

// --- Storage Schema ---
#define SETTINGS_SCHEMA_VERSION 4

// --- Default Values ---
#define DEFAULT_PHASE_MODE 3 // 3-phase
#define DEFAULT_SPEED_INDEX 0 // 33.3 RPM

#endif // CONFIG_H
