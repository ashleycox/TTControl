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
#define MIN_OUTPUT_FREQUENCY_HZ 10.0f
#define MAX_OUTPUT_FREQUENCY_HZ 1500.0f

// --- Preset Management ---
#define MAX_PRESET_SLOTS 5

// --- Serial Debugging ---
#ifndef SERIAL_MONITOR_ENABLE
#define SERIAL_MONITOR_ENABLE 1
#endif
#ifndef DUPLICATE_DISPLAY_TO_SERIAL
#define DUPLICATE_DISPLAY_TO_SERIAL 0
#endif

// --- Optional Features (Compile-time flags) ---
// Set to 1 to enable specific hardware features
#ifndef NETWORK_ENABLE
#if defined(PICO_CYW43_SUPPORTED)
#define NETWORK_ENABLE 1
#else
#define NETWORK_ENABLE 0
#endif
#endif

#ifndef PITCH_CONTROL_ENABLE
#define PITCH_CONTROL_ENABLE 0
#endif
#ifndef STANDBY_BUTTON_ENABLE
#define STANDBY_BUTTON_ENABLE 0
#endif
#ifndef SPEED_BUTTON_ENABLE
#define SPEED_BUTTON_ENABLE 0
#endif
#ifndef START_STOP_BUTTON_ENABLE
#define START_STOP_BUTTON_ENABLE 0
#endif

// --- Feature Flags ---
#ifndef ENABLE_STANDBY
#define ENABLE_STANDBY 1         // Set to 0 to disable all standby functionality
#endif
#ifndef ENABLE_MUTE_RELAYS
#define ENABLE_MUTE_RELAYS 1     // Set to 0 to disable muting relays entirely
#endif
#ifndef ENABLE_DPDT_RELAYS
#define ENABLE_DPDT_RELAYS 0     // Set to 1 to use 2x DPDT relays instead of 4x SPST
#endif
#ifndef ENABLE_4_CHANNEL_SUPPORT
#define ENABLE_4_CHANNEL_SUPPORT 0 // Set to 1 to enable optional 4-channel/Premotec bridge modes
#endif
#ifndef AMP_MONITOR_ENABLE
#define AMP_MONITOR_ENABLE 1       // Enable amplifier heatsink thermal monitoring
#endif

#if ENABLE_4_CHANNEL_SUPPORT
#define MAX_PHASE_MODE 4
#define MAX_ACTIVE_PHASE_OUTPUTS 4
#else
#define MAX_PHASE_MODE 3
#define MAX_ACTIVE_PHASE_OUTPUTS 3
#endif

// --- Pin Assignments (RP2350 / Arduino-Pico) ---

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

// DPDT Relay Pins (only used if ENABLE_DPDT_RELAYS is 1)
// Defaults to reusing Phase A and Phase B mute pins
#define PIN_RELAY_DPDT_1 PIN_MUTE_PHASE_A
#define PIN_RELAY_DPDT_2 PIN_MUTE_PHASE_B

#define PIN_BTN_STANDBY 21
#define PIN_BTN_SPEED 22
#define PIN_BTN_START_STOP 9
#define PIN_AMP_TEMP 26
#define PIN_AMP_THERM_OK 27

#define AMP_TEMP_WARN_C 65.0f
#define AMP_TEMP_SHUTDOWN_C 75.0f
#define AMP_TEMP_MIN_C 30.0f
#define AMP_TEMP_MAX_C 120.0f
#define AMP_TEMP_MIN_SHUTDOWN_MARGIN_C 1.0f
#define AMP_TEMP_WARN_HYSTERESIS_C 5.0f

// --- Network Defaults (Wi-Fi boards only) ---
#define NETWORK_CONFIG_MAGIC 0x54545746UL
#define NETWORK_CONFIG_VERSION 4
#define NETWORK_HOSTNAME_MAX 32
#define NETWORK_SSID_MAX 32
#define NETWORK_PASSWORD_MAX 64
#define NETWORK_WEB_PIN_MAX 8
#define NETWORK_DEFAULT_HOSTNAME "ttcontrol"
#define NETWORK_DEFAULT_AP_SSID "TTControl-Setup"
#define NETWORK_DEFAULT_AP_PASSWORD ""
#define NETWORK_DEFAULT_AP_CHANNEL 6
#define NETWORK_DEFAULT_WEB_PIN "1234"

// --- UI Strings ---
#define STANDBY_MESSAGE "TT Control Standby"
#define WELCOME_MESSAGE "Welcome to TT Control"

// --- Storage Schema ---
#define SETTINGS_SCHEMA_VERSION 5
#define SETTINGS_FILE_FORMAT_VERSION 1
#define SETTINGS_FILE_MAGIC 0x54544353UL // "TTCS"
#define PRESET_FILE_MAGIC 0x54544350UL   // "TTCP"
#define SPEED_SETTINGS_STORAGE_SIZE 52
#define GLOBAL_SETTINGS_STORAGE_SIZE 336

// --- Default Values ---
#define DEFAULT_PHASE_MODE 3 // 3-phase
#define DEFAULT_SPEED_INDEX 0 // 33.3 RPM

// --- Compile-time Safety Checks ---
#if NETWORK_ENABLE && !defined(PICO_CYW43_SUPPORTED)
#error "NETWORK_ENABLE requires a Wi-Fi-capable Pico/RP board target."
#endif

#if (ENABLE_STANDBY != 0 && ENABLE_STANDBY != 1)
#error "ENABLE_STANDBY must be 0 or 1."
#endif
#if (ENABLE_MUTE_RELAYS != 0 && ENABLE_MUTE_RELAYS != 1)
#error "ENABLE_MUTE_RELAYS must be 0 or 1."
#endif
#if (ENABLE_DPDT_RELAYS != 0 && ENABLE_DPDT_RELAYS != 1)
#error "ENABLE_DPDT_RELAYS must be 0 or 1."
#endif
#if (ENABLE_4_CHANNEL_SUPPORT != 0 && ENABLE_4_CHANNEL_SUPPORT != 1)
#error "ENABLE_4_CHANNEL_SUPPORT must be 0 or 1."
#endif
#if (AMP_MONITOR_ENABLE != 0 && AMP_MONITOR_ENABLE != 1)
#error "AMP_MONITOR_ENABLE must be 0 or 1."
#endif

#if ENABLE_DPDT_RELAYS && !ENABLE_MUTE_RELAYS
#error "ENABLE_DPDT_RELAYS requires ENABLE_MUTE_RELAYS."
#endif
#if ENABLE_DPDT_RELAYS && ENABLE_4_CHANNEL_SUPPORT
#error "DPDT relay mode does not provide independent mute control for four phase outputs."
#endif

static_assert(MIN_OUTPUT_FREQUENCY_HZ > 0.0f, "Minimum output frequency must be positive.");
static_assert(MAX_OUTPUT_FREQUENCY_HZ > MIN_OUTPUT_FREQUENCY_HZ, "Maximum output frequency must exceed minimum output frequency.");
static_assert(MAX_OUTPUT_FREQUENCY_HZ <= 2000.0f, "Review waveform timing before allowing output frequencies above 2 kHz.");
static_assert((LUT_MAX_SIZE & (LUT_MAX_SIZE - 1)) == 0, "LUT_MAX_SIZE must be a power of two.");
static_assert(LUT_MAX_SIZE >= 1024, "LUT_MAX_SIZE is too small for the DDS phase accumulator.");
static_assert(MAX_PRESET_SLOTS == 5, "Preset storage and UI currently expect exactly five preset slots.");
static_assert(AMP_TEMP_MIN_C < AMP_TEMP_WARN_C, "Amplifier warning temperature must exceed the minimum.");
static_assert(AMP_TEMP_WARN_C < AMP_TEMP_SHUTDOWN_C, "Amplifier shutdown temperature must exceed the warning temperature.");
static_assert(AMP_TEMP_SHUTDOWN_C <= AMP_TEMP_MAX_C, "Amplifier shutdown temperature must fit within the configured maximum.");
static_assert(AMP_TEMP_MIN_SHUTDOWN_MARGIN_C > 0.0f, "Amplifier thermal shutdown margin must be positive.");
static_assert(SETTINGS_SCHEMA_VERSION > 0, "Settings schema version must be positive.");
static_assert(SETTINGS_FILE_FORMAT_VERSION == 1, "Update settings file load/save code when changing the file format.");

#define TT_PIN_ASSERT_DISTINCT(a, b) static_assert((a) != (b), #a " must not share a GPIO with " #b)

TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_A, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_A, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_A, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_B, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_B, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_PWM_PHASE_C, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_I2C0_SDA, PIN_I2C0_SCL);
TT_PIN_ASSERT_DISTINCT(PIN_ENC_MAIN_CLK, PIN_ENC_MAIN_DT);
TT_PIN_ASSERT_DISTINCT(PIN_ENC_MAIN_CLK, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_ENC_MAIN_DT, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_PWM_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_I2C0_SDA);
TT_PIN_ASSERT_DISTINCT(PIN_RELAY_STANDBY, PIN_I2C0_SCL);

#if ENABLE_MUTE_RELAYS && !ENABLE_DPDT_RELAYS
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_A, PIN_MUTE_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_A, PIN_MUTE_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_A, PIN_MUTE_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_B, PIN_MUTE_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_B, PIN_MUTE_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_MUTE_PHASE_C, PIN_MUTE_PHASE_D);
#endif

#if PITCH_CONTROL_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_ENC_PITCH_CLK, PIN_ENC_PITCH_DT);
TT_PIN_ASSERT_DISTINCT(PIN_ENC_PITCH_CLK, PIN_ENC_PITCH_SW);
TT_PIN_ASSERT_DISTINCT(PIN_ENC_PITCH_DT, PIN_ENC_PITCH_SW);
#endif

#if STANDBY_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_BTN_STANDBY, PIN_RELAY_STANDBY);
TT_PIN_ASSERT_DISTINCT(PIN_BTN_STANDBY, PIN_ENC_MAIN_SW);
#endif
#if SPEED_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_BTN_SPEED, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_BTN_SPEED, PIN_RELAY_STANDBY);
#endif
#if START_STOP_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_BTN_START_STOP, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_BTN_START_STOP, PIN_RELAY_STANDBY);
#endif

#if AMP_MONITOR_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_AMP_TEMP, PIN_AMP_THERM_OK);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_TEMP, PIN_PWM_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_TEMP, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_TEMP, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_TEMP, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_THERM_OK, PIN_PWM_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_THERM_OK, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_THERM_OK, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_AMP_THERM_OK, PIN_PWM_PHASE_D);
#endif

#undef TT_PIN_ASSERT_DISTINCT

#endif // CONFIG_H
