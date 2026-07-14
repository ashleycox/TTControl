/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef CONFIG_H
#define CONFIG_H

/*
 * --- System Information ---
 * The version is embedded in the serial CLI and web diagnostics. BUILD_DATE can
 * be overridden by CI; otherwise Arduino's compile-time date is used.
 */
#define FIRMWARE_VERSION "v1.0.0"
#ifndef BUILD_DATE
#define BUILD_DATE __DATE__ " " __TIME__
#endif

/*
 * --- Flash Memory Configuration ---
 * Select a board flash layout with a non-zero LittleFS partition in the
 * Arduino-Pico board options. Partition size is a board property, not a sketch
 * macro; 8MB is recommended for PicoPlus2's 16MB flash.
 */

/*
 * --- Display Hardware ---
 * The UI renders into a one-bit canvas sized to the selected panel after
 * rotation. DisplayManager scales only when custom logical/physical dimensions
 * differ, while ui.cpp composes native compact, square, and large layouts.
 * Display selection is compile-time hardware configuration, not a persisted
 * setting, so a bad panel choice cannot make recovery depend on LittleFS.
 */
#define DISPLAY_DRIVER_NONE 0
#define DISPLAY_DRIVER_SSD1306 1
#define DISPLAY_DRIVER_SH1106 2
#define DISPLAY_DRIVER_SH1107 3
#define DISPLAY_DRIVER_SSD1327 4
#define DISPLAY_DRIVER_ST7735 5
#define DISPLAY_DRIVER_ST7789 6

#ifndef DISPLAY_DRIVER
#define DISPLAY_DRIVER DISPLAY_DRIVER_SSD1306
#endif

#define DISPLAY_TRANSPORT_I2C 0
#define DISPLAY_TRANSPORT_SOFTWARE_SPI 1
#define DISPLAY_TRANSPORT_HARDWARE_SPI 2

#ifndef DISPLAY_TRANSPORT
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735 || DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789
// Software SPI reuses the display-bus pins; the wiring profile below selects the remaining GPIO trade-off.
#define DISPLAY_TRANSPORT DISPLAY_TRANSPORT_SOFTWARE_SPI
#else
#define DISPLAY_TRANSPORT DISPLAY_TRANSPORT_I2C
#endif
#endif

/*
 * Standard-Pico wiring profiles make GPIO trade-offs explicit:
 * - MINIMAL: panel CS tied active, reset shared with RUN, fixed backlight.
 * - MANAGED: controlled CS and backlight; optional speed/standby buttons lost.
 * - FULL: controlled CS, reset, and backlight; all three optional buttons lost.
 * Individual PIN_DISPLAY_* overrides remain available for custom carrier boards.
 */
#define DISPLAY_WIRING_MINIMAL 0
#define DISPLAY_WIRING_MANAGED 1
#define DISPLAY_WIRING_FULL_CONTROL 2

#ifndef DISPLAY_WIRING_PROFILE
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#define DISPLAY_WIRING_PROFILE DISPLAY_WIRING_MINIMAL
#else
#define DISPLAY_WIRING_PROFILE DISPLAY_WIRING_MANAGED
#endif
#endif

#ifndef DISPLAY_I2C_ADDRESS
#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1107 || DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1327
#define DISPLAY_I2C_ADDRESS 0x3D
#else
#define DISPLAY_I2C_ADDRESS 0x3C
#endif
#endif
#ifndef DISPLAY_I2C_CLOCK_HZ
#define DISPLAY_I2C_CLOCK_HZ 400000UL
#endif
#ifndef DISPLAY_SPI_CLOCK_HZ
#define DISPLAY_SPI_CLOCK_HZ 12000000UL
#endif
#ifndef DISPLAY_SPI_PORT
#define DISPLAY_SPI_PORT 0
#endif
#ifndef DISPLAY_VALIDATE_STANDARD_SPI_PINS
#define DISPLAY_VALIDATE_STANDARD_SPI_PINS 1
#endif

// Constructor dimensions are before DISPLAY_ROTATION is applied.
#ifndef DISPLAY_PHYSICAL_WIDTH
#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1107 || DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1327
#define DISPLAY_PHYSICAL_WIDTH 128
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#define DISPLAY_PHYSICAL_WIDTH 128
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789
#define DISPLAY_PHYSICAL_WIDTH 240
#else
#define DISPLAY_PHYSICAL_WIDTH 128
#endif
#endif

#ifndef DISPLAY_PHYSICAL_HEIGHT
#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1107 || DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1327
#define DISPLAY_PHYSICAL_HEIGHT 128
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#define DISPLAY_PHYSICAL_HEIGHT 160
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789
#define DISPLAY_PHYSICAL_HEIGHT 240
#else
#define DISPLAY_PHYSICAL_HEIGHT 64
#endif
#endif

#ifndef DISPLAY_ROTATION
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#define DISPLAY_ROTATION 1
#else
#define DISPLAY_ROTATION 0
#endif
#endif

// Native UI geometry follows the rotated panel. Override only for unusual modules or deliberate scaling tests.
#ifndef DISPLAY_LOGICAL_WIDTH
#if DISPLAY_ROTATION == 1 || DISPLAY_ROTATION == 3
#define DISPLAY_LOGICAL_WIDTH DISPLAY_PHYSICAL_HEIGHT
#else
#define DISPLAY_LOGICAL_WIDTH DISPLAY_PHYSICAL_WIDTH
#endif
#endif
#ifndef DISPLAY_LOGICAL_HEIGHT
#if DISPLAY_ROTATION == 1 || DISPLAY_ROTATION == 3
#define DISPLAY_LOGICAL_HEIGHT DISPLAY_PHYSICAL_WIDTH
#else
#define DISPLAY_LOGICAL_HEIGHT DISPLAY_PHYSICAL_HEIGHT
#endif
#endif

// Aspect-fit remains available when custom logical and physical dimensions differ.
#ifndef DISPLAY_SCALE_TO_FIT
#define DISPLAY_SCALE_TO_FIT 1
#endif

// Rendering is deferred until the end of the Core 0 service loop, frame-capped, and skipped when the logical framebuffer is unchanged.
#ifndef DISPLAY_MAX_FPS
#if (DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735 || DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789) && \
    DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
#define DISPLAY_MAX_FPS 5
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735 || DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789
#define DISPLAY_MAX_FPS 10
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1327 && DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#define DISPLAY_MAX_FPS 5
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_SH1107 && DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#define DISPLAY_MAX_FPS 10
#else
#define DISPLAY_MAX_FPS 20
#endif
#endif

#ifndef DISPLAY_DIM_BRIGHTNESS
#define DISPLAY_DIM_BRIGHTNESS 16
#endif
#ifndef DISPLAY_TFT_FOREGROUND_COLOR
#define DISPLAY_TFT_FOREGROUND_COLOR 0xFFFF
#endif
#ifndef DISPLAY_TFT_BACKGROUND_COLOR
#define DISPLAY_TFT_BACKGROUND_COLOR 0x0000
#endif
#ifndef DISPLAY_TFT_INVERT
#define DISPLAY_TFT_INVERT 0
#endif

// ST7735 modules vary in controller offsets. These values map to the Adafruit initR profiles in display_backend_st77xx.cpp.
#define DISPLAY_ST7735_PROFILE_GREENTAB 0
#define DISPLAY_ST7735_PROFILE_REDTAB 1
#define DISPLAY_ST7735_PROFILE_BLACKTAB 2
#define DISPLAY_ST7735_PROFILE_MINI_160X80 3
#define DISPLAY_ST7735_PROFILE_HALLOWING 4
#ifndef DISPLAY_ST7735_PROFILE
#define DISPLAY_ST7735_PROFILE DISPLAY_ST7735_PROFILE_BLACKTAB
#endif

/*
 * --- Waveform Generation ---
 * The DDS LUT is power-of-two sized so phase accumulator wrapping can be cheap.
 * Frequency limits protect the motor and PWM/DMA timing budget.
 */
#define LUT_MAX_SIZE 16384
#define MIN_OUTPUT_FREQUENCY_HZ 10.0f
#define MAX_OUTPUT_FREQUENCY_HZ 1500.0f
#ifndef PWM_CARRIER_FREQUENCY_HZ
#define PWM_CARRIER_FREQUENCY_HZ 50000.0f
#endif

/*
 * --- Output Stage ---
 * TTControl can feed either the original filtered linear-amplifier interface or
 * a controller-free three-half-bridge board. The DRV8313/SimpleFOC-style 3-PWM
 * interface is the default for new builds; select OUTPUT_STAGE_LINEAR_PWM to
 * retain the original relay and disabled-duty behavior.
 */
#define OUTPUT_STAGE_LINEAR_PWM 0
#define OUTPUT_STAGE_3PWM_BRIDGE 1
#ifndef OUTPUT_STAGE_TYPE
#define OUTPUT_STAGE_TYPE OUTPUT_STAGE_3PWM_BRIDGE
#endif

#ifndef POWER_STAGE_ENABLE_ACTIVE_HIGH
#define POWER_STAGE_ENABLE_ACTIVE_HIGH 1
#endif
#ifndef POWER_STAGE_SHARED_ENABLE
#define POWER_STAGE_SHARED_ENABLE 1
#endif
#ifndef POWER_STAGE_FAULT_ENABLE
#define POWER_STAGE_FAULT_ENABLE 1
#endif
#ifndef POWER_STAGE_FAULT_ACTIVE_LOW
#define POWER_STAGE_FAULT_ACTIVE_LOW 1
#endif
#ifndef POWER_STAGE_PHASE_ENABLES
#define POWER_STAGE_PHASE_ENABLES 0
#endif
#ifndef POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH
#define POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH 1
#endif
#ifndef POWER_STAGE_SLEEP_ENABLE
#define POWER_STAGE_SLEEP_ENABLE 0
#endif
#ifndef POWER_STAGE_SLEEP_ACTIVE_HIGH
#define POWER_STAGE_SLEEP_ACTIVE_HIGH 1
#endif
#ifndef POWER_STAGE_RESET_ENABLE
#define POWER_STAGE_RESET_ENABLE 0
#endif
#ifndef POWER_STAGE_RESET_ACTIVE_HIGH
#define POWER_STAGE_RESET_ACTIVE_HIGH 0
#endif
#ifndef POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN
#define POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN 0
#endif
#ifndef POWER_STAGE_RESET_PULSE_MS
#define POWER_STAGE_RESET_PULSE_MS 2
#endif
#ifndef POWER_STAGE_PHASE_ENABLE_DELAY_MS
#define POWER_STAGE_PHASE_ENABLE_DELAY_MS 1
#endif
#ifndef POWER_STAGE_WAKE_DELAY_MS
#define POWER_STAGE_WAKE_DELAY_MS 2
#endif
#ifndef POWER_STAGE_NEUTRAL_BUFFER_COUNT
#define POWER_STAGE_NEUTRAL_BUFFER_COUNT 2
#endif

/*
 * --- Preset Management ---
 * The menu, serial CLI, web UI, and binary preset directory all assume five
 * user preset slots.
 */
#define MAX_PRESET_SLOTS 5

/*
 * --- Serial Debugging ---
 * SERIAL_MONITOR_ENABLE gates the CLI. DUPLICATE_DISPLAY_TO_SERIAL mirrors logical display
 * text for development without changing normal UI drawing.
 */
#ifndef SERIAL_MONITOR_ENABLE
#define SERIAL_MONITOR_ENABLE 1
#endif
#ifndef DUPLICATE_DISPLAY_TO_SERIAL
#define DUPLICATE_DISPLAY_TO_SERIAL 0
#endif

/*
 * --- Optional Features (Compile-time flags) ---
 * These flags describe hardware that may not be populated on every controller.
 * Keep them compile-time so absent GPIO and libraries do not affect lean builds.
 */
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

/*
 * --- Feature Flags ---
 * These change real output behavior. Be conservative when enabling them because
 * relay topology, connected amplifier protection, and output channel count vary
 * between builds.
 */
#ifndef ENABLE_STANDBY
#define ENABLE_STANDBY 1         // Set to 0 to disable all standby functionality
#endif
#ifndef ENABLE_MUTE_RELAYS
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_LINEAR_PWM
#define ENABLE_MUTE_RELAYS 1     // Linear output default: downstream amplifier mute relays are fitted
#else
#define ENABLE_MUTE_RELAYS 0     // Bridge output default: GP16-19 belong to the power-stage interface
#endif
#endif
#ifndef ENABLE_DPDT_RELAYS
#define ENABLE_DPDT_RELAYS 0     // Set to 1 to use 2x DPDT relays instead of 4x SPST
#endif
#ifndef ENABLE_4_CHANNEL_SUPPORT
#define ENABLE_4_CHANNEL_SUPPORT 0 // Set to 1 for optional four-channel linear/twin-motor output modes
#endif
#ifndef AMP_MONITOR_ENABLE
#define AMP_MONITOR_ENABLE 0       // Set to 1 only when the amplifier thermal monitor pins are populated and driven
#endif
#ifndef CLOSED_LOOP_SPEED_ENABLE
#define CLOSED_LOOP_SPEED_ENABLE 0 // Enable optional tachometer/quadrature speed feedback
#endif
#ifndef CLOSED_LOOP_TREND_SIZE
#define CLOSED_LOOP_TREND_SIZE 24  // Rolling runtime samples retained for closed-loop trend diagnostics
#endif

// The active channel count is derived once so waveform, menu, and diagnostics code can share the same feature-gated boundary.
#if ENABLE_4_CHANNEL_SUPPORT
#define MAX_PHASE_MODE 4
#define MAX_ACTIVE_PHASE_OUTPUTS 4
#else
#define MAX_PHASE_MODE 3
#define MAX_ACTIVE_PHASE_OUTPUTS 3
#endif

/*
 * --- Pin Assignments (RP2040/RP2350 / Arduino-Pico) ---
 * GPIO numbers are physical controller wiring, not arbitrary Arduino pins.
 * Changing them can energize the wrong relay or waveform output.
 */

/*
 * PWM Outputs (Waveform Generation)
 * Channels A-D are adjacent so the waveform generator can use predictable PWM
 * slice/channel mapping.
 */
#define PIN_PWM_PHASE_A 0
#define PIN_PWM_PHASE_B 1
#define PIN_PWM_PHASE_C 2
#define PIN_PWM_PHASE_D 3

// I2C display interface. SPI builds reuse GP4/GP5 as software-SPI clock/data by default.
#define PIN_I2C0_SDA 4
#define PIN_I2C0_SCL 5

/*
 * SPI display wiring.
 *
 * All standard profiles use GP4/GP5 and GP28 for SPI clock/data and D/C. The
 * managed profile adds CS on the optional speed-button pin and backlight on the
 * optional standby-button pin. Full control also uses the optional start/stop
 * pin for backlight and moves reset to GP21. Compile-time pin checks reject any
 * optional control that a selected display profile has consumed.
 */
#ifndef PIN_DISPLAY_SPI_SCK
#define PIN_DISPLAY_SPI_SCK PIN_I2C0_SDA
#endif
#ifndef PIN_DISPLAY_SPI_MOSI
#define PIN_DISPLAY_SPI_MOSI PIN_I2C0_SCL
#endif
#ifndef PIN_DISPLAY_DC
#define PIN_DISPLAY_DC 28
#endif
#ifndef PIN_DISPLAY_CS
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C || DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_MINIMAL
#define PIN_DISPLAY_CS -1
#else
#define PIN_DISPLAY_CS PIN_BTN_SPEED
#endif
#endif
#ifndef PIN_DISPLAY_RESET
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_FULL_CONTROL
#define PIN_DISPLAY_RESET PIN_BTN_STANDBY
#else
#define PIN_DISPLAY_RESET -1
#endif
#endif
#ifndef PIN_DISPLAY_BACKLIGHT
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_MANAGED
#define PIN_DISPLAY_BACKLIGHT PIN_BTN_STANDBY
#elif DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_FULL_CONTROL
#define PIN_DISPLAY_BACKLIGHT PIN_BTN_START_STOP
#else
#define PIN_DISPLAY_BACKLIGHT -1
#endif
#endif
#ifndef DISPLAY_BACKLIGHT_ACTIVE_HIGH
#define DISPLAY_BACKLIGHT_ACTIVE_HIGH 1
#endif

// Main Encoder
#define PIN_ENC_MAIN_CLK 10
#define PIN_ENC_MAIN_DT 11
#define PIN_ENC_MAIN_SW 12

/*
 * Optional Pins (only used if enabled above)
 * Pitch encoder and discrete buttons are compiled out unless their feature flag
 * is enabled, which keeps unpopulated pins from being polled.
 */
#define PIN_ENC_PITCH_CLK 13
#define PIN_ENC_PITCH_DT 14
#define PIN_ENC_PITCH_SW 15

#define PIN_RELAY_STANDBY 16
#define PIN_MUTE_PHASE_A 17
#define PIN_MUTE_PHASE_B 18
#define PIN_MUTE_PHASE_C 19
#define PIN_MUTE_PHASE_D 20

/*
 * DPDT Relay Pins (only used if ENABLE_DPDT_RELAYS is 1)
 * Defaults to reusing Phase A and Phase B mute pins
 */
#define PIN_RELAY_DPDT_1 PIN_MUTE_PHASE_A
#define PIN_RELAY_DPDT_2 PIN_MUTE_PHASE_B

#define PIN_BTN_STANDBY 21
#define PIN_BTN_SPEED 22
#define PIN_BTN_START_STOP 9
#define PIN_SPEED_SENSOR_A 6
#define PIN_SPEED_SENSOR_B 7
#define PIN_AMP_TEMP 26
#define PIN_AMP_THERM_OK 27

/*
 * Default controller-free DRV8313/SimpleFOC-style bridge interface. Boards
 * that expose EN1/EN2/EN3 separately can enable POWER_STAGE_PHASE_ENABLES and
 * use GP17-19. Bridge-only sleep/reset reuse GP20/GP3, which are reserved for
 * fourth-channel linear hardware and therefore unused by the bridge backend.
 */
#ifndef PIN_POWER_STAGE_ENABLE
#define PIN_POWER_STAGE_ENABLE 16
#endif
#ifndef PIN_POWER_STAGE_PHASE_ENABLE_A
#define PIN_POWER_STAGE_PHASE_ENABLE_A 17
#endif
#ifndef PIN_POWER_STAGE_PHASE_ENABLE_B
#define PIN_POWER_STAGE_PHASE_ENABLE_B 18
#endif
#ifndef PIN_POWER_STAGE_PHASE_ENABLE_C
#define PIN_POWER_STAGE_PHASE_ENABLE_C 19
#endif
#ifndef PIN_POWER_STAGE_FAULT
#define PIN_POWER_STAGE_FAULT 8
#endif
#ifndef PIN_POWER_STAGE_SLEEP
#define PIN_POWER_STAGE_SLEEP 20
#endif
#ifndef PIN_POWER_STAGE_RESET
#define PIN_POWER_STAGE_RESET 3
#endif

// Thermal values are validated in settings so user-entered thresholds stay in the same safe range as these firmware defaults.
#define AMP_TEMP_WARN_C 65.0f
#define AMP_TEMP_SHUTDOWN_C 75.0f
#define AMP_TEMP_MIN_C 30.0f
#define AMP_TEMP_MAX_C 120.0f
#define AMP_TEMP_MIN_SHUTDOWN_MARGIN_C 1.0f
#define AMP_TEMP_WARN_HYSTERESIS_C 5.0f

/*
 * --- Network Defaults (Wi-Fi boards only) ---
 * Magic/version identify the binary network config file independently from the
 * motor settings schema.
 */
#define NETWORK_CONFIG_MAGIC 0x54545746UL
#define NETWORK_CONFIG_VERSION 5
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

/*
 * --- Storage Schema ---
 * These sizes intentionally mirror sizeof() checks in types.h. If a persisted
 * struct changes, bump SETTINGS_SCHEMA_VERSION and add migration code before
 * changing the expected size.
 */
#define SETTINGS_SCHEMA_VERSION 12
#define SETTINGS_FILE_FORMAT_VERSION 1
#define SETTINGS_FILE_MAGIC 0x54544353UL // "TTCS"
#define PRESET_FILE_MAGIC 0x54544350UL   // "TTCP"
#define SPEED_SETTINGS_STORAGE_SIZE 56
#define CLOSED_LOOP_TUNING_STORAGE_SIZE 44
#define GLOBAL_SETTINGS_STORAGE_SIZE 620

// --- Default Values ---
#define DEFAULT_PHASE_MODE 3 // 3-phase
#define DEFAULT_SPEED_INDEX 0 // 33.3 RPM
#define DEFAULT_MOTOR_TOPOLOGY 2 // Balanced three-phase, with fully editable phase/gain tuning
#define DEFAULT_VF_BASE_FREQUENCY_HZ 58.66f

/*
 * --- Compile-time Safety Checks ---
 * These fail the build before a bad hardware combination reaches a board.
 */
#if DISPLAY_DRIVER < DISPLAY_DRIVER_NONE || DISPLAY_DRIVER > DISPLAY_DRIVER_ST7789
#error "DISPLAY_DRIVER is not a supported display backend."
#endif
#if DISPLAY_TRANSPORT < DISPLAY_TRANSPORT_I2C || DISPLAY_TRANSPORT > DISPLAY_TRANSPORT_HARDWARE_SPI
#error "DISPLAY_TRANSPORT is not supported."
#endif
#if DISPLAY_WIRING_PROFILE < DISPLAY_WIRING_MINIMAL || DISPLAY_WIRING_PROFILE > DISPLAY_WIRING_FULL_CONTROL
#error "DISPLAY_WIRING_PROFILE must be MINIMAL, MANAGED, or FULL_CONTROL."
#endif
#if (DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735 || DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789) && DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#error "ST7735/ST7789 displays require an SPI transport."
#endif
#if DISPLAY_DRIVER == DISPLAY_DRIVER_NONE && DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C
#error "The headless display backend does not use an SPI transport."
#endif
#if DISPLAY_SPI_PORT != 0 && DISPLAY_SPI_PORT != 1
#error "DISPLAY_SPI_PORT must be 0 (SPI) or 1 (SPI1)."
#endif
#if DISPLAY_VALIDATE_STANDARD_SPI_PINS != 0 && DISPLAY_VALIDATE_STANDARD_SPI_PINS != 1
#error "DISPLAY_VALIDATE_STANDARD_SPI_PINS must be 0 or 1."
#endif
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_HARDWARE_SPI && DISPLAY_VALIDATE_STANDARD_SPI_PINS
#if DISPLAY_SPI_PORT == 0 && !((PIN_DISPLAY_SPI_SCK == 2 && PIN_DISPLAY_SPI_MOSI == 3) || \
                              (PIN_DISPLAY_SPI_SCK == 6 && PIN_DISPLAY_SPI_MOSI == 7) || \
                              (PIN_DISPLAY_SPI_SCK == 18 && PIN_DISPLAY_SPI_MOSI == 19))
#error "SPI0 display pins must use a standard Pico SCK/MOSI pair: GP2/3, GP6/7, or GP18/19."
#endif
#if DISPLAY_SPI_PORT == 1 && !((PIN_DISPLAY_SPI_SCK == 10 && PIN_DISPLAY_SPI_MOSI == 11) || \
                              (PIN_DISPLAY_SPI_SCK == 14 && PIN_DISPLAY_SPI_MOSI == 15) || \
                              (PIN_DISPLAY_SPI_SCK == 26 && PIN_DISPLAY_SPI_MOSI == 27))
#error "SPI1 display pins must use a standard Pico SCK/MOSI pair: GP10/11, GP14/15, or GP26/27."
#endif
#endif
#if DISPLAY_ROTATION < 0 || DISPLAY_ROTATION > 3
#error "DISPLAY_ROTATION must be 0, 1, 2, or 3."
#endif
#if DISPLAY_MAX_FPS < 1 || DISPLAY_MAX_FPS > 60
#error "DISPLAY_MAX_FPS must be between 1 and 60."
#endif
#if (DISPLAY_SCALE_TO_FIT != 0 && DISPLAY_SCALE_TO_FIT != 1)
#error "DISPLAY_SCALE_TO_FIT must be 0 or 1."
#endif
#if (DISPLAY_TFT_INVERT != 0 && DISPLAY_TFT_INVERT != 1)
#error "DISPLAY_TFT_INVERT must be 0 or 1."
#endif
#if (DISPLAY_BACKLIGHT_ACTIVE_HIGH != 0 && DISPLAY_BACKLIGHT_ACTIVE_HIGH != 1)
#error "DISPLAY_BACKLIGHT_ACTIVE_HIGH must be 0 or 1."
#endif
#if DISPLAY_ST7735_PROFILE < DISPLAY_ST7735_PROFILE_GREENTAB || DISPLAY_ST7735_PROFILE > DISPLAY_ST7735_PROFILE_HALLOWING
#error "DISPLAY_ST7735_PROFILE is not supported."
#endif
#if DISPLAY_PHYSICAL_WIDTH <= 0 || DISPLAY_PHYSICAL_HEIGHT <= 0
#error "Display dimensions must be positive."
#endif
#if DISPLAY_LOGICAL_WIDTH <= 0 || DISPLAY_LOGICAL_HEIGHT <= 0
#error "Logical display dimensions must be positive."
#endif
#if DISPLAY_LOGICAL_WIDTH * DISPLAY_LOGICAL_HEIGHT > 65536
#error "The one-bit UI canvas is limited to 65,536 pixels; choose smaller logical dimensions."
#endif
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && PIN_DISPLAY_DC < 0
#error "SPI displays require PIN_DISPLAY_DC."
#endif
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && (PIN_DISPLAY_SPI_SCK < 0 || PIN_DISPLAY_SPI_MOSI < 0)
#error "SPI displays require clock and MOSI pins."
#endif
#if DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1306 && DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && PIN_DISPLAY_CS < 0
#error "Adafruit SSD1306 requires a controlled chip-select pin in SPI mode; use I2C or assign PIN_DISPLAY_CS."
#endif
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && SPEED_BUTTON_ENABLE && \
    (PIN_DISPLAY_CS == PIN_BTN_SPEED || PIN_DISPLAY_RESET == PIN_BTN_SPEED || PIN_DISPLAY_BACKLIGHT == PIN_BTN_SPEED)
#error "The selected SPI display wiring uses GP22, so SPEED_BUTTON_ENABLE must be 0 or the display pin must be overridden."
#endif
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && STANDBY_BUTTON_ENABLE && \
    (PIN_DISPLAY_CS == PIN_BTN_STANDBY || PIN_DISPLAY_RESET == PIN_BTN_STANDBY || PIN_DISPLAY_BACKLIGHT == PIN_BTN_STANDBY)
#error "The selected SPI display wiring uses GP21, so STANDBY_BUTTON_ENABLE must be 0 or the display pin must be overridden."
#endif
#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C && START_STOP_BUTTON_ENABLE && \
    (PIN_DISPLAY_CS == PIN_BTN_START_STOP || PIN_DISPLAY_RESET == PIN_BTN_START_STOP || PIN_DISPLAY_BACKLIGHT == PIN_BTN_START_STOP)
#error "The selected SPI display wiring uses GP9, so START_STOP_BUTTON_ENABLE must be 0 or the display pin must be overridden."
#endif

#if NETWORK_ENABLE && !defined(PICO_CYW43_SUPPORTED)
#error "NETWORK_ENABLE requires a Wi-Fi-capable Pico/RP board target."
#endif

#if (SERIAL_MONITOR_ENABLE != 0 && SERIAL_MONITOR_ENABLE != 1)
#error "SERIAL_MONITOR_ENABLE must be 0 or 1."
#endif
#if (DUPLICATE_DISPLAY_TO_SERIAL != 0 && DUPLICATE_DISPLAY_TO_SERIAL != 1)
#error "DUPLICATE_DISPLAY_TO_SERIAL must be 0 or 1."
#endif
#if (NETWORK_ENABLE != 0 && NETWORK_ENABLE != 1)
#error "NETWORK_ENABLE must be 0 or 1."
#endif
#if (PITCH_CONTROL_ENABLE != 0 && PITCH_CONTROL_ENABLE != 1)
#error "PITCH_CONTROL_ENABLE must be 0 or 1."
#endif
#if (STANDBY_BUTTON_ENABLE != 0 && STANDBY_BUTTON_ENABLE != 1)
#error "STANDBY_BUTTON_ENABLE must be 0 or 1."
#endif
#if (SPEED_BUTTON_ENABLE != 0 && SPEED_BUTTON_ENABLE != 1)
#error "SPEED_BUTTON_ENABLE must be 0 or 1."
#endif
#if (START_STOP_BUTTON_ENABLE != 0 && START_STOP_BUTTON_ENABLE != 1)
#error "START_STOP_BUTTON_ENABLE must be 0 or 1."
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
#if (CLOSED_LOOP_SPEED_ENABLE != 0 && CLOSED_LOOP_SPEED_ENABLE != 1)
#error "CLOSED_LOOP_SPEED_ENABLE must be 0 or 1."
#endif
#if (OUTPUT_STAGE_TYPE != OUTPUT_STAGE_LINEAR_PWM && OUTPUT_STAGE_TYPE != OUTPUT_STAGE_3PWM_BRIDGE)
#error "OUTPUT_STAGE_TYPE must select OUTPUT_STAGE_LINEAR_PWM or OUTPUT_STAGE_3PWM_BRIDGE."
#endif
#if (POWER_STAGE_ENABLE_ACTIVE_HIGH != 0 && POWER_STAGE_ENABLE_ACTIVE_HIGH != 1)
#error "POWER_STAGE_ENABLE_ACTIVE_HIGH must be 0 or 1."
#endif
#if (POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH != 0 && POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH != 1)
#error "POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH must be 0 or 1."
#endif
#if (POWER_STAGE_SLEEP_ACTIVE_HIGH != 0 && POWER_STAGE_SLEEP_ACTIVE_HIGH != 1)
#error "POWER_STAGE_SLEEP_ACTIVE_HIGH must be 0 or 1."
#endif
#if (POWER_STAGE_RESET_ACTIVE_HIGH != 0 && POWER_STAGE_RESET_ACTIVE_HIGH != 1)
#error "POWER_STAGE_RESET_ACTIVE_HIGH must be 0 or 1."
#endif
#if (POWER_STAGE_FAULT_ENABLE != 0 && POWER_STAGE_FAULT_ENABLE != 1)
#error "POWER_STAGE_FAULT_ENABLE must be 0 or 1."
#endif
#if (POWER_STAGE_FAULT_ACTIVE_LOW != 0 && POWER_STAGE_FAULT_ACTIVE_LOW != 1)
#error "POWER_STAGE_FAULT_ACTIVE_LOW must be 0 or 1."
#endif
#if (POWER_STAGE_PHASE_ENABLES != 0 && POWER_STAGE_PHASE_ENABLES != 1)
#error "POWER_STAGE_PHASE_ENABLES must be 0 or 1."
#endif
#if (POWER_STAGE_SHARED_ENABLE != 0 && POWER_STAGE_SHARED_ENABLE != 1)
#error "POWER_STAGE_SHARED_ENABLE must be 0 or 1."
#endif
#if (POWER_STAGE_SLEEP_ENABLE != 0 && POWER_STAGE_SLEEP_ENABLE != 1)
#error "POWER_STAGE_SLEEP_ENABLE must be 0 or 1."
#endif
#if (POWER_STAGE_RESET_ENABLE != 0 && POWER_STAGE_RESET_ENABLE != 1)
#error "POWER_STAGE_RESET_ENABLE must be 0 or 1."
#endif
#if (POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN != 0 && POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN != 1)
#error "POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN must be 0 or 1."
#endif

#if ENABLE_DPDT_RELAYS && !ENABLE_MUTE_RELAYS
#error "ENABLE_DPDT_RELAYS requires ENABLE_MUTE_RELAYS."
#endif
#if ENABLE_DPDT_RELAYS && ENABLE_4_CHANNEL_SUPPORT
#error "DPDT relay mode does not provide independent mute control for four phase outputs."
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && ENABLE_4_CHANNEL_SUPPORT
#error "The 3-PWM bridge backend provides three half-bridge outputs; use the linear backend for four-channel output."
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && ENABLE_DPDT_RELAYS
#error "DPDT relay output is only supported by the linear-amplifier backend."
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && ENABLE_MUTE_RELAYS
#error "Mute relays belong to the linear-amplifier backend and are not supported by the 3-PWM bridge backend."
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && !POWER_STAGE_SHARED_ENABLE && !POWER_STAGE_PHASE_ENABLES && !POWER_STAGE_SLEEP_ENABLE
#error "Bridge output requires at least one hardware disable path: shared enable, phase enables, or sleep."
#endif
#if POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN && (!POWER_STAGE_SHARED_ENABLE || !POWER_STAGE_FAULT_ENABLE)
#error "Shared open-drain enable/fault requires shared enable and fault support."
#endif

static_assert(MIN_OUTPUT_FREQUENCY_HZ > 0.0f, "Minimum output frequency must be positive.");
static_assert(MAX_OUTPUT_FREQUENCY_HZ > MIN_OUTPUT_FREQUENCY_HZ, "Maximum output frequency must exceed minimum output frequency.");
static_assert(MAX_OUTPUT_FREQUENCY_HZ <= 2000.0f, "Review waveform timing before allowing output frequencies above 2 kHz.");
static_assert(PWM_CARRIER_FREQUENCY_HZ >= 20000.0f && PWM_CARRIER_FREQUENCY_HZ <= 100000.0f, "PWM carrier must remain inside the supported power-stage range.");
static_assert(POWER_STAGE_WAKE_DELAY_MS <= 1000, "Power-stage wake delay must remain non-blocking and reasonably short.");
static_assert(POWER_STAGE_RESET_PULSE_MS <= 1000, "Power-stage reset pulse must remain non-blocking.");
static_assert(POWER_STAGE_PHASE_ENABLE_DELAY_MS <= 1000, "Power-stage phase-enable delay must remain non-blocking.");
static_assert(POWER_STAGE_NEUTRAL_BUFFER_COUNT >= 1 && POWER_STAGE_NEUTRAL_BUFFER_COUNT <= 8, "Neutral buffer confirmation count is unreasonable.");
static_assert((LUT_MAX_SIZE & (LUT_MAX_SIZE - 1)) == 0, "LUT_MAX_SIZE must be a power of two.");
static_assert(LUT_MAX_SIZE >= 1024, "LUT_MAX_SIZE is too small for the DDS phase accumulator.");
static_assert(MAX_PRESET_SLOTS == 5, "Preset storage and UI currently expect exactly five preset slots.");
static_assert(AMP_TEMP_MIN_C < AMP_TEMP_WARN_C, "Amplifier warning temperature must exceed the minimum.");
static_assert(AMP_TEMP_WARN_C < AMP_TEMP_SHUTDOWN_C, "Amplifier shutdown temperature must exceed the warning temperature.");
static_assert(AMP_TEMP_SHUTDOWN_C <= AMP_TEMP_MAX_C, "Amplifier shutdown temperature must fit within the configured maximum.");
static_assert(AMP_TEMP_MIN_SHUTDOWN_MARGIN_C > 0.0f, "Amplifier thermal shutdown margin must be positive.");
static_assert(SETTINGS_SCHEMA_VERSION > 0, "Settings schema version must be positive.");
static_assert(SETTINGS_FILE_FORMAT_VERSION == 1, "Update settings file load/save code when changing the file format.");
static_assert(CLOSED_LOOP_TREND_SIZE > 0 && CLOSED_LOOP_TREND_SIZE <= 64, "Closed-loop trend size must stay small and non-zero.");

// Pin uniqueness checks cover the always-present wiring first, then add checks for feature-gated hardware blocks below.
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

#if DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_SPI_MOSI);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_DC);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_SPI_MOSI, PIN_DISPLAY_DC);
#if PIN_DISPLAY_CS >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_CS, PIN_DISPLAY_SPI_SCK);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_CS, PIN_DISPLAY_SPI_MOSI);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_CS, PIN_DISPLAY_DC);
#endif
#if PIN_DISPLAY_RESET >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_RESET, PIN_DISPLAY_SPI_SCK);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_RESET, PIN_DISPLAY_SPI_MOSI);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_RESET, PIN_DISPLAY_DC);
#if PIN_DISPLAY_CS >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_RESET, PIN_DISPLAY_CS);
#endif
#endif
#if PIN_DISPLAY_BACKLIGHT >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_BACKLIGHT, PIN_DISPLAY_SPI_SCK);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_BACKLIGHT, PIN_DISPLAY_SPI_MOSI);
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_BACKLIGHT, PIN_DISPLAY_DC);
#if PIN_DISPLAY_CS >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_BACKLIGHT, PIN_DISPLAY_CS);
#endif
#if PIN_DISPLAY_RESET >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_BACKLIGHT, PIN_DISPLAY_RESET);
#endif
#endif

// Every active SPI-display pin is checked against always-present controller resources first.
#define TT_DISPLAY_PIN_ASSERT_BASE(pin) \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_A); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_B); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_C); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_CLK); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_DT); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_SW)

TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_SPI_SCK);
TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_SPI_MOSI);
TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_DC);
#if PIN_DISPLAY_CS >= 0
TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_CS);
#endif
#if PIN_DISPLAY_RESET >= 0
TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_RESET);
#endif
#if PIN_DISPLAY_BACKLIGHT >= 0
TT_DISPLAY_PIN_ASSERT_BASE(PIN_DISPLAY_BACKLIGHT);
#endif

#define TT_DISPLAY_ASSERT_ACTIVE_PIN(resourcePin) \
    TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_SPI_SCK, resourcePin); \
    TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_SPI_MOSI, resourcePin); \
    TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_DC, resourcePin)

#define TT_DISPLAY_ASSERT_OPTIONAL_CONTROLS(resourcePin) \
    static_assert(PIN_DISPLAY_CS < 0 || PIN_DISPLAY_CS != (resourcePin), "PIN_DISPLAY_CS conflicts with " #resourcePin); \
    static_assert(PIN_DISPLAY_RESET < 0 || PIN_DISPLAY_RESET != (resourcePin), "PIN_DISPLAY_RESET conflicts with " #resourcePin); \
    static_assert(PIN_DISPLAY_BACKLIGHT < 0 || PIN_DISPLAY_BACKLIGHT != (resourcePin), "PIN_DISPLAY_BACKLIGHT conflicts with " #resourcePin)

#define TT_DISPLAY_ASSERT_RESOURCE(resourcePin) \
    TT_DISPLAY_ASSERT_ACTIVE_PIN(resourcePin); \
    TT_DISPLAY_ASSERT_OPTIONAL_CONTROLS(resourcePin)

#if ENABLE_4_CHANNEL_SUPPORT
TT_DISPLAY_ASSERT_RESOURCE(PIN_PWM_PHASE_D);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_LINEAR_PWM
TT_DISPLAY_ASSERT_RESOURCE(PIN_RELAY_STANDBY);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_FAULT_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_FAULT);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SHARED_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_ENABLE);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_PHASE_ENABLES
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_A);
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_B);
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_C);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SLEEP_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_SLEEP);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_RESET_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_POWER_STAGE_RESET);
#endif
#if ENABLE_MUTE_RELAYS
#if ENABLE_DPDT_RELAYS
TT_DISPLAY_ASSERT_RESOURCE(PIN_RELAY_DPDT_1);
TT_DISPLAY_ASSERT_RESOURCE(PIN_RELAY_DPDT_2);
#else
TT_DISPLAY_ASSERT_RESOURCE(PIN_MUTE_PHASE_A);
TT_DISPLAY_ASSERT_RESOURCE(PIN_MUTE_PHASE_B);
TT_DISPLAY_ASSERT_RESOURCE(PIN_MUTE_PHASE_C);
#if ENABLE_4_CHANNEL_SUPPORT
TT_DISPLAY_ASSERT_RESOURCE(PIN_MUTE_PHASE_D);
#endif
#endif
#endif
#if PITCH_CONTROL_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_ENC_PITCH_CLK);
TT_DISPLAY_ASSERT_RESOURCE(PIN_ENC_PITCH_DT);
TT_DISPLAY_ASSERT_RESOURCE(PIN_ENC_PITCH_SW);
#endif
#if STANDBY_BUTTON_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_BTN_STANDBY);
#endif
#if SPEED_BUTTON_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_BTN_SPEED);
#endif
#if START_STOP_BUTTON_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_BTN_START_STOP);
#endif
#if CLOSED_LOOP_SPEED_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_SPEED_SENSOR_A);
TT_DISPLAY_ASSERT_RESOURCE(PIN_SPEED_SENSOR_B);
#endif
#if AMP_MONITOR_ENABLE
TT_DISPLAY_ASSERT_RESOURCE(PIN_AMP_TEMP);
TT_DISPLAY_ASSERT_RESOURCE(PIN_AMP_THERM_OK);
#endif

#undef TT_DISPLAY_ASSERT_RESOURCE
#undef TT_DISPLAY_ASSERT_OPTIONAL_CONTROLS
#undef TT_DISPLAY_ASSERT_ACTIVE_PIN
#undef TT_DISPLAY_PIN_ASSERT_BASE
#endif

// I2C normally consumes only GP4/GP5, but optional reset/backlight controls still need the same active-feature collision checks.
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#if PIN_DISPLAY_RESET >= 0 && PIN_DISPLAY_BACKLIGHT >= 0
TT_PIN_ASSERT_DISTINCT(PIN_DISPLAY_RESET, PIN_DISPLAY_BACKLIGHT);
#endif

#define TT_I2C_DISPLAY_CONTROL_ASSERT_BASE(pin) \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_I2C0_SDA); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_I2C0_SCL); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_A); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_B); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_PWM_PHASE_C); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_CLK); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_DT); \
    TT_PIN_ASSERT_DISTINCT(pin, PIN_ENC_MAIN_SW)

#if PIN_DISPLAY_RESET >= 0
TT_I2C_DISPLAY_CONTROL_ASSERT_BASE(PIN_DISPLAY_RESET);
#endif
#if PIN_DISPLAY_BACKLIGHT >= 0
TT_I2C_DISPLAY_CONTROL_ASSERT_BASE(PIN_DISPLAY_BACKLIGHT);
#endif

#define TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(resourcePin) \
    static_assert(PIN_DISPLAY_RESET < 0 || PIN_DISPLAY_RESET != (resourcePin), "PIN_DISPLAY_RESET conflicts with " #resourcePin); \
    static_assert(PIN_DISPLAY_BACKLIGHT < 0 || PIN_DISPLAY_BACKLIGHT != (resourcePin), "PIN_DISPLAY_BACKLIGHT conflicts with " #resourcePin)

#if ENABLE_4_CHANNEL_SUPPORT
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_PWM_PHASE_D);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_LINEAR_PWM
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_RELAY_STANDBY);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_FAULT_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_FAULT);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SHARED_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_ENABLE);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_PHASE_ENABLES
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_A);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_B);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_PHASE_ENABLE_C);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_SLEEP_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_SLEEP);
#endif
#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE && POWER_STAGE_RESET_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_POWER_STAGE_RESET);
#endif
#if ENABLE_MUTE_RELAYS
#if ENABLE_DPDT_RELAYS
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_RELAY_DPDT_1);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_RELAY_DPDT_2);
#else
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_MUTE_PHASE_A);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_MUTE_PHASE_B);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_MUTE_PHASE_C);
#if ENABLE_4_CHANNEL_SUPPORT
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_MUTE_PHASE_D);
#endif
#endif
#endif
#if PITCH_CONTROL_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_ENC_PITCH_CLK);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_ENC_PITCH_DT);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_ENC_PITCH_SW);
#endif
#if STANDBY_BUTTON_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_BTN_STANDBY);
#endif
#if SPEED_BUTTON_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_BTN_SPEED);
#endif
#if START_STOP_BUTTON_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_BTN_START_STOP);
#endif
#if CLOSED_LOOP_SPEED_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_SPEED_SENSOR_A);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_SPEED_SENSOR_B);
#endif
#if AMP_MONITOR_ENABLE
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_AMP_TEMP);
TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE(PIN_AMP_THERM_OK);
#endif

#undef TT_I2C_DISPLAY_CONTROL_ASSERT_RESOURCE
#undef TT_I2C_DISPLAY_CONTROL_ASSERT_BASE
#endif

#if OUTPUT_STAGE_TYPE == OUTPUT_STAGE_3PWM_BRIDGE
#define TT_POWER_STAGE_ASSERT_RESOURCE(resourcePin) \
    static_assert(!POWER_STAGE_SHARED_ENABLE || PIN_POWER_STAGE_ENABLE != (resourcePin), "PIN_POWER_STAGE_ENABLE conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_FAULT_ENABLE || PIN_POWER_STAGE_FAULT != (resourcePin), "PIN_POWER_STAGE_FAULT conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_PHASE_ENABLES || PIN_POWER_STAGE_PHASE_ENABLE_A != (resourcePin), "PIN_POWER_STAGE_PHASE_ENABLE_A conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_PHASE_ENABLES || PIN_POWER_STAGE_PHASE_ENABLE_B != (resourcePin), "PIN_POWER_STAGE_PHASE_ENABLE_B conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_PHASE_ENABLES || PIN_POWER_STAGE_PHASE_ENABLE_C != (resourcePin), "PIN_POWER_STAGE_PHASE_ENABLE_C conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_SLEEP_ENABLE || PIN_POWER_STAGE_SLEEP != (resourcePin), "PIN_POWER_STAGE_SLEEP conflicts with " #resourcePin); \
    static_assert(!POWER_STAGE_RESET_ENABLE || PIN_POWER_STAGE_RESET != (resourcePin), "PIN_POWER_STAGE_RESET conflicts with " #resourcePin)

TT_POWER_STAGE_ASSERT_RESOURCE(PIN_PWM_PHASE_A);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_PWM_PHASE_B);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_PWM_PHASE_C);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_I2C0_SDA);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_I2C0_SCL);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_MAIN_CLK);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_MAIN_DT);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_MAIN_SW);
#if PITCH_CONTROL_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_PITCH_CLK);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_PITCH_DT);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_ENC_PITCH_SW);
#endif
#if STANDBY_BUTTON_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_BTN_STANDBY);
#endif
#if SPEED_BUTTON_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_BTN_SPEED);
#endif
#if START_STOP_BUTTON_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_BTN_START_STOP);
#endif
#if CLOSED_LOOP_SPEED_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_SPEED_SENSOR_A);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_SPEED_SENSOR_B);
#endif
#if AMP_MONITOR_ENABLE
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_AMP_TEMP);
TT_POWER_STAGE_ASSERT_RESOURCE(PIN_AMP_THERM_OK);
#endif

static_assert(!POWER_STAGE_FAULT_ENABLE || !POWER_STAGE_SHARED_ENABLE || POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN ||
              PIN_POWER_STAGE_FAULT != PIN_POWER_STAGE_ENABLE,
              "PIN_POWER_STAGE_FAULT must not share PIN_POWER_STAGE_ENABLE unless shared open-drain mode is enabled.");
static_assert(!POWER_STAGE_PHASE_ENABLES || !POWER_STAGE_SHARED_ENABLE ||
              (PIN_POWER_STAGE_PHASE_ENABLE_A != PIN_POWER_STAGE_ENABLE &&
               PIN_POWER_STAGE_PHASE_ENABLE_B != PIN_POWER_STAGE_ENABLE &&
               PIN_POWER_STAGE_PHASE_ENABLE_C != PIN_POWER_STAGE_ENABLE),
              "Phase-enable pins must not share PIN_POWER_STAGE_ENABLE.");
static_assert(!POWER_STAGE_PHASE_ENABLES || !POWER_STAGE_FAULT_ENABLE ||
              (PIN_POWER_STAGE_PHASE_ENABLE_A != PIN_POWER_STAGE_FAULT &&
               PIN_POWER_STAGE_PHASE_ENABLE_B != PIN_POWER_STAGE_FAULT &&
               PIN_POWER_STAGE_PHASE_ENABLE_C != PIN_POWER_STAGE_FAULT),
              "Phase-enable pins must not share PIN_POWER_STAGE_FAULT.");
static_assert(!POWER_STAGE_PHASE_ENABLES ||
              (PIN_POWER_STAGE_PHASE_ENABLE_A != PIN_POWER_STAGE_PHASE_ENABLE_B &&
               PIN_POWER_STAGE_PHASE_ENABLE_A != PIN_POWER_STAGE_PHASE_ENABLE_C &&
               PIN_POWER_STAGE_PHASE_ENABLE_B != PIN_POWER_STAGE_PHASE_ENABLE_C),
              "Phase-enable pins must be distinct.");
static_assert(!POWER_STAGE_SLEEP_ENABLE || !POWER_STAGE_SHARED_ENABLE || PIN_POWER_STAGE_SLEEP != PIN_POWER_STAGE_ENABLE,
              "PIN_POWER_STAGE_SLEEP must not share PIN_POWER_STAGE_ENABLE.");
static_assert(!POWER_STAGE_SLEEP_ENABLE || !POWER_STAGE_FAULT_ENABLE || PIN_POWER_STAGE_SLEEP != PIN_POWER_STAGE_FAULT,
              "PIN_POWER_STAGE_SLEEP must not share PIN_POWER_STAGE_FAULT.");
static_assert(!POWER_STAGE_SLEEP_ENABLE || !POWER_STAGE_PHASE_ENABLES ||
              (PIN_POWER_STAGE_SLEEP != PIN_POWER_STAGE_PHASE_ENABLE_A &&
               PIN_POWER_STAGE_SLEEP != PIN_POWER_STAGE_PHASE_ENABLE_B &&
               PIN_POWER_STAGE_SLEEP != PIN_POWER_STAGE_PHASE_ENABLE_C),
              "PIN_POWER_STAGE_SLEEP must not share a phase-enable pin.");
static_assert(!POWER_STAGE_RESET_ENABLE || !POWER_STAGE_SHARED_ENABLE || PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_ENABLE,
              "PIN_POWER_STAGE_RESET must not share PIN_POWER_STAGE_ENABLE.");
static_assert(!POWER_STAGE_RESET_ENABLE || !POWER_STAGE_FAULT_ENABLE || PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_FAULT,
              "PIN_POWER_STAGE_RESET must not share PIN_POWER_STAGE_FAULT.");
static_assert(!POWER_STAGE_RESET_ENABLE || !POWER_STAGE_PHASE_ENABLES ||
              (PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_PHASE_ENABLE_A &&
               PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_PHASE_ENABLE_B &&
               PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_PHASE_ENABLE_C),
              "PIN_POWER_STAGE_RESET must not share a phase-enable pin.");
static_assert(!POWER_STAGE_RESET_ENABLE || !POWER_STAGE_SLEEP_ENABLE || PIN_POWER_STAGE_RESET != PIN_POWER_STAGE_SLEEP,
              "PIN_POWER_STAGE_RESET must not share PIN_POWER_STAGE_SLEEP.");

#undef TT_POWER_STAGE_ASSERT_RESOURCE
#endif

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

#if CLOSED_LOOP_SPEED_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_SPEED_SENSOR_B);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_PWM_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_I2C0_SDA);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_I2C0_SCL);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_MAIN_CLK);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_MAIN_DT);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_RELAY_STANDBY);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_PWM_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_PWM_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_PWM_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_PWM_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_I2C0_SDA);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_I2C0_SCL);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_MAIN_CLK);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_MAIN_DT);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_MAIN_SW);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_RELAY_STANDBY);
#if PITCH_CONTROL_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_PITCH_CLK);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_PITCH_DT);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_ENC_PITCH_SW);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_PITCH_CLK);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_PITCH_DT);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_ENC_PITCH_SW);
#endif
#if SPEED_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_BTN_SPEED);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_BTN_SPEED);
#endif
#if START_STOP_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_BTN_START_STOP);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_BTN_START_STOP);
#endif
#if STANDBY_BUTTON_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_BTN_STANDBY);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_BTN_STANDBY);
#endif
#if ENABLE_MUTE_RELAYS
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_MUTE_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_MUTE_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_MUTE_PHASE_C);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_MUTE_PHASE_A);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_MUTE_PHASE_B);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_MUTE_PHASE_C);
#if ENABLE_4_CHANNEL_SUPPORT
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_MUTE_PHASE_D);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_MUTE_PHASE_D);
#endif
#endif
#if AMP_MONITOR_ENABLE
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_AMP_TEMP);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_A, PIN_AMP_THERM_OK);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_AMP_TEMP);
TT_PIN_ASSERT_DISTINCT(PIN_SPEED_SENSOR_B, PIN_AMP_THERM_OK);
#endif
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
