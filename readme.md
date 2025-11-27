# TT Control

TT Control is an advanced turntable motor controller designed to provide precise, multi-phase sine wave generation for controlling synchronous AC and BLDC motors. Built on the Raspberry Pi Pico RP2053, using the Arduino-Pico core, it leverages the Pico's dual-core architecture for efficient operation, separating UI and control logic from high-precision waveform generation.

It features extensive configurability via a hierarchical menu system, hardware configurability via compile-time flags, multi-speed support, configurable settings presets, advanced amplitude and phase control, configurable hardware controls, support for a pitch control, digital filtering and sine wave interpolation, non-volatile settings storage and so much more.

---
## 1. Hardware & Environment Setup

| Component | Specification | Default Value / Assignment | Notes |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | Raspberry Pi Pico (RP2040) | | Dual Core architecture is mandatory. |
| **Board** | Pimoroni Pico+2 | | 16MB Flash split: **8MB Sketch / 8MB LittleFS** for settings. |
| **Development** | Arduino IDE / `earlephilhower/arduino-pico` | | Use `arduino-cli` best practices. |
| **OLED Display** | SSD1306 (128x64 I2C) | **I2C0 @ 0x3C** | |

---

### 1.1. Code Requirements

All code conforms to C coding best practices, ensuring optimal utilisation of all library features and functions.

All code is extensively commented. The menu system is data-driven and arranged as outlined.

---

### 1.2. Datasheets and Resources

- **Microcontroller:** Raspberry Pi Pico, [RP2350](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- **Board:** [Pimoroni Pico+2](https://shop.pimoroni.com/products/pimoroni-pico-plus-2)
- **Development environment:** Arduino IDE with [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico)
- **Tools:** arduino-cli, PioASM
- **Libraries:**
  - **OLED:** [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
  - **Graphics:** [Adafruit-GFX](https://github.com/adafruit/Adafruit-GFX-Library)
  - **Display I/O:** [Adafruit-BusIO](https://github.com/adafruit/Adafruit_BusIO)

---

### 1.3. Pin Assignments

| Pin | Function | Notes |
| :--- | :--- | :--- |
| 0 | PWM phase A | |
| 1 | PWM Phase B | |
| 2 | PWM Phase C | |
| 3 | PWM Phase D | |
| 4 | I2C0 SDA (OLED) | |
| 5 | I2C0 SCL (OLED) | |
| 10 | Primary Encoder CLK | |
| 11 | Primary Encoder DT | |
| 12 | Primary Encoder SW | |
| 13 | Secondary (Pitch) Encoder CLK | **Optional, controlled by compile flag.** |
| 14 | Secondary (Pitch) Encoder DT | **Optional, controlled by compile flag.** |
| 15 | Secondary (Pitch) Encoder SW | **Optional, controlled by compile flag.** |
| 16 | Standby Relay Control | Active High Default (Configurable). |
| 17 | Phase A Muting Control | Active High Default (Configurable). Acts as DPDT Relay 1 if `ENABLE_DPDT_RELAYS` is true. |
| 18 | Phase B Muting Control | Active High Default (Configurable). Acts as DPDT Relay 2 if `ENABLE_DPDT_RELAYS` is true. |
| 19 | Phase C Muting Control | Active High Default (Configurable). Unused if `ENABLE_DPDT_RELAYS` is true. |
| 20 | Phase D Muting Control | Active High Default (Configurable). Unused if `ENABLE_DPDT_RELAYS` is true. |
| 21 | Standby Button | **Optional, controlled by compile flag.** |
| 22 | Speed Button | **Optional, controlled by compile flag.** |
| 23 | Start / Stop Button | **Optional, controlled by compile flag.** |

---

### 1.4. Default phase offset angles

Default angles depend on phase mode and can be adjusted.

| Assigned Output Phase | Initial Default Angle |
| :--- | :--- |
| Phase 1 | 0.0° (Fixed Reference) |
| Phase 2 | +90.0° |
| Phase 3 | +120.0° |
| Phase 4 | +240.0°  |

---

## 2. Features in detail

### 2.1. Sine Wave Generation
- **High-Precision DMA-based Generation:** Uses the RP2040's Direct Memory Access (DMA) and hardware PWM slices for jitter-free, CPU-independent waveform generation.
- **Waveform Lookup Table:** Pre-computed, configurable sample lookup table with configurable lookup table size (1024, 2048, 4096, 8192, 16384).
- **Interpolation:** Linear interpolation to smooth output with smaller sample tables.
- **Frequency Range:** 10-3000 Hz in 0.1 Hz steps.
- **Multi-Speed Support:** Supports 33⅓ (default 50Hz), 45 (default 67.5Hz), and 78 (default 113.5Hz) RPM.
- **78 RPM Toggle:** 78RPM can be enabled or disabled in the menu.
- **Frequency Constraints:** Configurable minimum and maximum frequency constraint, per speed.

---

### 2.2. Multi-Phase Operation
- **Single Phase Mode:** 0 degrees.
- **2-Phase Mode:** 0° and 90°.
- **3-Phase Mode:** 0°, 120°, 240°.
- **4-Phase Mode:** 0°, 90°, 0°, 90°.
- **Configurable Phase Mode:** Phase mode configurable in menu (global setting), three-phase default.
- **Phase Offset Adjustment:** 0.1° resolution, -360 to +360 degrees.
- **Independent Channel Offsets:** Independent phase offset adjustment (per-channel).
- **Fixed Reference:** Phase A is always fixed at 0 degrees.
- **Per-Speed Offsets:** Independent phase offset settings per speed.

---

### 2.3. Motor Control Features
- **Max Amplitude Limit:** Configurable maximum amplitude to limit the output voltage to safe margins (global).
- **Soft Start:** Configurable soft start with 3 duration options (0.5s, 1s, 1.5s) (per speed).
- **Frequency Dependent Amplitude (FDA):**
  - Global setting that scales the output amplitude based on the current frequency.
  - Maintains a constant V/f ratio (or similar) to equalize torque across speeds and during frequency ramps (e.g., startup).
  - Useful for Permanent Magnet Synchronous (PMSM) or BLDC motors.
  - Configurable as a percentage of maximum amplitude at zero frequency (0-100%).
  - Set to **0** to disable (default).
- **Soft Start Curves:** Configurable three soft start acceleration curves (linear, logarithmic, exponential) (global).
- **Reduced Amplitude Mode:** Configurable reduced amplitude mode (50-100% in 1% steps) to compromise between torque and noise (per speed).
- **Reduced Amplitude Delay:** Configurable delay before amplitude reduction (1S, 2S, 3S, 5S, 10S, 15S, 20S) allows the motor to get up to speed before the torque is reduced (per speed).
- **Startup Kick:** Configurable accelerated startup (1x, 2x, 3x, 4x frequency). Starts the motor at a higher speed to overcome drag in the drive system, reduces gradually to desired nominal frequency after configured duration. (Per speed).
- **Kick Ramp Duration:** Configurable accelerated startup ramp duration (1S, 2S, 3S, 4S, 5S, 10S, 15S) (per speed).
- **Smooth Speed Switching:** Smooth speed switching with optional ramping and configurable ramp duration, gently increases or decreases the speed when the speed is toggled and the motor is running.
- **Switch Ramp Duration:** Configurable switch ramp duration (1S, 2S, 3S, 4S, 5S).
- **S-Curve (Sigmoidal) Soft Start:**
  - Configurable option to use an S-Curve profile for soft starts instead of a linear ramp.
  - **Benefits:** Provides a much smoother acceleration by easing in and easing out of the ramp. This significantly reduces mechanical stress on the belt and pulley system and prevents belt slippage during startup.
  - **Configuration:** Selectable via the "Motor" menu or CLI (`set ramp 1`).
- **Pitch Control:** Pitch control feature with a second encoder:
  - Optional via a compile time flag in config.h, disabled by default. If enabled at compile time, a setting shows in the menu to disable pitch control.
  - Menu setting to optionally reset the pitch control when the motor is stopped. Otherwise, it resets on standby.
  - Pressing the encoder toggles between ±10%, ±20%, ±30%, ±40%, or ±50% with brief message displayed for a few seconds to show current setting. The default is 10%.
  - Rotating the encoder adjusts the current frequency within this limit in 0.1Hz steps, with the same acceleration curve set for the primary encoder.
  - Pitch change percentage shown on speed status, for example 33.3RPM +5%.
    - Holding the pitch encoder for 2 seconds or more resets pitch to 0.
- **Motor Braking:** Motor braking feature to actively slow the platter when stopping:
  - Configurable Mode: Off, Pulse, or Ramp.
  - Configurable Duration: 1.0s to 10.0s.
  - Pulse Mode: Applies reverse power in pulses. Pulse gap is configurable from 0.2s to 2.0s.
  - Ramp Mode: Applies reverse power, ramping the frequency down from a configurable start frequency to a stop frequency over the specified duration.

---

### 2.4. Digital Filtering

The firmware includes two types of digital low-pass filters: a 1st-order Infinite Impulse Response (IIR) filter and an 8-tap Finite Impulse Response (FIR) filter. The selected digital filter is applied to the generated sine wave data before it is stored in the Look-Up Table (LUT) and subsequently sent to the DAC.

- **Optional Digital Filtering:** Configurable optional digital filtering to further smooth the sine wave output.
- **Filter Configuration:** Configurable filter type and settings.
- **IIR Low-Pass Filter:**
  - Type: 1st-order IIR.
  - Algorithm: Implemented as a simple exponential moving average.
  - Configurable Parameter: iir_alpha:
    - Range: `0.01` to `0.99` (0.01 steps).
    - Description: This parameter (`alpha`) controls the smoothing factor of the IIR filter.
    - A lower `alpha` value results in a stronger low-pass effect (more smoothing, slower response).
    - A higher `alpha` value results in a weaker low-pass effect (less smoothing, faster response).
- **FIR Low-Pass Filter:**
  - Type: 8-tap FIR.
  - Algorithm: Direct convolution with a set of coefficients.
  - Configurable Parameter: `fir_profile_index`, which selects from a pre-defined sets of coefficients:
    - Gentle: Provides minimal smoothing.
    - Medium: Offers a balanced amount of smoothing.
    - Aggressive: Applies significant smoothing, potentially leading to a softer, more rounded waveform.
- **Per-Speed Configuration:** Filters configured individually per speed.
- **Default State:** Filters default to 'off'.

The primary purpose of these digital low-pass filters is to smooth the generated sine wave output.

The alpha parameter directly influences the filter's cutoff frequency and transient response. Lower alpha values will make the waveform smoother but might introduce a slight lag.

The different FIR profiles provide distinct frequency responses. "Aggressive" profiles will yield a very smooth waveform but might also slightly attenuate desired signal components. The filters are provided as a way to eke out that last bit of performance from a motor, but are entirely optional and their use depends on each individual motor, turntable and desired performance.

---

### 2.5. User Interface
- **Display Support:** Support for 128x64 OLED display.
- **Menu System:** Comprehensive, logically organised hierarchical menu system to adjust all possible configuration settings via the user interface.
- **Full Configurability:** Every possible configurable option available as a menu setting.
- **Real-Time Updates:** All settings update in real time when adjustments are being made, to observe their effect on the motor. Settings are only saved to flash on selecting the 'save' option from the menu.
- **Single Encoder Control:** Control possible by single rotary encoder with pushbutton.
- **Primary Encoder Interaction:**
  - **Short Press:** Start/Stop Motor (or Wake from Standby).
  - **Double Press:** Enter Main Menu.
  - **Hold (Dashboard):** Enter Standby.
  - **Hold (Menu):** Save & Exit Menu.
  - **Rotate:** Change Speed (33/45/78).
  - **Press + Rotate:** Cycle Status Views (Standard -> Stats -> Dim).
- **Status Modes:**
  - **Standard:** Large RPM display + Pitch Bar.
  - **Stats:** Session Runtime + Total Runtime.
  - **Dim:** Minimal display with low brightness (auto-dim supported).
- **Auto Features:**
  - **Auto Standby:** Configurable delay (10m-30m) to enter standby if motor is STOPPED.
  - **Auto Dim:** Configurable delay (1m-10m) to switch to Dim mode if motor is RUNNING.
- **Encoder Acceleration:** Rotary encoder has configurable acceleration to quickly scroll through long menus or make adjustments to values.
- **Button Detection:** Button press detection (short, long, very long, double).
- **Optional Buttons:** Support for the following additional separate buttons, with compile time flags and pin assignments in config.h:
  - **Standby Toggle:** If this button is enabled, pressing it toggles standby. Works globally.
  - **Change Speed:** When this button is enabled, pressing it cycles the speeds in sequence. Works globally (updates menu if open).
  - **Start / Stop Motor:** If this button is enabled, it toggles motor start / stop from anywhere in the system (apart from in standby), including within the menu.
- **Runtime Tracking:** Optional Runtime tracking to track session and total runtime. Useful for monitoring stylus usage or turntable maintenance schedules.
- **Status Cycling:** Status display cycling (speed, frequency, phase mode, phase angles, runtime) with user settings to toggle the available statuses.
- **Scrolling Messages:**
  - **Welcome:** Scrolls "Welcome to TT Control" on boot.
  - **Goodbye:** Scrolls "Goodbye..." when entering standby, before the display turns off (if configured).

---

### 2.6. Serial Monitor Support
- **Optional Enable:** Optional, enabled or disabled by compile-time configuration flags.
- **Interactive CLI:** Full command-line interface for control and configuration.
- **Commands:** Type `help` or `list` to see available commands. Supports `start`, `stop`, `speed`, `set`, `get`, and more.
- **Keyboard Control:** Keyboard control over encoder and key functions (j/l/k/m/s/t/i/f).
- **Status Reporting:** Comprehensive status printing and error reporting to serial monitor, set by compile time flag.

---

### 2.7. Settings Management
- **Configurable Presets:** Multiple configurable presets (5 by default, the number defined in configuration flag).
- **Preset Management:** Presets can be reverted to defaults, loaded, renamed and duplicated.
- **Preset Naming:** Presets can have names up to 16 characters, a-z 0-9.
- **Non-Volatile Storage:** Non-volatile settings storage in Pico flash FS.
- **Flash Capacity:** The Pimoroni Pico+2 has 16MB of flash, which can be split at compile time. A standard Pico has more than enough flash for this application also.
- **Wear Levelling:** Flash wear levelling.
- **Factory Reset:** Factory reset with two-level confirmation.
- **Critical Confirmations:** Confirmations for critical settings, such as phase mode and max amplitude changes.
- **Settings File Schema:**
  - **Storage Location:** Stored on LittleFS.
  - **Schema Versioning:** Each file includes `"schema_version"` to ensure forward compatibility.
  - **Migration Strategy:** On schema mismatch or corruption, data is compared, old values are matched, new values are loaded to preserve user settings on firmware upgrade.

---

### 2.8. Power Management
- Standby mode to enter low power state
  - **Compile-time Flag**: `ENABLE_STANDBY` (default `true`). If disabled, the system boots directly to STOPPED state and all standby features are hidden.
- Relay control for secondary power to drive load driving circuitry
- Output muting pins assigned for each channel to control relays or drive the mute lines of amplifiers
  - **Compile-time Flag**: `ENABLE_MUTE_RELAYS` (default `true`). If disabled, relay logic is skipped.
  - **DPDT Support**: `ENABLE_DPDT_RELAYS` (default `false`) allows using 2 DPDT relays instead of 4 SPST relays.
- Staggered switching of muting pins with configurable switching delays
- Setting to switch muting relays with standby, or on motor start / stop
- Configurable Power-On Relay Delay (0-10s) to prevent audible pop or transient during initialisation.
- Configurable auto standby (0-60 minutes, 1 minute steps). 0 = Off.
- Configurable auto dim (0-60 minutes, 1 minute steps). 0 = Off.
- Configurable auto boot, boot directly into operation and bypass standby
- Configurable motor auto start, starts the motor immediately after waking from standby
- Configurable display sleep modes (Off, 10s, 20s, 30s, 1m, 5m, 10m).
- option to disable display in standby, or display a standby message
- screensaver mode, changes position of standby message on display if enabled to prevent burn in.
- Low-power idle when in standby

---

### 2.9. Multi-Core & DMA Architecture
- **Core 0:** Handles UI, menu system, encoder input, and high-level motor control logic.
- **Core 1:** Dedicated to waveform buffer management.
- **DMA & Hardware PWM:** The actual waveform generation is offloaded to the RP2040's DMA controller and PWM hardware. This ensures:
  - Zero CPU jitter in the output signal.
  - Extremely low CPU usage (Core 1 only wakes to refill buffers).
  - Robust operation even during heavy UI activity.
- **Inter-Core Communication:** Optimized inter-core communication.
- **PIO Usage:** Maximize use of Pico state machines.
- **FIFO Usage:** Optimal use of FIFO.
- **Non-Blocking Design:** Non-blocking code, including delays, and timers and interrupt routines.

---

### 2.10. Error Handling
- **Initialization Checks:** Display initialization check.
- **Range Validation:** Settings range validation.
- **Error Reporting:** Error code system to display errors on screen (configurable in menu) and via serial output (configurable in flag).
- **Error Display Duration:** Error codes should be displayed on OLED or LCD for a configurable duration (default 10s, configurable in menu).
- **Error Clearing:** Pressing encoder clears the error.
- **Safety Shutdown:** Error Mode switches off all relays when an error message is displayed.

---

## 3. User Interface

---

### 3.1. Basic Operation

| Action | Trigger | Condition |
| :--- | :--- | :--- |
| **Wake From Standby** | Short press encoder | N/A |
| **Start/Stop Motor** | Short press encoder | N/A |
| **Change Speed** | Turn encoder | N/A |
| **Cycle Status View** | Press + Turn encoder | Standard -> Stats -> Dim |
| **Enter Menu** | Double press encoder | N/A |
| **Enter Standby** | Hold encoder (Dashboard) | N/A |
| **Save & Exit Menu** | Hold encoder (Menu) | N/A |

---

### 3.2. Serial Commands
Connect at 115200 baud. The CLI supports a registry of settings that can be accessed via `set` and `get`.

| Command | Description |
| :--- | :--- |
| `start` | Start the motor |
| `stop` | Stop the motor |
| `speed <0-2>` | Set speed (0=33, 1=45, 2=78) |
| `s` | Cycle speed |
| `t` | Toggle standby |
| `p` | Reset pitch |
| `status` / `i` | Show current status |
| `list` | **List all available settings and values** |
| `set <key> <val>` | Set a parameter value |
| `get <key>` | Get a parameter value |
| `error dump` | Dump error log |
| `error clear` | Clear error log |
| `f` | Factory Reset |



#### Available Settings Keys
Use these keys with `set` and `get`. Speed-specific settings apply to the **currently selected speed**.

| Key | Description | Type |
| :--- | :--- | :--- |
| **Global** | | |
| `brightness` | OLED Brightness (0-255) | Int |
| `ramp` | Soft Start Ramp Type (0=Linear, 1=S-Curve) | Int |
| `pitch_step` | Pitch Step Size (e.g. 0.1) | Float |
| `rev_enc` | Reverse Encoder (0/1) | Bool |
| **Current Speed** | | |
| `freq` | Frequency (Hz) | Float |
| `phase1`..`phase4` | Phase Offsets (Degrees) | Float |
| `soft_start` | Soft Start Duration (s) | Float |
| `kick` | Startup Kick Multiplier (e.g. 2) | Int |
| `kick_dur` | Startup Kick Duration (s) | Int |
| **Live** | | |
| `pitch` | Current Pitch (%) | Float |

---

## 3.3. Screens and Menus
The menu structure is designed for a data-driven implementation.

### Main Menu
- **Edit Speed: [33/45/78]** (Toggle speed context for submenus)
- **Speed Tuning:** Frequency, Limits, Filters (Per-Speed).
- **Phase:** Phase Mode (Global), Offsets (Per-Speed).
- **Motor:** Amps/Kick (Per-Speed), Braking (Global).
- **Power:** Relays, Auto Standby/Boot (Global).
- **Display:** Sleep, Dim, Saver, Errors (Global).
- **System:** Pitch Reset, 78RPM, Logs (Global).
- **Presets:** Load, Save, Rename, and Clear presets.
- **Save & Exit:** Saves all changes to flash and returns to dashboard.
- **Cancel:** Discards changes and reloads from flash.

### Speed Tuning (Per-Speed)
- **Frequency:** Nominal frequency (Hz).
- **Min/Max Freq:** Limits for pitch control.
- **Filt Type:** Digital Filter Type (None, IIR, FIR).
- **IIR Alpha:** IIR Filter smoothing factor.
- **FIR Prof:** FIR Filter Profile (Gentle, Medium, Aggressive).

### Phase Control
- **Mode (Glb):** 1, 2, 3, or 4 phase operation.
- **Ph 2-4 Offs:** Phase offsets for multi-phase motors (Per-Speed).

### Motor Control
- **Soft Start**: Adjustable duration (0-10s) to ramp up amplitude gently.
- **FDA (Frequency Dependent Amplitude)**: Scales output amplitude based on frequency to maintain constant torque across speeds and during ramps. Useful for PM/BLDC motors. Set to **0** to disable (default).
- **Reduced Amplitude**: Automatically lower voltage after spin-up to reduce motor noise and heat (50-100%).
- **Startup Kick**: Momentary frequency boost (up to 4x) to overcome static friction.
- **Kick Mult:** Startup kick multiplier (Per-Speed).
- **Kick Dur:** Startup kick duration (s) (Per-Speed).
- **Kick Ramp:** Ramp down duration from kick (s) (Per-Speed).
- **Max Amp %:** Global maximum amplitude limit.
- **SS Curve:** Soft Start Curve (Linear, Log, Exp).
- **Smooth Sw:** Enable smooth frequency ramping between speeds.
- **Sw Ramp:** Duration (s) for speed switch ramp.
- **Brake Mode:** Off, Pulse, or Ramp.
- **Brake Dur:** Braking duration (s).
- **Brk Pulse:** Gap between pulses in Pulse mode (s).
- **Brk Start/Stop F:** Frequency ramp range for Ramp mode.
- **Ramp Type:** Soft Start Profile (0=Linear, 1=S-Curve).
- **Auto Start:** Start motor immediately after boot/wake.

### Power Control
- **Rly: ActHi:** Toggle relay logic (Active High/Low).
- **Rly: Stby:** Mute relays when entering Standby.
- **Rly: S/S:** Unmute relays on Start, Mute on Stop.
- **Rly: Delay:** Power-on delay for relays (s).
- **Auto Stby:** Auto-standby delay (min).
- **Auto Boot:** Boot directly to operation (bypass standby).

### Display
- **Brightness:** OLED Brightness/Contrast (0-255).
- **Sleep Dly:** Display sleep delay (off/10s/etc).
- **Scrn Saver:** Enable screensaver in standby.
- **Saver Mode:** Select screensaver animation:
    - **Bounce:** Bouncing text (Default).
    - **Matrix:** Digital rain effect.
    - **Lissajous:** Rotating 3D curves.
- **Auto Dim:** Auto-dim delay (min).
- **Show Runtime:** Toggle runtime display on dashboard.
- **Err Display:** Toggle on-screen error messages.
- **Err Dur:** Duration of error messages (s).

### System
- **Ver:** Firmware Version.
- **Rev Encoder:** Reverse the direction of the main encoder.
- **Pitch Step:** Step size for pitch control (0.01% - 1.0%).
- **Pitch Reset:** Reset pitch to 0% on motor stop.
- **Enable 78:** Toggle availability of 78 RPM mode.
- **Error Log:** View and clear system error logs.
- **Boot Speed:** Select default speed on boot (0=33, 1=45, 2=78, 3=Last Used).
- **Reset Runtime:** Reset the total runtime counter (with confirmation).
- **Fact Reset:** Factory Reset.
- **Welcome Msg:** Configurable welcome message on boot.
- **Goodbye Msg:** Configurable goodbye message on shutdown.

### Presets & Renaming
- **Load:** Load settings from a slot.
- **Save:** Save current settings to a slot.
- **Rename:** Enter text editor to rename slot.
  - **Rotate:** Cycle characters (A-Z, 0-9, Space).
  - **Click:** Move cursor / Select character.
  - **Apply Name:** Save the new name.
- **Clear:** Reset slot to defaults.

---

## 4. Compile-Time Flags

| Flag Category | Flag Name | Purpose | Default / Pin Assignment / Options | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **System Info** | `BUILD_DATE` | Stores the timestamp/date of the firmware compilation. | If possible, automatically generated by build script. | |
| **Flash** | `LITTLEFS_FS_SIZE` | Defines the size of the LittleFS partition for settings storage. | `8MB` (Min `1MB` recommended) | Critical for non-volatile storage setup. |
| **Core Hardware**| `OLED_I2C_ADDRESS` | Defines the I2C address for the SSD1306 display. | `0x3C` | Essential for display communication. |
| **Waveform** | `LUT_MAX_SIZE` | Sets the maximum size of the sine wave Look-Up Table (LUT). | `16384` | Affects memory usage and processing load. |
| **Presets** | `MAX_PRESET_SLOTS` | Defines the maximum number of user-configurable presets. | `5` | Used to size the data structure for presets. |
| **Serial/Debug** | `SERIAL_MONITOR_ENABLE` | Enables/Disables all Serial Monitor functionality and setup. | `true` or `false` | Global toggle for serial output. |
| **Serial/Debug** | `DUPLICATE_DISPLAY_TO_SERIAL` | Duplicates all display output to the serial monitor. | `true` or `false` | Requires `SERIAL_MONITOR_ENABLE` to be true. |
| **Features** | `ENABLE_STANDBY` | Enable/Disable Standby Mode | `true` | Controls standby functionality. |
| **Features** | `ENABLE_MUTE_RELAYS` | Enable/Disable Mute Relays | `true` | Controls relay logic. |
| **Features** | `ENABLE_DPDT_RELAYS` | Use 2x DPDT instead of 4x SPST | `false` | Changes relay switching logic. |
| **Optional Pins**| `PITCH_CONTROL_ENABLE` | Enables the secondary (Pitch) encoder functionality and logic. | `false` (Disabled) | **Required** flag for the optional pitch feature (3.3). |
| **Optional Pins**| `PITCH_ENCODER_CLK_PIN` | Assigns the pin for the Pitch Encoder Clock. | `13` | Only compiled if `PITCH_CONTROL_ENABLE` is true. |
| **Optional Pins**| `PITCH_ENCODER_DT_PIN` | Assigns the pin for the Pitch Encoder Data. | `14` | Only compiled if `PITCH_CONTROL_ENABLE` is true. |
| **Optional Pins**| `PITCH_ENCODER_SW_PIN` | Assigns the pin for the Pitch Encoder Switch. | `15` | Only compiled if `PITCH_CONTROL_ENABLE` is true. |
| **Optional Pins**| `STANDBY_BUTTON_ENABLE` | Enables the external Standby button. | `false` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `STANDBY_BUTTON_PIN` | Assigns the pin for the Standby Button. | `21` | Only compiled if `STANDBY_BUTTON_ENABLE` is true. |
| **Optional Pins**| `SPEED_BUTTON_ENABLE` | Enables the external Speed change button. | `false` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `SPEED_BUTTON_PIN` | Assigns the pin for the Speed Button. | `22` | Only compiled if `SPEED_BUTTON_ENABLE` is true. |
| **Optional Pins**| `START_STOP_BUTTON_ENABLE` | Enables the external Start/Stop motor button. | `false` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `START_STOP_BUTTON_PIN` | Assigns the pin for the Start/Stop Button. | `23` | Only compiled if `START_STOP_BUTTON_ENABLE` is true. |
| **Display Msg** | `STANDBY_MESSAGE` | Sets the message displayed while the system is in standby, if enabled in the menu. | `message` | Controls message display logic. |
| **Storage** | `SETTINGS_SCHEMA_VERSION` | Tag for settings file schema. | `1` | Use for compatibility checks when firmware is updated. |
