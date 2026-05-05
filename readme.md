# TT Control

TT Control is an advanced turntable motor controller designed to provide precise, multi-phase sine wave generation for controlling synchronous AC and BLDC motors. Built on the Raspberry Pi Pico RP2350, using the Arduino-Pico core, it leverages the Pico's dual-core architecture for efficient operation, separating UI and control logic from high-precision waveform generation.

It features extensive configurability via a hierarchical menu system, hardware configurability via compile-time flags, multi-speed support, configurable settings presets, advanced amplitude and phase control, configurable hardware controls, support for a pitch control, digital filtering and sine wave interpolation, non-volatile settings storage and so much more.

---
## 1. Hardware & Environment Setup

| Component | Specification | Default Value / Assignment | Notes |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | RP2350 | | Dual Core architecture is mandatory. |
| **Board** | Pimoroni Pico+2, Pimoroni PicoPlus2W, or Raspberry Pi Pico 2 W. | | 16MB Flash split on Pico+2/PicoPlus2W: **8MB Sketch / 8MB LittleFS** for settings. Wi-Fi builds use the Arduino-Pico W board targets. |
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
  - **JSON:** ArduinoJson
  - **Wi-Fi builds:** Arduino-Pico bundled `WiFi`, `WebServer`, and `DNSServer`

### 1.3. Build Targets

Non-Wi-Fi Pimoroni Pico+2 build:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 .
```

Pimoroni PicoPlus2W build with 8MB LittleFS:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2w:flash=16777216_8388608 .
```

Raspberry Pi Pico 2 W build with 1MB LittleFS:

```sh
arduino-cli compile --fqbn rp2040:rp2040:rpipico2w:flash=4194304_1048576 .
```

---

### 1.4. Pin Assignments

| Pin | Function | Notes |
| :--- | :--- | :--- |
| 0 | PWM phase A | |
| 1 | PWM Phase B | |
| 2 | PWM Phase C | |
| 3 | PWM Phase D | Can be disabled only in custom 3-channel builds. |
| 4 | I2C0 SDA (OLED) | |
| 5 | I2C0 SCL (OLED) | |
| 10 | Primary Encoder CLK | |
| 11 | Primary Encoder DT | |
| 12 | Primary Encoder SW | |
| 13 | Secondary (Pitch) Encoder CLK | **Optional, controlled by compile flag.** |
| 14 | Secondary (Pitch) Encoder DT | **Optional, controlled by compile flag.** |
| 15 | Secondary (Pitch) Encoder SW | **Optional, controlled by compile flag.** |
| 16 | Standby Relay Control | Active High Default (Configurable). |
| 17 | Phase A Muting Control | Active High Default (Configurable). Acts as DPDT Relay 1 if `ENABLE_DPDT_RELAYS` is `1`. |
| 18 | Phase B Muting Control | Active High Default (Configurable). Acts as DPDT Relay 2 if `ENABLE_DPDT_RELAYS` is `1`. |
| 19 | Phase C Muting Control | Active High Default (Configurable). Unused if `ENABLE_DPDT_RELAYS` is `1`. |
| 20 | Phase D Muting Control | Can be disabled only in custom 3-channel builds; unused if `ENABLE_DPDT_RELAYS` is `1`. |
| 21 | Standby Button | **Optional, controlled by compile flag.** |
| 22 | Speed Button | **Optional, controlled by compile flag.** |
| 9 | Start / Stop Button | **Optional, controlled by compile flag.** Uses a Pico 2 W header pin. |
| 26 | Amplifier Temperature | ADC input for a TMP36-style heatsink sensor, enabled by `AMP_MONITOR_ENABLE`. |
| 27 | Amplifier Thermal OK | Digital thermal cutout/status input, enabled by `AMP_MONITOR_ENABLE`; HIGH means healthy, LOW trips shutdown. |

---

### 1.5. Default phase offset angles

Default angles depend on phase mode and can be adjusted.

| Assigned Output Phase | Initial Default Angle |
| :--- | :--- |
| Phase 1 | 0.0° (Fixed Reference) |
| Phase 2 | +120.0° |
| Phase 3 | +240.0° |
| Phase 4 | +270.0° |

---

## 2. Features in detail

### 2.1. Sine Wave Generation
- **High-Precision DMA-based Generation:** Uses the RP2350's Direct Memory Access (DMA) and hardware PWM slices for jitter-free, CPU-independent waveform generation.
- **Waveform Lookup Table:** Pre-computed, configurable sample lookup table with configurable lookup table size (1024, 2048, 4096, 8192, 16384).
- **Interpolation:** Linear interpolation to smooth output with smaller sample tables.
- **Frequency Range:** 10-1500 Hz in 0.1 Hz steps.
- **Multi-Speed Support:** Supports 33⅓, 45, and 78 RPM. Factory defaults target the primary 12-pole motor: 25.07Hz, 33.85Hz, and 58.66Hz.
- **78 RPM Toggle:** 78RPM can be enabled or disabled in the menu.
- **Frequency Constraints:** Configurable minimum and maximum frequency constraint, per speed.

---

### 2.2. Multi-Phase Operation
- **Single Phase Mode:** 0 degrees.
- **2-Phase Mode:** 0° and 90°.
- **3-Phase Mode:** 0°, 120°, 240°.
- **4-Phase Mode:** Disabled by default. Set `ENABLE_4_CHANNEL_SUPPORT` to `1` in `config.h` to enable optional four-channel/Premotec bridge presets.
- **Configurable Phase Mode:** Phase mode configurable in menu (global setting), three-phase default. Standard firmware builds expose 1-3 phase modes; four-channel support is an explicit opt-in.
- **Phase Offset Adjustment:** 0.1° resolution, -360 to +360 degrees.
- **Independent Channel Offsets:** Independent phase offset adjustment (per-channel).
- **Fixed Reference:** Phase A is always fixed at 0 degrees.
- **Per-Speed Offsets:** Independent phase offset settings per speed.
- **Resonance Sweep:** gently sweeps the symmetrical phase separation angle back and forth while the motor continues spinning at its target frequency.
  - Allows you to physically find the optimal phase offset that best nullifies mechanical resonance and vibration.
  - Pressing the encoder when the motor vibration is at its lowest instantly locks and saves the phase parameters.

---

### 2.3. Motor Control Features
- **Max Amplitude Limit:** Configurable maximum amplitude to limit the output voltage to safe margins (global).
- **Soft Start:** Configurable soft start with continuous duration from 0.0s to 10.0s (per speed).
- **Soft Start:** Configurable soft start with continuous duration from 0.0s to 10.0s (per speed). Choose between Linear, S-Curve, Logarithmic, or Exponential ramping profiles.
- **Frequency Dependent Amplitude (FDA):**
  - Global setting that scales the output amplitude based on the current frequency.
  - Maintains a constant V/f ratio (or similar) to equalize torque across speeds and during frequency ramps (e.g., startup).
  - Useful for Permanent Magnet Synchronous (PMSM) or BLDC motors.
  - Configurable as a percentage of maximum amplitude at zero frequency (0-100%).
  - Set to **0** to disable (default).
- **Voltage/Frequency (V/f) Curves:** an advanced 3-point interpolation curve. Define a voltage boost percentage at a specific low frequency and a mid frequency to eliminate low-speed cogging without overdriving the motor mechanically. A master V/f blend% parameter controls the mix intensity linearly against the fully calculated target amplitude.
- **Soft Start Curves:** Configurable three soft start acceleration curves (linear, logarithmic, exponential) (global).
- **Reduced Amplitude Mode:** Configurable reduced amplitude mode (10-100% in 1% steps) to compromise between torque, heat, and noise (per speed).
- **Reduced Amplitude Delay:** Configurable delay before amplitude reduction (0s to 60s) allows the motor to get up to speed before the torque is reduced (per speed).
- **Startup Kick:** Configurable accelerated startup (1x, 2x, 3x, 4x frequency). Starts the motor at a higher speed to overcome drag in the drive system, reduces gradually to desired nominal frequency after configured duration. (Per speed).
- **Kick Ramp Duration:** Configurable accelerated startup ramp duration (0.0s to 15.0s) (per speed).
- **Active Brake Coasting:** Configurable motor braking mode to bring heavy platters to a halt precisely. Choose between:
  - **Pulse Braking:** Pulses reverse torque to quickly stop rotation.
  - **Ramp Braking:** Linearly ramps frequency down to `0Hz` while maintaining braking voltage.
  - **Soft-Stop Coasting:** Gently reduces the frequency to a user-defined cutoff. During the reduction, the motor maintains full driving amplitude, ensuring the drive belt remains under tension. Once the cutoff is reached, power is instantly cut, allowing the remainder of the platter's momentum to dissipate.
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
- **Context-Aware Menus:** Dependent settings are hidden until relevant, such as IIR/FIR filter options, phase offsets, V/f curve points, brake parameters, and screensaver/error durations.
- **Named Value Labels:** Mode settings show readable labels on the OLED instead of raw numeric values where practical.
- **Full Configurability:** Every possible configurable option available as a menu setting.
- **Real-Time Updates:** All settings update in real time when adjustments are being made, to observe their effect on the motor. Settings are only saved to flash on selecting the 'save' option from the menu.
- **Single Encoder Control:** If only the main encoder is installed, click to enter a value, turn to adjust, and click to save.
- **Dual Encoder System (Optional):** If the secondary (pitch) encoder is installed and the system is in the menu, a premium dual-encoder workflow is seamlessly enabled:
  - **Scroll and Adjust:** Use the primary encoder to freely scroll the menu list. Turn the secondary encoder to instantly "adjust" the currently highlighted value without needing to click into it.
  - **Coarse/Fine Editing:** Click the primary encoder to explicitly enter "Edit Mode." The primary encoder now acts as a fine adjustment knob (1X steps), while the secondary encoder acts as a coarse adjustment knob (10X steps), allowing extremely rapid traversal of large values.
- **Primary Encoder Interaction:**
  - **Short Press:** Start/Stop Motor (or Wake from Standby).
  - **Double Press:** Enter Main Menu.
  - **Hold (Dashboard):** Enter Standby.
  - **Hold (Menu):** Save & Exit Menu.
  - **Rotate:** Change Speed (33/45/78).
  - **Press + Rotate:** Cycle Status Views (Standard -> Stats -> Dim).
- **Status Modes:**
  - **Standard:** Large RPM display, state label, frequency readout, pitch/ramp bar, and start/brake/ramp progress when active.
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
- **Dashboard Mode:** When not in the menu, the display shows the current status. Press and rotate the primary encoder to cycle between:
  1. **Standard:** Large Target Speed, Output Frequency, Output Amplitude (V), and elapsed Time.
  2. **Stats:** Tracking counters for total hardware run time and current session time.
  3. **Dim:** Auto-lowers contrast and shows only the speed target.
  4. **Oscilloscope (Scope):** real-time Lissajous diagnostic dashboard drawing X/Y (Phase A / Phase B) waveforms sampled directly from the Pico's DMA bus to visually verify sine tracking integrity and structural Phase Offset health.
  5. **CPU:** Core 0 service load and Core 1 waveform generation load.
  6. **Memory:** Heap usage/free space, plus PSRAM when present.
  7. **Flash:** Sketch flash usage and LittleFS usage.
- **Runtime Tracking:** Optional Runtime tracking to track session and total runtime. Useful for monitoring stylus usage or turntable maintenance schedules.
- **Status Cycling:** Status display cycling (speed, frequency, phase mode, phase angles, runtime) with user settings to toggle the available statuses.
- **Scrolling Messages:**
  - **Welcome:** Scrolls "Welcome to TT Control" on boot.
  - **Goodbye:** Scrolls "Goodbye..." when entering standby, before the display turns off (if configured).

---

### 2.6. Serial Monitor Support
- **Optional Enable:** Optional, enabled or disabled by compile-time configuration flags.
- **Interactive CLI:** Full command-line interface for control and configuration.
- **Commands:** Type `help` or `list` to see available commands. Supports `start`, `stop`, `speed`, `set`, `get`, `save`, `reboot`, `dump settings`, preset management, and diagnostics.
- **Serial Wi-Fi Setup:** Wi-Fi builds add `wifi wizard` for guided Serial Monitor network setup, `wifi scan` for nearby SSIDs, `wifi connect <ssid> [password]` for quick DHCP station setup, and `wifi set ...` commands for hostname, mode, standby mode, hidden SSIDs, DHCP/static IP, setup AP, fallback, and web access options.
- **Diagnostics:** Serial commands include `brake test start`, `brake test stop`, and `relay test <stage|off>` for bench checks.
- **JSON Preset Export/Import:** Advanced configuration sharing.
  - `export preset <1-5>`: Dumps the entire layout of the requested preset, along with all global constraints and brake configurations, into a single minified JSON string.
  - `import preset <1-5> <json>`: Instantly parses and injects a shared JSON configuration string directly into the specified preset slot.
- **Keyboard Control:** Keyboard control over encoder and key functions (j/l/k/m/s/t/i/f).
- **Status Reporting:** Comprehensive status printing and error reporting to serial monitor, set by compile time flag.

---

### 2.7. Wi-Fi Web Interface
- **Automatic Wi-Fi Build Detection:** `NETWORK_ENABLE` defaults to `1` only when the selected Arduino-Pico board target defines `PICO_CYW43_SUPPORTED`. Non-Wi-Fi builds compile the network and web modules out.
- **Default Setup Access Point:** Wi-Fi-capable builds start in open setup access point mode by default using SSID `TTControl-Setup`, so the network setup UI can be reached before any home network credentials are entered.
- **Setup-Only Safety:** When reached through an open setup AP, the hosted server only serves Wi-Fi configuration pages and network APIs. Motor controls, full settings, presets, and error logs are blocked until the device is reached through the configured Wi-Fi network, or through a protected setup access point.
- **Guided Setup Wizard:** The open setup access point serves a network-only wizard for Wi-Fi mode, SSID/password, hidden SSID selection, DHCP/static addressing, fallback AP name/password, and AP channel. Leaving the setup AP password blank creates an open setup network.
- **Serial Setup Wizard:** The Serial Monitor can configure the same network storage with `wifi wizard`, including Wi-Fi scanning, station credentials, hidden SSID selection, DHCP/static addressing, hostname, fallback setup AP settings, and immediate save/reconnect.
- **Network Menu:** The OLED menu exposes Wi-Fi enable/disable, setup AP/station mode, standby mode, hostname, station SSID/password, hidden SSID selection, DHCP, AP fallback, AP SSID/password, AP channel, read-only guest mode, web PIN editing/reset, saved web home page, apply/reconnect, and setup AP actions.
- **Credential Characters:** Browser, serial, and OLED network entry support printable special characters in SSIDs and passwords, including `@`, `/`, `!`, `#`, `$`, and `£`. The OLED text editor includes a shift option for uppercase entry.
- **Hosted Web App:** When the device has a station connection or setup AP active, it hosts a local browser interface on port 80.
- **User-Selectable Home Page:** The device saves which page opens first, including Dashboard, Control, Settings, Calibrate, Network, Presets, Bench, Diagnostics, or Errors. The saved home page applies to any browser that opens the device UI.
- **Dashboard:** The browser dashboard mirrors the supported display modes: Standard, Stats, Dim, Scope, CPU, Memory, and Flash, using live status data from the firmware. The Standard, Stats, and Scope views include a live telemetry chart for frequency, pitch, and amplifier temperature when available.
- **Web UI Theme Presets:** The browser UI includes System, Light, Dark, Calm, Workshop, and High Contrast theme presets, plus large controls and visible focus states.
- **Polished Visual System:** The web UI uses stronger section identities, status color accents, compact button/tab badges, dashboard tiles, sticky navigation/search controls, and a mobile layout tuned for one-handed phone use.
- **Sticky Safety Controls:** Emergency stop, stop, standby/wake, start, and speed buttons remain available in a sticky browser control bar. Emergency stop immediately exits relay test mode, disables waveform output, mutes relays, and stops the motor.
- **Standby Networking:** Network standby keeps Wi-Fi available while the controller is in standby. Eco standby turns Wi-Fi off during standby and reconnects after a physical wake; the web Standby/Wake button can enter Eco standby but is disabled once the browser can no longer wake the device.
- **Simple Control View:** A dedicated large-button control page exposes the day-to-day start/stop/standby/speed/pitch controls without the full settings surface.
- **78 RPM Visibility:** Browser speed controls, bench speed controls, and calibration speed selectors hide 78 RPM while `Enable 78 RPM` is off. A stale 78 RPM browser action is rejected with a clear message.
- **Complete Controls:** Browser controls include start, stop, emergency stop, standby/wake, speed switching, pitch reset/set, relay test, relay test off, runtime reset, factory reset, and API support for reboot.
- **Schema-Driven Full Settings UI:** The web interface fetches `/api/schema` from firmware and builds the complete settings UI from that schema, including global phase, motor, brake, relay, display, system, network, and all per-speed speed/phase/filter/startup settings.
- **Settings Search and Contextual Help:** The full settings page includes search by label, key, or help text. Every schema-driven setting has a Help control with purpose, range, units, accepted choices, key name, and safety context where relevant.
- **Staged Editing and Validation:** Browser settings and network forms track unsaved changes, highlight safety-related edits, provide discard buttons, show a review of pending changes before save, and validate ranges, required fields, frequency limits, password length, and static IPv4 fields before applying.
- **Guided Calibration:** The Calibrate page provides task-based forms for speed frequency, phase offsets, startup kick, braking, and amplitude tuning.
- **Calibration Stepper:** Calibration tasks are presented as a stepper so each workflow can be tuned without scanning every calibration field at once.
- **Accessibility Preferences:** The browser UI includes a remembered home page, theme presets, large controls, visible focus states, live status announcements, semantic form grouping, and labels/error text for screen-reader navigation.
- **Optional Read-Only Guest Mode:** Read-only mode is off by default. When enabled from the web Network page or OLED Network menu, dashboard/status pages remain visible but write actions require the configured web PIN. The PIN can be changed from both the OLED Network menu and the web Network page.
- **Amplifier Status:** The web status API reports amplifier temperature and thermal state when `AMP_MONITOR_ENABLE` is compiled in; the dashboard and telemetry views show the same information.
- **Diagnostics and Events:** The Diagnostics page shows firmware/build info, compile-time feature flags, active pin assignments, network state, amplifier state, stored-file presence, and a recent browser event feed.
- **Bench Test Page:** The Bench page groups live pre-checks, relay output testing, brake start/stop checks, speed and pitch checks, amplifier status, and a generated bench report in one place.
- **Presets and Logs:** Preset load/save/rename/clear/import/export and error log viewing/clearing are available from the browser. Preset load and import actions include validation reports and preview diffs against the current motor settings before applying or storing JSON, and preset slots can be compared with each other.
- **Full Backup:** The Diagnostics page can export, validate, and import a full JSON backup containing motor settings, presets, non-secret network metadata, and the error log. Wi-Fi and web PIN passwords are intentionally not exported.
- **SSE Status Stream:** The dashboard uses a lightweight Server-Sent Events status stream when available, with normal status requests retained for direct refreshes and compatibility.
- **Chart Controls:** The telemetry chart has gridlines, a legend, and selectable series for frequency, pitch, and amplifier temperature.
- **Network Storage:** Wi-Fi credentials and network options are stored separately on LittleFS, avoiding changes to the binary motor settings schema.

---

### 2.8. Settings Management
- **Configurable Presets:** Multiple configurable presets (5 by default, the number defined in configuration flag).
- **Preset Management:** Presets can be reverted to defaults, loaded, renamed and duplicated.
- **Preset Naming:** Presets can have names up to 16 characters, a-z 0-9.
- **Non-Volatile Storage:** Non-volatile settings storage in Pico flash FS.
- **Flash Capacity:** The Pimoroni Pico+2 has 16MB of flash, which can be split at compile time. A standard Pico has more than enough flash for this application also.
- **Wear Levelling:** Flash wear levelling.
- **Factory Reset:** Factory reset with two-level confirmation.
- **Hardware Safe Mode Boot:** A catastrophic state recovery mechanism.
  - Holding down the primary encoder button during system power-on intercepts the boot sequence, bypassing loading configuration from Flash entirely.
  - The system boots directly into a safe, factory-default RAM state using the primary motor defaults, with waveform amplitude initially at 0%, no filters, and standard offsets.
  - This prevents boot-looping or hardware damage caused by corrupted or dangerous parameters, allowing the user to examine the system or overwrite the corrupted settings.
- **Critical Confirmations:** Confirmations for critical settings, such as phase mode and max amplitude changes.
- **Settings File Schema:**
  - **Storage Location:** Stored on LittleFS.
  - **Schema Versioning:** Each file includes `"schema_version"` to ensure forward compatibility.
  - **Migration Strategy:** On schema mismatch or corruption, data is compared, old values are matched, new values are loaded to preserve user settings on firmware upgrade.

---

### 2.9. Power Management
- Standby mode to enter low power state
  - **Compile-time Flag**: `ENABLE_STANDBY` (default `1`). If disabled, the system boots directly to STOPPED state and all standby features are hidden.
- Relay control for secondary power to drive load driving circuitry
- Output muting pins assigned for each channel to control relays or drive the mute lines of amplifiers
  - **Compile-time Flag**: `ENABLE_MUTE_RELAYS` (default `1`). If disabled, relay logic is skipped.
  - **DPDT Support**: `ENABLE_DPDT_RELAYS` (default `0`) allows using 2 DPDT relays instead of 4 SPST relays.
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

### 2.10. Amplifier Monitoring And Thermal Safety
- **Compile-Time Enable:** `AMP_MONITOR_ENABLE` enables the amplifier monitor. It is enabled by default and has no runtime enable/disable toggle.
- **Temperature Sensor:** `PIN_AMP_TEMP` reads a TMP36-style analogue heatsink sensor on ADC GP26. The firmware samples it every 500 ms and converts voltage to degrees Celsius.
- **Thermal Cutout Input:** `PIN_AMP_THERM_OK` reads the amplifier thermal cutout/status line on GP27 using an input pulldown. HIGH means the amplifier thermal chain is healthy; LOW triggers an immediate critical shutdown.
- **Warning Threshold:** At the configured warning threshold, defaulting to `AMP_TEMP_WARN_C` (`65.0C`), the firmware logs a non-critical `ERR_AMP_THERMAL` warning. The warning re-arms once temperature falls at least 5C below the warning threshold.
- **Shutdown Threshold:** At the configured shutdown threshold, defaulting to `AMP_TEMP_SHUTDOWN_C` (`75.0C`), or when the thermal OK input goes LOW, the firmware calls `motor.emergencyStop()`, logs a critical `ERR_AMP_THERMAL`, mutes/stops outputs, and latches shutdown behavior until reboot.
- **Status Reporting:** The serial `status` / `i` command reports amplifier temperature and thermal state. The web API and Stats dashboard also expose these readings when the feature is compiled in.
- **Configuration:** When `AMP_MONITOR_ENABLE` is `1`, warning and shutdown thresholds are configurable from the OLED System menu and the web Settings page. Pins remain compile-time hardware assignments in `config.h`.

---

### 2.11. Multi-Core & DMA Architecture
- **Core 0:** Handles UI, menu system, encoder input, and high-level motor control logic.
- **Core 1:** Dedicated to waveform buffer management.
- **DMA & Hardware PWM:** The actual waveform generation is offloaded to the RP2350's DMA controller and PWM hardware. This ensures:
  - Zero CPU jitter in the output signal.
  - Extremely low CPU usage (Core 1 only wakes to refill buffers).
  - Robust operation even during heavy UI activity.
- **Inter-Core Communication:** Optimized inter-core communication.
- **PIO Usage:** Maximize use of Pico state machines.
- **FIFO Usage:** Optimal use of FIFO.
- **Non-Blocking Design:** Non-blocking code, including delays, and timers and interrupt routines.

---

### 2.12. Error Handling
- **Initialization Checks:** Display initialization check.
- **Range Validation:** Settings range validation.
- **Error Reporting:** Error code system to display errors on screen (configurable in menu) and via serial output (configurable in flag).
- **Error Display Duration:** Error codes should be displayed on OLED or LCD for a configurable duration (default 10s, configurable in menu).
- **Error Clearing:** Pressing encoder clears the error.
- **Safety Shutdown:** Error Mode switches off all relays when an error message is displayed.
- **Amplifier Thermal Events:** Amplifier temperature warnings, thermal cutout trips, and amplifier over-temperature shutdowns are logged as `ERR_AMP_THERMAL`.

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
| `speed <0-2>` | Set speed (0=33, 1=45, 2=78). Speed 2 is rejected when 78 RPM is disabled. |
| `s` | Cycle speed |
| `t` | Toggle standby |
| `p` | Reset pitch |
| `status` / `i` | Show current status; includes CPU load, heap use, flash/filesystem use, and amplifier temperature/thermal state when enabled |
| `list` | **List all available settings and values** |
| `set <key> <val>` | Set a parameter value |
| `get <key>` | Get a parameter value |
| `save` | Save current RAM settings to flash |
| `reboot` | Reboot via watchdog |
| `dump settings` | Print a readable settings summary |
| `preset list` | List preset slots |
| `preset load <1-5>` | Load a preset into RAM and apply it |
| `preset save <1-5>` | Save current RAM settings to a preset slot |
| `brake test start` | Start motor for brake tuning |
| `brake test stop` | Stop motor using the configured brake mode |
| `relay test <0-N>` | Enter relay test and activate a relay output stage |
| `relay test off` | Exit relay test and restore normal relay handling |
| `wifi help` | List Serial Monitor Wi-Fi setup commands when network support is enabled |
| `wifi status` | Show current network state, active IP address, SSID, AP status, MAC, and RSSI |
| `wifi config` | Show saved network configuration without printing passwords |
| `wifi wizard` | Start guided Serial Monitor network setup |
| `wifi scan` | Scan nearby Wi-Fi networks |
| `wifi connect <ssid> [password]` | Save station credentials, enable Station + setup AP mode, use DHCP, and reconnect |
| `wifi set standby <network\|eco>` | Select Network standby or Eco standby |
| `wifi set hidden <on\|off>` | Mark the configured station SSID as hidden or visible |
| `wifi set <key> <value>` | Stage an individual network setting such as mode, standby mode, hostname, SSID, hidden SSID flag, DHCP, static IP, fallback, setup AP, web PIN, or read-only mode |
| `wifi clear password\|ap_password\|ssid` | Clear saved station password, setup AP password, or station SSID |
| `wifi apply` | Save staged network settings and reconnect |
| `wifi reset` | Restore network defaults and reconnect |
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
| `phase_mode` | Phase mode (1-3 by default; 1-4 if `ENABLE_4_CHANNEL_SUPPORT` is set to `1`) | Int |
| `max_amp` | Global maximum amplitude (0-100) | Int |
| `amp_warn` | Amplifier warning temperature in C (`AMP_MONITOR_ENABLE` only) | Float |
| `amp_shutdown` | Amplifier shutdown temperature in C (`AMP_MONITOR_ENABLE` only) | Float |
| `smooth_switch` | Smooth speed switching (0/1) | Bool |
| `switch_ramp` | Speed switch ramp duration (s) | Int |
| `brake_mode` | Brake mode (0=Off, 1=Pulse, 2=Ramp, 3=SoftStop) | Int |
| `brake_duration` | Brake duration (s) | Float |
| `brake_pulse_gap` | Pulse brake gap (s) | Float |
| `brake_start_freq` | Ramp brake start frequency (Hz) | Float |
| `brake_stop_freq` | Ramp brake stop frequency (Hz) | Float |
| `brake_cutoff` | Soft-stop cutoff frequency (Hz) | Float |
| `relay_active_high` | Relay active polarity (0/1) | Bool |
| `relay_delay` | Relay power-on delay (s) | Int |
| **Current Speed** | | |
| `freq` | Frequency (Hz) | Float |
| `phase1`..`phase4` | Phase Offsets (Degrees) | Float |
| `soft_start` | Soft Start Duration (s) | Float |
| `kick` | Startup Kick Multiplier (e.g. 2) | Int |
| `kick_dur` | Startup Kick Duration (s) | Int |
| `filter` | Current speed filter (0=None, 1=IIR, 2=FIR) | Int |
| `reduced_amp` | Current speed reduced amplitude (%) | Int |
| `amp_delay` | Current speed amplitude delay (s) | Int |
| **Live** | | |
| `pitch` | Current Pitch (%) | Float |

---

## 3.3. Screens and Menus
The menu structure is designed for a data-driven implementation.

### Main Menu
- **Exit Safe Mode** (Only visible if currently in Safe Mode: forces watchdog reboot targetting normal flash load)
- **Edit Speed: [33/45/78]** (Toggle speed context for submenus)
- **Speed Tuning:** Frequency, Limits, Filters (Per-Speed).
- **Phase Control:** Phase Mode (Global), Offsets (Per-Speed).
- **Motor Control:** Amps/Kick (Per-Speed), Braking (Global).
- **Power Control:** Relays, Auto Standby/Boot (Global).
- **Display:** Sleep, Dim, Saver, Errors (Global).
- **System:** Pitch Reset, 78RPM, Logs (Global).
- **Network:** Wi-Fi setup and local web interface connection flow (Wi-Fi builds only).
- **Presets:** Load, Save, Rename, and Clear presets.
- **Save & Exit:** Saves all changes to flash and returns to dashboard.
- **Cancel:** Discards changes and reloads from flash.

### Speed Tuning
- **Frequency:** Nominal frequency (Hz).
- **Min Freq:** Limit for pitch control.
- **Max Freq:** Limit for pitch control.
- **Filt Type:** Digital Filter Type (0=None, 1=IIR, 2=FIR).
- **IIR Alpha:** IIR Filter smoothing factor.
- **FIR Prof:** FIR Filter Profile (0=Gentle, 1=Medium, 2=Aggressive).

### Phase Control
- **Mode (Glb):** 1, 2, or 3 phase operation by default; 4 phase operation appears only when `ENABLE_4_CHANNEL_SUPPORT` is set to `1`.
- **Ph 2 Offs:** Phase 2 offset for multi-phase motors.
- **Ph 3 Offs:** Phase 3 offset for multi-phase motors.
- **Ph 4 Offs:** Phase 4 offset for multi-phase motors.
- **Sweep Diag.:** Symmetric Resonance Sweep mode.

### Motor Control
- **Soft Start**: Adjustable duration (0.0s to 10.0s) to ramp up amplitude gently.
- **Red. Amp %**: Automatically lower voltage after spin-up to reduce motor noise and heat (10-100%).
- **Amp Delay**: Configurable delay before amplitude reduction (0s to 60s).
- **Kick Mult**: Startup kick multiplier.
- **Kick Dur**: Startup kick duration.
- **Kick Ramp**: Ramp down duration from kick.
- **V/f Blend%**: Voltage/Frequency intensity mix against the base amplitude.
- **V/f LowHz**: Frequency defining the low-speed V/f boost point.
- **V/f Low%**: Voltage boost % at LowHz.
- **V/f MidHz**: Frequency defining the mid-speed V/f boost point.
- **V/f Mid%**: Voltage boost % at MidHz.
- **Max Amp %**: Global maximum amplitude limit.
- **SS Curve**: Soft Start Profile (0=Linear, 1=Log, 2=Exp).
- **Smooth Sw**: Enable smooth frequency ramping between speeds.
- **Sw Ramp**: Duration (s) for speed switch ramp.
- **Brake Mode**: Braking mechanism (0=Off, 1=Pulse, 2=Ramp, 3=SoftStop).
- **Brake Dur**: Braking duration (s).
- **Brk Pulse**: Gap between pulses in Pulse mode (s).
- **Brk StartF**: Frequency ramp range start for Ramp mode.
- **Brk StopF**: Frequency ramp range stop for Ramp mode.
- **Brk Cutoff**: Frequency when power drops in SoftStop coasting.
- **Ramp Type**: Frequency soft start style (0=Linear, 1=S-Curve).
- **Auto Start**: Start motor immediately after boot/wake.
- **Brake Tune:** Guided brake tuning page with mode-specific settings, explicit Start Motor and Brake Stop actions, and Save Brake.

### Power Control
- **Rly: ActHi:** Toggle relay logic (Active High/Low). (If `ENABLE_MUTE_RELAYS`)
- **Rly: Stby:** Mute relays when entering Standby. (If `ENABLE_STANDBY`)
- **Rly: S/S:** Unmute relays on Start, Mute on Stop.
- **Rly: Delay:** Power-on delay for relays (s).
- **Auto Stby:** Auto-standby delay (min). (If `ENABLE_STANDBY`)
- **Auto Boot:** Boot directly to operation (bypass standby).
- **Relay Test:** Bench diagnostic that turns waveform output off and activates one relay output stage at a time. It refuses to run while the motor is moving.

### Display
- **Brightness:** OLED Brightness/Contrast (0-255).
- **Sleep Dly:** Display sleep delay (s).
- **Scrn Saver:** Enable screensaver in standby.
- **Saver Mode:** Select screensaver animation (0=Bounce, 1=Matrix, 2=Lissajous).
- **Auto Dim:** Auto-dim delay (min).
- **Show Runtime:** Toggle runtime display on dashboard.
- **Show CPU:** Toggle the CPU load dashboard option.
- **Show Memory:** Toggle the memory usage dashboard option.
- **Show Flash:** Toggle the flash/filesystem dashboard option.
- **Err Display:** Toggle on-screen error messages.
- **Err Dur:** Duration of error messages (s).

### System
- **Ver:** Firmware Version info.
- **Rev Encoder:** Reverse the direction of the main encoder.
- **Pitch Step:** Step size for pitch control (0.01% - 1.0%).
- **Pitch Reset:** Reset pitch to 0% on motor stop.
- **Enable 78:** Toggle availability of 78 RPM mode.
- **Amp Warn C:** Amplifier warning temperature. (If `AMP_MONITOR_ENABLE`)
- **Amp Shut C:** Amplifier shutdown temperature. (If `AMP_MONITOR_ENABLE`)
- **Boot Speed:** Select default speed on boot (0=33, 1=45, 2=Last Used).
- **Welcome Msg:** Configurable welcome message on boot.
- **Goodbye Msg:** Configurable goodbye message on shutdown.
- **Error Log:** View and clear system error logs.
- **Reset Runtime:** Reset the total runtime counter (with confirmation).
- **Fact Reset:** Factory Reset.

When `AMP_MONITOR_ENABLE` is compiled in, amplifier monitoring runs automatically. Warnings and shutdowns appear through the Error Log, serial status, and web status/dashboard; thresholds are editable in the System menu and web Settings page.

### Network
- **Status:** Shows the current Wi-Fi connection state.
- **Web:** Shows the active IP address for the browser interface.
- **Wi-Fi:** Enable or disable network services.
- **Mode:** Select Setup AP, Station, or Station plus setup AP.
- **Host:** mDNS/DHCP hostname used by the device.
- **SSID:** Station network SSID.
- **Hidden SSID:** Connect to a network that does not advertise its name.
- **Pass:** Station network password.
- **DHCP:** Use DHCP for station networking.
- **AP Fallback:** Start the setup AP if station connection fails.
- **Standby:** Select Network standby, which keeps Wi-Fi enabled in standby, or Eco standby, which turns Wi-Fi off in standby and reconnects after physical wake.
- **AP SSID:** Setup access point SSID.
- **AP Pass:** Setup access point password.
- **AP Channel:** Setup access point channel.
- **ReadOnly:** Enables optional read-only guest mode for the web interface. Off by default.
- **Web PIN:** Sets the 4-8 character PIN used to unlock web write actions when read-only mode is enabled.
- **Web Home:** Selects the default page shown when the web interface opens.
- **Reset PIN:** Restores the web PIN to `NETWORK_DEFAULT_WEB_PIN`.
- **Apply:** Save network settings and reconnect.
- **Setup AP:** Force the device into setup access point mode.
- **Refresh:** Rebuild the Network page with current status.

### Presets

Lists numbered slots (1: Preset 1, 2: High Torque, etc.). Clicking a slot reveals:

- **Load:** Load settings from this slot.
- **Save:** Save current settings to this slot.
- **Apply Name:** Submits naming buffer to rename slot.
- **Clear:** Reset slot to defaults.

---

## 4. Compile-Time Flags

| Flag Category | Flag Name | Purpose | Default / Pin Assignment / Options | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **System Info** | `FIRMWARE_VERSION` | Defines the firmware version string. | `"v1.0.0"` | Displayed on splash screen and system menu. |
| **System Info** | `BUILD_DATE` | Stores the timestamp/date of the firmware compilation. | If possible, automatically generated by build script. | |
| **Flash** | `LITTLEFS_FS_SIZE` | Defines the size of the LittleFS partition for settings storage. | `8MB` (Min `1MB` recommended) | Critical for non-volatile storage setup. |
| **Core Hardware**| `OLED_I2C_ADDRESS` | Defines the I2C address for the SSD1306 display. | `0x3C` | Essential for display communication. |
| **Core Hardware**| `OLED_WIDTH` | Defines the width of the OLED display. | `128` | |
| **Core Hardware**| `OLED_HEIGHT` | Defines the height of the OLED display. | `64` | |
| **Waveform** | `LUT_MAX_SIZE` | Sets the maximum size of the sine wave Look-Up Table (LUT). | `16384` | Affects memory usage and processing load. |
| **Waveform** | `MIN_OUTPUT_FREQUENCY_HZ` | Minimum generated sine frequency. | `10.0` | Used by menus, validation, serial commands, and waveform output. |
| **Waveform** | `MAX_OUTPUT_FREQUENCY_HZ` | Maximum generated sine frequency. | `1500.0` | Used by menus, validation, serial commands, and waveform output. |
| **Presets** | `MAX_PRESET_SLOTS` | Defines the maximum number of user-configurable presets. | `5` | Used to size the data structure for presets. |
| **Network** | `NETWORK_ENABLE` | Enables Wi-Fi, local web server, network menu, and web settings UI. | Auto: `1` on `PICO_CYW43_SUPPORTED`, otherwise `0` | Can be overridden at compile time. |
| **Network** | `NETWORK_CONFIG_VERSION` | Tag for network settings schema. | `4` | Used to migrate `/network.bin` when Wi-Fi settings change. |
| **Network** | `NETWORK_WEB_PIN_MAX` | Maximum web unlock PIN length. | `8` | Web PINs shorter than 4 characters are rejected by the browser/API. |
| **Network** | `NETWORK_DEFAULT_HOSTNAME` | Default network hostname. | `"ttcontrol"` | Stored separately from motor settings in `/network.bin`. |
| **Network** | `NETWORK_DEFAULT_AP_SSID` | Default setup access point SSID. | `"TTControl-Setup"` | Used on first boot of Wi-Fi builds. |
| **Network** | `NETWORK_DEFAULT_AP_PASSWORD` | Default setup access point password. | `""` | Blank means open setup AP. Open setup AP requests are limited to Wi-Fi configuration only. If set, use at least 8 characters. |
| **Network** | `NETWORK_DEFAULT_AP_CHANNEL` | Default setup access point channel. | `6` | Configurable from menu and web UI. |
| **Network** | `NETWORK_DEFAULT_WEB_PIN` | Default PIN used if read-only guest mode is enabled before changing the PIN. | `"1234"` | Read-only guest mode is off by default; change this PIN before enabling it on a shared network. |
| **Serial/Debug** | `SERIAL_MONITOR_ENABLE` | Enables/Disables all Serial Monitor functionality and setup. | `1` or `0` | Global toggle for serial output. |
| **Serial/Debug** | `DUPLICATE_DISPLAY_TO_SERIAL` | Duplicates all display output to the serial monitor. | `1` or `0` | Requires `SERIAL_MONITOR_ENABLE` to be `1`. |
| **Features** | `ENABLE_STANDBY` | Enable/Disable Standby Mode | `1` | Controls standby functionality. |
| **Features** | `ENABLE_MUTE_RELAYS` | Enable/Disable Mute Relays | `1` | Controls relay logic. |
| **Features** | `ENABLE_DPDT_RELAYS` | Use 2x DPDT instead of 4x SPST | `0` | Changes relay switching logic. |
| **Features** | `ENABLE_4_CHANNEL_SUPPORT` | Enable 4-phase/4-channel support. | `0` | Normal firmware builds expose three channels. Set to `1` to enable optional four-channel/Premotec bridge modes on hardware that has the fourth output populated. |
| **Features** | `AMP_MONITOR_ENABLE` | Enable amplifier temperature and thermal cutout monitoring. | `1` | Samples every 500 ms using `PIN_AMP_TEMP` and `PIN_AMP_THERM_OK`; thresholds appear in the System menu and web Settings page. |
| **Optional Pins**| `PITCH_CONTROL_ENABLE` | Enables the secondary (Pitch) encoder functionality and logic. | `0` (Disabled) | **Required** flag for the optional pitch feature (3.3). |
| **Optional Pins**| `PIN_ENC_PITCH_CLK` | Assigns the pin for the Pitch Encoder Clock. | `13` | Only compiled if `PITCH_CONTROL_ENABLE` is `1`. |
| **Optional Pins**| `PIN_ENC_PITCH_DT` | Assigns the pin for the Pitch Encoder Data. | `14` | Only compiled if `PITCH_CONTROL_ENABLE` is `1`. |
| **Optional Pins**| `PIN_ENC_PITCH_SW` | Assigns the pin for the Pitch Encoder Switch. | `15` | Only compiled if `PITCH_CONTROL_ENABLE` is `1`. |
| **Optional Pins**| `STANDBY_BUTTON_ENABLE` | Enables the external Standby button. | `0` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `PIN_BTN_STANDBY` | Assigns the pin for the Standby Button. | `21` | Only compiled if `STANDBY_BUTTON_ENABLE` is `1`. |
| **Optional Pins**| `SPEED_BUTTON_ENABLE` | Enables the external Speed change button. | `0` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `PIN_BTN_SPEED` | Assigns the pin for the Speed Button. | `22` | Only compiled if `SPEED_BUTTON_ENABLE` is `1`. |
| **Optional Pins**| `START_STOP_BUTTON_ENABLE` | Enables the external Start/Stop motor button. | `0` (Disabled) | **Required** flag for the optional button (3.5). |
| **Optional Pins**| `PIN_BTN_START_STOP` | Assigns the pin for the Start/Stop Button. | `9` | Only compiled if `START_STOP_BUTTON_ENABLE` is `1`; selected because GP9 is exposed on Pico 2 W. |
| **Monitor Pins**| `PIN_AMP_TEMP` | Amplifier heatsink temperature ADC input. | `26` | TMP36-style analogue sensor. |
| **Monitor Pins**| `PIN_AMP_THERM_OK` | Amplifier thermal cutout/status input. | `27` | HIGH means healthy; LOW triggers critical shutdown. |
| **Thermal Limits**| `AMP_TEMP_WARN_C` | Factory default amplifier warning temperature. | `65.0` | Runtime configurable when `AMP_MONITOR_ENABLE` is `1`; logs non-critical `ERR_AMP_THERMAL`. |
| **Thermal Limits**| `AMP_TEMP_SHUTDOWN_C` | Factory default amplifier shutdown temperature. | `75.0` | Runtime configurable when `AMP_MONITOR_ENABLE` is `1`; calls emergency stop, disables waveform output, and mutes relays. |
| **Thermal Limits**| `AMP_TEMP_MIN_C` | Lowest configurable amplifier threshold. | `30.0` | Used to constrain saved settings. |
| **Thermal Limits**| `AMP_TEMP_MAX_C` | Highest configurable amplifier threshold. | `120.0` | Used to constrain saved settings. |
| **Thermal Limits**| `AMP_TEMP_MIN_SHUTDOWN_MARGIN_C` | Minimum gap between warning and shutdown thresholds. | `1.0` | Keeps warning below shutdown. |
| **Thermal Limits**| `AMP_TEMP_WARN_HYSTERESIS_C` | Temperature drop required to re-arm the warning. | `5.0` | Warning-only hysteresis. |
| **Display Msg** | `STANDBY_MESSAGE` | Sets the message displayed while the system is in standby, if enabled in the menu. | `message` | Controls message display logic. |
| **Display Msg** | `WELCOME_MESSAGE` | Sets the message displayed on boot. | `"Welcome to TT Control"` | |
| **Storage** | `SETTINGS_SCHEMA_VERSION` | Tag for settings file schema. | `5` | Use for compatibility checks when firmware is updated. |
| **Defaults** | `DEFAULT_PHASE_MODE` | Default phase mode. | `3` (3-phase) | |
| **Defaults** | `DEFAULT_SPEED_INDEX` | Default speed index. | `0` (33.3 RPM) | |
