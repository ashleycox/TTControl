# Features

TT Control combines a multi-phase DDS waveform generator, a motor state machine, persistent tuning, and optional monitoring and control systems. Compile-time flags remove hardware-specific features from builds which do not use them.

## Sine-wave generation

- **DMA and hardware PWM:** Four chained DMA channels feed two PWM slices from paired 256-sample buffers. Core 1 refills the free buffer while DMA and PWM maintain output timing independently of the user interface.
- **Direct digital synthesis:** A 32-bit phase accumulator sets motor frequency independently of the PWM carrier.
- **PWM carrier:** Output uses 10-bit duty values. The carrier defaults to 50 kHz and is calculated from the live system clock for RP2040 and RP2350 targets.
- **Lookup table:** The sine table contains 16,384 signed samples by default. `LUT_MAX_SIZE` is a compile-time, power-of-two setting with a minimum of 1,024 samples.
- **Interpolation:** Fractional phase-accumulator bits linearly interpolate between adjacent table entries.
- **Frequency range:** The waveform generator accepts 10-1500 Hz. OLED frequency tuning uses 0.1 Hz steps, and each speed has independent minimum and maximum limits.
- **Three speeds:** 33⅓, 45, and 78 RPM have separate frequency and tuning records. The factory frequencies for the primary 12-pole, 7.52:1 belt-drive setup are 25.07 Hz, 33.85 Hz, and 58.66 Hz.
- **78 RPM control:** 78 RPM can be removed from speed selection without deleting its stored tuning.
- **Diagnostics:** Serial and web status expose sample rate, DMA health, phase vectors, channel gains, modulation headroom, and per-channel clipping counters where applicable.

The electrical output stages and PWM semantics are described in [Output configuration](output-configuration.md).

## Multi-phase output

- **Active output count:** Standard builds expose one, two, or three active outputs. Four-channel output is an explicit linear-backend option controlled by `ENABLE_4_CHANNEL_SUPPORT`.
- **Inactive outputs:** Waveform channels above the selected count are held at neutral, independently of relay availability.
- **Motor topology:** Custom, Twin-phase synchronous, and Three-phase sine layouts provide named tuning contexts and reset values.
  - **Custom:** Leaves the stored phase tune unchanged when reset.
  - **Twin-phase synchronous:** Resets A=0°, B=180°, and C=270°.
  - **Three-phase sine:** Resets A=0°, B=120°, and C=240°.
- **Editable topology:** Selecting a topology does not lock the phase or gain controls after the reset values have been applied.
- **Per-speed phase trim:** Every active channel has an independent offset from -360° to +360° with 0.1° OLED resolution.
- **Per-speed gain trim:** Every active channel has an independent gain from 50-150%.
- **Live transitions:** Configurable phase and gain slew rates soften live adjustments. A rate of zero applies the new value immediately.
- **Output sweep:** A diagnostic sweep can vary symmetrical phase separation, an individual active phase, or an individual active gain while the motor runs. Pressing the encoder locks the current point; leaving without locking restores the full pre-sweep tune.
- **Direction:** Normal drive direction is fixed. Reverse phase progression is used only by configured braking modes.

### Three-PWM bridge backend

- Drives phases A-C as 3PWM inputs for a controller-free triple half-bridge driver.
- Uses 50% duty as the zero-amplitude neutral command.
- Supports a shared enable, active-low fault input, optional independent phase enables, and optional sleep and reset controls.
- Independent phase enables follow the selected active output count when `POWER_STAGE_PHASE_ENABLES` is compiled.
- Start-up holds neutral buffers before enabling the power stage. Wake, enable and phase-enable timing are compile-time hardware settings.
- An asserted driver fault disables the hardware paths in the GPIO interrupt path, records a fault snapshot, and latches the critical interlock until reboot.
- External mute relays and four-channel output are rejected at compile time.
- Active braking remains inhibited until **Regen Safe** confirms that the DC bus can absorb returned energy.

### Linear PWM backend

- Drives up to four filtered PWM channels for external linear amplifiers.
- Retains the original standby relay, downstream mute relays, staggered switching, relay polarity, and optional two-relay DPDT arrangement.
- Relay-test mode disables waveform output and activates one relay stage at a time.
- Bridge lifecycle, phase-enable, fault-snapshot, and regenerative-braking confirmation controls are omitted.

Pin ownership, polarity and commissioning checks are in [Output configuration](output-configuration.md), with flags and assignments in [Build and hardware configuration](build-configuration.md).

## Motor control

The state machine has Standby, Stopped, Starting, Running, and Stopping states. Stopping remains an active motion state until the selected braking sequence has completed.

### Starting and amplitude

- **Maximum amplitude:** A global 0-100% limit caps all drive and braking output.
- **Soft start:** Each speed has a continuous 0.0-10.0 second amplitude ramp.
- **Soft-start profiles:** S-curve mode uses a sine-shaped ease-in and ease-out. Linear ramp mode can use linear, logarithmic, or exponential amplitude shaping.
- **Startup kick:** Each speed can start at one to four times its target frequency for 0-15 seconds, then return over a configurable 0.0-15.0 second ramp.
- **Reduced amplitude:** Each speed can reduce its running amplitude to 10-100% after a 0-60 second delay, allowing full starting torque before heat and noise are reduced.
- **V/f shaping:** A three-point curve provides low-frequency and mid-frequency output levels, an explicit base frequency at which the curve reaches 100%, and a 0-100% blend. Zero blend bypasses V/f scaling and is the default.
- **Complete V/f application:** The curve follows the absolute commanded frequency during startup, normal running, smooth speed changes, pitch adjustment, closed-loop correction, and braking. Output remains at 100% of the current drive envelope above the base frequency.
- **Live V/f changes:** OLED, serial, and web changes are re-evaluated while the motor runs. The global maximum amplitude remains the ceiling.
- **Auto Start:** The motor can start automatically after boot or after waking from standby.

### Speed changes

- **Smooth switching:** Changing speed while running can move directly to the new frequency or use a configurable 1-5 second ramp.
- **Per-speed limits:** Pitch and closed-loop corrections remain inside the selected speed's frequency limits.
- **Closed-loop ramp handling:** Feedback correction can remain open-loop until a speed ramp completes or use a separate, limited proportional correction against the live ramp target.

### Braking

- **Off:** Ramps amplitude down at the current frequency, or cuts immediately when duration is zero.
- **Pulse:** Applies reverse torque in pulses with a configurable 0.1-2.0 second gap.
- **Ramp:** Uses reverse progression while frequency moves from the configured start frequency towards the stop frequency and amplitude falls.
- **SoftStop:** Reduces frequency under drive until the configured cut-off, then removes power.
- **Duration:** Braking duration is configurable from 0.0-10.0 seconds.
- **Bench tuning:** OLED and Serial Monitor provide explicit brake-test start and stop actions.
- **Bridge safeguard:** A bridge build with **Regen Safe** off substitutes the non-regenerative amplitude ramp and then disables the stage.

### Pitch control

`PITCH_CONTROL_ENABLE` adds a second encoder for pitch and menu editing.

- Pitch remains inside both the selected ±10%, ±20%, ±30%, ±40%, or ±50% range and the active speed's frequency limits.
- The adjustment step is configurable from 0.01-1.0%.
- A short pitch-encoder press cycles the available percentage range.
- Holding the pitch encoder for at least two seconds resets pitch to zero.
- Pitch can reset automatically when the motor stops and always resets on standby.
- In the menu, the pitch encoder adjusts the highlighted value directly. During explicit edit mode it supplies coarse steps while the main encoder supplies fine steps.
- Closed-loop Follow mode moves the RPM target with effective pitch so feedback does not oppose a deliberate pitch change.

## Closed-loop speed control

`CLOSED_LOOP_SPEED_ENABLE` adds platter-speed monitoring and correction. Existing hardware without a speed sensor is unchanged when the feature is compiled out.

- **Sensor support:** GP6 can read a pulse tachometer. GP6 and GP7 can read A/B quadrature feedback with x1, x2, or x4 decoding.
- **Direction handling:** Quadrature mode reports raw and corrected direction, supports a reversed sensor orientation, and can ignore, warn, or stop on reverse motion.
- **Control modes:** Monitor reports measured RPM and lock state without changing the output. Correct applies bounded feedback adjustment to the generated frequency.
- **Per-speed targets:** 33, 45, and 78 RPM have independent target values.
- **Per-speed tuning:** Deadband, lock tolerance and time, Kp, Ki, Kd, integral limit, correction limit, slew limit, ramp Kp, and ramp correction limit are stored separately for each speed.
- **Global timing:** Sensor debounce, signal timeout, engagement delay, update interval, and RPM filter alpha are global.
- **Startup engagement:** Correction remains off during soft start, startup kick, kick ramp-down, and braking. It can wait for a valid signal and measured speed near the target before engaging.
- **Smooth speed changes:** Feedback can remain open-loop until the ramp settles or track its live target using separate limits. Controller state is reset when the transition completes.
- **Pitch target modes:** Fixed holds the configured RPM target. Follow applies the effective pitch ratio after the same frequency limits used by the motor output.
- **Pitch target changes:** A target slew rate can soften deliberate pitch changes, and a configurable threshold resets controller state after a large target jump.
- **Reduced-amplitude recovery:** Loss of lock after amplitude reduction can warn or restore full amplitude after a delay.
- **Signal-loss actions:** Dropout can open the loop, hold the last correction, or stop the motor.
- **Safety actions:** Correction saturation, implausible RPM, lock timeout, and reverse direction can be ignored, warned, or escalated to a motor stop as appropriate.
- **Sensor setup:** OLED, Serial Monitor, and web Bench controls can capture one manual platter revolution and apply the suggested counts-per-revolution. Quadrature setup can also suggest direction reversal.
- **Guided tuning:** The tuning sequence covers sensor validation, monitor-only running, Kp, Ki, limits, and final verification. The current safe recommendation can be applied directly.
- **Base-frequency calibration:** After sufficient stable data, the average correction can be previewed, applied in RAM, or applied and saved to the current speed's base frequency.
- **Stability metrics:** Runtime figures include valid and locked samples and time, average and peak RPM error, correction saturation time, dropout, direction, plausibility and lock-timeout events, amplitude recovery, and error sign changes.
- **Sensor health:** Diagnostics include accepted and rejected transitions, debounce rejection, interval minimum, maximum and average, and interval jitter.
- **Trend capture:** A rolling buffer records target RPM, measured RPM, error, correction, signal validity, and lock state.
- **Fault detail:** Closed-loop log entries include the target, measured RPM, RPM error, correction, signal state, count, and direction when a sample is available.
- **Status surfaces:** OLED tools, `cl status`, `cl health`, `cl trend`, the web dashboard, Bench page, diagnostics, presets, and backup data expose the relevant compiled state.

Operating and tuning details are in [Closed-loop speed control](closed-loop-control.md).

## Digital filtering

Filtering is selected independently for each speed and is off by default.

- **None:** Uses the interpolated lookup-table sample without an additional digital filter.
- **IIR:** A first-order exponential filter with alpha from 0.01-0.99. Lower alpha gives stronger smoothing and slower response; higher alpha follows changes more quickly.
- **FIR:** An eight-tap convolution filter with Gentle, Medium, and Aggressive coefficient sets.
- **Independent state:** Each output channel keeps its own IIR and FIR history, so phase channels do not share filter state.
- **Output position:** Filtering is applied to the generated sample before it is converted to PWM duty.

The filters can reduce high-frequency steps in the generated samples, but stronger profiles can soften transients and attenuate the desired waveform. Their usefulness depends on the motor, amplifier and output filter.

## OLED user interface

- **Display:** A 128x64 SSD1306 provides the dashboard, hierarchical menus, confirmations, text entry, diagnostic tools, and stored error log.
- **Context-aware menu:** Filter parameters, active phases, V/f points, braking controls, closed-loop values, network pages, relays, and bridge controls appear only when relevant.
- **Named modes:** Enumerated settings use readable names rather than unexplained numbers where display width permits.
- **Edit buffer:** Menu changes remain in RAM until **Save & Exit**. Cancel reloads the saved settings, and filesystem failures are reported.
- **Single-encoder editing:** Click enters a value, turn adjusts it, and click accepts it.
- **Dual-encoder editing:** With the pitch encoder compiled, turning it changes the highlighted value without entering edit mode. In edit mode the main encoder is fine adjustment and the pitch encoder is coarse adjustment.
- **Encoder acceleration:** Faster rotation increases the adjustment rate for long ranges.
- **Long-page navigation:** Menus use a moving window and right-edge position indicator.

### Primary encoder actions

- **Short press:** Wakes from standby or starts and stops the motor. A running motor stops on release without waiting for the double-press interval.
- **Double press:** Opens the Main Menu.
- **Turn:** Changes between enabled speeds.
- **Press and turn:** Cycles dashboard views.
- **Hold on dashboard:** Enters standby.
- **Hold in submenu:** Returns one level.
- **Hold in Main Menu:** Cancels menu edits and exits.
- **Very long hold in menu:** Saves and exits.

### Dashboard and display behaviour

- **Standard:** Shows target or measured speed, state, frequency, pitch or ramp deviation, start and braking progress, and a real closed-loop lock indicator when available.
- **Stats:** Shows session and total runtime.
- **Dim:** Provides a low-brightness speed display.
- **Scope:** Draws a Lissajous view from phase A and phase B waveform samples.
- **CPU:** Shows Core 0 and Core 1 utilisation.
- **Memory:** Shows heap and optional PSRAM use.
- **Flash:** Shows sketch and LittleFS use.
- CPU, Memory, and Flash pages can be removed from the cycle individually. Standard, Stats, Dim, and Scope remain available.
- Auto Dim can select the Dim page after 0-60 minutes of running inactivity.
- Display sleep offers Off, 10s, 20s, 30s, 1m, 5m, and 10m delays.
- Standby screensavers include Bounce, Matrix, and Lissajous modes.
- Boot and standby transitions show the welcome and goodbye messages.
- Error display and duration can be configured independently of persistent logging.

### Optional physical controls and locking

- Separate standby, speed, and start/stop buttons are compiled independently.
- The shared 4-8 character PIN can lock menu entry, normal serial writes, browser writes, starting, speed changes, and pitch changes.
- Physical stop and standby remain available while locked.
- The OLED can set, enable, disable, apply, and immediately lock the shared PIN.

The complete menu tree and backend-specific visibility table are in [OLED user interface](user-interface.md).

## Serial Monitor

- **Compile-time control:** `SERIAL_MONITOR_ENABLE` includes or removes the 115200-baud interface.
- **Interactive commands:** Motor start, stop, standby, speed, pitch reset, status, settings, reboot, presets, braking tests, optional relay tests, diagnostics, errors, factory reset, closed-loop tools, and Wi-Fi setup are available according to the build.
- **Settings registry:** `list`, `get`, and `set` expose registered settings with strict type parsing and range limits.
- **Explicit persistence:** Registry changes remain in RAM until `save` is issued.
- **Preset exchange:** `export preset` prints one-line JSON. `import preset` validates JSON before replacing the selected slot.
- **Closed-loop tools:** Status, health, trend, sensor setup, guided tuning, and base-frequency calibration are present when feedback is compiled.
- **Wi-Fi setup:** Wi-Fi builds provide guided setup, scanning, quick station connection, direct setters, apply, reconnect, and defaults commands.
- **Safety diagnostic:** `diag safety` reports settings and interlock checks without actuating hardware.
- **Bench commands:** Brake and supported linear relay tests are explicit commands and respect motion and critical-fault interlocks.
- **UI lock:** Serial writes are rejected while Device UI Lock is active until the shared PIN is supplied.
- **Input injection:** `j`, `l`, `k`, and `m` inject encoder and menu input for testing. The normal `s`, `t`, `i`, and `f` commands provide speed, standby, status, and factory-reset actions.
- **Status reporting:** Output backend, waveform, resource, thermal, feedback, lock, clipping, and stored-error information is included according to the compiled features.

The complete command and setting registry is in [Serial interface](serial-interface.md).

## Wi-Fi web interface

Wi-Fi-capable Arduino-Pico targets can build a local browser control room and JSON API on port 80. Non-Wi-Fi builds compile the network and web modules out.

### Setup and network management

- **Automatic build detection:** `NETWORK_ENABLE` defaults to on only when the board target defines `PICO_CYW43_SUPPORTED`.
- **Initial access point:** Fresh network settings start `TTControl-Setup` so configuration is reachable before station credentials exist.
- **Setup-only safety:** An open setup access point exposes only network setup pages and APIs. Motor control, general settings, presets, logs, and diagnostics remain blocked.
- **Guided setup:** Browser and Serial Monitor workflows cover station SSID and password, hidden networks, DHCP or static IPv4, hostname, fallback access point, access-point name and password, channel, and standby mode.
- **OLED network menu:** The same settings are grouped into overview, Station, IP/Power, Setup AP, and Web Access pages.
- **Credential editing:** Printable special characters are accepted. Blank browser password fields retain the saved value; removal is a separate action.
- **Credential privacy:** Station passwords, setup access-point passwords, and PINs are write-only and never returned by status, diagnostics, or backup export.
- **Standby modes:** Network standby keeps Wi-Fi available. Eco standby disconnects Wi-Fi until a physical wake.
- **Network storage:** Network configuration uses a separate versioned, CRC-checked LittleFS file with temporary-file promotion and backup fallback.

### Browser pages and controls

- **Dashboard:** Mirrors Standard, Stats, Dim, Scope, CPU, Memory, and Flash views with live telemetry where available.
- **Control:** Provides large start, stop, standby, speed, and pitch controls without the full settings surface.
- **Settings:** Builds global and per-speed controls from the firmware schema and filters them to the compiled features and output backend.
- **Calibrate:** Groups speed frequency, phase, startup kick, braking, and amplitude tuning into task-based steps.
- **Network:** Edits station, access-point, addressing, standby, and access-control settings.
- **Presets:** Loads, saves, renames, clears, compares, imports, and exports motor-tuning slots.
- **Bench:** Combines pre-checks, supported relay tests, brake checks, speed and pitch checks, closed-loop setup and tuning, amplifier state, and a generated bench report.
- **Diagnostics:** Reports firmware and build data, compile-time flags, pins, network state, stored files, output status, resources, and recent browser events.
- **Errors:** Displays and clears the persistent error log.
- **Home page:** Dashboard, Control, Settings, Calibrate, Network, Presets, Bench, Diagnostics, or Errors can be stored as the initial page.
- **78 RPM visibility:** Speed and calibration controls hide 78 RPM when it is disabled. Stale requests are still rejected by firmware.
- **Emergency control:** Current state and emergency stop remain available in the sticky control bar.
- **Output-aware controls:** Bridge builds show driver lifecycle, fault snapshot, phase vectors, headroom and clipping. Linear builds show DMA and waveform status and expose only their compiled relay controls.

### Settings workflow and accessibility

- **Search and help:** Schema settings can be found by label, key, or help text. Help includes purpose, range, units, choices, key name, and relevant safety context.
- **Staged editing:** Settings and network pages track pending changes, support discard, review changes before save, and show validation errors.
- **Apply timing:** Help distinguishes immediate changes from values applied on the next start, stop, or relay transition.
- **Write failures:** Settings, preset, network, runtime-reset, and factory-reset operations report LittleFS failures instead of presenting false success.
- **Themes:** System, Light, Dark, and High Contrast themes are provided.
- **Responsive layout:** The control-room interface uses restrained instrument-style panels, grouped primary and secondary tools, and mobile layouts which keep drive controls usable on narrow screens.
- **Accessibility:** Large controls, visible focus, reduced-motion support, semantic grouping, text status indicators, and labelled validation errors are included. Live refresh avoids replacing a focused form.
- **Telemetry:** Server-Sent Events provide live status with normal JSON refresh retained as a fallback. Chart controls include a grid, legend, and selectable data series.

### Access control and data

- **Read-only guest mode:** Status remains available while changes require PIN unlock.
- **Device UI Lock:** The same PIN protects browser and serial writes and OLED menu entry.
- **Sessions:** Successful unlock creates a random, inactivity-limited session. Repeated failures receive progressively longer delays.
- **Browser write checks:** Mutating requests require same-origin JSON, correct types, bounded bodies, and write-access approval.
- **Security headers:** Framing, referrer, content-type, script, and style restrictions are sent with responses.
- **Closed-loop data:** Dashboard, telemetry, Bench, diagnostics, presets, and backups include the compiled controller state, sensor health, stability metrics, setup state, tuning guidance, and trend data.
- **Amplifier data:** Temperature and thermal state appear in status, telemetry, diagnostics, and Bench when monitoring is compiled.
- **Preset isolation:** Browser preset actions change motor tuning without replacing network, display, relay, runtime, preset-name, or current-speed metadata.
- **Full backup:** Export contains settings, presets, non-secret network metadata, and a copy of the error log. Import restores settings, presets, and non-secret network metadata, but does not restore the exported log.
- **API versioning:** Responses identify API version 1, and settings, network, preference, and control writes reject incorrect JSON types rather than coercing them.

Network setup and access-control details are in [Web interface](web-interface.md).

## Settings and presets

- **Non-volatile settings:** Global and per-speed data is stored in binary LittleFS records.
- **Integrity header:** Settings and preset files contain a file magic, file-format version, settings-schema version, payload size, and CRC32.
- **Schema migration:** Known older layouts are migrated only when version, size, and CRC match the expected structure. Unsupported or damaged payloads are rejected.
- **Known-good rollback:** Explicit saves retain the previous bootable configuration and mark the new file pending. The new settings are confirmed only after Core 1 begins servicing waveform buffers on the next boot.
- **Five presets:** `MAX_PRESET_SLOTS` controls the default five motor-tuning slots.
- **Preset actions:** OLED, Serial Monitor, and web workflows can load, save, rename, clear, import, export, preview, and compare presets as applicable.
- **Preset names:** Names contain up to 16 characters.
- **Preset contents:** Per-speed frequency, limits, phase, gain, filtering, amplitude, startup and closed-loop tuning are stored with global topology, phase count, V/f, ramping, braking and output tuning.
- **Closed-loop preset import:** Current preset JSON stores three per-speed tuning records. Older single-value closed-loop tuning keys are accepted and copied into all three speeds.
- **Preset isolation:** Loading a preset preserves display and input preferences, relay and standby hardware settings, runtime counters, preset names, current speed selection, and network data.
- **Critical confirmations:** Destructive or safety-sensitive actions use confirmation pages before applying the change.
- **Factory reset:** Reset requires confirmation, refuses to run while the motor is moving, forces output off, formats LittleFS, and restores defaults.
- **Safe Mode:** Holding the primary encoder at power-on bypasses saved settings and starts conservative factory values in RAM with zero initial amplitude and Wi-Fi off.
- **Safe Mode persistence:** Settings, presets, runtime, boot confirmation, network configuration, and error logs remain read-only for the complete Safe Mode session.
- **Recovery:** Serial diagnostics remain available in Safe Mode. The OLED exit action reboots and attempts a normal flash load.
- **Flash wear:** Live tuning stays in RAM until saved, and speed-selection writes are deferred. LittleFS wear levelling does not replace the need to avoid repeated saves.

Storage scope and recovery behaviour are in [Settings and presets](settings-and-presets.md).

## Standby and power management

- **Compile-time standby:** `ENABLE_STANDBY` includes or removes standby behaviour and its menu controls. A build without it starts in Stopped.
- **Automatic standby:** A 0-60 minute delay can enter standby while stopped; zero disables it.
- **Auto Boot:** The controller can bypass standby at boot.
- **Auto Start:** The motor can start after boot or wake.
- **Display sleep:** Standby display delays range from immediate sleep to ten minutes.
- **Screensaver:** The display can run Bounce, Matrix, or Lissajous mode instead of sleeping.
- **Bridge standby:** The power-stage enable paths are made inactive while logical standby remains available to the UI.
- **Linear standby:** GP16 controls the external standby relay.
- **Linear mute control:** Mute relays can follow standby or motor start and stop, use active-high or active-low polarity, and wait 0-10 seconds after power-on.
- **Staggered switching:** SPST relay outputs change one stage at a time. DPDT mode uses two relay controls and is incompatible with four-channel support.
- **Relay testing:** Supported linear builds can energise one output stage at a time while waveform output is disabled. Tests are blocked during motion and after a critical fault.
- **Low-power idle:** Standby avoids motor drive and display work according to the selected display and network modes.

## Amplifier monitoring and thermal safety

`AMP_MONITOR_ENABLE` is compile-time only because both monitor inputs must be populated and driven.

- **Temperature input:** GP26 reads a TMP36-style analogue heatsink sensor every 500 ms and converts the result to degrees Celsius.
- **Thermal-chain input:** GP27 is an active-high healthy input sampled on the same interval. A low value enters the critical thermal path when detected.
- **Warning threshold:** The warning defaults to 65°C, records `ERR_AMP_THERMAL`, and re-arms after temperature falls 5°C below the threshold.
- **Shutdown threshold:** The shutdown defaults to 75°C. A shutdown-temperature reading or unhealthy thermal input performs an emergency stop and latches the critical interlock until reboot.
- **Validation:** Warning and shutdown values are limited to 30-120°C with at least 1°C between them.
- **Status:** OLED errors, Serial Monitor, web status, telemetry, diagnostics, and Bench expose temperature and thermal state when compiled.

Pins and limits are listed in [Build and hardware configuration](build-configuration.md).

## Multi-core, DMA and system monitoring

- **Core 0:** Runs input, UI, menus, Serial Monitor, settings, network handlers, relay handling, power-stage sequencing, and the motor state machine.
- **Core 1:** Services waveform buffers and publishes its heartbeat.
- **DMA and PWM:** Hardware maintains carrier timing while Core 1 wakes to refill completed buffers.
- **Aligned start:** Both PWM slices are aligned before enable, and both primary DMA channels are armed together.
- **Atomic waveform settings:** Core 0 publishes pending frequency, amplitude, filter, phase and gain state between buffers so a buffer is generated from one coherent tune.
- **Independent filters:** Per-channel filter history remains on Core 1 with waveform generation.
- **Non-blocking control:** Motor transitions, relay sequencing, bridge wake delays, input handling, and network service use periodic state rather than long blocking delays.
- **Resource monitoring:** Core load, heap, optional PSRAM, sketch flash, and LittleFS use feed OLED and web diagnostics.
- **Waveform health:** Core 0 checks the Core 1 heartbeat and DMA buffer-fill age. A stalled path records `ERR_WAVEFORM_HEALTH`, enters critical stop, and allows watchdog recovery if Core 1 remains unhealthy.

## Error handling and recovery

- **Initialisation checks:** Display, storage, power-stage, sensor, network, and optional subsystem failures are reported through the available surfaces.
- **Range validation:** Loaded, imported, serial, menu, and web settings pass through the relevant range and relationship checks.
- **OLED errors:** Error display can be enabled or disabled, its duration is configurable, and the encoder clears the current message.
- **Persistent log:** Stored entries can be viewed and cleared from OLED, Serial Monitor, and web controls.
- **Critical shutdown:** Critical faults disable waveform output and the compiled hardware interlocks. Start and relay-test paths remain blocked until reboot.
- **Bridge faults:** The hardware disable path is asserted in the GPIO interrupt, followed by normal Core 0 cleanup and a stored fault snapshot.
- **Thermal faults:** Amplifier thermal warnings and shutdowns use `ERR_AMP_THERMAL` with the configured criticality.
- **Closed-loop faults:** Entries can include target and measured RPM, error, correction, signal validity, count, and direction.
- **Waveform faults:** A stale Core 1 heartbeat or buffer-fill age records `ERR_WAVEFORM_HEALTH` before watchdog recovery.
- **Settings rollback:** Failure to confirm a pending saved configuration restores the known-good file and records `ERR_SETTINGS_ROLLBACK`.
- **Reset cause:** Each boot records the best available watchdog, software, run-pin, power-on, brownout, debug, glitch, or unknown reset cause reported by the core.
- **Session identifiers:** A random boot-session identifier groups entries across resets even though millisecond timestamps restart at zero.
- **Filesystem failures:** Menu, serial, and web save paths report failed persistence instead of displaying unconditional success.

## Related documentation

- [Build and hardware configuration](build-configuration.md)
- [Output configuration](output-configuration.md)
- [OLED user interface](user-interface.md)
- [Serial interface](serial-interface.md)
- [Web interface](web-interface.md)
- [Closed-loop control](closed-loop-control.md)
- [Settings and presets](settings-and-presets.md)

[Back to the project README](../readme.md)
