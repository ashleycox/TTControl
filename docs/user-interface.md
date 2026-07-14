# User interface

TT Control uses a one-bit display surface and a rotary encoder. The default physical panel is a 128x64 SSD1306 OLED. SH1106, SH1107, SSD1327, ST7735, ST7789 and headless builds use the same interface through the [display layer](display.md). The UI draws at the configured logical dimensions. Menu entries are added from the compiled feature set, so an option which does not apply to the selected hardware is omitted rather than shown greyed out.

## Basic operation

| Action | Control | Notes |
| :--- | :--- | :--- |
| Wake from standby | Short press | |
| Start or stop the motor | Short press | A running motor stops on release without waiting for the double-press window. |
| Change speed | Turn | Cycles through the enabled speeds. |
| Change dashboard view | Press and turn | |
| Open the menu | Double press | |
| Enter standby | Hold on the dashboard | |
| Go back | Hold in a submenu | |
| Cancel menu edits | Hold in the Main Menu | Reloads settings from flash. |
| Save and exit | Very long hold in the menu | Saves all pending changes. |

With the optional pitch encoder fitted, turning it adjusts the highlighted value without entering edit mode. In edit mode the primary encoder makes fine adjustments and the pitch encoder makes coarse adjustments.

## Display layouts

| Layout | Default panels | Menu rows | Display details |
| :--- | :--- | ---: | :--- |
| Compact | SSD1306/SH1106 128x64 | 5 | Single-height body text, compact status header and compact diagnostic plots. |
| Medium | SH1107/SSD1327 128x128; rotated ST7735 160x128 | 8 | Card layout for runtime and health data, labelled RPM and a larger XY plot. |
| Large | ST7789 240x240 | 10 | Double-height body text, larger controls and modals, and full-width diagnostic plots. |

All layouts share the same menu tree and safety behaviour. The compact layout is selected at 72 pixels high or less. The large layout requires at least 200x160 pixels. Other supported sizes use the medium layout. The physical backend scales the frame only when the configured logical and physical dimensions differ, as described in [Display architecture and hardware](display.md).

## Dashboard views

- **Standard:** Target or measured speed, motor state, frequency, pitch or ramp deviation, and start or braking progress. Closed-loop builds use measured RPM when the signal is valid and show the lock icon only for a real feedback lock.
- **Stats:** Session and total runtime.
- **Dim:** A low-brightness speed display.
- **Scope:** A Lissajous display using phase A and phase B waveform samples.
- **CPU:** Core 0 and Core 1 utilisation.
- **Memory:** Heap and optional PSRAM use.
- **Flash:** Sketch and LittleFS use.

Stats, CPU, Memory, and Flash can be removed from the dashboard cycle in the Display menu. Standard, Dim, and Scope remain available.

## Output-backend menu differences

| Menu item | 3PWM bridge mode | Linear PWM mode |
| :--- | :--- | :--- |
| **Phase D / Gain D** | Unavailable; bridge output uses phases A-C. | Shown only when four-channel support is compiled and four active phases are selected. |
| **Regen Safe** | Shown under **Output > Transitions**. It permits active braking only when the DC bus can absorb regenerated energy safely. | Omitted; the setting does not control linear-output braking. |
| **Driver Status** | Shows bridge lifecycle, start, and fault information. | Omitted and replaced by **Output Status**. |
| **Output Status** | Omitted and replaced by **Driver Status**. | Shows waveform DMA and sample-rate information. |
| **Rly: ActHi**, **Rly: Stby**, **Rly: S/S**, **Rly: Delay** | Omitted. External mute relays are not supported. Optional phase-enable pins follow the selected active phase count automatically. | Shown only when the corresponding standby or mute-relay hardware is compiled. |
| **Relay Test** | Omitted. | Shown when at least one standby or mute-relay output is compiled. |
| **Auto Stby / Auto Boot** | Shown when standby support is compiled. Standby logically disables the bridge driver. | Shown when standby support is compiled. The standby relay is controlled as part of the transition. |

## Main Menu

- **Exit Safe Mode:** Visible only in Safe Mode. Reboots and attempts a normal settings load.
- **Edit Speed: [33/45/78]:** Selects the speed context used by the tuning pages.
- **Speed Tuning:** Frequency, limits, and filters for the selected speed.
- **Output:** Motor layout, phase and gain trim, transition limits, sweep tool, and output status.
- **Motor Control:** Startup, amplitude, ramping, and braking.
- **Power Control:** Standby and hardware-appropriate relay controls.
- **Display:** Brightness, sleep, screensaver, dashboard, and error display settings.
- **System:** Input direction, pitch, 78 RPM, optional amplifier limits, UI lock, logs, runtime reset, boot speed, and factory reset.
- **Closed Loop:** Visible only when `CLOSED_LOOP_SPEED_ENABLE` is `1`.
- **Network:** Visible only on Wi-Fi builds.
- **Presets:** Load, save, rename, and clear presets.
- **Save & Exit:** Saves changes and returns to the dashboard.
- **Cancel:** Discards changes and reloads settings from flash.

## Speed Tuning

- **Frequency:** Nominal frequency in Hz.
- **Min Freq / Max Freq:** Limits used by pitch control.
- **Filt Type:** None, IIR, or FIR filtering.
- **IIR Alpha:** IIR smoothing factor.
- **FIR Prof:** Gentle, Medium, or Aggressive FIR profile.

## Output

- **Motor Layout:** **Topology** selects Custom, Twin Sync, or 3-Phase; **Phases** selects the active output count; **Reset Tune** loads the topology defaults into the edit buffer.
- **Phase Trim:** **Phase A**, **Phase B**, **Phase C**, and optional **Phase D** edit the per-speed raw offsets. Only active channels are shown. Phase D is unavailable in bridge mode and requires four-channel support in linear mode.
- **Gain Trim:** **Gain A %**, **Gain B %**, **Gain C %**, and optional **Gain D %** set per-speed balance from 50-150%. Only active channels are shown.
- **Transitions:** **Phase Slew** is in degrees/s and **Gain Slew** is in percent/s. Zero applies changes immediately. **Regen Safe** appears only in bridge mode.
- **Sweep Tool:** Opens **Output Sweep**. **Parameter** selects symmetric phase, an active phase, or an active gain; **Minimum**, **Maximum**, and **Speed/s** define the sweep; **Start Sweep** begins it. Press to lock and save the current point.
- **Driver Status / Output Status:** Shows the diagnostics appropriate to the compiled backend.

## Motor Control

### Startup

- **Soft Start:** Amplitude ramp duration from 0.0s to 10.0s.
- **Kick Mult:** Startup kick multiplier.
- **Kick Dur:** Startup kick duration. Shown when the multiplier is greater than 1.
- **Kick Ramp:** Ramp-down from the startup kick. Shown when the multiplier is greater than 1.

### Amplitude

- **Red. Amp %:** Reduced running amplitude from 10-100%.
- **Amp Delay:** Delay before reduced amplitude is applied.
- **V/f Blend%:** Mix between the normal drive amplitude and the V/f scale; zero disables it.
- **V/f LowHz / V/f Low%:** Low-frequency point and amplitude scale.
- **V/f MidHz / V/f Mid%:** Mid-frequency point and amplitude scale.
- **V/f BaseHz:** Frequency at which the curve reaches 100%.
- **Max Amp %:** Global maximum amplitude.
- **Gain A-D %:** Per-speed output balance. Only active and compiled channels are shown.

The curve remains at 100% above its base frequency and follows the absolute commanded frequency in either drive direction.

### Ramping

- **Ramp Type:** Linear or S-curve frequency ramp.
- **SS Curve:** Linear, logarithmic, or exponential soft-start profile. Shown for the linear ramp type.
- **Smooth Sw:** Enables smooth changes between speeds.
- **Sw Ramp:** Speed-change ramp duration.
- **Auto Start:** Starts the motor after boot or wake.

### Braking

- **Brake Mode:** Off, Pulse, Ramp, or SoftStop.
- **Brake Dur:** Braking duration.
- **Brk Pulse:** Pulse gap in Pulse mode.
- **Brk StartF / Brk StopF:** Frequency range used by Ramp mode.
- **Brk Cutoff:** Power cut-off frequency used by SoftStop.
- **Brake Tune:** Guided brake setup with explicit start, stop, and save actions.

**Brake Tune** repeats the mode-specific **Duration**, **Pulse Gap**, **Start Hz**, **Stop Hz**, or **Cutoff Hz** setting as appropriate. **Start Motor** begins a test run, **Brake Stop** applies the selected stop, and **Save Brake** persists the result.

While the motor is in Stopping, local menu edits are frozen and menu entry is blocked. Editing resumes only after the configured braking sequence has completed.

## Power Control

- **Rly: ActHi:** Relay polarity. Linear mode only; shown when standby or mute-relay hardware is compiled.
- **Rly: Stby:** Mutes the outputs in standby. Linear mode only; shown when `ENABLE_MUTE_RELAYS` and `ENABLE_STANDBY` are enabled.
- **Rly: S/S:** Unmutes on start and mutes on stop. Linear mode only; shown when `ENABLE_MUTE_RELAYS` is enabled.
- **Rly: Delay:** Mute-relay power-on delay. Linear mode only; shown when `ENABLE_MUTE_RELAYS` is enabled.
- **Auto Stby:** Automatic standby delay. Shown in either backend when `ENABLE_STANDBY` is enabled.
- **Auto Boot:** Bypasses standby at boot. Shown in either backend when `ENABLE_STANDBY` is enabled.
- **Relay Test:** Turns waveform output off and activates one linear relay stage at a time. It is omitted in bridge mode and when no linear relay output is compiled, and will not run while the motor is moving.

Within **Relay Test**, **Output** selects the active stage, **All Off** de-energises every stage, and **Exit Test** restores normal relay handling.

## Display

- **Brightness:** OLED contrast or a configured TFT backlight level from 0-255. A TFT with a fixed backlight ignores brightness changes.
- **Sleep Dly:** Display sleep delay.
- **Scrn Saver:** Enables the standby screensaver.
- **Saver Mode:** Bounce, Matrix, or Lissajous.
- **Auto Dim:** Delay before the running display dims.
- **Show Runtime:** Controls availability of the Stats dashboard page.
- **Show CPU / Show Memory / Show Flash:** Controls availability of the corresponding dashboard pages.
- **Err Display / Err Dur:** Controls on-screen error messages and their duration.

## System

- **Ver:** Firmware version.
- **Rev Encoder:** Reverses the primary encoder direction.
- **Pitch Step / Pitch Reset:** Pitch adjustment step and reset-on-stop behaviour.
- **Enable 78:** Makes 78 RPM available.
- **Amp Warn C / Amp Shut C:** Amplifier warning and shutdown temperatures when `AMP_MONITOR_ENABLE` is enabled.
- **Boot Speed:** 33, 45, 78, or the last selected speed.
- **UI Lock:** **PIN** edits the shared value; **Save PIN**, **Enable Lock**, **Disable Lock**, and **Lock Now** perform the named actions. When locked, the **UI Locked** page accepts the PIN before menu entry.
- **Error Log:** Views stored errors and provides **Clear Log**.
- **Reset Runtime:** Resets the total runtime after confirmation.
- **Fact Reset:** Restores factory settings after confirmation.

Amplifier monitoring runs automatically when compiled. Warning and shutdown events appear in the error log, Serial Monitor, and web status.

## Closed Loop

This menu is shown only when `CLOSED_LOOP_SPEED_ENABLE` is `1`. See [Closed-loop control](closed-loop-control.md) for the operating and tuning notes.

### Control

- **Target:** Speed context currently being edited.
- **Enable:** Enables or disables feedback.
- **Control:** Monitor or Correct mode.
- **Target RPM:** Per-speed target.
- **Update ms:** Controller update interval.
- **Filter A:** RPM filter alpha.
- **Apply:** Reconfigures feedback without leaving the menu.

### Sensor

- **Sensor:** Pulse tachometer or quadrature.
- **Counts/Rev:** Counts per platter revolution after decoding.
- **Pulse Edge:** Rising, falling, or change counting in pulse mode.
- **Quad Decode:** x1, x2, or x4 quadrature decoding.
- **Reverse Dir:** Reverses the interpreted quadrature direction.
- **Dir Fault:** Ignore, warn, or stop on reverse motion.
- **Debounce us:** Minimum accepted interval between transitions.
- **Timeout ms:** Time without a valid count before the signal is considered lost.

### Engage

- **Engage ms:** Delay after stable running before correction starts.
- **Req Signal:** Requires a valid sensor signal before correction starts.
- **Req Near:** Requires measured RPM near the target before correction starts.
- **Eng Tol:** RPM tolerance used by near-target engagement.

### Tuning

- **Dead RPM:** Controller deadband.
- **Lock Tol / Lock ms:** Tolerance and duration used to declare speed lock.
- **Kp / Ki / Kd:** PID gains.
- **I Lim Hz:** Integral contribution limit.
- **Corr Hz:** Total correction limit.
- **Slew Hz/s:** Correction slew limit.
- **Ramp CL:** Disables correction during a smooth speed change or tracks the live ramp target.
- **Ramp Kp / Ramp Lim:** Gain and limit used while tracking the ramp target.
- **Pitch Mode:** Fixed target or Follow current pitch.
- **Pitch Slew / Pitch Reset:** Target slew and controller-reset threshold for pitch changes.

### Safety

- **Dropout:** Open-loop, hold correction, or stop on signal loss.
- **Sat ms / Sat Act:** Correction-saturation timeout and response.
- **Min RPM / Max RPM / Plaus Act:** Plausible RPM range and response.
- **Lock To / Lock Act:** Lock timeout and response.
- **Amp Rec / Amp Rec ms:** Reduced-amplitude recovery action and delay.

### Tools

- **Apply:** Reconfigures feedback immediately.
- **Reset PID:** Clears controller state and feedback counters.
- **Sensor Test:** Shows live signal and RPM state.
- **Base Preview / Base Apply / Base Save:** Previews, applies, or saves a base-frequency correction derived from stable running.
- **Setup Start / Setup Stat / Setup Apply / Setup Stop:** Captures one platter revolution and applies the suggested sensor configuration.
- **Tune Start / Tune Next / Tune Stat / Tune Apply / Tune Stop:** Runs the guided tuning sequence.

## Network

This menu is available only on Wi-Fi builds. See [Web interface](web-interface.md) for setup and access-control details.

### Network overview

- **Status / Web:** Current connection state and browser address.
- **Wi-Fi:** Enables or disables network services.
- **Mode:** Setup AP, Station, or Station plus setup AP.
- **Station / IP/Power / Setup AP / Web Access:** Opens the corresponding settings page.
- **Apply:** Saves network settings and reconnects.
- **Force AP:** Starts setup access point mode.
- **Refresh:** Rebuilds the page with current status.

### Station

- **SSID:** Station network name.
- **Hidden SSID:** Connects to a network which does not advertise its name.
- **Pass:** Station password.

### IP/Power

- **Host:** mDNS and DHCP hostname.
- **DHCP:** Enables DHCP addressing.
- **AP Fallback:** Starts the setup AP if station connection fails.
- **Standby:** Network keeps Wi-Fi running in standby; Eco turns Wi-Fi off until a physical wake.

### Setup AP

- **AP SSID / AP Pass / AP Channel:** Setup access point name, password, and channel.

### Web Access

- **ReadOnly:** Allows status viewing but requires PIN unlock for changes.
- **Web PIN:** Shared 4-8 character browser and device PIN.
- **Web Home:** Initial browser page.
- **Reset PIN:** Restores `NETWORK_DEFAULT_WEB_PIN`.

## Presets

Selecting a preset slot opens these actions:

- **Load:** Loads the slot into RAM and applies it.
- **Save:** Stores the current motor settings in the slot.
- **Apply Name:** Renames the slot from the edit buffer.
- **Clear:** Deletes the stored slot and restores its default name.

See [Settings and presets](settings-and-presets.md) for persistence and preset scope.

## Related documentation

- [Features](features.md)
- [Output configuration](output-configuration.md)
- [Serial interface](serial-interface.md)
- [Web interface](web-interface.md)
- [Closed-loop control](closed-loop-control.md)

[Back to the project README](../readme.md)
