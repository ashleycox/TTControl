# Serial interface

When `SERIAL_MONITOR_ENABLE` is `1`, connect at 115200 baud. Enter `help` for commands or `list` for registered settings and their current values. Closed-loop commands and feature-specific setting keys are present only when their feature is compiled. The `wifi` command remains present in non-network builds and reports that Wi-Fi is unavailable.

## Commands

| Command | Description |
| :--- | :--- |
| `help` | List commands available in the compiled build. |
| `start` | Start the motor. |
| `stop` | Stop the motor. |
| `speed <0-2>` | Select 33, 45, or 78 RPM. Speed 2 is rejected when 78 RPM is disabled. |
| `s` | Cycle through enabled speeds. |
| `t` | Toggle standby. |
| `p` | Reset pitch. |
| `status` / `i` | Show motor, waveform, output backend, display driver/transport/wiring profile/geometry/power, resource, and optional monitor status. |
| `list` | List registered settings and values. |
| `set <key> <value>` | Set a registered value. Numbers are parsed strictly and constrained to the setting range. |
| `get <key>` | Read a registered value. |
| `save` | Save current RAM settings to flash. |
| `reboot` | Reboot through the watchdog. |
| `dump settings` | Print a readable settings summary. |
| `lock` | Lock the device controls when Device UI Lock is enabled. |
| `unlock <PIN>` | Unlock local-display, serial, and browser write controls. |
| `ui lock <on\|off>` | Enable or disable Device UI Lock. |
| `ui pin <PIN>` | Set the shared 4-8 character device and browser PIN. |
| `preset list` | List preset slots. |
| `preset load <1-5>` | Load and apply a preset. |
| `preset save <1-5>` | Save current motor settings to a preset. |
| `export preset <1-5>` | Print a preset as minified JSON. |
| `import preset <1-5> <json>` | Validate and store preset JSON. |
| `brake test start` | Start the motor for brake tuning. |
| `brake test stop` | Stop using the configured brake mode. |
| `relay test <0-N>` | Activate one output stage in a supported linear build. |
| `relay test off` | Leave relay test mode. |
| `diag safety` | Run the non-actuating settings and interlock diagnostic. |
| `error dump` | Print the error log. |
| `error clear` | Clear the error log. |
| `f` / `factory reset` | Request factory-reset confirmation. |
| `factory reset confirm` | Erase settings and restore defaults. The motor must be stopped. |

While the motor is in Stopping, start, standby, speed, pitch-reset, preset-load, and registry-setting changes are rejected until braking completes. `stop` remains safe to repeat and does not restart the braking timer.

### Closed-loop commands

These commands are present when `CLOSED_LOOP_SPEED_ENABLE` is `1`.

| Command | Description |
| :--- | :--- |
| `cl help` | List the closed-loop commands present in the build. `cl` alone has the same effect. |
| `cl status` | Show target, measured RPM, correction, lock, direction, and count state. |
| `cl health` | Show accepted and rejected counts, timing, and jitter. |
| `cl trend` | Show recent target, measured RPM, error, correction, signal, and lock samples. |
| `cl reset` | Reset the controller and feedback counters. |
| `cl setup start` | Start a one-revolution sensor capture. |
| `cl setup status` | Show capture state, counts, direction, and suggested counts/rev. |
| `cl setup apply` | Apply the suggested counts/rev and direction setting. |
| `cl setup stop\|cancel` | Cancel sensor capture. |
| `cl tune start` | Start guided tuning. |
| `cl tune next` | Advance to the next tuning step. |
| `cl tune status` | Show the current instruction, recommendation, and stability figures. |
| `cl tune suggest` | Show the current recommendation. |
| `cl tune apply` | Apply the current safe recommendation. |
| `cl tune stop\|cancel` | Stop guided tuning. |
| `cl calibrate preview\|apply\|save` | Preview, apply, or save a base-frequency correction. |

### Wi-Fi commands

These operations are available when network support is enabled. In a non-network build, `wifi` and `wifi help` identify Wi-Fi as unavailable. Passwords are never printed by status or configuration commands.

| Command | Description |
| :--- | :--- |
| `wifi help` | List Wi-Fi commands. `wifi` alone has the same effect. |
| `wifi status` | Show connection state, address, SSID, AP state, MAC address, and RSSI. |
| `wifi config` | Show saved network settings without passwords. |
| `wifi wizard\|setup` | Run guided network setup. |
| `wifi scan` | Scan for nearby networks. |
| `wifi connect <ssid> [password]` | Save station credentials, select Station plus setup AP mode, enable DHCP, and reconnect. |
| `wifi set enabled <on\|off>` | Enable or disable network services. |
| `wifi set mode <ap\|sta\|sta_ap>` | Select setup AP, station, or combined mode. |
| `wifi set standby <network\|eco>` | Select Network or Eco standby. |
| `wifi set ssid <ssid>` | Set the station network name. |
| `wifi set hidden <on\|off>` | Mark the station SSID as hidden or visible. |
| `wifi set password <password>` | Set the station password. |
| `wifi set hostname <name>` | Set the mDNS and DHCP hostname. |
| `wifi set dhcp <on\|off>` | Enable or disable DHCP. |
| `wifi set static <ip> <gateway> <subnet> <dns>` | Set all static IPv4 values and disable DHCP. |
| `wifi set ip\|gateway\|subnet\|dns <address>` | Set one static IPv4 value and disable DHCP. |
| `wifi set fallback <on\|off>` | Control setup access point fallback. |
| `wifi set ap_ssid <ssid>` | Set the setup access point name. |
| `wifi set ap_password <password>` | Set the setup access point password. |
| `wifi set ap_channel <1-13>` | Set the setup access point channel. |
| `wifi set read_only <on\|off>` | Control read-only guest mode. |
| `wifi set device_lock <on\|off>` | Enable or disable Device UI Lock. |
| `wifi set web_pin <PIN>` | Set the shared browser and device PIN. |
| `wifi clear password\|ap_password\|ssid` | Clear the selected saved value. |
| `wifi apply\|save` | Save staged network settings and reconnect. |
| `wifi restart\|reconnect` | Reconnect using the current in-memory network settings. |
| `wifi reset\|defaults` | Restore network defaults and reconnect. |

## Settings registry

Speed-specific keys apply to the currently selected speed.

### Global and motor settings

| Key | Description | Type |
| :--- | :--- | :--- |
| `brightness` | Display contrast or controlled backlight, 0-255 | Integer |
| `ramp` | Soft-start ramp type: 0=Linear, 1=S-curve | Integer |
| `pitch_step` | Pitch adjustment step | Float |
| `rev_enc` | Reverse primary encoder | Boolean |
| `saver_mode` | 0=Bounce, 1=Matrix, 2=Lissajous | Integer |
| `show_cpu` | Shows or hides the CPU dashboard | Boolean |
| `show_memory` | Shows or hides the Memory dashboard | Boolean |
| `show_flash` | Shows or hides the Flash dashboard | Boolean |
| `phase_mode` | Active outputs, 1-3 by default or 1-4 with four-channel support | Integer |
| `motor_topology` | 0=Custom, 1=Twin-phase synchronous, 2=Three-phase sine | Integer |
| `active_braking` | Confirms a verified regenerative energy path; registered only in bridge builds | Boolean |
| `phase_slew` | Live phase adjustment limit in degrees/s; 0 is immediate | Float |
| `gain_slew` | Live gain adjustment limit in percent/s; 0 is immediate | Float |
| `max_amp` | Global maximum amplitude, 0-100% | Integer |
| `vf_blend` | V/f curve blend; 0 disables it | Integer |
| `vf_low_freq` | Low-frequency V/f point in Hz | Float |
| `vf_low_level` | Output level at the low-frequency V/f point, 0-100% | Integer |
| `vf_mid_freq` | Mid-frequency V/f point in Hz | Float |
| `vf_mid_level` | Output level at the mid-frequency V/f point, 0-100% | Integer |
| `vf_base_freq` | Frequency at which the V/f curve reaches 100% | Float |
| `amp_warn` | Amplifier warning temperature when monitoring is compiled | Float |
| `amp_shutdown` | Amplifier shutdown temperature when monitoring is compiled | Float |
| `smooth_switch` | Smooth speed switching | Boolean |
| `switch_ramp` | Speed-change duration in seconds | Integer |
| `brake_mode` | 0=Off, 1=Pulse, 2=Ramp, 3=SoftStop | Integer |
| `brake_duration` | Braking duration in seconds | Float |
| `brake_pulse_gap` | Pulse braking gap in seconds | Float |
| `brake_start_freq` | Ramp braking start frequency | Float |
| `brake_stop_freq` | Ramp braking stop frequency | Float |
| `brake_cutoff` | Soft-stop cut-off frequency | Float |
| `relay_active_high` | Linear relay active polarity; registered only when standby or mute relays are compiled | Boolean |
| `relay_delay` | Linear mute-relay power-on delay; registered only when mute relays are compiled | Integer |

### Current-speed settings

| Key | Description | Type |
| :--- | :--- | :--- |
| `freq` | Base frequency in Hz | Float |
| `phase1`..`phase3` | Raw phase offsets in degrees | Float |
| `gain1`..`gain3` | Channel gain, 50-150%; 100% is unity | Integer |
| `phase4`, `gain4` | Fourth-channel offset and gain; registered only in four-channel linear builds | Float / integer |
| `soft_start` | Soft-start duration | Float |
| `kick` | Startup kick multiplier | Integer |
| `kick_dur` | Startup kick duration | Integer |
| `filter` | 0=None, 1=IIR, 2=FIR | Integer |
| `reduced_amp` | Reduced running amplitude | Integer |
| `amp_delay` | Delay before reduced amplitude | Integer |

### Live settings

| Key | Description | Type |
| :--- | :--- | :--- |
| `pitch` | Current pitch percentage | Float |

### Closed-loop settings

These keys are registered when `CLOSED_LOOP_SPEED_ENABLE` is `1`. Per-speed values apply to the selected speed.

| Key | Description | Type |
| :--- | :--- | :--- |
| `cl_enable` | Enables feedback | Boolean |
| `cl_control` | 0=Monitor, 1=Correct | Integer |
| `cl_mode` | 0=Pulse tachometer, 1=Quadrature | Integer |
| `cl_target_rpm` | Per-speed target RPM | Float |
| `cl_counts` | Counts per platter revolution after decoding | Integer |
| `cl_edge` | 0=Rising, 1=Falling, 2=Change | Integer |
| `cl_quad` | 0=x1, 1=x2, 2=x4 decoding | Integer |
| `cl_reverse` | Reverses quadrature direction | Boolean |
| `cl_dir_fault` | 0=Ignore, 1=Warn, 2=Stop | Integer |
| `cl_debounce_us` | Minimum transition interval in microseconds | Integer |
| `cl_timeout_ms` | Feedback timeout in milliseconds | Integer |
| `cl_engage_ms` | Delay before correction engages | Integer |
| `cl_req_signal` | Requires a valid signal before engagement | Boolean |
| `cl_req_near` | Requires measured RPM near target before engagement | Boolean |
| `cl_engage_tol` | Near-target engagement tolerance | Float |
| `cl_update_ms` | Controller update interval | Integer |
| `cl_filter` | RPM filter alpha | Float |
| `cl_deadband` | Per-speed RPM error deadband | Float |
| `cl_lock_tol` | Per-speed lock tolerance | Float |
| `cl_lock_ms` | Per-speed time within tolerance before lock | Integer |
| `cl_kp` | Per-speed proportional gain | Float |
| `cl_ki` | Per-speed integral gain | Float |
| `cl_kd` | Per-speed derivative gain | Float |
| `cl_i_limit` | Per-speed integral contribution limit | Float |
| `cl_corr_limit` | Per-speed total correction limit | Float |
| `cl_slew` | Per-speed correction slew limit | Float |
| `cl_dropout` | 0=Open-loop, 1=Hold correction, 2=Stop | Integer |
| `cl_ramp_mode` | 0=Disabled, 1=Track ramp target | Integer |
| `cl_ramp_kp` | Per-speed ramp-tracking proportional gain | Float |
| `cl_ramp_limit` | Per-speed ramp-tracking correction limit | Float |
| `cl_pitch_mode` | 0=Fixed target, 1=Follow current pitch | Integer |
| `cl_pitch_slew` | Maximum pitch target change in RPM/s | Float |
| `cl_pitch_reset` | Target jump which resets controller state | Float |
| `cl_sat_ms` | Time at correction limit before action; 0 disables | Integer |
| `cl_sat_action` | 0=Ignore, 1=Warn, 2=Stop | Integer |
| `cl_min_rpm` | Minimum plausible RPM; 0 disables the low check | Float |
| `cl_max_rpm` | Maximum plausible RPM | Float |
| `cl_plaus_action` | 0=Ignore, 1=Warn, 2=Stop | Integer |
| `cl_lock_timeout` | Time allowed to reach lock; 0 disables | Integer |
| `cl_lock_action` | 0=Ignore, 1=Warn, 2=Stop | Integer |
| `cl_amp_recovery` | 0=Off, 1=Warn, 2=Restore full amplitude | Integer |
| `cl_amp_recovery_ms` | Delay before reduced-amplitude recovery | Integer |

## Input injection

The single-character test controls are useful when exercising the interface over Serial Monitor:

| Key | Action |
| :--- | :--- |
| `j` / `l` | Turn the primary encoder left or right. |
| `k` | Press the primary encoder. |
| `m` | Double-press the primary encoder. |
| `s` | Cycle speed. |
| `t` | Toggle standby. |
| `i` | Print status. |
| `f` | Request factory reset. |

## Related documentation

- [Features](features.md)
- [User interface](user-interface.md)
- [Closed-loop control](closed-loop-control.md)
- [Web interface](web-interface.md)
- [Settings and presets](settings-and-presets.md)

[Back to the project README](../readme.md)
