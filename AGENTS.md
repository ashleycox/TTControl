# AGENTS.md

Guidance for coding agents working on TT Control, an Arduino firmware project for an RP2350/Pimoroni PicoPlus2 turntable speed controller.

## Project Snapshot

- Firmware target: Pimoroni PicoPlus2 using the Earle Philhower `rp2040:rp2040` Arduino core.
- Board FQBN: `rp2040:rp2040:pimoroni_pico_plus_2`.
- Main sketch: `TTControl.ino`.
- Core 0 handles UI, input, serial commands, settings, relay control, and the motor state machine.
- Core 1 handles waveform generation buffer work for DDS/PWM/DMA output.
- Persistent settings and presets live in LittleFS as binary structs, with JSON import/export support for presets.
- Optional subsystems include closed-loop tachometer/quadrature feedback, amplifier thermal monitoring, Wi-Fi management, and an embedded web control room. Most are feature-gated in `config.h` and may not compile in the default board configuration.
- This project is under git. Check status before editing and keep changes focused.

## Build And Verification

Use this compile check before handing off firmware changes:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 .
```

The local `.arduino-ci.yml` uses the same board target. Required Arduino libraries include:

- Adafruit SSD1306
- Adafruit GFX
- Adafruit BusIO
- ArduinoJson

Optional display builds require the matching Adafruit SH110X, Adafruit SSD1327, or Adafruit ST7735 and ST7789 library. Display selection, wiring, and compile examples are in `docs/display.md`.

As of this guide, the sketch compiles successfully with the current local Pico/RP2040/RP2350 core and libraries.

When changing feature-gated code, also compile at least one relevant feature combination. For example, the non-network optional paths can be checked with:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DOUTPUT_STAGE_TYPE=0 -DCLOSED_LOOP_SPEED_ENABLE=1 -DAMP_MONITOR_ENABLE=1 -DPITCH_CONTROL_ENABLE=1 -DSTANDBY_BUTTON_ENABLE=1 -DSPEED_BUTTON_ENABLE=1 -DSTART_STOP_BUTTON_ENABLE=1 -DENABLE_4_CHANNEL_SUPPORT=1' .
```

Use a separate build for `ENABLE_DPDT_RELAYS=1` because DPDT and four-channel support are intentionally incompatible. Network code requires a Wi-Fi-capable board/core target; do not bypass the `PICO_CYW43_SUPPORTED` guard just to make a feature build pass.

Do not upload firmware or interact with connected hardware unless explicitly asked. Uploading can energise motor, relay, and mute outputs.

## Key Files

- `TTControl.ino`: global objects, `setup()`/`loop()` for Core 0, `setup1()`/`loop1()` for Core 1.
- `config.h`: firmware version, feature flags, hardware pins, storage schema version, defaults.
- `types.h`: shared enums and settings structs. Changes here can affect binary LittleFS compatibility.
- `globals.*`: shared object declarations and cross-core volatile state.
- `hal.*`: hardware abstraction for GPIO, PWM wrappers, watchdog, timing, semantic relay helpers.
- `waveform.*`: DDS state, LUT generation, PWM setup, DMA setup, ISR handling, sample buffers.
- `motor.*`: high-level motor state machine, speed and pitch control, soft start, braking, relay sequencing, resonance sweep.
- `settings.*`: LittleFS mount/load/save, defaults, validation, schema migration, presets, runtime counters, JSON import/export.
- `speed_feedback.*`: optional tachometer/quadrature ISRs, RPM filtering, signal health, setup capture, and closed-loop diagnostics.
- `amp_monitor.*`: optional amplifier temperature and thermal-cutout monitoring.
- `network_manager.*`: versioned LittleFS network configuration, Wi-Fi/AP lifecycle, device lock, and PIN verification.
- `web_interface.*`: embedded HTML control room, JSON APIs, request-body handling, authentication, settings/preset workflows, and diagnostics.
- `system_monitor.*`: cross-core utilisation plus heap, PSRAM, flash, and LittleFS metrics.
- `input.*`: encoder, button, debounce, acceleration, and serial/test input injection.
- `display.*`: native logical canvas, display manager, power policy, and refresh policy.
- `display_backend*`: compile-time-selected controller backends, transport setup, frame mapping, scaling, and backlight control.
- `ui.*`: display-independent drawing, dashboard, menu routing, dialogs, screensaver, display-to-serial mirror.
- `menu_system.*` and `menu_data.*`: data-driven menu primitives and actual menu tree.
- `serial_cmd.*`: 115200 baud command interface and setting registry.
- `error_handler.*`: persistent error log and critical error reporting.
- `bitmaps.h`: display bitmap assets.

## Architecture Rules

- Keep the Core 0/Core 1 split intact. UI, menus, settings, and serial work belong on Core 0; waveform timing and DMA/PWM buffer work belong in `WaveformGenerator`.
- Avoid blocking code in `loop()`, `loop1()`, DMA interrupt handlers, and any paths that can delay waveform updates or watchdog feeding.
- Do not perform filesystem, heap-heavy, display, or serial work from timing-critical waveform or ISR paths.
- Treat `currentMotorState`, `currentFrequency`, `currentPitchPercent`, and `systemInitialized` as cross-core shared state. Use the existing APIs instead of adding casual globals.
- `volatile` alone is not a general cross-core lock. Preserve the atomic state publication in `WaveformGenerator` and the interrupt-guarded snapshots in `SpeedFeedback`.
- Keep direct GPIO/PWM/watchdog access in `hal.*` unless the code must use RP2040/RP2350 hardware APIs directly, as `waveform.*` does for DMA/PWM.
- Keep UI drawing against `DisplayCanvas`. Controller library calls belong in the matching `DisplayBackend`; `DisplayManager` owns controller-independent refresh, power, and brightness policy.
- Preserve safety ranges for frequency, amplitude, phase, relay timing, and braking. These values can drive real hardware.
- Treat `STATE_STOPPING` as an active motion state. Idle, standby, display, network, and UI changes must not silently interrupt a configured braking sequence unless they are explicit emergency-stop paths.
- Critical faults must leave drive outputs interlocked off. Any new start, relay-test, web, serial, or menu path must respect the latched critical state until the intended recovery action occurs.

## Settings And Schema Changes

Settings are persisted as binary `GlobalSettings` data. If you add, remove, reorder, or resize fields in `GlobalSettings`, `SpeedSettings`, or `ClosedLoopSpeedTuning`:

- Bump `SETTINGS_SCHEMA_VERSION` in `config.h`.
- Add a migration path in `settings.cpp` for the previous schema.
- Update `setDefaults()` and `validate()`.
- Update menu bindings in `menu_data.cpp` if the setting is user-facing.
- Update serial registry commands in `serial_cmd.cpp` if the setting should be scriptable.
- Update web schema, GET/POST mapping, validation, backup/preset mapping, and status output in `web_interface.cpp` when applicable.
- Update preset JSON import/export in `settings.cpp` when the setting belongs in motor presets.
- Update the storage-size constants and `static_assert`s in `config.h`/`types.h`.
- Update `readme.md` when user-facing behaviour, commands, flags, or menus change.

Avoid repeated flash writes. Use existing deferred-save patterns where live controls change values frequently.

## UI And Menu Work

- The menu system is data-driven. Prefer adding or updating `MenuItem` instances in `menu_data.cpp` instead of hard-coding special UI flows.
- Per-speed edits use `menuShadowSettings` and `menuShadowSpeedIndex`; preserve the Save/Cancel behaviour.
- Menu Save/Cancel and preset actions must surface filesystem failures rather than displaying unconditional success.
- Keep local-display text concise. Layouts are responsive, but labels still need to fit the compact 128x64 target.
- For serial-testable UI changes, use the existing injection commands: `j`, `l`, `k`, and `m`.

## Serial CLI

The serial monitor runs at 115200 baud when `SERIAL_MONITOR_ENABLE` is `1`. Useful commands:

- `help`
- `status` or `i`
- `list`
- `set <key> <value>`
- `get <key>`
- `start`, `stop`, `speed <0-2>`, `s`, `t`, `p`
- `export preset <1-5>`
- `import preset <1-5> <json>`
- `error dump`, `error clear`

When adding a setting that should be remotely controllable, register it in `initCLI()` in `serial_cmd.cpp`.

## Network And Web Control

- All mutating web routes must reject cross-origin browser writes, reject access from an open setup AP unless the operation is explicitly part of setup, and call `requireWriteAccess()` when web/device locking applies.
- Open setup mode should expose only the minimum network configuration needed to make the unit reachable. Never expose motor control, presets, logs, diagnostics, saved passwords, or the web PIN there.
- Keep request bodies bounded and release temporary body buffers on every response/error path. Avoid large `String` or `JsonDocument` allocations in high-frequency status endpoints.
- Password and PIN fields are write-only: GET/status/diagnostic responses may report whether a credential is set but must not return its value.
- Network settings use their own versioned, CRC-checked binary format. Changes to `NetworkConfig` require a `NETWORK_CONFIG_VERSION` bump, migration handling, string termination, validation, and web/serial updates.
- The web server runs on Core 0. Handlers must stay short and non-blocking so motor transitions, watchdog health checks, and UI input continue to run.

## Hardware Safety

- Be conservative with pin assignments in `config.h`; they map to the actual controller wiring.
- Feature flags such as `PITCH_CONTROL_ENABLE`, `ENABLE_STANDBY`, `ENABLE_MUTE_RELAYS`, and `ENABLE_DPDT_RELAYS` alter hardware behaviour and menu visibility.
- Relay logic has active-high/active-low support and staggered switching. Preserve sequencing unless the hardware design changes.
- Waveform amplitude is normalised through `setAmplitude()` and constrained by settings. Do not bypass those limits.
- Safe Mode is entered by holding the primary encoder button at boot. Its RAM defaults and flash bypass must remain in force for the whole boot session: menu Cancel, presets, web, serial, network startup, and deferred saves must not reload or write persistent configuration unless an explicit recovery workflow is designed and confirmed.

## Style

- Match the existing Arduino C++ style: 4-space indentation, braces on function/control lines, concise comments for non-obvious logic.
- Preserve the project header comment in source files.
- Keep memory use modest. Avoid new dynamic allocation in frequently executed code paths.
- Prefer existing enums and structs from `types.h` over magic values in new code.
- Keep changes small and module-local. Do not refactor unrelated firmware areas during a targeted fix.

## Documentation

- Document only behaviour, hardware, interfaces, commands and constraints present in the code being delivered.
- Do not add roadmaps, proposed features, speculative extension guidance, aspirational statements or descriptions of work that is not implemented.
- State limitations directly. Use `not supported` rather than wording which implies that an absent feature is planned.
- Use direct, specific technical prose. Remove generic introductions, conclusions, filler, marketing language, conversational asides and automated-sounding summaries.
- Use UK English in prose, including `behaviour`, `colour`, `initialise` and `centre`. Preserve source identifiers, protocol names, upstream project names and verbatim text.
- Link to the libraries, hardware documentation and component datasheets used by the documented feature where those links aid configuration or wiring.

## Git Hygiene

- Start by checking `git status --short`.
- Do not overwrite or revert unrelated user changes.
- Keep generated build products, local workspace files, and macOS metadata out of commits.
- Make focused commits with a short explanation of hardware or settings implications when relevant.
- Run the compile command before committing firmware changes whenever practical.
