# AGENTS.md

Guidance for coding agents working on TT Control, an Arduino firmware project for an RP2350/Pimoroni PicoPlus2 turntable speed controller.

## Project Snapshot

- Firmware target: Pimoroni PicoPlus2 using the Earle Philhower `rp2040:rp2040` Arduino core.
- Board FQBN: `rp2040:rp2040:pimoroni_pico_plus_2`.
- Main sketch: `TTControl.ino`.
- Core 0 handles UI, input, serial commands, settings, relay control, and the motor state machine.
- Core 1 handles waveform generation buffer work for DDS/PWM/DMA output.
- Persistent settings and presets live in LittleFS as binary structs, with JSON import/export support for presets.
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

As of this guide, the sketch compiles successfully with the current local Pico/RP2040/RP2350 core and libraries.

Do not upload firmware or interact with connected hardware unless explicitly asked. Uploading can energize motor, relay, and mute outputs.

## Key Files

- `TTControl.ino`: global objects, `setup()`/`loop()` for Core 0, `setup1()`/`loop1()` for Core 1.
- `config.h`: firmware version, feature flags, hardware pins, storage schema version, defaults.
- `types.h`: shared enums and settings structs. Changes here can affect binary LittleFS compatibility.
- `globals.*`: shared object declarations and cross-core volatile state.
- `hal.*`: hardware abstraction for GPIO, PWM wrappers, watchdog, timing, semantic relay helpers.
- `waveform.*`: DDS state, LUT generation, PWM setup, DMA setup, ISR handling, sample buffers.
- `motor.*`: high-level motor state machine, speed and pitch control, soft start, braking, relay sequencing, resonance sweep.
- `settings.*`: LittleFS mount/load/save, defaults, validation, schema migration, presets, runtime counters, JSON import/export.
- `input.*`: encoder, button, debounce, acceleration, and serial/test input injection.
- `ui.*`: OLED drawing, dashboard, menu routing, dialogs, screensaver, display-to-serial mirror.
- `menu_system.*` and `menu_data.*`: data-driven menu primitives and actual menu tree.
- `serial_cmd.*`: 115200 baud command interface and setting registry.
- `error_handler.*`: persistent error log and critical error reporting.
- `bitmaps.h`: display bitmap assets.

## Architecture Rules

- Keep the Core 0/Core 1 split intact. UI, menus, settings, and serial work belong on Core 0; waveform timing and DMA/PWM buffer work belong in `WaveformGenerator`.
- Avoid blocking code in `loop()`, `loop1()`, DMA interrupt handlers, and any paths that can delay waveform updates or watchdog feeding.
- Do not perform filesystem, heap-heavy, display, or serial work from timing-critical waveform or ISR paths.
- Treat `currentMotorState`, `currentFrequency`, `currentPitchPercent`, and `systemInitialized` as cross-core shared state. Use the existing APIs instead of adding casual globals.
- Keep direct GPIO/PWM/watchdog access in `hal.*` unless the code must use RP2040/RP2350 hardware APIs directly, as `waveform.*` does for DMA/PWM.
- Preserve safety ranges for frequency, amplitude, phase, relay timing, and braking. These values can drive real hardware.

## Settings And Schema Changes

Settings are persisted as binary `GlobalSettings` data. If you add, remove, reorder, or resize fields in `GlobalSettings` or `SpeedSettings`:

- Bump `SETTINGS_SCHEMA_VERSION` in `config.h`.
- Add a migration path in `settings.cpp` for the previous schema.
- Update `setDefaults()` and `validate()`.
- Update menu bindings in `menu_data.cpp` if the setting is user-facing.
- Update serial registry commands in `serial_cmd.cpp` if the setting should be scriptable.
- Update `readme.md` when user-facing behavior, commands, flags, or menus change.

Avoid repeated flash writes. Use existing deferred-save patterns where live controls change values frequently.

## UI And Menu Work

- The menu system is data-driven. Prefer adding or updating `MenuItem` instances in `menu_data.cpp` instead of hard-coding special UI flows.
- Per-speed edits use `menuShadowSettings` and `menuShadowSpeedIndex`; preserve the Save/Cancel behavior.
- Keep OLED text concise. The display is 128x64, so labels need to fit.
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

## Hardware Safety

- Be conservative with pin assignments in `config.h`; they map to the actual controller wiring.
- Feature flags such as `PITCH_CONTROL_ENABLE`, `ENABLE_STANDBY`, `ENABLE_MUTE_RELAYS`, and `ENABLE_DPDT_RELAYS` alter hardware behavior and menu visibility.
- Relay logic has active-high/active-low support and staggered switching. Preserve sequencing unless the hardware design changes.
- Waveform amplitude is normalized through `setAmplitude()` and constrained by settings. Do not bypass those limits.
- Safe Mode is entered by holding the primary encoder button at boot. It intentionally bypasses flash-loaded settings; do not make it write settings on entry.

## Style

- Match the existing Arduino C++ style: 4-space indentation, braces on function/control lines, concise comments for non-obvious logic.
- Preserve the project header comment in source files.
- Keep memory use modest. Avoid new dynamic allocation in frequently executed code paths.
- Prefer existing enums and structs from `types.h` over magic values in new code.
- Keep changes small and module-local. Do not refactor unrelated firmware areas during a targeted fix.

## Git Hygiene

- Start by checking `git status --short`.
- Do not overwrite or revert unrelated user changes.
- Keep generated build products, local workspace files, and macOS metadata out of commits.
- Make focused commits with a short explanation of hardware or settings implications when relevant.
- Run the compile command before committing firmware changes whenever practical.
