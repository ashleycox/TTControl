# TT Control

TT Control is a configurable turntable motor controller for synchronous AC and BLDC motors. It runs on RP2040 and RP2350 microcontrollers using the Arduino-Pico core. User-interface and motor-control work runs on Core 0, while Core 1 services the timing-critical DMA waveform buffers.

The default [output backend](docs/output-configuration.md) drives a controller-free DRV8313 or SimpleFOC-style three-PWM bridge. A filtered linear-amplifier output remains available as a compile-time option for existing controllers.

Uploading firmware or changing the output configuration can energise the motor, bridge, amplifier, and relay wiring. Disconnect or disable the motor supply while flashing firmware and during initial GPIO checks.

## Features

The [complete feature reference](docs/features.md) covers operating ranges, feature gates, backend differences, safety handling, and the interfaces which expose each facility.

- DMA and hardware-PWM sine generation from 10-1500 Hz.
- 33⅓, 45, and optional 78 RPM operation.
- One, two, or three active outputs, with optional four-channel linear output.
- Custom, twin-phase synchronous, and three-phase motor contexts with per-speed phase and gain tuning.
- Soft start, startup kick, reduced amplitude, V/f shaping, smooth speed changes, and configurable braking.
- Bridge enable, fault, optional phase-enable, sleep, and reset handling, documented in [Output configuration](docs/output-configuration.md).
- Optional [pulse or quadrature closed-loop speed control](docs/closed-loop-control.md).
- Optional amplifier temperature and thermal-cut-out monitoring.
- Five [motor-tuning presets](docs/settings-and-presets.md) with JSON import and export.
- A [local display UI](docs/user-interface.md), [Serial Monitor interface](docs/serial-interface.md), and an optional [local web interface](docs/web-interface.md) on Wi-Fi boards.
- CRC-checked LittleFS settings, known-good rollback, Safe Mode, and a persistent error log, covered in [Settings and presets](docs/settings-and-presets.md).

## Supported hardware

| Component | Standard configuration |
| :--- | :--- |
| Microcontroller | RP2350 primary: [Pimoroni PicoPlus2](https://shop.pimoroni.com/products/pimoroni-pico-plus-2), PicoPlus2W, or [Raspberry Pi Pico 2/Pico 2 W](https://www.raspberrypi.com/products/raspberry-pi-pico-2/). RP2040 compatibility: Raspberry Pi Pico or Pico W. |
| Arduino core | [Earle Philhower Arduino-Pico](https://github.com/earlephilhower/arduino-pico) |
| Display | SSD1306 128x64 OLED on I2C0 at `0x3C` by default; native layouts support SH1106, SH1107, SSD1327, ST7735, ST7789, SPI wiring profiles, and headless builds |
| Main control | Rotary encoder with push switch |
| Default output | Three-PWM bridge with shared enable and active-low fault |
| Alternative output | Up to four channels of filtered PWM for linear amplifiers |
| Storage | LittleFS; 8MB recommended on the 16MB PicoPlus2 |

The controller is built around the [RP2350](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf); RP2040 support is retained as a compatibility target without replacing RP2350 hardware paths. Required libraries and further development resources are linked from [Build and hardware configuration](docs/build-configuration.md#resources).

## Build

The standard non-Wi-Fi Pimoroni PicoPlus2 build is:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2:flash=16777216_8388608 .
```

This selects the three-PWM bridge backend. The linear backend can be selected without editing `config.h`:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DOUTPUT_STAGE_TYPE=0' .
```

Board targets, Wi-Fi builds, pins, feature flags, and optional build checks are listed in [Build and hardware configuration](docs/build-configuration.md).

## Documentation

| Guide | Contents |
| :--- | :--- |
| [Features](docs/features.md) | Complete feature catalogue, operating ranges, optional subsystems, safety behaviour, and diagnostics. |
| [Output configuration](docs/output-configuration.md) | Bridge and linear wiring, enable paths, PWM behaviour, braking safety, phase enables, and commissioning checks. |
| [Display architecture and hardware](docs/display.md) | Display components, supported controllers, libraries, wiring, standard-Pico pin budget, compile options, scaling and diagnostics. |
| [User interface](docs/user-interface.md) | Encoder controls, dashboard views, complete menu reference, and bridge/linear menu differences. |
| [Serial interface](docs/serial-interface.md) | Commands, settings keys, diagnostics, and input injection. |
| [Web interface](docs/web-interface.md) | Network setup, browser pages, access control, backend-aware controls, and status transport. |
| [Closed-loop control](docs/closed-loop-control.md) | Sensor modes, engagement, tuning, safety actions, calibration, and diagnostics. |
| [Build and hardware configuration](docs/build-configuration.md) | Board targets, libraries, pin assignments, compile-time options, and firmware architecture. |
| [Settings and presets](docs/settings-and-presets.md) | LittleFS format, boot confirmation, presets, backup, factory reset, and Safe Mode. |

## Basic operation

| Action | Control |
| :--- | :--- |
| Wake, start, or stop | Short press the main encoder |
| Change speed | Turn the main encoder |
| Change dashboard view | Press and turn |
| Open the Main Menu | Double press |
| Enter standby | Hold on the dashboard |
| Go back or cancel | Hold in the menu |
| Save and exit | Very long hold in the menu |

Settings edited in the menu remain in RAM until **Save & Exit** is selected. Holding from the Main Menu cancels the edit session and reloads the saved configuration.

The complete control map and menu tree are in [User interface](docs/user-interface.md). The [Serial interface](docs/serial-interface.md) runs at 115200 baud when enabled; enter `help` for commands, `status` for the current operating state and display backend, or `list` for registered settings.

## Output selection

`OUTPUT_STAGE_TYPE` in `config.h` selects the electrical backend:

```cpp
#define OUTPUT_STAGE_TYPE OUTPUT_STAGE_3PWM_BRIDGE
```

Use `OUTPUT_STAGE_LINEAR_PWM` for the original filtered linear-amplifier interface. This is a compile-time choice because it changes disabled PWM duty, GPIO ownership, relay availability, and the order in which hardware is energised.

Bridge builds use phases A-C. Optional EN1, EN2, and EN3 signals can be enabled for boards which expose them; the active phase count then selects A, A+B, or A+B+C automatically. External mute relays are not used in bridge builds.

Linear builds can use standby and per-channel mute relays. Four-channel output is available only when `ENABLE_4_CHANNEL_SUPPORT` is enabled.

Read [Output configuration](docs/output-configuration.md) before connecting a power stage.

## Settings and recovery

[Settings and presets](docs/settings-and-presets.md) are stored as versioned, CRC-checked LittleFS records. A newly saved configuration is confirmed only after waveform servicing starts successfully on the next boot. If that boot does not complete, the known-good settings file is restored on the following boot.

Hold the primary encoder during power-on to enter Safe Mode. Safe Mode ignores saved configuration, uses conservative RAM defaults, keeps Wi-Fi off, and prevents flash writes for that boot session. Serial diagnostics remain available when `SERIAL_MONITOR_ENABLE` is `1`.

Factory reset requires confirmation and cannot run while the motor is moving.

## Safety

- Keep a hardware pull resistor on each bridge enable path so the power stage remains disabled while the Pico is unpowered or in reset.
- Do not connect TT Control PWM and enable signals in parallel with another microcontroller.
- Treat 50% bridge PWM as a neutral command, not as a hardware shutdown. The enable path is the interlock.
- Leave **Regen Safe** off unless the complete DC bus has a verified path for returned braking energy.
- Begin commissioning with the motor supply disconnected, then use a current-limited isolated supply at reduced voltage.
- Confirm pin polarity, phase-to-phase voltage, winding current, direction, and fault shutdown before increasing bus voltage or amplitude.
- Critical faults latch the output interlock until reboot.

The backend-specific start, stop, fault, and commissioning procedures are in [Output configuration](docs/output-configuration.md).

## Main source files

| File | Purpose |
| :--- | :--- |
| `TTControl.ino` | Core 0 and Core 1 setup and loops |
| `config.h` | Hardware selection, pins, feature flags, defaults, and schema versions |
| `display.cpp`, `display_backend*` | Native logical canvas, display manager, controller backends, transport, scaling, power and refresh policy |
| `ui.cpp` | Display-independent dashboards, menus, dialogs, and screensavers |
| `motor.cpp` | Motor state machine, ramping, braking, and relay sequencing |
| `waveform.cpp` | DDS, PWM, DMA, waveform buffers, and interrupt handling |
| `power_stage.cpp` | Bridge enable, reset, sleep, phase-enable, and fault handling |
| `menu_data.cpp` | Data-driven local-display menu tree |
| `serial_cmd.cpp` | Serial command and setting registry |
| `settings.cpp` | LittleFS settings, migration, presets, and JSON import/export |
| `network_manager.cpp` | Wi-Fi configuration and connection lifecycle |
| `web_interface.cpp` | Embedded browser interface and JSON APIs |
| `speed_feedback.cpp` | Optional pulse and quadrature feedback |
| `amp_monitor.cpp` | Optional amplifier thermal monitoring |

## Licence

No standalone licence file is included. The source-file notices prohibit commercial use or reproduction without written permission and contractual agreement. External libraries remain governed by their own licences. Publication of this repository does not grant rights beyond those notices; contact the author before reuse.
