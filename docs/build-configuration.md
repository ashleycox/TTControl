# Build and hardware configuration

TT Control targets RP2040 and RP2350 boards using the Earle Philhower Arduino-Pico core. The main supported target is the Pimoroni PicoPlus2.

## Resources

- [RP2350 datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Pimoroni PicoPlus2](https://shop.pimoroni.com/products/pimoroni-pico-plus-2)
- [Raspberry Pi Pico 2](https://www.raspberrypi.com/products/raspberry-pi-pico-2/)
- [Earle Philhower Arduino-Pico core](https://github.com/earlephilhower/arduino-pico)
- [Arduino CLI documentation](https://arduino.github.io/arduino-cli/)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [ArduinoJson](https://arduinojson.org/)

## Requirements

- Arduino-Pico `rp2040:rp2040` core.
- Adafruit SSD1306.
- Adafruit GFX.
- Adafruit BusIO.
- ArduinoJson.
- Arduino-Pico `WiFi`, `WebServer`, and `DNSServer` for Wi-Fi builds.

The display is a 128x64 SSD1306 on I2C0 at address `0x3C`.

## Build targets

Pimoroni PicoPlus2 without Wi-Fi:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 .
```

The default build uses `OUTPUT_STAGE_3PWM_BRIDGE`. To compile the linear backend without editing `config.h`:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DOUTPUT_STAGE_TYPE=0' .
```

Pimoroni PicoPlus2W with an 8MB LittleFS partition:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2w:flash=16777216_8388608 .
```

Raspberry Pi Pico 2 W with a 1MB LittleFS partition:

```sh
arduino-cli compile --fqbn rp2040:rp2040:rpipico2w:flash=4194304_1048576 .
```

Optional non-network paths can be checked together:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DCLOSED_LOOP_SPEED_ENABLE=1 -DAMP_MONITOR_ENABLE=1 -DPITCH_CONTROL_ENABLE=1 -DSTANDBY_BUTTON_ENABLE=1 -DSPEED_BUTTON_ENABLE=1 -DSTART_STOP_BUTTON_ENABLE=1 -DENABLE_4_CHANNEL_SUPPORT=1' .
```

Build `ENABLE_DPDT_RELAYS=1` separately because DPDT and four-channel support are incompatible. Network support requires a Wi-Fi board target which defines `PICO_CYW43_SUPPORTED`.

## Pin assignments

| GP | Bridge build | Linear build | Other use |
| :--- | :--- | :--- | :--- |
| 0 | PWM A / IN1 | PWM phase A | |
| 1 | PWM B / IN2 | PWM phase B | |
| 2 | PWM C / IN3 | PWM phase C | |
| 3 | Unused | PWM phase D | Four-channel support only. |
| 4 | I2C0 SDA | I2C0 SDA | OLED. |
| 5 | I2C0 SCL | I2C0 SCL | OLED. |
| 6 | Speed sensor A | Speed sensor A | Optional pulse or quadrature input. |
| 7 | Speed sensor B | Speed sensor B | Optional quadrature input. |
| 8 | nFAULT | Unused | Active-low by default. |
| 9 | Start/stop button | Start/stop button | Optional; exposed on Pico 2 W. |
| 10 | Main encoder CLK | Main encoder CLK | |
| 11 | Main encoder DT | Main encoder DT | |
| 12 | Main encoder switch | Main encoder switch | |
| 13 | Pitch encoder CLK | Pitch encoder CLK | Optional. |
| 14 | Pitch encoder DT | Pitch encoder DT | Optional. |
| 15 | Pitch encoder switch | Pitch encoder switch | Optional. |
| 16 | Shared bridge enable | Standby relay | |
| 17 | Optional EN1 | Phase A mute or DPDT 1 | |
| 18 | Optional EN2 | Phase B mute or DPDT 2 | |
| 19 | Optional EN3 | Phase C mute | |
| 20 | Unused | Phase D mute | Unused in DPDT mode. |
| 21 | Standby button | Standby button | Optional. |
| 22 | Speed button | Speed button | Optional. |
| 23 | Optional bridge sleep | Unused | |
| 24 | Optional bridge reset | Unused | |
| 26 | Amplifier temperature | Amplifier temperature | Optional analogue input. |
| 27 | Amplifier thermal OK | Amplifier thermal OK | Optional active-high healthy input. |

Pin assignments describe the default configuration. Check [Output configuration](output-configuration.md) before wiring a power stage. Changing a pin can energise the wrong bridge input or relay.

## Main compile-time options

| Name | Default | Purpose |
| :--- | :--- | :--- |
| `OUTPUT_STAGE_TYPE` | `OUTPUT_STAGE_3PWM_BRIDGE` | Selects bridge or linear output semantics. |
| `PWM_CARRIER_FREQUENCY_HZ` | `50000.0f` | PWM carrier target; supported range is 20-100kHz. |
| `LUT_MAX_SIZE` | `16384` | Maximum sine lookup-table size. Must be a power of two. |
| `MIN_OUTPUT_FREQUENCY_HZ` | `10.0f` | Lowest accepted generated frequency. |
| `MAX_OUTPUT_FREQUENCY_HZ` | `1500.0f` | Highest accepted generated frequency. |
| `SERIAL_MONITOR_ENABLE` | `1` | Builds the Serial Monitor interface. |
| `DUPLICATE_DISPLAY_TO_SERIAL` | `0` | Mirrors OLED text to Serial Monitor. |
| `ENABLE_STANDBY` | `1` | Builds standby behaviour and controls. |
| `ENABLE_MUTE_RELAYS` | Linear `1`; bridge `0` | Builds downstream amplifier mute relays. Bridge builds reject `1`. |
| `ENABLE_DPDT_RELAYS` | `0` | Uses two DPDT relays instead of four SPST relays. |
| `ENABLE_4_CHANNEL_SUPPORT` | `0` | Adds the fourth linear waveform channel and related tuning. |
| `AMP_MONITOR_ENABLE` | `0` | Builds amplifier temperature and thermal-cut-out monitoring. |
| `CLOSED_LOOP_SPEED_ENABLE` | `0` | Builds pulse or quadrature speed feedback. |
| `CLOSED_LOOP_TREND_SIZE` | `24` | Number of recent closed-loop samples, from 1-64. |
| `PITCH_CONTROL_ENABLE` | `0` | Builds the secondary pitch encoder. |
| `STANDBY_BUTTON_ENABLE` | `0` | Builds the discrete standby button. |
| `SPEED_BUTTON_ENABLE` | `0` | Builds the discrete speed button. |
| `START_STOP_BUTTON_ENABLE` | `0` | Builds the discrete start/stop button. |
| `NETWORK_ENABLE` | Automatic | Enabled by default only on a Wi-Fi-capable board target. |
| `MAX_PRESET_SLOTS` | `5` | Number of preset slots. |

## Bridge interface options

The complete electrical and sequencing notes are in [Output configuration](output-configuration.md).

| Name | Default | Purpose |
| :--- | :--- | :--- |
| `POWER_STAGE_SHARED_ENABLE` | `1` | Uses the shared enable on GP16. |
| `POWER_STAGE_ENABLE_ACTIVE_HIGH` | `1` | Shared-enable polarity. |
| `POWER_STAGE_FAULT_ENABLE` | `1` | Uses the fault input on GP8. |
| `POWER_STAGE_FAULT_ACTIVE_LOW` | `1` | Fault-input polarity. |
| `POWER_STAGE_PHASE_ENABLES` | `0` | Uses EN1, EN2, and EN3 on GP17-GP19. |
| `POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH` | `1` | Per-phase-enable polarity. |
| `POWER_STAGE_SLEEP_ENABLE` | `0` | Uses the optional sleep signal on GP23. |
| `POWER_STAGE_SLEEP_ACTIVE_HIGH` | `1` | Sleep signal operating polarity. |
| `POWER_STAGE_RESET_ENABLE` | `0` | Uses the optional reset signal on GP24. |
| `POWER_STAGE_RESET_ACTIVE_HIGH` | `0` | Reset assertion polarity. |
| `POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN` | `0` | Uses the supported shared open-drain enable/fault arrangement. |
| `POWER_STAGE_RESET_PULSE_MS` | `2` | Reset pulse duration. |
| `POWER_STAGE_PHASE_ENABLE_DELAY_MS` | `1` | Delay after enabling phase legs. |
| `POWER_STAGE_WAKE_DELAY_MS` | `2` | Delay after waking the stage. |
| `POWER_STAGE_NEUTRAL_BUFFER_COUNT` | `2` | Complete neutral DMA buffers required before enable. |

At least one bridge hardware-disable path is required: shared enable, phase enables, or sleep. Compile-time checks reject missing interlocks, conflicting pins, bridge mute relays, and bridge four-channel output.

## Network defaults

| Name | Default | Purpose |
| :--- | :--- | :--- |
| `NETWORK_CONFIG_VERSION` | `5` | Version of the separate network settings file. |
| `NETWORK_DEFAULT_HOSTNAME` | `"ttcontrol"` | mDNS and DHCP hostname. |
| `NETWORK_DEFAULT_AP_SSID` | `"TTControl-Setup"` | Setup access point name. |
| `NETWORK_DEFAULT_AP_PASSWORD` | Empty | Empty creates an open, setup-only access point. |
| `NETWORK_DEFAULT_AP_CHANNEL` | `6` | Setup access point channel. |
| `NETWORK_DEFAULT_WEB_PIN` | `"1234"` | Initial shared PIN. Change it before using the device on a shared network. |
| `NETWORK_WEB_PIN_MAX` | `8` | Maximum PIN length; the minimum accepted length is four characters. |

See [Web interface](web-interface.md) for setup-mode restrictions and credential handling.

## Storage and thermal constants

| Name | Default | Purpose |
| :--- | :--- | :--- |
| `LITTLEFS_FS_SIZE` | 8MB | Recommended LittleFS partition for the 16MB PicoPlus2. |
| `SETTINGS_SCHEMA_VERSION` | `12` | Current binary motor-settings schema. |
| `SETTINGS_FILE_FORMAT_VERSION` | `1` | Settings wrapper format. |
| `AMP_TEMP_WARN_C` | `65.0f` | Factory amplifier warning temperature. |
| `AMP_TEMP_SHUTDOWN_C` | `75.0f` | Factory amplifier shutdown temperature. |
| `AMP_TEMP_MIN_C` | `30.0f` | Lowest accepted threshold. |
| `AMP_TEMP_MAX_C` | `120.0f` | Highest accepted threshold. |
| `AMP_TEMP_MIN_SHUTDOWN_MARGIN_C` | `1.0f` | Minimum separation between warning and shutdown. |
| `AMP_TEMP_WARN_HYSTERESIS_C` | `5.0f` | Drop required to re-arm the warning. |

When amplifier monitoring is compiled, GP26 reads a TMP36-style analogue sensor every 500 ms and GP27 reads the thermal chain on the same interval. A low thermal-OK input or an over-temperature reading performs a critical stop when detected and latches the interlock until reboot.

## Firmware architecture

Core 0 runs the OLED, input handling, Serial Monitor, network server, settings, relay handling, and motor state machine. Core 1 services waveform buffers. DMA and hardware PWM generate the carrier independently of UI load.

Core 0 monitors the Core 1 heartbeat and DMA buffer age. A stalled waveform path records `ERR_WAVEFORM_HEALTH`, performs the critical stop path, and allows the watchdog to reset the controller if Core 1 remains unhealthy.

Critical errors disable waveform output and the compiled hardware interlocks. Start and relay-test paths remain blocked until the intended reboot recovery.

## Related documentation

- [Features](features.md)
- [Output configuration](output-configuration.md)
- [Settings and presets](settings-and-presets.md)
- [Web interface](web-interface.md)

[Back to the project README](../readme.md)
