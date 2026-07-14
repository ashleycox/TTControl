# Output configuration

TT Control can drive either a controller-free triple-half-bridge power stage or a filtered linear-amplifier interface. Both use the same DDS waveform engine, motor state machine, per-speed tuning, presets, local-display menus, Serial Monitor commands, and web settings. The compile-time output selection changes the electrical meaning and safety handling of the output pins.

The default build targets a DRV8313/SimpleFOC-style board with three PWM inputs, one shared enable input, and an active-low fault output. The board must not contain another microcontroller driving the same inputs.

## 1. Selecting the output stage

`OUTPUT_STAGE_TYPE` in `config.h` selects the electrical backend:

```cpp
#define OUTPUT_STAGE_TYPE OUTPUT_STAGE_3PWM_BRIDGE
```

This is the default. To build for linear-amplifier hardware, use:

```cpp
#define OUTPUT_STAGE_TYPE OUTPUT_STAGE_LINEAR_PWM
```

The setting can also be overridden without editing `config.h`:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DOUTPUT_STAGE_TYPE=0' .
```

`0` selects `OUTPUT_STAGE_LINEAR_PWM`; `1` selects `OUTPUT_STAGE_3PWM_BRIDGE`.

The output stage is deliberately compile-time only. Selecting the wrong backend can change disabled PWM duty, GPIO ownership, relay behaviour, and the order in which hardware is energised.

## 2. Default three-PWM bridge output

### 2.1. Expected interface

The default bridge configuration expects the following signals:

| TT Control pin | Bridge signal | Default behaviour |
| :--- | :--- | :--- |
| GP0 | PWM A / IN1 | Phase A sine-modulated PWM |
| GP1 | PWM B / IN2 | Phase B sine-modulated PWM |
| GP2 | PWM C / IN3 | Phase C sine-modulated PWM |
| GP16 | EN | Shared active-high bridge enable |
| GP8 | nFAULT | Active-low fault input with Pico pull-up |
| GND | Logic GND | Must be connected to bridge logic ground |

This matches controller-free DRV8313 boards and SimpleFOC-style boards that expose `IN1`, `IN2`, `IN3`, `EN`, and `nFAULT`. If a board also exposes `nSLEEP` or `nRESET`, those signals must be held in their documented operating state by the board or external wiring. They must not be left floating.

The enable line must have a hardware pull resistor that keeps the bridge disabled while the Pico is unpowered or held in reset. Firmware establishes the inactive level at the beginning of `setup()`, but software cannot define a GPIO level before the microcontroller starts.

Do not connect TT Control in parallel with another MCU. If a board contains an onboard controller, electrically isolate that controller from every PWM and enable signal before connecting the Pico.

### 2.2. Exact interface capabilities

Boards differ in the signals they expose, so there are no generic board profiles. Configure the signals wired on the chosen power-stage board. The interface can use a shared enable, independent phase enables, sleep, reset, or the supported shared open-drain enable/fault arrangement:

```cpp
#define POWER_STAGE_SHARED_ENABLE 1
#define POWER_STAGE_PHASE_ENABLES 1
#define POWER_STAGE_SLEEP_ENABLE 0
#define POWER_STAGE_RESET_ENABLE 0
#define POWER_STAGE_ENABLE_FAULT_SHARED_OPEN_DRAIN 0
```

Boards exposing independent `EN1`, `EN2`, and `EN3` use GP17, GP18, and GP19 by default. Sleep and reset default to GP20 and GP3 when enabled. These pins are available on standard Pico 2 W boards and only overlap fourth-channel linear hardware, which is incompatible with the three-phase bridge backend. Each capability has an associated polarity option. At least one real hardware disable path is required in a bridge build: shared enable, phase enables, or sleep. Compile-time pin checks reject conflicting assignments.

The default SimpleFOC-style arrangement is a shared active-high enable plus a separate active-low fault input. Set `POWER_STAGE_SHARED_ENABLE` to `0` for a board that only exposes phase enables. Set `POWER_STAGE_PHASE_ENABLES` to `1` in addition to the shared enable when both are wired; the firmware then uses both interlocks.

When independent phase enables are compiled, the active output count controls their mask: one output enables A, two enable A+B, and three enable A+B+C. Reducing the output count while running disables the removed legs immediately. Increasing it leaves newly required legs disabled until the next normal motor start, so they pass through the neutral-buffer and enable-delay sequence before being connected.

### 2.3. Polarity and timing options

The bridge interface can be adapted in `config.h`:

```cpp
#define POWER_STAGE_ENABLE_ACTIVE_HIGH 1
#define POWER_STAGE_SHARED_ENABLE 1
#define POWER_STAGE_FAULT_ENABLE 1
#define POWER_STAGE_FAULT_ACTIVE_LOW 1
#define POWER_STAGE_PHASE_ENABLES 0
#define POWER_STAGE_PHASE_ENABLE_ACTIVE_HIGH 1
#define POWER_STAGE_SLEEP_ENABLE 0
#define POWER_STAGE_SLEEP_ACTIVE_HIGH 1
#define POWER_STAGE_RESET_ENABLE 0
#define POWER_STAGE_RESET_ACTIVE_HIGH 0
#define POWER_STAGE_RESET_PULSE_MS 2
#define POWER_STAGE_WAKE_DELAY_MS 2
#define POWER_STAGE_PHASE_ENABLE_DELAY_MS 1
#define POWER_STAGE_NEUTRAL_BUFFER_COUNT 2
```

Set `POWER_STAGE_FAULT_ENABLE` to `0` only when the board genuinely has no accessible fault output. This removes an important shutdown signal and is not recommended for finished hardware.

### 2.4. Bridge start, stop, and fault behaviour

At boot the firmware configures the bridge enable output at its inactive level before waveform generation starts. A normal start then:

1. Holds every configured enable path inactive and optionally asserts reset.
2. Publishes zero amplitude and enables DMA waveform output.
3. Releases reset, wakes the stage, and waits for complete neutral PWM buffers.
4. Enables the optional per-phase controls together and observes their settling delay.
5. Enables the bridge globally, or completes the equivalent phase/sleep-only sequence.
6. Begins the configured amplitude ramp and records the lifecycle transition.

At zero amplitude, all active bridge PWM inputs are held at 50% duty. This produces a neutral common-mode command, but it is not treated as a hardware shutdown. The enable signal remains the primary interlock.

An asserted fault immediately drives the enable outputs inactive in the GPIO interrupt path. Core 0 subsequently records `ERR_POWER_STAGE_FAULT`, performs the normal emergency-stop cleanup, and latches the critical interlock until reboot. The boot-session snapshot includes the power-stage and motor states, speed, frequency, waveform-buffer count, phase/gain tune, and per-channel clipping counters. Serial status and the web Driver Status page expose the snapshot and lifecycle counters.

### 2.5. Active braking

Bridge builds default `Regen Safe` to off. Pulse, reverse, ramp, and driven soft-stop behaviours can return energy to the DC bus, while an ordinary DC supply may be unable to absorb it. Enabling this setting confirms that the DC-bus energy path has been verified; it does not select the braking mode. When it is off, stop uses an amplitude ramp-down and then disables the bridge.

Enable active braking only after the complete power system has a verified energy path such as a suitably rated clamp, brake chopper/resistor, regenerative supply, or a demonstrated safe bus-capacitance margin. Firmware disable cannot prevent all energy already flowing through MOSFET body diodes.

## 3. Linear-amplifier output

The linear backend preserves the interface for which TT Control was originally designed and which is used by many existing DIY and commercial controllers:

| TT Control pin | Linear function |
| :--- | :--- |
| GP0-GP3 | PWM waveform channels A-D |
| GP16 | Standby relay |
| GP17-GP20 | Per-channel mute relays, or the configured DPDT relay mapping |

An external reconstruction filter and amplifier convert the PWM duty waveform into the required low-frequency analogue drive. Active samples remain centred on 50% duty, while disabled DMA buffers retain the original zero-duty behaviour. Relay polarity, standby linking, staggered unmute, and DPDT options remain controlled by the existing linear settings and compile-time flags.

The linear backend defaults `ENABLE_MUTE_RELAYS` to `1`. Bridge builds reject `ENABLE_MUTE_RELAYS=1`: the bridge enable is the output interlock, and GP17-GP19 are reserved for optional per-phase enables when the driver board exposes them. Without per-phase enables those pins remain unused rather than becoming relay outputs.

## 4. PWM carrier configuration

The carrier target is configured independently of motor frequency:

```cpp
#define PWM_CARRIER_FREQUENCY_HZ 50000.0f
```

The firmware derives the PWM divider from the actual system clock. RP2350 and RP2040 builds therefore remain at the requested carrier for every supported board clock selection. The supported compile-time range is 20-100 kHz. Motor frequency remains controlled by the DDS phase accumulator.

## 5. Motor topology and tuning

Motor topology is a persisted setting available through the local display, Serial Monitor, presets, and web interface:

| Value | Topology | Nominal phase starting point |
| :--- | :--- | :--- |
| 0 | Custom | Existing raw phase tune |
| 1 | Twin-phase synchronous | A=0°, B=180°, C=270° |
| 2 | Three-phase sine | A=0°, B=120°, C=240° |

The topology setting provides the motor context and reset defaults; it never locks the tuning controls. At every speed the user can adjust:

- Raw A/B/C phase offsets, plus D when four-channel support is compiled.
- Per-channel gain from 50-150%, where 100% is unity.
- Configurable phase and gain slew rates for quiet live adjustment; zero means immediate application.
- Overall and reduced amplitude.
- Frequency, filters, startup kick, and ramp settings.

Phase and gain tuning remain live-adjustable so vibration can be minimised while observing the running motor. User-selectable reverse drive is intentionally not provided; reverse phase progression exists only inside the configured braking implementation.

`Reset Tune` under **Output > Motor Layout** loads the nominal phase map for the selected topology and resets channel gains to 100%. It changes the menu shadow copy only; select **Save & Exit** to persist it. Custom topology leaves phase offsets unchanged when reset, while still resetting channel gains.

The output sweep tool can sweep symmetric phase separation, any active channel's raw phase, or any active channel's gain. In twin-phase synchronous mode the symmetric parameter follows the shared-leg winding relationship. Pressing the encoder locks and saves the current value; other exits restore the complete pre-sweep phase and gain tune.

Useful Serial Monitor keys include:

```text
motor_topology      0=Custom, 1=Twin-phase synchronous, 2=Three-phase
active_braking      0=Coast/ramp down, 1=DC-bus energy path verified
phase_slew          Phase adjustment rate in degrees/sec; 0=immediate
gain_slew           Gain adjustment rate in percent/sec; 0=immediate
phase1..phase4      Per-speed raw phase offsets
gain1..gain4        Per-speed channel gains, 50-150%
```

## 6. Verification before connecting a motor

Do not begin testing at full bus voltage or unrestricted current.

1. Compile the intended backend and confirm it in serial `status` or web diagnostics.
2. Power the Pico without the motor bus and confirm GP16 remains inactive during boot and stop.
3. Confirm all PWM inputs show the configured carrier and 50% duty at zero amplitude in bridge mode.
4. Confirm `nFAULT` immediately disables GP16 and latches the firmware fault.
5. Use a current-limited isolated DC supply at reduced voltage.
6. Verify phase-to-phase voltage, winding current, direction, and enable sequencing using test equipment.
7. Increase bus voltage and amplitude gradually while monitoring bridge and motor temperature.

Uploading firmware or changing output configuration can energise motor wiring. Disconnect or disable the motor supply while flashing firmware to the Pico and during initial GPIO checks.

## Related documentation

- [Features](features.md)
- [Build and hardware configuration](build-configuration.md)
- [User interface](user-interface.md)
- [Settings and presets](settings-and-presets.md)

[Back to the project README](../readme.md)
