# Closed-loop speed control

Closed-loop speed control is optional and compiled out by default. Set `CLOSED_LOOP_SPEED_ENABLE` to `1` to use a pulse tachometer on GP6 or quadrature feedback on GP6 and GP7. Existing hardware without a sensor is unchanged while the feature remains disabled.

## Sensor modes

### Pulse tachometer

GP6 counts rising, falling, or both signal edges. Configure the accepted edge, counts per platter revolution, debounce interval, and signal timeout.

### Quadrature

GP6 and GP7 are the A and B inputs. x1, x2, and x4 decoding are supported. Direction can be inverted to match the installation, and reverse motion can be ignored, logged as a warning, or treated as a stop fault.

Counts per revolution are measured after the selected edge or quadrature decoding mode.

## Control modes

- **Monitor:** Reports measured RPM, signal health, and lock state without changing output frequency.
- **Correct:** Applies bounded frequency correction against the target RPM.

Each speed has its own target RPM, PID gains, deadband, lock tolerance, lock time, correction limit, correction slew limit, and ramp-tracking values. The controller update interval and RPM filter alpha are global.

## Engagement

Correction remains off during soft start, startup kick, kick ramp-down, and braking. After the motor reaches stable running, engagement can wait for:

- A configured delay.
- A valid sensor signal.
- Measured RPM within the configured distance of the target.

These checks prevent a controller from acting on missing or implausible feedback during startup.

## Speed changes and pitch

During a smooth speed change, feedback can remain open-loop until the ramp finishes or track the live ramp target with a separate proportional gain and correction limit. Controller state is reset when the ramp settles.

Pitch has two target modes:

- **Fixed:** Holds the configured RPM target.
- **Follow:** Applies the effective pitch ratio to the configured target after the same per-speed frequency limits used by the motor output.

Follow is the default. Target slew can soften pitch changes, and a large target jump can reset controller state.

## Reduced-amplitude recovery

If the motor loses lock after reduced amplitude has been applied, the firmware can:

- Take no action.
- Log a warning.
- Restore full amplitude after a configured delay.

## Safety actions

The following conditions have configurable responses:

| Condition | Available responses |
| :--- | :--- |
| Signal dropout | Open-loop, hold the last correction, or stop |
| Correction saturation | Ignore, warn, or stop |
| Implausible RPM | Ignore, warn, or stop |
| Failure to reach lock | Ignore, warn, or stop |
| Reverse quadrature direction | Ignore, warn, or stop |

Closed-loop error records include the target, measured RPM, error, correction, signal state, count, and direction when a feedback sample is available.

## Sensor setup

The OLED, Serial Monitor, and web Bench page can capture one manually turned platter revolution. The result reports:

- Accepted and rejected transitions.
- Captured count.
- Input pin state.
- Detected direction.
- Suggested counts per revolution.
- Suggested quadrature direction reversal where applicable.

Review the result before applying it. A clean manual revolution is more useful than a fast one with contact bounce or missed transitions.

## Guided tuning

The tuning sequence proceeds through sensor validation, monitor-only running, proportional gain, integral gain, correction limits, and verification. At each stage the firmware reports a recommendation based on live stability figures. **Tune Apply** applies only recommendations which the firmware marks as safe.

Start with Monitor mode. Confirm stable counts and believable RPM before allowing correction. Increase Kp until speed error responds without sustained hunting, then add only enough Ki to remove the remaining steady error. Kd is available but is usually best left at zero unless the sensor signal is clean and the mechanical response warrants it.

## Base-frequency calibration

After at least 20 valid samples and 80% lock time, the controller can derive a proposed base-frequency change from the average correction. The change can be previewed, applied in RAM, or applied and saved. This is intended to move normal running closer to zero correction; it is not a substitute for correct sensor scaling.

## Diagnostics

Runtime diagnostics include:

- Valid and locked sample counts and time.
- Average and peak RPM error.
- Correction saturation time.
- Dropout, direction, plausibility, lock-timeout, and amplitude-recovery events.
- Error sign changes.
- Accepted, rejected, and debounced transition counts.
- Minimum, maximum, and average transition interval.
- Interval jitter.
- A rolling trend of target, measured RPM, error, correction, signal, and lock state.

These values are available through the OLED tools, [Serial interface](serial-interface.md), web dashboard and Bench page, diagnostics, status API, preset JSON, and full backup when the feature is compiled.

## Related documentation

The complete OLED menu is listed in [OLED user interface](user-interface.md), and all `cl_*` keys and commands are listed in [Serial interface](serial-interface.md).

- [Features](features.md)
- [Web interface](web-interface.md)
- [Settings and presets](settings-and-presets.md)

[Back to the project README](../readme.md)
