# Web interface

Wi-Fi board targets can host a local control and configuration interface on port 80. `NETWORK_ENABLE` defaults to `1` only when the Arduino-Pico board target defines `PICO_CYW43_SUPPORTED`; network code is omitted from other builds.

## Initial setup

Fresh Wi-Fi builds start the `TTControl-Setup` access point when no station configuration is available. An open setup access point exposes only the network setup pages and APIs. Motor controls, settings, presets, logs, and diagnostics remain unavailable until the device is reached through the configured station network or a password-protected setup access point.

The setup wizard covers:

- Station SSID and password, including hidden networks.
- DHCP or static IPv4 configuration.
- Hostname.
- Setup access point fallback, name, password, and channel.
- Network or Eco standby.

The same settings can be entered through the local Network menu or the [Serial interface](serial-interface.md). Blank browser password fields retain the saved credential; the removal controls clear it explicitly.

## Pages

- **Dashboard:** Mirrors Standard, Stats, Dim, Scope, CPU, Memory, and Flash display modes, with live telemetry where available.
- **Control:** Start, stop, standby, speed, and pitch controls.
- **Settings:** Schema-driven global and per-speed settings, filtered to the compiled feature set and output backend.
- **Calibrate:** Guided frequency, phase, startup, braking, and amplitude tasks.
- **Network:** Station, access point, addressing, standby, and access-control settings.
- **Presets:** Load, save, rename, clear, compare, import, and export.
- **Bench:** Pre-checks, supported relay tests, brake checks, speed and pitch checks, closed-loop setup and tuning, amplifier state, and a bench report.
- **Diagnostics:** Firmware and build information, display driver/transport/wiring profile/geometry/state, feature flags, pin assignments, network state, stored-file state, output status, and recent browser events.
- **Errors:** Stored error log and clear action.

The selected home page is stored on the device. System, Light, Dark, and High Contrast themes are available, together with large controls and reduced-motion support.

## Output-aware controls

The settings schema and Bench page follow the compiled output backend:

- Bridge builds show driver lifecycle, fault snapshot, phase vectors, modulation headroom, and clipping counters.
- Linear builds show waveform and DMA status without inactive bridge fields.
- Relay settings and relay-test controls are available only for linear builds with the corresponding relay hardware.
- Bridge active braking is unavailable until **Regen Safe** confirms that the DC bus has a suitable energy path.

The 78 RPM controls are hidden when 78 RPM is disabled. Requests from an out-of-date page are still checked and rejected by the firmware.

During Stopping, start, standby, speed, pitch, general-settings, and preset-load writes return HTTP 409 until the active braking sequence completes. Stop remains idempotent and does not restart the braking timer.

## Access control

Fresh network defaults enable read-only guest mode and Device UI Lock. Status pages remain available, but changes require the shared 4-8 character PIN.

- Successful unlocks create random, inactivity-limited sessions.
- Repeated failed PIN attempts receive progressively longer delays.
- Mutating requests must be same-origin JSON and pass write-access checks.
- Open setup mode cannot reach motor, preset, log, diagnostic, or general settings routes.
- Password and PIN fields are write-only. Status and diagnostics report whether a credential exists but never return it.
- Security headers restrict framing, referrers, content types, and script or style sources.
- Device UI Lock also protects Serial Monitor writes and local menu entry. Physical stop and standby controls remain available for safety.

## Applying settings

Settings and network forms stage changes and show validation errors before saving. Help text identifies whether a value applies immediately, on the next start or stop, or on the next relay transition. Failed LittleFS writes are reported rather than presented as successful saves.

Preset operations affect motor-tuning fields only. Full backup export includes motor settings, presets, non-secret network metadata, and a copy of the error log. Import restores the settings, presets, and non-secret network metadata, but not the exported error log. Saved passwords and the web PIN are excluded in both directions. See [Settings and presets](settings-and-presets.md).

## Status transport

The dashboard uses a Server-Sent Events stream where available and retains normal status requests for manual refresh and compatibility. Status data can include frequency, pitch, measured RPM, amplifier temperature, output vectors, closed-loop state, and resource use according to the compiled features.

The JSON API identifies version `1` in the `X-TTControl-API-Version` response header. Settings, network, preference, and control writes reject values of the wrong JSON type rather than coercing them.

## Standby networking

- **Network standby:** Wi-Fi remains available while the motor controller is in standby.
- **Eco standby:** Wi-Fi is turned off in standby and reconnects after a physical wake. The browser can request Eco standby, but cannot wake the device once the network has stopped.

Network settings use their own versioned, CRC-checked LittleFS file with temporary-file promotion and backup fallback. They do not alter the binary motor-settings schema.

## Related documentation

- [Features](features.md)
- [User interface](user-interface.md)
- [Serial interface](serial-interface.md)
- [Closed-loop control](closed-loop-control.md)
- [Settings and presets](settings-and-presets.md)

[Back to the project README](../readme.md)
