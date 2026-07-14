# Settings and presets

TT Control stores settings and presets in LittleFS. The live configuration remains in RAM while the motor runs; flash is written only by explicit save actions and the existing deferred-save paths.

## Main settings

The main settings file contains global controls, three per-speed records, optional closed-loop tuning, display preferences, relay settings for the linear backend, runtime metadata, and preset names. Network credentials and network options are kept in a separate file and schema.

Binary settings files contain:

- A settings-specific magic value.
- File-format version.
- Settings schema version.
- Payload size.
- CRC32 of the payload.

The current `SETTINGS_SCHEMA_VERSION` is defined in `config.h`. Known older layouts are migrated only when their version, size, and CRC match exactly. An unsupported or invalid payload is rejected rather than interpreted as the current structure.

## Save and boot confirmation

An explicit settings save keeps the previous bootable configuration in `/settings_good.bin` and marks the new settings as pending. On the next boot, the new file is confirmed only after Core 1 has started servicing waveform buffers. If the pending boot fails before confirmation, the following boot restores the known-good file and records `ERR_SETTINGS_ROLLBACK`.

This protects against a validly written but unusable configuration. It does not replace the hardware interlocks described in [Output configuration](output-configuration.md).

## Presets

There are five preset slots by default, controlled by `MAX_PRESET_SLOTS`. A preset can be loaded, saved, renamed, cleared, imported, or exported.

Preset names are limited to 16 characters. Presets contain motor-tuning data, including:

- Per-speed frequency, phase, gain, filters, amplitude, and startup values.
- Global motor topology, phase count, ramping, braking, and output-tuning values.
- Per-speed closed-loop tuning and the global pitch target mode when closed-loop support is compiled.

Loading a preset does not replace:

- Display and input preferences.
- Relay and standby hardware settings.
- Runtime counters.
- Preset names.
- Current speed selection.
- Network settings or credentials.

JSON import accepts the current preset layout. Older single-value closed-loop tuning keys remain accepted and are copied to all three speed records. Import validates the data before replacing a slot.

The relevant OLED actions are listed in [OLED user interface](user-interface.md), and command-line import and export are listed in [Serial interface](serial-interface.md).

## Full backup

Wi-Fi builds can export and import a full JSON backup from the Diagnostics page. Export includes motor settings, presets, non-secret network metadata, and a copy of the error log. Import restores the settings, presets, and non-secret network metadata; it does not restore the exported error log. Station passwords, setup access point passwords, and the shared web PIN are not exported.

Preset import and full-backup import are different operations. A preset is portable motor tuning; a full backup is device recovery data.

## Factory reset

Factory reset requires confirmation and will not run while the motor is moving. The firmware forces output off before formatting LittleFS, then restores defaults.

## Safe Mode

Hold the primary encoder button during power-on to enter Safe Mode. The firmware bypasses flash settings and starts with conservative factory defaults in RAM, zero initial waveform amplitude, no filters, and nominal phase values.

Safe Mode is read-only for the whole boot session:

- Settings and presets are not written or reloaded.
- Runtime counters and boot confirmation are not written.
- Network configuration and error logs are not written.
- Wi-Fi remains off.

Serial diagnostics remain available. Leave Safe Mode through the OLED action, which reboots the controller and attempts a normal flash load.

## Flash wear

LittleFS provides wear levelling, but repeated writes are still avoided. Live tuning changes remain in RAM until saved. Applications using the serial or web interfaces should not call save continuously while adjusting a value.

## Related documentation

- [Features](features.md)
- [Output configuration](output-configuration.md)
- [Serial interface](serial-interface.md)
- [Web interface](web-interface.md)

[Back to the project README](../readme.md)
