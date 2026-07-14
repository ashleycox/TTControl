# Display architecture and hardware

TT Control renders its interface through a display-independent one-bit canvas, sized to the selected panel after rotation. The selected display backend handles transport, panel initialisation, frame transfer, power, contrast and backlight control. Frames are transferred after the time-sensitive Core 0 services have run. Display hardware is selected at compile time in `config.h`; it is not stored in LittleFS.

The default is a 128x64 SSD1306 OLED on I2C0 at address `0x3C`. Each display selection requires its matching Arduino library.

## Architecture

`ui.cpp` draws only to the global `DisplayCanvas`, which derives from [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library). The display code is divided as follows:

| Component | Included implementation |
| :--- | :--- |
| `DisplayCanvas` in `display.h` | Fixed-storage, one-bit logical framebuffer used by all UI drawing. |
| `DisplayManager` in `display.cpp` | Frame-rate limiting, unchanged-frame suppression, power and brightness policy, and dispatch to the selected backend. |
| `DisplayBackend` in `display_backend.h` | Interface for initialisation, frame presentation, power, brightness, physical dimensions, driver name and capability reporting. |
| `display_backend_common.*` | I2C and hardware-SPI setup, aspect-preserving frame mapping, changed-run drawing and optional backlight PWM. |
| `display_backend_none.cpp` | Headless backend. |
| `display_backend_ssd1306.cpp` | SSD1306 backend. |
| `display_backend_sh110x.cpp` | SH1106 and SH1107 backends. |
| `display_backend_ssd1327.cpp` | SSD1327 backend. |
| `display_backend_st77xx.cpp` | ST7735 and ST7789 backends. |

Exactly one backend provides `activeDisplayBackend()` in each build. Backend objects use static storage; display selection does not allocate a backend from the heap. `BinaryDisplayFrame` passes the canvas buffer, previous frame and geometry to the backend without exposing a panel library to the UI.

The canvas and unchanged-frame copy each use one bit per logical pixel: 1,024 bytes each at 128x64, 2,048 bytes each at 128x128, 2,560 bytes each at 160x128, and 7,200 bytes each at 240x240. Buffered OLED libraries allocate their own physical frame buffer as well; TFT controllers hold their colour frame buffer in the panel and are drawn a row-run at a time without a large microcontroller colour framebuffer.

The UI has three native layouts: compact 128x64 panels use a five-row menu; 128x128 and 160x128 panels use taller dashboards, cards and additional menu rows; 240x240 panels use larger typography, controls and diagnostic plots. Safety, sweep, modal, screensaver and serial-mirror layouts all use the active geometry. `DISPLAY_LOGICAL_WIDTH` and `DISPLAY_LOGICAL_HEIGHT` can be overridden for unusual panels; `DISPLAY_SCALE_TO_FIT` then controls how that custom canvas is fitted to the physical panel.

## Supported backends

| `DISPLAY_DRIVER` | Value | Typical panel | Default geometry | Transport | Required library | Example hardware | Controller data |
| :--- | ---: | :--- | :--- | :--- | :--- | :--- | :--- |
| `DISPLAY_DRIVER_NONE` | `0` | No local display | 128x64 build surface | None | Adafruit GFX | Serial/web-only controller | - |
| `DISPLAY_DRIVER_SSD1306` | `1` | Monochrome OLED | 128x64 | I2C, software SPI, hardware SPI | [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) | [Adafruit 0.96-inch 128x64](https://www.adafruit.com/product/326) | [SSD1306 datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf) |
| `DISPLAY_DRIVER_SH1106` | `2` | Monochrome OLED | 128x64 | I2C, software SPI, hardware SPI | [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110x) | [Pimoroni 1.3-inch 128x64 SPI](https://shop.pimoroni.com/products/graphical-oled-display-128x64-1-3-white-spi) | [SH1106G module/controller specification](https://cdn-shop.adafruit.com/product-files/5228/5223-ds.pdf) |
| `DISPLAY_DRIVER_SH1107` | `3` | Monochrome OLED | 128x128 | I2C, software SPI, hardware SPI | [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110x) | [Adafruit 1.12-inch 128x128](https://www.adafruit.com/product/5297) | [SH1107 datasheet](https://cdn-shop.adafruit.com/product-files/5297/SH1107V2.1.pdf) |
| `DISPLAY_DRIVER_SSD1327` | `4` | 16-level greyscale OLED | 128x128 | I2C, software SPI, hardware SPI | [Adafruit SSD1327](https://github.com/adafruit/Adafruit_SSD1327) | [Adafruit 1.5-inch 128x128](https://www.adafruit.com/product/4741) | [SSD1327 datasheet](https://cdn-learn.adafruit.com/assets/assets/000/111/025/original/SSD1327_v1.8.pdf) |
| `DISPLAY_DRIVER_ST7735` | `5` | Colour TFT | 128x160 before rotation | Software SPI, hardware SPI | [Adafruit ST7735/ST7789](https://github.com/adafruit/Adafruit-ST7735-Library) | [Adafruit 1.8-inch 128x160](https://www.adafruit.com/product/358) | [ST7735R datasheet](https://www.adafruit.com/datasheets/ST7735R_V0.2.pdf) |
| `DISPLAY_DRIVER_ST7789` | `6` | Colour IPS TFT | 240x240 | Software SPI, hardware SPI | [Adafruit ST7735/ST7789](https://github.com/adafruit/Adafruit-ST7735-Library) | [Adafruit 1.3-inch 240x240](https://www.adafruit.com/product/4313) | [ST7789V datasheet](https://support.newhavendisplay.com/hc/en-us/article_attachments/4414880310423) |

Greyscale and colour panels render the same high-contrast binary UI as monochrome panels. SSD1327 uses its brightest greyscale level. TFT foreground and background are configurable RGB565 values.

Not every module sold under a controller name has the same geometry, address, reset circuit, colour order, or row/column offsets. Check the module schematic as well as the controller datasheet. In particular, ST7735 modules frequently require a different `DISPLAY_ST7735_PROFILE`, and some ST7789 geometries require physical width, height, rotation, or library offset support beyond the default 240x240 case.

Adafruit SSD1306 is the one supported SPI library here which requires a real chip-select GPIO: it unconditionally configures and toggles the supplied CS pin. An SSD1306 SPI build with `PIN_DISPLAY_CS=-1` is therefore rejected at compile time. Use SSD1306 over I2C, assign a non-conflicting CS pin, or choose an SH110X/SSD1327/TFT module whose Adafruit transport accepts an omitted CS signal.

## Library installation

The default build needs Adafruit GFX, Adafruit BusIO, and Adafruit SSD1306. Install the other driver libraries only for the panels you intend to compile:

```sh
arduino-cli lib install "Adafruit GFX Library" \
  "Adafruit BusIO" \
  "Adafruit SSD1306" \
  "Adafruit SH110X" \
  "Adafruit SSD1327" \
  "Adafruit ST7735 and ST7789 Library"
```

## Selecting a display

Defaults can be changed in `config.h` or supplied as compiler flags. The driver constants are preprocessor values, so command-line examples use their numeric values.

SSD1306 I2C, the normal build:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 .
```

SH1107 128x128 I2C:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=3' .
```

ST7789 240x240 using the recommended managed software-SPI wiring:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=6' .
```

Headless build:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=0' .
```

The selected driver, transport, wiring profile, physical dimensions, availability and power state are printed during startup and by the Serial Monitor `status` command. Web diagnostics on Wi-Fi builds also report whether the backend represents a physical panel and whether it provides power, brightness and colour capabilities.

## I2C wiring

I2C is the preferred option whenever the display controller supports it. It needs only two signal pins and is compatible with every defined controller feature.

| Pico signal | Default GP | Display signal |
| :--- | :--- | :--- |
| I2C0 SDA | GP4 | SDA |
| I2C0 SCL | GP5 | SCL |
| 3V3 | - | VIN/VCC for a 3.3V-compatible module |
| GND | - | GND |

`DISPLAY_I2C_ADDRESS` defaults to `0x3C` for SSD1306/SH1106 and `0x3D` for the example SH1107/SSD1327 panels. `DISPLAY_I2C_CLOCK_HZ` defaults to 400 kHz. Module addresses vary, and some breakouts have fixed pull-ups or require a reset signal. The standard I2C wiring does not assign a display reset pin. Use a module with a suitable power-on reset, or assign `PIN_DISPLAY_RESET` and account for any GPIO conflict.

The Pimoroni PicoPlus2 also exposes GP4/GP5 through its Qw/ST connector, which is convenient for compatible I2C breakouts. A standard Pico/Pico 2 uses the same GPIO through its header.

## SPI wiring and the standard-Pico pin budget

The [Pico 2 datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf) exposes 26 GPIO: GP0-GP22 and GP26-GP28. The most pin-heavy valid bridge build reserves 25 of them before adding SPI D/C: three waveform outputs, every optional bridge interlock, GP4/GP5 for the display bus, feedback, amplifier monitoring, both encoders, and all discrete buttons. GP28 is then the only unassigned pin. A fully featured four-channel linear build reserves 24 and leaves GP8 as well as GP28 free because it has no bridge fault input. A conventional four-wire SPI display usually wants SCK, MOSI, D/C, CS, reset, and sometimes backlight control, so it still cannot have every signal independently controlled in the maximum configurations.

Every standard SPI profile starts with the same three signals:

| Pico signal | GP | Display signal | Note |
| :--- | :--- | :--- | :--- |
| Software SPI clock | GP4 | SCK/CLK | Reuses the display-bus allocation. |
| Software SPI data | GP5 | MOSI/SDA/DIN | Reuses the display-bus allocation. |
| Data/command | GP28 | D/C or A0 | Uses the otherwise spare standard-Pico GPIO. |

`DISPLAY_WIRING_PROFILE` then makes the remaining GPIO trade-off explicit:

| Profile | Value | CS | Reset | Backlight | Optional buttons still available | Appropriate hardware |
| :--- | ---: | :--- | :--- | :--- | :--- | :--- |
| `DISPLAY_WIRING_MINIMAL` | `0` | Tied active | Shared with Pico `RUN` | Fixed on | Speed, standby, start/stop | A dedicated-bus module that tolerates all three fixed signals. |
| `DISPLAY_WIRING_MANAGED` | `1` | GP22 | Shared with Pico `RUN` | GP21 PWM | Start/stop | Recommended TFT wiring: normal CS plus useful dim/sleep control. |
| `DISPLAY_WIRING_FULL_CONTROL` | `2` | GP22 | GP21 | GP9 PWM | None | Modules which genuinely require independent reset as well as CS/backlight. |

The managed profile is the default for SPI displays. It intentionally consumes the GPIOs otherwise assigned to the optional speed and standby buttons; the main encoder still selects speed and standby, and the dedicated start/stop button remains available. The full-control profile also consumes the optional start/stop button. Selecting one of those buttons with a profile which owns its pin produces a direct compile-time error explaining the collision.

The minimal profile preserves every physical control, but it is an electrical promise about the chosen breakout: CS must be safe when permanently asserted on a dedicated bus, display reset must reliably follow board reset or power-on reset, and a TFT backlight will remain electrically on. Do not tie CS active when another device shares SCK/MOSI, and do not connect a bare LED backlight without the specified resistor or driver. Adafruit SSD1306 cannot use the minimal SPI profile because its driver requires a controlled CS pin; use SSD1306 over I2C or choose managed/custom SPI wiring.

Managed ST7789 with the optional start/stop button:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=6 -DSTART_STOP_BUTTON_ENABLE=1' .
```

Minimal ST7789 preserving all three optional buttons:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=6 -DDISPLAY_WIRING_PROFILE=0 -DSTANDBY_BUTTON_ENABLE=1 -DSPEED_BUTTON_ENABLE=1 -DSTART_STOP_BUTTON_ENABLE=1' .
```

For a custom carrier, override `PIN_DISPLAY_CS`, `PIN_DISPLAY_RESET`, or `PIN_DISPLAY_BACKLIGHT`; the actual pin values, not merely the profile name, are checked against the controller pin assignments and active optional features. A full-featured linear build can, for example, place SSD1306 CS on GP8 and retain every defined linear controller feature:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DOUTPUT_STAGE_TYPE=0 -DDISPLAY_TRANSPORT=1 -DPIN_DISPLAY_CS=8 -DPIN_DISPLAY_BACKLIGHT=-1 -DCLOSED_LOOP_SPEED_ENABLE=1 -DAMP_MONITOR_ENABLE=1 -DPITCH_CONTROL_ENABLE=1 -DSTANDBY_BUTTON_ENABLE=1 -DSPEED_BUTTON_ENABLE=1 -DSTART_STOP_BUTTON_ENABLE=1 -DENABLE_4_CHANNEL_SUPPORT=1' .
```

On the maximum bridge configuration there are no further spare standard-header pins after D/C; on the maximum linear configuration there is room for D/C and one additional control. More simultaneous functions require moving or omitting an optional controller feature, or using board-specific RP2350B GPIO. The Pimoroni PicoPlus2 uses an RP2350B internally, but its normal Pico-compatible headers still present the standard pin set, so extra connectors are not assumed portable.

### Hardware SPI

Set `DISPLAY_TRANSPORT=2` for hardware SPI, choose `DISPLAY_SPI_PORT`, and assign pins accepted by the Arduino-Pico core. The core's [SPI documentation](https://arduino-pico.readthedocs.io/en/stable/spi.html) and [pin reassignment documentation](https://arduino-pico.readthedocs.io/en/stable/pins.html) are the authority for other boards.

The standard exposed pairs below are validated at compile time. Set `DISPLAY_VALIDATE_STANDARD_SPI_PINS=0` only for a board-specific RP2350B carrier whose extra pins are known to support the selected SPI function; `DisplayManager::begin()` still asks the Arduino-Pico core to accept the assignment before enabling the display.

The conventional exposed SCK/TX choices on a standard Pico are already owned by some TT Control configuration:

| Port | SCK | MOSI/TX | TT Control use |
| :--- | :--- | :--- | :--- |
| SPI0 | GP2 | GP3 | Waveform phases C/D. |
| SPI0 | GP6 | GP7 | Optional speed feedback A/B. |
| SPI0 | GP18 | GP19 | Linear mute outputs or optional bridge phase enables. |
| SPI1 | GP10 | GP11 | Main encoder CLK/DT. |
| SPI1 | GP14 | GP15 | Optional pitch encoder DT/switch. |
| SPI1 | GP26 | GP27 | Optional amplifier temperature/thermal inputs. |

This means hardware SPI is practical for a reduced feature set, but there is no single standard-Pico hardware-SPI assignment that preserves every defined feature. For example, an ST7789 can use SPI0 on GP6/GP7 when closed-loop feedback is disabled:

```sh
arduino-cli compile --fqbn rp2040:rp2040:pimoroni_pico_plus_2 \
  --build-property 'compiler.cpp.extra_flags=-DDISPLAY_DRIVER=6 -DDISPLAY_TRANSPORT=2 -DDISPLAY_SPI_PORT=0 -DPIN_DISPLAY_SPI_SCK=6 -DPIN_DISPLAY_SPI_MOSI=7' .
```

Enabling `CLOSED_LOOP_SPEED_ENABLE=1` in that build intentionally produces a compile-time pin-collision error. Pin checks also cover D/C and every configured CS, reset, and backlight pin.

## Configuration reference

| Setting | Default | Purpose |
| :--- | :--- | :--- |
| `DISPLAY_DRIVER` | `DISPLAY_DRIVER_SSD1306` | Selects the controller backend. |
| `DISPLAY_TRANSPORT` | I2C for OLEDs; software SPI for TFTs | Selects I2C, software SPI, or hardware SPI. |
| `DISPLAY_WIRING_PROFILE` | `MINIMAL` for I2C; `MANAGED` for SPI | Selects the standard-Pico CS/reset/backlight versus optional-button trade-off. |
| `DISPLAY_I2C_ADDRESS` | `0x3C` or `0x3D` | OLED I2C address; SH1107/SSD1327 default to `0x3D`. |
| `DISPLAY_I2C_CLOCK_HZ` | `400000` | Display I2C bus rate. |
| `DISPLAY_SPI_CLOCK_HZ` | `12000000` | Requested hardware/TFT SPI rate. Software-SPI speed is CPU/library dependent. |
| `DISPLAY_SPI_PORT` | `0` | Arduino-Pico `SPI` or `SPI1` instance for hardware SPI. |
| `DISPLAY_VALIDATE_STANDARD_SPI_PINS` | `1` | Reject hardware-SPI pairs not exposed by a standard Pico/Pico 2 header. |
| `DISPLAY_PHYSICAL_WIDTH`, `DISPLAY_PHYSICAL_HEIGHT` | Driver-specific | Panel constructor dimensions before rotation. |
| `DISPLAY_ROTATION` | ST7735 `1`; others `0` | Adafruit GFX rotation from 0-3. |
| `DISPLAY_LOGICAL_WIDTH`, `DISPLAY_LOGICAL_HEIGHT` | Rotated physical size | Native UI canvas; override only for unusual geometry or scaling tests. |
| `DISPLAY_SCALE_TO_FIT` | `1` | Aspect-fit a custom logical canvas; `0` uses a 1:1 centred viewport. |
| `DISPLAY_MAX_FPS` | Driver/transport-specific | Caps physical refreshes: most small OLEDs use 20, SH1107 I2C and hardware TFT use 10, and SSD1327 I2C or software-SPI TFT use 5. |
| `DISPLAY_DIM_BRIGHTNESS` | `16` | Maximum contrast/backlight level in Dim mode. |
| `DISPLAY_TFT_FOREGROUND_COLOR` | `0xFFFF` | RGB565 logical-white colour. |
| `DISPLAY_TFT_BACKGROUND_COLOR` | `0x0000` | RGB565 logical-black colour. |
| `DISPLAY_TFT_INVERT` | `0` | Applies the panel driver's inversion mode. |
| `DISPLAY_ST7735_PROFILE` | `BLACKTAB` | Selects Adafruit's controller-offset initialisation profile. |
| `PIN_DISPLAY_SPI_SCK`, `PIN_DISPLAY_SPI_MOSI` | GP4, GP5 | Software- or hardware-SPI signals. |
| `PIN_DISPLAY_DC` | GP28 | SPI data/command signal. |
| `PIN_DISPLAY_CS`, `PIN_DISPLAY_RESET`, `PIN_DISPLAY_BACKLIGHT` | Profile-specific | Explicit overrides for controlled signals; negative means not connected. |
| `DISPLAY_BACKLIGHT_ACTIVE_HIGH` | `1` | Backlight PWM polarity when a control pin is present. |

**Brightness** changes OLED controller contrast or the managed/full TFT profile's backlight-control pin. The minimal profile has a fixed backlight, so the controller can stop drawing while the light remains electrically on. Drive a breakout's logic-level backlight input or a correctly rated transistor; do not source an arbitrary backlight directly from a GPIO.

## Refresh and UI behaviour

`ui.update()` handles input, timeouts and display policy. `ui.render()` runs after the motor, feedback, network, web and system-monitor services in the Core 0 loop. `DisplayManager` skips unchanged frames and enforces `DISPLAY_MAX_FPS` before dispatching to the selected backend. At native size or when scaling up, the backend redraws only changed horizontal runs. Buffered OLED libraries then apply their own panel-transfer policy. A forced refresh or downscaled frame redraws the complete viewport.

The display UI provides:

- running, modal, menu, sweep, and critical-fault screens wake the panel;
- the first input from Dim mode wakes the normal dashboard without changing speed;
- `Show Runtime` controls whether the Stats page appears in dashboard cycling;
- menu values are right-aligned and retain a dedicated dirty marker;
- menu row counts and scrolling follow the actual panel height;
- square and large displays use native dashboard cards, larger speed typography, and expanded diagnostic plots;
- confirmation, message, and error text is wrapped within its modal rather than overflowing;
- screensaver bounds use the actual configured standby message width; and
- each frame resets font, colour, wrapping, and size before drawing, preventing state leakage between pages.

The logical display mirror is available when both `DUPLICATE_DISPLAY_TO_SERIAL` and `SERIAL_MONITOR_ENABLE` are `1`, independent of the selected physical controller.

## Practical recommendations

- **Maximum compatibility:** use a 128x64 SSD1306 or SH1106 I2C module on GP4/GP5.
- **Larger monochrome display:** use a 128x128 SH1107 over I2C for a native square dashboard and taller menus without losing any controls.
- **Greyscale OLED:** SSD1327 works, but I2C transfers its 8,192-byte physical buffer and is capped at 5 fps. Prefer SPI when spare pins are available. Adafruit SSD1327 1.0.4 emits deprecation warnings from its bundled splash bitmap with Arduino-Pico 5.5.1; these are dependency warnings, not TT Control diagnostics.
- **Colour and best viewing angle:** use a 240x240 ST7789 with managed software SPI. This uses the large layout and controlled dim/sleep, and retains the primary encoder plus optional start/stop button; speed and standby remain available through the encoder. Use minimal wiring with a suitable module when all discrete buttons are required. Hardware SPI is faster when the chosen feature set leaves a legal pair free.
- **128x160 colour TFT:** use an ST7735 with rotation 1 and the profile matching the module.
- **No local panel:** use the headless backend with Serial Monitor and, on Wi-Fi boards, the web interface. Keep a physical stop control in safety-critical installations.

All Pico GPIO uses 3.3V logic. Use a breakout explicitly compatible with 3.3V signals and check its supply, regulator, level-shifter, backlight, and pull-up requirements before wiring. The controller datasheet describes the IC; the breakout schematic determines what can safely be connected to the Pico.

## Related documentation

- [User interface](user-interface.md)
- [Build and hardware configuration](build-configuration.md)
- [Serial interface](serial-interface.md)
- [Web interface](web-interface.md)
- [Output configuration](output-configuration.md)

[Back to the project README](../readme.md)
