/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "config.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1106 || DISPLAY_DRIVER == DISPLAY_DRIVER_SH1107

#include <Adafruit_SH110X.h>
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#include <Wire.h>
#endif
#include "display_backend_common.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1106
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
static Adafruit_SH1106G hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &Wire, PIN_DISPLAY_RESET,
                                        DISPLAY_I2C_CLOCK_HZ, 100000UL);
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
static Adafruit_SH1106G hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, PIN_DISPLAY_SPI_MOSI,
                                        PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS);
#else
static Adafruit_SH1106G hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &DISPLAY_SPI_BUS,
                                        PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS, DISPLAY_SPI_CLOCK_HZ);
#endif
#else
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
static Adafruit_SH1107 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &Wire, PIN_DISPLAY_RESET,
                                       DISPLAY_I2C_CLOCK_HZ, 100000UL);
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
static Adafruit_SH1107 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, PIN_DISPLAY_SPI_MOSI,
                                       PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS);
#else
static Adafruit_SH1107 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &DISPLAY_SPI_BUS,
                                       PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS, DISPLAY_SPI_CLOCK_HZ);
#endif
#endif

class Sh110xDisplayBackend : public DisplayBackend {
public:
    bool begin() override {
        if (!beginConfiguredDisplayTransport()) return false;
        if (!hardwareDisplay.begin(DISPLAY_I2C_ADDRESS, true)) return false;
        hardwareDisplay.setRotation(DISPLAY_ROTATION);
        return true;
    }

    bool present(const BinaryDisplayFrame& frame) override {
        renderBinaryDisplayFrame(hardwareDisplay, frame, 1, 0);
        hardwareDisplay.display();
        return true;
    }

    void setPower(bool enabled) override {
        hardwareDisplay.oled_command(enabled ? 0xAF : 0xAE);
        if (!enabled) setConfiguredDisplayBacklight(0);
    }

    void setBrightness(uint8_t level) override {
        hardwareDisplay.setContrast(level);
        setConfiguredDisplayBacklight(level);
    }

    uint16_t width() const override { return hardwareDisplay.width(); }
    uint16_t height() const override { return hardwareDisplay.height(); }
#if DISPLAY_DRIVER == DISPLAY_DRIVER_SH1106
    const char* driverName() const override { return "SH1106"; }
#else
    const char* driverName() const override { return "SH1107"; }
#endif
    DisplayCapabilities capabilities() const override {
        return {true, true, true, false};
    }
};

DisplayBackend& activeDisplayBackend() {
    static Sh110xDisplayBackend backend;
    return backend;
}

#endif
