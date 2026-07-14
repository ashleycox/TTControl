/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "config.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_SSD1306

#include <Adafruit_SSD1306.h>
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#include <Wire.h>
#endif
#include "display_backend_common.h"

#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
static Adafruit_SSD1306 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &Wire, PIN_DISPLAY_RESET,
                                        DISPLAY_I2C_CLOCK_HZ, 100000UL);
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
static Adafruit_SSD1306 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, PIN_DISPLAY_SPI_MOSI,
                                        PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS);
#else
static Adafruit_SSD1306 hardwareDisplay(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, &DISPLAY_SPI_BUS,
                                        PIN_DISPLAY_DC, PIN_DISPLAY_RESET, PIN_DISPLAY_CS, DISPLAY_SPI_CLOCK_HZ);
#endif

class Ssd1306DisplayBackend : public DisplayBackend {
public:
    bool begin() override {
        if (!beginConfiguredDisplayTransport()) return false;
        bool ready = hardwareDisplay.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS, true,
                                           DISPLAY_TRANSPORT != DISPLAY_TRANSPORT_I2C);
        if (!ready) return false;
        hardwareDisplay.setRotation(DISPLAY_ROTATION);
        return true;
    }

    bool present(const BinaryDisplayFrame& frame) override {
        renderBinaryDisplayFrame(hardwareDisplay, frame, 1, 0);
        hardwareDisplay.display();
        return true;
    }

    void setPower(bool enabled) override {
        hardwareDisplay.ssd1306_command(enabled ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
        if (!enabled) setConfiguredDisplayBacklight(0);
    }

    void setBrightness(uint8_t level) override {
        hardwareDisplay.ssd1306_command(SSD1306_SETCONTRAST);
        hardwareDisplay.ssd1306_command(level);
        setConfiguredDisplayBacklight(level);
    }

    uint16_t width() const override { return hardwareDisplay.width(); }
    uint16_t height() const override { return hardwareDisplay.height(); }
    const char* driverName() const override { return "SSD1306"; }
    DisplayCapabilities capabilities() const override {
        return {true, true, true, false};
    }
};

DisplayBackend& activeDisplayBackend() {
    static Ssd1306DisplayBackend backend;
    return backend;
}

#endif
