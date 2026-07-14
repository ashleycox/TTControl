/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "config.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735 || DISPLAY_DRIVER == DISPLAY_DRIVER_ST7789

#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#include <Adafruit_ST7735.h>
#else
#include <Adafruit_ST7789.h>
#endif
#include "display_backend_common.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
static Adafruit_ST7735 hardwareDisplay(PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_SPI_MOSI,
                                       PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_RESET);
#else
static Adafruit_ST7735 hardwareDisplay(&DISPLAY_SPI_BUS, PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_RESET);
#endif
#else
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_SOFTWARE_SPI
static Adafruit_ST7789 hardwareDisplay(PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_SPI_MOSI,
                                       PIN_DISPLAY_SPI_SCK, PIN_DISPLAY_RESET);
#else
static Adafruit_ST7789 hardwareDisplay(&DISPLAY_SPI_BUS, PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_RESET);
#endif
#endif

class St77xxDisplayBackend : public DisplayBackend {
public:
    bool begin() override {
        if (!beginConfiguredDisplayTransport()) return false;
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
#if DISPLAY_ST7735_PROFILE == DISPLAY_ST7735_PROFILE_GREENTAB
        hardwareDisplay.initR(INITR_GREENTAB);
#elif DISPLAY_ST7735_PROFILE == DISPLAY_ST7735_PROFILE_REDTAB
        hardwareDisplay.initR(INITR_REDTAB);
#elif DISPLAY_ST7735_PROFILE == DISPLAY_ST7735_PROFILE_MINI_160X80
        hardwareDisplay.initR(INITR_MINI160x80);
#elif DISPLAY_ST7735_PROFILE == DISPLAY_ST7735_PROFILE_HALLOWING
        hardwareDisplay.initR(INITR_HALLOWING);
#else
        hardwareDisplay.initR(INITR_BLACKTAB);
#endif
#else
        hardwareDisplay.init(DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT);
#endif
        hardwareDisplay.setSPISpeed(DISPLAY_SPI_CLOCK_HZ);
        hardwareDisplay.invertDisplay(DISPLAY_TFT_INVERT != 0);
        hardwareDisplay.setRotation(DISPLAY_ROTATION);
        return true;
    }

    bool present(const BinaryDisplayFrame& frame) override {
        renderBinaryDisplayFrame(hardwareDisplay, frame,
                                 DISPLAY_TFT_FOREGROUND_COLOR, DISPLAY_TFT_BACKGROUND_COLOR);
        return true;
    }

    void setPower(bool enabled) override {
        hardwareDisplay.enableDisplay(enabled);
        if (!enabled) setConfiguredDisplayBacklight(0);
    }

    void setBrightness(uint8_t level) override {
        setConfiguredDisplayBacklight(level);
    }

    uint16_t width() const override { return hardwareDisplay.width(); }
    uint16_t height() const override { return hardwareDisplay.height(); }
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ST7735
    const char* driverName() const override { return "ST7735"; }
#else
    const char* driverName() const override { return "ST7789"; }
#endif
    DisplayCapabilities capabilities() const override {
        return {true, true, PIN_DISPLAY_BACKLIGHT >= 0, true};
    }
};

DisplayBackend& activeDisplayBackend() {
    static St77xxDisplayBackend backend;
    return backend;
}

#endif
