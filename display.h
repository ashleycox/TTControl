/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Arduino.h>
#include "config.h"
#include "display_backend.h"

// UI drawing is deliberately binary even when the physical backend supports grayscale or colour.
static const uint16_t DISPLAY_BLACK = 0;
static const uint16_t DISPLAY_WHITE = 1;

/*
 * Fixed-storage logical framebuffer. Its native rotated geometry is selected
 * at compile time; GFXcanvas1 keeps UI code and the serial mirror independent
 * of physical framebuffer formats.
 */
class DisplayCanvas : public GFXcanvas1 {
public:
    DisplayCanvas();
    void clearDisplay();

private:
    uint8_t _storage[(DISPLAY_LOGICAL_WIDTH + 7) / 8 * DISPLAY_LOGICAL_HEIGHT];
};

/*
 * Owns display power and brightness policy, refresh throttling, and
 * unchanged-frame suppression. Physical panel access is delegated to the
 * selected backend. All calls are made from Core 0.
 */
class DisplayManager {
public:
    DisplayManager();

    bool begin();
    bool frameDue(bool force = false) const;
    bool present(bool force = false);
    void requestRefresh();

    void setPower(bool enabled);
    void setBrightness(uint8_t brightness);
    void setDimmed(bool dimmed);

    bool isAvailable() const;
    bool isPowered() const;
    uint16_t physicalWidth() const;
    uint16_t physicalHeight() const;
    const char* driverName() const;
    const char* transportName() const;
    const char* wiringProfileName() const;
    DisplayCapabilities capabilities() const;

private:
    bool _available;
    bool _powered;
    bool _dimmed;
    bool _frameInvalid;
    uint8_t _brightness;
    uint16_t _physicalWidth;
    uint16_t _physicalHeight;
    uint32_t _lastFrameMs;
    uint8_t _lastFrame[(DISPLAY_LOGICAL_WIDTH + 7) / 8 * DISPLAY_LOGICAL_HEIGHT];

    void applyBrightness();
};

extern DisplayCanvas display;
extern DisplayManager displayManager;

#endif // DISPLAY_H
