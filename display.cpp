/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "display.h"
#include <string.h>

DisplayCanvas display;
DisplayManager displayManager;

DisplayCanvas::DisplayCanvas()
    : GFXcanvas1(DISPLAY_LOGICAL_WIDTH, DISPLAY_LOGICAL_HEIGHT, false) {
    buffer = _storage;
    clearDisplay();
}

void DisplayCanvas::clearDisplay() {
    fillScreen(DISPLAY_BLACK);
}

DisplayManager::DisplayManager() {
    _available = false;
    _powered = true;
    _dimmed = false;
    _frameInvalid = true;
    _brightness = 255;
    _physicalWidth = 0;
    _physicalHeight = 0;
    _lastFrameMs = 0;
    memset(_lastFrame, 0, sizeof(_lastFrame));
}

bool DisplayManager::begin() {
    display.clearDisplay();

    DisplayBackend& backend = activeDisplayBackend();
    DisplayCapabilities backendCapabilities = backend.capabilities();
    bool initialised = backend.begin();

    _available = initialised && backendCapabilities.physicalPanel;
    _physicalWidth = backend.width();
    _physicalHeight = backend.height();
    if (!initialised) return false;

    _powered = true;
    _frameInvalid = true;
    present(true);
    applyBrightness();
    return true;
}

bool DisplayManager::frameDue(bool force) const {
    if (force) return true;
    if (!_powered && capabilities().physicalPanel) return false;
    const uint32_t frameIntervalMs = (1000UL + DISPLAY_MAX_FPS - 1UL) / DISPLAY_MAX_FPS;
    return millis() - _lastFrameMs >= frameIntervalMs;
}

bool DisplayManager::present(bool force) {
    if (!frameDue(force)) return false;
    _lastFrameMs = millis();

    if (!_frameInvalid && !force && memcmp(_lastFrame, display.getBuffer(), sizeof(_lastFrame)) == 0) {
        return false;
    }

    bool clearPhysical = force || _frameInvalid;
    DisplayBackend& backend = activeDisplayBackend();
    if (backend.capabilities().physicalPanel && (!_available || !_powered)) return false;

    BinaryDisplayFrame frame = {
        display.getBuffer(),
        clearPhysical ? nullptr : _lastFrame,
        DISPLAY_LOGICAL_WIDTH,
        DISPLAY_LOGICAL_HEIGHT,
        clearPhysical
    };
    bool presented = backend.present(frame);
    if (presented) {
        memcpy(_lastFrame, display.getBuffer(), sizeof(_lastFrame));
        _frameInvalid = false;
    }
    return presented;
}

void DisplayManager::requestRefresh() {
    _frameInvalid = true;
}

void DisplayManager::setPower(bool enabled) {
    if (_powered == enabled) return;
    _powered = enabled;

    if (_available) activeDisplayBackend().setPower(enabled);

    if (enabled) {
        _frameInvalid = true;
        _lastFrameMs = 0;
        applyBrightness();
    }
}

void DisplayManager::setBrightness(uint8_t brightness) {
    if (_brightness == brightness) return;
    _brightness = brightness;
    applyBrightness();
}

void DisplayManager::setDimmed(bool dimmed) {
    if (_dimmed == dimmed) return;
    _dimmed = dimmed;
    applyBrightness();
}

void DisplayManager::applyBrightness() {
    if (!_available) return;

    uint8_t effective = _brightness;
    if (_dimmed && effective > DISPLAY_DIM_BRIGHTNESS) effective = DISPLAY_DIM_BRIGHTNESS;
    activeDisplayBackend().setBrightness(_powered ? effective : 0);
}

bool DisplayManager::isAvailable() const {
    return _available;
}

bool DisplayManager::isPowered() const {
    return _powered;
}

uint16_t DisplayManager::physicalWidth() const {
    return _physicalWidth;
}

uint16_t DisplayManager::physicalHeight() const {
    return _physicalHeight;
}

const char* DisplayManager::driverName() const {
    return activeDisplayBackend().driverName();
}

const char* DisplayManager::transportName() const {
#if DISPLAY_DRIVER == DISPLAY_DRIVER_NONE
    return "none";
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
    return "I2C";
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_HARDWARE_SPI
    return DISPLAY_SPI_PORT == 1 ? "SPI1" : "SPI0";
#else
    return "software SPI";
#endif
}

const char* DisplayManager::wiringProfileName() const {
#if DISPLAY_DRIVER == DISPLAY_DRIVER_NONE
    return "none";
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
    return "two-wire";
#elif DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_FULL_CONTROL
    return "full-control";
#elif DISPLAY_WIRING_PROFILE == DISPLAY_WIRING_MANAGED
    return "managed";
#else
    return "minimal";
#endif
}

DisplayCapabilities DisplayManager::capabilities() const {
    return activeDisplayBackend().capabilities();
}
