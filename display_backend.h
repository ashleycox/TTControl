/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef DISPLAY_BACKEND_H
#define DISPLAY_BACKEND_H

#include <Arduino.h>

struct BinaryDisplayFrame {
    const uint8_t* pixels;
    const uint8_t* previousPixels;
    uint16_t width;
    uint16_t height;
    bool clearPhysical;

    bool getPixel(uint16_t x, uint16_t y) const {
        return getBufferPixel(pixels, x, y);
    }

    bool getPreviousPixel(uint16_t x, uint16_t y) const {
        return getBufferPixel(previousPixels, x, y);
    }

    bool hasPreviousFrame() const {
        return previousPixels != nullptr;
    }

private:
    bool getBufferPixel(const uint8_t* source, uint16_t x, uint16_t y) const {
        if (!source || x >= width || y >= height) return false;
        size_t stride = (width + 7U) / 8U;
        return (source[(x / 8U) + y * stride] & (0x80U >> (x & 7U))) != 0;
    }
};

struct DisplayCapabilities {
    bool physicalPanel;
    bool powerControl;
    bool brightnessControl;
    bool colourPanel;
};

class DisplayBackend {
public:
    virtual ~DisplayBackend() = default;

    virtual bool begin() = 0;
    virtual bool present(const BinaryDisplayFrame& frame) = 0;
    virtual void setPower(bool enabled) = 0;
    virtual void setBrightness(uint8_t level) = 0;

    virtual uint16_t width() const = 0;
    virtual uint16_t height() const = 0;
    virtual const char* driverName() const = 0;
    virtual DisplayCapabilities capabilities() const = 0;
};

// Exactly one backend implementation provides this function for each DISPLAY_DRIVER selection.
DisplayBackend& activeDisplayBackend();

#endif // DISPLAY_BACKEND_H
