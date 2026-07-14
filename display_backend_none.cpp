/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "config.h"

#if DISPLAY_DRIVER == DISPLAY_DRIVER_NONE

#include "display_backend.h"

class HeadlessDisplayBackend : public DisplayBackend {
public:
    bool begin() override { return true; }
    bool present(const BinaryDisplayFrame& frame) override {
        (void)frame;
        return true;
    }
    void setPower(bool enabled) override { (void)enabled; }
    void setBrightness(uint8_t level) override { (void)level; }

    uint16_t width() const override { return DISPLAY_LOGICAL_WIDTH; }
    uint16_t height() const override { return DISPLAY_LOGICAL_HEIGHT; }
    const char* driverName() const override { return "Headless"; }
    DisplayCapabilities capabilities() const override {
        return {false, false, false, false};
    }
};

DisplayBackend& activeDisplayBackend() {
    static HeadlessDisplayBackend backend;
    return backend;
}

#endif
