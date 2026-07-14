/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "config.h"

#if DISPLAY_DRIVER != DISPLAY_DRIVER_NONE

#include "display_backend_common.h"
#include "hal.h"
#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
#include <Wire.h>
#endif

bool beginConfiguredDisplayTransport() {
#if PIN_DISPLAY_BACKLIGHT >= 0
    hal.setPinMode(PIN_DISPLAY_BACKLIGHT, OUTPUT);
    setConfiguredDisplayBacklight(0);
#endif

#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_I2C
    Wire.setSDA(PIN_I2C0_SDA);
    Wire.setSCL(PIN_I2C0_SCL);
    Wire.begin();
    Wire.setClock(DISPLAY_I2C_CLOCK_HZ);
    return true;
#elif DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_HARDWARE_SPI
    return DISPLAY_SPI_BUS.setSCK(PIN_DISPLAY_SPI_SCK) &&
           DISPLAY_SPI_BUS.setTX(PIN_DISPLAY_SPI_MOSI);
#else
    return true;
#endif
}

void renderBinaryDisplayFrame(Adafruit_GFX& target, const BinaryDisplayFrame& frame,
                              uint16_t foreground, uint16_t background) {
    int16_t targetWidth = target.width();
    int16_t targetHeight = target.height();
    int16_t renderWidth = frame.width;
    int16_t renderHeight = frame.height;

#if DISPLAY_SCALE_TO_FIT
    renderWidth = targetWidth;
    renderHeight = (int32_t)renderWidth * frame.height / frame.width;
    if (renderHeight > targetHeight) {
        renderHeight = targetHeight;
        renderWidth = (int32_t)renderHeight * frame.width / frame.height;
    }
#endif

    if (renderWidth < 1) renderWidth = 1;
    if (renderHeight < 1) renderHeight = 1;

    int16_t offsetX = (targetWidth - renderWidth) / 2;
    int16_t offsetY = (targetHeight - renderHeight) / 2;
    bool drawChangesOnly = frame.hasPreviousFrame() && !frame.clearPhysical &&
                           renderWidth >= (int16_t)frame.width &&
                           renderHeight >= (int16_t)frame.height;

    if (frame.clearPhysical) target.fillScreen(background);
    target.startWrite();

    for (uint16_t y = 0; y < frame.height; y++) {
        int16_t destY0 = offsetY + (int32_t)y * renderHeight / frame.height;
        int16_t destY1 = offsetY + (int32_t)(y + 1) * renderHeight / frame.height;
        if (destY1 <= destY0) continue;

        uint16_t x = 0;
        while (x < frame.width) {
            if (drawChangesOnly && frame.getPixel(x, y) == frame.getPreviousPixel(x, y)) {
                x++;
                continue;
            }

            bool foregroundRun = frame.getPixel(x, y);
            uint16_t runStart = x;
            x++;
            while (x < frame.width && frame.getPixel(x, y) == foregroundRun &&
                   (!drawChangesOnly || frame.getPixel(x, y) != frame.getPreviousPixel(x, y))) {
                x++;
            }

            int16_t destX0 = offsetX + (int32_t)runStart * renderWidth / frame.width;
            int16_t destX1 = offsetX + (int32_t)x * renderWidth / frame.width;
            if (destX1 > destX0 && (foregroundRun || !frame.clearPhysical)) {
                target.writeFillRect(destX0, destY0, destX1 - destX0, destY1 - destY0,
                                     foregroundRun ? foreground : background);
            }
        }
    }

    target.endWrite();
}

void setConfiguredDisplayBacklight(uint8_t level) {
#if PIN_DISPLAY_BACKLIGHT >= 0
    uint8_t output = DISPLAY_BACKLIGHT_ACTIVE_HIGH ? level : (uint8_t)(255 - level);
    hal.analogWrite(PIN_DISPLAY_BACKLIGHT, output);
#else
    (void)level;
#endif
}

#endif
