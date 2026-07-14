/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef DISPLAY_BACKEND_COMMON_H
#define DISPLAY_BACKEND_COMMON_H

#include <Adafruit_GFX.h>
#include "config.h"
#include "display_backend.h"

#if DISPLAY_TRANSPORT == DISPLAY_TRANSPORT_HARDWARE_SPI
#include <SPI.h>
#if DISPLAY_SPI_PORT == 1
#define DISPLAY_SPI_BUS SPI1
#else
#define DISPLAY_SPI_BUS SPI
#endif
#endif

bool beginConfiguredDisplayTransport();
void renderBinaryDisplayFrame(Adafruit_GFX& target, const BinaryDisplayFrame& frame,
                              uint16_t foreground, uint16_t background);
void setConfiguredDisplayBacklight(uint8_t level);

#endif // DISPLAY_BACKEND_COMMON_H
