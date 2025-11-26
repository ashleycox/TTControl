/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include "types.h"
#include <Adafruit_SSD1306.h>

// Forward declarations to avoid circular dependencies
class Settings;
class WaveformGenerator;
class MotorController;
class UserInterface;

// --- Global Object References ---
// Defined in TTControl.ino or respective .cpp files
extern Settings settings;
extern WaveformGenerator waveform;
extern MotorController motor;
extern UserInterface ui;

extern Adafruit_SSD1306 display;

// --- Shared State Variables ---
// Volatile is used for variables shared between Core 0 and Core 1 (or ISRs)
extern volatile MotorState currentMotorState;
extern volatile float currentFrequency;
extern volatile float currentPitchPercent;
extern volatile bool systemInitialized;

#endif // GLOBALS_H
