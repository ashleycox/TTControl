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

// Forward declarations keep every module from needing the full class headers.
// Include the concrete header in .cpp files when member functions are used.
class Settings;
class WaveformGenerator;
class MotorController;
class UserInterface;

// --- Global Object References ---
// Defined in TTControl.ino. Modules use these shared singletons to coordinate
// the firmware without creating ownership cycles.
extern Settings settings;
extern WaveformGenerator waveform;
extern MotorController motor;
extern UserInterface ui;

// The OLED driver is also global because the UI owns drawing but setup() owns
// I2C/display initialization.
extern Adafruit_SSD1306 display;

// --- Shared State Variables ---
// Volatile is used for values shared between Core 0 and Core 1 or ISRs. These
// are status mirrors; write through the relevant controller whenever possible.
extern volatile MotorState currentMotorState;
extern volatile float currentFrequency;
extern volatile float currentPitchPercent;

// Safe Mode is latched during setup before settings are loaded.
extern bool safeModeActive;

// Core 1 waits on this so waveform setup cannot run before settings and shared
// objects exist.
extern volatile bool systemInitialized;

#endif // GLOBALS_H
