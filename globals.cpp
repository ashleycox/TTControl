/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "globals.h"
#include "menu_system.h"
#include "settings.h"

// --- Global Menu Page Pointers ---
// These are dynamically allocated in buildMenuSystem()
MenuPage* pageMain = nullptr;
MenuPage* pagePresets = nullptr;
MenuPage* pageErrorLog = nullptr;
MenuPage* pageSpeedTuning = nullptr;
MenuPage* pagePhase = nullptr;
MenuPage* pageMotor = nullptr;
MenuPage* pageDisplay = nullptr;
MenuPage* pageSystem = nullptr;

// --- Shadow Settings State ---
// Used for temporary storage during menu editing operations
SpeedSettings menuShadowSettings;
int menuShadowSpeedIndex = 0;

// --- Core Synchronization ---
// Flag to indicate Core 0 has completed setup, allowing Core 1 to proceed
volatile bool systemInitialized = false;
