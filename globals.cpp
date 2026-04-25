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
MenuPage* pagePower = nullptr;
MenuPage* pageMotor = nullptr;
MenuPage* pageDisplay = nullptr;
MenuPage* pageSystem = nullptr;
MenuPage* pageBrakeTune = nullptr;
MenuPage* pageRelayTest = nullptr;

// --- Shadow Settings State ---
// Used for temporary storage during menu editing operations
SpeedSettings menuShadowSettings;
int menuShadowSpeedIndex = 0;

// --- Core Synchronization ---
// Set to true when Core 0 has finished initializing shared resources
volatile bool systemInitialized = false;

// Safe Mode Boot Flag (Set in setup() if encoder held)
bool safeModeActive = false;
