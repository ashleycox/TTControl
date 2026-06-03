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

/*
 * --- Global Menu Page Pointers ---
 * The menu tree is built once and then reused. Pointers live here because UI and
 * menu_data both need access without making either module own the other.
 */
MenuPage* pageMain = nullptr;
MenuPage* pagePresets = nullptr;
MenuPage* pageErrorLog = nullptr;
MenuPage* pageUnlock = nullptr;
MenuPage* pageSpeedTuning = nullptr;
MenuPage* pagePhase = nullptr;
MenuPage* pagePower = nullptr;
MenuPage* pageMotor = nullptr;
MenuPage* pageDisplay = nullptr;
MenuPage* pageSystem = nullptr;
MenuPage* pageSecurity = nullptr;
MenuPage* pageBrakeTune = nullptr;
MenuPage* pageRelayTest = nullptr;

/*
 * --- Shadow Settings State ---
 * Per-speed menu editing works on this copy until the user chooses Save. That
 * prevents accidental flash writes while scrolling through values.
 */
SpeedSettings menuShadowSettings;
int menuShadowSpeedIndex = 0;

/*
 * --- Core Synchronization ---
 * Set to true when Core 0 has finished initializing shared resources. Core 1
 * must not touch waveform settings before this is raised.
 */
volatile bool systemInitialized = false;

// Set in setup() when the main encoder button is held during boot. Settings uses this to bypass flash-loaded values for recovery.
bool safeModeActive = false;
