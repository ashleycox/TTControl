/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef MENU_DATA_H
#define MENU_DATA_H

#include "menu_system.h"
#include "settings.h"

// Global Menu Pages
extern MenuPage* pageMain;
extern MenuPage* pagePresets;
extern MenuPage* pageErrorLog;

// New Submenus
extern MenuPage* pageSpeedTuning;
extern MenuPage* pagePhase;
extern MenuPage* pagePower;
extern MenuPage* pageMotor;
extern MenuPage* pageDisplay;
extern MenuPage* pageSystem;

// Shadow State for Speed Settings
// Used to edit settings temporarily before saving
extern SpeedSettings menuShadowSettings;
extern int menuShadowSpeedIndex;

// Builder Function
void buildMenuSystem();
void initMenuState();

#endif // MENU_DATA_H
