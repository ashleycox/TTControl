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

// Global Menu Pages. buildMenuSystem() allocates these once; UI navigates by switching the current MenuPage pointer.
extern MenuPage* pageMain;
extern MenuPage* pagePresets;
extern MenuPage* pageErrorLog;
extern MenuPage* pageUnlock;

// New Submenus
extern MenuPage* pageSpeedTuning;
extern MenuPage* pagePhase;
extern MenuPage* pagePower;
extern MenuPage* pageMotor;
extern MenuPage* pageDisplay;
extern MenuPage* pageSystem;
extern MenuPage* pageSecurity;
extern MenuPage* pageBrakeTune;
extern MenuPage* pageRelayTest;
extern MenuPage* pageNetwork;
extern MenuPage* pageClosedLoop;

// Shadow State for Speed Settings. Per-speed tuning edits this copy first so Save/Cancel can commit or discard a whole speed tune at once.
extern SpeedSettings menuShadowSettings;
extern int menuShadowSpeedIndex;

// Builder Function
void buildMenuSystem();
void initMenuState();
void commitMenuShadowSettings();
void saveMenuChangesAndExit();
void cancelMenuChangesAndExit();

#endif // MENU_DATA_H
