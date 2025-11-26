/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <Arduino.h>
#include <vector>
#include <functional>
#include "config.h"
#include "globals.h"
#include "motor.h"
#include "settings.h"

/**
 * @brief Handles Serial Command Interface.
 * 
 * Parses incoming serial strings and executes commands for:
 * - Motor control (start, stop, speed)
 * - System status reporting
 * - UI input injection (for testing)
 * - Error log management
 */

void handleSerialCommands();
void printStatus();
void printHelp();

#endif // SERIAL_CMD_H
