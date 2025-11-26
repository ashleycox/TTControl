/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "input.h"
#include "input.h"
#include "hal.h"
#include "globals.h"
#include "settings.h" // Need definition of Settings class too

InputManager* _inputInstance = nullptr;

InputManager::InputManager() {
    _encDelta = 0;
    _lastEncTime = 0;
    _encAccel = 0;
    
    _pitchLastClk = HIGH;
    _pitchDelta = 0;
    _lastPitchTime = 0;
    _pitchAccel = 0;
    
    _btnPressed = false;
    _btnPressTime = 0;
    _waitingForDoubleClick = false;
    _doubleClickTimer = 0;
    _clickCount = 0;
    
    _encoderPosition = 0;
    _lastEncoderPosition = 0;
    _inputInstance = this;
    
    _speedBtnState = HIGH;
    _speedBtnTime = 0;
    _startStopBtnState = HIGH;
    _startStopBtnTime = 0;
    _standbyBtnState = HIGH;
    _standbyBtnTime = 0;
    
    _pendingEvent = EVT_NONE;
    
    _injectedDelta = 0;
    _injectedBtn = false;
}

void InputManager::begin() {
    hal.setPinMode(PIN_ENC_MAIN_CLK, INPUT_PULLUP);
    hal.setPinMode(PIN_ENC_MAIN_DT, INPUT_PULLUP);
    hal.setPinMode(PIN_ENC_MAIN_SW, INPUT_PULLUP);
    
    // Attach Interrupt
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_MAIN_CLK), isrEncoder, CHANGE);
    
    if (SPEED_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_SPEED, INPUT_PULLUP);
    if (START_STOP_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_START_STOP, INPUT_PULLUP);
    if (STANDBY_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_STANDBY, INPUT_PULLUP);
    
    // Read initial state (only for pitch encoder now, main encoder handled by ISR)
    // _encLastClk = hal.digitalRead(PIN_ENC_MAIN_CLK); // No longer needed for main encoder
    
    #if PITCH_CONTROL_ENABLE
    hal.setPinMode(PIN_ENC_PITCH_CLK, INPUT_PULLUP);
    hal.setPinMode(PIN_ENC_PITCH_DT, INPUT_PULLUP);
    hal.setPinMode(PIN_ENC_PITCH_SW, INPUT_PULLUP); // Optional switch
    _pitchLastClk = hal.digitalRead(PIN_ENC_PITCH_CLK);
    #endif
}

void InputManager::isrEncoder() {
    if (!_inputInstance) return;
    
    // Simple Quadrature Decode
    // Read CLK and DT
    // We interrupt on CLK change
    // If DT != CLK, it's one direction, else other
    
    int clk = digitalRead(PIN_ENC_MAIN_CLK);
    int dt = digitalRead(PIN_ENC_MAIN_DT);
    
    if (clk != dt) {
        _inputInstance->_encoderPosition++;
    } else {
        _inputInstance->_encoderPosition--;
    }
}

void InputManager::update() {
    uint32_t now = hal.getMillis();
    
    // --- Encoder Reading from ISR ---
    // Atomic read not strictly necessary for long on 32-bit, but good practice
    noInterrupts();
    long pos = _encoderPosition;
    interrupts();
    
    long delta = pos - _lastEncoderPosition;
    _lastEncoderPosition = pos;
    
    // Handle Input Injection (for testing/serial control)
    if (_injectedDelta != 0) {
        delta += _injectedDelta;
        _injectedDelta = 0;
    }
    
    // --- Acceleration Logic ---
    // If rotation is fast, increase the delta multiplier
    if (delta != 0) {
        if (now - _lastEncTime < 50) { // Fast rotation threshold
            _encAccel++;
            if (_encAccel > 5) delta *= 5;
            else if (_encAccel > 2) delta *= 2;
        } else {
            _encAccel = 0;
        }
        _lastEncTime = now;
        
        _encDelta += delta; // Accumulate for value editing
        
        // Map to Navigation Events
        // We generate events for navigation but keep raw delta for smooth value editing
        if (delta > 0) _pendingEvent = EVT_NAV_UP;
        else if (delta < 0) _pendingEvent = EVT_NAV_DOWN;
    }
    
    // --- Pitch Encoder Reading ---
    #if PITCH_CONTROL_ENABLE
    int pDelta = readPitchEncoder();
    if (pDelta != 0) {
        // Optional acceleration for pitch too?
        // Pitch usually needs fine control, so maybe less aggressive accel
        if (now - _lastPitchTime < 30) {
             _pitchAccel++;
             if (_pitchAccel > 5) pDelta *= 2; // Only 2x for pitch
        } else {
            _pitchAccel = 0;
        }
        _lastPitchTime = now;
        _pitchDelta += pDelta;
    }
    #endif
    
    // --- Button Handling ---
    bool btnState = (hal.digitalRead(PIN_ENC_MAIN_SW) == LOW) || _injectedBtn;
    if (_injectedBtn) _injectedBtn = false;
    
    // Debounce Logic
    static uint32_t lastBtnChange = 0;
    static bool lastBtnState = false;
    if (btnState != lastBtnState) {
        lastBtnChange = now;
        lastBtnState = btnState;
    }
    
    if (now - lastBtnChange > 20) { // Stable state
        if (btnState && !_btnPressed) {
            // Press Start
            _btnPressed = true;
            _btnPressTime = now;
        } else if (!btnState && _btnPressed) {
            // Press Release
            _btnPressed = false;
            uint32_t duration = now - _btnPressTime;
            
            if (duration > 5000) {
                _pendingEvent = EVT_EXIT; // Very Long Press (>5s)
            } else if (duration > 3000) { 
                _pendingEvent = EVT_BACK; // Long Press (>3s)
            } else {
                // Short Press - Check for double click
                if (_waitingForDoubleClick) {
                    _clickCount++;
                } else {
                    _waitingForDoubleClick = true;
                    _doubleClickTimer = now;
                    _clickCount = 1;
                }
            }
        }
    }
    
    // Double Click Timeout
    if (_waitingForDoubleClick && (now - _doubleClickTimer > 400)) {
        _waitingForDoubleClick = false;
        if (_clickCount == 2) {
            _pendingEvent = EVT_DOUBLE_CLICK;
        } else {
            _pendingEvent = EVT_SELECT; // Single Click confirmed
        }
    }
}

InputEvent InputManager::getEvent() {
    InputEvent e = _pendingEvent;
    _pendingEvent = EVT_NONE; // Consume event
    return e;
}

int InputManager::getEncoderDelta() {
    int d = _encDelta;
    _encDelta = 0; // Consume delta
    return d;
}

int InputManager::getPitchDelta() {
    int d = _pitchDelta;
    _pitchDelta = 0;
    return d;
}

void InputManager::injectDelta(int delta) {
    _injectedDelta += delta;
}

void InputManager::injectButton(bool pressed) {
    if (pressed) _injectedBtn = true;
}


// --- Global Button Implementations ---

bool InputManager::isSpeedButtonPressed() {
    #ifdef SPEED_BUTTON_ENABLE
    if (!SPEED_BUTTON_ENABLE) return false;
    bool reading = hal.digitalRead(PIN_BTN_SPEED);
    if (reading == LOW && _speedBtnState == HIGH && (millis() - _speedBtnTime > 200)) {
        _speedBtnTime = millis();
        _speedBtnState = LOW;
        return true;
    }
    if (reading == HIGH) _speedBtnState = HIGH;
    #endif
    return false;
}

bool InputManager::isStartStopPressed() {
    #ifdef START_STOP_BUTTON_ENABLE
    if (!START_STOP_BUTTON_ENABLE) return false;
    bool reading = hal.digitalRead(PIN_BTN_START_STOP);
    if (reading == LOW && _startStopBtnState == HIGH && (millis() - _startStopBtnTime > 200)) {
        _startStopBtnTime = millis();
        _startStopBtnState = LOW;
        return true;
    }
    if (reading == HIGH) _startStopBtnState = HIGH;
    #endif
    return false;
}

bool InputManager::isStandbyPressed() {
    #ifdef STANDBY_BUTTON_ENABLE
    if (!STANDBY_BUTTON_ENABLE) return false;
    bool reading = hal.digitalRead(PIN_BTN_STANDBY);
    if (reading == LOW && _standbyBtnState == HIGH && (millis() - _standbyBtnTime > 200)) {
        _standbyBtnTime = millis();
        _standbyBtnState = LOW;
        return true;
    }
    if (reading == HIGH) _standbyBtnState = HIGH;
    #endif
    return false;
}

int InputManager::readPitchEncoder() {
    #if PITCH_CONTROL_ENABLE
    int clk = hal.digitalRead(PIN_ENC_PITCH_CLK);
    int delta = 0;
    
    if (clk != _pitchLastClk) {
        if (hal.digitalRead(PIN_ENC_PITCH_DT) != clk) {
            delta = 1;
        } else {
            delta = -1;
        }
    }
    _pitchLastClk = clk;
    return delta;
    #else
    return 0;
    #endif
}
