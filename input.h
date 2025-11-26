/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>
#include "config.h"

// Abstracted Input Events
enum InputEvent {
    EVT_NONE,
    EVT_NAV_UP,      // Encoder Right (Clockwise)
    EVT_NAV_DOWN,    // Encoder Left (Counter-Clockwise)
    EVT_SELECT,      // Short Press
    EVT_BACK,        // Long Press
    EVT_EXIT,        // Very Long Press
    EVT_DOUBLE_CLICK // Double Click
};

/**
 * @brief Manages physical input devices (Encoder, Buttons).
 * 
 * Handles:
 * - Quadrature decoding
 * - Acceleration (faster rotation = larger delta)
 * - Button debouncing
 * - Click detection (Single, Double, Long, Very Long)
 */
class InputManager {
public:
    InputManager();
    void begin();
    void update();
    
    // Check for pending high-level events (consumes the event)
    InputEvent getEvent();
    
    // Get raw encoder delta for value editing (consumes the delta)
    int getEncoderDelta();
    
    // Get pitch encoder delta (consumes the delta)
    int getPitchDelta();
    
    // Check if main button is currently held down
    bool isButtonDown() { return _btnPressed; }
    
    // Injection for Serial/Testing
    void injectDelta(int delta);
    void injectButton(bool pressed);

    // Global Button Accessors (Debounced)
    bool isSpeedButtonPressed();
    bool isStartStopPressed();
    bool isStandbyPressed();

private:
    // Encoder State
    volatile long _encoderPosition;
    long _lastEncoderPosition;
    
    int _encDelta; // Accumulated delta since last read
    uint32_t _lastEncTime;
    int _encAccel;
    
    // Pitch Encoder State
    int _pitchLastClk;
    int _pitchDelta;
    uint32_t _lastPitchTime;
    int _pitchAccel;
    
    // Button State
    bool _btnPressed;
    uint32_t _btnPressTime;
    bool _waitingForDoubleClick;
    uint32_t _doubleClickTimer;
    int _clickCount;
    
    // Global Button State
    bool _speedBtnState;
    uint32_t _speedBtnTime;
    bool _startStopBtnState;
    uint32_t _startStopBtnTime;
    bool _standbyBtnState;
    uint32_t _standbyBtnTime;
    
    static void isrEncoder();

    // Event Queue (Single item buffer)
    InputEvent _pendingEvent;
    
    // Injection State
    int _injectedDelta;
    bool _injectedBtn;
    
    int readPitchEncoder();
};

#endif // INPUT_H
