/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef UI_H
#define UI_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include "config.h"
#include "types.h"
#include "globals.h"
#include "input.h"
#include "menu_system.h"

// Owns the Core 0 user experience: input polling, menu routing, transient
// dialogs, dashboard rendering, and standby/screensaver display policy.
class UserInterface {
public:
    UserInterface();
    
    // Initialize input hardware, build the static menu tree, and show the
    // blocking boot splash before the main loop starts.
    void begin();

    // Poll input, apply idle policies, advance animations, and redraw once.
    void update();
    
    // --- Navigation ---
    // Page navigation keeps a stack so Back returns to the previous menu page.
    void navigateTo(MenuPage* page);
    void back();
    void exitMenu();
    void enterMenu(); 
    
    // --- Dialogs ---
    // Dialog strings are copied into local buffers so callers may pass stack
    // buffers, including short serial/debug messages.
    void showMessage(const char* msg, uint32_t duration);
    void showConfirm(const char* msg, void (*action)()); 
    void showError(const char* msg, uint32_t duration, bool muteOutputs = false);
    
    // --- Input Injection (for Serial/Debug) ---
    // Serial UI tests feed the same event path as the physical encoder.
    void injectInput(int delta, bool btn);

private:
    InputManager _input;
    
    // Menu stack contains pointers to the static pages built in menu_data.cpp.
    std::vector<MenuPage*> _menuStack;
    MenuPage* _currentPage;
    
    // Top-level view flags. Dialog flags are checked before menu/dashboard.
    bool _inMenu;
    bool _screensaverActive;
    
    // Short-lived informational modal.
    bool _showingMessage;
    const char* _messageText;
    char _messageBuffer[64];
    uint32_t _messageStartTime;
    uint32_t _messageDuration;
    
    // Two-choice modal. The action callback is run only on confirmed Select.
    bool _showingConfirm;
    const char* _confirmMsg;
    char _confirmBuffer[64];
    void (*_confirmAction)();
    bool _confirmResult;
    
    // Error modal may also mute outputs through MotorController::setRelays().
    bool _showingError;
    const char* _errorMsg;
    char _errorBuffer[64];
    uint32_t _errorStartTime;
    uint32_t _errorDuration;
    
    // Bouncing-text screensaver position and velocity.
    int _saverX, _saverY, _saverDX, _saverDY;
    
    // Reserved slide state for menu transitions. Current drawing switches pages
    // immediately but keeps the fields for future animation work.
    int _transitionDirection; // 0=None, 1=Forward, -1=Backward
    float _transitionProgress; // 0.0 to 1.0
    
    // Dashboard page selector: 0=Standard, 1=Stats, 2=Dim, 3=Scope,
    // 4=CPU, 5=Memory, 6=Flash.
    int _statusMode; // 0=Standard, 1=Stats, 2=Dim, 3=Scope, 4=CPU, 5=Memory, 6=Flash
    
    // Menu scroll animation state.
    MenuPage* _nextPage;
    float _smoothScrollY;
    
    // Last contrast sent to the SSD1306; avoids repeated I2C commands.
    uint8_t _lastBrightness;
    
    // Matrix and Lissajous screensaver animation state.
    uint8_t _matrixDrops[16]; // Y positions for 16 columns (128/8 = 16)
    float _lissajousPhase;
    
    void drawMatrixRain();
    void drawLissajous();
    
    // Inactivity clock drives auto standby, dim, and display sleep.
    uint32_t _lastInputTime;
    
    // Input and drawing are split so update() has one clear sequence:
    // poll, handle, policy, animate, draw.
    void handleInput();
    void draw();
    void drawDashboard();
    void drawSweepScreen();
    void drawMenu();
    void drawScreensaver();
    void drawConfirm();
    void drawMessage();
    void drawError();
    void drawGoodbye();
    
    // Optional serial mirror for UI tests without the OLED in view.
    void dumpDisplayToSerial();

    // Standby transition animation state.
    bool _showingGoodbye;
    uint32_t _goodbyeStartTime;
};

#endif // UI_H
