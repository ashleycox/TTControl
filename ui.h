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

/**
 * @brief Manages the User Interface (Display and Input).
 * 
 * Handles:
 * - OLED drawing (Menus, Status, Dialogs)
 * - Input event routing (Encoder, Buttons)
 * - Menu navigation stack
 * - Screensaver and Error displays
 */
class UserInterface {
public:
    UserInterface();
    
    void begin();
    void update();
    
    // --- Navigation ---
    void navigateTo(MenuPage* page);
    void back();
    void exitMenu();
    void enterMenu(); 
    
    // --- Dialogs ---
    void showMessage(const char* msg, uint32_t duration);
    void showConfirm(const char* msg, void (*action)()); 
    void showError(const char* msg, uint32_t duration);
    
    // --- Input Injection (for Serial/Debug) ---
    void injectInput(int delta, bool btn);

private:
    InputManager _input;
    
    // Menu Stack for navigation history
    std::vector<MenuPage*> _menuStack;
    MenuPage* _currentPage;
    
    // UI State
    bool _inMenu;
    bool _screensaverActive;
    
    // Message Dialog State
    bool _showingMessage;
    const char* _messageText;
    uint32_t _messageStartTime;
    uint32_t _messageDuration;
    
    // Confirmation Dialog State
    bool _showingConfirm;
    const char* _confirmMsg;
    void (*_confirmAction)();
    bool _confirmResult;
    
    // Error Dialog State
    bool _showingError;
    const char* _errorMsg;
    uint32_t _errorStartTime;
    uint32_t _errorDuration;
    
    // Screensaver State
    int _saverX, _saverY, _saverDX, _saverDY;
    
    // Status Display State
    int _transitionDirection; // 0=None, 1=Forward, -1=Backward
    float _transitionProgress; // 0.0 to 1.0
    
    // Dashboard State
    int _statusMode; // 0=Standard, 1=Stats, 2=Dim
    
    // Smooth Scroll State
    MenuPage* _nextPage;
    float _smoothScrollY;
    
    uint8_t _lastBrightness;
    
    // Screensaver State
    uint8_t _matrixDrops[16]; // Y positions for 16 columns (128/8 = 16)
    float _lissajousPhase;
    
    void drawMatrixRain();
    void drawLissajous();
    
    // Inactivity Tracking
    uint32_t _lastInputTime;
    
    // Internal Drawing Methods
    void handleInput();
    void draw();
    void drawDashboard(); // Replaces drawStatus
    void drawMenu();
    void drawScreensaver();
    void drawConfirm();
    void drawMessage();
    void drawError();
    void drawGoodbye();
    
    // Serial Mirror
    void dumpDisplayToSerial();

    // Goodbye State
    bool _showingGoodbye;
    uint32_t _goodbyeStartTime;
};

#endif // UI_H
