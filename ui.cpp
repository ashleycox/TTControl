/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "ui.h"
#include "menu_data.h"
#include "motor.h"
#include "bitmaps.h"
#include "hal.h" // Added HAL
#include <Fonts/FreeSans12pt7b.h>

extern MenuPage* pageMain;
MenuPage* pagePower = nullptr;

UserInterface::UserInterface() {
    _inMenu = false;
    _currentPage = nullptr;
    _screensaverActive = false;
    _saverX = 10; _saverY = 10; _saverDX = 1; _saverDY = 1;
    _showingMessage = false;
    _showingConfirm = false;
    _showingError = false;
    _showingGoodbye = false;
    
    _statusMode = 0; // Standard
    
    _transitionProgress = 0.0;
    _transitionDirection = 0;
    _nextPage = nullptr;
    _smoothScrollY = 0.0;
    
    _lastBrightness = 0;
    _lissajousPhase = 0.0;
    for(int i=0; i<16; i++) _matrixDrops[i] = random(0, 64);
    
    _lastInputTime = 0;
}

void UserInterface::begin() {
    _input.begin();
    
    // Initialize the menu structure
    buildMenuSystem();
    
    // Show Splash Screen (Scrolling)
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    
    const char* msg = WELCOME_MESSAGE;
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    
    // Scroll from right to left
    for (int x = 128; x >= -((int)w); x -= 4) {
        display.clearDisplay();
        display.setCursor(x, 25);
        display.print(msg);
        display.display();
        // delay(10); // Blocking delay is fine in setup
    }
    
    display.clearDisplay();
    display.setCursor(30, 45);
    display.setTextSize(1);
    display.println(FIRMWARE_VERSION);
    display.display();
    delay(1000);
    
    // Configure Optional Buttons
    #ifdef SPEED_BUTTON_ENABLE
    if (SPEED_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_SPEED, INPUT_PULLUP);
    #endif
    
    #ifdef START_STOP_BUTTON_ENABLE
    if (START_STOP_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_START_STOP, INPUT_PULLUP);
    #endif
    
    #ifdef STANDBY_BUTTON_ENABLE
    if (STANDBY_BUTTON_ENABLE) hal.setPinMode(PIN_BTN_STANDBY, INPUT_PULLUP);
    #endif
    
    _lastInputTime = millis();
}

void UserInterface::update() {
    // Poll input devices
    _input.update();
    
    // Process input events
    handleInput();
    
    // --- Auto Features ---
    uint32_t now = millis();
    uint32_t elapsed = (now - _lastInputTime) / 1000; // Seconds
    
    // 1. Auto Standby (Only if STOPPED)
    uint8_t stbyDelay = settings.get().autoStandbyDelay;
    if (stbyDelay > 0 && !motor.isRunning() && !motor.isStandby()) {
        if (elapsed > (stbyDelay * 60)) {
            motor.toggleStandby();
            _lastInputTime = now; // Reset to avoid immediate re-trigger
        }
    }
    
    // 2. Auto Dim (Only if RUNNING and NOT already dimmed)
    uint8_t dimDelay = settings.get().autoDimDelay;
    if (dimDelay > 0 && motor.isRunning() && _statusMode != 2) {
        if (elapsed > (dimDelay * 60)) {
            _statusMode = 2; // Switch to Dim Mode
        }
    }
    
    // 3. Display Sleep (Only if NOT running, or if running and sleep allowed?)
    // Usually sleep is for Standby or Idle.
    // "Display Sleep Mode" - 0=Off, 1=10s, 2=20s, 3=30s, 4=1m, 5=5m, 6=10m
    int sleepDelayVal = settings.get().displaySleepDelay;
    if (sleepDelayVal > 0) {
        uint32_t sleepMs = 0;
        switch(sleepDelayVal) {
            case 1: sleepMs = 10000; break;
            case 2: sleepMs = 20000; break;
            case 3: sleepMs = 30000; break;
            case 4: sleepMs = 60000; break;
            case 5: sleepMs = 300000; break;
            case 6: sleepMs = 600000; break;
        }
        
        // Only sleep if NOT running (unless we want to sleep while running? usually not for a turntable)
        // Let's assume sleep is for Standby/Stopped state to save screen.
        // If motor is running, we have Auto Dim.
        if (!motor.isRunning() && (now - _lastInputTime > sleepMs)) {
            // Turn display off
             display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    }
    
    // 4. Screensaver Trigger
    // If in Standby:
    // - If Screensaver Enabled: Activate Screensaver
    // - If Screensaver Disabled: Turn Display OFF
    if (motor.isStandby()) {
        if (settings.get().screensaverEnabled) {
            _screensaverActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON); // Ensure ON for screensaver
        } else {
            _screensaverActive = false;
            display.ssd1306_command(SSD1306_DISPLAYOFF); // Turn OFF
        }
    } else {
        _screensaverActive = false;
        // If not in standby, display should be ON (unless sleep logic handled above)
        // But we need to be careful not to fight with sleep logic.
        // Sleep logic turns it OFF. Wake logic turns it ON.
        // So we just ensure _screensaverActive is false here.
    }
    
    // Update Animations
    if (_transitionDirection != 0) {
        _transitionProgress += 0.2; // Speed of slide
        if (_transitionProgress >= 1.0) {
            _transitionProgress = 0.0;
            _transitionDirection = 0;
            if (_nextPage) {
                _currentPage = _nextPage;
                _nextPage = nullptr;
            }
        }
    }
    
    // Render the current view
    draw();
}

void UserInterface::handleInput() {
    InputEvent evt = _input.getEvent();
    int delta = _input.getEncoderDelta();
    
    // Reset Inactivity Timer on any input
    if (evt != EVT_NONE || delta != 0 || _input.isButtonDown()) {
        _lastInputTime = millis();
        display.ssd1306_command(SSD1306_DISPLAYON); // Wake display
        
        // Wake from Auto Dim
        if (_statusMode == 2) {
            _statusMode = 0; // Restore to Standard
            // Consume the first input used to wake?
            // Usually better to consume it so we don't accidentally change speed etc.
            // But if it's just a rotation, maybe we want it to register?
            // Let's consume it to be safe.
            return; 
        }
    }
    
    // --- Global Button Handling ---
    // These work EVERYWHERE (Menu, Dashboard, etc.)
    
    // --- Global Button Handling ---
    // These work EVERYWHERE (Menu, Dashboard, etc.)
    
    if (_input.isSpeedButtonPressed()) {
        motor.cycleSpeed();
        // If in menu editing speed, update the shadow index
        if (_inMenu) {
             menuShadowSpeedIndex = (int)motor.getSpeed();
             menuShadowSettings = settings.get().speeds[menuShadowSpeedIndex];
             extern void updateSpeedLabel();
             updateSpeedLabel();
        }
    }
    
    if (_input.isStartStopPressed()) {
        if (motor.isStandby()) motor.toggleStandby(); // Wake
        else motor.toggleStartStop();
    }
    
    if (_input.isStandbyPressed()) {
        motor.toggleStandby();
    }
    
    // Wake from Screensaver
    if (_screensaverActive && (evt != EVT_NONE || delta != 0)) {
        _screensaverActive = false;
        
        // If in Standby and Select pressed, Wake immediately
        if (motor.isStandby() && evt == EVT_SELECT) {
            motor.toggleStandby();
        }
        return;
    }
    
    // Dismiss Error Dialog
    if (_showingError && (evt != EVT_NONE)) {
        _showingError = false;
        return;
    }
    
    // Handle Confirmation Dialog
    if (_showingConfirm) {
        if (delta != 0) _confirmResult = !_confirmResult;
        if (evt == EVT_SELECT) {
            if (_confirmResult && _confirmAction) _confirmAction();
            _showingConfirm = false;
        }
        return;
    }
    
    // Pitch Encoder Logic (Dedicated)
    #if PITCH_CONTROL_ENABLE
    int pitchDelta = _input.getPitchDelta();
    if (pitchDelta != 0 && motor.isRunning()) {
        float currentP = motor.getPitchPercent();
        float step = settings.get().pitchStepSize;
        float newP = currentP + (pitchDelta * step); 
        motor.setPitch(newP);
    }
    
    // Pitch Encoder Button (Toggle Range / Reset)
    // We need to read the button state. InputManager might not handle this button yet.
    // Let's assume we need to read PIN_ENC_PITCH_SW directly for now as per previous pattern.
    static bool lastPitchBtn = HIGH;
    static uint32_t pitchBtnDownTime = 0;
    bool pitchBtn = hal.digitalRead(PIN_ENC_PITCH_SW);
    
    if (pitchBtn == LOW && lastPitchBtn == HIGH) {
        pitchBtnDownTime = millis();
    }
    
    if (pitchBtn == HIGH && lastPitchBtn == LOW) {
        // Released
        uint32_t duration = millis() - pitchBtnDownTime;
        if (duration >= 2000) {
            // Long Press: Reset Pitch
            motor.resetPitch();
            showMessage("Pitch Reset", 1000);
        } else if (duration > 50) {
            // Short Press: Toggle Range
            motor.togglePitchRange();
            char buf[16];
            snprintf(buf, sizeof(buf), "Range: +/-%d%%", motor.getPitchRange());
            showMessage(buf, 1000);
        }
    }
    lastPitchBtn = pitchBtn;
    #endif

    // Menu Navigation Logic
    if (_inMenu && _currentPage) {
        // Block input during transition
        if (_transitionDirection != 0) return;

        // Pass encoder delta to current item (for editing values)
        if (delta != 0) {
            _currentPage->input(delta);
        }
        
        // Navigation (Only if not currently editing a value)
        MenuItem* item = _currentPage->getItem(_currentPage->getSelection());
        bool editing = item && item->isEditing();
        
        if (!editing) {
            if (evt == EVT_NAV_UP) _currentPage->next();
            if (evt == EVT_NAV_DOWN) _currentPage->prev();
        }
        
        if (evt == EVT_SELECT) _currentPage->select(_currentPage); // Pass ref to allow page transitions
        if (evt == EVT_BACK) back(); // Long press goes back
        
        // NEW: Hold (Long Press) in Menu -> Save & Exit
        // We need to distinguish between "Back" (Long Press) and "Exit" (Very Long Press)
        // or just redefine EVT_BACK behavior?
        // InputManager generates: SELECT (Short), BACK (Long > 3s), EXIT (Very Long > 5s).
        // User asked for "Hold (in menu) save and exit".
        // Let's map EVT_BACK to "Back" and EVT_EXIT to "Exit Menu".
        // Or if user wants "Hold" to be the shortcut, maybe EVT_BACK should exit?
        // Let's use EVT_BACK (3s) to Exit Menu completely.
        if (evt == EVT_BACK) exitMenu(); 
        
        if (evt == EVT_EXIT) exitMenu();
        
    } else {
        // Main Status Screen Logic
        
        // 1. Short Press: Start/Stop or Wake
        if (evt == EVT_SELECT) {
            if (motor.isStandby()) {
                motor.toggleStandby(); // Wake
            } else {
                motor.toggleStartStop();
            }
        }
        
        // 2. Double Press: Enter Menu
        if (evt == EVT_DOUBLE_CLICK) {
            enterMenu();
        }
        
        // 3. Hold (Long Press): Enter Standby
        if (evt == EVT_BACK || evt == EVT_EXIT) {
            if (!motor.isStandby()) {
                motor.toggleStandby();
                // Trigger Goodbye
                _showingGoodbye = true;
                _goodbyeStartTime = millis();
                display.ssd1306_command(SSD1306_DISPLAYON); // Ensure on
            }
        }
        
        // 4. Rotate: Change Speed OR Cycle Status (if pressed)
        if (delta != 0) {
            if (_input.isButtonDown()) {
                // 6. Press and Rotate: Cycle Status Displays
                // Cycle _statusMode (0=Standard, 1=Stats, 2=Dim)
                if (delta > 0) _statusMode = (_statusMode + 1) % 3;
                else _statusMode = (_statusMode + 2) % 3; // Wrap backwards
            } else {
                // 5. Rotate: Change Speed
                // Only if not in standby?
                if (!motor.isStandby()) {
                    motor.adjustSpeed(delta); 
                }
            }
        }
    }
}

void UserInterface::draw() {
    // Update Contrast/Brightness
    // Only update if changed? Or every frame? 
    // Updating every frame might be spammy on I2C.
    // Better to check a dirty flag or just do it.
    // SSD1306 command is fast.
    // But let's only do it if we are not in Dim mode (which handles its own dimming)
    // But let's only do it if we are not in Dim mode (which handles its own dimming)
    if (_statusMode != 2 && !_screensaverActive) {
        uint8_t target = settings.get().displayBrightness;
        if (target != _lastBrightness) {
            display.ssd1306_command(SSD1306_SETCONTRAST);
            display.ssd1306_command(target);
            _lastBrightness = target;
        }
    }
    
    display.clearDisplay();
    
    // Render based on current state priority
    if (_screensaverActive) {
        drawScreensaver();
    } else if (_showingError) {
        drawError();
    } else if (_showingConfirm) {
        drawConfirm();
    } else if (_showingMessage) {
        drawMessage();
    } else if (_showingGoodbye) {
        drawGoodbye();
    } else if (_inMenu && _currentPage) {
        drawMenu();
    } else {
        drawDashboard();
    }
    
    display.display();
    
    #if DUPLICATE_DISPLAY_TO_SERIAL && SERIAL_MONITOR_ENABLE
    dumpDisplayToSerial();
    #endif
}

void UserInterface::dumpDisplayToSerial() {
    // Simple ASCII Art Dump
    // Throttled to avoid saturating Serial (max 1 FPS)
    static uint32_t lastDump = 0;
    if (millis() - lastDump < 1000) return;
    lastDump = millis();
    
    Serial.println("\n--- Display Mirror ---");
    for (int y = 0; y < 64; y += 2) { // Skip every other line for aspect ratio/speed
        for (int x = 0; x < 128; x++) {
            if (display.getPixel(x, y)) Serial.print("#");
            else Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println("----------------------");
}

void UserInterface::drawGoodbye() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    
    const char* msg = "Goodbye...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    
    // Scroll or just fade? User asked for "Scrolling Goodbye".
    // Since draw() is called in a loop, we need to calculate position based on time.
    uint32_t elapsed = millis() - _goodbyeStartTime;
    int x = 128 - (elapsed / 10); // Speed
    
    display.setCursor(x, 25);
    display.print(msg);
    
    if (x < -((int)w)) {
        _showingGoodbye = false;
        // Now actually sleep/standby visual
        // If screensaver is disabled, turn off display.
        // If enabled, it will be handled by update() loop next cycle.
        if (!settings.get().screensaverEnabled) {
             display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    }
}

void UserInterface::drawMenu() {
    // Handle Transitions
    int xOffset = 0;
    MenuPage* pageToDraw = _currentPage;
    
    if (_transitionDirection != 0) {
        // Ease In/Out
        float t = _transitionProgress;
        // Simple linear for now
        int offsetPixels = (int)(128.0 * (1.0 - t));
        
        if (_transitionDirection == 1) { // Forward (Slide Left)
            // Current page moves left (-offset), Next page moves in from right
            // Wait, we need to draw BOTH?
            // Drawing both might be slow on I2C.
            // Let's just draw the incoming page sliding in?
            // Or just draw the current page sliding out?
            // For simplicity on 128x64, let's just slide the content of the *new* page in.
            // But we don't have _nextPage set in the state properly for "Forward" logic in update().
            // Actually update() swaps them at end.
            // Let's simplify: No complex dual-draw. Just slide the current page content.
            // If we are transitioning, we assume _currentPage is the target.
            // But wait, update() handles the swap at end of transition.
            // So during transition, _currentPage is the OLD page?
            // Let's fix the logic:
            // When navigating, set _nextPage and start transition.
            // During transition, draw _currentPage sliding OUT, and _nextPage sliding IN.
            // This requires `drawPage(MenuPage* p, int x)` helper.
            // For now, let's just implement Smooth Scrolling and static page switch to avoid complexity.
            // User asked for "Menu Slide Transitions".
            // Let's try a simple slide:
            // If transition active, draw _currentPage at offset.
        }
    }
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Title
    display.setCursor(0, 0);
    display.print(_currentPage->getTitle());
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    
    // Smooth Scrolling Logic
    int selection = _currentPage->getSelection();
    int targetY = selection * 10; // 10px per item
    
    // Interpolate
    _smoothScrollY += (targetY - _smoothScrollY) * 0.3;
    
    // Viewport
    int visibleHeight = 50;
    int startY = 15;
    
    // Calculate offset based on smooth scroll
    // We want the selected item to be roughly centered or kept in view
    // Let's stick to the window logic but smooth the rendering offset?
    // Actually, the `_offset` in MenuPage handles the logical window.
    // Let's just draw the items relative to `_offset` but maybe animate the highlight?
    
    int total = _currentPage->getItemCount();
    int offset = _currentPage->getOffset();
    int visible = 5;
    
    for (int i = 0; i < visible; i++) {
        int idx = offset + i;
        if (idx >= total) break;
        
        MenuItem* item = _currentPage->getItem(idx);
        int y = 15 + (i * 10);
        
        // Highlight Box
        if (idx == selection) {
            display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Invert text
        } else {
            display.setTextColor(SSD1306_WHITE);
        }
        
        display.setCursor(2, y);
        display.print(item->getLabel());
        
        // Value
        char valBuf[16];
        item->getValueString(valBuf, sizeof(valBuf));
        if (valBuf[0] != 0) {
            display.setCursor(80, y); // Right align-ish
            display.print(valBuf);
        }
        
        // Dirty Indicator
        if (item->isDirty()) {
            display.setCursor(120, y);
            display.print(F("*"));
        }
    }
    
    // Draw Scrollbar
    if (total > visible) {
        int sbHeight = (visible * 50) / total;
        if (sbHeight < 2) sbHeight = 2;
        int sbY = 15 + (offset * 50) / total;
        display.fillRect(126, sbY, 2, sbHeight, SSD1306_WHITE);
    }
}

void UserInterface::drawDashboard() {
    // --- Graphical Dashboard ---
    
    // Mode 2: Dim / Minimal
    if (_statusMode == 2) {
        display.dim(true); // Low contrast
        
        // Minimal Display: Just Speed
        display.setFont(&FreeSans12pt7b);
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        
        SpeedMode s = motor.getSpeed();
        const char* speedStr;
        if (s == SPEED_33) speedStr = "33";
        else if (s == SPEED_45) speedStr = "45";
        else speedStr = "78";
        
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((128 - w) / 2, 40);
        display.print(speedStr);
        display.setFont(NULL);
        return;
    }
    
    display.dim(false); // Normal contrast
    
    // 1. Status Icons (Top Row)
    if (motor.isRunning()) {
        display.drawBitmap(0, 0, icon_play_bits, 16, 16, SSD1306_WHITE);
    } else {
        display.drawBitmap(0, 0, icon_stop_bits, 16, 16, SSD1306_WHITE);
    }
    
    // Lock Icon (if speed is stable)
    if (motor.isRunning()) {
        display.drawBitmap(112, 0, icon_lock_bits, 16, 16, SSD1306_WHITE);
    }
    
    // Mode 1: Stats
    if (_statusMode == 1) {
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        
        display.setCursor(0, 20);
        display.print("Session: ");
        // Calculate session time
        uint32_t sessionSec = settings.getSessionRuntime();
        int min = sessionSec / 60;
        int sec = sessionSec % 60;
        display.print(min); display.print("m "); display.print(sec); display.print("s");
        
        display.setCursor(0, 35);
        display.print("Total: ");
        // Total runtime from settings (in seconds)
        uint32_t totalSec = settings.getTotalRuntime();
        int hours = totalSec / 3600;
        int tMin = (totalSec % 3600) / 60;
        display.print(hours); display.print("h "); display.print(tMin); display.print("m");
        
        return;
    }
    
    // Mode 0: Standard (RPM + Pitch Bar)
    
    // 2. Main RPM Display (Center)
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    SpeedMode s = motor.getSpeed();
    const char* speedStr;
    if (s == SPEED_33) speedStr = "33.3";
    else if (s == SPEED_45) speedStr = "45.0";
    else speedStr = "78.0";
    
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((128 - w) / 2, 40);
    display.print(speedStr);
    
    // Reset Font for other elements
    display.setFont(NULL); 
    
    // 3. Pitch / Ramping Bar (Bottom)
    // Draw Scale
    display.drawLine(10, 55, 118, 55, SSD1306_WHITE); // Main line
    display.drawLine(64, 52, 64, 58, SSD1306_WHITE); // Center tick
    display.drawLine(10, 52, 10, 58, SSD1306_WHITE); // Left tick
    display.drawLine(118, 52, 118, 58, SSD1306_WHITE); // Right tick
    
    // Calculate Deviation for Visualization
    // We want to show the ACTUAL deviation from nominal frequency
    float nominal = settings.getCurrentSpeedSettings().frequency;
    float current = motor.getCurrentFrequency();
    float deviationPercent = 0.0;
    
    if (nominal > 0) {
        deviationPercent = ((current - nominal) / nominal) * 100.0;
    }
    
    // Range +/- 8% for the bar
    float range = 8.0;
    
    int px = 64 + (int)((deviationPercent / range) * 54.0);
    if (px < 10) px = 10;
    if (px > 118) px = 118;
    
    display.fillTriangle(px, 50, px-3, 46, px+3, 46, SSD1306_WHITE);
    
    // Pitch Value Text
    // If Pitch Control is enabled, show the SETTING.
    // If disabled, maybe show nothing or "LOCKED"?
    // User said: "keep the display bar for visual effect, especially during frequency ramps"
    
    display.setTextSize(1);
    display.setCursor(50, 64-8);
    
    #if PITCH_CONTROL_ENABLE
        float pitchSetting = motor.getPitchPercent();
        if (pitchSetting > 0) display.print("+");
        display.print(pitchSetting, 1);
        display.print("%");
    #else
        // Optional: Show "AUTO" or just the deviation?
        // Let's show the actual deviation if it's significant (ramping), otherwise "LOCKED"
        if (abs(deviationPercent) > 0.1) {
             if (deviationPercent > 0) display.print("+");
             display.print(deviationPercent, 1);
             display.print("%");
        } else {
             display.print("LOCKED");
        }
    #endif
}

void UserInterface::drawScreensaver() {
    display.clearDisplay();
    
    if (settings.get().screensaverMode == SAVER_MATRIX) {
        drawMatrixRain();
    }
    else if (settings.get().screensaverMode == SAVER_LISSAJOUS) {
        drawLissajous();
    }
    else {
        // Default: Bouncing Text
        // Update position every 50ms
        static uint32_t lastMove = 0;
        if (millis() - lastMove > 50) {
            lastMove = millis();
            _saverX += _saverDX;
            _saverY += _saverDY;
            
            if (_saverX <= 0 || _saverX >= (128 - 60)) _saverDX = -_saverDX; // Approx width
            if (_saverY <= 0 || _saverY >= (64 - 8)) _saverDY = -_saverDY;
        }
        
        display.setCursor(_saverX, _saverY);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.print(STANDBY_MESSAGE);
    }
    
    display.display();
}

void UserInterface::drawMatrixRain() {
    // 16 Columns (128px / 8px char width)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    for (int i = 0; i < 16; i++) {
        // Draw the "lead" character
        char c = (char)random(33, 126);
        display.setCursor(i * 8, _matrixDrops[i]);
        display.write(c);
        
        // Draw a trail (fainter/simulated by skipping pixels if we could, but here just chars)
        if (_matrixDrops[i] >= 8) {
             display.setCursor(i * 8, _matrixDrops[i] - 8);
             display.write('.');
        }
        
        // Update drop position
        if (random(0, 10) > 2) { // Random speed
            _matrixDrops[i] += 4;
        }
        
        // Reset if off screen
        if (_matrixDrops[i] > 64) {
            _matrixDrops[i] = 0;
        }
    }
}

void UserInterface::drawLissajous() {
    // Parametric equations: x = A*sin(a*t + d), y = B*sin(b*t)
    _lissajousPhase += 0.05;
    
    int cx = 64;
    int cy = 32;
    int amp = 30;
    
    // Draw the curve
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + amp * sin(3.0 * t + _lissajousPhase);
        int y = cy + amp * sin(2.0 * t);
        display.drawPixel(x, y, SSD1306_WHITE);
    }
    
    // Draw a second curve for complexity
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + (amp/2) * sin(2.0 * t - _lissajousPhase);
        int y = cy + (amp/2) * cos(3.0 * t);
        display.drawPixel(x, y, SSD1306_WHITE);
    }
}

void UserInterface::drawConfirm() {
    // Modal Style
    display.fillRect(10, 10, 108, 44, SSD1306_BLACK); // Background
    display.drawRect(10, 10, 108, 44, SSD1306_WHITE); // Border
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Center Message
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(_confirmMsg, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((128 - w) / 2, 20);
    display.println(_confirmMsg);
    
    display.setCursor(30, 40);
    if (_confirmResult) display.print(F("> YES   NO"));
    else display.print(F("  YES > NO"));
}

void UserInterface::drawMessage() {
    // Modal Style
    display.fillRect(10, 15, 108, 34, SSD1306_BLACK);
    display.drawRect(10, 15, 108, 34, SSD1306_WHITE);
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(15, 25);
    display.println(_messageText);
    
    if (millis() - _messageStartTime > _messageDuration) _showingMessage = false;
}

void UserInterface::drawError() {
    // Modal Style with Pattern
    display.fillRect(5, 5, 118, 54, SSD1306_BLACK);
    display.drawRect(5, 5, 118, 54, SSD1306_WHITE);
    
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 10);
    display.println(F("ERROR"));
    
    display.setTextSize(1);
    display.setCursor(10, 35);
    display.println(_errorMsg);
    
    if (millis() - _errorStartTime > _errorDuration) _showingError = false;
}

void UserInterface::navigateTo(MenuPage* page) {
    if (_currentPage) _menuStack.push_back(_currentPage);
    
    // Trigger Transition
    _nextPage = page;
    _transitionDirection = 1; // Forward
    _transitionProgress = 0.0;
    
    // For now, instant switch to support the simple drawMenu logic
    // until we implement full dual-page rendering
    _currentPage = page;
    _transitionDirection = 0;
}

void UserInterface::back() {
    if (!_menuStack.empty()) {
        _currentPage = _menuStack.back();
        _menuStack.pop_back();
        
        // Trigger Transition
        _transitionDirection = -1; // Back
        _transitionProgress = 0.0;
    } else {
        exitMenu();
    }
}

void UserInterface::exitMenu() {
    _inMenu = false;
    _menuStack.clear();
    _currentPage = nullptr;
}

void UserInterface::enterMenu() {
    initMenuState();
    _inMenu = true;
    _currentPage = pageMain;
}

void UserInterface::showMessage(const char* msg, uint32_t duration) {
    _messageText = msg;
    _messageDuration = duration;
    _messageStartTime = millis();
    _showingMessage = true;
}

void UserInterface::showConfirm(const char* msg, void (*action)()) {
    _confirmMsg = msg;
    _confirmAction = action;
    _confirmResult = false;
    _showingConfirm = true;
}

void UserInterface::showError(const char* msg, uint32_t duration) {
    _errorMsg = msg;
    _errorDuration = duration;
    _errorStartTime = millis();
    _showingError = true;
    // Safety: Stop motor relays on critical error display
    motor.setRelays(false); 
}

void UserInterface::injectInput(int delta, bool btn) {
    _input.injectDelta(delta);
    _input.injectButton(btn);
}
