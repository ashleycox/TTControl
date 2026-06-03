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
#include "hal.h"
#include "waveform.h"
#include "system_monitor.h"
#include "network_manager.h"
#include <Fonts/FreeSans12pt7b.h>

// The dashboard uses four-character state labels so the top row still has space for frequency and the lock icon on a 128px display.
static const char* dashboardStateLabel() {
    if (motor.isRelayTestMode()) return "TEST";
    if (motor.isSpeedRamping()) return "RAMP";

    switch (motor.getState()) {
        case STATE_STANDBY: return "STBY";
        case STATE_STOPPED: return "STOP";
        case STATE_STARTING: return "START";
        case STATE_RUNNING: return "RUN";
        case STATE_STOPPING: return "BRAKE";
    }
    return "----";
}

// Diagnostic dashboards can be hidden individually to keep press-and-rotate dashboard cycling short for users who only want the core views.
static bool dashboardModeEnabled(int mode) {
    switch (mode) {
        case 4: return settings.get().showCpuDashboard;
        case 5: return settings.get().showMemoryDashboard;
        case 6: return settings.get().showFlashDashboard;
        default: return true;
    }
}

// Walk until an enabled dashboard is found. At least mode 0 is always enabled.
static int nextDashboardMode(int current, int direction) {
    const int maxMode = 6;
    for (int i = 0; i <= maxMode; i++) {
        current += direction;
        if (current > maxMode) current = 0;
        if (current < 0) current = maxMode;
        if (dashboardModeEnabled(current)) return current;
    }
    return 0;
}

// Shared percentage bar for CPU, memory, flash, and filesystem gauges.
static void drawMetricBar(int x, int y, int w, int h, uint32_t used, uint32_t total) {
    display.drawRect(x, y, w, h, SSD1306_WHITE);
    if (total == 0) return;
    uint32_t fill = (used * (uint32_t)(w - 2)) / total;
    if (fill > (uint32_t)(w - 2)) fill = w - 2;
    display.fillRect(x + 1, y + 1, (int)fill, h - 2, SSD1306_WHITE);
}

// Compact byte formatting for the small OLED diagnostic pages.
static void printKilobytes(uint32_t bytes) {
    display.print(bytes / 1024UL);
    display.print("K");
}

UserInterface::UserInterface() {
    // The UI starts on the dashboard; menu pages are selected only after buildMenuSystem() creates their static objects in begin().
    _inMenu = false;
    _currentPage = nullptr;
    _screensaverActive = false;
    _saverX = 10; _saverY = 10; _saverDX = 1; _saverDY = 1;

    // Dialog text pointers always point at owned buffers after construction.
    _showingMessage = false;
    _showingConfirm = false;
    _showingError = false;
    _showingGoodbye = false;
    _messageBuffer[0] = 0;
    _confirmBuffer[0] = 0;
    _errorBuffer[0] = 0;
    _messageText = _messageBuffer;
    _confirmMsg = _confirmBuffer;
    _errorMsg = _errorBuffer;

    _statusMode = 0; // Standard

    // Transition fields are kept initialized even though current navigation snaps pages into place after starting the animation.
    _transitionProgress = 0.0;
    _transitionDirection = 0;
    _nextPage = nullptr;
    _smoothScrollY = 0.0;

    _lastBrightness = 0;
    _lissajousPhase = 0.0;

    // Randomize column starts so the matrix screensaver is not a flat line on first entry.
    for(int i=0; i<16; i++) _matrixDrops[i] = random(0, 64);

    _lastInputTime = 0;
}

void UserInterface::begin() {
    _input.begin();

    // Build once at boot. Menu pages/items are static and reused throughout the session to avoid heap churn while the controller is running.
    buildMenuSystem();

    // The splash is allowed to block because waveform generation has not started yet and setup() is still in progress.
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    const char* msg = WELCOME_MESSAGE;
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

    // Scroll the welcome text from right to left across the OLED.
    for (int x = 128; x >= -((int)w); x -= 4) {
        display.clearDisplay();
        display.setCursor(x, 25);
        display.print(msg);
        display.display();
    }

    display.clearDisplay();
    display.setCursor(30, 45);
    display.setTextSize(1);
    display.println(FIRMWARE_VERSION);
    display.display();
    delay(1000);

    // Optional dedicated buttons bypass the encoder event stream and are sampled by InputManager on every UI update.
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
    // Poll first, then route events. The rest of update() is display policy and rendering, so hardware actions happen before the current frame is drawn.
    _input.update();
    handleInput();

    // Idle timers are measured from the last physical or injected input.
    uint32_t now = millis();
    uint32_t elapsed = (now - _lastInputTime) / 1000; // Seconds

    // Auto-standby only applies when stopped; a running record should continue until explicit user or fault action stops it.
    uint8_t stbyDelay = settings.get().autoStandbyDelay;
    if (stbyDelay > 0 && !motor.isRunning() && !motor.isStandby()) {
        if (elapsed > (stbyDelay * 60)) {
            motor.toggleStandby();
            _lastInputTime = now;
        }
    }

    // Auto-dim is a running-only low-distraction dashboard mode. First input wakes the standard dashboard and is consumed in handleInput().
    uint8_t dimDelay = settings.get().autoDimDelay;
    if (dimDelay > 0 && motor.isRunning() && _statusMode != 2) {
        if (elapsed > (dimDelay * 60)) {
            _statusMode = 2; // Switch to Dim Mode
        }
    }

    // Display sleep saves OLED life when stopped/idle. Running display policy is handled by auto-dim so status remains visible while a record plays.
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

        if (!motor.isRunning() && (now - _lastInputTime > sleepMs)) {
             display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    }

    // Standby either animates a screensaver or turns the panel off, depending on the user setting. Leaving standby clears the screensaver flag.
    if (motor.isStandby()) {
        if (settings.get().screensaverEnabled) {
            _screensaverActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        } else {
            _screensaverActive = false;
            display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    } else {
        _screensaverActive = false;
    }

    // Advance any pending menu transition.
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

    // Draw exactly one frame from the highest-priority active view.
    draw();
}

void UserInterface::handleInput() {
    InputEvent evt = _input.getEvent();
    int delta = _input.getEncoderDelta();

    // Resonance sweep has a dedicated modal behavior: any exit-style press locks the current swept phase offsets, saves them, and leaves the menu.
    if (motor.isSweepingMode()) {
        if (evt == EVT_SELECT || evt == EVT_DOUBLE_CLICK || evt == EVT_BACK || evt == EVT_EXIT) {
            motor.stopSymmetricSweep(true);
            settings.save(false, true);
            showMessage("Locked & Saved!", 2000);
            exitMenu();
        }
        // Encoder rotation is intentionally ignored during the sweep so the diagnostic value is controlled only by the configured sweep range.
        return;
    }

    // Any physical or injected input wakes the panel and resets idle timers.
    if (evt != EVT_NONE || delta != 0 || _input.isButtonDown()) {
        _lastInputTime = millis();
        display.ssd1306_command(SSD1306_DISPLAYON);

        // The first input from dim mode is consumed to avoid an accidental speed change or menu action while the user is only waking the display.
        if (_statusMode == 2) {
            _statusMode = 0; // Restore to Standard
            return;
        }
    }

    // Locked mode still allows menu unlock entry, stop, and standby. Other actions show a short "UI Locked" message without changing settings.
    if (!_inMenu && networkManager.isDeviceLocked()) {
        bool speedButton = _input.isSpeedButtonPressed();
        bool startStopButton = _input.isStartStopPressed();
        bool standbyButton = _input.isStandbyPressed();

        if (evt == EVT_DOUBLE_CLICK) {
            enterMenu();
            return;
        }

        if ((evt == EVT_SELECT || startStopButton) && motor.isRunning()) {
            motor.stop();
            showMessage("Locked Stop", 1000);
            return;
        }

        if ((evt == EVT_BACK || evt == EVT_EXIT || standbyButton) && !motor.isStandby()) {
            motor.toggleStandby();
            _showingGoodbye = true;
            _goodbyeStartTime = millis();
            display.ssd1306_command(SSD1306_DISPLAYON);
            return;
        }

        if (evt != EVT_NONE || delta != 0 || speedButton || startStopButton || standbyButton) {
            showMessage("UI Locked", 1000);
        }
        return;
    }

    // Dedicated buttons are global because they are intended as direct hardware controls, not menu-only controls.
    if (_input.isSpeedButtonPressed()) {
        motor.cycleSpeed();
        // Keep the per-speed menu shadow aligned with the motor speed when the hardware speed button is used while inside the menu.
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

    // Screensaver wake consumes the input except for Select in standby, which intentionally wakes the controller out of standby as well as the display.
    if (_screensaverActive && (evt != EVT_NONE || delta != 0)) {
        _screensaverActive = false;

        if (motor.isStandby() && evt == EVT_SELECT) {
            motor.toggleStandby();
        }
        return;
    }

    // Error dialogs are dismiss-only; corrective action is already performed by the error source or by showError(muteOutputs=true).
    if (_showingError && (evt != EVT_NONE)) {
        _showingError = false;
        return;
    }

    // Confirmation dialogs own the encoder until resolved.
    if (_showingConfirm) {
        if (delta != 0) _confirmResult = !_confirmResult;
        if (evt == EVT_SELECT) {
            if (_confirmResult && _confirmAction) _confirmAction();
            _showingConfirm = false;
        }
        return;
    }

    #if PITCH_CONTROL_ENABLE
    int pitchDelta = _input.getPitchDelta();

    /*
     * In menus, the pitch encoder acts as a secondary edit control. When an item
     * is already editing it moves in coarse steps; otherwise it performs a quick
     * one-step edit and save.
     */
    if (pitchDelta != 0 && _inMenu && _currentPage) {
        MenuItem* item = _currentPage->getItem(_currentPage->getSelection());
        if (_transitionDirection == 0 && item && item->isEditable()) {
            if (item->isEditing()) {
                item->onInput(pitchDelta * 10);
            } else {
                MenuPage* dummy = nullptr;
                item->onSelect(dummy);
                item->onInput(pitchDelta);
                item->onSelect(dummy);
            }
        }
    }
    // Outside menus the pitch encoder adjusts running speed directly.
    else if (pitchDelta != 0 && motor.isRunning() && !_inMenu) {
        float step = settings.get().pitchStepSize;
        motor.adjustPitchFreq(pitchDelta * step);
    }

    // Pitch encoder switch is read directly because InputManager owns the primary encoder/button state machine.
    static bool lastPitchBtn = HIGH;
    static uint32_t pitchBtnDownTime = 0;
    bool pitchBtn = hal.digitalRead(PIN_ENC_PITCH_SW);

    if (pitchBtn == LOW && lastPitchBtn == HIGH) {
        pitchBtnDownTime = millis();
    }

    if (pitchBtn == HIGH && lastPitchBtn == LOW) {
        uint32_t duration = millis() - pitchBtnDownTime;
        if (duration >= 2000) {
            motor.resetPitch();
            showMessage("Pitch Reset", 1000);
        } else if (duration > 50) {
            motor.togglePitchRange();
            char buf[16];
            snprintf(buf, sizeof(buf), "Range: +/-%d%%", motor.getPitchRange());
            showMessage(buf, 1000);
        }
    }
    lastPitchBtn = pitchBtn;
    #endif

    // Menu pages consume encoder movement for item edit/scroll. Navigation keys only move pages when the selected item is not actively editing.
    if (_inMenu && _currentPage) {
        if (_transitionDirection != 0) return;

        if (delta != 0) {
            _currentPage->input(delta);
        }

        MenuItem* item = _currentPage->getItem(_currentPage->getSelection());
        bool editing = item && item->isEditing();

        if (!editing) {
            if (evt == EVT_NAV_UP) _currentPage->next();
            if (evt == EVT_NAV_DOWN) _currentPage->prev();
        }

        if (evt == EVT_SELECT) {
            // Link items navigate; editable/action items handle selection through their MenuItem implementation.
            MenuPage* target = item ? item->getTargetPage() : nullptr;
            if (target) {
                navigateTo(target);
            } else {
                _currentPage->select(_currentPage);
            }
        }

        if (evt == EVT_BACK) {
            item = _currentPage->getItem(_currentPage->getSelection());
            if (item && item->onBack(_currentPage)) return;
            // Relay test mode controls real relay outputs, so leaving that page always returns them to the normal off/test-complete state.
            if (_currentPage == pageRelayTest) motor.endRelayTest();
            if (_menuStack.empty()) cancelMenuChangesAndExit();
            else back();
        }

        if (evt == EVT_EXIT) saveMenuChangesAndExit();

    } else {
        // Dashboard Select toggles start/stop, except standby uses it as wake.
        if (evt == EVT_SELECT) {
            if (motor.isStandby()) {
                motor.toggleStandby(); // Wake
            } else {
                motor.toggleStartStop();
            }
        }

        // Double-click keeps menu entry distinct from the safety-critical start/stop action on a single click.
        if (evt == EVT_DOUBLE_CLICK) {
            enterMenu();
        }

        // Long press/back enters standby and shows a short goodbye animation before the panel sleeps or screensaver takes over.
        if (evt == EVT_BACK || evt == EVT_EXIT) {
            if (!motor.isStandby()) {
                motor.toggleStandby();
                _showingGoodbye = true;
                _goodbyeStartTime = millis();
                display.ssd1306_command(SSD1306_DISPLAYON);
            }
        }

        // Plain rotation changes speed; press-and-rotate cycles dashboard pages.
        if (delta != 0) {
            if (_input.isButtonDown()) {
                _statusMode = nextDashboardMode(_statusMode, delta > 0 ? 1 : -1);
            } else {
                SpeedMode s = motor.getSpeed();
                int currentIdx = (int)s;
                currentIdx += delta;
                if (currentIdx > 2) currentIdx = 2;
                if (currentIdx < 0) currentIdx = 0;
                if (currentIdx == 2 && !settings.get().enable78rpm) currentIdx = 1;

                motor.setSpeed((SpeedMode)currentIdx);
            }
        }
    }
}

void UserInterface::draw() {
    // Contrast is sent only when changed. Dim mode and screensaver own their own brightness behavior, so the normal setting is skipped there.
    if (_statusMode != 2 && !_screensaverActive) {
        uint8_t target = settings.get().displayBrightness;
        if (target != _lastBrightness) {
            display.ssd1306_command(SSD1306_SETCONTRAST);
            display.ssd1306_command(target);
            _lastBrightness = target;
        }
    }

    display.clearDisplay();

    // Draw priority is safety/status first, then modal dialogs, then the active menu/dashboard surface.
    if (_showingError) {
        drawError();
    } else if (_showingConfirm) {
        drawConfirm();
    } else if (_showingMessage) {
        drawMessage();
    } else if (_showingGoodbye) {
        drawGoodbye();
    } else if (motor.isSweepingMode()) {
        drawSweepScreen();
    } else if (_screensaverActive) {
        drawScreensaver();
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
    // ASCII mirror is throttled because a full 128x64 dump can otherwise dominate the 115200 baud serial link.
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
    // The goodbye screen is rendered after standby has been requested but before the display is allowed to go dark.
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    const char* msg = "Goodbye...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

    // Position is time-based so the animation speed is independent of loop rate.
    uint32_t elapsed = millis() - _goodbyeStartTime;
    int x = 128 - (elapsed / 10);

    display.setCursor(x, 25);
    display.print(msg);

    if (x < -((int)w)) {
        _showingGoodbye = false;
        // Screensaver-enabled standby is handled by the next update cycle.
        if (!settings.get().screensaverEnabled) {
             display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    }
}

void UserInterface::drawMenu() {
    // Transition offsets are currently zeroed by navigateTo()/back(), but the math is retained so future dual-page rendering has stable state.
    int xOffset = 0;

    if (_transitionDirection != 0) {
        float t = _transitionProgress;

        if (_transitionDirection == 1) { // Forward (Slide Left)
            xOffset = (int)(128.0 * (1.0 - t));
        } else if (_transitionDirection == -1) { // Back (Slide Right)
            xOffset = (int)(-128.0 * (1.0 - t));
        }
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Header line leaves five 10px menu rows below it.
    display.setCursor(0 + xOffset, 0);
    display.print(_currentPage->getTitle());
    display.drawLine(0 + xOffset, 10, 128 + xOffset, 10, SSD1306_WHITE);

    // Smooth scroll state is updated even though MenuPage owns the logical window; this keeps the animation hook ready without changing layout.
    int selection = _currentPage->getSelection();
    int targetY = selection * 10; // 10px per item

    _smoothScrollY += (targetY - _smoothScrollY) * 0.3;

    int total = _currentPage->getItemCount();
    int offset = _currentPage->getOffset();
    int visible = 5;

    for (int i = 0; i < visible; i++) {
        int idx = offset + i;
        if (idx >= total) break;

        MenuItem* item = _currentPage->getItem(idx);
        int y = 15 + (i * 10);

        // The selected row is inverted to stay visible in a dense OLED list.
        if (idx == selection) {
            display.fillRect(0 + xOffset, y - 1, 128, 11, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        } else {
            display.setTextColor(SSD1306_WHITE);
        }

        display.setCursor(2 + xOffset, y);
        display.print(item->getLabel());

        // Values start at x=80 by convention; labels in menu_data.cpp are kept short so both columns fit.
        char valBuf[18];
        item->getValueString(valBuf, sizeof(valBuf));
        if (valBuf[0] != 0) {
            display.setCursor(80 + xOffset, y);
            display.print(valBuf);
        }

        // Dirty marks values that differ from the active settings or shadow.
        if (item->isDirty()) {
            display.setCursor(120 + xOffset, y);
            display.print(F("*"));
        }
    }

    // Small right-edge scrollbar shows position in long pages.
    if (total > visible) {
        int sbHeight = (visible * 50) / total;
        if (sbHeight < 2) sbHeight = 2;
        int sbY = 15 + (offset * 50) / total;
        display.fillRect(126 + xOffset, sbY, 2, sbHeight, SSD1306_WHITE);
    }
}

void UserInterface::drawDashboard() {
    extern bool safeModeActive;

    // Dashboard modes can be hidden while selected through settings; advance to the next enabled page instead of leaving the OLED blank.
    if (!dashboardModeEnabled(_statusMode)) {
        _statusMode = nextDashboardMode(_statusMode, 1);
    }

    // Mode 2: minimal dim view for low-distraction playback.
    if (_statusMode == 2) {
        display.dim(true); // Low contrast

        display.setFont(NULL);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(4, 4);
        display.print(dashboardStateLabel());

        // Large speed text stays legible from across the room.
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

    // Top row is shared by every non-dim dashboard mode.
    if (safeModeActive) {
        // Safe Mode bypasses loaded settings, so it replaces normal status icons with a full-width warning banner.
        display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(35, 4);
        display.print("SAFE MODE");
        display.setTextColor(SSD1306_WHITE); // reset
    } else {
        if (motor.isRunning()) {
            display.drawBitmap(0, 0, icon_play_bits, 16, 16, SSD1306_WHITE);
        } else {
            display.drawBitmap(0, 0, icon_stop_bits, 16, 16, SSD1306_WHITE);
        }

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(18, 4);
        display.print(dashboardStateLabel());

        char freqBuf[12];
        snprintf(freqBuf, sizeof(freqBuf), "%.1fHz", motor.getCurrentFrequency());
        int16_t fx1, fy1;
        uint16_t fw, fh;
        display.getTextBounds(freqBuf, 0, 0, &fx1, &fy1, &fw, &fh);
        int freqX = 110 - (int)fw;
        if (freqX < 48) freqX = 48;
        display.setCursor(freqX, 4);
        display.print(freqBuf);

        // Lock icon means "running and no programmed ramp is active"; feedback lock status is reported through serial/web diagnostics.
        if (motor.getState() == STATE_RUNNING && !motor.isSpeedRamping()) {
            display.drawBitmap(112, 0, icon_lock_bits, 16, 16, SSD1306_WHITE);
        }
    }

    // Mode 1: runtime counters.
    if (_statusMode == 1) {
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        display.setCursor(0, 20);
        display.print("Session: ");
        uint32_t sessionSec = settings.getSessionRuntime();
        int min = sessionSec / 60;
        int sec = sessionSec % 60;
        display.print(min); display.print("m "); display.print(sec); display.print("s");

        display.setCursor(0, 35);
        display.print("Total: ");
        uint32_t totalSec = settings.getTotalRuntime();
        int hours = totalSec / 3600;
        int tMin = (totalSec % 3600) / 60;
        display.print(hours); display.print("h "); display.print(tMin); display.print("m");

        return;
    }

    // Mode 3: XY scope from the latest generated waveform samples.
    if (_statusMode == 3) {
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("SCOPE");

        display.setCursor(0, 40);
        display.print(motor.getCurrentFrequency(), 1);
        display.print("Hz");

        // 60x60 plot leaves a text column on the left.
        display.drawRect(64, 2, 60, 60, SSD1306_WHITE);

        if (motor.isRunning()) {
            // X axis is phase A, Y axis is phase B. This visualizes generated output, not measured motor feedback.
            int16_t sampleA = waveform.getSample(0);
            int16_t sampleB = waveform.getSample(1);

            // Generated diagnostic samples are roughly +/-511 at full amplitude.
            int px = 64 + 30 + (sampleA / 18);
            int py = 2 + 30 - (sampleB / 18); // Invert Y so positive is up

            if (px < 65) px = 65; if (px > 123) px = 123;
            if (py < 3) py = 3; if (py > 61) py = 61;

            display.fillRect(px-1, py-1, 3, 3, SSD1306_WHITE);
        } else {
            display.fillRect(64+29, 2+29, 3, 3, SSD1306_WHITE);
        }

        return;
    }

    // Mode 4: CPU load snapshot from SystemMonitor.
    if (_statusMode == 4) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("CPU Load");

        display.setCursor(0, 34);
        display.print("Core0 ");
        display.print(metrics.core0LoadPercent, 0);
        display.print("%");
        drawMetricBar(62, 32, 58, 7, (uint32_t)metrics.core0LoadPercent, 100);

        display.setCursor(0, 50);
        display.print("Wave  ");
        display.print(metrics.core1LoadPercent, 0);
        display.print("%");
        drawMetricBar(62, 48, 58, 7, (uint32_t)metrics.core1LoadPercent, 100);
        return;
    }

    // Mode 5: heap and PSRAM usage.
    if (_statusMode == 5) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("Memory");

        display.setCursor(0, 34);
        display.print("Heap U:");
        printKilobytes(metrics.heapUsedBytes);
        display.print(" F:");
        printKilobytes(metrics.heapFreeBytes);
        drawMetricBar(0, 43, 120, 6, metrics.heapUsedBytes, metrics.heapTotalBytes);

        display.setCursor(0, 54);
        if (metrics.psramTotalBytes > 0) {
            display.print("PSRAM U:");
            printKilobytes(metrics.psramUsedBytes);
            display.print(" F:");
            printKilobytes(metrics.psramFreeBytes);
        } else {
            display.print("Total ");
            printKilobytes(metrics.heapTotalBytes);
        }
        return;
    }

    // Mode 6: sketch flash and LittleFS usage.
    if (_statusMode == 6) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("Flash");

        display.setCursor(0, 34);
        display.print("Sketch ");
        printKilobytes(metrics.sketchUsedBytes);
        display.print("/");
        printKilobytes(metrics.sketchCapacityBytes);
        drawMetricBar(0, 43, 120, 6, metrics.sketchUsedBytes, metrics.sketchCapacityBytes);

        display.setCursor(0, 54);
        display.print("FS ");
        if (metrics.filesystemMounted) {
            printKilobytes(metrics.filesystemUsedBytes);
            display.print("/");
            printKilobytes(metrics.filesystemTotalBytes);
        } else {
            display.print("not mounted");
        }
        return;
    }

    // Mode 0: main speed view with pitch/ramp deviation.
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

    // Reset the Adafruit font before drawing small text and primitives.
    display.setFont(NULL);

    // During starts, stops, and programmed speed changes, the upper bar shows motion progress through the motor state machine.
    float motionProgress = motor.getMotionProgress();
    bool showProgress = motor.getState() == STATE_STARTING ||
                        motor.getState() == STATE_STOPPING ||
                        motor.isSpeedRamping();
    if (showProgress) {
        int barX = 18;
        int barY = 45;
        int barW = 92;
        int fillW = (int)(motionProgress * (barW - 2));
        if (fillW < 0) fillW = 0;
        if (fillW > barW - 2) fillW = barW - 2;
        display.drawRect(barX, barY, barW, 5, SSD1306_WHITE);
        display.fillRect(barX + 1, barY + 1, fillW, 3, SSD1306_WHITE);
    }

    // Bottom scale visualizes actual frequency deviation from the nominal speed.
    display.drawLine(10, 55, 118, 55, SSD1306_WHITE); // Main line
    display.drawLine(64, 52, 64, 58, SSD1306_WHITE); // Center tick
    display.drawLine(10, 52, 10, 58, SSD1306_WHITE); // Left tick
    display.drawLine(118, 52, 118, 58, SSD1306_WHITE); // Right tick

    float nominal = settings.getCurrentSpeedSettings().frequency;
    float current = motor.getCurrentFrequency();
    float deviationPercent = 0.0;

    if (nominal > 0) {
        deviationPercent = ((current - nominal) / nominal) * 100.0;
    }

    // The visual range is fixed at +/-8% so ramps and pitch changes remain comparable across speeds.
    float range = 8.0;

    int px = 64 + (int)((deviationPercent / range) * 54.0);
    if (px < 10) px = 10;
    if (px > 118) px = 118;

    display.fillTriangle(px, 50, px-3, 46, px+3, 46, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(50, 64-8);

    #if PITCH_CONTROL_ENABLE
        // With pitch control compiled in, show the requested pitch offset.
        float pitchSetting = motor.getPitchPercent();
        if (pitchSetting > 0) display.print("+");
        display.print(pitchSetting, 1);
        display.print("%");
    #else
        // Without pitch control, show ramp deviation only when it is visible.
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

    // The mode enum is stored in settings; unknown values fall back to the simple bouncing standby text.
    if (settings.get().screensaverMode == SAVER_MATRIX) {
        drawMatrixRain();
    }
    else if (settings.get().screensaverMode == SAVER_LISSAJOUS) {
        drawLissajous();
    }
    else {
        // Move text at a fixed cadence so display frame rate does not affect animation speed.
        static uint32_t lastMove = 0;
        if (millis() - lastMove > 50) {
            lastMove = millis();
            _saverX += _saverDX;
            _saverY += _saverDY;

            if (_saverX <= 0 || _saverX >= (128 - 60)) _saverDX = -_saverDX; // approximate text width
            if (_saverY <= 0 || _saverY >= (64 - 8)) _saverDY = -_saverDY;
        }

        display.setCursor(_saverX, _saverY);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.print(STANDBY_MESSAGE);
    }
}

void UserInterface::drawMatrixRain() {
    // 16 columns matches the 128px width at the default 8px text cell.
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    for (int i = 0; i < 16; i++) {
        // Lead character uses printable ASCII to avoid font lookup surprises.
        char c = (char)random(33, 126);
        display.setCursor(i * 8, _matrixDrops[i]);
        display.write(c);

        // A dot gives a cheap one-cell trail without grayscale support.
        if (_matrixDrops[i] >= 8) {
             display.setCursor(i * 8, _matrixDrops[i] - 8);
             display.write('.');
        }

        // Randomized movement keeps adjacent columns from marching in sync.
        if (random(0, 10) > 2) { // Random speed
            _matrixDrops[i] += 4;
        }

        // Reset once below the visible OLED area.
        if (_matrixDrops[i] > 64) {
            _matrixDrops[i] = 0;
        }
    }
}

void UserInterface::drawLissajous() {
    // Parametric curves use phase advance rather than saved points, keeping the screensaver allocation-free.
    _lissajousPhase += 0.05;

    int cx = 64;
    int cy = 32;
    int amp = 30;

    // Full-size curve.
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + amp * sin(3.0 * t + _lissajousPhase);
        int y = cy + amp * sin(2.0 * t);
        display.drawPixel(x, y, SSD1306_WHITE);
    }

    // Smaller counter-phase curve adds motion while staying within bounds.
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + (amp/2) * sin(2.0 * t - _lissajousPhase);
        int y = cy + (amp/2) * cos(3.0 * t);
        display.drawPixel(x, y, SSD1306_WHITE);
    }
}

void UserInterface::drawConfirm() {
    // Modal rectangle is small enough to preserve context around it.
    display.fillRect(10, 10, 108, 44, SSD1306_BLACK); // Background
    display.drawRect(10, 10, 108, 44, SSD1306_WHITE); // Border

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Confirm messages are expected to be short; showConfirm() copies/truncates.
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(_confirmMsg, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(10 + (108 - w) / 2, 25);
    display.print(_confirmMsg);

    // Encoder rotation toggles the bracketed answer in handleInput().
    display.setCursor(20, 40);
    if (_confirmResult) {
        display.print("[YES]  NO ");
    } else {
        display.print(" YES  [NO]");
    }
}

void UserInterface::drawSweepScreen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Dedicated diagnostic screen makes it clear that normal menu navigation is temporarily trapped until the sweep is locked.
    display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(5, 4);
    display.print("SWEEPING RESONANCE");
    display.setTextColor(SSD1306_WHITE);

    float ph2 = settings.getCurrentSpeedSettings().phaseOffset[1];
    float ph3 = settings.getCurrentSpeedSettings().phaseOffset[2];

    display.setCursor(0, 25);
    display.print("Phase 2: "); display.print(ph2, 1); display.print((char)247);

    if (settings.get().phaseMode == 3) {
        display.setCursor(0, 40);
        display.print("Phase 3: "); display.print(ph3, 1); display.print((char)247);
    }

    display.setCursor(0, 56);
    display.print("PRESS TO LOCK & SAVE");
}

void UserInterface::drawMessage() {
    // Timed message overlays the current screen and clears itself on expiry.
    display.fillRect(10, 15, 108, 34, SSD1306_BLACK);
    display.drawRect(10, 15, 108, 34, SSD1306_WHITE);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(15, 25);
    display.println(_messageText);

    if (millis() - _messageStartTime > _messageDuration) _showingMessage = false;
}

void UserInterface::drawError() {
    // Error modal is intentionally larger and higher contrast than a message.
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
    if (!page || page == _currentPage) return;

    // Save the current page so Back can return without rebuilding menu objects.
    if (_currentPage) _menuStack.push_back(_currentPage);

    // Start transition state for future animation support.
    _nextPage = page;
    _transitionDirection = 1; // Forward
    _transitionProgress = 0.0;

    // Current renderer draws one page, so switch immediately.
    _currentPage = page;
    _transitionDirection = 0;
}

void UserInterface::back() {
    if (!_menuStack.empty()) {
        // Pop back to the previous static page.
        _currentPage = _menuStack.back();
        _menuStack.pop_back();

        // Keep transition fields coherent even though drawMenu snaps pages.
        _transitionDirection = -1; // Back
        _transitionProgress = 0.0;
    } else {
        exitMenu();
    }
}

void UserInterface::exitMenu() {
    // Leaving the menu discards navigation history. Save/cancel decisions are handled by menu_data.cpp before this is called.
    _inMenu = false;
    _menuStack.clear();
    _currentPage = nullptr;
}

void UserInterface::enterMenu() {
    // initMenuState() refreshes shadow settings and dynamic labels before the first menu page is displayed.
    initMenuState();
    _menuStack.clear();
    _inMenu = true;
    // Locked devices enter the unlock page only; unlocked devices start at the main menu root.
    _currentPage = networkManager.isDeviceLocked() ? pageUnlock : pageMain;
}

void UserInterface::showMessage(const char* msg, uint32_t duration) {
    // Copy text so callers may pass stack buffers from command handlers.
    snprintf(_messageBuffer, sizeof(_messageBuffer), "%s", msg ? msg : "");
    _messageText = _messageBuffer;
    _messageDuration = duration;
    _messageStartTime = millis();
    _showingMessage = true;
}

void UserInterface::showConfirm(const char* msg, void (*action)()) {
    // Confirm defaults to "No" so an accidental click cannot run the action.
    snprintf(_confirmBuffer, sizeof(_confirmBuffer), "%s", msg ? msg : "");
    _confirmMsg = _confirmBuffer;
    _confirmAction = action;
    _confirmResult = false;
    _showingConfirm = true;
}

void UserInterface::showError(const char* msg, uint32_t duration, bool muteOutputs) {
    // Some error sources need immediate mute relay action but not a full motor state transition.
    snprintf(_errorBuffer, sizeof(_errorBuffer), "%s", msg ? msg : "");
    _errorMsg = _errorBuffer;
    _errorDuration = duration;
    _errorStartTime = millis();
    _showingError = true;
    if (muteOutputs) {
        motor.setRelays(false);
    }
}

void UserInterface::injectInput(int delta, bool btn) {
    // Serial commands use this to exercise the same UI path as hardware input.
    _input.injectDelta(delta);
    _input.injectButton(btn);
}
