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
#include "error_handler.h"
#include "speed_feedback.h"
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <string.h>

static int uiWidth() { return DISPLAY_LOGICAL_WIDTH; }
static int uiHeight() { return DISPLAY_LOGICAL_HEIGHT; }
static bool uiCompact() { return uiHeight() <= 72; }
static bool uiLarge() { return uiWidth() >= 200 && uiHeight() >= 160; }
static uint8_t uiTextScale() { return uiLarge() ? 2 : 1; }
static int uiHeaderHeight() { return uiCompact() ? 16 : (uiLarge() ? 30 : 20); }

static int centeredClassicX(const char* text, uint8_t scale) {
    if (!text) return 0;
    int width = (int)strlen(text) * 6 * scale;
    return (uiWidth() - width) / 2;
}

static void drawTitleBand(const char* title, bool warning = false) {
    const int height = uiHeaderHeight();
    const uint8_t scale = uiTextScale();
    display.fillRect(0, 0, uiWidth(), height, DISPLAY_WHITE);
    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_BLACK, DISPLAY_WHITE);
    int y = (height - 8 * scale) / 2;
    display.setCursor(centeredClassicX(title, scale), y);
    display.print(title);
    if (warning && uiWidth() >= 128) {
        display.fillTriangle(3, height / 2 + 5, 9, height / 2 - 5, 15, height / 2 + 5, DISPLAY_BLACK);
    }
    display.setTextColor(DISPLAY_WHITE);
}

static void drawCenteredClassic(const char* text, int y, uint8_t scale = 1) {
    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);
    display.setCursor(centeredClassicX(text, scale), y);
    display.print(text);
}

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
        case 1: return settings.get().showRuntime;
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
    display.drawRect(x, y, w, h, DISPLAY_WHITE);
    if (total == 0) return;
    uint32_t fill = (used * (uint32_t)(w - 2)) / total;
    if (fill > (uint32_t)(w - 2)) fill = w - 2;
    display.fillRect(x + 1, y + 1, (int)fill, h - 2, DISPLAY_WHITE);
}

// Compact byte formatting for the small logical diagnostic pages.
static void printKilobytes(uint32_t bytes) {
    display.print(bytes / 1024UL);
    display.print("K");
}

// Wrap short modal text without heap allocation. The classic GFX font is six pixels wide at text size 1.
static void drawWrappedText(const char* text, int x, int y, int width, int maxLines, int lineHeight,
                            uint8_t textScale = 1) {
    if (!text || width < 6 * textScale || maxLines < 1) return;

    display.setFont(NULL);
    display.setTextSize(textScale);
    const int maxChars = width / (6 * textScale);
    const char* cursor = text;
    for (int lineIndex = 0; lineIndex < maxLines && *cursor; lineIndex++) {
        while (*cursor == ' ') cursor++;
        size_t remaining = strlen(cursor);
        size_t take = remaining < (size_t)maxChars ? remaining : (size_t)maxChars;

        if (take < remaining) {
            size_t wordBreak = take;
            while (wordBreak > 0 && cursor[wordBreak] != ' ') wordBreak--;
            if (wordBreak > 0) take = wordBreak;
        }

        char line[48];
        if (take >= sizeof(line)) take = sizeof(line) - 1;
        memcpy(line, cursor, take);
        line[take] = 0;

        const char* next = cursor + take;
        while (*next == ' ') next++;
        bool truncated = lineIndex == maxLines - 1 && *next;
        if (truncated && take >= 3) {
            line[take - 3] = '.';
            line[take - 2] = '.';
            line[take - 1] = '.';
        }

        int16_t boundsX, boundsY;
        uint16_t boundsWidth, boundsHeight;
        display.getTextBounds(line, 0, 0, &boundsX, &boundsY, &boundsWidth, &boundsHeight);
        display.setCursor(x + (width - (int)boundsWidth) / 2, y + lineIndex * lineHeight);
        display.print(line);
        cursor = next;
    }
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
    _smoothScrollY = 0.0;

    _lissajousPhase = 0.0;

    // Randomize column starts so the matrix screensaver is not a flat line on first entry.
    for (size_t i = 0; i < sizeof(_matrixDrops) / sizeof(_matrixDrops[0]); i++) {
        _matrixDrops[i] = random(0, uiHeight());
    }

    _lastInputTime = 0;
}

void UserInterface::begin() {
    _input.begin();

    // Build once at boot. Menu pages/items are static and reused throughout the session to avoid heap churn while the controller is running.
    buildMenuSystem();

    // The splash is allowed to block before waveform startup, but still honours the panel frame cap. Headless and failed displays skip it.
    if (displayManager.isAvailable()) {
        display.clearDisplay();
        uint8_t splashScale = uiLarge() ? 3 : 2;
        display.setTextSize(splashScale);
        display.setTextColor(DISPLAY_WHITE);

        const char* msg = WELCOME_MESSAGE;
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

        // Scroll from right to left. Logical positions advance at 50 Hz while the manager samples only frames allowed by the physical backend.
        for (int x = DISPLAY_LOGICAL_WIDTH; x >= -((int)w); x -= 4) {
            display.clearDisplay();
            display.setCursor(x, (uiHeight() - 8 * splashScale) / 2);
            display.print(msg);
            displayManager.present();
            delay(20);
        }

        display.clearDisplay();
        display.setTextSize(uiTextScale());
        display.getTextBounds(FIRMWARE_VERSION, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((DISPLAY_LOGICAL_WIDTH - (int)w) / 2, uiHeight() - (10 * uiTextScale()));
        display.println(FIRMWARE_VERSION);
        displayManager.present(true);
        delay(1000);
    }

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
    // Poll first, then route events. Physical display transfer is deferred to render() near the end of the Core 0 loop.
    _input.update();
    handleInput();

    // Idle timers are measured from the last physical or injected input.
    uint32_t now = millis();
    uint32_t elapsed = (now - _lastInputTime) / 1000; // Seconds

    // Auto-standby only applies when stopped; a running record should continue until explicit user or fault action stops it.
    uint8_t stbyDelay = settings.get().autoStandbyDelay;
    if (stbyDelay > 0 && motor.getState() == STATE_STOPPED) {
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

    // Display sleep applies to every backend; OLEDs save panel life and TFTs can disable a wired backlight.
    bool sleepExpired = false;
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

        sleepExpired = !motor.isRunning() && (now - _lastInputTime > sleepMs);
    }

    // Standby either animates a screensaver or requests panel sleep. Critical and modal screens override that request below.
    if (motor.isStandby()) {
        if (settings.get().screensaverEnabled) {
            _screensaverActive = true;
        } else {
            _screensaverActive = false;
            sleepExpired = true;
        }
    } else {
        _screensaverActive = false;
    }

    bool attentionScreen = errorHandler.hasCriticalError() || _showingError || _showingConfirm ||
                           _showingMessage || _showingGoodbye || motor.isSweepingMode() || _inMenu;
    bool displayNeeded = attentionScreen || motor.isRunning() || _screensaverActive || !sleepExpired;
    displayManager.setPower(displayNeeded);
    displayManager.setBrightness(settings.get().displayBrightness);
    displayManager.setDimmed(_statusMode == 2 && !attentionScreen);

    // Advance any pending menu transition.
    if (_transitionDirection != 0) {
        _transitionProgress += 0.2; // Speed of slide
        if (_transitionProgress >= 1.0) {
            _transitionProgress = 0.0;
            _transitionDirection = 0;
        }
    }

}

void UserInterface::render() {
    if (!displayManager.frameDue()) return;
    draw();
}

void UserInterface::handleInput() {
    InputEvent evt = _input.getEvent();
    int delta = _input.getEncoderDelta();

    // A short/double press locks the current value; Back cancels and restores the complete pre-sweep tune.
    if (motor.isSweepingMode()) {
        if (evt == EVT_SELECT || evt == EVT_DOUBLE_CLICK) {
            motor.stopOutputSweep(true);
            if (settings.save(false, true)) {
                showMessage("Locked & Saved!", 2000);
                exitMenu();
            } else {
                showError(safeModeActive ? "Safe Mode Read Only" : "Save Failed", 2000);
            }
        } else if (evt == EVT_BACK || evt == EVT_EXIT) {
            motor.stopOutputSweep(false);
            showMessage("Sweep Cancelled", 1500);
        }
        // Encoder rotation is intentionally ignored during the sweep so the diagnostic value is controlled only by the configured sweep range.
        return;
    }

    // Any physical or injected input wakes the panel and resets idle timers.
    if (evt != EVT_NONE || delta != 0 || _input.isButtonDown()) {
        _lastInputTime = millis();
        displayManager.setPower(true);

        // The first input from dim mode is consumed to avoid an accidental speed change or menu action while the user is only waking the display.
        if (_statusMode == 2) {
            _statusMode = 0; // Restore to Standard
            return;
        }
    }

    // A stop sequence owns the active waveform until its snapshotted braking
    // parameters complete. If a remote command starts braking while the local
    // menu is open, freeze edits instead of allowing live setting pointers to
    // alter the drive envelope mid-sequence.
    if (_inMenu && motor.getState() == STATE_STOPPING) {
#if PITCH_CONTROL_ENABLE
        int ignoredPitchDelta = _input.getPitchDelta();
#endif
        bool attemptedInput = evt != EVT_NONE || delta != 0 || _input.isButtonDown();
#if PITCH_CONTROL_ENABLE
        attemptedInput = attemptedInput || ignoredPitchDelta != 0;
#endif
        if (attemptedInput) showMessage("Braking in progress", 1000);
        return;
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
            displayManager.setPower(true);
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
                displayManager.setPower(true);
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
    display.clearDisplay();
    display.setFont(NULL);
    display.setTextSize(1);
    display.setTextColor(DISPLAY_WHITE);
    display.setTextWrap(false);

    // Draw priority is safety/status first, then modal dialogs, then the active menu/dashboard surface.
    if (errorHandler.hasCriticalError()) {
        drawCriticalInterlock();
    } else if (_showingError) {
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

    displayManager.present();

    #if DUPLICATE_DISPLAY_TO_SERIAL && SERIAL_MONITOR_ENABLE
    dumpDisplayToSerial();
    #endif
}

void UserInterface::dumpDisplayToSerial() {
    // Bound the mirror to 120x32 cells so large displays cannot monopolize the serial command interface.
    static uint32_t lastDump = 0;
    if (millis() - lastDump < 1000) return;
    lastDump = millis();

    Serial.println("\n--- Display Mirror ---");
    int stepX = (uiWidth() + 119) / 120;
    int stepY = (uiHeight() + 31) / 32;
    if (stepX < 1) stepX = 1;
    if (stepY < 1) stepY = 1;
    for (int y = 0; y < uiHeight(); y += stepY) {
        for (int x = 0; x < uiWidth(); x += stepX) {
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
    uint8_t scale = uiLarge() ? 3 : 2;
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);

    const char* msg = "Goodbye...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

    // Position is time-based so the animation speed is independent of loop rate.
    uint32_t elapsed = millis() - _goodbyeStartTime;
    int x = uiWidth() - (elapsed / 10);

    display.setCursor(x, (uiHeight() - 8 * scale) / 2);
    display.print(msg);

    if (x < -((int)w)) {
        _showingGoodbye = false;
        // Screensaver-enabled standby is handled by the next update cycle.
        if (!settings.get().screensaverEnabled) {
             displayManager.setPower(false);
        }
    }
}

void UserInterface::drawMenu() {
    int xOffset = 0;
    if (_transitionDirection != 0) {
        float t = _transitionProgress;
        if (_transitionDirection == 1) xOffset = (int)(uiWidth() * (1.0f - t));
        else if (_transitionDirection == -1) xOffset = (int)(-uiWidth() * (1.0f - t));
    }

    const uint8_t scale = uiTextScale();
    const int headerHeight = uiCompact() ? 12 : uiHeaderHeight();
    const int rowHeight = uiLarge() ? 20 : (uiCompact() ? 10 : 12);
    const int listTop = headerHeight + 2;
    int visible = (uiHeight() - listTop) / rowHeight;
    if (visible < 1) visible = 1;
    _currentPage->setVisibleRows(visible);

    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);

    if (!uiCompact()) display.fillRect(xOffset, 0, uiWidth(), headerHeight, DISPLAY_WHITE);
    display.setTextColor(uiCompact() ? DISPLAY_WHITE : DISPLAY_BLACK,
                         uiCompact() ? DISPLAY_BLACK : DISPLAY_WHITE);
    display.setCursor(3 + xOffset, (headerHeight - 8 * scale) / 2);
    display.print(_currentPage->getTitle());
    display.setTextColor(DISPLAY_WHITE);
    if (uiCompact()) display.drawLine(xOffset, headerHeight - 1, uiWidth() + xOffset, headerHeight - 1, DISPLAY_WHITE);

    int selection = _currentPage->getSelection();
    int targetY = selection * rowHeight;
    _smoothScrollY += (targetY - _smoothScrollY) * 0.3;

    int total = _currentPage->getItemCount();
    int offset = _currentPage->getOffset();
    for (int i = 0; i < visible; i++) {
        int idx = offset + i;
        if (idx >= total) break;

        MenuItem* item = _currentPage->getItem(idx);
        int rowY = listTop + i * rowHeight;
        int textY = rowY + (rowHeight - 8 * scale) / 2;
        int contentRight = uiWidth() - (total > visible ? 5 : 2);

        if (idx == selection) {
            display.fillRoundRect(1 + xOffset, rowY, contentRight - 1, rowHeight - 1,
                                  uiLarge() ? 3 : 1, DISPLAY_WHITE);
            display.setTextColor(DISPLAY_BLACK, DISPLAY_WHITE);
        } else {
            display.setTextColor(DISPLAY_WHITE);
        }

        char valBuf[18];
        item->getValueString(valBuf, sizeof(valBuf));
        int valueX = contentRight;
        if (valBuf[0] != 0) {
            int16_t valueX1, valueY1;
            uint16_t valueWidth, valueHeight;
            display.getTextBounds(valBuf, 0, 0, &valueX1, &valueY1, &valueWidth, &valueHeight);
            int valueRight = contentRight - (item->isDirty() ? 8 * scale : 2);
            valueX = valueRight - (int)valueWidth;
            display.setCursor(valueX + xOffset, textY);
            display.print(valBuf);
        }

        const char* label = item->getLabel();
        int labelLimit = valBuf[0] ? valueX - 4 : contentRight - 2;
        int maxLabelChars = (labelLimit - 4) / (6 * scale);
        if (maxLabelChars < 1) maxLabelChars = 1;
        char labelBuf[40];
        size_t labelLength = strlen(label);
        size_t copyLength = labelLength < (size_t)maxLabelChars ? labelLength : (size_t)maxLabelChars;
        if (copyLength >= sizeof(labelBuf)) copyLength = sizeof(labelBuf) - 1;
        memcpy(labelBuf, label, copyLength);
        labelBuf[copyLength] = 0;
        if (labelLength > copyLength && copyLength >= 2) {
            labelBuf[copyLength - 2] = '.';
            labelBuf[copyLength - 1] = '.';
        }
        display.setCursor(4 + xOffset, textY);
        display.print(labelBuf);

        if (item->isDirty()) {
            display.setCursor(contentRight - 6 * scale + xOffset, textY);
            display.print(F("*"));
        }
    }

    if (total > visible) {
        int trackY = listTop;
        int trackHeight = uiHeight() - trackY - 1;
        int thumbHeight = (visible * trackHeight) / total;
        if (thumbHeight < 3) thumbHeight = 3;
        int travel = trackHeight - thumbHeight;
        int maxOffset = total - visible;
        int thumbY = trackY + (maxOffset > 0 ? offset * travel / maxOffset : 0);
        display.drawFastVLine(uiWidth() - 2 + xOffset, trackY, trackHeight, DISPLAY_WHITE);
        display.fillRect(uiWidth() - 3 + xOffset, thumbY, 3, thumbHeight, DISPLAY_WHITE);
    }
}

void UserInterface::drawDashboard() {
    extern bool safeModeActive;

    // Dashboard modes can be hidden while selected through settings; advance to the next enabled page instead of leaving the display blank.
    if (!dashboardModeEnabled(_statusMode)) {
        _statusMode = nextDashboardMode(_statusMode, 1);
    }

    // Mode 2: minimal dim view for low-distraction playback.
    if (_statusMode == 2) {
        display.setFont(NULL);
        display.setTextSize(uiTextScale());
        display.setTextColor(DISPLAY_WHITE);
        display.setCursor(uiLarge() ? 8 : 4, uiLarge() ? 8 : 4);
        display.print(dashboardStateLabel());

        // Dim mode deliberately removes every secondary element while scaling the speed natively for the panel.
        if (uiLarge()) display.setFont(&FreeSansBold24pt7b);
        else if (!uiCompact()) display.setFont(&FreeSansBold18pt7b);
        else display.setFont(&FreeSans12pt7b);
        display.setTextColor(DISPLAY_WHITE);
        display.setTextSize(1);

        SpeedMode s = motor.getSpeed();
        const char* speedStr;
        if (s == SPEED_33) speedStr = "33";
        else if (s == SPEED_45) speedStr = "45";
        else speedStr = "78";

        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);
        int baseline = uiCompact() ? 40 : (uiLarge() ? 145 : 78);
        display.setCursor((uiWidth() - w) / 2, baseline);
        display.print(speedStr);
        display.setFont(NULL);
        if (!uiCompact()) drawCenteredClassic("RPM", baseline + (uiLarge() ? 15 : 10), uiTextScale());
        return;
    }

    // Top row is shared by every non-dim dashboard mode.
    if (safeModeActive) {
        drawTitleBand("SAFE MODE", true);
    } else {
        const int headerHeight = uiHeaderHeight();
        const uint8_t scale = uiTextScale();
        bool inverseHeader = !uiCompact();
        if (inverseHeader) display.fillRect(0, 0, uiWidth(), headerHeight, DISPLAY_WHITE);
        uint16_t headerColor = inverseHeader ? DISPLAY_BLACK : DISPLAY_WHITE;
        display.drawBitmap(0, (headerHeight - 16) / 2,
                           motor.isRunning() ? icon_play_bits : icon_stop_bits,
                           16, 16, headerColor);

        display.setFont(NULL);
        display.setTextSize(scale);
        display.setTextColor(headerColor, inverseHeader ? DISPLAY_WHITE : DISPLAY_BLACK);
        display.setCursor(18, (headerHeight - 8 * scale) / 2);
        display.print(dashboardStateLabel());

        char freqBuf[12];
        snprintf(freqBuf, sizeof(freqBuf), "%.1fHz", motor.getCurrentFrequency());
        int16_t fx1, fy1;
        uint16_t fw, fh;
        display.getTextBounds(freqBuf, 0, 0, &fx1, &fy1, &fw, &fh);
        int frequencyRight = uiWidth() - 3;
#if CLOSED_LOOP_SPEED_ENABLE
        SpeedFeedbackStatus headerFeedback = speedFeedback.getStatus();
        if (headerFeedback.configured && headerFeedback.locked) frequencyRight -= 18;
#endif
        int freqX = frequencyRight - (int)fw;
        int minFrequencyX = 18 + 5 * 6 * scale;
        if (freqX < minFrequencyX) freqX = minFrequencyX;
        display.setCursor(freqX, (headerHeight - 8 * scale) / 2);
        display.print(freqBuf);

#if CLOSED_LOOP_SPEED_ENABLE
        if (headerFeedback.configured && headerFeedback.locked) {
            display.drawBitmap(uiWidth() - 17, (headerHeight - 16) / 2,
                               icon_lock_bits, 16, 16, headerColor);
        }
#endif
        display.setTextColor(DISPLAY_WHITE);
    }

    // Mode 1: runtime counters.
    if (_statusMode == 1) {
        display.setTextSize(1);
        display.setTextColor(DISPLAY_WHITE);

        uint32_t sessionSec = settings.getSessionRuntime();
        uint32_t totalSec = settings.getTotalRuntime();
        char session[24];
        char total[24];
        snprintf(session, sizeof(session), "%lum %lus",
                 (unsigned long)(sessionSec / 60), (unsigned long)(sessionSec % 60));
        snprintf(total, sizeof(total), "%luh %lum",
                 (unsigned long)(totalSec / 3600), (unsigned long)((totalSec % 3600) / 60));

        if (uiCompact()) {
            display.setCursor(0, 20);
            display.print("SESSION ");
            display.print(session);
            display.setCursor(0, 38);
            display.print("TOTAL   ");
            display.print(total);
        } else {
            const uint8_t scale = uiTextScale();
            const int top = uiHeaderHeight() + 6;
            const int gap = uiLarge() ? 8 : 4;
            const int cardHeight = (uiHeight() - top - gap - 5) / 2;
            const int radius = uiLarge() ? 6 : 3;
            display.drawRoundRect(4, top, uiWidth() - 8, cardHeight, radius, DISPLAY_WHITE);
            display.drawRoundRect(4, top + cardHeight + gap, uiWidth() - 8, cardHeight, radius, DISPLAY_WHITE);
            display.setTextSize(scale);
            display.setCursor(10, top + 6);
            display.print("SESSION");
            drawCenteredClassic(session, top + cardHeight / 2, scale);
            display.setCursor(10, top + cardHeight + gap + 6);
            display.print("TOTAL");
            drawCenteredClassic(total, top + cardHeight + gap + cardHeight / 2, scale);
        }

        return;
    }

    // Mode 3: XY scope from the latest generated waveform samples.
    if (_statusMode == 3) {
        display.setFont(NULL);
        display.setTextSize(uiTextScale());
        display.setTextColor(DISPLAY_WHITE);
        int plotX;
        int plotY;
        int plotSize;
        if (uiCompact()) {
            display.setCursor(0, 20);
            display.print("SCOPE");
            display.setCursor(0, 40);
            display.print(motor.getCurrentFrequency(), 1);
            display.print("Hz");
            plotX = 64;
            plotY = 2;
            plotSize = 60;
        } else {
            int footerHeight = uiLarge() ? 28 : 18;
            int availableHeight = uiHeight() - uiHeaderHeight() - footerHeight - 8;
            plotSize = uiWidth() - 16;
            if (plotSize > availableHeight) plotSize = availableHeight;
            plotX = (uiWidth() - plotSize) / 2;
            plotY = uiHeaderHeight() + 4;
            char frequency[20];
            snprintf(frequency, sizeof(frequency), "XY OUTPUT  %.1f Hz", motor.getCurrentFrequency());
            drawCenteredClassic(frequency, uiHeight() - footerHeight + 4, uiTextScale());
        }

        display.drawRect(plotX, plotY, plotSize, plotSize, DISPLAY_WHITE);
        display.drawFastHLine(plotX + 1, plotY + plotSize / 2, plotSize - 2, DISPLAY_WHITE);
        display.drawFastVLine(plotX + plotSize / 2, plotY + 1, plotSize - 2, DISPLAY_WHITE);

        if (motor.isRunning()) {
            int16_t sampleA = waveform.getSample(0);
            int16_t sampleB = waveform.getSample(1);
            int radius = (plotSize - 6) / 2;
            int px = plotX + plotSize / 2 + (int32_t)sampleA * radius / 511;
            int py = plotY + plotSize / 2 - (int32_t)sampleB * radius / 511;
            if (px < plotX + 2) px = plotX + 2;
            if (px > plotX + plotSize - 3) px = plotX + plotSize - 3;
            if (py < plotY + 2) py = plotY + 2;
            if (py > plotY + plotSize - 3) py = plotY + plotSize - 3;
            display.fillCircle(px, py, uiLarge() ? 3 : 2, DISPLAY_WHITE);
        } else {
            display.fillCircle(plotX + plotSize / 2, plotY + plotSize / 2,
                               uiLarge() ? 3 : 2, DISPLAY_WHITE);
        }

        return;
    }

    // Mode 4: CPU load snapshot from SystemMonitor.
    if (_statusMode == 4) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setFont(NULL);
        display.setTextSize(uiTextScale());
        display.setTextColor(DISPLAY_WHITE);
        if (uiCompact()) {
            display.setCursor(0, 20);
            display.print("CPU LOAD");
            display.setCursor(0, 34);
            display.print("CORE0 ");
            display.print(metrics.core0LoadPercent, 0);
            display.print("%");
            drawMetricBar(62, 32, 58, 7, (uint32_t)metrics.core0LoadPercent, 100);
            display.setCursor(0, 50);
            display.print("WAVE  ");
            display.print(metrics.core1LoadPercent, 0);
            display.print("%");
            drawMetricBar(62, 48, 58, 7, (uint32_t)metrics.core1LoadPercent, 100);
        } else {
            const uint8_t scale = uiTextScale();
            const int top = uiHeaderHeight() + 8;
            const int rowHeight = (uiHeight() - top - 4) / 2;
            const int barHeight = uiLarge() ? 14 : 8;
            char percent[12];
            display.setCursor(6, top);
            display.print("CORE 0 / UI");
            snprintf(percent, sizeof(percent), "%.0f%%", metrics.core0LoadPercent);
            display.setCursor(uiWidth() - (int)strlen(percent) * 6 * scale - 6, top);
            display.print(percent);
            drawMetricBar(6, top + 10 * scale, uiWidth() - 12, barHeight,
                          (uint32_t)metrics.core0LoadPercent, 100);
            int secondTop = top + rowHeight;
            display.setCursor(6, secondTop);
            display.print("CORE 1 / WAVE");
            snprintf(percent, sizeof(percent), "%.0f%%", metrics.core1LoadPercent);
            display.setCursor(uiWidth() - (int)strlen(percent) * 6 * scale - 6, secondTop);
            display.print(percent);
            drawMetricBar(6, secondTop + 10 * scale, uiWidth() - 12, barHeight,
                          (uint32_t)metrics.core1LoadPercent, 100);
        }
        return;
    }

    // Mode 5: heap and PSRAM usage.
    if (_statusMode == 5) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setFont(NULL);
        display.setTextSize(uiTextScale());
        display.setTextColor(DISPLAY_WHITE);
        if (uiCompact()) {
            display.setCursor(0, 20);
            display.print("MEMORY");
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
        } else {
            const uint8_t scale = uiTextScale();
            const int top = uiHeaderHeight() + 8;
            const int rowHeight = (uiHeight() - top - 4) / 2;
            const int barHeight = uiLarge() ? 14 : 8;
            char value[32];
            display.setCursor(6, top);
            display.print("HEAP");
            snprintf(value, sizeof(value), "USED %luK  FREE %luK",
                     (unsigned long)(metrics.heapUsedBytes / 1024UL),
                     (unsigned long)(metrics.heapFreeBytes / 1024UL));
            drawCenteredClassic(value, top + 10 * scale, scale);
            drawMetricBar(6, top + 20 * scale, uiWidth() - 12, barHeight,
                          metrics.heapUsedBytes, metrics.heapTotalBytes);
            int secondTop = top + rowHeight;
            display.setCursor(6, secondTop);
            display.print(metrics.psramTotalBytes > 0 ? "PSRAM" : "HEAP TOTAL");
            if (metrics.psramTotalBytes > 0) {
                snprintf(value, sizeof(value), "USED %luK  FREE %luK",
                         (unsigned long)(metrics.psramUsedBytes / 1024UL),
                         (unsigned long)(metrics.psramFreeBytes / 1024UL));
                drawCenteredClassic(value, secondTop + 10 * scale, scale);
                drawMetricBar(6, secondTop + 20 * scale, uiWidth() - 12, barHeight,
                              metrics.psramUsedBytes, metrics.psramTotalBytes);
            } else {
                snprintf(value, sizeof(value), "%luK INTERNAL RAM",
                         (unsigned long)(metrics.heapTotalBytes / 1024UL));
                drawCenteredClassic(value, secondTop + 12 * scale, scale);
            }
        }
        return;
    }

    // Mode 6: sketch flash and LittleFS usage.
    if (_statusMode == 6) {
        SystemMetricsSnapshot metrics = systemMonitor.snapshot();

        display.setFont(NULL);
        display.setTextSize(uiTextScale());
        display.setTextColor(DISPLAY_WHITE);
        if (uiCompact()) {
            display.setCursor(0, 20);
            display.print("FLASH");
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
        } else {
            const uint8_t scale = uiTextScale();
            const int top = uiHeaderHeight() + 8;
            const int rowHeight = (uiHeight() - top - 4) / 2;
            const int barHeight = uiLarge() ? 14 : 8;
            char value[32];
            display.setCursor(6, top);
            display.print("FIRMWARE");
            snprintf(value, sizeof(value), "%luK OF %luK",
                     (unsigned long)(metrics.sketchUsedBytes / 1024UL),
                     (unsigned long)(metrics.sketchCapacityBytes / 1024UL));
            drawCenteredClassic(value, top + 10 * scale, scale);
            drawMetricBar(6, top + 20 * scale, uiWidth() - 12, barHeight,
                          metrics.sketchUsedBytes, metrics.sketchCapacityBytes);
            int secondTop = top + rowHeight;
            display.setCursor(6, secondTop);
            display.print("LITTLEFS");
            if (metrics.filesystemMounted) {
                snprintf(value, sizeof(value), "%luK OF %luK",
                         (unsigned long)(metrics.filesystemUsedBytes / 1024UL),
                         (unsigned long)(metrics.filesystemTotalBytes / 1024UL));
                drawCenteredClassic(value, secondTop + 10 * scale, scale);
                drawMetricBar(6, secondTop + 20 * scale, uiWidth() - 12, barHeight,
                              metrics.filesystemUsedBytes, metrics.filesystemTotalBytes);
            } else {
                drawCenteredClassic("NOT MOUNTED", secondTop + 12 * scale, scale);
            }
        }
        return;
    }

    // Mode 0: main speed view with pitch/ramp deviation.
    if (uiLarge()) display.setFont(&FreeSansBold24pt7b);
    else if (!uiCompact()) display.setFont(&FreeSansBold18pt7b);
    else display.setFont(&FreeSans12pt7b);
    display.setTextColor(DISPLAY_WHITE);
    display.setTextSize(1);

    SpeedMode s = motor.getSpeed();
    const char* speedStr;
#if CLOSED_LOOP_SPEED_ENABLE
    char measuredSpeed[12];
    SpeedFeedbackStatus feedback = speedFeedback.getStatus();
    if (feedback.configured && feedback.signalValid) {
        snprintf(measuredSpeed, sizeof(measuredSpeed), "%.2f", feedback.filteredRpm);
        speedStr = measuredSpeed;
    } else
#endif
    if (s == SPEED_33) speedStr = "33.3";
    else if (s == SPEED_45) speedStr = "45.0";
    else speedStr = "78.0";

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);
    int speedBaseline = uiCompact() ? 40 : (uiLarge() ? 124 : 72);
    display.setCursor((uiWidth() - w) / 2, speedBaseline);
    display.print(speedStr);

    // Reset the Adafruit font before drawing small text and primitives.
    display.setFont(NULL);
    if (!uiCompact()) {
        drawCenteredClassic("RPM", speedBaseline + (uiLarge() ? 12 : 8), uiTextScale());
    }

    // During starts, stops, and programmed speed changes, the upper bar shows motion progress through the motor state machine.
    float motionProgress = motor.getMotionProgress();
    bool showProgress = motor.getState() == STATE_STARTING ||
                        motor.getState() == STATE_STOPPING ||
                        motor.isSpeedRamping();
    if (showProgress) {
        int barX = uiLarge() ? 24 : 18;
        int barY = uiCompact() ? 45 : speedBaseline + (uiLarge() ? 32 : 20);
        int barW = uiWidth() - 2 * barX;
        int barH = uiLarge() ? 8 : 5;
        int fillW = (int)(motionProgress * (barW - 2));
        if (fillW < 0) fillW = 0;
        if (fillW > barW - 2) fillW = barW - 2;
        display.drawRoundRect(barX, barY, barW, barH, barH / 2, DISPLAY_WHITE);
        display.fillRect(barX + 1, barY + 1, fillW, barH - 2, DISPLAY_WHITE);
    }

    // Bottom scale visualizes actual frequency deviation from the nominal speed.
    int meterLeft = uiLarge() ? 20 : 10;
    int meterRight = uiWidth() - meterLeft;
    int meterCenter = uiWidth() / 2;
    int meterY = uiCompact() ? 55 : (uiLarge() ? uiHeight() - 32 : uiHeight() - 20);
    int tickHeight = uiLarge() ? 9 : 6;
    display.drawLine(meterLeft, meterY, meterRight, meterY, DISPLAY_WHITE);
    display.drawLine(meterCenter, meterY - tickHeight / 2, meterCenter, meterY + tickHeight / 2, DISPLAY_WHITE);
    display.drawLine(meterLeft, meterY - tickHeight / 2, meterLeft, meterY + tickHeight / 2, DISPLAY_WHITE);
    display.drawLine(meterRight, meterY - tickHeight / 2, meterRight, meterY + tickHeight / 2, DISPLAY_WHITE);

    float nominal = settings.getCurrentSpeedSettings().frequency;
    float current = motor.getCurrentFrequency();
    float deviationPercent = 0.0;

    if (nominal > 0) {
        deviationPercent = ((current - nominal) / nominal) * 100.0;
    }

    // The visual range is fixed at +/-8% so ramps and pitch changes remain comparable across speeds.
    float range = 8.0;

    int meterRadius = (meterRight - meterLeft) / 2;
    int px = meterCenter + (int)((deviationPercent / range) * meterRadius);
    if (px < meterLeft) px = meterLeft;
    if (px > meterRight) px = meterRight;

    int pointerHeight = uiLarge() ? 8 : 5;
    int pointerHalfWidth = uiLarge() ? 5 : 3;
    display.fillTriangle(px, meterY - 2, px - pointerHalfWidth, meterY - pointerHeight,
                         px + pointerHalfWidth, meterY - pointerHeight, DISPLAY_WHITE);

    char deviationText[20];

    #if PITCH_CONTROL_ENABLE
        // With pitch control compiled in, show the requested pitch offset.
        float pitchSetting = motor.getPitchPercent();
        snprintf(deviationText, sizeof(deviationText), "%+.1f%%", pitchSetting);
    #else
        // Without pitch control, show ramp deviation only when it is visible.
        if (abs(deviationPercent) > 0.1) {
             snprintf(deviationText, sizeof(deviationText), "%+.1f%%", deviationPercent);
        } else {
             snprintf(deviationText, sizeof(deviationText), "LOCKED");
        }
    #endif
    drawCenteredClassic(deviationText, meterY + (uiLarge() ? 8 : 2), uiLarge() ? 2 : 1);
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
        uint8_t saverScale = uiLarge() ? 2 : 1;
        display.setFont(NULL);
        display.setTextSize(saverScale);
        display.setTextColor(DISPLAY_WHITE);
        // Move text at a fixed cadence so display frame rate does not affect animation speed.
        static uint32_t lastMove = 0;
        if (millis() - lastMove > 50) {
            lastMove = millis();
            _saverX += _saverDX;
            _saverY += _saverDY;

            int16_t textX, textY;
            uint16_t textWidth, textHeight;
            display.getTextBounds(STANDBY_MESSAGE, 0, 0, &textX, &textY, &textWidth, &textHeight);
            int maxX = DISPLAY_LOGICAL_WIDTH - (int)textWidth;
            int maxY = DISPLAY_LOGICAL_HEIGHT - (int)textHeight;
            if (maxX < 0) maxX = 0;
            if (maxY < 0) maxY = 0;
            if (_saverX <= 0 || _saverX >= maxX) _saverDX = -_saverDX;
            if (_saverY <= 0 || _saverY >= maxY) _saverDY = -_saverDY;
            if (_saverX < 0) _saverX = 0;
            if (_saverX > maxX) _saverX = maxX;
            if (_saverY < 0) _saverY = 0;
            if (_saverY > maxY) _saverY = maxY;
        }

        display.setCursor(_saverX, _saverY);
        display.print(STANDBY_MESSAGE);
    }
}

void UserInterface::drawMatrixRain() {
    display.setTextSize(1);
    display.setTextColor(DISPLAY_WHITE);

    const int columns = sizeof(_matrixDrops) / sizeof(_matrixDrops[0]);
    for (int i = 0; i < columns; i++) {
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

        // Reset once below the visible logical display area.
        if (_matrixDrops[i] > uiHeight()) {
            _matrixDrops[i] = 0;
        }
    }
}

void UserInterface::drawLissajous() {
    // Parametric curves use phase advance rather than saved points, keeping the screensaver allocation-free.
    _lissajousPhase += 0.05;

    int cx = uiWidth() / 2;
    int cy = uiHeight() / 2;
    int amp = (uiWidth() < uiHeight() ? uiWidth() : uiHeight()) / 2 - 3;

    // Full-size curve.
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + amp * sin(3.0 * t + _lissajousPhase);
        int y = cy + amp * sin(2.0 * t);
        display.drawPixel(x, y, DISPLAY_WHITE);
    }

    // Smaller counter-phase curve adds motion while staying within bounds.
    for (float t = 0; t < 2 * PI; t += 0.1) {
        int x = cx + (amp/2) * sin(2.0 * t - _lissajousPhase);
        int y = cy + (amp/2) * cos(3.0 * t);
        display.drawPixel(x, y, DISPLAY_WHITE);
    }
}

void UserInterface::drawConfirm() {
    const uint8_t scale = uiTextScale();
    const int boxWidth = uiCompact() ? 108 : (uiLarge() ? 216 : uiWidth() - 12);
    const int boxHeight = uiCompact() ? 44 : (uiLarge() ? 150 : 88);
    const int boxX = (uiWidth() - boxWidth) / 2;
    const int boxY = (uiHeight() - boxHeight) / 2;
    display.fillRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_BLACK);
    display.drawRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_WHITE);
    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);
    if (!uiCompact()) drawCenteredClassic("CONFIRM", boxY + 7, scale);
    int messageY = uiCompact() ? boxY + 8 : boxY + (uiLarge() ? 34 : 24);
    drawWrappedText(_confirmMsg, boxX + 6, messageY, boxWidth - 12,
                    uiCompact() ? 2 : 3, 9 * scale, scale);

    if (uiCompact()) {
        display.setCursor(boxX + 10, boxY + boxHeight - 14);
        display.print(_confirmResult ? "[YES]  NO " : " YES  [NO]");
    } else {
        int gap = uiLarge() ? 12 : 6;
        int buttonWidth = (boxWidth - 18 - gap) / 2;
        int buttonHeight = 12 * scale;
        int buttonY = boxY + boxHeight - buttonHeight - 8;
        const char* labels[2] = {"YES", "NO"};
        for (int i = 0; i < 2; i++) {
            bool selected = i == (_confirmResult ? 0 : 1);
            int buttonX = boxX + 9 + i * (buttonWidth + gap);
            if (selected) display.fillRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 3, DISPLAY_WHITE);
            else display.drawRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 3, DISPLAY_WHITE);
            display.setTextColor(selected ? DISPLAY_BLACK : DISPLAY_WHITE,
                                 selected ? DISPLAY_WHITE : DISPLAY_BLACK);
            int textX = buttonX + (buttonWidth - (int)strlen(labels[i]) * 6 * scale) / 2;
            display.setCursor(textX, buttonY + (buttonHeight - 8 * scale) / 2);
            display.print(labels[i]);
        }
        display.setTextColor(DISPLAY_WHITE);
    }
}

void UserInterface::drawSweepScreen() {
    display.clearDisplay();
    display.setFont(NULL);
    display.setTextSize(uiTextScale());
    display.setTextColor(DISPLAY_WHITE);

    drawTitleBand(uiCompact() ? "RESONANCE SWEEP" : "SWEEPING RESONANCE");
    const uint8_t scale = uiTextScale();
    int top = uiHeaderHeight() + (uiLarge() ? 24 : 9);
    display.setTextSize(scale);
    display.setCursor(uiLarge() ? 12 : 2, top);
    display.print(motor.getOutputSweepParameterName());
    display.print(": "); display.print(motor.getOutputSweepValue(), 1);
    if (motor.getOutputSweepParameter() <= MotorController::SWEEP_PHASE_D) display.print((char)247);

    drawCenteredClassic("LISTEN / MEASURE", top + 16 * scale, scale);
    drawCenteredClassic(uiCompact() ? "PRESS=LOCK HOLD=BACK" : "PRESS TO LOCK  /  HOLD TO CANCEL",
                        uiHeight() - 10 * scale, scale);
}

void UserInterface::drawMessage() {
    const uint8_t scale = uiTextScale();
    const int boxWidth = uiCompact() ? 108 : (uiLarge() ? 210 : uiWidth() - 12);
    const int boxHeight = uiCompact() ? 34 : (uiLarge() ? 94 : 66);
    const int boxX = (uiWidth() - boxWidth) / 2;
    const int boxY = (uiHeight() - boxHeight) / 2;
    display.fillRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_BLACK);
    display.drawRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_WHITE);
    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);
    if (!uiCompact()) drawCenteredClassic("NOTICE", boxY + 7, scale);
    drawWrappedText(_messageText, boxX + 6,
                    uiCompact() ? boxY + 9 : boxY + (uiLarge() ? 30 : 24),
                    boxWidth - 12, uiCompact() ? 2 : 3, 10 * scale, scale);

    if (millis() - _messageStartTime > _messageDuration) _showingMessage = false;
}

void UserInterface::drawError() {
    const uint8_t scale = uiTextScale();
    const int boxWidth = uiWidth() - (uiLarge() ? 24 : 10);
    const int boxHeight = uiCompact() ? 54 : (uiLarge() ? 140 : 96);
    const int boxX = (uiWidth() - boxWidth) / 2;
    const int boxY = (uiHeight() - boxHeight) / 2;
    display.fillRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_BLACK);
    display.drawRoundRect(boxX, boxY, boxWidth, boxHeight, uiLarge() ? 8 : 4, DISPLAY_WHITE);
    display.setFont(NULL);
    display.setTextSize(uiCompact() ? 2 : scale);
    display.setTextColor(DISPLAY_WHITE);
    drawCenteredClassic("ERROR", boxY + (uiCompact() ? 5 : 9), uiCompact() ? 2 : scale);
    display.setTextSize(scale);
    drawWrappedText(_errorMsg, boxX + 6, boxY + (uiCompact() ? 29 : (uiLarge() ? 38 : 28)),
                    boxWidth - 12, uiCompact() ? 2 : 4, 10 * scale, scale);

    if (millis() - _errorStartTime > _errorDuration) _showingError = false;
}

void UserInterface::drawCriticalInterlock() {
    drawTitleBand("OUTPUT LOCKED", true);
    const uint8_t scale = uiTextScale();
    display.setFont(NULL);
    display.setTextSize(scale);
    display.setTextColor(DISPLAY_WHITE);
    int top = uiHeaderHeight() + (uiCompact() ? 5 : 10);
    display.setCursor(uiLarge() ? 12 : 2, top);
    display.print("FAULT ");
    display.print((int)errorHandler.getCriticalCode());

    const char* message = errorHandler.getCriticalMessage();
    drawWrappedText(message ? message : "Critical fault", uiLarge() ? 12 : 2,
                    top + 12 * scale, uiWidth() - (uiLarge() ? 24 : 4),
                    uiCompact() ? 1 : 4, 10 * scale, scale);
    drawCenteredClassic(uiCompact() ? "CORRECT FAULT + REBOOT" : "CORRECT THE FAULT, THEN REBOOT",
                        uiHeight() - 10 * scale, scale);
}

void UserInterface::navigateTo(MenuPage* page) {
    if (!page || page == _currentPage) return;

    // Save the current page so Back can return without rebuilding menu objects.
    if (_currentPage) _menuStack.push_back(_currentPage);

    // Switch to the destination and slide it into view.
    _currentPage = page;
    _transitionDirection = 1; // Forward
    _transitionProgress = 0.0;
}

void UserInterface::back() {
    if (!_menuStack.empty()) {
        // Pop back to the previous static page.
        _currentPage = _menuStack.back();
        _menuStack.pop_back();

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
    if (motor.getState() == STATE_STOPPING) {
        showMessage("Braking in progress", 1200);
        return;
    }
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
