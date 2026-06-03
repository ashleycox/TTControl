/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "menu_system.h"

static char* copyMenuString(const char* text) {
    // Menu items own labels so generated/dynamic labels can outlive the caller's temporary String.
    if (!text) text = "";
    size_t length = strlen(text);
    char* copy = new char[length + 1];
    memcpy(copy, text, length + 1);
    return copy;
}

// --- MenuItem Base Class ---
MenuItem::MenuItem(const char* label) : _label(copyMenuString(label)) {}

MenuItem::~MenuItem() {
    delete[] _label;
}

void MenuItem::setLabel(const char* label) {
    delete[] _label;
    _label = copyMenuString(label);
}

void MenuItem::setVisibleWhen(VisibilityCallback callback) {
    _visibleWhen = callback;
}

bool MenuItem::isVisible() const {
    return !_visibleWhen || _visibleWhen();
}

// --- MenuPage Container ---
MenuPage::MenuPage(const char* title) : _title(copyMenuString(title)), _selection(0), _offset(0) {}

MenuPage::~MenuPage() {
    clear();
    delete[] _title;
}

void MenuPage::addItem(MenuItem* item) {
    _items.push_back(item);
}

void MenuPage::clear() {
    // Delete all dynamically allocated items to prevent memory leaks.
    for (auto item : _items) {
        delete item;
    }
    _items.clear();
    _selection = 0;
    _offset = 0;
}

MenuItem* MenuPage::getItem(size_t index) const {
    return getVisibleItem(index);
}

void MenuPage::next() {
    clampSelection();
    int count = (int)getVisibleItemCount();
    if (count == 0) return;
    
    // Cycle selection forward
    _selection++;
    if (_selection >= count) _selection = 0;
    
    // Scroll logic: keep selection within the visible window used by ui.cpp.
    if (_selection >= _offset + 5) _offset = _selection - 4;
    if (_selection < _offset) _offset = _selection;
}

void MenuPage::prev() {
    clampSelection();
    int count = (int)getVisibleItemCount();
    if (count == 0) return;
    
    // Cycle selection backward
    _selection--;
    if (_selection < 0) _selection = count - 1;
    
    // Scroll logic
    if (_selection >= _offset + 5) _offset = _selection - 4;
    if (_selection < _offset) _offset = _selection;
}

void MenuPage::select(MenuPage*& currentPage) {
    clampSelection();
    MenuItem* item = getVisibleItem(_selection);
    if (!item) return;
    // Trigger the selected item's action or page transition.
    item->onSelect(currentPage);
}

void MenuPage::input(int delta) {
    clampSelection();
    MenuItem* item = getVisibleItem(_selection);
    if (!item) return;
    // Pass encoder input to the selected item (if it supports editing)
    item->onInput(delta);
}

size_t MenuPage::getVisibleItemCount() const {
    // Visibility can change while the same page is open, so counts are computed from callbacks instead of cached.
    size_t count = 0;
    for (auto item : _items) {
        if (item && item->isVisible()) count++;
    }
    return count;
}

MenuItem* MenuPage::getVisibleItem(size_t index) const {
    size_t visibleIndex = 0;
    for (auto item : _items) {
        if (!item || !item->isVisible()) continue;
        if (visibleIndex == index) return item;
        visibleIndex++;
    }
    return nullptr;
}

void MenuPage::clampSelection() {
    // Called before navigation/render interaction so hidden items cannot leave selection pointing past the visible list.
    int count = (int)getVisibleItemCount();
    if (count <= 0) {
        _selection = 0;
        _offset = 0;
        return;
    }
    if (_selection < 0) _selection = 0;
    if (_selection >= count) _selection = count - 1;
    if (_offset < 0) _offset = 0;
    if (_offset > _selection) _offset = _selection;
    if (_selection >= _offset + 5) _offset = _selection - 4;
    int maxOffset = count > 5 ? count - 5 : 0;
    if (_offset > maxOffset) _offset = maxOffset;
}

// --- MenuAction (Callback Trigger) ---
MenuAction::MenuAction(const char* label, ActionCallback callback) 
    : MenuItem(label), _callback(callback) {}

void MenuAction::onSelect(MenuPage*& currentPage) {
    if (_callback) _callback();
}

// --- MenuNav (Submenu Link) ---
MenuNav::MenuNav(const char* label, MenuPage* target) 
    : MenuItem(label), _target(target) {}

void MenuNav::onSelect(MenuPage*& currentPage) {
    if (_target) currentPage = _target;
}

// --- MenuInt (Integer Editor) ---
MenuInt::MenuInt(const char* label, int* target, int min, int max)
    : MenuSetting<int>(label, target, min, max) {}

void MenuInt::onInput(int delta) {
    if (_editing) {
        _temp += delta;
        // Clamp value.
        if (_temp < _min) _temp = _min;
        if (_temp > _max) _temp = _max;
        // Live preview keeps the UI and setting value in sync during editing.
        *_target = _temp; 
    }
}

void MenuInt::getValueString(char* buffer, size_t size) const {
    snprintf(buffer, size, "%d", _editing ? _temp : *_target);
}

// --- MenuByte (uint8_t Editor) ---
MenuByte::MenuByte(const char* label, uint8_t* target, int min, int max)
    : MenuByte(label, target, min, max, nullptr, 0, nullptr) {}

MenuByte::MenuByte(const char* label, uint8_t* target, int min, int max,
                   const char* const* labels, size_t labelCount)
    : MenuByte(label, target, min, max, labels, labelCount, nullptr) {}

MenuByte::MenuByte(const char* label, uint8_t* target, int min, int max,
                   const char* const* labels, size_t labelCount, ChangeCallback callback)
    : MenuItem(label),
      _target(target),
      _min(min),
      _max(max),
      _temp(*target),
      _editing(false),
      _labels(labels),
      _labelCount(labelCount),
      _changeCallback(callback) {}

void MenuByte::onSelect(MenuPage*& currentPage) {
    // Selecting enters/exits edit mode. Exit commits the last previewed value and notifies callbacks such as immediate waveform refresh.
    _editing = !_editing;
    if (_editing) {
        _temp = *_target;
    } else {
        *_target = (uint8_t)_temp;
        if (_changeCallback) _changeCallback((uint8_t)_temp);
    }
}

void MenuByte::onInput(int delta) {
    if (_editing) {
        _temp += delta;
        if (_temp < _min) _temp = _min;
        if (_temp > _max) _temp = _max;
        *_target = (uint8_t)_temp;
        if (_changeCallback) _changeCallback((uint8_t)_temp);
    }
}

bool MenuByte::onBack(MenuPage*& currentPage) {
    if (!_editing) return false;
    // Same live-preview semantics as MenuSetting: back accepts current temp.
    *_target = (uint8_t)_temp;
    if (_changeCallback) _changeCallback((uint8_t)_temp);
    _editing = false;
    return true;
}

void MenuByte::getValueString(char* buffer, size_t size) const {
    int value = _editing ? _temp : (int)*_target;
    if (_labels && value >= 0 && (size_t)value < _labelCount && _labels[value]) {
        snprintf(buffer, size, "%s", _labels[value]);
    } else {
        snprintf(buffer, size, "%d", value);
    }
}

void MenuByte::setChangeCallback(ChangeCallback callback) {
    _changeCallback = callback;
}

// --- MenuUInt16 (Unsigned 16-bit Editor) ---
MenuUInt16::MenuUInt16(const char* label, uint16_t* target, uint16_t step, uint16_t min, uint16_t max)
    : MenuSetting<uint16_t>(label, target, min, max), _step(step) {}

void MenuUInt16::onInput(int delta) {
    if (_editing) {
        int32_t value = (int32_t)_temp + ((int32_t)delta * (int32_t)_step);
        if (value < (int32_t)_min) value = _min;
        if (value > (int32_t)_max) value = _max;
        _temp = (uint16_t)value;
        *_target = _temp;
    }
}

void MenuUInt16::getValueString(char* buffer, size_t size) const {
    snprintf(buffer, size, "%u", (unsigned)(_editing ? _temp : *_target));
}

// --- MenuFloat (Float Editor) ---
MenuFloat::MenuFloat(const char* label, float* target, float step, float min, float max)
    : MenuSetting<float>(label, target, min, max), _step(step) {}

void MenuFloat::onInput(int delta) {
    if (_editing) {
        _temp += (delta * _step);
        // Clamp value.
        if (_temp < _min) _temp = _min;
        if (_temp > _max) _temp = _max;
        *_target = _temp;
    }
}

void MenuFloat::getValueString(char* buffer, size_t size) const {
    // Two decimals fit the OLED and are adequate for current motor settings.
    snprintf(buffer, size, "%.2f", _editing ? _temp : *_target);
}

// --- MenuBool (Toggle Editor) ---
MenuBool::MenuBool(const char* label, bool* target)
    : MenuSetting<bool>(label, target, false, true) {}

void MenuBool::onInput(int delta) {
    if (_editing && delta != 0) {
        _temp = !_temp;
        *_target = _temp;
    }
}

void MenuBool::getValueString(char* buffer, size_t size) const {
    bool val = _editing ? _temp : *_target;
    snprintf(buffer, size, "%s", val ? "ON" : "OFF");
}

/*
 * --- MenuText (String Editor) ---
 * Internal one-byte tokens are used so the encoder can offer a Shift action and
 * a pound-sign character without making the editable buffer variable-width.
 */
static const char MENU_TEXT_SHIFT_TOKEN = 1;
static const char MENU_TEXT_POUND_TOKEN = 2;
static const char MENU_TEXT_POUND_UTF8[] = "\xC2\xA3";
static const char MENU_TEXT_LOWER_CHARSET[] =
    "\x01"
    " abcdefghijklmnopqrstuvwxyz0123456789"
    "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~"
    "\x02";
static const char MENU_TEXT_UPPER_CHARSET[] =
    "\x01"
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~"
    "\x02";

static const char* menuTextCharset(bool uppercase) {
    return uppercase ? MENU_TEXT_UPPER_CHARSET : MENU_TEXT_LOWER_CHARSET;
}

static void appendMenuTextDisplayChar(char c, char* buffer, size_t size, size_t& out) {
    // Translate internal edit tokens into printable OLED/serial text.
    if (!buffer || out >= size) return;

    if (c == MENU_TEXT_SHIFT_TOKEN) {
        const char* text = "^";
        while (*text && out + 1 < size) buffer[out++] = *text++;
    } else if (c == MENU_TEXT_POUND_TOKEN) {
        const char* text = MENU_TEXT_POUND_UTF8;
        while (*text && out + 1 < size) buffer[out++] = *text++;
    } else if ((uint8_t)c >= 32 && out + 1 < size) {
        buffer[out++] = c;
    }
}

MenuText::MenuText(const char* label, char* target, size_t maxLength)
    : MenuItem(label), _target(target), _maxLength(maxLength), _editing(false), _uppercase(false), _cursorPos(0) {
    _temp = new char[maxLength + 1];
    loadTargetIntoTemp();
}

MenuText::~MenuText() {
    delete[] _temp;
}

void MenuText::onSelect(MenuPage*& currentPage) {
    if (!_editing) {
        // Enter edit mode from the start of the temp buffer.
        _editing = true;
        _uppercase = false;
        loadTargetIntoTemp();
        _cursorPos = 0;
    } else {
        if (_temp[_cursorPos] == MENU_TEXT_SHIFT_TOKEN) {
            _uppercase = !_uppercase;
            _temp[_cursorPos] = _uppercase ? 'A' : 'a';
            if (_cursorPos == strlen(_temp) && _cursorPos < _maxLength) {
                _temp[_cursorPos + 1] = 0;
            }
            return;
        }

        // Advance cursor while inside the string; selecting at the end saves.
        if (_cursorPos < strlen(_temp) && _cursorPos < _maxLength - 1) {
            _cursorPos++;
        } else {
            // Save and Exit
            saveTempToTarget();
            _editing = false;
        }
    }
}

void MenuText::onInput(int delta) {
    if (_editing) {
        size_t tempLen = strlen(_temp);
        if (_cursorPos > tempLen) _cursorPos = tempLen;
        char c = (_cursorPos < tempLen) ? _temp[_cursorPos] : ' ';
        
        // Find current index in the active charset, then wrap encoder movement through all available characters.
        const char* charset = menuTextCharset(_uppercase);
        const int charsetLen = strlen(charset);
        int idx = 0;
        for (int i=0; i<charsetLen; i++) {
            if (charset[i] == c) { idx = i; break; }
        }
        
        idx += delta;
        while (idx < 0) idx += charsetLen;
        while (idx >= charsetLen) idx -= charsetLen;
        
        _temp[_cursorPos] = charset[idx];
        
        // If we just changed the null terminator, extend the editable string by one character.
        if (_cursorPos == tempLen && _cursorPos < _maxLength) {
            _temp[_cursorPos + 1] = 0;
        }
    }
}

bool MenuText::onBack(MenuPage*& currentPage) {
    if (!_editing) return false;
    saveTempToTarget();
    _editing = false;
    return true;
}

void MenuText::getValueString(char* buffer, size_t size) const {
    if (!buffer || size == 0) return;

    if (_editing) {
        if (_temp[_cursorPos] == MENU_TEXT_SHIFT_TOKEN) {
            snprintf(buffer, size, "%s Shift", _uppercase ? "U" : "L");
            return;
        }

        size_t out = 0;
        if (out + 1 < size) buffer[out++] = _uppercase ? 'U' : 'L';
        if (out + 1 < size) buffer[out++] = ':';
        for (size_t i = 0; _temp[i] && out + 1 < size; i++) {
            appendMenuTextDisplayChar(_temp[i], buffer, size, out);
        }
        buffer[out] = 0;
    } else {
        snprintf(buffer, size, "%s", _target);
    }
}

bool MenuText::isDirty() const {
    // Compare target text to temp text while treating internal pound token as either UTF-8 pound or legacy single-byte pound.
    size_t targetIndex = 0;
    size_t tempIndex = 0;

    while (_temp[tempIndex] != 0) {
        char c = _temp[tempIndex++];
        if (c == MENU_TEXT_SHIFT_TOKEN) continue;

        if (c == MENU_TEXT_POUND_TOKEN) {
            if ((uint8_t)_target[targetIndex] == 0xC2 &&
                (uint8_t)_target[targetIndex + 1] == 0xA3) {
                targetIndex += 2;
                continue;
            }
            if ((uint8_t)_target[targetIndex] == 0xA3) {
                targetIndex++;
                continue;
            }
            return true;
        }

        if (_target[targetIndex++] != c) return true;
    }

    return _target[targetIndex] != 0;
}

void MenuText::loadTargetIntoTemp() {
    // Convert target text into the single-byte token representation used while editing.
    size_t out = 0;
    for (size_t i = 0; _target && _target[i] && out < _maxLength; i++) {
        uint8_t c = (uint8_t)_target[i];
        if (c == 0xC2 && (uint8_t)_target[i + 1] == 0xA3) {
            _temp[out++] = MENU_TEXT_POUND_TOKEN;
            i++;
        } else if (c == 0xA3) {
            _temp[out++] = MENU_TEXT_POUND_TOKEN;
        } else {
            _temp[out++] = (char)c;
        }
    }
    _temp[out] = 0;
}

void MenuText::saveTempToTarget() {
    // Convert temp tokens back into a normal null-terminated target string.
    size_t out = 0;
    for (size_t i = 0; _temp[i] && out < _maxLength; i++) {
        char c = _temp[i];
        if (c == MENU_TEXT_SHIFT_TOKEN) continue;

        if (c == MENU_TEXT_POUND_TOKEN) {
            if (out + 2 > _maxLength) break;
            _target[out++] = MENU_TEXT_POUND_UTF8[0];
            _target[out++] = MENU_TEXT_POUND_UTF8[1];
        } else {
            _target[out++] = c;
        }
    }
    _target[out] = 0;
    loadTargetIntoTemp();
}
