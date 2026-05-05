/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "menu_system.h"

static char* copyMenuString(const char* text) {
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
    // Delete all dynamically allocated items to prevent memory leaks
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
    
    // Scroll logic: Keep selection within the visible window (assuming 5 lines)
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
    // Trigger the selected item's action
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
        // Clamp value
        if (_temp < _min) _temp = _min;
        if (_temp > _max) _temp = _max;
        // Live preview (optional, currently enabled)
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

// --- MenuFloat (Float Editor) ---
MenuFloat::MenuFloat(const char* label, float* target, float step, float min, float max)
    : MenuSetting<float>(label, target, min, max), _step(step) {}

void MenuFloat::onInput(int delta) {
    if (_editing) {
        _temp += (delta * _step);
        // Clamp value
        if (_temp < _min) _temp = _min;
        if (_temp > _max) _temp = _max;
        *_target = _temp;
    }
}

void MenuFloat::getValueString(char* buffer, size_t size) const {
    // Format to 2 decimal places
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

// --- MenuText (String Editor) ---
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
        // Enter Edit Mode
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

        // Advance Cursor or Save
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
        
        // Find current index in our charset
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
        
        // If we just changed the null terminator (end of string), extend string
        if (_cursorPos == tempLen && _cursorPos < _maxLength) {
            _temp[_cursorPos + 1] = 0;
        }
    }
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
