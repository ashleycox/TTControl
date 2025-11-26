/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "menu_system.h"

// --- MenuItem Base Class ---
MenuItem::MenuItem(const char* label) : _label(label) {}

// --- MenuPage Container ---
MenuPage::MenuPage(const char* title) : _title(title), _selection(0), _offset(0) {}

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
    if (index < _items.size()) return _items[index];
    return nullptr;
}

void MenuPage::next() {
    if (_items.empty()) return;
    
    // Cycle selection forward
    _selection++;
    if (_selection >= _items.size()) _selection = 0;
    
    // Scroll logic: Keep selection within the visible window (assuming 5 lines)
    if (_selection >= _offset + 5) _offset = _selection - 4;
    if (_selection < _offset) _offset = _selection;
}

void MenuPage::prev() {
    if (_items.empty()) return;
    
    // Cycle selection backward
    _selection--;
    if (_selection < 0) _selection = _items.size() - 1;
    
    // Scroll logic
    if (_selection >= _offset + 5) _offset = _selection - 4;
    if (_selection < _offset) _offset = _selection;
}

void MenuPage::select(MenuPage*& currentPage) {
    if (_items.empty()) return;
    // Trigger the selected item's action
    _items[_selection]->onSelect(currentPage);
}

void MenuPage::input(int delta) {
    if (_items.empty()) return;
    // Pass encoder input to the selected item (if it supports editing)
    _items[_selection]->onInput(delta);
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
MenuText::MenuText(const char* label, char* target, size_t maxLength)
    : MenuItem(label), _target(target), _maxLength(maxLength), _editing(false), _cursorPos(0) {
    _temp = new char[maxLength + 1];
    strcpy(_temp, target);
}

void MenuText::onSelect(MenuPage*& currentPage) {
    if (!_editing) {
        // Enter Edit Mode
        _editing = true;
        strcpy(_temp, _target);
        _cursorPos = 0;
    } else {
        // Advance Cursor or Save
        if (_cursorPos < strlen(_temp) && _cursorPos < _maxLength - 1) {
            _cursorPos++;
        } else {
            // Save and Exit
            strcpy(_target, _temp);
            _editing = false;
        }
    }
}

void MenuText::onInput(int delta) {
    if (_editing) {
        char c = _temp[_cursorPos];
        // Cycle: Space -> A-Z -> 0-9
        // ASCII: Space=32, 0-9=48-57, A-Z=65-90
        // Simplified Map: Space, 0-9, A-Z
        
        // Find current index in our charset
        const char* charset = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
        const int len = strlen(charset);
        int idx = 0;
        for (int i=0; i<len; i++) {
            if (charset[i] == c) { idx = i; break; }
        }
        
        idx += delta;
        while (idx < 0) idx += len;
        while (idx >= len) idx -= len;
        
        _temp[_cursorPos] = charset[idx];
        
        // If we just changed the null terminator (end of string), extend string
        if (_cursorPos == strlen(_temp)) {
            _temp[_cursorPos + 1] = 0;
        }
    }
}

void MenuText::getValueString(char* buffer, size_t size) const {
    if (_editing) {
        // Show cursor
        char tempBuf[32];
        strncpy(tempBuf, _temp, sizeof(tempBuf));
        // Mark cursor position (e.g., with brackets or just rely on blinking?)
        // Let's wrap the char in brackets: "A" -> "[A]"
        // This is hard to fit in the buffer.
        // Let's just return the string. The UI renderer should handle cursor indication if possible.
        // But UI renderer is generic.
        // Let's modify the char at cursor to be inverted or something?
        // We can't easily.
        // Let's just return the string for now.
        snprintf(buffer, size, "%s", _temp);
    } else {
        snprintf(buffer, size, "%s", _target);
    }
}
