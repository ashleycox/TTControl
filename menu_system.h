/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <Arduino.h>
#include <vector>
#include <functional>
#include "input.h"

// Forward declaration
class MenuPage;

/**
 * @brief Base class for all menu items.
 *
 * Provides a common interface for rendering, selection, and input handling.
 */
class MenuItem {
public:
    typedef std::function<bool()> VisibilityCallback;

    MenuItem(const char* label);
    virtual ~MenuItem();

    virtual const char* getLabel() const { return _label; }
    void setLabel(const char* label);
    void setVisibleWhen(VisibilityCallback callback);
    bool isVisible() const;

    // --- Interaction Methods ---
    virtual void onSelect(MenuPage*& currentPage) {} // Called when Select button pressed
    virtual void onInput(int delta) {} // Called when Encoder rotated (if editing)

    // --- State Methods ---
    virtual bool isDirty() const { return false; } // Returns true if unsaved changes exist
    virtual bool isEditing() const { return false; } // Returns true if currently capturing input
    virtual bool isEditable() const { return false; } // Returns true if item can be edited
    virtual MenuPage* getTargetPage() const { return nullptr; }

    // --- Rendering Helpers ---
    virtual void getValueString(char* buffer, size_t size) const { buffer[0] = 0; }

protected:
    char* _label;
    VisibilityCallback _visibleWhen;
};

/**
 * @brief Container for a list of MenuItems.
 *
 * Manages selection state and scrolling offset.
 */
class MenuPage {
public:
    MenuPage(const char* title);
    ~MenuPage();

    void addItem(MenuItem* item);
    void clear(); // Delete all items and clear list

    const char* getTitle() const { return _title; }
    size_t getItemCount() const { return getVisibleItemCount(); }
    MenuItem* getItem(size_t index) const;

    // --- Navigation ---
    void next(); // Move selection down
    void prev(); // Move selection up
    int getSelection() const { return _selection; }
    int getOffset() const { return _offset; } // Scroll offset

    // --- Interaction ---
    void select(MenuPage*& currentPage); // Activate selected item
    void input(int delta); // Pass input to selected item

private:
    char* _title;
    std::vector<MenuItem*> _items;
    int _selection;
    int _offset;

    size_t getVisibleItemCount() const;
    MenuItem* getVisibleItem(size_t index) const;
    void clampSelection();
};

// --- Derived Item Types ---

/**
 * @brief Menu item that triggers a callback function.
 */
class MenuAction : public MenuItem {
public:
    typedef std::function<void()> ActionCallback;
    MenuAction(const char* label, ActionCallback callback);
    void onSelect(MenuPage*& currentPage) override;
private:
    ActionCallback _callback;
};

/**
 * @brief Read-only informational item.
 */
class MenuInfo : public MenuItem {
public:
    MenuInfo(const char* label) : MenuItem(label) {}
};

/**
 * @brief Dynamic informational item that owns its label string memory.
 * Useful for displaying generated text (e.g., log lines).
 */
class MenuDynamicInfo : public MenuItem {
public:
    MenuDynamicInfo(const String& text) : MenuItem(text.c_str()) {}
};

/**
 * @brief Menu item that navigates to another MenuPage.
 */
class MenuNav : public MenuItem {
public:
    MenuNav(const char* label, MenuPage* target);
    void onSelect(MenuPage*& currentPage) override;
    MenuPage* getTargetPage() const override { return _target; }
private:
    MenuPage* _target;
};

/**
 * @brief Template base for editable settings.
 * Handles edit mode toggling and temporary value storage.
 */
template <typename T>
class MenuSetting : public MenuItem {
public:
    MenuSetting(const char* label, T* target, T min, T max)
        : MenuItem(label), _target(target), _min(min), _max(max), _temp(*target), _editing(false) {}

    void onSelect(MenuPage*& currentPage) override {
        _editing = !_editing;
        if (!_editing) {
            // Save on exit edit
            *_target = _temp;
        } else {
            // Reset temp on enter edit
            _temp = *_target;
        }
    }

    bool isEditable() const override { return true; }
    bool isEditing() const override { return _editing; }
    bool isDirty() const override { return _temp != *_target; }

protected:
    T* _target;
    T _min;
    T _max;
    T _temp;
    bool _editing;
};

// Concrete Setting Types
class MenuInt : public MenuSetting<int> {
public:
    MenuInt(const char* label, int* target, int min, int max);
    void onInput(int delta) override;
    void getValueString(char* buffer, size_t size) const override;
};

class MenuByte : public MenuItem {
public:
    typedef std::function<void(uint8_t)> ChangeCallback;

    MenuByte(const char* label, uint8_t* target, int min, int max);
    MenuByte(const char* label, uint8_t* target, int min, int max,
             const char* const* labels, size_t labelCount);
    MenuByte(const char* label, uint8_t* target, int min, int max,
             const char* const* labels, size_t labelCount, ChangeCallback callback);
    void onSelect(MenuPage*& currentPage) override;
    void onInput(int delta) override;
    void getValueString(char* buffer, size_t size) const override;
    void setChangeCallback(ChangeCallback callback);

    bool isEditable() const override { return true; }
    bool isEditing() const override { return _editing; }
    bool isDirty() const override { return _temp != *_target; }

private:
    uint8_t* _target;
    int _min;
    int _max;
    int _temp;
    bool _editing;
    const char* const* _labels;
    size_t _labelCount;
    ChangeCallback _changeCallback;
};

class MenuFloat : public MenuSetting<float> {
public:
    MenuFloat(const char* label, float* target, float step, float min, float max);
    void onInput(int delta) override;
    void getValueString(char* buffer, size_t size) const override;
private:
    float _step;
};

class MenuBool : public MenuSetting<bool> {
public:
    MenuBool(const char* label, bool* target);
    void onInput(int delta) override;
    void getValueString(char* buffer, size_t size) const override;
};

class MenuText : public MenuItem {
public:
    MenuText(const char* label, char* target, size_t maxLength);
    ~MenuText();
    void onSelect(MenuPage*& currentPage) override;
    void onInput(int delta) override;
    void getValueString(char* buffer, size_t size) const override;

    bool isEditable() const override { return true; }
    bool isEditing() const override { return _editing; }
    bool isDirty() const override;

private:
    char* _target;
    char* _temp;
    size_t _maxLength;
    bool _editing;
    bool _uppercase;
    size_t _cursorPos;

    void loadTargetIntoTemp();
    void saveTempToTarget();
};

#endif // MENU_SYSTEM_H
