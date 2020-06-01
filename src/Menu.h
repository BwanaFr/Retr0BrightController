#ifndef __MENU_INCLUDED_H
#define __MENU_INCLUDED_H
#include <Arduino.h>
#include "Process.h"

class LiquidCrystal_I2C;
class LCDMenu;

class Button {
public:
    enum class State {IDLE, DEBOUNCE, PRESSED_SHORT, SHORT, PRESSED_LONG, LONG};
    Button(uint8_t pin);
    State getState(unsigned long now);
    void setup(uint8_t mode = INPUT_PULLUP);
private:
    uint8_t m_pin;
    unsigned long m_lastPress;
    State m_prevState;
};

class MenuEntry {
public:
    typedef bool (*ActionFunc)(LCDMenu*, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/);
    inline MenuEntry(const char* name, ActionFunc func = nullptr) : 
        m_name(name), m_func(func)
    {
    }
    inline const char* getName() { return m_name; }
    inline ActionFunc getAction() { return m_func; }
private:
    const char* m_name;
    ActionFunc m_func;
};

class LCDMenu {
public:
    LCDMenu(LiquidCrystal_I2C& lcd, Process& process,
            uint8_t upBtn, uint8_t dwBtn, uint8_t selBtn);
    void loop();
    void setup();
    inline Process& getProcess(){ return m_proc; }

    static bool startAction(LCDMenu* p, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/);
    static bool pumpAction(LCDMenu* l, Button::State& upState,
                    Button::State& dwState, Button::State& selState);
private:
    void handleIDLEMenu();
    void printSelectedMenu();

    LiquidCrystal_I2C& m_lcd;
    Process& m_proc;
    Button m_upBtn;
    Button m_dwBtn;
    Button m_selBtn;
    unsigned int m_lastUpdate;
    Process::State m_lastPState;
    int8_t m_selectedIndex;
    MenuEntry::ActionFunc m_action;


};
#endif