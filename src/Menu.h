#ifndef __MENU_INCLUDED_H
#define __MENU_INCLUDED_H
#include <Arduino.h>
#include "Process.h"
#include <LiquidCrystal_I2C.h>

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
    inline MenuEntry(const char* name, MenuEntry* previous = nullptr) : 
        m_name(name), m_previous(previous), m_next(nullptr),
        m_running(false)
    {
        if(previous != nullptr){
            previous->m_next = this;
        }
    }
    inline virtual ~MenuEntry(){}
    inline const char* getName() const { return m_name; };
    inline MenuEntry* getNext() const { return m_next; }
    inline MenuEntry* getPrevious() const { return m_previous; }
    inline void setNext(MenuEntry* next) { m_next = next; }
    inline void setPrevious(MenuEntry* previous) { m_previous = previous; }
    inline bool handleAction(LiquidCrystal_I2C& lcd, Button::State& upState,
                        Button::State& dwState, Button::State& selState) {
        if(m_running){
            m_running = doAction(lcd, upState, dwState, selState);
        }
        return m_running;
    }
    void startAction();
    inline bool isRunning() const { return m_running; }
private:
    virtual bool doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                        Button::State& /*dwState*/, Button::State& /*selState*/) = 0;
    const char* m_name;
    MenuEntry* m_previous;
    MenuEntry* m_next;
    bool m_running;
};

class Menu : public MenuEntry{
public:
    Menu(MenuEntry* firstEntry);
    inline virtual ~Menu(){}
    bool isActive();
    void execute(LiquidCrystal_I2C& lcd, Button::State& upState,
                Button::State& dwState, Button::State& selState);
    void activateMenu(LiquidCrystal_I2C& lcd);
    void resetMenu();
private:
    void printSelectedMenu(LiquidCrystal_I2C& lcd);
    bool doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                        Button::State& /*dwState*/, Button::State& /*selState*/);
    MenuEntry* m_firstMenu;
    MenuEntry* m_selectedMenu;
    bool m_active;
    
};

class LCDMenu {
public:
    LCDMenu(PCF8574_address lcdAddr,
            uint8_t upBtn, uint8_t dwBtn, uint8_t selBtn);
    void loop();
    void setup();

    static bool startAction(LCDMenu* p, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/);
    static bool pumpAction(LCDMenu* l, Button::State& upState,
                    Button::State& dwState, Button::State& selState);
    
private:
    LiquidCrystal_I2C m_lcd;
    Button m_upBtn;
    Button m_dwBtn;
    Button m_selBtn;
    Menu* m_actualMenu;
    unsigned int m_lastUpdate;
    Process::State m_lastPState;
    
};
#endif