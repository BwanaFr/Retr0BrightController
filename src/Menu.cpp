#include "Menu.h"
#include <LiquidCrystal_I2C.h>
#include "Process.h"

#define DEBOUNCE_TIME 50
#define LONG_PRESS 2000

#define LCD_UPDATE 200

bool LCDMenu::startAction(LCDMenu* p, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/) {
    Serial.println("Starting pump");
    return true;
}

bool LCDMenu::pumpAction(LCDMenu* l, Button::State& upState,
                    Button::State& dwState, Button::State& selState) {
    static uint8_t pump = l->getProcess().getPumpSpeed();
    if(upState == Button::State::SHORT){
        ++pump;
    }else if(upState == Button::State::PRESSED_LONG){
        pump += 5;
    }else if(dwState == Button::State::SHORT){
        --pump;
    }else if(dwState == Button::State::PRESSED_LONG){
        pump -= 5;
    }
    l->m_lcd.setCursor(0,1);
    l->m_lcd.print("Pump : ");
    if(pump < 10){
        l->m_lcd.print("  ");
    }else if(pump<100){
        l->m_lcd.print(" ");
    }
    l->m_lcd.print(pump);
    if(selState == Button::State::LONG){
        l->getProcess().setPumpSpeed(pump);
        return true;
    }else if(selState == Button::State::SHORT){
        return true;
    }
    return false;
}

static MenuEntry idleMenu[] ={
    MenuEntry("Start", &LCDMenu::startAction),
    MenuEntry("Pump", &LCDMenu::pumpAction),
    MenuEntry("Setup"),
};


Button::Button(uint8_t pin) : 
                m_pin(pin), m_lastPress(0)
{
    m_prevState = State::IDLE;
}

void Button::setup(uint8_t mode)
{
    pinMode(m_pin, mode);
}

Button::State Button::getState(unsigned long now)
{
    State ret = State::IDLE;
    if(!digitalRead(m_pin)){
        if(m_prevState == State::IDLE){
            m_lastPress = now;
        }
         unsigned long elapsed = now - m_lastPress;
        if(elapsed < DEBOUNCE_TIME){
            m_prevState = State::DEBOUNCE;
            ret = State::IDLE;
        }else if(elapsed < LONG_PRESS){
            ret = m_prevState = State::PRESSED_SHORT;
        }else{
            ret = m_prevState = State::PRESSED_LONG;
        }
    }else{
        if(m_prevState == State::PRESSED_SHORT){
            ret = State::SHORT;
        }else if(m_prevState == State::PRESSED_LONG){
            ret = State::LONG;
        }
        m_prevState = State::IDLE;
    }
    return ret;
}


LCDMenu::LCDMenu(LiquidCrystal_I2C& lcd,  Process& process,
            uint8_t upBtn, uint8_t dwBtn, uint8_t selBtn) :
    m_lcd(lcd), m_proc(process), 
    m_upBtn(upBtn), m_dwBtn(dwBtn), m_selBtn(selBtn),
    m_lastUpdate(0), m_lastPState(Process::State::IDLE),
    m_selectedIndex(-1), m_action(nullptr)
{
    
}

void LCDMenu::setup(){
    m_upBtn.setup();
    m_dwBtn.setup();
    m_selBtn.setup();
}

void LCDMenu::loop(){
    unsigned int now = millis();
    if(m_proc.getState() == Process::State::IDLE){
        if((m_lastPState != Process::State::IDLE) || 
            (m_lastUpdate == 0)){
            //Back to IDLE state
            m_lcd.clear();
            m_lcd.setCursor(0, 0);  
            m_lcd.print(F("T:"));
            m_lcd.setCursor(5, 0);
            m_lcd.print((char)223);
            m_lcd.print(F("C"));
            m_lcd.setCursor(8, 0);
            m_lcd.print(F("Pump:"));
            m_lastUpdate = 0;
            m_selectedIndex = -1;
        }
        if(now - m_lastUpdate > LCD_UPDATE){
            int temp = m_proc.getActualTemp();
            if(temp>99){
                m_lcd.setCursor(2, 0);
            }else{
                m_lcd.setCursor(3, 0);
            }
            m_lcd.print(temp);
            m_lcd.setCursor(13, 0);
            uint8_t pump = m_proc.getPumpSpeed();
            if(pump == 0){
                m_lcd.print("OFF");
            }else{
                m_lcd.print(pump);
            }
            m_lastUpdate = now;
        }
        handleIDLEMenu();
    }
    m_lastPState  = m_proc.getState();
}

void LCDMenu::handleIDLEMenu()
{
    unsigned long now = millis();
    Button::State upState = m_upBtn.getState(now);
    Button::State dwState = m_dwBtn.getState(now);
    Button::State selState = m_selBtn.getState(now);
    if(m_action != nullptr){
        bool v = m_action(this, upState, dwState, selState);
        if(!v){
            return;
        }else{
            m_action = nullptr;
            printSelectedMenu();
        }
    }
    if(m_selectedIndex == -1){
        m_lcd.setCursor(0, 1);
        m_lcd.print("   Retr0bright ");
    }else{
        if(upState == Button::State::SHORT){
            if(m_selectedIndex == -1){
                m_selectedIndex = 0;
            }else if(m_selectedIndex>0){
                --m_selectedIndex;
            }else{
                m_selectedIndex = (sizeof(idleMenu)/sizeof(MenuEntry)-1);
            }
            printSelectedMenu();
        }
        if(dwState == Button::State::SHORT){
            if(m_selectedIndex<(sizeof(idleMenu)/sizeof(MenuEntry)-1)){
                ++m_selectedIndex;
            }else{
                m_selectedIndex = 0;
            }
            printSelectedMenu();
        }
    }
    if(selState == Button::State::SHORT){
        if(m_selectedIndex != -1){
            MenuEntry::ActionFunc func = idleMenu[m_selectedIndex].getAction();
            m_action = func;
        }else{
            m_selectedIndex = 0;
            printSelectedMenu();
        }
    }
}

void LCDMenu::printSelectedMenu()
{
    m_lcd.setCursor(0, 1);
    m_lcd.print(">");
    m_lcd.print(idleMenu[m_selectedIndex].getName());
    int nameLen = strlen(idleMenu[m_selectedIndex].getName());
    for(int i = nameLen;i<16;++i){
        m_lcd.print(" ");
    }
}
