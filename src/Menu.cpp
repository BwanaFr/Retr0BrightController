#include "Menu.h"
#include "Process.h"

#define DEBOUNCE_TIME 50
#define LONG_PRESS 1000

#define LCD_UPDATE 200
#define LCD_COLUMNS  16
#define LCD_ROWS      2

class StartAction : public MenuEntry {
public:
    StartAction(MenuEntry* previous = nullptr) : 
        MenuEntry("Start", previous)
    {
    }
    
    bool doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/){
        Serial.println("Starting action...");
        Process::process.setState(Process::State::RUNNING);
        return false;
    };
};

class StopAction : public MenuEntry {
public:
    StopAction(MenuEntry* previous = nullptr) : 
        MenuEntry("Stop", previous)
    {
    }
    
    bool doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/){
        Process::process.setState(Process::State::IDLE);
        return false;
    };
};

class PumpAction : public MenuEntry {
private:
    bool pump;
public:
    PumpAction(MenuEntry* previous) : 
        MenuEntry("Pump", previous), pump(false)
    {
    }
    
    bool doAction(LiquidCrystal_I2C& l, Button::State& upState,
                    Button::State& dwState, Button::State& selState){

        if(upState == Button::State::SHORT || 
            dwState == Button::State::SHORT){
            pump = !pump;
        }
        l.setCursor(0,1);
        l.print(F("Pump : "));
        if(pump){
            l.print(F("ON  "));
        }else{
            l.print(F("OFF "));
        }
        if(selState == Button::State::PRESSED_LONG){
            Serial.print("Setting pump : ");
            Serial.println(pump);
            Process::process.setPumpState(pump);
            return false;
        }else if(selState == Button::State::SHORT){
            Serial.println("Cancelled");
            return false;
        }
        //Still active
        return true;
    };
};

class CfgTemperature : public MenuEntry {
private:
    double m_temperature;
    bool m_tempUpated = true;
public:
    CfgTemperature(MenuEntry* previous = nullptr) : 
        MenuEntry("Temperature", previous),
        m_temperature(Process::process.getTargetTemp())
    {
    }
    
    bool doAction(LiquidCrystal_I2C& l, Button::State& upState,
                    Button::State& dwState, Button::State& selState){
        if(selState == Button::State::SHORT){
            Serial.println("Cancelled");
            return false;
        }else if(selState == Button::State::PRESSED_LONG){
            Process::process.setTargetTemp(m_temperature);
            return false;
        }else if(upState == Button::State::SHORT ||
            upState == Button::State::PRESSED_LONG){
                m_temperature += 1.0;
                if(m_temperature>70.0){
                    m_temperature = 70.0;
                }
                m_tempUpated = true;
        }else if(dwState == Button::State::SHORT ||
            dwState == Button::State::PRESSED_LONG){
                m_temperature -= 1.0;
                if(m_temperature<0.0){
                    m_temperature = 0.0;
                }
                m_tempUpated = true;
        }
        if(m_tempUpated){
            l.setCursor(0,1);
            l.print(F("T. Temp : "));
            if(m_temperature < 10){
                l.print(F(" "));
            }
            l.print(static_cast<int>(m_temperature));
            l.print((char)223);
            l.print(F("C  "));
        }
        m_tempUpated = false;
        return true;
    };
}; 
static CfgTemperature tempCfgMenu(nullptr); 
static Menu cfgMenu(&tempCfgMenu);

class SetupAction : public MenuEntry {
private:
    bool m_firstCall;
public:
    SetupAction(MenuEntry * previous) : 
    MenuEntry("Setup", previous),
    m_firstCall(true)
    {
    }
    
    bool doAction(LiquidCrystal_I2C& l, Button::State& upState,
                    Button::State& dwState, Button::State& selState){
        if(m_firstCall){
            cfgMenu.activateMenu(l);
            m_firstCall = false;
        }
        cfgMenu.execute(l, upState, dwState, selState);
        return cfgMenu.isActive();
    }
};

static StartAction startAction(nullptr);
static PumpAction pumpAction(&startAction);
static SetupAction setupAction(&pumpAction);
static Menu idleMenu(&startAction);

static StopAction stopAction(nullptr);
static Menu runningMenu(&stopAction);

Button::Button(uint8_t pin) : 
                m_pin(pin), m_lastPress(0),
                m_prevState(State::IDLE)
{
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

Menu::Menu(MenuEntry* firstEntry) :
    MenuEntry("<< Back", nullptr),
    m_firstMenu(firstEntry), m_selectedMenu(nullptr),
    m_active(false)
{
    //Find the last menu
    MenuEntry* lastMenu = m_firstMenu;
    while(true){
        if(lastMenu->getNext() == nullptr){
            break;
        }
        lastMenu = lastMenu->getNext();
    }
    //Insert back menu
    m_firstMenu->setPrevious(this);
    lastMenu->setNext(this);
    this->setPrevious(lastMenu);
    this->setNext(m_firstMenu);
}

bool Menu::isActive()
{
    return m_active;
}

bool Menu::doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                        Button::State& /*dwState*/, Button::State& selState)
{
    //Reset to initial condition
    m_active = false;
    m_selectedMenu = nullptr;
    return false;
}

void Menu::execute(LiquidCrystal_I2C& lcd, Button::State& upState,
            Button::State& dwState, Button::State& selState)
{
    if(m_selectedMenu != nullptr){
        if(m_selectedMenu->isRunning()){
            bool v = m_selectedMenu->handleAction(lcd, upState, dwState, selState);
            if(!v){
                printSelectedMenu(lcd);
            }
            return;
        }
        if(upState == Button::State::SHORT){
            if(m_selectedMenu){
                m_selectedMenu = m_selectedMenu->getNext();
                printSelectedMenu(lcd);
            }
        }
        if(dwState == Button::State::SHORT){
            if(m_selectedMenu){
                m_selectedMenu = m_selectedMenu->getPrevious();
                printSelectedMenu(lcd);
            }
        }
    }
    if(selState == Button::State::SHORT){
        m_active = true;
        if(m_selectedMenu == nullptr){
            Serial.println("Menu woke up by key-press");
            activateMenu(lcd);
            return;
        }
        m_selectedMenu->startAction();
        return;
    }
}
void MenuEntry::startAction() { 
    m_running = true;
    Serial.println("Starting action");
}

void Menu::activateMenu(LiquidCrystal_I2C& lcd)
{
    m_selectedMenu = m_firstMenu;
    printSelectedMenu(lcd);
    m_active = true;
}

void Menu::resetMenu()
{
    m_active = false;
    m_selectedMenu = nullptr;
}

void Menu::printSelectedMenu(LiquidCrystal_I2C& lcd)
{
    if(m_selectedMenu != nullptr){
        lcd.setCursor(0, 1);
        if(m_selectedMenu != this)
            lcd.print(F(">"));
        else
            lcd.print(F("<"));
        lcd.print(m_selectedMenu->getName());
        int nameLen = strlen(m_selectedMenu->getName());
        for(int i = nameLen;i<LCD_COLUMNS;++i){
            lcd.print(F(" "));
        }
    }
}

LCDMenu::LCDMenu(PCF8574_address lcdAddr,
            uint8_t upBtn, uint8_t dwBtn, uint8_t selBtn) :
    m_lcd(lcdAddr),
    m_upBtn(upBtn), m_dwBtn(dwBtn), m_selBtn(selBtn),
    m_actualMenu(nullptr),
    m_lastUpdate(0), m_lastPState(Process::State::IDLE)
{
}

void LCDMenu::setup(){
    //LCD configuration
    if(m_lcd.begin(LCD_COLUMNS, LCD_ROWS) != 1)
    {
        Serial.println(F("LCD not found"));
    }
    m_lcd.setCursor(0, 0);
    m_lcd.print(F("Retr0bright!"));
    m_upBtn.setup();
    m_dwBtn.setup();
    m_selBtn.setup();
}

Button::State prevState = Button::State::IDLE;

void LCDMenu::loop(){
    unsigned int now = millis();
    Button::State upState = m_upBtn.getState(now);
    Button::State dwState = m_dwBtn.getState(now);
    Button::State selState = m_selBtn.getState(now);
    Process::State state = Process::process.getState();
    if(selState != prevState){
        Serial.print("New state : ");
        Serial.println(((int)selState));
        prevState = selState;
    }
    if(m_actualMenu != nullptr){
        m_actualMenu->execute(m_lcd, upState, dwState, selState);
    
        if(!m_actualMenu->isActive()){
            if(state == Process::State::IDLE){
                m_lcd.setCursor(0, 1);
                m_lcd.print(F("   Retr0bright  "));
            }else if(state == Process::State::RUNNING){
                m_lcd.setCursor(0,1);
                m_lcd.print(F("Run : "));
                m_lcd.print(Process::process.getTargetTemp());
                m_lcd.print((char)223);
                m_lcd.print(F("C"));
            }
        }
    }
    if(state == Process::State::IDLE){
        if((m_lastUpdate == 0)  || 
            (m_lastPState != Process::State::IDLE)){
            idleMenu.resetMenu();
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
            m_actualMenu = &::idleMenu;
        }
    }else{
        if(m_lastPState != Process::State::RUNNING){
            runningMenu.resetMenu();
            m_actualMenu = &::runningMenu;
        }
    }
    
    if(now - m_lastUpdate > LCD_UPDATE){
        int temp = Process::process.getActualTemp();
        m_lcd.setCursor(2,0);
        if(temp<10){
            m_lcd.print(F(" "));
        }
        if(temp<100){
            m_lcd.print(F(" "));
        }
        m_lcd.print(temp);
        m_lcd.setCursor(13, 0);
        if(Process::process.isPumpOn()){
            m_lcd.print("ON ");
        }else{
            m_lcd.print("OFF");
        }
        m_lastUpdate = now;
    }
    m_lastPState  = state;
}
