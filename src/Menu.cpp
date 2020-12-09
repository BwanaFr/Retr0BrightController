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
        MenuEntry("Pump state", previous), pump(false)
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
        if(selState == Button::State::SHORT){
            Process::process.setPumpState(pump);
            return false;
        }else if(selState == Button::State::PRESSED_LONG){
            return false;
        }
        //Still active
        return true;
    };
};

class CfgTemperature : public MenuEntry {
private:
    double m_temperature;
    bool m_tempUpated;
public:
    inline void setTemperature(double temp){m_temperature = temp;}

    CfgTemperature(MenuEntry* previous = nullptr) : 
        MenuEntry("Temperature", previous),
        m_temperature(Process::process.getTargetTemp()),
        m_tempUpated(true)
    {
    }
    
    bool doAction(LiquidCrystal_I2C& l, Button::State& upState,
                    Button::State& dwState, Button::State& selState){
        if(selState == Button::State::PRESSED_LONG){
            m_tempUpated = true;
            return false;
        }else if(selState == Button::State::SHORT){
            Process::process.setTargetTemp(m_temperature);
            m_tempUpated = true;
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

class CfgPump : public MenuEntry {
private:
    int m_pump;
    bool m_pumpUpated;
public:
    inline void setPumpValue(int value){m_pump = value;}

    CfgPump(MenuEntry* previous = nullptr) : 
        MenuEntry("Pump", previous),
        m_pump(Process::process.getPumpTarget()),
        m_pumpUpated(true)
    {
    }
    
    bool doAction(LiquidCrystal_I2C& l, Button::State& upState,
                    Button::State& dwState, Button::State& selState){
        if(selState == Button::State::PRESSED_LONG){
            m_pumpUpated = true;
            return false;
        }else if(selState == Button::State::SHORT){
            Process::process.setPumpTarget(m_pump);
            m_pumpUpated = true;
            return false;
        }else if(upState == Button::State::SHORT ||
            upState == Button::State::PRESSED_LONG){
                m_pump += 1;
                if(m_pump>255){
                    m_pump = 255;
                }
                m_pumpUpated = true;
        }else if(dwState == Button::State::SHORT ||
            dwState == Button::State::PRESSED_LONG){
                m_pump -= 1;
                if(m_pump<0){
                    m_pump = 0;
                }
                m_pumpUpated = true;
        }
        if(m_pumpUpated){
            l.setCursor(0,1);
            l.print(F("Pump : "));
            if(m_pump < 10){
                l.print(F(" "));
            }
            if(m_pump < 100){
                l.print(F(" "));
            }
            l.print(m_pump);
            /*l.print((char)223);
            l.print(F("C  "));*/
        }
        m_pumpUpated = false;
        return true;
    };
}; 

class PIDAutoAction : public MenuEntry {
public:
    PIDAutoAction(MenuEntry* previous = nullptr) : 
        MenuEntry("PID autotune", previous)
    {
    }
    
    bool doAction(LiquidCrystal_I2C&, Button::State& /*upState*/,
                    Button::State& /*dwState*/, Button::State& /*selState*/){
        Process::process.setState(Process::State::PID_AUTOTUNE);
        return false;
    };
};

static CfgTemperature tempCfgMenu(nullptr);
static CfgPump pumpCfgMenu(&tempCfgMenu);
static PIDAutoAction pidAutoCfgMenu(&pumpCfgMenu); 
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
        bool active = cfgMenu.isActive();
        if(!active){
            m_firstCall = false;
        }
        return active;
    }

    void doStartAction(){
        m_firstCall = true;
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
            activateMenu(lcd);
            return;
        }
        m_selectedMenu->startAction();
        return;
    }
}
void MenuEntry::startAction() { 
    m_running = true;
    doStartAction();
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
    //Values are loaded now, so update menu with them
    setTargetPump(Process::process.getPumpTarget());
    setTargetTemperature(Process::process.getTargetTemp());
}

static const char animation[] = "|/-";
#define ANIMATION_LEN 3

void LCDMenu::loop(){
    static bool menuActive = false;
    static uint8_t animationStep = 0;
    static uint8_t runSec = 0;
    static uint8_t runMin = 0;
    static uint8_t runHours = 0;
    static unsigned long lastTimeRead = 0;
    static enum {TARGET, TIME} runDispState = TARGET;
    static uint8_t runDispCount = 0;

    unsigned long now = millis();
    Button::State upState = m_upBtn.getState(now);
    Button::State dwState = m_dwBtn.getState(now);
    Button::State selState = m_selBtn.getState(now);
    Process::State state = Process::process.getState();
    bool stateChanged = false;
    if((m_lastPState != state) || (m_lastUpdate == 0)){
        stateChanged = true;
    }
    //Execute menu if applicable
    if(m_actualMenu != nullptr){
        m_actualMenu->execute(m_lcd, upState, dwState, selState);
        bool active = m_actualMenu->isActive();
        if(!active && menuActive){
            m_lastUpdate = 0;
        }
        menuActive = active;
    }else{
        menuActive = false;
    }
    //Re-render first line and reset menus if state changed
    if(stateChanged){
        m_lcd.clear();
        renderFirstLine(state);
        m_lastUpdate = 0;
        if(state == Process::State::IDLE){
            runSec = 0;
            runMin = 0;
            runHours = 0;
            lastTimeRead = now;
            m_actualMenu = &::idleMenu;
        }else if(state == Process::State::ERROR){
            m_actualMenu = nullptr;
        }else{
            if(m_lastPState != Process::State::RUNNING){
                m_actualMenu = &::runningMenu;
            }
        }
        if(m_actualMenu != nullptr){
            m_actualMenu->resetMenu();
        }
    }

    if(now - m_lastUpdate > LCD_UPDATE){
        printProcessValues(state);
        if(!menuActive){
            bool showAnimation = false;
            if(state == Process::State::IDLE){
                m_lcd.setCursor(0,1);
                m_lcd.print(F("   Retr0bright  "));
            }else if(state == Process::State::RUNNING){
                //Compute time
                if((now - lastTimeRead) > 1000){
                    ++runSec;
                    if(runSec>59){
                        runSec = 0;
                        ++runMin;
                        if(runMin>59){
                            runMin = 0;
                            ++runHours;
                        }
                    }
                    lastTimeRead = now;
                } 
                
                ++runDispCount;
                if(runDispCount>50){
                    runDispState = runDispState == TARGET ? TIME : TARGET;
                    runDispCount = 0;
                }
                m_lcd.setCursor(0,1);
                switch(runDispState){
                    case TARGET:
                    {
                        m_lcd.print(F("Run : "));
                        int targetTemp = static_cast<int>(Process::process.getTargetTemp());
                        if(targetTemp<10){
                            m_lcd.print(" ");
                        }
                        if(targetTemp<100){
                            m_lcd.print(" ");
                        }
                        m_lcd.print(targetTemp);
                        m_lcd.print((char)223);
                        m_lcd.print(F("C    "));
                    }
                    break;
                    case TIME:
                        m_lcd.print(F("Time: "));
                        if(runHours<10)
                            m_lcd.print(F("0"));
                        m_lcd.print(runHours);
                        m_lcd.print(F(":"));
                        if(runMin<10)
                            m_lcd.print(F("0"));
                        m_lcd.print(runMin);
                        m_lcd.print(F(":"));
                        if(runSec<10)
                            m_lcd.print(F("0"));
                        m_lcd.print(runSec);
                        m_lcd.print(F("    "));
                        break;
                }
                
                showAnimation = true;
            }else if(state == Process::State::PID_AUTOTUNE){
                m_lcd.setCursor(0,1);
                m_lcd.print(F("  PID autotune "));
                showAnimation = true;
            }else if(state == Process::State::ERROR){
                m_lcd.setCursor(0,1);
                m_lcd.print(Process::process.getError());
            }
            if(showAnimation){
                m_lcd.setCursor(15,1);
                uint8_t step = animationStep>>2;
                m_lcd.print(animation[step]);
                ++animationStep;
                if((animationStep>>2) >= ANIMATION_LEN){
                    animationStep = 0;
                }
            }
        }
        m_lastUpdate = now;
    }
    m_lastPState  = state;
}

void LCDMenu::renderFirstLine(Process::State state){
    switch (state)
    {
    case Process::State::IDLE:
    case Process::State::RUNNING:
    case Process::State::PID_AUTOTUNE:
        m_lcd.setCursor(0, 0);  
        m_lcd.print(F("T:"));
        m_lcd.setCursor(5, 0);
        m_lcd.print((char)223);
        m_lcd.print(F("C"));
        m_lcd.setCursor(8, 0);
        m_lcd.print(F("Pump:"));
        break;
    case Process::State::ERROR:
        m_lcd.setCursor(0, 0);
        m_lcd.print(F("     ERROR      "));
    default:
        break;
    }
}

void LCDMenu::printProcessValues(Process::State state){
    if(state != Process::State::ERROR){
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
        int pumpValue = Process::process.getPumpValue();
        if(pumpValue != 0){
            if(pumpValue<10){
                m_lcd.print(F(" "));
            }
            if(pumpValue<100){
                m_lcd.print(F(" "));
            }
            m_lcd.print(pumpValue);
        }else{
            m_lcd.print(F("OFF"));
        }
    }
}

void LCDMenu::setTargetTemperature(double temperature)
{
    tempCfgMenu.setTemperature(temperature);
}

void LCDMenu::setTargetPump(int pump)
{
    pumpCfgMenu.setPumpValue(pump);
}