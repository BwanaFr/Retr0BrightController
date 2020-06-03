#include "Process.h"

#define INITIAL_KP 1
#define INITIAL_KI 0.8
#define INITIAL_KD 0.5

//Outputs
//Pump is at pin 3
//Heater (SSR) is at pin 7
#define PUMP 3
#define HEATER LED_BUILTIN

//Temperature probe
#define TH_PIN A7
#define TH_TYPE 0

#define RELAY_PERIOD 20

Process Process::process(TH_PIN, TH_TYPE, PUMP, HEATER);

Process::Process(uint8_t thPin, uint8_t thSensor,
            uint8_t pumpPin, uint8_t heaterPin) : 
            m_pumpPin(pumpPin), m_heaterPin(heaterPin),
            m_state(State::IDLE), m_pumpOn(false),
            m_temperature(0), m_tempTarget(50),
            m_pidOutput(0), m_pidWindowSize(1000), m_pidWindowStartTime(0),
            m_th(static_cast<int>(thPin), static_cast<int>(thSensor)),
            m_pid(&m_temperature, &m_tempTarget, &m_relayState, 20, INITIAL_KP, INITIAL_KI, INITIAL_KD)
{
}

void Process::setState(State state)
{
    if(state == State::IDLE){
        m_pumpOn = false;
        //Turn heater off
        digitalWrite(m_heaterPin, false);
    }else{
        m_pumpOn = true;
    }
    //Turn pump on
    digitalWrite(m_pumpPin, m_pumpOn);
    m_state = state;
}

void Process::setup()
{
    pinMode(m_pumpPin, OUTPUT);
    pinMode(m_heaterPin, OUTPUT);
}

void Process::loop()
{
    m_temperature = m_th.analog2temp();
    if(m_state == State::RUNNING){
        m_pid.run();
        digitalWrite(m_heaterPin, m_relayState);
    }else{
        digitalWrite(m_heaterPin, LOW);
    }
}

void Process::setPumpState(bool state) {
    m_pumpOn = state;
    digitalWrite(m_pumpPin, state);
}