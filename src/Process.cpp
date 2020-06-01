#include "Process.h"

#define INITIAL_KP 2
#define INITIAL_KI 5
#define INITIAL_KD 1

Process::Process(uint8_t thPin, uint8_t thSensor,
            uint8_t pumpPin, uint8_t heaterPin) : 
            m_pumpPin(pumpPin), m_heaterPin(heaterPin),
            m_state(State::IDLE), m_pumpOn(false),
            m_pumpSpeed(0), m_temperature(0), m_tempTarget(50),
            m_pidOutput(0), m_pidWindowSize(1000), m_pidWindowStartTime(0),
            m_th(static_cast<int>(thPin), static_cast<int>(thSensor)),
            m_pid(&m_temperature, &m_pidOutput, &m_tempTarget, INITIAL_KP, INITIAL_KI, INITIAL_KD, DIRECT)
{
    
}

void Process::setState(State state)
{
    if(state == State::IDLE){
        analogWrite(m_pumpPin, 0);
        m_pid.SetMode(MANUAL);
    }else{
        //Turn pump on
        analogWrite(m_pumpPin, m_pumpSpeed);
        m_pidWindowStartTime = millis();
        //tell the PID to range between 0 and the full window size
        m_pid.SetOutputLimits(0, m_pidWindowSize);
        //turn the PID on
        m_pid.SetMode(AUTOMATIC);
    }
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
    unsigned long now = millis();
    if(m_state == State::RUNNING){
        m_pid.Compute();
        /************************************************
         * turn the output pin on/off based on pid output
         ************************************************/
        if (now - m_pidWindowStartTime > m_pidWindowSize)
        { //time to shift the Relay Window
            m_pidWindowStartTime += m_pidWindowSize;
        }
        if (m_pidOutput < now - m_pidWindowStartTime)
             digitalWrite(m_heaterPin, HIGH);
        else
            digitalWrite(m_heaterPin, LOW);
    }
}