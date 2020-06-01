#ifndef __PROCESS_INCLUDED_H
#define __PROCESS_INCLUDED_H

#include <Arduino.h>
#include <thermistor.h>
#include <PID_v1.h>

class Process {
public:
    enum class State {IDLE, RUNNING};

    Process(uint8_t thPin, uint8_t thSensor,
            uint8_t pumpPin, uint8_t heaterPin);

    inline State getState() { return m_state; }
    void setState(State state);

    inline bool isPumpOn() { return m_pumpOn; }
    inline uint8_t getPumpSpeed() { return m_pumpSpeed; }
    inline void setPumpSpeed(uint8_t speed) { m_pumpSpeed = speed ;}

    inline double getActualTemp() { return m_temperature; }
    inline double getTargetTemp() { return m_tempTarget; }
    inline void setTargetTemp(double target) { m_tempTarget = target; }

    void setup();
    void loop();

private:
    uint8_t m_pumpPin;
    uint8_t m_heaterPin;
    State m_state;
    bool m_pumpOn;
    uint8_t m_pumpSpeed;

    double m_temperature;
    double m_tempTarget;
    double m_pidOutput;
    unsigned long m_pidWindowSize;
    unsigned long m_pidWindowStartTime;
    thermistor m_th;
    PID m_pid;
};

#endif