#ifndef __PROCESS_INCLUDED_H
#define __PROCESS_INCLUDED_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

class Process {
public:
    enum class State {IDLE, RUNNING, PID_AUTOTUNE, ERROR};

    inline State getState() { return m_state; }
    void setState(State state);

    inline int getPumpValue() { return m_pumpValue; }
    void setPumpState(bool state);
    void setPumpTarget(int target);
    inline int getPumpTarget() { return m_pumpTarget; }
    inline double getActualTemp() { return m_temperature; }
    inline double getTargetTemp() { return m_tempTarget; }
    void setTargetTemp(double target);

    void setup();
    void loop();
    inline const __FlashStringHelper * getError() { return m_error; }

    static Process process;
private:
    void loadParameters();
    void saveParameters();
    void setHeater(uint8_t out);
    Process(uint8_t thPin, uint8_t pumpPin, uint8_t heaterPin, int resolution=10);
    uint8_t m_pumpPin;                  //Pin used for pump
    uint8_t m_heaterPin;                //Pin used for heater
    State m_state;                      //Actual state
    int m_pumpValue;                    //Actual pump value
    int m_pumpTarget;                   //Pump set point
    double m_temperature;               //Actual temperature
    double m_tempTarget;                //Temperature setpoint
    double m_pidOutput;                 //PID output
    unsigned long m_pidWindowSize;      //PID window size
    unsigned long m_pidWindowStartTime; //PID window start time
    double m_pidKI;                     //PID KI parameter
    double m_pidKP;                     //PID KP parameter
    double m_pidKD;                     //PID KD parameter
    OneWire m_oneWire;                  //One wire bus
    DallasTemperature m_tempSensor;     //DS18B20 sensor
    DeviceAddress m_tempAddress;        //DS18B20 address
    int m_tempResolution;               //DS18B20 resolution
    unsigned long m_lastTempRead;       //Last temperature readout
    unsigned long m_tempReadDelay;      //Delay before each temperature read
    unsigned long m_lastTempReached;    //Last temperature reached
    PID m_pid;                          //PID controller
    PID_ATune m_autoTune;               //PID autotuner
    const __FlashStringHelper *m_error; //Error msg
};

#endif