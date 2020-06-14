#include "Process.h"
#include <EEPROM.h>

#define INITIAL_KP 1
#define INITIAL_KI 5
#define INITIAL_KD 1

//Outputs
//Pump is at pin 3
//Heater (SSR) is at pin 7
#define PUMP 3
#define HEATER 7

//Temperature probe
#define TH_PIN 17


// EEPROM addresses for persisted data
const uint8_t eepromKey = 123;
const int keyAddress = 0;
const int SpAddress = 8;
const int KpAddress = 16;
const int KiAddress = 24;
const int KdAddress = 32;

const double aTuneStep = 500;
const double aTuneNoise = 1;
const unsigned int aTuneLookBack = 20;

//Instance (singleton)
Process Process::process(TH_PIN, PUMP, HEATER);

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (unsigned int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}
 
// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (unsigned int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}


Process::Process(uint8_t thPin,
            uint8_t pumpPin, uint8_t heaterPin, int resolution) : 
            m_pumpPin(pumpPin), m_heaterPin(heaterPin),
            m_state(State::IDLE), m_pumpOn(false),
            m_temperature(0), m_tempTarget(50),
            m_pidOutput(0), m_pidWindowSize(1000), m_pidWindowStartTime(0),
            m_oneWire(thPin), m_tempSensor(&m_oneWire),
            m_tempResolution(resolution), m_lastTempRead(0), m_tempReadDelay(0),
            m_pid(&m_temperature, &m_pidOutput, &m_tempTarget, INITIAL_KP, INITIAL_KI, INITIAL_KD, DIRECT),
            m_autoTune(&m_temperature, &m_pidOutput), m_error(nullptr)
{
}

void Process::setState(State state)
{
    if(m_state == State::ERROR){
        return;
    }
    if(state == State::IDLE){
        m_pumpOn = false;
        //Turn heater off
        setHeater(LOW);
        m_pid.SetMode(MANUAL);
    }else if(state == State::RUNNING){
        m_pumpOn = true;
        m_pidWindowStartTime = millis();
        m_pid.SetMode(AUTOMATIC);
    }else if(state == State::PID_AUTOTUNE){
         m_pumpOn = true;
        // set up the auto-tune parameters
        m_autoTune.SetNoiseBand(aTuneNoise);
        m_autoTune.SetOutputStep(aTuneStep);
        m_autoTune.SetLookbackSec((int)aTuneLookBack);
    }
    //Turn pump on
    digitalWrite(m_pumpPin, m_pumpOn);
    m_state = state;
}

void Process::setup()
{
    pinMode(m_pumpPin, OUTPUT);
    pinMode(m_heaterPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    m_tempSensor.begin();
    m_tempSensor.getAddress(m_tempAddress, 0);
    m_tempSensor.setResolution(m_tempAddress, m_tempResolution);
    m_tempSensor.setWaitForConversion(false);
    m_tempSensor.requestTemperatures();
    m_tempReadDelay = 750 / (1 << (12 - m_tempResolution)); 
    m_lastTempRead = millis(); 
    loadParameters();
    m_pid.SetTunings(m_pidKP, m_pidKI, m_pidKD);
    m_pid.SetSampleTime(1000);
    m_pid.SetOutputLimits(0, m_pidWindowSize);
}

void Process::loop()
{
    if(m_state == State::ERROR){
        setHeater(LOW);
        return;
    }
    unsigned long now = millis();
    if (millis() - m_lastTempRead >= m_tempReadDelay)
    {
        float tempC = m_tempSensor.getTempCByIndex(0);
        if(tempC == DEVICE_DISCONNECTED_C){
            m_state = State::ERROR;
            m_error = F("T Read error");
            return;
        }
        m_temperature = tempC;
        if(m_temperature > 100){
            m_state = State::ERROR;
            m_error = F("Too hot");
            return;
        }
        m_tempSensor.requestTemperatures();
        m_lastTempRead = now;
    }
    if(m_state == State::RUNNING){
        m_pid.Compute();
    }else if(m_state == State::IDLE) {
        setHeater(LOW);
    }else if(m_state == State::PID_AUTOTUNE) {
        if(m_autoTune.Runtime()) {
            // Extract the auto-tune calculated parameters
            m_pidKP = m_autoTune.GetKp();
            m_pidKI = m_autoTune.GetKi();
            m_pidKD = m_autoTune.GetKd();

            // Re-tune the PID and revert to normal control mode
            m_pid.SetTunings(m_pidKP, m_pidKI, m_pidKD);
            m_pid.SetMode(MANUAL);

            // Persist any changed parameters to EEPROM
            saveParameters();
            setState(State::IDLE);
        }
    }
    if(m_state != State::IDLE) {
        /************************************************
        * turn the output pin on/off based on pid output
        ************************************************/
        if (millis() - m_pidWindowStartTime > m_pidWindowSize)
        { //time to shift the Relay Window
            m_pidWindowStartTime += m_pidWindowSize;
        }
        if (m_pidOutput < millis() - m_pidWindowStartTime)
            setHeater(LOW);
        else
            setHeater(HIGH);
    }
}

void Process::setPumpState(bool state) {
    m_pumpOn = state;
    digitalWrite(m_pumpPin, state);
}

void Process::loadParameters() {
    uint8_t key = EEPROM.read(keyAddress);
    if(key != eepromKey){
        //EEPROM not initialized
        m_pidKI = INITIAL_KI;
        m_pidKP = INITIAL_KP;
        m_pidKD = INITIAL_KD;
    }else{
        m_tempTarget = EEPROM_readDouble(SpAddress);
        m_pidKI = EEPROM_readDouble(KiAddress);
        m_pidKP = EEPROM_readDouble(KpAddress);
        m_pidKD = EEPROM_readDouble(KdAddress);
    }
}

void Process::saveParameters() {
    EEPROM.write(keyAddress, eepromKey);
    EEPROM_writeDouble(SpAddress, m_tempTarget);
    EEPROM_writeDouble(KiAddress, m_pidKI);
    EEPROM_writeDouble(KpAddress, m_pidKP);
    EEPROM_writeDouble(KdAddress, m_pidKD);
}

void Process::setHeater(uint8_t out) {
    digitalWrite(m_heaterPin, out);
    digitalWrite(LED_BUILTIN, out);
}