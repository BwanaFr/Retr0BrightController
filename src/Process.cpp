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
const int pumpAddress = 40;

const double aTuneStep = 500;
const double aTuneNoise = 1;
const unsigned int aTuneLookBack = 20;

const double tempMargin = 5;
//Maximum time allowed to reach temperature (5 minutes)
const unsigned long tempHeatMaxTime = 300000;

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

// ************************************************
// Write int values to EEPROM
// ************************************************
void EEPROM_writeInt(int address, int value)
{
   byte* p = (byte*)(void*)&value;
   for (unsigned int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}
 
// ************************************************
// Read int values from EEPROM
// ************************************************
int EEPROM_readInt(int address)
{
   int value = 0;
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
            m_state(State::IDLE), m_pumpTarget(255),
            m_temperature(0), m_tempTarget(50),
            m_pidOutput(0), m_pidWindowSize(1000), m_pidWindowStartTime(0),
            m_oneWire(thPin), m_tempSensor(&m_oneWire),
            m_tempResolution(resolution), m_lastTempRead(0),
            m_tempReadDelay(0), m_lastTempReached(0),
            m_pid(&m_temperature, &m_pidOutput, &m_tempTarget, INITIAL_KP, INITIAL_KI, INITIAL_KD, DIRECT),
            m_autoTune(&m_temperature, &m_pidOutput), m_error(nullptr)
{
}

void Process::setState(State state)
{
    if(m_state == State::ERROR){
        return;
    }
    bool pumpOn = false;
    if(state == State::IDLE){
        //Turn heater off
        setHeater(LOW);
        m_pid.SetMode(MANUAL);
    }else if(state == State::RUNNING){
        pumpOn = true;
        unsigned long now = millis();
        m_pidWindowStartTime = now;
        m_lastTempReached = now;
        m_pid.SetMode(AUTOMATIC);
    }else if(state == State::PID_AUTOTUNE){
        //Activate the pump during PID auto-tune
        pumpOn = true;
        // set up the auto-tune parameters
        m_autoTune.SetNoiseBand(aTuneNoise);
        m_autoTune.SetOutputStep(aTuneStep);
        m_autoTune.SetLookbackSec((int)aTuneLookBack);
    }
    setPumpState(pumpOn);
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
    if (now - m_lastTempRead >= m_tempReadDelay)
    {
        float tempC = m_tempSensor.getTempCByIndex(0);
        if(tempC == DEVICE_DISCONNECTED_C){
            m_state = State::ERROR;
            m_error = F("T read error");
            return;
        }
        m_temperature = tempC;
        if(m_state != State::IDLE){
            //Check temperature
            if(m_temperature > 100){
                m_state = State::ERROR;
                m_error = F("Too hot");
                return;
            }
            if(abs(m_temperature-m_tempTarget)<tempMargin){
                //Temperature in range
                m_lastTempReached = now;
            }else{
                //Temperature not in range
                if(now - m_lastTempReached>tempHeatMaxTime){
                    m_state = State::ERROR;
                    m_error = F("T not reached");
                    return;
                }
            }
        }
        //Plan next temperature readout
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
    if(state){
        m_pumpValue = m_pumpTarget;
    }else{
        m_pumpValue = 0;
    }
    if(m_pumpValue<0){
        m_pumpValue = 0;
    }else if(m_pumpValue>255){
        m_pumpValue = 255;
    }
    analogWrite(m_pumpPin, m_pumpValue);
}

void Process::setPumpTarget(int target) {
    m_pumpTarget = target;
    if(m_pumpTarget<0){
        m_pumpTarget = 0;
    }else if(m_pumpTarget>255){
        m_pumpTarget = 255;
    }
    if(m_pumpValue != 0){
        setPumpState(true);
    }
    saveParameters();
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
        m_pumpTarget = EEPROM_readInt(pumpAddress);
    }
}

void Process::saveParameters() {
    EEPROM.write(keyAddress, eepromKey);
    EEPROM_writeDouble(SpAddress, m_tempTarget);
    EEPROM_writeDouble(KiAddress, m_pidKI);
    EEPROM_writeDouble(KpAddress, m_pidKP);
    EEPROM_writeDouble(KdAddress, m_pidKD);
    EEPROM_writeInt(pumpAddress, m_pumpTarget);
}

void Process::setHeater(uint8_t out) {
    digitalWrite(m_heaterPin, out);
    digitalWrite(LED_BUILTIN, out);
}

void Process::setTargetTemp(double target) { 
    m_tempTarget = target;
    saveParameters();
}