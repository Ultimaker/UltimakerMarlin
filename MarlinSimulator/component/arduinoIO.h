#ifndef ARDUINO_IO_SIM_H
#define ARDUINO_IO_SIM_H

#include "base.h"
#include "delegate.h"
#include <Arduino.h>

typedef Delegate<int, bool> ioDelegate;

class ArduinoIOSim : public SimBaseComponent
{
public:
    ArduinoIOSim();
    virtual ~ArduinoIOSim();
    
    void registerPortCallback(int portNr, ioDelegate func);
private:
    int portIdxToPinNr[13*8];
    ioDelegate ioWriteDelegate[NUM_DIGITAL_PINS];

    void IO_PORTA_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTB_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTC_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTD_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTE_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTF_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTG_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTH_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTJ_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTK_callback(uint8_t oldValue, uint8_t& newValue);
    void IO_PORTL_callback(uint8_t oldValue, uint8_t& newValue);
    
    void checkPinChange(int portID, uint8_t oldValue, uint8_t newValue);
};

bool readOutput(int arduinoPinNr);
uint8_t readAnalogOutput(int arduinoPinNr);
void writeInput(int arduinoPinNr, bool value);

#endif//ARDUINO_IO_SIM_H
