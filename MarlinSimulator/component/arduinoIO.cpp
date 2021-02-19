#include "arduinoIO.h"

// The following include must be after the other includes because it sets a few defines that mask glibC functions such as min() and max()
#include <Arduino.h>

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12

ArduinoIOSim::ArduinoIOSim()
{
    PORTA.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTA_callback));
    PORTB.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTB_callback));
    PORTC.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTC_callback));
    PORTD.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTD_callback));
    PORTE.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTE_callback));
    PORTF.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTF_callback));
    PORTG.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTG_callback));
    PORTH.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTH_callback));
    PORTJ.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTJ_callback));
    PORTK.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTK_callback));
    PORTL.setCallback(DELEGATE(registerDelegate, ArduinoIOSim, *this, IO_PORTL_callback));
    
    for(unsigned int n=0; n<11*8; n++)
        portIdxToPinNr[n] = -1;
    for(unsigned int n=0; n<NUM_DIGITAL_PINS; n++)
    {
        if (digital_pin_to_port_PGM[n] == NOT_A_PORT)
            continue;
        int idx = digital_pin_to_port_PGM[n] * 8;
        int mask = digital_pin_to_bit_mask_PGM[n];
        for(unsigned int i=0;i<8;i++)
            if (mask == _BV(i)) idx += i;
        portIdxToPinNr[idx] = n;
    }
}
ArduinoIOSim::~ArduinoIOSim()
{
}

void ArduinoIOSim::registerPortCallback(int portNr, ioDelegate func)
{
    if (portNr < 0 || portNr >= NUM_DIGITAL_PINS)
        return;
    ioWriteDelegate[portNr] = func;
}

void ArduinoIOSim::checkPinChange(int portID, uint8_t oldValue, uint8_t newValue)
{
    uint8_t change = oldValue ^ newValue;
    if (change)
    {
        for(unsigned int i=0;i<8;i++)
        {
            if (change & _BV(i))
            {
                int pinNr = portIdxToPinNr[portID * 8 + i];
                if (pinNr >= 0)
                    ioWriteDelegate[pinNr](pinNr, newValue & _BV(i));
            }
        }
    }
}

void ArduinoIOSim::IO_PORTA_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PA, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTB_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PB, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTC_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PC, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTD_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PD, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTE_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PE, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTF_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PF, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTG_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PG, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTH_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PH, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTJ_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PJ, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTK_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PK, oldValue, newValue);
}
void ArduinoIOSim::IO_PORTL_callback(uint8_t oldValue, uint8_t& newValue)
{
    checkPinChange(PL, oldValue, newValue);
}

bool readOutput(int arduinoPinNr)
{
	uint8_t bit = digitalPinToBitMask(arduinoPinNr);
    uint8_t port = digitalPinToPort(arduinoPinNr);

	if (port == NOT_A_PORT) return false;

	AVRRegistor* out = portOutputRegister(port);

    return (*out) & bit;
}

uint8_t readAnalogOutput(int arduinoPinNr)
{
    switch(digitalPinToTimer(arduinoPinNr))
    {
    #if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
    case TIMER0A:
        if (TCCR0 & _BV(COM00))
            return OCR0;
        break;
    #endif

    #if defined(TCCR0A) && defined(COM0A1)
    case TIMER0A:
        if (TCCR0A & _BV(COM0A1))
            return OCR0A;
        break;
    #endif

    #if defined(TCCR0A) && defined(COM0B1)
    case TIMER0B:
        if (TCCR0A & _BV(COM0B1))
            return OCR0B;
        break;
    #endif

    #if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:
        if (TCCR1A & _BV(COM1A1))
            return OCR1A;
        break;
    #endif

    #if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:
        if (TCCR1A & _BV(COM1B1))
            return OCR1B;
        break;
    #endif

    #if defined(TCCR2) && defined(COM21)
    case TIMER2:
        if (TCCR2 & _BV(COM21))
            return OCR2;
        break;
    #endif

    #if defined(TCCR2A) && defined(COM2A1)
    case TIMER2A:
        if (TCCR2A & _BV(COM2A1))
            return OCR2A;
        break;
    #endif

    #if defined(TCCR2A) && defined(COM2B1)
    case TIMER2B:
        if (TCCR2A & _BV(COM2B1))
            return OCR2B;
        break;
    #endif

    #if defined(TCCR3A) && defined(COM3A1)
    case TIMER3A:
        if (TCCR3A & _BV(COM3A1))
            return OCR3A;
        break;
    #endif

    #if defined(TCCR3A) && defined(COM3B1)
    case TIMER3B:
        if (TCCR3A & _BV(COM3B1))
            return OCR3B;
        break;
    #endif

    #if defined(TCCR3A) && defined(COM3C1)
    case TIMER3C:
        if (TCCR3A & _BV(COM3C1))
            return OCR3C;
        break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
        if (TCCR4A & _BV(COM4A1))
        {
            #if defined(COM4A0)		// only used on 32U4
            if (!(TCCR4A & _BV(COM4A0)))
            #endif
            return OCR4A;
        }
        break;
    #endif
    
    #if defined(TCCR4A) && defined(COM4B1)
    case TIMER4B:
        if (TCCR4A & _BV(COM4B1))
            return OCR4B;
        break;
    #endif

    #if defined(TCCR4A) && defined(COM4C1)
    case TIMER4C:
        if (TCCR4A & _BV(COM4C1))
            return OCR4C;
        break;
    #endif
        
    #if defined(TCCR4C) && defined(COM4D1)
    case TIMER4D:				
        // connect pwm to pin on timer 4, channel D
        if (TCCR4C & _BV(COM4D1))
        {
            #if defined(COM4D0)		// only used on 32U4
            if (!(TCCR4C & _BV(COM4D0))
            #endif
            return OCR4D;
        }
        break;
    #endif

                    
    #if defined(TCCR5A) && defined(COM5A1)
    case TIMER5A:
        if (TCCR5A & _BV(COM5A1))
            return OCR5A;
        break;
    #endif

    #if defined(TCCR5A) && defined(COM5B1)
    case TIMER5B:
        if (TCCR5A & _BV(COM5B1))
            return OCR5B;
        break;
    #endif

    #if defined(TCCR5A) && defined(COM5C1)
    case TIMER5C:
        if (TCCR5A & _BV(COM5C1))
            return OCR5C;
        break;
    #endif
    }
    return readOutput(arduinoPinNr) ? 255 : 0;
}

void writeInput(int arduinoPinNr, bool value)
{
	uint8_t bit = digitalPinToBitMask(arduinoPinNr);
    uint8_t port = digitalPinToPort(arduinoPinNr);

	if (port == NOT_A_PIN) return;

	AVRRegistor* in = portInputRegister(port);
	if (value)
        (*in) |= bit;
    else
        (*in) &=~bit;
}
