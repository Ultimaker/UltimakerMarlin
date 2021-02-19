#include <avr/io.h>

#include "adc.h"

AvrAdcSim::AvrAdcSim()
{
    ADCSRA.setCallback(DELEGATE(registerDelegate, AvrAdcSim, *this, ADC_ADCSRA_callback));
}

AvrAdcSim::~AvrAdcSim()
{
}

void AvrAdcSim::setReadCallback(int index, AdcDelegate function)
{
    if (index >= 0 && index < 16)
        read_callbacks[index] = function;
}

void AvrAdcSim::ADC_ADCSRA_callback(uint8_t oldValue, uint8_t& newValue)
{
    if ((newValue & _BV(ADEN)) && (newValue & _BV(ADSC)))
    {   //Start ADC conversion
        int idx = ADMUX & (_BV(MUX4)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0));
        if (ADCSRB & _BV(MUX5))
            idx += 8;
        
        int output = 0;
        read_callbacks[idx](255, output);
        ADC = output;
        newValue &=~_BV(ADSC);
    }
}
