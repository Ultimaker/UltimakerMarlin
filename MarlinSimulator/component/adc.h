#ifndef ADC_SIM_H
#define ADC_SIM_H

#include "base.h"
#include "delegate.h"

/* Callback to get an ADC value. First parameter is the maximum ADC range, 2nd parameter is the return value */
typedef Delegate<int, int&> AdcDelegate;

class AvrAdcSim : public SimBaseComponent
{
public:
    AvrAdcSim();
    virtual ~AvrAdcSim();
    
    void setReadCallback(int index, AdcDelegate function);
private:
    AdcDelegate read_callbacks[16];

    void ADC_ADCSRA_callback(uint8_t oldValue, uint8_t& newValue);
};

#endif//I2C_SIM_H
