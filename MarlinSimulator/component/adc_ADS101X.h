#ifndef ADC_ADS101X_SIM_H
#define ADC_ADS101X_SIM_H

#include "i2c.h"
#include "adc.h"

class AdcADS101X : public SimBaseComponent, public I2CDevice
{
public:
    AdcADS101X(I2CSim* i2c, int id = 0x48);
    virtual ~AdcADS101X();
    
    void setReadCallback(int index, AdcDelegate function);
private:
    uint8_t pointer_register;
    uint8_t config_msb;
    uint8_t config_lsb;
    
    AdcDelegate read_callbacks[4];

    void writeI2C(const uint8_t* data, int length);
    int readI2C(uint8_t* data);
    
    int16_t temperatureToRaw(short temperature);
};

#endif//ADC_ADS101X_SIM_H
