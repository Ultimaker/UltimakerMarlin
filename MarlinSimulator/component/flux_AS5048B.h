#ifndef FLUX_AS5048B_SIM_H
#define FLUX_AS5048B_SIM_H

#include "i2c.h"

class FluxAS5048B : public SimBaseComponent, public I2CDevice
{
public:
    FluxAS5048B(I2CSim* i2c, int id = 0x40);
    virtual ~FluxAS5048B();
    
    uint16_t position;
private:
    void writeI2C(const uint8_t* data, int length);
    int readI2C(uint8_t* data);
    
    uint8_t address;
};

#endif//FLUX_AS5048B_SIM_H
