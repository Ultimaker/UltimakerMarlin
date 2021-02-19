#ifndef FDC1004_SIM_H
#define FDC1004_SIM_H

#include "i2c.h"

class FDC1004 : public SimBaseComponent, public I2CDevice
{
public:
    FDC1004(I2CSim* i2c, int id = 0x50);
    virtual ~FDC1004();
    
private:
    void writeI2C(const uint8_t* data, int length);
    int readI2C(uint8_t* data);
    
    uint8_t addr;
    
    uint16_t conf_meas[4];
    
    int16_t getSample(int index);
};

#endif//FDC1004_SIM_H
