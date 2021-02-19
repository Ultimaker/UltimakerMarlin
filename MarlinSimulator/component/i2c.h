#ifndef I2C_SIM_H
#define I2C_SIM_H

#include "base.h"

/* Interface base class for I2C devices */
class I2CDevice
{
public:
    virtual void writeI2C(const uint8_t* data, int size) = 0;
    virtual int readI2C(uint8_t* data) = 0;
};

class I2CSim : public SimBaseComponent
{
public:
    I2CSim();
    virtual ~I2CSim();
    
    void registerDevice(int id, I2CDevice* device);
    
    void I2C_TWCR_callback(uint8_t oldValue, uint8_t& newValue);
private:
    enum EState
    {
        Idle,
        StartInitiated,
        MasterWrite,
        MasterRead
    };
    EState state;
    uint8_t target_address;

    int i2cMessagePos;
    uint8_t i2cMessage[2048];
    I2CDevice* i2cDevice[128];
};

#endif//I2C_SIM_H
