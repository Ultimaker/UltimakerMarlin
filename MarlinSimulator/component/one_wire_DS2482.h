#ifndef ONE_WIRE_DS2482_SIM_H
#define ONE_WIRE_DS2482_SIM_H

#include "i2c.h"

class OneWireDevice
{
public:
    virtual bool reset() = 0;
    virtual void write(uint8_t data) = 0;
    virtual uint8_t read() = 0;
};

class OneWireDS2482 : public SimBaseComponent, public I2CDevice
{
public:
    OneWireDS2482(I2CSim* i2c, int id = 0x18);
    virtual ~OneWireDS2482();

    void setDevice(OneWireDevice* device);
private:
    uint8_t read_pointer;
    uint8_t configuration_data;
    uint8_t status_data;
    uint8_t read_data;
    
    OneWireDevice* device;

    void writeI2C(const uint8_t* data, int length);
    int readI2C(uint8_t* data);
};

#endif//ONE_WIRE_DS2482_SIM_H
