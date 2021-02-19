#include <avr/io.h>

#include "flux_AS5048B.h"

#define AS5048B_ZERO_HI         (0x16) //bits 0..7
#define AS5048B_ZERO_LO         (0x17) //bits 0..5
#define AS5048B_AGC             (0xfa)
#define AS5048B_DIAG            (0xfb)
#define AS5048B_MAGN_HI         (0xfc) //bits 0..7
#define AS5048B_MAGN_HI_MASK    (0xff)
#define AS5048B_MAGN_LO         (0xfd) //bits 0..5
#define AS5048B_MAGN_LO_MASK    (0x3f)
#define AS5048B_ANGLE_HI        (0xfe) //bits 0..7
#define AS5048B_ANGLE_HI_MASK   (0xff)
#define AS5048B_ANGLE_LO        (0xff) //bits 0..5
#define AS5048B_ANGLE_LO_MASK   (0x3f)

FluxAS5048B::FluxAS5048B(I2CSim* i2c, int id)
{
    i2c->registerDevice(id, this);
    
    position = 0;
    address = 0;
}

FluxAS5048B::~FluxAS5048B()
{
}

void FluxAS5048B::writeI2C(const uint8_t* data, int length)
{
    for(int n=0; n<length; n+=2)
    {
        address = data[n];
        if (n + 1 < length)
        {
            switch(address)
            {
            case AS5048B_ZERO_HI:
                position = (position & AS5048B_ANGLE_LO_MASK) | (data[n + 1] << 6);
                break;
            case AS5048B_ZERO_LO:
                position = (position & (AS5048B_ANGLE_HI_MASK << 6)) | (data[n + 1] & AS5048B_ANGLE_LO_MASK);
                break;
            default:
                printf("FluxAS5048B write[%02x]: %02x\n", address, data[n + 1]);
                break;
            }
        }
    }
}

int FluxAS5048B::readI2C(uint8_t* data)
{
    switch(address)
    {
    case AS5048B_ANGLE_HI:
        data[0] = (position >> 6);
        return 1;
    case AS5048B_ANGLE_LO:
        data[0] = (position & AS5048B_ANGLE_LO_MASK);
        return 1;
    default:
        printf("FluxAS5048B read[%02x]\n", address);
        return 0;
    }
}
