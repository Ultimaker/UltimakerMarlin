#include <avr/io.h>

#include "FDC1004.h"

FDC1004::FDC1004(I2CSim* i2c, int id)
{
    i2c->registerDevice(id, this);
    for(int n=0; n<4; n++)
        conf_meas[n] = 0x1C00;
}

FDC1004::~FDC1004()
{
}

void FDC1004::writeI2C(const uint8_t* data, int length)
{
    addr = data[0];
    if (length >= 3)
    {
        switch(addr)
        {
        case 0x08://CONF_MEAS1
        case 0x09://CONF_MEAS2
        case 0x0a://CONF_MEAS3
        case 0x0b://CONF_MEAS4
            conf_meas[addr-8] = (int(data[1]) << 8) | int(data[2]);
            return;
        case 0x0C://FDC_CONF
            return;
        }
        printf("FDC1004 write[%02x]: %02x%02x\n", addr, data[1], data[2]);
    }
}

int FDC1004::readI2C(uint8_t* data)
{
    switch(addr)
    {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
        {
            uint16_t sample = getSample(addr);
            data[0] = sample >> 8;
            data[1] = sample;
        }
        return 2;
    case 0x0C:
        data[0] = 0x00;
        data[1] = 0x08;
        return 2;
    }
    printf("FDC1004 read[%02x]\n", addr);
    data[0] = 0;
    data[1] = 0;
    return 2;
}

int16_t FDC1004::getSample(int index)
{
    float sample = 50000 + rand() % 100; //Sample in the range of 0 to 100000 femto Farat

    float offset = ((conf_meas[index] & 0x03e0) >> 5) * 3125; //3125 femto Farat per CAPDAC
    if ((conf_meas[index] & 0x1C) == 0x1C)
        offset = 0;
    int sample_value = (sample - offset) / 0.5; //0.5 femto Farat

    if (sample_value < -0x8000)
        sample_value = -0x8000;
    if (sample_value > 0x7FFF)
        sample_value = 0x7FFF;
    return sample_value;
}
