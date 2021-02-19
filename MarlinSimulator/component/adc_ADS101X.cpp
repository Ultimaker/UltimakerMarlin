#include <avr/io.h>

#include "adc_ADS101X.h"

#define ADS101X_REGISTER_DATA 0x00
#define ADS101X_REGISTER_CONFIG 0x01

#define ADS101X_CONFIG_MSB_OS _BV(7)
#define ADS101X_CONFIG_MSB_MUX(n) ((n) << 4)
#define ADS101X_CONFIG_MSB_MUX_AIN(n) ADS101X_CONFIG_MSB_MUX(4 | (n))
#define ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN1 ADS101X_CONFIG_MSB_MUX(0)
#define ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(1)
#define ADS101X_CONFIG_MSB_MUX_AIN1_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(2)
#define ADS101X_CONFIG_MSB_MUX_AIN2_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(3)
#define ADS101X_CONFIG_MSB_PGA(n) ((n) << 1)
#define ADS101X_CONFIG_MSB_PGA_6V114 ADS101X_CONFIG_MSB_PGA(0)
#define ADS101X_CONFIG_MSB_PGA_4V096 ADS101X_CONFIG_MSB_PGA(1)
#define ADS101X_CONFIG_MSB_PGA_2V048 ADS101X_CONFIG_MSB_PGA(2)
#define ADS101X_CONFIG_MSB_PGA_1V024 ADS101X_CONFIG_MSB_PGA(3)
#define ADS101X_CONFIG_MSB_PGA_0V512 ADS101X_CONFIG_MSB_PGA(4)
#define ADS101X_CONFIG_MSB_PGA_0V256 ADS101X_CONFIG_MSB_PGA(5)
#define ADS101X_CONFIG_MSB_MODE _BV(0)
#define ADS101X_CONFIG_LSB_DR(n) ((n) << 5)
#define ADS101X_CONFIG_LSB_DR_128SPS ADS101X_CONFIG_LSB_DR(0)
#define ADS101X_CONFIG_LSB_DR_250SPS ADS101X_CONFIG_LSB_DR(1)
#define ADS101X_CONFIG_LSB_DR_490SPS ADS101X_CONFIG_LSB_DR(2)
#define ADS101X_CONFIG_LSB_DR_920SPS ADS101X_CONFIG_LSB_DR(3)
#define ADS101X_CONFIG_LSB_DR_1600SPS ADS101X_CONFIG_LSB_DR(4)
#define ADS101X_CONFIG_LSB_DR_2400SPS ADS101X_CONFIG_LSB_DR(5)
#define ADS101X_CONFIG_LSB_DR_3300SPS ADS101X_CONFIG_LSB_DR(6)
#define ADS101X_CONFIG_LSB_COMP_MODE _BV(4)
#define ADS101X_CONFIG_LSB_COMP_POL _BV(3)
#define ADS101X_CONFIG_LSB_COMP_LAT _BV(2)
#define ADS101X_CONFIG_LSB_COMP_QUE(n) ((n) << 0)
#define ADS101X_CONFIG_LSB_COMP_QUE_ONE ADS101X_CONFIG_LSB_COMP_QUE(0)
#define ADS101X_CONFIG_LSB_COMP_QUE_TWO ADS101X_CONFIG_LSB_COMP_QUE(1)
#define ADS101X_CONFIG_LSB_COMP_QUE_FOUR ADS101X_CONFIG_LSB_COMP_QUE(2)
#define ADS101X_CONFIG_LSB_COMP_QUE_NONE ADS101X_CONFIG_LSB_COMP_QUE(3)

AdcADS101X::AdcADS101X(I2CSim* i2c, int id)
{
    i2c->registerDevice(id, this);
}

AdcADS101X::~AdcADS101X()
{
}

void AdcADS101X::setReadCallback(int index, AdcDelegate function)
{
    if (index >= 0 && index < 4)
        read_callbacks[index] = function;
}

void AdcADS101X::writeI2C(const uint8_t* data, int length)
{
    pointer_register = data[0];
    if (length >= 3)
    {
        switch(pointer_register)
        {
        case ADS101X_REGISTER_CONFIG:
            config_msb = data[1];
            config_lsb = data[2];
            break;
        }
    }
}

int AdcADS101X::readI2C(uint8_t* data)
{
    switch(pointer_register)
    {
    case ADS101X_REGISTER_DATA:
        {
            int mux = config_msb & ADS101X_CONFIG_MSB_MUX(7);
            int raw = 0;
            for(int n=0; n<4; n++)
                if (mux == ADS101X_CONFIG_MSB_MUX_AIN(n))
                    read_callbacks[n](0x7FFF, raw);
            if (mux == ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN1)
                read_callbacks[0](0x7FFF, raw);
            if (mux == ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN3)
                read_callbacks[0](0x7FFF, raw);
            if (mux == ADS101X_CONFIG_MSB_MUX_AIN1_MIN_AIN3)
                read_callbacks[1](0x7FFF, raw);
            if (mux == ADS101X_CONFIG_MSB_MUX_AIN2_MIN_AIN3)
                read_callbacks[2](0x7FFF, raw);
            
            data[0] = raw >> 4;
            data[1] = ((raw << 4) & 0xF0) | 0x0F;
        }
        break;
    case ADS101X_REGISTER_CONFIG:
        data[0] = config_msb;
        data[1] = config_lsb; 
        break;
    default:
        printf("ADS101X: Read of unknown pointer address: %02x\n", pointer_register);
        data[0] = 0xFF;
        data[1] = 0xFF;
        break;
    }
    return 2;
}
