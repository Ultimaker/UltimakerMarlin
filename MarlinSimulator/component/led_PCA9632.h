#ifndef LED_PCA953X_SIM_H
#define LED_PCA953X_SIM_H

#include "i2c.h"
#include "../frontend/frontend.h"

template<int output_count> class LedPCA963XSim : public SimBaseComponent, public I2CDevice
{
public:
    LedPCA963XSim(I2CSim* i2c, int id = 0xC0)
    {
        i2c->registerDevice(id, this);
        for(int n=0; n<output_count; n++)
        {
            pwm[n] = 0;
            ledout[n] = 0;
        }
    }
    virtual ~LedPCA963XSim()
    {
    }
    
    virtual void draw(int x, int y)
    {
        for(int n=0; n<output_count; n++)
            Frontend::instance->drawRect(x + n * 3, y, 3, 3, get(n) << 16 | get(n) << 8 | get(n));
    }
    
    uint8_t get(int index)
    {
        if (index < 0 || index >= output_count)
            return 0;
        if (ledout[index] == 1) return 0xFF;
        if (ledout[index] == 2) return pwm[index];
        return 0;
    }
private:
    void writeI2C(const uint8_t* data, int length)
    {
        int reg_addr = data[0] & 0x7F;
        for(int n=1; n<length; n++)
        {
            if (reg_addr == 0)
                mode0 = data[n];
            else if (reg_addr == 1)
                mode1 = data[n];
            else if (reg_addr - 2 < output_count)
                pwm[reg_addr - 2] = data[n];
            else if (reg_addr == 2 + output_count)
                grppwm = data[n];
            else if (reg_addr == 2 + output_count + 1)
                grpfreq = data[n];
            else if (reg_addr < 2 + output_count + 2 + ((output_count + 3) / 4))
            {
                for(int offset=0; offset<4; offset++)
                {
                    ledout[(reg_addr - (2 + output_count + 2)) * 4 + offset] = (data[n] >> (offset * 2)) & 0x03;
                }
            }

            if (data[0] & 0x80)
                reg_addr++;
        }
    }
    
    int readI2C(uint8_t* data)
    {
        return 0;
    }

    uint8_t mode0;
    uint8_t mode1;
    uint8_t grppwm;
    uint8_t grpfreq;
    uint8_t pwm[output_count];
    uint8_t ledout[output_count];
};

typedef LedPCA963XSim<4> LedPCA9632Sim;
typedef LedPCA963XSim<16> LedPCA9635Sim;

#endif//LED_PCA953X_SIM_H
