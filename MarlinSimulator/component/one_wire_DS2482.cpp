#include <avr/io.h>

#include "one_wire_DS2482.h"

#define DS2482_RESET            0xF0
#define DS2482_SET_READ_POINTER 0xE1
#define DS2482_WRITE_CONFIG     0xD2
#define DS2482_1WIRE_RESET      0xB4
#define DS2482_1WIRE_SINGLE_BIT 0x87
#define DS2482_1WIRE_WRITE_BYTE 0xA5
#define DS2482_1WIRE_READ_BYTE  0x96
#define DS2482_1WIRE_TRIPLET    0x78

#define DS2482_STATUS_REGISTER  0xF0
#define DS2482_DATA_REGISTER    0xE1
#define DS2482_CONFIG_REGISTER  0xC3

//1 wire busy
#define DS2482_STATUS_1WB       _BV(0)
//Presense-Pulse detect
#define DS2482_STATUS_PPD       _BV(1)
//Short detect
#define DS2482_STATUS_SD        _BV(2)
//Logic level
#define DS2482_STATUS_LL        _BV(3)
//Device reset
#define DS2482_STATUS_RST       _BV(4)
//Single bit result
#define DS2482_STATUS_SBR       _BV(5)
//Triplet second bit
#define DS2482_STATUS_TSB       _BV(6)
//Branch direction taken
#define DS2482_STATUS_DIR       _BV(7)

//Active Pull Up
#define DS2482_CONFIG_APU       _BV(0)
#define DS2482_CONFIG_NO_APU    _BV(4)
//Strong pull up
#define DS2482_CONFIG_SPU       _BV(2)
#define DS2482_CONFIG_NO_SPU    _BV(6)
//Enable 1wire overdrive speed
#define DS2482_CONFIG_1WS       _BV(3)
#define DS2482_CONFIG_NO_1WS    _BV(7)
//This bit should always be set on configuration
#define DS2482_CONFIG_ALWAYS_ON _BV(5)

OneWireDS2482::OneWireDS2482(I2CSim* i2c, int id)
{
    i2c->registerDevice(id, this);
    
    status_data = 0;
    read_data = 0xFF;
    device = nullptr;
}

OneWireDS2482::~OneWireDS2482()
{
}

void OneWireDS2482::setDevice(OneWireDevice* device)
{
    this->device = device;
}

void OneWireDS2482::writeI2C(const uint8_t* data, int length)
{
    for(int n=0; n<length; n++)
    {
        switch(data[n])
        {
        case DS2482_RESET:
            read_pointer = DS2482_STATUS_REGISTER;
            configuration_data = 0;
            status_data = DS2482_STATUS_RST;
            break;
        case DS2482_SET_READ_POINTER:
            n++;
            read_pointer = data[n];
            break;
        case DS2482_WRITE_CONFIG:
            n++;
            if (((data[n] >> 4) ^ 0x0F) == (data[n] & 0x0F))
            {
                configuration_data = data[n] & 0x0F;
                status_data &=~DS2482_STATUS_RST;
            }else{
                printf("OneWireDS2482 config error: %02x\n", data[n]);
            }
            break;
        case DS2482_1WIRE_RESET:
            if (device && device->reset())   //Set PPD if device is present
                status_data |= DS2482_STATUS_PPD;
            else
                status_data &=~DS2482_STATUS_PPD;
            status_data &=~DS2482_STATUS_SD;//We never report a short circuit
            break;
        case DS2482_1WIRE_SINGLE_BIT:
            printf("OneWireDS2482 bit\n");
            break;
        case DS2482_1WIRE_WRITE_BYTE:
            n++;
            if (device)
                device->write(data[n]);
            break;
        case DS2482_1WIRE_READ_BYTE:
            if (device)
                read_data = device->read();
            else
                read_data = 0xFF;
            break;
        case DS2482_1WIRE_TRIPLET:
            printf("OneWireDS2482 triplet\n");
            break;
        default:
            printf("OneWireDS2482 unknown write[%d]: %02x\n", n, data[n]);
        }
    }
}

int OneWireDS2482::readI2C(uint8_t* data)
{
    switch(read_pointer)
    {
    case DS2482_STATUS_REGISTER:
        data[0] = status_data;
        break;
    case DS2482_DATA_REGISTER:
        data[0] = read_data;
        break;
    case DS2482_CONFIG_REGISTER:
        data[0] = configuration_data;
        break;
    default:
        printf("OneWireDS2482 unknown read address: %02x\n", read_pointer);
        data[0] = 0xFF;
        break;
    }
    return 1;
}
