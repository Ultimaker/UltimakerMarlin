/*
    Copyright (c) 2015-2021 Ultimaker B.V. All rights reserved.

    Marlin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Marlin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Configuration.h"
#include "i2c_driver.h"
#include "i2c_onewire_ds2482.h"

static const uint8_t i2cOneWireAddress1 = 0b0011000;
static const uint8_t i2cOneWireAddress2 = 0b0011001;

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

i2cCommand i2cOneWireCommandNoParam[EXTRUDERS];
i2cCommand i2cOneWireCommandWithParam[EXTRUDERS];
i2cCommand i2cOneWireReadCommand[EXTRUDERS];
uint8_t i2cOneWireBuffer[2];

void i2cOneWireInit()
{
    i2cDriverCommandSetup(i2cOneWireCommandNoParam[0],   i2cOneWireAddress1 << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 1);
    i2cDriverCommandSetup(i2cOneWireCommandWithParam[0], i2cOneWireAddress1 << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 2);
    i2cDriverCommandSetup(i2cOneWireReadCommand[0],      i2cOneWireAddress1 << 1 | I2C_READ_BIT,  I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 1);
    
    i2cDriverCommandSetup(i2cOneWireCommandNoParam[1],   i2cOneWireAddress2 << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 1);
    i2cDriverCommandSetup(i2cOneWireCommandWithParam[1], i2cOneWireAddress2 << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 2);
    i2cDriverCommandSetup(i2cOneWireReadCommand[1],      i2cOneWireAddress2 << 1 | I2C_READ_BIT,  I2C_QUEUE_PRIO_MEDIUM, i2cOneWireBuffer, 1);
    
    i2cOneWireBuffer[0] = DS2482_RESET;
    for(uint8_t index = 0; index < EXTRUDERS; index++)
        i2cDriverExecuteAndWait(&i2cOneWireCommandNoParam[index]);

    i2cOneWireBuffer[0] = DS2482_WRITE_CONFIG;
    i2cOneWireBuffer[1] = DS2482_CONFIG_APU | DS2482_CONFIG_NO_SPU | DS2482_CONFIG_NO_1WS | DS2482_CONFIG_ALWAYS_ON;
    for(uint8_t index = 0; index < EXTRUDERS; index++)
        i2cDriverExecuteAndWait(&i2cOneWireCommandWithParam[index]);
}

static uint8_t readStatus(uint8_t index)
{
    i2cOneWireBuffer[0] = DS2482_SET_READ_POINTER;
    i2cOneWireBuffer[1] = DS2482_STATUS_REGISTER;
    i2cDriverExecuteAndWait(&i2cOneWireCommandWithParam[index]);
    i2cDriverExecuteAndWait(&i2cOneWireReadCommand[index]);
    return i2cOneWireBuffer[0];
}

bool i2cOneWireReset(uint8_t index)
{
    i2cOneWireBuffer[0] = DS2482_1WIRE_RESET;
    i2cDriverExecuteAndWait(&i2cOneWireCommandNoParam[index]);
    
    uint8_t status;
    while((status = readStatus(index)) & DS2482_STATUS_1WB)
    {
        //Wait till busy signal is done. (about 1.2ms for reset)
    }
    if (status & DS2482_STATUS_PPD)
        return true;
    return false;
}

void i2cOneWireWrite(uint8_t index, uint8_t data)
{
    i2cOneWireBuffer[0] = DS2482_1WIRE_WRITE_BYTE;
    i2cOneWireBuffer[1] = data;
    i2cDriverExecuteAndWait(&i2cOneWireCommandWithParam[index]);
    
    while(readStatus(index) & DS2482_STATUS_1WB)
    {
        //Wait till the write is done
    }
}

uint8_t i2cOneWireRead(uint8_t index)
{
    i2cOneWireBuffer[0] = DS2482_1WIRE_READ_BYTE;
    i2cDriverExecuteAndWait(&i2cOneWireCommandNoParam[index]);
    
    while(readStatus(index) & DS2482_STATUS_1WB)
    {
        //Wait till the read is done
    }
    
    i2cOneWireBuffer[0] = DS2482_SET_READ_POINTER;
    i2cOneWireBuffer[1] = DS2482_DATA_REGISTER;
    i2cDriverExecuteAndWait(&i2cOneWireCommandWithParam[index]);
    i2cDriverExecuteAndWait(&i2cOneWireReadCommand[index]);
    return i2cOneWireBuffer[0];
}
