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
#include "i2c_onewire_ds2482.h"
#include "oneWireDS2431_eeprom.h"

#define READ_ROM_COMMAND            0x33
#define MATCH_ROM_COMMAND           0x55
#define SEARCH_ROM_COMMAND          0xF0
#define SKIP_ROM_COMMAND            0xCC
#define RESUME_COMMAND              0xA5
#define OVERDRIVE_SKIP_ROM_COMMAND  0x3C
#define OVERDRIVE_MATCH_ROM_COMMAND 0x69

//The commands need to follow a ROM command.
#define WRITE_SCRATCHPAD_COMMAND    0x0F
#define READ_SCRATCHPAD_COMMAND     0xAA
#define COPY_SCRATCHPAD_COMMAND     0x55
#define READ_MEMORY_COMMAND         0xF0

bool oneWireDS2431ReadSerial(uint8_t index, uint8_t serial[6])
{
    if (!i2cOneWireReset(index))
        return false;
    i2cOneWireWrite(index, READ_ROM_COMMAND);
    i2cOneWireRead(index);//Family code
    serial[0] = i2cOneWireRead(index);
    serial[1] = i2cOneWireRead(index);
    serial[2] = i2cOneWireRead(index);
    serial[3] = i2cOneWireRead(index);
    serial[4] = i2cOneWireRead(index);
    serial[5] = i2cOneWireRead(index);
    i2cOneWireRead(index);//crc
    return true;
}

bool oneWireDS2431Read(uint8_t index, uint8_t address, uint8_t* data, uint8_t count)
{
    if (!i2cOneWireReset(index))
        return false;
    i2cOneWireWrite(index, SKIP_ROM_COMMAND);
    i2cOneWireWrite(index, READ_MEMORY_COMMAND);
    i2cOneWireWrite(index, address);
    i2cOneWireWrite(index, 0x00);//High byte of address.
    while(count)
    {
        *data = i2cOneWireRead(index);
        data++;
        count--;
    }
    return true;
}

bool oneWireDS2431WritePage(uint8_t index, uint8_t address, const uint8_t data[8])
{
    //TODO: EM-2761 [New] - Check crc?
        
    if (!i2cOneWireReset(index))
        return false;
    i2cOneWireWrite(index, SKIP_ROM_COMMAND);
    i2cOneWireWrite(index, WRITE_SCRATCHPAD_COMMAND);
    i2cOneWireWrite(index, address);
    i2cOneWireWrite(index, 0x00);//High byte of address.
    i2cOneWireWrite(index, data[0]);
    i2cOneWireWrite(index, data[1]);
    i2cOneWireWrite(index, data[2]);
    i2cOneWireWrite(index, data[3]);
    i2cOneWireWrite(index, data[4]);
    i2cOneWireWrite(index, data[5]);
    i2cOneWireWrite(index, data[6]);
    i2cOneWireWrite(index, data[7]);
    //crc16 read
    i2cOneWireRead(index);
    i2cOneWireRead(index);
    
    if (!i2cOneWireReset(index))
        return false;
    i2cOneWireWrite(index, SKIP_ROM_COMMAND);
    i2cOneWireWrite(index, COPY_SCRATCHPAD_COMMAND);
    i2cOneWireWrite(index, address);
    i2cOneWireWrite(index, 0x00);//High byte of address.
    i2cOneWireWrite(index, 0x07);//Internal E/S register
    _delay_ms(10); //Wait 10ms for the EEPROM write to complete, during this time the line needs to be high.
    if (i2cOneWireRead(index) != 0xAA)
        return false;

    return true;
}
