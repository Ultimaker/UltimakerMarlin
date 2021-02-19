#include "one_wire_DS2431_EEPROM.h"
#include <stdio.h>

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

constexpr uint8_t OneWireDS2431EEPROM::default_data[];

OneWireDS2431EEPROM::OneWireDS2431EEPROM(uint64_t serial_number)
{
    this->serial_number = serial_number;
    
    for(unsigned int n=0; n<sizeof(data); n++)
        data[n] = this->default_data[n];
    present = true;
}

bool OneWireDS2431EEPROM::reset()
{
    state = 0;
    read_index = -1;
    write_index = 0;
    for(unsigned int n=0; n<sizeof(command); n++)
        command[n] = 0xFF;
    return present;
}

void OneWireDS2431EEPROM::write(uint8_t data)
{
    command[write_index] = data;
    if (write_index < 7) write_index++;
}

uint8_t OneWireDS2431EEPROM::read()
{
    read_index++;
    switch(command[0])
    {
    case READ_ROM_COMMAND:
        if (read_index == 0) return 0xEE;//Family code
        if (read_index >= 1 && read_index < 7) return serial_number >> (8 * (read_index - 1)); //Serial number
        return 0xFF;
    case SKIP_ROM_COMMAND:
        switch(command[1])
        {
        case READ_MEMORY_COMMAND:
            {
                unsigned int address = (command[2] | (command[3] << 8)) + read_index;
                if (address < sizeof(data))
                    return data[address];
                return 0xFF;
            }
            break;
        }
        printf("OneWireDS2431EEPROM::read() with ROM command: %02x\n", command[1]);
        return 0xFF;
    }
    printf("OneWireDS2431EEPROM::read() with command: %02x\n", command[0]);
    return 0xFF;
}
