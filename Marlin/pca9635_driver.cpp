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
#include "pca9635_driver.h"
#include "i2c_driver.h"
#include "Marlin.h"

#define PCA9635_ADDRESS 0b1000100
#define PCA9635_OUTPUT_COUNT 16

static i2cCommand update_command;
static uint8_t output_values[PCA9635_OUTPUT_COUNT];
static uint8_t update_buffer[PCA9635_OUTPUT_COUNT+1];

void initPCA9635()
{
    i2cCommand init_command;
    uint8_t init_buffer[25];

    i2cDriverCommandSetup(update_command, PCA9635_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, update_buffer, sizeof(update_buffer));
    i2cDriverCommandSetup(init_command, PCA9635_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, init_buffer, sizeof(init_buffer));

    init_buffer[0] = 0x80; //Start writing from address 0, with auto increase.
    init_buffer[1] = 0x00; //MODE1, disable sleep, disable all other addresses.
    init_buffer[2] = _BV(2); //MODE2, OUTDRV enabled
    //All PWM outputs to 0
    for(uint8_t n=3; n<19; n++)
        init_buffer[n] = 0x00;
    init_buffer[19] = 0x00; //GRPPWM
    init_buffer[20] = 0x00; //GRPFREQ
    init_buffer[21] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[22] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[23] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[24] = 0xAA; //Drivers enabled PWM only (no GRPPWM)

    //Setup the update buffer.
    update_buffer[0] = 0x82; //Start at PWM0 with auto increase.
    for(uint8_t n=0; n<PCA9635_OUTPUT_COUNT; n++)
    {
        update_buffer[n+1] = 0x00;
        output_values[n] = 0x00;
    }

    i2cDriverExecuteAndWait(&init_command);
}

void setPCA9635output(uint8_t channel, uint8_t value)
{
    if (channel >= PCA9635_OUTPUT_COUNT)
        return;
    output_values[channel] = value;
}

void updatePCA9635()
{
    if (update_command.finished)
    {
        bool update = false;
        for(uint8_t n=0; n<PCA9635_OUTPUT_COUNT; n++)
        {
            if (output_values[n] != update_buffer[n + 1])
            {
                update = true;
                update_buffer[n + 1] = output_values[n];
            }
        }
        if (update)
            i2cDriverPlan(&update_command);
    }
}

void setPCA9635led(uint8_t tool, uint8_t red, uint8_t green, uint8_t blue)
{
    if (tool == 0)
    {
        setPCA9635output(12, red);
        setPCA9635output(11, green);
        setPCA9635output(10, blue);
    }
    else
    {
        setPCA9635output(13, red);
        setPCA9635output(14, green);
        setPCA9635output(15, blue);
    }
}
