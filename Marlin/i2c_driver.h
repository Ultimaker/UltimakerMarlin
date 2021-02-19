/*
    Copyright (c) 2014-2021 Ultimaker B.V. All rights reserved.

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
#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
/**
An AVR I2C driver using the TWI hardware in the AVR controller.

It has the following design considerations:
* Interrupt based. No busy waiting for normal operation.
* Automated retry on failure. And reporting back to the application when finished.
* User defined memory buffers, means there is no limit on buffer sizes, and the application holds the buffers.
* No dynamic memory, no dynamic configuration. All pieces of memory and settings are static defined at compile time.
* Priorities: Some I2C communication is more important than others, schedule the next action on priority.
*/

const uint32_t I2C_CLOCK_FREQUENCY = 100000;  //Clock frequency of the I2C driver. Most chips support both 100khz and 400khz operation.
const uint8_t I2C_WRITE_BIT = 0x00;
const uint8_t I2C_READ_BIT = 0x01;

// Messages with a higher priority will be sent from the I2C command queue first.
const uint8_t I2C_QUEUE_PRIO_LOW = 1;
const uint8_t I2C_QUEUE_PRIO_MEDIUM = 50;
const uint8_t I2C_QUEUE_PRIO_HIGH = 100;


struct i2cCommand {
    uint8_t priority;           //Priority of the command (higher takes preference)
    uint8_t slave_address_rw;   //Address and Read/Write bit of the slave ship (SLA+RW)
    uint8_t* buffer;            //Pointed to the read or write buffer. This buffer is send when RW bit is i2cWrite. Or filled when the RW bit is i2cWrite
    uint8_t buffer_size;        //Size of the buffer in bytes.
    volatile uint8_t finished;  //false when the i2c command is being executed. During executing all other fields should not be accessed. True when the execution is finished.

    //Private, do not access, used by the priority queue
    i2cCommand* volatile next;
};

/**
    Initialize the I2C driver, called during setup.
 */
void i2cDriverInit();
/**
    Plan a i2cCommand to be executed by the I2C driver. The command is scheduled and executed with interrupts.
    You can call this function from interrupt context.
    Do not call this function when the i2cCommand.finished is false.
 */
void i2cDriverPlan(i2cCommand* command);
/**
    Execute i2cCommand, and wait for it to finish before returning.
    Do not call this function from interrupt context.
    Do not call this function when the i2cCommand.finished is false.
 */
void i2cDriverExecuteAndWait(i2cCommand* command);
/**
    Setup the I2C command structure.
    Set the priority (higher priority commands take preference over lower)
    Sets the I2C address + read/write bit (SLA+RW)
    And sets the buffer used for reading or writing and the size of that buffer.
 */
static inline void i2cDriverCommandSetup(i2cCommand& command, uint8_t address_rw, uint8_t priority, uint8_t* buffer, uint8_t buffer_size)
{
    command.priority = priority;
    command.slave_address_rw = address_rw;
    command.buffer = buffer;
    command.buffer_size = buffer_size;
    command.finished = true;
}

#endif//I2C_DRIVER_H
