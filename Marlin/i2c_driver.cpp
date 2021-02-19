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
#include <avr/io.h>
#include <util/twi.h>
#include <util/atomic.h>

#include "Configuration.h"
#include "pins.h"
#include "fastio.h"
#include "i2c_driver.h"

#ifndef I2C_SDA_PIN
//I2C pin numbers for the ArduinoMega, which we assume by default if the pins are not set by pins.h.
#define I2C_SDA_PIN   20
#define I2C_SCL_PIN   21
#endif

//Clock frequency of the I2C driver during clock recovery. We use a higher rate here then during normal operation,
// as we are just trying to recover the clock. Introduced errors are no problem, and as we are doing this in an interrupt, we want to do it as fast as possible.
const uint32_t I2C_CLOCK_RECOVERY_FREQUENCY = 400000;  

static uint8_t current_command_buffer_index;
static i2cCommand* volatile current_command;
static i2cCommand* volatile command_queue;
static bool i2c_broken = false;

/*
 Function to delay in generating the I2C clock during initialization of the I2C peripheral.
 This delays the perioud of the I2C_CLOCK_FREQUENCY.
 */
static FORCE_INLINE void i2cClockDelay()
{
    _delay_us(1000000/I2C_CLOCK_RECOVERY_FREQUENCY);
}

/* I2C clock recovery.
   This function needs to be called when the bus should be idle.
   This function checks if the bus is actually idle, if it is not, then it will force the bus to be released.
 */
static FORCE_INLINE void i2cCheckForClockRecovery()
{
    if (!READ(I2C_SDA_PIN))
    {
        //Disable the I2C subsystem when we are recovering the clock, and the I2C subsystem cannot do this.
        TWCR = 0;
        //I2C Clock recovery:
        //Pulse the clock line as long as the data line is not released, this to release the bus from chips that might still be addressed in read mode,
        // and the slave still has the data line asserted.
        // We only loop up to 16 times, as we are running this from interrupt, and even if we are not recovering, we need to give the rest of the processor some time.
        for(uint8_t loop_count=0; loop_count<16 && !READ(I2C_SDA_PIN); loop_count++)
        {
            WRITE(I2C_SCL_PIN, 0);
            i2cClockDelay();
            WRITE(I2C_SCL_PIN, 1);
            i2cClockDelay();
        }

        //Generate a "fake" stop condition here. The I2C peripheral cannot generate a stop condition without claiming the bus.
        //But we want to make sure all slave chips have released the bus, even if they are in reading mode.
        i2cClockDelay();
        WRITE(I2C_SDA_PIN, 0);
        SET_OUTPUT(I2C_SDA_PIN);
        i2cClockDelay();
        SET_INPUT(I2C_SDA_PIN);
        WRITE(I2C_SDA_PIN, 1);
        i2cClockDelay();
        
        TWCR = _BV(TWIE) | _BV(TWEN);
    }
}

void i2cDriverInit()
{
    //Set the SDA pin as input so we can enable the pullup, set the SCL pin at output, as we want to force this line into a state.
    SET_INPUT(I2C_SDA_PIN);
    SET_OUTPUT(I2C_SCL_PIN);
    //Enable the internal pullups for the I2C driver. While the board has external pullups as well, this does not harm the functionality.
    WRITE(I2C_SCL_PIN, 1);
    WRITE(I2C_SDA_PIN, 1);
    
    //Disable the hardware I2C so we can enable it at the end of the init, and have it initialize properly, and clear all internal state.
    TWCR = 0;
    
    i2cClockDelay();

    //Set the clock frequency for the I2C by the following formula:
    //ClockFreq = (F_CPU) / (16 + 2*TWBR * 4^TWPS)
    //TWPS is set in TWSR to 0 for a prescaler of *1.
    TWBR = ((F_CPU / I2C_CLOCK_FREQUENCY) - 16)/(2*1);
    TWSR = 0x00;    //TWPS=0, other bits are read-only status bits.
    TWCR = _BV(TWIE) | _BV(TWEN);

    i2cCheckForClockRecovery();
}

static void i2cDriverExecuteNextCommand()
{
    //Called from interrupt context
    if (command_queue)
    {
        current_command = command_queue;
        command_queue = command_queue->next;
        current_command_buffer_index = 0;
        TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWSTA) | _BV(TWINT); //START will be transmitted, interrupt will be generated.
    }else{
        current_command = NULL;
        TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWSTO) | _BV(TWINT); //STOP condition will be transmitted and TWSTO Flag will be reset
    }
}

void i2cDriverPlan(i2cCommand* command)
{
    if (!command->finished)
        return;
    if (i2c_broken)
    {
        if (command->slave_address_rw & I2C_READ_BIT)
            memset(command->buffer, 0xFF, command->buffer_size);
        return;
    }
    command->finished = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (current_command == NULL)
        {
            // Wait till stop condition is transmitted, while unlikely to happen often,
            // but it could be that the stop condition is still being generated after the previous command was send,
            // at which point we want to wait for this before we initiate a new START action.
            loop_until_bit_is_clear(TWCR, TWSTO);

            current_command = command;
            current_command_buffer_index = 0;
            i2cCheckForClockRecovery(); //Before generating a start condition, check if the bus is free.
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWSTA) | _BV(TWINT); //START will be transmitted, interrupt will be generated.
        }else{
            if (command_queue == NULL || command_queue->priority < command->priority)
            {
                command->next = command_queue;
                command_queue = command;
            }else{
                for(i2cCommand* c = command_queue; ; c = c->next)
                {
                    if (c->next == NULL || c->next->priority < command->priority)
                    {
                        command->next = c->next;
                        c->next = command;
                        break;
                    }
                }
            }
        }
    }
}

void i2cDriverExecuteAndWait(i2cCommand* command)
{
    i2cDriverPlan(command);
    unsigned long start_time = millis();
    while(!command->finished)
    {
        if (millis() - start_time > 100)
        {
            SERIAL_ECHOPGM("LOG:I2C_COMM_ERROR:");
            SERIAL_ECHO(int(command->slave_address_rw));
            SERIAL_ECHO(':');
            SERIAL_ECHO(int(command->buffer_size));
            for(uint8_t n=0; n<command->buffer_size; n++)
            {
                SERIAL_ECHO(':');
                SERIAL_ECHO(int(command->buffer[n]));
            }
            SERIAL_ECHO('\n');
            stop(STOP_REASON_I2C_COMM_ERROR);
            i2c_broken = true;
            return;
        }
    }
}

//TODO: EM-2757 [New] - Research of ISR_NOBLOCK is possible so other interrupts can interrupt the I2C driver.
ISR(TWI_vect, ISR_BLOCK)
{
    //Called after the TWI has transmitted a START/REPEATED START condition
    //Called after the TWI has transmitted SLA+R/W
    //Called after the TWI has transmitted an address byte
    //Called after the TWI has lost arbitration
    //Called after the TWI has been addressed by own slave address or general call
    //Called after the TWI has received a data byte
    //Called after a STOP or REPEATED START has been received while still addressed as a Slave
    //Called when a bus error has occurred due to an illegal START or STOP condition
    switch(TW_STATUS)
    {
        /* Master */
    case TW_START: //0x08
    case TW_REP_START: //0x10
        /** start condition transmitted */
        /** repeated start condition transmitted */
        TWDR = current_command->slave_address_rw;
        TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT); //SLA+R/W will be transmitted. Depending on R/W, will be in transmit or receive.
        break;
        /* Master Transmitter */
    case TW_MT_SLA_ACK: //0x18
    case TW_MT_SLA_NACK: //0x20
    case TW_MT_DATA_ACK: //0x28
    case TW_MT_DATA_NACK: //0x30
        /** SLA+W transmitted, ACK received */
        /** SLA+W transmitted, NACK received */
        /** data transmitted, ACK received */
        /** data transmitted, NACK received */
        if (current_command_buffer_index < current_command->buffer_size)
        {
            TWDR = current_command->buffer[current_command_buffer_index++];
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT); //Data byte will be transmitted and ACK or NOT ACK will be received
        }else{
            current_command->finished = true;
            i2cDriverExecuteNextCommand();      // initiate a repeated START or a normal STOP action.
        }
        break;
        /* Master Receiver */
    case TW_MR_SLA_ACK: //0x40
        /** SLA+R transmitted, ACK received */
        // If reading only a single byte.
        if (current_command->buffer_size == 1)
        {
            // According to I2C standards: the master terminates multiple reads sequence by NACK-ing the last byte, then STOP.
            // The NAK tells the slave transmitter to let go of control of the SDA line so that the master can send a STOP.
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT);              // Data byte will be received and NACK will be returned
        } else {
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT) | _BV(TWEA);  // Data byte will be received and ACK will be returned
        }
        break;
    case TW_MR_SLA_NACK: //0x48
        /** SLA+R transmitted, NACK received */
        //Failed to address slave. Clear the buffer and mark command as finished.
        memset(current_command->buffer, 0xFF, current_command->buffer_size);
        current_command->finished = true;
        i2cDriverExecuteNextCommand();
        break;
    case TW_MR_DATA_ACK: //0x50
        /** data received, ACK returned */
        current_command->buffer[current_command_buffer_index++] = TWDR;
        // Is the next byte the last byte to be received?
        if (current_command_buffer_index == current_command->buffer_size - 1)
        {
            // According to I2C standards: the master terminates multiple reads sequence by NACK-ing the last byte, then STOP.
            // The NAK tells the slave transmitter to let go of control of the SDA line so that the master can send a STOP.
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT);              // Data byte will be received and NACK will be returned
        } else {
            TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWINT) | _BV(TWEA);  // Data byte will be received and ACK will be returned
        }
        break;
    case TW_MR_DATA_NACK: //0x58
        /** data received, NACK returned */
        current_command->buffer[current_command_buffer_index] = TWDR;   // Read the last byte.
        current_command->finished = true;
        i2cDriverExecuteNextCommand();  // initiate a repeated START or a normal STOP action.
        break;

    /* Misc */
    case TW_MT_ARB_LOST: //0x38
    //case TW_MR_ARB_LOST: //0x38
    case TW_BUS_ERROR: //0x00
    default:
        /** arbitration lost in SLA+W or data */
        /** arbitration lost in SLA+R or NACK */
        /** illegal start or stop condition */
        //Normal TWI action is to become a slave on arbitration lost. But we cannot become a slave, as there is no other master.
        //In case of a BUS error or any other state that we cannot handle, reset the whole TWI module and start from the beginning.
        //Release the TWI module and restart it. So we can become a new master again. (This can happen due to noise on the line)
        TWCR = 0;
        TWCR = _BV(TWIE) | _BV(TWEN);
        i2cCheckForClockRecovery(); //Before generating a start condition, check if the bus is free.
        TWCR = _BV(TWIE) | _BV(TWEN) | _BV(TWSTA) | _BV(TWINT); //START will be transmitted
        current_command_buffer_index = 0;
        break;
    }
}
