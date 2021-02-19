#include <string.h>
#include <avr/io.h>
#include <util/twi.h>

#include "i2c.h"

void TWI_vect();

I2CSim::I2CSim()
{
    TWCR.setCallback(DELEGATE(registerDelegate, I2CSim, *this, I2C_TWCR_callback));
    
    state = Idle;
    for(int n=0; n<128; n++)
        i2cDevice[n] = NULL;
}

I2CSim::~I2CSim()
{
}

#define WRITE_REQUEST 0
#define START_REQUEST _BV(TWSTA)
#define STOP_REQUEST _BV(TWSTO)
#define RESTART_REQUEST (_BV(TWSTA) | _BV(TWSTO))

void I2CSim::I2C_TWCR_callback(uint8_t oldValue, uint8_t& newValue)
{
    if ((oldValue ^ newValue) == _BV(TWIE) && (newValue & _BV(TWIE)))
        return;
    if ((newValue & _BV(TWINT)) && (newValue & _BV(TWEN)))
    {
        //Requested new TWI action
        uint8_t start_stop_state = newValue & (_BV(TWSTA) | _BV(TWSTO));
        switch(state)
        {
        case Idle:
            if (start_stop_state == START_REQUEST)
            {
                //Generate start condition.
                state = StartInitiated;
                TWSR = TW_START;
                if (newValue & _BV(TWIE))
                    TWI_vect();
            }else{
                printf("I2C: Unknown start/stop request in [Idle] state: %02x\n", start_stop_state);
                exit(1);
            }
            break;
        case StartInitiated:
            if (start_stop_state == WRITE_REQUEST)
            {
                target_address = TWDR >> 1;
                i2cMessagePos = 0;
                if (TWDR & TW_READ)
                {
                    state = MasterRead;
                    TWSR = TW_MT_SLA_ACK;
                    if (i2cDevice[target_address])
                    {
                        TWSR = TW_MR_SLA_ACK;
                        i2cDevice[target_address]->readI2C(i2cMessage);
                    }else{
                        printf("I2C: read from unknown device with address: %02x\n", target_address);
                        memset(i2cMessage, 0xFF, sizeof(i2cMessage));
                        TWSR = TW_MR_SLA_NACK;
                    }
                }else{
                    state = MasterWrite;
                    if (i2cDevice[target_address])
                        TWSR = TW_MT_SLA_ACK;
                    else
                        TWSR = TW_MT_SLA_NACK;
                }
                if (newValue & _BV(TWIE))
                    TWI_vect();
            }else{
                printf("I2C: Unknown start/stop request in [StartInitiated] state: %02x\n", start_stop_state);
                exit(1);
                state = Idle;
            }
            break;
        case MasterWrite:
            if (start_stop_state == WRITE_REQUEST)
            {
                //Load data
                i2cMessage[i2cMessagePos] = TWDR;
                i2cMessagePos++;
                if (i2cDevice[target_address])
                    TWSR = TW_MT_DATA_ACK;
                else
                    TWSR = TW_MT_DATA_NACK;
                if (newValue & _BV(TWIE))
                    TWI_vect();
            }else if (start_stop_state == STOP_REQUEST)
            {
                if (i2cDevice[target_address])
                {
                    i2cDevice[target_address]->writeI2C(i2cMessage, i2cMessagePos);
                }else{
                    printf("I2C: write to unknown device with address: %02x\n", target_address);
                    exit(1);
                }
                newValue &=~_BV(TWSTO);
                state = Idle;
            }else{
                printf("I2C: Unknown start/stop request in [MasterWrite] state: %02x\n", start_stop_state);
                exit(1);
                state = Idle;
            }
            break;
        case MasterRead:
            if (start_stop_state == 0x00)
            {
                //Load data
                TWDR = i2cMessage[i2cMessagePos];
                i2cMessagePos++;
                if (i2cDevice[target_address])
                {
                    if (newValue & _BV(TWEA))
                        TWSR = TW_MR_DATA_ACK;
                    else
                        TWSR = TW_MR_DATA_NACK;
                }
                else
                {
                    TWSR = TW_MR_DATA_NACK;
                }
                if (newValue & _BV(TWIE))
                    TWI_vect();
            }else if (start_stop_state == STOP_REQUEST)
            {
                newValue &=~_BV(TWSTO);
                state = Idle;
            }else{
                printf("I2C: Unknown start/stop request in [MasterRead] state: %02x\n", start_stop_state);
                exit(1);
                state = Idle;
            }
            break;
        }
    }
}

void I2CSim::registerDevice(int id, I2CDevice* device)
{
    if (id < 0 || id > 127)
        return;
    i2cDevice[id] = device;
}
