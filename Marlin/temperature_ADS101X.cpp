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
#include <util/delay.h>

#include "Marlin.h"

#include "temperature_ADS101X.h"

#define ADS101X_REGISTER_DATA 0x00
#define ADS101X_REGISTER_CONFIG 0x01

#define ADS101X_CONFIG_MSB_OS _BV(7)

#define ADS101X_CONFIG_MSB_MUX(n) ((n) << 4)
//Mux setting for reading 1 specific channel
#define ADS101X_CONFIG_MSB_MUX_AIN(n) ADS101X_CONFIG_MSB_MUX(4 | (n))
//Mux setting for reading channel 0 in respect to channel 1.
#define ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN1 ADS101X_CONFIG_MSB_MUX(0)
//Mux setting for reading channel 0 to 2 in respect to channel 3. (Note that channel 0 is mux setting 1)
#define ADS101X_CONFIG_MSB_MUX_AIN0_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(1)
#define ADS101X_CONFIG_MSB_MUX_AIN1_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(2)
#define ADS101X_CONFIG_MSB_MUX_AIN2_MIN_AIN3 ADS101X_CONFIG_MSB_MUX(3)
#define ADS101X_CONFIG_MSB_MUX_AINn_MIN_AIN3(n) ADS101X_CONFIG_MSB_MUX((n) + 1)

#define ADS101X_CONFIG_MSB_PGA(n) ((n) << 1)
#define ADS101X_CONFIG_MSB_PGA_6V114 ADS101X_CONFIG_MSB_PGA(0)
#define ADS101X_CONFIG_MSB_PGA_4V096 ADS101X_CONFIG_MSB_PGA(1)
#define ADS101X_CONFIG_MSB_PGA_2V048 ADS101X_CONFIG_MSB_PGA(2)
#define ADS101X_CONFIG_MSB_PGA_1V024 ADS101X_CONFIG_MSB_PGA(3)
#define ADS101X_CONFIG_MSB_PGA_0V512 ADS101X_CONFIG_MSB_PGA(4)
#define ADS101X_CONFIG_MSB_PGA_0V256 ADS101X_CONFIG_MSB_PGA(5)
#define ADS101X_CONFIG_MSB_MODE_CONTINUOUS  0
#define ADS101X_CONFIG_MSB_MODE_SINGLE_SHOT _BV(0)
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

// The total number of consecutive read errors we allow before throwing a major error into the system.
#define MAXIMUM_ALLOWED_READ_ERRORS 10

TemperatureADS101X::TemperatureADS101X(uint8_t address)
{
    this->address = address;
    error_counter = 0;
    state = STATE_INIT;
    delay = 0;
    sample_channel_nr = 0;
    results[0] = results[1] = results[2] = 0;
}

void TemperatureADS101X::init()
{
    i2cDriverCommandSetup(i2c_adc_setup_command,    address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2c_adc_setup_buffer,    sizeof(i2c_adc_setup_buffer));
    i2cDriverCommandSetup(i2c_adc_pointer_command,  address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2c_adc_pointer_buffer,  sizeof(i2c_adc_pointer_buffer));
    i2cDriverCommandSetup(i2c_adc_read_command,     address << 1 | I2C_READ_BIT,  I2C_QUEUE_PRIO_HIGH, i2c_adc_read_buffer,     sizeof(i2c_adc_read_buffer));

    logAIN3();

    state = STATE_INIT;
}

void TemperatureADS101X::logAIN3()
{
    /**
        Read AIN3, which is connected to a register ladder of 3.3V - 1k21kOhm -|- 205Ohm - 0V
        The result reading should be 0.478V, which is an ADC value of 956 (if the PGA is set to 1.024V FS)
     */
    i2c_adc_setup_buffer[0] = ADS101X_REGISTER_CONFIG;
    i2c_adc_setup_buffer[1] = ADS101X_CONFIG_MSB_OS | ADS101X_CONFIG_MSB_MUX_AIN(3) | ADS101X_CONFIG_MSB_PGA_1V024 | ADS101X_CONFIG_MSB_MODE_SINGLE_SHOT;
    i2c_adc_setup_buffer[2] = ADS101X_CONFIG_LSB_DR_1600SPS | ADS101X_CONFIG_LSB_COMP_QUE_NONE;
    i2cDriverExecuteAndWait(&i2c_adc_setup_command);

    _delay_ms(1);

    i2c_adc_pointer_buffer[0] = ADS101X_REGISTER_DATA;
    i2cDriverExecuteAndWait(&i2c_adc_pointer_command);

    i2cDriverExecuteAndWait(&i2c_adc_read_command);

    storeResult(0);

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("ADS1015 #0x");
    MSerial.print(address, HEX);
    SERIAL_ECHOPGM(" ref.voltage=");
    MSerial.println(getResult(0), DEC);
}

void TemperatureADS101X::setupAIN(uint8_t channel)
{
    i2c_adc_setup_buffer[0] = ADS101X_REGISTER_CONFIG;
    i2c_adc_setup_buffer[1] = ADS101X_CONFIG_MSB_OS | ADS101X_CONFIG_MSB_MUX_AINn_MIN_AIN3(channel) | ADS101X_CONFIG_MSB_PGA_0V256 | ADS101X_CONFIG_MSB_MODE_SINGLE_SHOT;
    i2c_adc_setup_buffer[2] = ADS101X_CONFIG_LSB_DR_1600SPS | ADS101X_CONFIG_LSB_COMP_QUE_NONE;
    i2cDriverPlan(&i2c_adc_setup_command);
}

void TemperatureADS101X::requestResult()
{
    i2c_adc_pointer_buffer[0] = ADS101X_REGISTER_DATA;
    i2cDriverPlan(&i2c_adc_pointer_command);
    i2cDriverPlan(&i2c_adc_read_command);
}

bool TemperatureADS101X::isReady()
{
    if (!i2c_adc_setup_command.finished)
        return false;
    if (!i2c_adc_pointer_command.finished)
        return false;
    if (!i2c_adc_read_command.finished)
        return false;

    // Check for a read error
    if (i2c_adc_read_buffer[0] == 0xFF && i2c_adc_read_buffer[1] == 0xFF)
    {
        // Allow for some read errors. Occasionally, we cannot read the I2C chip due to signal problems to the print head.
        // Note that the class will keep the old ADC value until it can read the chip again.
        if (error_counter < MAXIMUM_ALLOWED_READ_ERRORS)
        {
            requestResult();
            error_counter++;
        }
        else
        {
            // Too many I2C errors. Stop the machine.
            switch(address)
            {
                case ADS101X_ADC_ADDRESS_HOTEND:
                    stop(STOP_REASON_I2C_HEAD_COMM_ERROR);
                    break;
                case ADS101X_ADC_ADDRESS_BED:
                    stop(STOP_REASON_I2C_BED_COMM_ERROR);
                    break;
            }
        }
        return false;
    }
    error_counter = 0;
    return true;
}

int16_t TemperatureADS101X::getResult(uint8_t channel)
{
    return (channel < MAX_CHANNELS) ? results[channel] : 0;
}

void TemperatureADS101X::storeResult(uint8_t channel)
{
    int16_t result = int16_t(i2c_adc_read_buffer[0]) << 4 | (i2c_adc_read_buffer[1] >> 4);
    if (result & 0x800)//Fix two complements result.
    {
        result |= 0xF000;
    }
    results[channel] = result;
}

void TemperatureADS101X::updateState()
{
    //We only trigger the state machine if our i2c communication is finished.
    if (isReady())
    {
        switch (state)
        {
        case STATE_INIT:
            sample_channel_nr = 0;
            state = STATE_SAMPLE_NEXT;
            break;
        case STATE_READ_RESULT:
            requestResult();
            state = STATE_STORE;
            break;
        case STATE_STORE:
            storeResult(sample_channel_nr);
            sample_channel_nr = (sample_channel_nr + 1) % MAX_CHANNELS;
            state = STATE_SAMPLE_NEXT;
            break;
        case STATE_SAMPLE_NEXT:
            setupAIN(sample_channel_nr);
            last_update_time = millis();
            delay = 2;
            state = STATE_WAIT;
            break;
        case STATE_WAIT:
            if (millis() - last_update_time > delay)
            {
                delay = 0;
                state = STATE_READ_RESULT;
            }
            break;
        }
    }
}
