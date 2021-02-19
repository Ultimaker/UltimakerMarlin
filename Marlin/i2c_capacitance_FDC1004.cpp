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
#include "i2c_capacitance_FDC1004.h"

#ifdef ENABLE_BED_LEVELING_PROBE

//FDC1004 4-Channel Capacitance-to-Digital Converter
static const uint8_t I2C_SENSOR_ADDRESS = 0b1010000;

#define ADDR_MEAS_MSB(n)       (0x00 + (n) * 2)
#define ADDR_MEAS_LSB(n)       (0x01 + (n) * 2)
#define ADDR_CONF_MEAS(n)      (0x08 + (n))
#define ADDR_FDC_CONF          (0x0C)
#define ADDR_OFFSET_CAL_CIN(n) (0x0D + (n))
#define ADDR_GAIN_CAL_CIN(n)   (0x11 + (n))
#define ADDR_MANUFACTURER_ID   (0xFE)
#define ADDR_DEVICE_ID         (0xFF)

#define CONF_MSB_CHA_CIN1          (0x00)
#define CONF_MSB_CHA_CIN2          (0x20)
#define CONF_MSB_CHA_CIN3          (0x40)
#define CONF_MSB_CHA_CIN4          (0x60)

#define CONF_MSB_CHB_CIN1          (0x00)
#define CONF_MSB_CHB_CIN2          (0x04)
#define CONF_MSB_CHB_CIN3          (0x08)
#define CONF_MSB_CHB_CIN4          (0x0C)
#define CONF_MSB_CHB_CAPDAC        (0x10)
#define CONF_MSB_CHB_DISABLED      (0x1C)

#define CONF_MSB_CAPDAC_OFFSET(n)  (((n) >> 3) & 0x03)
#define CONF_LSB_CAPDAC_OFFSET(n)  ((n) << 5)

#define FDC_CONF_MSB_RESET         (0x80)
#define FDC_CONF_MSB_RATE_100HZ    (0x04)
#define FDC_CONF_MSB_RATE_200HZ    (0x08)
#define FDC_CONF_MSB_RATE_400HZ    (0x0C)
#define FDC_CONF_MSB_REPEAT        (0x01)
#define FDC_CONF_LSB_MEAS_EN(n)    (0x80 >> (n))
#define FDC_CONF_LSB_MEAS_DONE(n)  (0x08 >> (n))

static i2cCommand i2cSensorWriteCommand;
static uint8_t i2cSensorWriteBuffer[1];
static i2cCommand i2cSensorReadCommand;
static uint8_t i2cSensorReadBuffer[2];

enum capstate_t { FIRSTRUN, WAITING, READ };


void i2cCapacitanceInit()
{
    i2cCommand i2cSensorInitCommand;
    uint8_t i2cSensorInitBuffer[3];

    i2cDriverCommandSetup(i2cSensorWriteCommand, I2C_SENSOR_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2cSensorWriteBuffer, sizeof(i2cSensorWriteBuffer));
    i2cDriverCommandSetup(i2cSensorReadCommand, I2C_SENSOR_ADDRESS << 1 | I2C_READ_BIT, I2C_QUEUE_PRIO_HIGH, i2cSensorReadBuffer, sizeof(i2cSensorReadBuffer));

    i2cDriverCommandSetup(i2cSensorInitCommand, I2C_SENSOR_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2cSensorInitBuffer, sizeof(i2cSensorInitBuffer));

    i2cSensorInitBuffer[0] = ADDR_FDC_CONF;
    if (CONFIG_BED_LEVELING_SAMPLE_FREQUENCY == 100)
    {
        i2cSensorInitBuffer[1] = FDC_CONF_MSB_RATE_100HZ;
    }
    else if (CONFIG_BED_LEVELING_SAMPLE_FREQUENCY == 200)
    {
        i2cSensorInitBuffer[1] = FDC_CONF_MSB_RATE_200HZ;
    }
    else if (CONFIG_BED_LEVELING_SAMPLE_FREQUENCY == 400)
    {
        i2cSensorInitBuffer[1] = FDC_CONF_MSB_RATE_400HZ;
    }
    i2cSensorInitBuffer[1] |= FDC_CONF_MSB_REPEAT;
    i2cSensorInitBuffer[2] = FDC_CONF_LSB_MEAS_EN(0);
    i2cDriverExecuteAndWait(&i2cSensorInitCommand);

    i2cSensorWriteBuffer[0] = ADDR_MEAS_MSB(0);
}

void i2cCapacitanceReset()
{
    i2cCommand i2cSensorResetCommand;
    uint8_t i2cSensorResetBuffer[2];

    i2cDriverCommandSetup(i2cSensorResetCommand, I2C_SENSOR_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2cSensorResetBuffer, sizeof(i2cSensorResetBuffer));

    i2cSensorResetBuffer[0] = ADDR_FDC_CONF;
    i2cSensorResetBuffer[1] = FDC_CONF_MSB_RESET;
    i2cDriverExecuteAndWait(&i2cSensorResetCommand);
}

void i2cCapacitanceSetCAPDAC(uint8_t value)
{
    i2cCommand i2cSensorCapdacCommand;
    uint8_t i2cSensorCapdacBuffer[3];

    i2cDriverCommandSetup(i2cSensorCapdacCommand, I2C_SENSOR_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_HIGH, i2cSensorCapdacBuffer, sizeof(i2cSensorCapdacBuffer));

    i2cSensorCapdacBuffer[0] = ADDR_CONF_MEAS(0);

    if (value == 0)
    {
        // disable CAPDAC
        i2cSensorCapdacBuffer[1] = CONF_MSB_CHA_CIN1 | CONF_MSB_CHB_DISABLED;
        i2cSensorCapdacBuffer[2] = 0x00;
    }
    else
    {
        // set CAPDAC
        i2cSensorCapdacBuffer[1] = CONF_MSB_CHA_CIN1 | CONF_MSB_CHB_CAPDAC | CONF_MSB_CAPDAC_OFFSET(value);
        i2cSensorCapdacBuffer[2] = CONF_LSB_CAPDAC_OFFSET(value);
    }
    i2cDriverExecuteAndWait(&i2cSensorCapdacCommand);
}

// TODO: EM-2758 - timeout as in i2cDriverExecuteAndWait() (with timeout value in header file)
bool i2cCapacitanceDone(int16_t& value)
{
    static capstate_t capstate = FIRSTRUN;

    // all commands finished?
    if (!i2cSensorWriteCommand.finished || !i2cSensorReadCommand.finished)
        return false;

    switch (capstate)
    {
    case FIRSTRUN:
        // read sensor status
        i2cSensorWriteBuffer[0] = ADDR_FDC_CONF;
        i2cDriverPlan(&i2cSensorWriteCommand);
        i2cDriverPlan(&i2cSensorReadCommand);
        capstate = WAITING;
        return false;

    case WAITING:
        // measurement completed?
        if (i2cSensorReadBuffer[1] & FDC_CONF_LSB_MEAS_DONE(0))
        {
            // new value available; issue read command
            i2cSensorWriteBuffer[0] = ADDR_MEAS_MSB(0);
            i2cDriverPlan(&i2cSensorWriteCommand);
            i2cDriverPlan(&i2cSensorReadCommand);
            capstate = READ;
            return false;
        }
        else
        {
            // measurement in progress; issue another read status command
            i2cSensorWriteBuffer[0] = ADDR_FDC_CONF;
            i2cDriverPlan(&i2cSensorWriteCommand);
            i2cDriverPlan(&i2cSensorReadCommand);
            return false;
        }

    case READ:
        value = int16_t(i2cSensorReadBuffer[0]) << 8 | i2cSensorReadBuffer[1];
        capstate = FIRSTRUN;
        return true;

    default:
        // should never reach this point (keep compiler happy)
        return false;
    }
}

#endif//ENABLE_BED_LEVELING_PROBE
