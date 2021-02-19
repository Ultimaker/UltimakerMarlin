/*
    Copyright (c) 2017-2021 Ultimaker B.V. All rights reserved.

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
#include "flow_A1335.h"

#include "Marlin.h"

const uint8_t   A1335_ADDRESS_0 = 0x0C;     // 0b0001100
const uint8_t   A1335_ADDRESS_1 = 0x0D;     // 0b0001101
const uint8_t   A1335_ADDRESS_2 = 0x0E;     // 0b0001110
const uint8_t   A1335_ADDRESS_3 = 0x0F;     // 0b0001111

// Register definitions
const uint8_t   A1335_EWA_0     = 0x02;     // EWA - Extended Write Address
const uint8_t   A1335_EWA_1     = 0x03;     //
const uint8_t   A1335_EWD_0     = 0x04;     // EWD - Extended Write Data
const uint8_t   A1335_EWD_1     = 0x05;     //
const uint8_t   A1335_EWD_2     = 0x06;     //
const uint8_t   A1335_EWD_3     = 0x07;     //
const uint8_t   A1335_EWCS_0    = 0x08;     // EWCS - Extended Write Control and Status
const uint8_t   A1335_EWCS_1    = 0x09;     //
const uint8_t   A1335_ERA_0     = 0x0A;     // ERA - Extended Read Address
const uint8_t   A1335_ERA_1     = 0x0B;     //
const uint8_t   A1335_ERCS_0    = 0x0C;     // ERCS - Extended Read Control and Status
const uint8_t   A1335_ERCS_1    = 0x0D;     //
const uint8_t   A1335_ERD_0     = 0x0E;     // ERD - Extended Read Data
const uint8_t   A1335_ERD_1     = 0x0F;     //
const uint8_t   A1335_ERD_2     = 0x10;     //
const uint8_t   A1335_ERD_3     = 0x11;     //


const uint8_t   A1335_CTRL      = 0x1E;     // Device control
const uint8_t   A1335_ANG       = 0x20;     // Current angle data
const uint8_t   A1335_STA       = 0x22;     // Device status
const uint8_t   A1335_ERR       = 0x24;     // Device error status
const uint8_t   A1335_XERR      = 0x26;     // Extended error status
const uint8_t   A1335_TSEN      = 0x28;     // Temperature sensor data
const uint8_t   A1335_FIELD     = 0x2A;     // Magnetic field strength
const uint8_t   A1335_ERM       = 0x34;     // Device error status masking
const uint8_t   A1335_XERM      = 0x36;     // Extended error status masking

const uint8_t   A1335_ANGLE_HI_MASK = 0x0F; // Bits 8 .. 11 of the angle reading
const uint8_t   A1335_ANGLE_LO_MASK = 0xFF; // Bits 0 .. 7  of the angle reading

const uint32_t  A1335_UNLOCK_KEYCODE= 0x27811F77;   // Chip defined Customer Keycode for unlocking the protected capabilities.

const uint8_t   A1335_COMMAND_TIMEOUT = 10; // Commands shouldn't take more milliseconds than specified here.

// SRAM addresses
// Address 6
typedef union {
  struct {
    uint16_t ZeroOffset;    // 0 - 15: Zero Offset Angle
    int FI: 1;              // 16 Filter Enable
    int unused3: 1;         // 17 -
    int LR: 1;              // 18 Pre-Linearization Rotation Direction
    int HL: 1;              // 19 Harmonic Linearization
    int SL: 1;              // 20 Segmented Linearization
    int RO: 1;              // 21 Rotation Direction
    int IV: 1;              // 22 Invert Angle
    int unused2: 1;         // 23 -
    int SS: 1;              // 24 Short Stroke Mode
    int SB: 1;              // 25 Segmented Linearization Bypass
    int LS: 1;              // 26 Segmented Linearization Select
    int RD: 1;              // 27 Rotate Die
    int unused: 4;          // 28-31 -
  } reg;

  uint32_t all;
} SRAM_6;

// Used for endianness conversion
typedef union
{
    unsigned char bytes[4];
    uint32_t value;
} byte4_host_order;

FlowA1335::FlowA1335(uint8_t sensor_nr)
{
    this->sensor_nr = sensor_nr;        // Only stored for easy identification of the sensor in log messages.
    sensor_address = A1335_ADDRESS_0 + sensor_nr;
    is_initialized = false;
    dir_is_clockwise = true;
}

bool FlowA1335::isDiscovered()
{
    i2cCommand i2c_flow_discover_command;
    uint8_t i2c_flow_discover_buffer[1] = {};

    i2cDriverCommandSetup(i2c_flow_discover_command, sensor_address << 1 | I2C_READ_BIT, I2C_QUEUE_PRIO_LOW, i2c_flow_discover_buffer, sizeof(i2c_flow_discover_buffer));
    i2cDriverExecuteAndWait(&i2c_flow_discover_command);

    // Presence detection is very simple. When we receive data then there was an I2C device present and
    // we just assume it is a flow sensor.
    return i2c_flow_discover_buffer[0] != 0xFF;
}

uint8_t FlowA1335::init()
{
    SERIAL_ECHOPGM("LOG:flowA1335Init: sensor #");
    MSerial.print(sensor_nr, DEC);
    SERIAL_ECHOPGM(" sensor_address=0x");
    MSerial.println(sensor_address, HEX);

    i2cDriverCommandSetup(i2c_flow_write_command, sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, i2c_write_buffer, sizeof(i2c_write_buffer));
    i2cDriverCommandSetup(i2c_flow_read_command,  sensor_address << 1 | I2C_READ_BIT,  I2C_QUEUE_PRIO_MEDIUM, i2c_read_buffer,  sizeof(i2c_read_buffer));

    if (!resetDevice())
    {
        SERIAL_ECHOLNPGM("LOG:A1335 reset failed");
        return ERROR_GENERIC_FAILURE;
    }

    if (!unlockDevice())
    {
        SERIAL_ECHOLNPGM("LOG:A1335 unlock failed");
        return ERROR_GENERIC_FAILURE;
    }

    // Reset the angle zero offset value.
    if (!setZeroOffset(0))
    {
        SERIAL_ECHOLNPGM("LOG:A1335 failed setting zero offset");
        return ERROR_GENERIC_FAILURE;
    }

    // Set the magnetic sensor direction.
    if (!setMagneticDirectionClockwise())
    {
        SERIAL_ECHOLNPGM("LOG:A1335 failed to set magnetic direction");
        return ERROR_GENERIC_FAILURE;
    }

    SERIAL_ECHOPGM("LOG:A1335 FIELD=");
    MSerial.println(getFieldStrength(), DEC);

    SERIAL_ECHOPGM("LOG:A1335 Temperature=");
    MSerial.println(getTemperature());

    is_initialized = true;

    return ERROR_SUCCESS;
}

bool FlowA1335::isBusy()
{
      if (i2c_flow_write_command.finished && i2c_flow_read_command.finished)
      {
          return false;
      }

      return true;
}

bool FlowA1335::start()
{
    if (isBusy())
    {
        SERIAL_ECHOPGM("LOG:A1335start: error, unexpected busy for sensor #");
        MSerial.println(sensor_nr, DEC);
        return false;
    }

    i2c_write_buffer[0] = A1335_ANG;
    i2cDriverPlan(&i2c_flow_write_command);
    i2cDriverPlan(&i2c_flow_read_command);

    return true;
}

uint16_t FlowA1335::getRawAngle()
{
    if ((! isSensorPresent()) || isBusy())
    {
        return UINT16_MAX;
    }

    uint16_t angle = (uint16_t(i2c_read_buffer[0] & A1335_ANGLE_HI_MASK) << 8) + (i2c_read_buffer[1] & A1335_ANGLE_LO_MASK);
    return angle;
}

uint16_t FlowA1335::getFieldStrength()
{
    uint16_t strength = readRegisterBlocking(A1335_FIELD) & 0x0FFF;
    return strength;
}

float FlowA1335::getTemperature()
{
    uint16_t temperature_in_K = readRegisterBlocking(A1335_TSEN) & 0x0FFF;
    float temperature = (float(temperature_in_K) / 8) - 273.15;

    return temperature;
}

uint16_t FlowA1335::getAngleWait()
{
    if (!isSensorPresent())
    {
        return UINT16_MAX;
    }

    // Ensure we are not corrupting another ongoing transfer.
    unsigned long start_time = millis();
    while(isBusy())
    { // Wait

        if (millis() - start_time > A1335_COMMAND_TIMEOUT)
        {
            SERIAL_ERROR_START;
            SERIAL_ECHOPGM("A1335_FLOW_SENSOR_ERROR: timeout on reading flow angle, sensor=");
            MSerial.println(sensor_nr, DEC);
            return UINT16_MAX;
        }
    }

    uint16_t angle = readRegisterBlocking(A1335_ANG);
    return angle & 0x0FFF;
}

bool FlowA1335::setOutputRate(uint8_t output_rate)
{
    if (!isSensorPresent())
    {
        return false;
    }

    // Goto idle mode
    if (!setRunMode(false))
    {
        return false;
    }

    extendedWrite(0xFFD0, output_rate);

    // Return to run mode
    setRunMode(true);

    return true;
}

void FlowA1335::setDirection(bool direction_is_clockwise)
{
    dir_is_clockwise = direction_is_clockwise;

    // When the sensor is not yet initialized we just remember the setting and quit.
    if (!is_initialized)
    {
        SERIAL_ECHOPGM("LOG:A1335 Delayed setting direction sensor nr: ");
        MSerial.println(sensor_nr, DEC);
        return;
    }
    setMagneticDirectionClockwise();
}

bool FlowA1335::setMagneticDirectionClockwise()
{
    SRAM_6 sram_6;
    if (!extendedRead(0x06, sram_6.all))
    {
        return false;
    }

    sram_6.reg.LR = dir_is_clockwise;
    return extendedWrite(0x06, sram_6.all);
}

bool FlowA1335::resetDevice()
{
    // Hard reset command
    writeRegisterBlocking(A1335_CTRL, 0x20B9);

    // Wait for the reset to finish.
    unsigned long start_time = millis();
    uint16_t read_flags;
    do
    {
        // Make sure the reset is not taking too long
        if (millis() - start_time > 100)
        {
            SERIAL_ECHOPGM("LOG:Failed resetting A1335 sensor");
            MSerial.println(sensor_nr, HEX);
            return false;
        }

        read_flags = readRegisterBlocking(A1335_STA);
    } while ((read_flags & 0x000F) != 0x0001);

    return true;
}

bool FlowA1335::unlockDevice()
{
    if (!extendedWrite(0xFFFE, A1335_UNLOCK_KEYCODE))   // Write hard coded unlock key.
    {
        return false;
    }

    // Make sure the device is unlocked
    uint32_t flags;
    if (!extendedRead(0xFFF8, flags))
    {
        return false;
    }

    if (!(flags & 0x0020))
    {
        return false;
    }

    return true;
}

bool FlowA1335::setZeroOffset(uint16_t zero_offset)
{
    SRAM_6 sram_6;
    if (!extendedRead(0x06, sram_6.all))
    {
        return false;
    }
    sram_6.reg.ZeroOffset = zero_offset;
    return extendedWrite(0x06, sram_6.all);
}

uint16_t FlowA1335::readRegisterBlocking(uint8_t register_address)
{
    i2cCommand i2c_set_command;
    uint8_t i2c_cmd_buffer[1];
    i2c_cmd_buffer[0] = register_address;
    i2cDriverCommandSetup(i2c_set_command, sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_LOW, i2c_cmd_buffer, sizeof(i2c_cmd_buffer));
    i2cDriverPlan(&i2c_set_command);

    i2cCommand i2c_get_command;
    uint8_t i2c_read_buffer[2] = {};
    i2cDriverCommandSetup(i2c_get_command, sensor_address << 1 | I2C_READ_BIT, I2C_QUEUE_PRIO_LOW, i2c_read_buffer, sizeof(i2c_read_buffer));
    i2cDriverExecuteAndWait(&i2c_get_command);

    return (uint16_t(i2c_read_buffer[0]) << 8) + i2c_read_buffer[1];
}

void FlowA1335::writeRegisterBlocking(uint8_t register_address, uint16_t value)
{
    i2cCommand i2c_set_command;
    uint8_t i2c_cmd_buffer[3] = {};

    i2c_cmd_buffer[0] = register_address;
    i2c_cmd_buffer[1] = uint8_t(value >> 8);
    i2c_cmd_buffer[2] = uint8_t(value);
    i2cDriverCommandSetup(i2c_set_command, sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_LOW, i2c_cmd_buffer, sizeof(i2c_cmd_buffer));
    i2cDriverExecuteAndWait(&i2c_set_command);
}

bool FlowA1335::isInRunMode()
{
    uint16_t sta = readRegisterBlocking(A1335_STA);

    return (sta & 0x0F) == 0x01;
}

bool FlowA1335::setRunMode(bool set_running)
{
    uint16_t register_value = uint16_t((set_running ? 0xC0 : 0x80)) << 8;
    register_value += 0x46;                                      // A compulsory fixed key code for the Run command

    writeRegisterBlocking(A1335_CTRL, register_value);

    if (!set_running)
    {
        // Wait for A1335 to enter idle mode (can take up to 125us)
        unsigned long start_time = millis();
        while(isInRunMode())
        {
            // wait

            // Make sure the change to idle mode is not taking too long.
            if (millis() - start_time > A1335_COMMAND_TIMEOUT)
            {

                SERIAL_ECHOPGM("WARNING:A1335_FLOW_SENSOR_PROBLEM:");
                SERIAL_ECHO(uint16_t(sensor_nr));
                SERIAL_ECHOLNPGM(":timeout on entering idle mode");

                return false;
            }
        }
    }
    return true;
}

bool FlowA1335::extendedWrite(uint16_t address, uint32_t data)
{
    i2cCommand i2c_set_command;
    uint8_t i2c_cmd_buffer[9] = {};
    byte4_host_order convert;

    convert.value = data;

    i2c_cmd_buffer[0] = A1335_EWA_0;
    i2c_cmd_buffer[1] = uint8_t(address >> 8);  // 0x02 Address high
    i2c_cmd_buffer[2] = uint8_t(address);       // 0x03 Address low
    i2c_cmd_buffer[3] = convert.bytes[3];       // 0x04 Data high
    i2c_cmd_buffer[4] = convert.bytes[2];       // 0x05 Data
    i2c_cmd_buffer[5] = convert.bytes[1];       // 0x06 Data
    i2c_cmd_buffer[6] = convert.bytes[0];       // 0x07 Data low
    i2c_cmd_buffer[7] = 0x80;                   // 0x08 Initiate extended address Write
    i2c_cmd_buffer[8] = 0x00;                   // 0x09 EWCS low byte, does nothing but is used to create a multiple of 16-bits.
    i2cDriverCommandSetup(i2c_set_command, sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_LOW, i2c_cmd_buffer, sizeof(i2c_cmd_buffer));
    i2cDriverExecuteAndWait(&i2c_set_command);

    // Wait for the write to complete.
    unsigned long start_time = millis();
    uint16_t write_flags;
    do
    {
        // Make sure the write is not taking too long.
        if (millis() - start_time > A1335_COMMAND_TIMEOUT)
        {
            SERIAL_ECHOPGM("LOG:A1335 ExtendedWrite timed out. Sensor=");
            MSerial.print(sensor_nr, DEC);
            SERIAL_ECHOPGM(" addr=0x");
            MSerial.println(address, HEX);
            return false;
        }

        write_flags = readRegisterBlocking(A1335_EWCS_0);
    } while ((write_flags & 0x0001) != 0x0001);

    return true;
}

bool FlowA1335::extendedRead(uint16_t address, uint32_t &data)
{
    i2cCommand i2c_set_command;
    uint8_t i2c_cmd_buffer[5] = {};

    i2c_cmd_buffer[0] = A1335_ERA_0;
    i2c_cmd_buffer[1] = uint8_t(address >> 8);  // 0x0A Write the address to the Extended Read Address register
    i2c_cmd_buffer[2] = uint8_t(address);       // 0x0B
    i2c_cmd_buffer[3] = 0x80;                   // 0x0C Initiate extended address Read
    i2c_cmd_buffer[4] = 0x00;                   // 0X0D ERCS low byte, does nothing but is used to create a multiple of 16-bits.
    i2cDriverCommandSetup(i2c_set_command, sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_LOW, i2c_cmd_buffer, sizeof(i2c_cmd_buffer));
    i2cDriverExecuteAndWait(&i2c_set_command);

    // Wait for the read to complete.
    unsigned long start_time = millis();
    uint16_t read_flags;
    do
    {
        // Make sure the read is not taking too long.
        if (millis() - start_time > A1335_COMMAND_TIMEOUT)
        {
            SERIAL_ECHOPGM("LOG:A1335 ExtendedRead timed out. Sensor=");
            MSerial.print(sensor_nr, DEC);
            SERIAL_ECHOPGM(" addr=0x");
            MSerial.println(address, HEX);
            return false;
        }

        read_flags = readRegisterBlocking(A1335_ERCS_0);
    } while ((read_flags & 0x0001) != 0x0001);

    // Read result
    data = uint32_t(readRegisterBlocking(A1335_ERD_0)) << 16;
    data += readRegisterBlocking(A1335_ERD_2);
    return true;
}
