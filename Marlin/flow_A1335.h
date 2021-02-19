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
#ifndef FLOW_A1335
#define FLOW_A1335

#define  __STDC_LIMIT_MACROS        // Required to get UINTxx_MAX macros to work
#include <stdint.h>
#include "i2c_driver.h"


const uint8_t   MAX_A1335_SENSOR_COUNT  = 4;    // Maximum number of A1335 flow sensors is limited by the number of I2C addresses == 4

class FlowA1335
{
public:
    // @brief   Constructs a device driver for the Allegro A1335 contactless magnetic angle position sensor IC.
    //          We access the A1335 over I2C. A total of 4 devices can be addressed (chip limitation) and every of those
    //          devices needs its own instance of this driver.
    //
    //          The A1335 detects a full 360 rotation with 12 bits resolution [0-4095 range]. The chip has its own signal
    //          processor which performs compensation for temperature and performs output linearization for situations
    //          where the chip is mounted with an offset to the magnet's center.
    //
    //          The A1335 will continuously sample the actual angle position but precision can be increased by averaging
    //          multiple samples, this is called 'setting the output rate'.
    //
    //          Reading 1 sample from the A1335 over I2C was measured to take about 0.5ms, hence an asynchronous read
    //          function was added. Start sampling by calling start() and when isBusy() returns false the result can be
    //          read by calling getRawAngle().
    // @param   Sensor_nr is used as reference number and from this the I2C address is derived.
    FlowA1335(uint8_t sensor_nr);

    // @brief   Initialization of sensor A1335.
    // @return  ERROR_SUCCESS when successful, ERROR_GENERIC_FAILURE on error.
    uint8_t init();

    // @brief   Check whether the sensor can be detected in the system.
    // @return  True when the sensor is present.
    bool isDiscovered();

    // @brief   Returns true when the sensor is still busy communicating.
    // @return  True when busy.
    bool isBusy();

    // @brief   Starts reading the angle data from the given sensor number.
    //          Since this is an asynchronous command over the relative slow I2C bus you will have to poll the
    //          isBusy function for readiness.
    // @return  True on success, False on error (not ready).
    bool start();

    // @brief   Returns the current raw sensor angle.
    //          The sensor value is a range 0-4095 which maps linear on a 0-360 degrees circle.
    //          Formula to convert the raw value to degrees:
    //               degrees = raw_angle / 4096
    //          Before calling this function you have to start the asynchronous reading by calling flowA1335Start().
    //          Since this is an asynchronous command over the relative slow I2C bus you will have to poll the
    //          isBusy function for readiness, otherwise this function returns UINT16_MAX.
    // @return  Current angle [0-4095]. Returns UINT16_MAX when sensor isn't ready.
    uint16_t getRawAngle();

    // @brief   Returns the field strength of the external magnet.
    //          Function blocks until response is returned.
    // @return  Magnetic field strength in Gauss.
    uint16_t getFieldStrength();

    // @brief   Returns the sensor's temperature.
    //          Function blocks until response is returned.
    // @return  Temperature in Celsius.
    float getTemperature();

    // @brief   Returns the magnetic angle.
    //          Function blocks until response is returned.
    //          Timed this function to take 0.53ms.
    // @return  Current angle [0-4095].
    uint16_t getAngleWait();

    // @brief   Set output rate (averaging).
    // @param   output_rate configures the refresh rate of the angle data, i.e. how many samples are averaged before output.
    //          0 =   1 sample,   31.25 μs update
    //          1 =   2 samples,  62.5 μs update
    //          2 =   4 samples, 125 μs update
    //          3 =   8 samples, 250 μs update
    //          4 =  16 samples, 500 μs update
    //          5 =  32 samples, 1 ms update
    //          6 =  64 samples, 2 ms update
    //          7 = 128 samples, 4 ms update
    // @return  True on success.
    bool setOutputRate(uint8_t output_rate);

    // @brief   Set the direction for increasing the magnetic angle.
    //          When the sensor is not yet initialized we just remember the setting for use in the next initialization and quit.
    // @param   direction_is_clockwise defines the direction for positive angle increments.
    //          false = Output angle value increases with counterclockwise rotation (viewing from above the magnet and device).
    //          true  = Output angle value increases with clockwise rotation (viewing from above the magnet and device).
    void setDirection(bool direction_is_clockwise);

    // @brief   Returns true when sensor is present.
    // @return  True when sensor is present.
    inline bool isSensorPresent()
    {
        return is_initialized;
    }


private:
    uint8_t sensor_nr;
    uint8_t sensor_address;
    bool    is_initialized;             // True when the sensor was discovered and initialized at start up.
    bool    dir_is_clockwise;           // True when the magnetic direction is clockwise, or False when counter clockwise.

    i2cCommand i2c_flow_write_command;
    uint8_t i2c_write_buffer[1];

    i2cCommand i2c_flow_read_command;
    uint8_t i2c_read_buffer[2];

    // @brief   Set the direction for increasing the magnetic angle.
    // @return  True on success, false on error.
    bool setMagneticDirectionClockwise();

    // @brief   Reset the A1335 device.
    //          Commands a hard reset, which resets digital logic, and resets the A1335's processor. Reloads values from EEPROM.
    //          The analog front end circuitry is not reset.
    // @return  True on success, false on error.
    bool resetDevice();

    // @brief   Unlock device (remains unlocked till power cycle or reset).
    //          Some commands for the A1335 are protected and need to be unlocked first.
    //          This command can always be written, even during Run mode.
    // @return  True on success, false on error.
    bool unlockDevice();

    // @brief   Sets the output angular offset to relocate the 0 degree reference point for the output angle.
    // @param   zero_offset is the offset angle [0-4095].
    // @return  True on success, false on error.
    bool setZeroOffset(uint16_t zero_offset);

    // @brief   Reads 2 bytes from the given register address.
    //          Function blocks until response is returned.
    // @param   register_address is the address to read from.
    // @return  The 16-bit value read.
    uint16_t readRegisterBlocking(uint8_t register_address);

    // @brief   Writes 2 bytes to the given register address.
    //          Function blocks until response is returned.
    // @param   register_address is the address to write to.
    // @param   value is the value to write in the specified register.
    void writeRegisterBlocking(uint8_t register_address, uint16_t value);

    // @brief   Returns True when the A1335 is in Run mode.
    //          Function blocks until response is returned.
    // @return  True when the A1335 is in Run mode.
    bool isInRunMode();

    // @brief   Sets the chip to either Running mode or Idle mode.
    // @param   set_running is set to True when the A1335 should go into Run Mode. Set to False when the chip should go to Idle Mode.
    // @return  True on success, false on error.
    bool setRunMode(bool set_running);

    // @brief   Write to extended address registers.
    // @param   address is the extended registers address.
    // @param   data is the 32-bit data to be written.
    // @return  True on success.
    bool extendedWrite(uint16_t address, uint32_t data);

    // @brief   Read from extended address registers.
    // @param   address is the extended registers address.
    // @param   data is a reference to the 32-bit read data.
    // @return  True on success.
    bool extendedRead(uint16_t address, uint32_t &data);
};

#endif /* FLOW_A1335 */

