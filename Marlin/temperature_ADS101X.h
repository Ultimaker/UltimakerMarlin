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
#ifndef TEMPERATURE_ADS101X
#define TEMPERATURE_ADS101X

#include <stdint.h>
#include "i2c_driver.h"

#define ADS101X_ADC_ADDRESS_HOTEND 0b1001000
#define ADS101X_ADC_ADDRESS_BED 0b1001001

class TemperatureADS101X
{
public:
    /** @brief Initializes the Temperature class using the I2C address
     *  @param address The I2C address of the PT100 device
     */
    TemperatureADS101X(uint8_t address);

    /** @brief Initializes the I2C buffers
     */
    void init();

    /** @brief Returns the result of the specified channel
     *  @param channel The channel to get the result from (0, 1, or 2)
     *  @return Returns the stored result of the last read action, or 0 when channel is out of range (>= MAX_CHANNELS)
     */
    int16_t getResult(uint8_t channel);

    /** @brief This state machine will read MAX_CHANNELS channels in sequence.
     *         1. Send the request to read data for the next channel.
     *         2. Wait for data
     *         3. Read the data and store it.
     *         4. Restart from step 1.
     */
    void updateState();

private:
    /** @brief The maximum number of channels to read */
    static const uint8_t MAX_CHANNELS = 3;

    /** @brief Log some debug info about the voltage on the reference input 3 */
    void logAIN3();

    /** @brief Sets up the AIN for the given channel (request data)
     *  @param channel The channel to setup
     */
    void setupAIN(uint8_t channel);

    /** @brief Performs the request for result based on the data request setup
     */
    void requestResult();

    /** @brief Determine if the device is ready to send or receive data (including commands) over the I2C
     *  @return Returns true if ready, otherwise false
     */
    bool isReady();

    /** @brief Stores the result gotten from the ADS101X to the given channel in the internal buffer
     *  @param channel The channel from which the data is being read
     */
    void storeResult(uint8_t channel);

    uint8_t address;

    i2cCommand i2c_adc_setup_command;
    uint8_t i2c_adc_setup_buffer[3];

    i2cCommand i2c_adc_pointer_command;
    uint8_t i2c_adc_pointer_buffer[1];

    i2cCommand i2c_adc_read_command;
    uint8_t i2c_adc_read_buffer[2];

    uint8_t error_counter;

    /** @brief Keep track of the state */
    enum EState{
        STATE_INIT,
        STATE_READ_RESULT,
        STATE_STORE,
        STATE_SAMPLE_NEXT,
        STATE_WAIT,
    } state;
    /** @brief Delay counter, if contains a value higher than 1, it will delay the state machine update by this many miliseconds. */
    uint8_t delay;
    /** @brief Current actively sampling channel number, goes from 0 to 2, managed by the internal state machine */
    uint8_t sample_channel_nr;
    /** @brief Keep track of the last milli second time that we did an update. This to prevent updating in a tight loop */
    unsigned long last_update_time;

    /** @brief To store the results internally (only last value read) */
    int16_t results[MAX_CHANNELS];
};

#endif//TEMPERATURE_ADS101X
