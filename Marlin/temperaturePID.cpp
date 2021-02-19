/*
    Copyright (c) 2016-2021 Ultimaker B.V. All rights reserved.

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
#include "temperaturePID.h"
#include "fan_driver.h"
#include "stepper.h"

#define DUMP_PID_VALUES(temperature, ff, pcf, p, i, d, result) \
    if (debug_dump) { \
        SERIAL_ECHOPGM("||PID:"); \
        SERIAL_ECHO(temperature); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHO(ff); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHO(pcf); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHO(p); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHO(i); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHO(d); \
        SERIAL_ECHO(';'); \
        SERIAL_ECHOLN(int16_t(result)); \
    }

TemperaturePID::TemperaturePID()
{
    Kff = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    Ki_max = 255;
    Kpcf = 0.0;
    Ke = 0.0;
    Ke_extruder_index = 255;
    functional_range = DEFAULT_PID_FUNCTIONAL_RANGE;
    max_output = 255;
    min_output = 0;
    target_temperature = 0.0;

    i_state = 0.0;
    d_state = 0.0;
    previous_sample = 0.0;

    debug_dump = false;
}

uint8_t TemperaturePID::update(const float current_temperature)
{
    float error = target_temperature - current_temperature;
    float temperature_delta = current_temperature - previous_sample;
    uint8_t result;

    //Reject single bad samples, don't do PID control when our previous sample is so much out of line with our current sample.
    if (fabs(temperature_delta) > PID_IGNORE_TEMPERATURE_DELTA)
    {
        previous_sample = current_temperature;
        result = 0;

        DUMP_PID_VALUES(current_temperature, 0,0,0,0,0, result);
        return result;
    }

    if (error > functional_range)
    {
        resetState();
        result = max_output;

        DUMP_PID_VALUES(current_temperature, 0,0,0,0,0, result);
    }
    else if (error < -functional_range || target_temperature == 0)
    {
        result = min_output;

        DUMP_PID_VALUES(current_temperature, 0,0,0,0,0, result);
    }
    else
    {
        float p_term = Kp * error;
        i_state += error * Ki;
        i_state = constrain(i_state, -Ki_max, Ki_max);

        d_state = (1.0 - K1) * (Kd * temperature_delta) + (K1 * d_state);
        float ff_term = Kff * (target_temperature - FEED_FORWARD_MINIMAL_TEMPERATURE);
        float pcf_term = Kpcf * getMaterialCoolingFanSpeed();

        result = constrain(ff_term + pcf_term + p_term + i_state - d_state, min_output, max_output);

        DUMP_PID_VALUES(current_temperature, ff_term, pcf_term, p_term, i_state, d_state, result);
    }
    if (current_temperature < min_input || current_temperature > max_input)
    {
        result = min_output;
    }
    previous_sample = current_temperature;
    return result;
}

void TemperaturePID::resetState()
{
    i_state = 0.0;
    d_state = 0.0;
}
