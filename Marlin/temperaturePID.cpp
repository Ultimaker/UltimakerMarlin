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
#include <stdio.h>


TemperaturePID::TemperaturePID() : d_state_lpf(PID_D_LPF_RC_CONSTANT)
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
    previous_error = 0.0;

    debug_dump = false;
}

uint8_t TemperaturePID::update(const float current_temperature, const float time_delta)
{
    float error = target_temperature - current_temperature;
    float temperature_delta = current_temperature - previous_sample;
    uint8_t result;

    if(debug_dump)
    {
        fprintf_P(stderr, PSTR("PID: dt: %.1f\n"), time_delta);
    }

    //Reject single bad samples, don't do PID control when our previous sample is so much out of line with our current sample.
    if (fabs(temperature_delta) > PID_IGNORE_TEMPERATURE_DELTA)
    {
        previous_sample = current_temperature;
        result = 0;

        if (debug_dump)
        {
            fprintf_P(stderr, PSTR("PID: ignore sample\n"));
        }

        return result;
    }

    if (error > functional_range)
    {
        resetState();
        result = max_output;

        if (debug_dump)
        {
            fprintf_P(stderr, PSTR("PID: max output\n"));
        }
    }
    else if (error < -functional_range || target_temperature == 0)
    {
        result = min_output;

        if (debug_dump)
        {
            fprintf_P(stderr, PSTR("PID: min output\n"));
        }
    }
    else
    {
        i_state += error * time_delta;
        i_state = constrain(i_state, -Ki_max, Ki_max);

        d_state = (error - previous_error) / time_delta;
        d_state_lpf.take_input(d_state, time_delta);
        float d_state_filtered = d_state_lpf.get_output();

        float ff_term = Kff * (target_temperature - FEED_FORWARD_MINIMAL_TEMPERATURE);
        float pcf_term = Kpcf * getMaterialCoolingFanSpeed();
        float p_term = Kp * error;
        float i_term = Ki * i_state;
        float d_term = Kd * d_state_filtered;
        float output = ff_term + pcf_term + p_term + i_term + d_term;
        result = constrain(output, min_output, max_output);

        if (debug_dump) {
            fprintf_P(stderr, PSTR("PID: err: %.1f ff: %.1f pcf: %.1f p: %.1f i: %.1f d: %.1f out: %d\n"),
                    error,
                    ff_term,
                    pcf_term,
                    p_term,
                    i_term,
                    d_term,
                    result);
        }
    }

    if (current_temperature < min_input || current_temperature > max_input)
    {
        result = min_output;
    }

    previous_sample = current_temperature;
    previous_error = error;
    return result;
}

void TemperaturePID::resetState()
{
    i_state = 0.0;
    d_state = 0.0;
}
