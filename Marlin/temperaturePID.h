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
#ifndef TEMPERATURE_PID_H
#define TEMPERATURE_PID_H

#include <stdint.h>
#include "Configuration.h"
#include "LowPassFilter.h"

/**
    PID controller for temperature.
    Uniform implementation for an advanced PID controller.
    For standard PID controller information, look at https://en.wikipedia.org/wiki/PID_controller
    Runs at a fixed time interval controlled by the define PID_dT.
    Has the following standard control features:
    * Kp = proportional, part of standard PID controllers.
    * Ki = integral, part of standard PID controllers.
    * Kd = derivative, part of standard PID controllers.
    It has the following additional control features not found in standard PID controllers
    * Kff = Feed-forward. This uses the actual setpoint to generate a certain amount of output.
    * Kpcf = Print cooling fan feed-forward. Uses the print fan speed to add a certain amount of output.

    Additional features are:
    * Functional range, if the setpoint differs more than this value a simple "on/off" logic is used.
    * Control range, outside of this control range the output is always off.
*/
class TemperaturePID
{
public:
    TemperaturePID();

    /**
     Update the PID controller. Call when a new temperature sample is ready.
     Implementation assumes this is called every PID_dT.
     @param current_temperature current temperature sample in degree celsius.
     @param time_delta time passed since previous update in seconds
     @return pwm value for the output in the range 0 to 255.
     */
    uint8_t update(float current_temperature, float time_delta);

    /**
     Set the target temperature to a new value.
     @param temperature in degree celsius.
     */
    FORCE_INLINE void setTargetTemperature(float temperature)
    {
        target_temperature = temperature;
    }

    /**
     Get the target temperature
     @return target temperature in degree celsius.
     */
    FORCE_INLINE float getTargetTemperature()
    {
        return target_temperature;
    }

    /**
        Modify the Kff factor.
        output = Kff * target_temperature
        @param Kff new Kff factor.
     */
    FORCE_INLINE void setKff(float Kff)
    {
        this->Kff = Kff;
    }

    /**
        Modify the Kp factor.
        output = Kp * (current_temperature - target_temperature)
        @param Kp new Kp factor.
     */
    FORCE_INLINE void setKp(float Kp)
    {
        this->Kp = Kp;
    }

    /**
        Modify the Ki factor.
        i_state += (current_temperature - target_temperature)
        output = Ki * i_state
        @param Ki new Ki factor.
     */
    FORCE_INLINE void setKi(float Ki)
    {
        this->Ki = Ki;
        this->i_state = 0;
    }

    /**
        Set the maximum value for the integral. Limiting the amount of output the integral can provide.
        i_state = constrain(i_state, -Ki_max, Ki_max)
        @param Ki_max new maximum i_state
     */
    FORCE_INLINE void setKiMax(float Ki_max)
    {
        this->Ki_max = Ki_max;
    }

    /**
        Modify the Kd factor.
        output = Kd * (previous_temperature - current_temperature)
        @param Kd new Kd factor.
     */
    FORCE_INLINE void setKd(float Kd)
    {
        this->Kd = Kd;
        this->d_state = 0;
    }

    /**
        Modify the Kpcf factor, Kpcf of 1.0 = 100% fan => 100% PWM output.
        output = Kpcf * current_fan_speed
        @param Kpcf new Kpcf factor.
     */
    FORCE_INLINE void setKpcf(float Kpcf)
    {
        this->Kpcf = Kpcf;
    }

    /**
        Set the functional range for this PID controller.
        Outside of this range from the target temperature the controller does a simple on/off logic.
        @param functional_range range of this controller, measured from the target temperature in degree celsius.
    */
    FORCE_INLINE void setFunctionalRange(float functional_range)
    {
        this->functional_range = functional_range;
    }

    /**
        Set the control range for this PID controller. If the current temperature is outside of this control range the PID controller always returns "off"
        @param min_input minimal measured temperature for this controller to function.
        @param max_input maximum measured temperature for this controller to function.
    */
    FORCE_INLINE void setControlRange(float min_input, float max_input)
    {
        this->min_input = min_input;
        this->max_input = max_input;
    }

    /**
        Set the debug dump flag.
        The debug dump feature will dump the PID state of this PID controller on each update cycle.
        This will be send to opinicus, which will log it in /tmp/
        @param dump True if this PID controller needs to debug dump.
    */
    FORCE_INLINE void setDebugDump(bool dump)
    {
        debug_dump = dump;
    }

private:
    /* PID configuration values. */
    /** Basic feed forward factor configuration. */
    float Kff;
    /** Proportional factor configuration. */
    float Kp;
    /** Integral factor configuration. */
    float Ki;
    /** Derivative factor configuration. */
    float Kd;
    /** Maximum value for the integral buildup, to prevent large overshoot due to integral buildup. */
    float Ki_max;
    /** Print cooling fan feed forward factor. Multiplied with the current fan speed to compensate for the cooling fan drawing away heat from the hotends. */
    float Kpcf;
    /** Extrusion feed forward factor. If the current active extruder equals the extruder linked to this controller then this factor is multiplied by the amount of mm E movement that is done */
    float Ke;
    /** Linked extruder for the Ke factor. 255 if not set. */
    uint8_t Ke_extruder_index;
    /** Maximum difference from the current temperature and the target temperature in which the controller does the advanced control loop. */
    float functional_range;
    /** Minimum input temperature for the PID controller to function. */
    float min_input;
    /** Maximum input temperature for the PID controller to function. */
    float max_input;
    /*8 Maximum PWM output that will be returned by this controller, defaults to 255 */
    uint8_t max_output;
    /** Minimum PWM output that will be returned by this controller, defaults to 0 */
    uint8_t min_output;
    /*< Target temperature for the PID controller in degree celsius */
    float target_temperature;

    /* PID runtime values */

    /*< State of the integral, updated every update call, and added to the output. */
    float i_state;
    /** State of the derivative, updated every update call, and slightly smoothed, to prevent noise in the derivative. */
    float d_state;

    /** The low pass filter for the derivative term (d_state) **/
    LowPassFilter d_state_lpf;

    /** Previous temperature sample, used to calculate the derivative */
    float previous_sample;
    /** Previous error from setpoint to target */
    float previous_error;

    /** Dump the PID controller state on each update trough the serial port. Note that this causes high bandwith use. */
    bool debug_dump;

    /**
        Reset the internal state of the PID controller.
        This is called when the PID controller decides that it has no work to do, and thus internal tracked values need to be reset to prevent old build up state
        to propagate to the next time the PID controller does work.
    */
    void resetState();
};

#endif//TEMPERATURE_PID_H
