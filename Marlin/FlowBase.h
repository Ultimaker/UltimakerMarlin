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
#ifndef FLOW_BASE_H
#define FLOW_BASE_H

#define  __STDC_LIMIT_MACROS        // Required to get UINTxx_MAX macros to work
#include <stdint.h>
#include <math.h>                   // For fabs()

/* This is an empty base class for the flow sensing implementation which allows us to easily switch between machines
   which have no flow sensors and machines which do have a flow sensor, i.e. instead of lots of checks for the flow
   sensor being present we can just switch a pointer to the class implementation.
*/



class FlowBase
{
public:
    // @brief   Constructor.
    FlowBase() {};

    // @brief   Initialize the flow module.
    virtual void init() {};

    // @brief  Initialize all flow position variables.
    virtual void initFlowData() {};
 
    // @brief   Configures the number of feeder steps required to move the filament 1mm.
    // @param   stepsPerMm is the number of steps to move the filament 1mm [float].
    virtual void setStepsPerMm(float stepsPerMm) {(void)stepsPerMm;};

    // @brief Sets the minimal extrusion as a factor of the desired extrusion
    // @param sensor_index The index of the sensor to set the factor for.
    // @param factor Factor to use.
    virtual void setMinimumExtrusionFactor(uint8_t /*sensor_index*/, float /*factor*/) {};

    // @brief   Set output rate (averaging) for all flow sensors.
    // @param   output_rate configures the refresh rate of the angle data, i.e. how many samples are averaged before output.
    //          0 =   1 sample,   31.25 μs update
    //          1 =   2 samples,  62.5 μs update
    //          2 =   4 samples, 125 μs update
    //          3 =   8 samples, 250 μs update
    //          4 =  16 samples, 500 μs update
    //          5 =  32 samples, 1 ms update
    //          6 =  64 samples, 2 ms update
    //          7 = 128 samples, 4 ms update
    virtual void setAllOutputRates(uint8_t /* output_rate */ ) {};

    // @brief   Get the raw angle sensor value for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The raw angle sensor value.
    virtual uint16_t getSensorRaw(uint8_t /* sensor_nr */) { return UINT16_MAX; };

    // @brief   Get the last E-axis position (in steps) for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The E-axis position in steps.
    virtual int32_t getExtrusionPosition(uint8_t /* sensor_nr */ ) { return 0; };

    // @brief   Get the extrusion position change for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The number of steps changed compared to the previous storage location.
    virtual int32_t getExtrusionPositionChange(uint8_t /* sensor_nr */) { return 0; };

    // @brief   Update the position tracking and flow sensor data.
    //          This function should be called at frequent intervals during printing (the more often is better for higher accuracy).
    //          In order to reduce system load this function will only do something when more than 1mm3 has been extruded.
    virtual void update() {};

    // @brief   Update extrusion flow position for active extruder.
    // @param   e_pos_in_steps is the number of feeder steps you want to set. Used when there is a sudden E-change without actual
    //          physical movement, for example after a G92 (Set current position) command.
    virtual void setExtrusionPosition(int32_t /* e_pos_in_steps */) {};

    // @brief   Update extrusion flow position for active extruder.
    //          Retrieves active feeder position and sensor position (blocking call, so not optimal)
    virtual void updateExtrusionPosition() {};

private:

    // @brief   Update extrusion flow position for active extruder.
    // @param   sensor_nr is the sensor number [0-3].
    // @param   current_angle_raw is the current sensor angle [0-4095].
    // @param   current_E_pos_in_steps  is the current E position in steps.
    virtual void updateExtrusionPosition(uint8_t /*sensor_nr */, uint16_t /* current_angle_raw */, int32_t /* current_E_pos_in_steps */) {};
};

#endif // FLOW_BASE_H
