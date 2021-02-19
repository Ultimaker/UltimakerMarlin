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
#ifndef CAP_SENSE_PROBE_H
#define CAP_SENSE_PROBE_H

// each state explained:
// INIT     call initialization code and move to FILLING state
// FILLING  block analysis until the buffers have been filled
// WORKING  perform analysis
// DONE     analysis is complete and has found the inflection point
// CONTINUE collect some samples pressing the buildplate and nozzle together
//          a bit more after already being DONE (for debugging only)
// ERROR    a serious error has been detected; analysis will stop
enum probe_state_t { INIT = 0, FILLING, WORKING, DONE, CONTINUE, ERROR };

// For communicating probe state and result. (storing global data)
struct ProbeResult
{
    enum probe_state_t state;
    const char* error_message;
    float z;
};

/**
 *  Probe the bed with the capacitive sensor. Report back the Z position where the bed is hit.
 *  Does not change the X/Y position. Sets the Z position on the reported Z position.
 */
ProbeResult probeWithCapacitiveSensor(const float start_position[], const int16_t feedrate, const int16_t verbosity, const float move_distance, const float extra_z_move);

/**
 *  Make a move in the Z direction only while logging alle capacitive sensor data.
 *  Does not change the X/Y position. Sets the Z position on the reported Z position.
 *  @param feedrate: speed of which to move in mm/minute
 *  @param move_distance: distance moved in Z that the bed is moved upwards, in mm.
 */
void moveWithCapacitiveSensor(const int16_t feedrate, const float move_distance);

/**
 * Compensate any static capacitance by setting the CAPDAC. Finds best value by increasing until
 * measured capacitance is closest to zero, but still positive.
 */
void calibrateCapacitanceOffset(int16_t verbosity);

/**
 * sets the base capacitive offsets based on the average of a couple of current samples from the capacitive sensor
 */
void updateCapacitiveSensorBaseline(int16_t verbosity);

#endif//CAP_SENSE_PROBE_H
