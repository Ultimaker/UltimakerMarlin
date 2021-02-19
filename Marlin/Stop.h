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
#ifndef STOP_H
#define STOP_H

#include <inttypes.h>

// A list of error codes
// These are now located here for convenience, but when the list becomes larger it
// is warranted to move them to a separate file.
const uint8_t ERROR_SUCCESS = 0;
const uint8_t ERROR_GENERIC_FAILURE = 1;


#define STOP_REASON_MAXTEMP(e)              (1 + e * 50)
#define STOP_REASON_MINTEMP(e)              (2 + e * 50)
#define STOP_REASON_MAXTEMP_BED             3
#define STOP_REASON_HEATER_ERROR(e)         (4 + e * 50)
#define STOP_REASON_Z_ENDSTOP_BROKEN_ERROR  5
#define STOP_REASON_Z_ENDSTOP_STUCK_ERROR   6
#define STOP_REASON_XY_ENDSTOP_BROKEN_ERROR 7
#define STOP_REASON_XY_ENDSTOP_STUCK_ERROR  8
#define STOP_REASON_I2C_HEAD_COMM_ERROR     9
#define STOP_REASON_I2C_COMM_ERROR          10
#define STOP_REASON_SAFETY_TRIGGER          11
#define STOP_REASON_GCODE                   12
#define STOP_REASON_SERIAL_INPUT_TIMEOUT    13
#define STOP_REASON_POSITION_ERROR          14
#define STOP_REASON_PCB_INIT_POWERFAIL_N    15
#define STOP_REASON_PCB_INIT_PRE_CHARGE     16
#define STOP_REASON_PCB_INIT_HP_SW_SHUTDOWN 17
#define STOP_REASON_I2C_BED_COMM_ERROR      18

void stop(uint8_t reasonNr);
void clearStopReason();

bool isStopped();
uint8_t getStoppedReason();

#endif  //STOP_H
