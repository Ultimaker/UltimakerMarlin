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
#include "Stop.h"
#include "Marlin.h"
#include "temperature.h"
#include "Board.h"
#include "stepper.h"

static uint8_t stopped_reason = 0;             // 0 == not stopped, other value == stop reason


void stop(uint8_t reasonNr)
{
    disable_all_heaters();
    Board::powerDownSafely();
    if(stopped_reason == 0)
    {
        stopped_reason = reasonNr;

        // no message otherwise griffin would report another error...
        if (reasonNr != STOP_REASON_GCODE)
        {
            SERIAL_ERROR_START;
        }
        switch(reasonNr)
        {
        case STOP_REASON_MAXTEMP(0):
            SERIAL_ECHOLNPGM("EXTRUDER_MAXTEMP_ERROR:0: Extruder switched off. MAXTEMP triggered !");
            break;
        case STOP_REASON_MAXTEMP(1):
            SERIAL_ECHOLNPGM("EXTRUDER_MAXTEMP_ERROR:1: Extruder switched off. MAXTEMP triggered !");
            break;
        case STOP_REASON_MINTEMP(0):
            SERIAL_ECHOLNPGM("EXTRUDER_MINTEMP_ERROR:0: Extruder switched off. MINTEMP triggered !");
            break;
        case STOP_REASON_MINTEMP(1):
            SERIAL_ECHOLNPGM("EXTRUDER_MINTEMP_ERROR:1: Extruder switched off. MINTEMP triggered !");
            break;
        case STOP_REASON_MAXTEMP_BED:
            SERIAL_ECHOPGM("BED_MAXTEMP_ERROR: Temperature heated bed switched off. MAXTEMP triggered !!");
            break;
        case STOP_REASON_HEATER_ERROR(0):
            SERIAL_ECHOLNPGM("HEATER_ERROR:0");
            break;
        case STOP_REASON_HEATER_ERROR(1):
            SERIAL_ECHOLNPGM("HEATER_ERROR:1");
            break;
        case STOP_REASON_Z_ENDSTOP_BROKEN_ERROR:
            SERIAL_ECHOLNPGM("Z_ENDSTOP_ERROR: Endstop not pressed after homing down. Endstop broken?");
            break;
        case STOP_REASON_Z_ENDSTOP_STUCK_ERROR:
            SERIAL_ECHOLNPGM("Z_ENDSTOP_ERROR: Endstop still pressed after backing off. Endstop stuck?");
            break;
        case STOP_REASON_XY_ENDSTOP_BROKEN_ERROR:
            SERIAL_ECHOLNPGM("XY_ENDSTOP_ERROR: Endstop not pressed after homing down. Endstop broken?");
            break;
        case STOP_REASON_XY_ENDSTOP_STUCK_ERROR:
            SERIAL_ECHOLNPGM("XY_ENDSTOP_ERROR: Endstop still pressed after backing off. Endstop stuck?");
            break;
        case STOP_REASON_I2C_HEAD_COMM_ERROR:
            SERIAL_ECHOLNPGM("I2C_HEAD_COMM_ERROR");
            break;
        case STOP_REASON_I2C_COMM_ERROR:
            SERIAL_ECHOLNPGM("I2C_COMM_ERROR");
            break;
        case STOP_REASON_SAFETY_TRIGGER:
            SERIAL_ECHOLNPGM("SAFETY_ERROR: Safety circuit triggered");
            break;
        case STOP_REASON_SERIAL_INPUT_TIMEOUT:
            SERIAL_ECHOLNPGM("SERIAL_INPUT_TIMEOUT: No commands received over time. Safety shutdown");
            break;
        case STOP_REASON_POSITION_ERROR:
            SERIAL_ECHOLNPGM("POSITION_ERROR: Ordered to move outside of normal volume.");
            break;
        case STOP_REASON_PCB_INIT_POWERFAIL_N:
            SERIAL_ECHOLNPGM("PCB INIT: POWERFAIL line did not signal good within .5s.");
            break;
        case STOP_REASON_PCB_INIT_PRE_CHARGE:
            SERIAL_ECHOLNPGM("PCB INIT: Voltage level did not drop below 6V within 1s.");
            break;
        case STOP_REASON_PCB_INIT_HP_SW_SHUTDOWN:
            SERIAL_ECHOLNPGM("PCB INIT: HP output voltage is too low.");
            break;
        case STOP_REASON_I2C_BED_COMM_ERROR:
            SERIAL_ECHOLNPGM("I2C_BED_COMM_ERROR");
            break;
        case STOP_REASON_GCODE:
            // no message since it was initiated by gcode
            break;
        default:
            SERIAL_ECHOLNPGM(MSG_ERR_STOPPED);
            break;
        }
    }
}

void clearStopReason()
{
    stopped_reason = 0;
}

bool isStopped()
{
    return stopped_reason;
}

uint8_t getStoppedReason()
{
    return stopped_reason;
}
