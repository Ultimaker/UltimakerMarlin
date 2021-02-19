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
#include "fan_driver.h"
#include "pca9635_driver.h"
#include "Marlin.h"
#include "Board.h"

// UM3: Front fan connected to output 4 of the PCA9635
#define FAN_FRONT  3   /* Front location */

// UM3: Side fans connected to output 5 and 6 of the PCA9635
#define FAN_LEFT   4   /* Left location */
#define FAN_RIGHT  5   /* Right location */

static uint8_t current_fan_speed;

void initFans()
{
    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    SET_OUTPUT(UM2_HOTEND_FAN_PIN);

    if (Board::hasCaseFans())
    {
        // For some unknown reason analogWrite to this pin is not working. Using digital instead to turn either on or off
        SET_OUTPUT(CASE_FAN_24V_PIN);
        WRITE(CASE_FAN_24V_PIN, LOW);

        setCaseFanSpeed(0);
    }

#if (MATERIAL_COOLING_FAN_PIN > -1)
    SET_OUTPUT(MATERIAL_COOLING_FAN_PIN);
    #ifdef FAST_PWM_FAN
        setPwmFrequency(MATERIAL_COOLING_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
#endif
}

void setCaseFanSpeed(uint8_t fan_speed)
{
    if (!Board::hasCaseFans())
    {
        return;
    }

    switch (fan_speed)
    {
        case 0:
        {
            WRITE(CASE_FAN_24V_PIN, LOW);
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM("Case fan 24V OFF");
            break;
        }
        case 255:
        {
            WRITE(CASE_FAN_24V_PIN, HIGH);
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM("Case fan 24V ON");
            break;
        }
        default:
        {
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM("Case fan 24V, fan_speed = ");
            MSerial.print(fan_speed, DEC);
            SERIAL_ECHOLNPGM(" (pwm) is not supported!");
            break;
        }
    }
}

/** @brief Sets the fan speed
 *  @param fan_location Identify which fan
 *  @param fan_speed The speed of the fan
 */
static void setFanSpeed(uint8_t fan_location, uint8_t fan_speed)
{
    if (fan_location != FAN_LEFT &&
        fan_location != FAN_RIGHT &&
        fan_location != FAN_FRONT)
    {
        return;
    }

    setPCA9635output(fan_location, 255 - fan_speed);
}

void setCoolingFanSpeed(uint8_t fan_speed)
{
    setFanSpeed(FAN_LEFT, fan_speed);
    setFanSpeed(FAN_RIGHT, fan_speed);
#if MATERIAL_COOLING_FAN_PIN > -1
        analogWrite(MATERIAL_COOLING_FAN_PIN, fan_speed);
#endif
    current_fan_speed = fan_speed;
}

uint8_t getCoolingFanSpeed()
{
    return current_fan_speed;
}

void setHotendCoolingFanSpeed(uint8_t fan_speed)
{
    setFanSpeed(FAN_FRONT, fan_speed);

    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    if (fan_speed)
    {
        WRITE(UM2_HOTEND_FAN_PIN, HIGH);
    }
    else
    {
        WRITE(UM2_HOTEND_FAN_PIN, LOW);
    }
}
