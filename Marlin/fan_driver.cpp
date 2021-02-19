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
#include "Marlin.h"
#include "Board.h"

static uint8_t current_fan_speed;
bool topcap_fan_is_on = false;

void initFans()
{
    SET_OUTPUT(HOTEND_FAN_PIN);
    setHotendCoolingFanSpeed(0);

    if (Board::hasCaseFans())
    {
        pinMode(CASE_FAN_24V_PIN, OUTPUT);
        setCaseFanSpeed(0);
    }

#if (MATERIAL_COOLING_FAN_PIN > -1)
    pinMode(MATERIAL_COOLING_FAN_PIN, OUTPUT);
    setMaterialCoolingFanSpeed(0);

    #ifdef FAST_PWM_FAN
        setPwmFrequency(MATERIAL_COOLING_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
#endif

    pinMode(TOPCAP_FAN_PIN, OUTPUT);
    setTopCapFanSpeed(0);
}

void setCaseFanSpeed(uint8_t fan_speed)
{
    if (!Board::hasCaseFans())
    {
        return;
    }

    analogWrite(CASE_FAN_24V_PIN, fan_speed);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Case fan 24V, speed = ");
    MSerial.print(fan_speed, DEC);
    SERIAL_ECHOLN("");
}

void setMaterialCoolingFanSpeed(uint8_t fan_speed)
{
#if MATERIAL_COOLING_FAN_PIN > -1
    analogWrite(MATERIAL_COOLING_FAN_PIN, fan_speed);
#endif
    current_fan_speed = fan_speed;
}

uint8_t getMaterialCoolingFanSpeed()
{
    return current_fan_speed;
}

void setHotendCoolingFanSpeed(uint8_t fan_speed)
{
    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    if (fan_speed)
    {
        WRITE(HOTEND_FAN_PIN, HIGH);
    }
    else
    {
        WRITE(HOTEND_FAN_PIN, LOW);
    }
}

void setTopCapFanSpeed(uint8_t fan_speed)
{
    if (target_topcap_fan_speed == TOP_CAP_FAN_AUTO_MODE)
    {
        analogWrite(TOPCAP_FAN_PIN, fan_speed);
        topcap_fan_is_on = (fan_speed > 0) && TOPCAP_IS_PRESENT;
    }
    else
    {
        analogWrite(TOPCAP_FAN_PIN, target_topcap_fan_speed);
        topcap_fan_is_on = (target_topcap_fan_speed > 0) && TOPCAP_IS_PRESENT;
    }
}
