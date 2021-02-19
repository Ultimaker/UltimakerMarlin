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
#include "PowerBudgetManagementHotendSlot.h"

PowerBudgetManagementHotendSlot::PowerBudgetManagementHotendSlot()
{
    // all defaults set here are meant to not cause errors or the heaters or bed being turned on until valid values are sent by griffin.
    requested_output = 0;
    max_power = 0;
    current_power_usage = 0;
    resistance = 1;
    voltage = 0;
    calculated_max_output = 0;
    calculated_power_usage = 0;
}

uint8_t PowerBudgetManagementHotendSlot::getActualHeaterOutput()
{
    uint8_t output = min(calculated_max_output, requested_output);
    current_power_usage = calculated_power_usage * output / PID_MAX;
    return output;
}
