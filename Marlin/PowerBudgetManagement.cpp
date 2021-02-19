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
#include "PowerBudgetManagement.h"

PowerBudgetManagement::PowerBudgetManagement(uint8_t nr_of_heaters)
{
    this->nr_of_heaters = min(nr_of_heaters, EXTRUDERS);
    idle_power_consumption = 0;
    total_power_budget = 0;
}

uint8_t PowerBudgetManagement::getActualBedOutput()
{
    float remaining_power_budget = total_power_budget - idle_power_consumption;
    for (uint8_t i = 0; i < nr_of_heaters; i++)
    {
        remaining_power_budget -= slots[i].getCurrentPowerUsage();
    }
    bed.setRemainingPowerBudget(remaining_power_budget);
    
    return bed.getActualHeaterOutput();
}
