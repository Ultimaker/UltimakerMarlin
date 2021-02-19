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
#ifndef POWER_BUDGET_MANAGEMENT_H
#define POWER_BUDGET_MANAGEMENT_H

#include <stdint.h>

#include "Configuration.h"
#include "PowerBudgetManagementBed.h"
#include "PowerBudgetManagementHotendSlot.h"


class PowerBudgetManagement
{
public:
    /**
     * Construct a power budget Management object, once power levels have been set, power suppression/squelching of the bed
     * (the bed always uses the most power) power consumption can be done to avoid overloading the power supply.
     * @param nr_of_heaters The amount of heaters excluding the bed as the bed is a special case.
     */
    PowerBudgetManagement(uint8_t nr_of_heaters);

    /**
     * @param power_budget Maximum amount of power that the printer can use in Watts.    
     */
    FORCE_INLINE void setTotalPowerBudget(float power_budget) { total_power_budget = power_budget; };

    /**
     * @param idle_power_consumption Amount of power always lost due to cpu's + stepper motors, etc.
     */
    FORCE_INLINE void setIdlePowerConsumption(float idle_power_consumption) { this->idle_power_consumption = idle_power_consumption; };

    /**
     * Set the nominal heater resistance.
     * @param heater_index The heater to set the power level of.
     * @param resistance The nominal hotend resistance (no temperature is specified since the resistance changes little for these heaters).
     */
    FORCE_INLINE void setNominalHotendResistance(uint8_t heater_index, float resistance) { if (heater_index < nr_of_heaters) this->slots[heater_index].setNominalResistance(resistance); };

    /**
     * @param heater_index The heater to set the voltage level of.
     * @param voltage The voltage used to power the hotend.
     */
    FORCE_INLINE void setHotendVoltage(uint8_t heater_index, float voltage) { if (heater_index < nr_of_heaters) this->slots[heater_index].setVoltage(voltage); };
    
    /**
     * @param heater_index The heater to set the power level of.
     * @param max_power Maximum allowed power usage in watts for the printer in this heater/slot.
     */
    FORCE_INLINE void setMaxPowerUsageForHeater(uint8_t heater_index, float max_power) { if (heater_index < nr_of_heaters) this->slots[heater_index].setMaxPowerUsage(max_power); };

    /**
     * @param bed_voltage The voltage used to power the bed.
     */
    FORCE_INLINE void setBedVoltage(float bed_voltage) { bed.setBedVoltage(bed_voltage); };

    /**
     * @param nominal_bed_resistance The nominal bed resistance at 60 degC (60 degC since values measured at the low end proved inaccurate in measurements).
     */
    FORCE_INLINE void setNominalBedResistance(float nominal_bed_resistance) { bed.setNominalResistance(nominal_bed_resistance); };

    /**
     * @param bed_resistance_per_degree the bed resistance changes linearly based on temperature, so to calculate the power used we need to calculate the resistance first based on the temperature of the bed.
     */
    FORCE_INLINE void setBedResistancePerDegree(float bed_resistance_per_degree) { bed.setResistancePerDegree(bed_resistance_per_degree); };

    /**
     * @param heater_index The heater to set the power level of.
     * @param output The 0-255 output to be sent to the heater
     */
    FORCE_INLINE void setRequestedHeaterOutput(uint8_t heater_index, uint8_t output) { if (heater_index < nr_of_heaters) this->slots[heater_index].setRequestedHeaterOutput(output); };
    FORCE_INLINE uint8_t getRequestedHeaterOutput(uint8_t heater_index) { if (heater_index < nr_of_heaters) return this->slots[heater_index].getRequestedHeaterOutput(); return 0; };    

    /**
     * @param output The 0-255 output to be sent to the heater
     */
    FORCE_INLINE void setRequestedBedOutput(uint8_t output) { bed.setRequestedHeaterOutput(output); };

    /**
     * Set the bed temperature as it's resistance is somewhat temperature dependent.
     * @param temp The temperature in degrees Celsius.
     */
    FORCE_INLINE void setCurrentBedTemperature(float temp) { bed.setCurrentBedTemperature(temp); };
    
    /**
     * Get the currently calculated heater output.
     * @param heater_index The heater to get the power level of.
     * @return The 0-255 output value to be set as the PWM level.
     */
    FORCE_INLINE uint8_t getActualHeaterOutput(uint8_t heater_index)
    {
        if (heater_index < nr_of_heaters)
            return this->slots[heater_index].getActualHeaterOutput();
        return 0;
    };

    /**
     * Get the currently calculated heater output.
     * @return The 0-255 output value to be set as the PWM level.
     */
    uint8_t getActualBedOutput();

private:
    uint8_t nr_of_heaters;

    PowerBudgetManagementHotendSlot slots[EXTRUDERS];
    PowerBudgetManagementBed bed;
    float total_power_budget;
    float idle_power_consumption;
};

#endif /* POWER_BUDGET_MANAGEMENT_H */
