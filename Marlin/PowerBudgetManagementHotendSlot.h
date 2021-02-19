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
#ifndef POWER_BUDGET_MANAGEMENT_HEATER_H
#define POWER_BUDGET_MANAGEMENT_HEATER_H

#include <stdint.h>
#include "Configuration.h"


/**
 * This class is used to track all values needed to calculate the power usage of a heater, it can also be used to limit the power output of a heater.
 */
class PowerBudgetManagementHotendSlot
{
public:
    PowerBudgetManagementHotendSlot();

    /**
     * This function takes the output of the PID system and stores it, after setting this the getActualHeaterOutput()
     * function can be used to get the possibly smaller value that is to be set the PWM duty-cycle of the heater.
     * @param output The Requested output.
     */
    FORCE_INLINE void setRequestedHeaterOutput(uint8_t output) { requested_output = output; };
    FORCE_INLINE uint8_t getRequestedHeaterOutput() { return requested_output; }
    
    /**
     * @return The 0-255 output to be set the PWM duty-cycle of the heater.
     */
    uint8_t getActualHeaterOutput();
    
    /**
     * @return The power usage in wats since the last call to getActualHeaterOutput()
     */
    FORCE_INLINE float getCurrentPowerUsage() { return current_power_usage; };
    
    /**
     * @param voltage The voltage used to power the bed.
     */
    FORCE_INLINE void setVoltage(float voltage) { this->voltage = voltage; recalculateMaxOutput(); };
    
    /**
     * This function sets the nominal heater resistance.
     * @param resistance The nominal resistance in ohms (no temperature is specified since the resistance changes little for these heaters).
     */
    FORCE_INLINE void setNominalResistance(float resistance) { this->resistance = resistance; recalculateMaxOutput(); };
    
    /**
     * This function sets the maximum power usage of the heater.
     * @param power The maximum power usage in watts.
     */
    FORCE_INLINE void setMaxPowerUsage(float power) { max_power = power; recalculateMaxOutput(); };

private:
    /**
     * This function is a private inline function because it is only used to recalculate class data when input values change.
     * @return The max pid output (in a 0-255 range) that this heater can be set to given the MaxPowerUsage, the NominalResistance and the Voltage.
     */
    FORCE_INLINE void recalculateMaxOutput()
    {
        // The math here is very simple but just in case:
        // P = power in watts, U = tension in Volts, I = current in Ampere and R is resistance in Ohm.
        // P = U * I
        // R = U/I
        // I = U/R
        // P = U * U / R or P = U^2 / R
        if (resistance == 0.0) //Special case, if the resistance is specified as 0, then this is infinite resistance, and thus the power usage is zero.
        {
            calculated_max_output = 0;
        }else{
            calculated_power_usage = voltage * voltage / resistance;
            // this is combined with the max_power setting to calculate the maximum PID_output allowed.
            calculated_max_output = min(PID_MAX, PID_MAX * max_power / calculated_power_usage);
        }
    }
    
    uint8_t requested_output;
    uint8_t calculated_max_output;
    float max_power;
    float resistance;
    float voltage;
    float calculated_power_usage;
    float current_power_usage;
};

#endif /* POWER_BUDGET_MANAGEMENT_HEATER_H */
