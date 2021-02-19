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
#ifndef STEPPER_TMC2130_H
#define STEPPER_TMC2130_H

#include <stdint.h>

/** TMC2130 stepper motor driver controller
    
    Step, direction and enable is directly controlled by stepper.cpp for runtime speed efficiency.
 */
class StepperTMC2130
{
public:
    static void init();
    
    static uint32_t readRegister(uint8_t stepper_index, uint8_t register_index);
    static void writeRegister(uint8_t stepper_index, uint8_t register_index, uint32_t value);
};

#endif//STEPPER_TMC2130_H
