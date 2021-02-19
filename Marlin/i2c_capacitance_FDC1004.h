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
#ifndef I2C_CAPACITANCE_H
#define I2C_CAPACITANCE_H

#ifdef ENABLE_BED_LEVELING_PROBE

#include <stdint.h>

void i2cCapacitanceInit();
void i2cCapacitanceReset();
void i2cCapacitanceSetCAPDAC(uint8_t value);
bool i2cCapacitanceDone(int16_t& value);

#endif//ENABLE_BED_LEVELING_PROBE

#endif//I2C_CAPACITANCE_H
