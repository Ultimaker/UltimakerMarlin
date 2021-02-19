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
#ifndef GAIN_COMPENSATOR_FACTORY
#define GAIN_COMPENSATOR_FACTORY

#include <stdint.h>
#include "GainCompensator.h"

/** @brief This factory makes sure that gain compensators will be created as a singleton for a given type and identification
 */
class GainCompensatorFactory
{
public:
    /** @brief Two types are possible right now */
    typedef enum
    {
        UNKNOWN = -1,
        EXTRUDER,
        BUILDPLATE,
        INVALID
    } CompensatorType;

    static GainCompensator& getInstance(const GainCompensatorFactory::CompensatorType compensator_type, const uint8_t id);

private:
    /** @brief We can keep track of multiple extruders, hence a list of compensators */
    static GainCompensator* extruder_compensators;
    /** @brief But there is only 1 buildplate */
    static GainCompensator buildplate_compensator;
};

#endif//GAIN_COMPENSATOR_FACTORY
