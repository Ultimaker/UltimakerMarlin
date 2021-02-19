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
#include "GainCompensatorFactory.h"

#include "Configuration.h"

/** @brief Make sure there is always a compensator to return when calling getInstance, which uses a gain of 1.0 */
static GainCompensator defacto_compensator;

/** @brief Define the number of extruder compensators */
static GainCompensator extruder_compensators[EXTRUDERS];

/** @brief Function to initialize the static extruder compensators array with GainCompensator instances.
 *  The extruder_compensators[EXTRUDERS] = {} will not work!?
 */
static GainCompensator* initializeExtruderCompensators()
{
    for (uint8_t i=0; i<EXTRUDERS; i++)
    {
        extruder_compensators[i] = GainCompensator();
    }
    return extruder_compensators;
}

GainCompensator* GainCompensatorFactory::extruder_compensators = initializeExtruderCompensators();
GainCompensator GainCompensatorFactory::buildplate_compensator;

GainCompensator& GainCompensatorFactory::getInstance(const GainCompensatorFactory::CompensatorType compensator_type, const uint8_t id)
{
    /* Reinitialize the defacto_compensator to make sure that it will be a 1.0 gain at all times */
    defacto_compensator = GainCompensator(1.0, 0.0);

    switch (compensator_type)
    {
        case GainCompensatorFactory::EXTRUDER:
        {
            // Verify the id is less than the number of extruders
            if (id < EXTRUDERS)
            {
                return GainCompensatorFactory::extruder_compensators[id];
            }
            // Invalid extruder, return default
            return defacto_compensator;
        }
        break;

        case GainCompensatorFactory::BUILDPLATE:
        {
            return GainCompensatorFactory::buildplate_compensator;
        }
        break;

        default:
            // Invalid or unknown will return a default
            return defacto_compensator;
    }
}
