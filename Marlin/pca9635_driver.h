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
#ifndef PCA9635_DRIVER_H
#define PCA9635_DRIVER_H

#include <stdint.h>

void initPCA9635();

/** @brief sets up the command to set the value to the specified channel
 *  @param channel The channel to write to
 *  @param value The value to write
 */
void setPCA9635output(uint8_t channel, uint8_t value);

/** @brief Check if the PCA9635 needs to be updated, and schedule an update command it required.
 *  This should be called from the main loop.
 */
void updatePCA9635();

/** @brief sets head led x to a specific rgb color
 *  @param tool the toolhead
 *  @param red the red value of the rgb set
 *  @param green the green value of the rgb set
 *  @param blue the blue value of the rgb set
 */
void setPCA9635led(uint8_t tool, uint8_t red, uint8_t green, uint8_t blue);

#endif//PCA9635_DRIVER_H
