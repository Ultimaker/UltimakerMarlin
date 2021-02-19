/*
    Copyright (c) 2018-2021 Ultimaker B.V. All rights reserved.

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
#ifndef CRC8_H
#define CRC8_H

#include <stdlib.h>
#include <inttypes.h>

/** @brief Based on the Crc articles of M. Barr
 * Implements an efficient CRC calculation mechanism
 */
class Crc8
{
public:
    /** Instantiates the CRC class
     *  @param message The message over which the CRC needs to be calculated
     *  @param nr_of_bytes The length of the message
     */
    Crc8(const uint8_t* message=NULL, const size_t nr_of_bytes=0);

    /** Adds additional message(s) that are needed for the CRC calculation
     *  @param message The message over which the CRC needs to be calculated
     *  @param nr_of_bytes The length of the message
     */
    void update(const void* message, const size_t nr_of_bytes);

    /** Adds a single byte to the CRC calculation
     *  @param value the value to be added
     */
    void update(const uint8_t value);

    //! Resets the CRC to itś initial state
    inline void reset()
    {
        result = 0x0;
    }

    //! @return Returns the calculated CRC
    inline uint8_t getCrc() const
    {
        return result;
    }

private:
    //! Keeps track of the calculated CRC
    uint8_t result;
};

#endif//CRC8_H
