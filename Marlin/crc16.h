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
#ifndef CRC16_H
#define CRC16_H

#include <stdlib.h>
#include <inttypes.h>

/** Based on the Crc articles of M. Barr
 *  Implements an effecient CRC calculation mechanism, polynomial is the CCITT 0x1021
 */
class Crc16
{
public:
    /** Instantiates the CRC class
     *  @param message The message over which the CRC needs to be calculated
     *  @param nr_of_bytes The length of the message
     */
    Crc16(const uint8_t* message=NULL, const size_t nr_of_bytes=0);

    /** Adds additional message(s) that are needed for the CRC calculation
     *  @param message The message over which the CRC needs to be calculated
     *  @param nr_of_bytes The length of the message
     */
    void update(const uint8_t* message, const size_t nr_of_bytes);

    /** Adds a single byte to the CRC calculation
     *  @param value the value to be added
     */
    void update(const uint8_t value);

    //! Resets the CRC to itś initial state
    inline void reset()
    {
        // CCITT start with 0xFFFF
        result = 0xFFFF;
    }

    //! @return Returns the calculated CRC
    inline uint16_t getCrc() const
    {
        return result;
    }

private:
    //! Keeps track of the calculated CRC
    uint16_t result;
};

#endif//CRC16_H