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
#ifndef USART_SPI_DRIVER_H
#define USART_SPI_DRIVER_H

#include <stdint.h>

/** Driver to use a USART in SPI mode. */
class UsartSpiDriver
{
public:
    /**
     * Initialize the USART in SPI mode
     * @param mode: Mode of SPI, in the range 0 to 3
     */
    static void init(uint8_t mode=3);
    
    /**
     * Transcieve a single byte on SPI
     * @param data: single byte that will be send on SPI
     * @return single byte that is read back on SPI.
    */
    static uint8_t transceive(uint8_t data);
    /**
     * Transcieve multiple bites on SPI, and read back the result. The result will be placed in the buffer and will replace the send data.
     * @param buffer: The buffer that is transmitted on SPI byte for byte. Received data is also placed in this buffer.
     * @param buffer_size: The size in bytes of the buffer parameter.
     */
    static void transceive(uint8_t* buffer, uint8_t buffer_size);
};

#endif//USART_SPI_DRIVER_H
