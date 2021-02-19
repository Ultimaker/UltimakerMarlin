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
#ifndef ONEWIRE_DS2431_EEPROM_H
#define ONEWIRE_DS2431_EEPROM_H

#define DS2431_NR_OF_PAGES 4
#define DS2431_PAGE_SIZE 32
#define DS2431_CHUNK_SIZE 8
#define DS2431_SERIAL_SIZE 6

bool oneWireDS2431ReadSerial(uint8_t index, uint8_t serial[6]);
bool oneWireDS2431Read(uint8_t index, uint8_t address, uint8_t* data, uint8_t count);
bool oneWireDS2431WritePage(uint8_t index, uint8_t address, const uint8_t data[8]);

#endif//ONEWIRE_DS2431_EEPROM_H
