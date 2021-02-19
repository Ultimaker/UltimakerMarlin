/*
    Copyright (c) 2016-2020 Ultimaker B.V. All rights reserved.

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
#ifndef WATCHDOG_H
#define WATCHDOG_H

// initialise watch dog with a 4 sec interrupt time
void watchdog_init();
// pet the dog/reset watchdog. MUST be called at least every second after the first watchdog_init or avr will go into emergency procedures..
void watchdog_reset();

#endif
