/*
    Copyright (c) 2016-2021 Ultimaker B.V. All rights reserved.

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
#include <avr/wdt.h>
#include "watchdog.h"

//===========================================================================
//=============================private variables ============================
//===========================================================================

//===========================================================================
//=============================functions         ============================
//===========================================================================

/// initialise watch dog with a 4 sec interrupt time
void watchdog_init()
{
    wdt_enable(WDTO_4S);
}

/// reset watchdog. MUST be called every 1s after init or avr will reset.
void watchdog_reset()
{
    wdt_reset();
}
