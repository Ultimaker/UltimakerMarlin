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
#ifndef DEBUG_LEDS_H
#define DEBUG_LEDS_H

// Define simple macro's to use them (define __USE_DEBUG_LEDS__ before including file)
#if defined(__USE_DEBUG_LEDS__)

#define INITIALIZE_DEBUG_LEDS do { SET_OUTPUT(DEBUG_LED0); SET_OUTPUT(DEBUG_LED1); SET_OUTPUT(DEBUG_LED2); } while(0)
#define SWITCH_ON_DEBUG_LED(led) do { WRITE(led, HIGH); } while(0)
#define SWITCH_OFF_DEBUG_LED(led) do { WRITE(led, LOW); } while(0)

#else

#define INITIALIZE_DEBUG_LEDS do {} while(0)
#define SWITCH_ON_DEBUG_LED(led) do {} while(0)
#define SWITCH_OFF_DEBUG_LED(led) do {} while(0)

#endif

#endif//DEBUG_LEDS_H
