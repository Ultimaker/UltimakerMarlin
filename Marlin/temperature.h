/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "Marlin.h"
#include "planner.h"
#include "temperaturePID.h"
#include "PowerBudgetManagement.h"

// 60 degC is hot but usually not too hot to handle and it is also not an expected ambient temperature.
#define HOTEND_HUMAN_TOUCHABLE_TEMPERATURE 60

// public functions
void temperatureInit(); //initialise the heating
void manage_heater(); //it is critical that this is called periodically.

extern TemperaturePID hotend_pid[EXTRUDERS];
extern TemperaturePID heated_bed_pid;
extern PowerBudgetManagement pwr;

// low level conversion routines
// do not use these routines and variables outside of temperature.cpp
extern float current_temperature[EXTRUDERS];
extern float current_temperature_bed;
extern uint16_t heater_output_accumulator[EXTRUDERS];
extern uint16_t heater_output_accumulator_counter[EXTRUDERS];
extern uint16_t bed_heater_output_accumulator;
extern uint16_t bed_heater_output_accumulator_counter;
extern bool stop_heaters_pwm;

//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius

FORCE_INLINE float degHotend(uint8_t extruder) {
  return current_temperature[extruder];
};

FORCE_INLINE float degBed() {
  return current_temperature_bed;
};

float degTopcap();

void disable_all_heaters();

#endif  // TEMPERATURE_H
