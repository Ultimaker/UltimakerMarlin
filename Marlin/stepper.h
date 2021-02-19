/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef STEPPER_H
#define STEPPER_H

#include "planner.h"

/** Usage of these macros is to optimize the code to as minimal amount of instructions as required. Do not touch unless you know what you are doing */
#if EXTRUDERS > 2
  #define WRITE_E_STEP(v) { if(current_block->active_extruder == 2) { WRITE(E2_STEP_PIN, v); } else { if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}}
  #define NORM_E_DIR() { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, !invert_direction[E_AXIS+2]); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !invert_direction[E_AXIS+1]); } else { WRITE(E0_DIR_PIN, !invert_direction[E_AXIS+0]); }}}
  #define REV_E_DIR() { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, invert_direction[E_AXIS+2]); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, invert_direction[E_AXIS+1]); } else { WRITE(E0_DIR_PIN, invert_direction[E_AXIS+0]); }}}
#elif EXTRUDERS > 1
  #define WRITE_E_STEP(v) { if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}
  #define NORM_E_DIR() { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !invert_direction[E_AXIS+1]); } else { WRITE(E0_DIR_PIN, !invert_direction[E_AXIS+0]); }}
  #define REV_E_DIR() { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, invert_direction[E_AXIS+1]); } else { WRITE(E0_DIR_PIN, invert_direction[E_AXIS+0]); }}
#else
  #define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
  #define NORM_E_DIR() WRITE(E0_DIR_PIN, !invert_direction[E_AXIS+0])
  #define REV_E_DIR() WRITE(E0_DIR_PIN, invert_direction[E_AXIS+0])
#endif

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)

extern bool invert_direction[NUM_AXIS+EXTRUDERS-1];

// Initialize and start the stepper motor subsystem
void st_init();

// Enable the handling of the interrupts
void st_enable_interrupt();

// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long &x, const long &y, const long &z, const long &e);
void st_set_e_position(const long &e);

// Get current position in steps
long st_get_position(uint8_t axis);

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();


void checkHitEndstops();        // Call from somewhere to create a serial error message with the locations where the endstops were hit, in case they were triggered
void endstops_hit_on_purpose(); // Avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops().

void enable_endstops(bool check); // Enable/disable endstop checking

void checkStepperErrors(); //Print errors detected by the stepper
bool isEndstopHit();

void finishAndDisableSteppers();

void disable_all_steppers(); // Disable all stepper motor axes, do not block until the marlin buffer is empty

extern block_t *current_block;  // A pointer to the block currently being traced

void quickStop();

#endif  // STEPPER_H
