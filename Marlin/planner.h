/*
  planner.h - buffers movement commands and manages the acceleration profile plan
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

// This module is to be considered a sub-module of stepper.c. Please don't include
// this file from any other module.

#ifndef planner_h
#define planner_h

#include <limits.h>

#include "Marlin.h"

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  int32_t steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  uint32_t step_event_count;           // The number of step events required to complete this block
  int32_t accelerate_until;                    // The index of the step event on which to stop acceleration
  int32_t decelerate_after;                    // The index of the step event on which to start decelerating
  int32_t acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint32_t nominal_rate;                        // The nominal step rate for this block in step_events/sec
  uint32_t initial_rate;                        // The jerk-adjusted step rate at start of block
  uint32_t final_rate;                          // The minimal rate at exit
  uint32_t acceleration_st;                     // acceleration steps/sec^2
  uint32_t fan_speed;
  volatile char busy;
} block_t;

// Initialize the motion plan subsystem
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in
// millimeters. Feed rate specifies the speed of the motion.
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder);

// Set position. Used for G92 instructions.
void plan_set_position(const float &x, const float &y, const float &z, const float &e);
void plan_set_e_position(const float &e);



void check_axes_activity();
uint8_t plan_buf_free_positions();  // return the number of free positions in the planner buffer.

extern uint32_t minsegmenttime;
extern float max_feedrate[NUM_AXIS]; // set the max speeds
extern float axis_steps_per_unit[NUM_AXIS];
extern uint32_t max_acceleration_units_per_sq_second[4]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern float max_xy_jerk;          //speed that can be stopped at once, if I understand correctly.
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern uint32_t axis_steps_per_sqr_second[NUM_AXIS];

/* Motion planner buffer statistics */
extern uint8_t  planner_min;
extern uint8_t  planner_motions;
extern uint8_t  buffer_underruns;


extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail;
// Called when the current block is no longer needed. Discards the block and makes the memory
// available for new blocks.
FORCE_INLINE void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) {
    uint8_t  used;

    block_buffer_tail = (block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);

    used = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
    if (!used && buffer_underruns < UCHAR_MAX)
      buffer_underruns++;
    if (used < planner_min)
      planner_min = used;
    if (planner_motions < UCHAR_MAX)
      planner_motions++;
  }
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) {
    return(NULL);
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

// Returns true when blocks are queued, false otherwise.
FORCE_INLINE bool blocks_queued()
{
  if (block_buffer_head == block_buffer_tail) {
    return false;
  }
  else
    return true;
}

#ifdef PREVENT_DANGEROUS_EXTRUDE
void set_extrude_min_temp(float temp);
#endif

void reset_acceleration_rates();
#endif
