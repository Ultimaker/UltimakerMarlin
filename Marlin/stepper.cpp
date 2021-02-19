/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "language.h"
#include "speed_lookuptable.h"


//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced
bool invert_direction[NUM_AXIS+EXTRUDERS-1] = {DEFAULT_INVERT_X_DIR, DEFAULT_INVERT_Y_DIR, DEFAULT_INVERT_Z_DIR, DEFAULT_INVERT_E0_DIR};

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y,
            counter_z,
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for decelaration start point
static uint8_t step_loops;
static uint8_t step_loops_nominal;
static unsigned short OCR1A_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;

#if defined(X_MIN_PIN) && X_MIN_PIN > -1
static bool old_x_min_endstop=false;
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1
static bool old_x_max_endstop=false;
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
static bool old_y_min_endstop=false;
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
static bool old_y_max_endstop=false;
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
static bool old_z_min_endstop=false;
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
static bool old_z_max_endstop=false;
#endif

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define A(CODE) " " CODE "\n"

#ifdef __AVR
// intRes = intIn1 * intIn2 >> 16
// uses:
// tmp store 0
// intRes to store the byte 1 of the 24 bit result
static FORCE_INLINE uint16_t MultiU16X8toH16(uint8_t charIn1, uint16_t intIn2) {
  register uint8_t tmp;
  register uint16_t intRes;
  __asm__ __volatile__ (
    A("clr %[tmp]")
    A("mul %[charIn1], %B[intIn2]")
    A("movw %A[intRes], r0")
    A("mul %[charIn1], %A[intIn2]")
    A("add %A[intRes], r1")
    A("adc %B[intRes], %[tmp]")
    A("lsr r0")
    A("adc %A[intRes], %[tmp]")
    A("adc %B[intRes], %[tmp]")
    A("clr r1")
      : [intRes] "=&r" (intRes),
        [tmp] "=&r" (tmp)
      : [charIn1] "d" (charIn1),
        [intIn2] "d" (intIn2)
      : "cc"
  );
  return intRes;
}

// intRes = longIn1 * longIn2 >> 24
// uses:
// A[tmp] to store 0
// B[tmp] to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
static FORCE_INLINE uint16_t MultiU24X32toH16(uint32_t longIn1, uint32_t longIn2) {
  register uint8_t tmp1;
  register uint8_t tmp2;
  register uint16_t intRes;
  __asm__ __volatile__( \
    A("clr %[tmp1]")
    A("mul %A[longIn1], %B[longIn2]")
    A("mov %[tmp2], r1")
    A("mul %B[longIn1], %C[longIn2]")
    A("movw %A[intRes], r0")
    A("mul %C[longIn1], %C[longIn2]")
    A("add %B[intRes], r0")
    A("mul %C[longIn1], %B[longIn2]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %A[longIn1], %C[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %B[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %C[longIn1], %A[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %A[longIn2]")
    A("add %[tmp2], r1")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("lsr %[tmp2]")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("mul %D[longIn2], %A[longIn1]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %D[longIn2], %B[longIn1]")
    A("add %B[intRes], r0")
    A("clr r1")
      : [intRes] "=&r" (intRes),
        [tmp1] "=&r" (tmp1),
        [tmp2] "=&r" (tmp2)
      : [longIn1] "d" (longIn1),
        [longIn2] "d" (longIn2)
      : "cc"
  );
  return intRes;
}
#else

// intRes = intIn1 * intIn2 >> 16
#define MultiU16X8toH16(intRes, charIn1, intIn2) do { (intRes) = (uint32_t(charIn1) * uint32_t(intIn2)) >> 16; } while(0)

// intRes = longIn1 * longIn2 >> 24
#define MultiU24X32toH16(intRes, longIn1, longIn2) do { (intRes) = (uint64_t(longIn1) * uint64_t(longIn2)) >> 24; } while(0)
#endif

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)


void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   SERIAL_ECHOLN("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
 }
}

bool isEndstopHit()
{
    return endstop_x_hit || endstop_y_hit || endstop_z_hit;
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
  step_rate -= (F_CPU/500000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate
    const uint8_t* table_address = (const uint8_t*)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    uint8_t tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    timer = MultiU16X8toH16(tmp_step_rate, gain);
    timer = (uint16_t)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    const uint8_t* table_address = (const uint8_t*)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (uint16_t)pgm_read_word_near(table_address);
    timer -= (((uint16_t)pgm_read_word_near(table_address+2) * (uint8_t)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; SERIAL_ECHO_START; SERIAL_ECHOPGM(MSG_STEPPER_TOO_HIGH); MSerial.println(step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
// This function is timing critical, adding too much code/time/instructions in here will cause broken behaviour at high movement speeds.
// Do not touch this function unless you really know what you are doing.
ISR(TIMER1_COMPA_vect)
{
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;
    }
    else {
        OCR1A=2000; // 1kHz.
    }
  }

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;


    // Set the direction bits
    if((out_bits & (1<<X_AXIS))!=0){
      WRITE(X_DIR_PIN, invert_direction[X_AXIS]);
      count_direction[X_AXIS]=-1;
    }
    else{
      WRITE(X_DIR_PIN, !invert_direction[X_AXIS]);
      count_direction[X_AXIS]=1;
    }
    if((out_bits & (1<<Y_AXIS))!=0){
      WRITE(Y_DIR_PIN, invert_direction[Y_AXIS]);
      count_direction[Y_AXIS]=-1;
    }
    else{
      WRITE(Y_DIR_PIN, !invert_direction[Y_AXIS]);
      count_direction[Y_AXIS]=1;
    }

    // Set direction en check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
      if (check_endstops)
      {
        #if defined(X_MIN_PIN) && X_MIN_PIN > -1
          bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_min_endstop = x_min_endstop;
        #endif
      }
    }
    else { // +direction
      if(check_endstops)
      {
        #if defined(X_MAX_PIN) && X_MAX_PIN > -1
          bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_max_endstop = x_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      if(check_endstops)
      {
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      if(check_endstops)
      {
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      WRITE(Z_DIR_PIN,invert_direction[Z_AXIS]);

      count_direction[Z_AXIS]=-1;
      if(check_endstops)
      {
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction
      WRITE(Z_DIR_PIN,!invert_direction[Z_AXIS]);

      count_direction[Z_AXIS]=1;
      if(check_endstops)
      {
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }

      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }



    for (uint8_t i = step_loops; i > 0; i--) { // Take multiple steps per interrupt (For high speed moves)
        MSerial.checkRx(); // Check for serial chars.

        counter_x += current_block->steps_x;
        if (counter_x > 0) {
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          counter_x -= current_block->step_event_count;
          count_position[X_AXIS]+=count_direction[X_AXIS];
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        }

        counter_y += current_block->steps_y;
        if (counter_y > 0) {
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count;
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);

        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
      }

      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        WRITE_E_STEP(!INVERT_E_STEP_PIN);
        counter_e -= current_block->step_event_count;
        count_position[E_AXIS]+=count_direction[E_AXIS];
        WRITE_E_STEP(INVERT_E_STEP_PIN);
      }

      step_events_completed += 1;
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculate new timer value
    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {
      acc_step_rate = MultiU24X32toH16(acceleration_time, current_block->acceleration_rate) + current_block->initial_rate;

      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      uint16_t timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
    }
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
      uint16_t step_rate = MultiU24X32toH16(deceleration_time, current_block->acceleration_rate);

      if (step_rate < acc_step_rate) { // Still decelerating?
        step_rate = max(acc_step_rate - step_rate, current_block->final_rate);
      }
      else {
        step_rate = current_block->final_rate;  // lower limit
      }

      // step_rate to timer interval
      uint16_t timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
    }
    else {
      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // Hack to address stuttering caused by ISR not finishing in time.
    // When the ISR does not finish in time, the timer will wrap in the computation of the next interrupt time.
    // This hack replaces the correct (past) time with a time not far in the future.
    // (Note that OCR1A and TCNT1 are registers, so using the max() macro or std::max() can cause problems, especially when compiling the simulator)
    if (OCR1A < TCNT1 + 16)
        OCR1A = TCNT1 + 16;

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}

void st_init()
{
  //Initialize Dir Pins
  #if defined(X_DIR_PIN) && X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if defined(Y_DIR_PIN) && Y_DIR_PIN > -1
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if defined(Z_DIR_PIN) && Z_DIR_PIN > -1
    SET_OUTPUT(Z_DIR_PIN);
  #endif
  #if defined(E0_DIR_PIN) && E0_DIR_PIN > -1
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if defined(E1_DIR_PIN) && (E1_DIR_PIN > -1)
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if defined(E2_DIR_PIN) && (E2_DIR_PIN > -1)
    SET_OUTPUT(E2_DIR_PIN);
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  #endif
  #if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
    SET_OUTPUT(E0_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
  #endif
  #if defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
    SET_OUTPUT(E1_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
  #endif
  #if defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
    SET_OUTPUT(E2_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
  #endif

  //endstops and pullups

  #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

  //Initialize Step Pins
  #if defined(X_STEP_PIN) && (X_STEP_PIN > -1)
    SET_OUTPUT(X_STEP_PIN);
    WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
    disable_x();
  #endif
  #if defined(Y_STEP_PIN) && (Y_STEP_PIN > -1)
    SET_OUTPUT(Y_STEP_PIN);
    WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y();
  #endif
  #if defined(Z_STEP_PIN) && (Z_STEP_PIN > -1)
    SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    disable_z();
  #endif
  #if defined(E0_STEP_PIN) && (E0_STEP_PIN > -1)
    SET_OUTPUT(E0_STEP_PIN);
    WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e0();
  #endif
  #if defined(E1_STEP_PIN) && (E1_STEP_PIN > -1)
    SET_OUTPUT(E1_STEP_PIN);
    WRITE(E1_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e1();
  #endif
  #if defined(E2_STEP_PIN) && (E2_STEP_PIN > -1)
    SET_OUTPUT(E2_STEP_PIN);
    WRITE(E2_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e2();
  #endif

  enable_endstops(true); // Start with endstops active. After homing they can be disabled
}


// Block until all buffered steps are executed
void st_synchronize()
{
    while( blocks_queued())
    {
        manage_heater();
        manage_inactivity();
    }
}

/**
 * Set the stepper positions directly in steps
 */
void st_set_position(const long &x, const long &y, const long &z, const long &e)
{
    // Wait until all buffered moves are done so the following will not interfere with moves executed at this moment
    st_synchronize();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count_position[X_AXIS] = x;
        count_position[Y_AXIS] = y;
        count_position[Z_AXIS] = z;
        count_position[E_AXIS] = e;
    }
}

void st_set_e_position(const long &e)
{
    // Wait until all buffered moves are done so the following will not interfere with moves executed at this moment
    st_synchronize();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count_position[E_AXIS] = e;
    }
}

/**
 * Get a stepper's position in steps.
 */
long st_get_position(uint8_t axis)
{
    long count_pos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count_pos = count_position[axis];
    }
    return count_pos;
}

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
}

void disable_all_steppers()
{
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #if EXTRUDERS == 1
  current_position[X_AXIS] = float(st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS];
  current_position[Y_AXIS] = float(st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS];
  current_position[Z_AXIS] = float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS];
  current_position[E_AXIS] = float(st_get_position(E_AXIS)) / axis_steps_per_unit[E_AXIS];
  #else
  // The + extruder offset is necessary to prevent excessive location drift while in a offsetted situation/aka while printing.
  // Though perhaps this code should not be called during printing at all.
  current_position[X_AXIS] = float(st_get_position(X_AXIS) + extruder_offset[X_AXIS][active_extruder]) / axis_steps_per_unit[X_AXIS];
  current_position[Y_AXIS] = float(st_get_position(Y_AXIS) + extruder_offset[Y_AXIS][active_extruder]) / axis_steps_per_unit[Y_AXIS];
  current_position[Z_AXIS] = float(st_get_position(Z_AXIS) + extruder_offset[Z_AXIS][active_extruder]) / axis_steps_per_unit[Z_AXIS];
  current_position[E_AXIS] = float(st_get_position(E_AXIS)) / axis_steps_per_unit[E_AXIS];
  #endif
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

// Enable the handling of the interrupts
void st_enable_interrupt()
{
    // waveform generation = 0100 = CTC
    TCCR1B &= ~(1<<WGM13);
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~(1<<WGM11);
    TCCR1A &= ~(1<<WGM10);

    // output mode = 00 (disconnected)
    TCCR1A &= ~(3<<COM1A0);
    TCCR1A &= ~(3<<COM1B0);

    // Set the timer pre-scaler
    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU. If you are going to change this, be
    // sure to regenerate speed_lookuptable.h with
    // create_speed_lookuptable.py
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

    // Init Stepper ISR to 122 Hz for quick starting
    OCR1A = 0x4000;
    TCNT1 = 0;
    ENABLE_STEPPER_DRIVER_INTERRUPT();

    sei();
}
