#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value.
#define FAN_KICKSTART_TIME 100

//Any G0/G1 moves received are checked if they are within the normal build volume.
//Because of definition problems that require a whole lot more fixes we have an "allow error" outside of our normal build volume.
//This is okish, as we are trying to catch the case where GCode is corrupted and very long moves are requested.
#define ALLOWED_POSITION_ERROR 50

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// Define the temperature for cooling the extruder's cold zone.
// Don't set below 40C, because fan turns off with a hysteresis of 5C and
// we have to function at ambient temperatures of 35C.
#define EXTRUDER_COLD_ZONE_FAN_TEMPERATURE 50

// Temperature for switching the TopCap fan. This defines the temperature at which filament starts emitting particles.
// Don't set below 40C, because fan turns off with a hysteresis of 5C and
// we have to function at ambient temperatures of 35C.
#define TOPCAP_FAN_TEMPERATURE 50

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_RETRACT_MM 7
#define Y_HOME_RETRACT_MM 7
#define Z_HOME_RETRACT_MM 7
#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN
// Part of SLOWDOWN: minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

// Default motor current for XY,Z,E in mA
#define DEFAULT_PWM_MOTOR_CURRENT {1300, 1300, 1250}

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

//everything with less than this number of steps will be ignored as move and joined with the next movement
#define MINIMAL_STEPS_FOR_PLANNED_MOVE 5

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#define BLOCK_BUFFER_SIZE 16


// The ASCII buffer for receiving from the serial:
// Length of 96 is kind of random, we've seen a payload of 63 (!)
// The current size including overhead for packet control data, would only allow for 2 messages in the receive buffer,
// but since a payload nearly never is maximum size and the fact that the ringbuffer is being read (emptied) all the time
// it's deemed unlikely a buffer overrun will occur.
// If it would occur, then the protocol is robust enough to recover itself from this event.
#define MAX_CMD_SIZE    96
#define BUFSIZE 8


//===========================================================================
//=============================  Define Defines  ============================
//===========================================================================
#if TEMP_SENSOR_0 > 0
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
#endif
#if TEMP_SENSOR_1 > 0
  #define THERMISTORHEATER_1 TEMP_SENSOR_1
#endif
#if TEMP_SENSOR_2 > 0
  #define THERMISTORHEATER_2 TEMP_SENSOR_2
#endif
#if TEMP_SENSOR_BED > 0
  #define THERMISTORBED TEMP_SENSOR_BED
#endif

#if TEMP_SENSOR_0 == 0
  #undef HEATER_0_MINTEMP
  #undef HEATER_0_MAXTEMP
#endif
#if TEMP_SENSOR_1 == 0
  #undef HEATER_1_MINTEMP
  #undef HEATER_1_MAXTEMP
#endif
#if TEMP_SENSOR_2 == 0
  #undef HEATER_2_MINTEMP
  #undef HEATER_2_MAXTEMP
#endif
#if TEMP_SENSOR_BED == 0
  #undef BED_MINTEMP
  #undef BED_MAXTEMP
#endif


#endif //__CONFIGURATION_ADV_H
