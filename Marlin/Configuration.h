#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configuration file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 2

// This determines the communication speed of the printer
#define BAUDRATE 250000

// This defines the number of extruders
#define EXTRUDERS 1

// This defines the number of flow sensors
#define NR_OF_FLOW_SENSORS          2

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//// Temperature sensor settings:
// 0 is not used
// 21 is ADS101X with Ultiboard v2.x

#define TEMP_SENSOR_0 21
#define TEMP_SENSOR_1 21
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 21

//Check if the heater heats up MAX_HEATING_TEMPERATURE_INCREASE within MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
// If not, raise an error because most likely the heater is not heating up the temperature sensor. Indicating an issue in the system.
#define MAX_HEATING_TEMPERATURE_INCREASE 1
#define MAX_HEATING_CHECK_MILLIS (30 * 1000)

// PID settings:
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current

//#define PID_DEBUG // Sends debug data to the serial port.
#define DEFAULT_PID_FUNCTIONAL_RANGE 25 // If the temperature difference between the target temperature and the actual temperature
                              // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

#define PID_D_LPF_RC_CONSTANT 0.5f // Low pass filter rc constant for the D term of the temperature PID controllers

#define PID_MINIMUM_SAMPLING_INTERVAL 0.5f //Temperature PID controller sampling interval in seconds

//Let the PID controller ignore consecutive temperature samples that are too far apart.
//This prevents single bad samples from screwing up the PID controller logic.
#define PID_IGNORE_TEMPERATURE_DELTA 10.0

//The 0 point for the feed forward factor of the PID controller.
#define FEED_FORWARD_MINIMAL_TEMPERATURE 35

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170
#define EXTRUDE_MAXLENGTH 1000.0 //prevent extrusion of very large distances.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // Define endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined.
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

#define DEFAULT_INVERT_X_DIR false    // for Ultimaker E2 set to false
#define DEFAULT_INVERT_Y_DIR true     // for Ultimaker E2 set to true
#define DEFAULT_INVERT_Z_DIR false    // for Ultimaker E2 set to false
#define DEFAULT_INVERT_E0_DIR true    // for UM2/XL set to false, for UM2+/3 to true
#define DEFAULT_INVERT_E1_DIR false   // for UM2/XL set to true,  for UM2+/3 set to false
#define DEFAULT_INVERT_E2_DIR false   // obsolete

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// Travel limits after homing
#define X_MAX_POS 223
#define X_MIN_POS 0
#define Y_MAX_POS 220
#define Y_MIN_POS 0
#define Z_MAX_POS 207
#define Z_MIN_POS 0

//// MOVEMENT SETTINGS
#define HOMING_FEEDRATE {100*60, 100*60, 40*60, 0}  // set the homing speeds (mm/min)

// default settings

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0, 80.0, 400, 369}    // default steps per unit for Ultimaker E2
#define DEFAULT_MAX_FEEDRATE          {300, 300, 40, 45}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,10000} // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000      // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000      // X, Y, Z and E max acceleration in mm/s^2 for retracts

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0      // (mm/sec)
#define DEFAULT_ZJERK                 0.4       // (mm/sec)
#define DEFAULT_EJERK                 5.0       // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

#define ENABLE_WATCHDOG_SERIAL_INPUT // Implement monitoring getting commands to be executed
#ifdef ENABLE_WATCHDOG_SERIAL_INPUT
  #define MONITOR_SERIAL_INPUT_TIMEOUT 5 * 60       // Timeout for 5 minutes: after this time of no communication received, marlin will stop the heating of hotends and bed.
  #define MONITOR_MINIMUM_BED_TEMPERATURE    50     // Minimum temperature threshold for marking the bed as hot
  #define MONITOR_MINIMUM_HOTEND_TEMPERATURE 50     // Minimum temperature threshold for marking the hotend as hot
#endif//ENABLE_WATCHDOG_SERIAL_INPUT

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif // CONFIGURATION_H
