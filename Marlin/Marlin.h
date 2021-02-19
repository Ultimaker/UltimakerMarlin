// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
#include "language.h"

#define  HardwareSerial_h // trick to disable the standard HWserial

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselves.
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#define SERIAL_ECHO(x) MSerial.print(x);
#define SERIAL_ECHO_F(x,y) MSerial.print(x,y);
#define SERIAL_ECHOPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_ECHOLN(x) do {MSerial.print(x);MSerial.println();} while(0)
#define SERIAL_ECHOLNPGM(x) do{serialprintPGM(PSTR(x));MSerial.println();} while(0)

#define SERIAL_ECHO_OK(sequence_number) do { SerialProtocol::sendAck(sequence_number); } while (0)

const char errormagic[] PROGMEM ="\nError:";
const char logmagic[] PROGMEM ="LOG:";  // TODO: EM-2759 [New] - should add a \n before LOG but this may break even more
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ECHO_START serialprintPGM(logmagic);

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))
void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, uint32_t v);

//Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MSerial.write(ch);
    ch=pgm_read_byte(++str);
  }
}

void manage_inactivity();

#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
  #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif

// The axis order in all axis related arrays is X, Y, Z, E
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3, NUM_AXIS=4};

#if (EXTRUDERS > 1)
#define NUM_MOTOR_DRIVERS   (NUM_AXIS + 1)
#else
#define NUM_MOTOR_DRIVERS   NUM_AXIS
#endif

#include "Stop.h"

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int16_t val);
#endif


extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int16_t feedmultiply;
extern int16_t extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float add_homing[3];
extern float min_pos[3];
extern float max_pos[3];
extern uint8_t target_fan_speed;

extern uint32_t starttime;
extern uint32_t stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;
// handle extruder offset in steps to avoid floating point errors accumulating
// (and move handling to the planner)
extern int32_t extruder_offset[NUM_AXIS-1][EXTRUDERS];

#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif
# define ARRAY_BY_EXTRUDERS_INIT(v) ARRAY_BY_EXTRUDERS(v, v, v)

#endif
