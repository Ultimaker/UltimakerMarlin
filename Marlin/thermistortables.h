#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#include "Marlin.h"

#if (TEMP_SENSOR_0 == 21) || (TEMP_SENSOR_1 == 21) || (TEMP_SENSOR_2 == 21) || (TEMP_SENSOR_BED == 21)
// PT100 with ADS1015 amp on Ultimaker v2.x
const int16_t temptable_21[][2] PROGMEM = {
{   -2181, -50},
{   -1809,   0},
{   -1454,  50},
{   -1113, 100},
{    -787, 150},
{    -475, 200},
{    -175, 250},
{     112, 300},
{     388, 350},
{     652, 400},
{     906, 450},
{    1150, 500},
{    2036, 700}
};
#define TEMPTABLE_21_LEN    (sizeof(temptable_21)/sizeof(*temptable_21))
#endif


#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
# define HEATER_0_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
# define HEATER_0_TEMPTABLE_LEN (sizeof(HEATER_0_TEMPTABLE)/sizeof(*HEATER_0_TEMPTABLE))
#else
#  define HEATER_0_TEMPTABLE NULL
#  define HEATER_0_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORHEATER_1
# define HEATER_1_TEMPTABLE TT_NAME(THERMISTORHEATER_1)
# define HEATER_1_TEMPTABLE_LEN (sizeof(HEATER_1_TEMPTABLE)/sizeof(*HEATER_1_TEMPTABLE))
#else
#  define HEATER_1_TEMPTABLE NULL
#  define HEATER_1_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORHEATER_2
# define HEATER_2_TEMPTABLE TT_NAME(THERMISTORHEATER_2)
# define HEATER_2_TEMPTABLE_LEN (sizeof(HEATER_2_TEMPTABLE)/sizeof(*HEATER_2_TEMPTABLE))
#else
#  define HEATER_2_TEMPTABLE NULL
#  define HEATER_2_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORBED
# define BEDTEMPTABLE TT_NAME(THERMISTORBED)
# define BEDTEMPTABLE_LEN (sizeof(BEDTEMPTABLE)/sizeof(*BEDTEMPTABLE))
#endif

#endif //THERMISTORTABLES_H_
