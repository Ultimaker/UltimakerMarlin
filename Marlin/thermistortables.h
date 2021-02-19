#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#include "Marlin.h"

#define OVERSAMPLENR 8

#if (THERMISTORHEATER_0 == 20) || (THERMISTORHEATER_1 == 20) || (THERMISTORHEATER_2 == 20) || (THERMISTORBED == 20) // PT100 with INA826 amp on Ultimaker v2.0 electronics
const int16_t temptable_20[][2] PROGMEM = {
{         0*OVERSAMPLENR ,       0     },
{       227*OVERSAMPLENR ,       1     },
{       245*OVERSAMPLENR ,       20     },
{       262*OVERSAMPLENR ,       40     },
{       279*OVERSAMPLENR ,       60     },
{       295*OVERSAMPLENR ,       80     },
{       312*OVERSAMPLENR ,       100     },
{       329*OVERSAMPLENR ,       120     },
{       345*OVERSAMPLENR ,       140     },
{       361*OVERSAMPLENR ,       160     },
{       377*OVERSAMPLENR ,       180     },
{       393*OVERSAMPLENR ,       200     },
{       409*OVERSAMPLENR ,       220     },
{       424*OVERSAMPLENR ,       240     },
{       440*OVERSAMPLENR ,       260     },
{       455*OVERSAMPLENR ,       280     },
{       470*OVERSAMPLENR ,       300     },
{       485*OVERSAMPLENR ,       320     },
{       500*OVERSAMPLENR ,       340     },
{       515*OVERSAMPLENR ,       360     },
{       529*OVERSAMPLENR ,       380     },
{       544*OVERSAMPLENR ,       400     },
{       614*OVERSAMPLENR ,       500     },
{       681*OVERSAMPLENR ,       600     },
{       744*OVERSAMPLENR ,       700     },
{       805*OVERSAMPLENR ,       800     },
{       862*OVERSAMPLENR ,       900     },
{       917*OVERSAMPLENR ,       1000     },
{       968*OVERSAMPLENR ,       1100     }
};
#define TEMPTABLE_20_LEN    (sizeof(temptable_20)/sizeof(*temptable_20))
#endif

#if (THERMISTORHEATER_0 == 21) || (THERMISTORHEATER_1 == 21) || (THERMISTORHEATER_2 == 21) || (THERMISTORBED == 21)
// PT100 with ADS1015 amp on Ultimaker v2.x
const int16_t temptable_21[][2] PROGMEM = {
{   -2181*OVERSAMPLENR, -50},
{   -1809*OVERSAMPLENR,   0},
{   -1454*OVERSAMPLENR,  50},
{   -1113*OVERSAMPLENR, 100},
{    -787*OVERSAMPLENR, 150},
{    -475*OVERSAMPLENR, 200},
{    -175*OVERSAMPLENR, 250},
{     112*OVERSAMPLENR, 300},
{     388*OVERSAMPLENR, 350},
{     652*OVERSAMPLENR, 400},
{     906*OVERSAMPLENR, 450},
{    1150*OVERSAMPLENR, 500},
{    2036*OVERSAMPLENR, 700}
};
#define TEMPTABLE_21_LEN    (sizeof(temptable_21)/sizeof(*temptable_21))
#endif


#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
# define HEATER_0_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
# define HEATER_0_TEMPTABLE_LEN (sizeof(HEATER_0_TEMPTABLE)/sizeof(*HEATER_0_TEMPTABLE))
#else
# ifdef HEATER_0_USES_THERMISTOR
#  error No heater 0 thermistor table specified
# else  // HEATER_0_USES_THERMISTOR
#  define HEATER_0_TEMPTABLE NULL
#  define HEATER_0_TEMPTABLE_LEN 0
# endif // HEATER_0_USES_THERMISTOR
#endif

#ifdef THERMISTORHEATER_1
# define HEATER_1_TEMPTABLE TT_NAME(THERMISTORHEATER_1)
# define HEATER_1_TEMPTABLE_LEN (sizeof(HEATER_1_TEMPTABLE)/sizeof(*HEATER_1_TEMPTABLE))
#else
# ifdef HEATER_1_USES_THERMISTOR
#  error No heater 1 thermistor table specified
# else  // HEATER_1_USES_THERMISTOR
#  define HEATER_1_TEMPTABLE NULL
#  define HEATER_1_TEMPTABLE_LEN 0
# endif // HEATER_1_USES_THERMISTOR
#endif

#ifdef THERMISTORHEATER_2
# define HEATER_2_TEMPTABLE TT_NAME(THERMISTORHEATER_2)
# define HEATER_2_TEMPTABLE_LEN (sizeof(HEATER_2_TEMPTABLE)/sizeof(*HEATER_2_TEMPTABLE))
#else
# ifdef HEATER_2_USES_THERMISTOR
#  error No heater 2 thermistor table specified
# else  // HEATER_2_USES_THERMISTOR
#  define HEATER_2_TEMPTABLE NULL
#  define HEATER_2_TEMPTABLE_LEN 0
# endif // HEATER_2_USES_THERMISTOR
#endif

#ifdef THERMISTORBED
# define BEDTEMPTABLE TT_NAME(THERMISTORBED)
# define BEDTEMPTABLE_LEN (sizeof(BEDTEMPTABLE)/sizeof(*BEDTEMPTABLE))
#else
# ifdef BED_USES_THERMISTOR
#  error No bed thermistor table specified
# endif // BED_USES_THERMISTOR
#endif

#endif //THERMISTORTABLES_H_
