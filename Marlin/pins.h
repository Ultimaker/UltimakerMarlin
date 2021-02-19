#ifndef PINS_H
#define PINS_H

/******************************************************************
 * Ultiboard pin assignment                                       *
 ******************************************************************/

#ifndef __AVR_ATmega2560__
 #error Only the Arduino Mega 2560 MCU is supported
#endif

// Version/revision
#define BOARD_REV_1_PIN             D6  // PD6, Board revision bit 0
#define BOARD_REV_2_PIN             38  // PD7, Board revision bit 1
#define BOARD_REV_3_PIN             J7  // PJ7, Board revision bit 2

// Steppers
#define X_STEP_PIN                  25  // PA3, X-axis stepper driver step output
#define X_DIR_PIN                   23  // PA1, X-axis stepper driver direction output
#define X_ENABLE_PIN                27  // PA5, X-axis stepper driver enable output
#define X_DIAG_PIN                  10  // PB4, X-axis stepper driver interrupt input
#define X_SPI_CS_PIN                F0  // PF0, X-axis stepper driver chip select output
#define X_STOP_PIN                  22  // PA0, X-axis limit switch input

#define Y_STEP_PIN                  32  // PC5, Y-axis stepper driver step output
#define Y_DIR_PIN                   33  // PC4, Y-axis stepper driver direction output
#define Y_ENABLE_PIN                31  // PC6, Y-axis stepper driver enable output
#define Y_DIAG_PIN                  11  // PB5, Y-axis stepper driver interrupt input
#define Y_SPI_CS_PIN                F1  // PF1, Y-axis stepper driver chip select output
#define Y_STOP_PIN                  26  // PA4, Y-axis limit switch input

#define Z_STEP_PIN                  35  // PC2, Z-axis stepper driver step output
#define Z_DIR_PIN                   36  // PC1, Z-axis stepper driver direction output
#define Z_ENABLE_PIN                34  // PC3, Z-axis stepper driver enable output
#define Z_DIAG_PIN                  12  // PB6, Z-axis stepper driver interrupt input
#define Z_SPI_CS_PIN                F2  // PF2, Z-axis stepper driver chip select output
#define Z_STOP_PIN                  29  // PA7, Z-axis limit switch input

#define E0_STEP_PIN                 42  // PL6, Extruder 0 stepper driver step output
#define E0_DIR_PIN                  43  // PL7, Extruder 0 stepper driver direction output
#define E0_ENABLE_PIN               37  // PC0, Extruder 0 stepper driver enable output
#define E0_DIAG_PIN                 J3  // PJ3, Extruder 0 stepper driver interrupt input
#define E0_SPI_CS_PIN               F3  // PF3, Extruder 0 stepper driver chip select output

#define E1_STEP_PIN                 -1  // PL0, Extruder 1 stepper driver step output
#define E1_DIR_PIN                  -1  // PL2, Extruder 1 stepper driver direction output
#define E1_ENABLE_PIN               -1  // PC1, Extruder 1 stepper driver enable output
#define E1_DIAG_PIN                 -1  // PJ4, Extruder 1 stepper driver interrupt input
#define E1_SPI_CS_PIN               -1  // PF4, Extruder 1 stepper driver chip select output

// Heaters
#define HEATER_BED_PIN               4  // PG5, Bed heater output with fast PWM.
#define HEATER_BED_XL_PIN            9  // PH6, Bed heater output for the S5-board which has maximum PWM of 10Hz
#define HEATER_0_PIN                 2  // PE4, Extruder 0 heater
#define HEATER_1_PIN                 3  // PE5, Extruder 1 heater
#define HEATER_2_PIN                -1  // xxx, Extruder 2 heater

#define LED_PIN                      8  // PH5, The cabinet lights (led strip)
#define MATERIAL_COOLING_FAN_PIN     7  // PH4, Fan for material cooling
#define CASE_FAN_24V_PIN            -1  // xxx, Fan for case cooling
#define HOTEND_FAN_PIN              13  // PB7, Fan for hotend cooling
#define TOPCAP_FAN_PIN               6  // PH3, Fan for TopCap aka AirManager

#define PS_ON_PIN                   24  // PA2, EN_VCC24HP, Switches on the 24V High Power to steppers and nozzles (bed is not protected on UM3/S5)
#define V5_EXT_ENABLE_PIN           41  // PG0, Enable the 5V external power supply. Used by external components (current protected)

#define ADC_HIGH_POWER_VOLTAGE_PIN  13  // PK5, ADC13 measuring VCC24_HP

#define POWER_FAIL_N_PIN            J5  // PJ5, nPWRFAIL interrupt

// LEDs
#define DEBUG_LED0                  H7  // PH7, Debug LED 0
#define DEBUG_LED1                  G3  // PG3, Debug LED 1
#define DEBUG_LED2                  G4  // PG4, Debug LED 2

// Topcap
#define TOPCAP_PRESENT_PIN          39  // PG2, TopCap present signal

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#endif
