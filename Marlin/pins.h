#ifndef PINS_H
#define PINS_H

/******************************************************************
 * Ultiboard pin assignment                                       *
 ******************************************************************/

#ifndef __AVR_ATmega2560__
 #error Only the Arduino Mega 2560 MCU is supported
#endif

/*
 * Version/revision
 */
#define ADC_BOARD_ID_PIN            14  /* PK6, ADC14 measuring board ID */
#define BOARD_REV_LSB_PIN           D6  /* PD6, BOARD_2621B/BOARD_V4 revision least significant bit */
#define BOARD_REV_XSB_PIN           38  /* PD7, BOARD_2621B/BOARD_V4 revision middle significant bit */
#define BOARD_REV_MSB_PIN           38  /* PJ7, BOARD_2621B/BOARD_V4 revision most significant bit */

/*
 * Steppers
 */
#define X_STEP_PIN                  25  /* PA3, X-axis stepper driver step output */
#define X_DIR_PIN                   23  /* PA1, X-axis stepper driver direction output */
#define X_ENABLE_PIN                27  /* PA5, X-axis stepper driver enable output */
#define X_DIAG_PIN                  10  /* PB4, X-axis stepper driver interrupt input (Trinamics stepper driver only) */
#define X_SPI_CS                    F0  /* PF0, X-axis stepper driver SPI bus chip select output (Trinamics stepper driver only) */
#define X_STOP_PIN                  22  /* PA0, X-axis limit switch input */

#define Y_STEP_PIN                  32  /* PC5, Y-axis stepper driver step output */
#define Y_DIR_PIN                   33  /* PC4, Y-axis stepper driver direction output */
#define Y_ENABLE_PIN                31  /* PC6, Y-axis stepper driver enable output */
#define Y_DIAG_PIN                  11  /* PB5, Y-axis stepper driver interrupt input (Trinamics stepper driver only) */
#define Y_SPI_CS                    F1  /* PF1, Y-axis stepper driver SPI bus chip select output (Trinamics stepper driver only) */
#define Y_STOP_PIN                  26  /* PA4, Y-axis limit switch input */

#define Z_STEP_PIN                  35  /* PC2, Z-axis stepper driver step output */
#define Z_DIR_PIN                   36  /* PC1, Z-axis stepper driver direction output */
#define Z_ENABLE_PIN                34  /* PC3, Z-axis stepper driver enable output */
#define Z_DIAG_PIN                  12  /* PB6, Z-axis stepper driver interrupt input (Trinamics stepper driver only) */
#define Z_SPI_CS                    F2  /* PF2, Z-axis stepper driver SPI bus chip select output (Trinamics stepper driver only) */
#define Z_STOP_PIN                  29  /* PA7, Z-axis limit switch input */

#define E0_STEP_PIN                 42  /* PL6, Extruder 0 stepper driver step output */
#define E0_DIR_PIN                  43  /* PL7, Extruder 0 stepper driver direction output */
#define E0_ENABLE_PIN               37  /* PC0, Extruder 0 stepper driver enable output */
#define E0_DIAG_PIN                 J3  /* PJ3, Extruder 0 stepper driver interrupt input (Trinamics stepper driver only) */
#define E0_SPI_CS                   F3  /* PF3, Extruder 0 stepper driver SPI bus chip select output (Trinamics stepper driver only) */

#define E1_STEP_PIN                 49  /* PL0, Extruder 1 stepper driver step output */
#define E1_DIR_PIN                  47  /* PL2, Extruder 1 stepper driver direction output */
#define E1_ENABLE_PIN               48  /* PC1, Extruder 1 stepper driver enable output */
#define E1_DIAG_PIN                 J4  /* PJ4, Extruder 1 stepper driver interrupt input (Trinamics stepper driver only) */

/* Extruder 1 stepper driver SPI bus chip select output (Trinamics stepper driver only) */
#define E1_SPI_CS_V3                F4
#define E1_SPI_CS_V4                D4

/*
 * Heaters
 */
#define HEATER_BED_PIN               4  /* PG5,  */
#define HEATER_BED_XL_PIN            9  /* PH6, Heater output pin for the XL-board which has maximum PWM of 10Hz, used depending on the board ID */
#define TEMP_BED_PIN                10  /* ADC10, not regular I/O #10 */

#define HEATER_0_PIN                 2
#define TEMP_0_PIN                   8  /* ADC8, not regular I/O #8 */

#define HEATER_1_PIN                 3
#define TEMP_1_PIN                   9  /* ADC9, not regular I/O #9 */

#define HEATER_2_PIN                -1
#define TEMP_2_PIN                  -1

#define LED_PIN                      8
#define MATERIAL_COOLING_FAN_PIN     7  /* Fan for material cooling */
#define CASE_FAN_24V_PIN             8  /* Fan for case cooling */

#define UM2_HOTEND_FAN_PIN          J6  /* Fan for UM2 hotend */

#define PS_ON_PIN                   A2  /* PA2, EN_VCC24HP, Pin to switch on the 24V High Power to steppers, bed and nozzles */
#define SAFETY_TRIGGERED_PIN        A6  /* Pin to detect the safety circuit has triggered */

#define ADC_OUTPUT_VOLTAGE_PIN      13  /* PK5, ADC13 measuring VCC24_HP */

#define POWER_FAIL_N_PIN            J5  /* nPWRFAIL interrupt */

#define MOTOR_CURRENT_PWM_XY_PIN    44  /* (A4988 stepper driver only) */
#define MOTOR_CURRENT_PWM_Z_PIN     45  /* (A4988 stepper driver only) */
#define MOTOR_CURRENT_PWM_E_PIN     46  /* (A4988 stepper driver only) */
#define MOTOR_CURRENT_PWM_RANGE     2000 /* Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range */

/*
 * LEDs
 */
/* Board v3.x - v4.1 */
#define DEBUG_LED0_V3               F5  /* F5, Debug LED D29, AL0 */
#define DEBUG_LED1_V3               F6  /* F6, Debug LED D30, AL1 */
#define DEBUG_LED2_V3               F7  /* F7, Debug LED D31, AL2 */

/* Board v4.2 */
#define DEBUG_LED0_V4               H7  /* H7, Debug LED0 D29, AL0 */
#define DEBUG_LED1_V4               G3  /* G3, Debug LED1 D30, AL1 */
#define DEBUG_LED2_V4               G4  /* G4, Debug LED3 D31, AL2 */


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
