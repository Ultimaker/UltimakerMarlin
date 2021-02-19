/*
    Copyright (c) 2017-2021 Ultimaker B.V. All rights reserved.

    Marlin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Marlin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Marlin.h"
#include "stepper_A4988.h"

int StepperA4988::motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;

/** @brief Initializes the output pins and set the outut current to the default
 */
void StepperA4988::init() //Initialize Digipot Motor Current
{
    #if MOTOR_CURRENT_PWM_XY_PIN > -1
    pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
    #endif

    #if MOTOR_CURRENT_PWM_Z_PIN > -1
    pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
    #endif

    #if MOTOR_CURRENT_PWM_E_PIN > -1
    pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
    #endif

    setCurrent(0, motor_current_setting[0]);
    setCurrent(1, motor_current_setting[1]);
    setCurrent(2, motor_current_setting[2]);

    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible.
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
}

void StepperA4988::setCurrent(uint8_t driver, int current)
{
  long analog_value = (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE;
  #if MOTOR_CURRENT_PWM_XY_PIN > -1
    if (driver == 0) analogWrite(MOTOR_CURRENT_PWM_XY_PIN, analog_value);
  #endif

  #if MOTOR_CURRENT_PWM_Z_PIN
    if (driver == 1) analogWrite(MOTOR_CURRENT_PWM_Z_PIN, analog_value);
  #endif

  #if MOTOR_CURRENT_PWM_E_PIN
    if (driver == 2) analogWrite(MOTOR_CURRENT_PWM_E_PIN, analog_value);
  #endif
}
