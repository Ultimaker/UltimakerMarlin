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
#ifndef STEPPER_A4988_H
#define STEPPER_A4988_H

/** A4988 stepper motor driver controller
    This controls only the motor current of the stepper driver.
    Step, direction and enable is directly controlled by stepper.cpp for runtime speed efficienty.
 */
class StepperA4988
{
private:
    static int motor_current_setting[3];
public:
    static void init();
    static void setCurrent(uint8_t driver, int current);
};

#endif//STEPPER_A4988_H
