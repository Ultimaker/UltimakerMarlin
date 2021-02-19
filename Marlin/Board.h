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
#ifndef BOARD_H
#define BOARD_H


#include <stdint.h>

/** @brief Class to determine the type of hardware being used (board identification)
 *  based on the voltage divider.
 */
class Board
{
public:
    /** @brief Enum for different board variations. There are a few slightly different variants on the boards, the board type can be used to react differently for these boards. */
    typedef enum
    {
        BOARD_NOT_YET_DETECTED=-1,
        BOARD_UNKNOWN,  // Board type cannot be discovered. Newer board, or something is wrong with the board/power supply.
        BOARD_V2_0,     // Ultimaker 2 board or one of the initial prototype board for the Ultimaker 3. Not used in the field. Has 8 microsteps on the Z axis.
        BOARD_V2_X,     // Initial production board for the Ultimaker 3. Has 16 microsteps on the Z.
        BOARD_REV_I,    // Update of the Ultimaker 3 board. Removes all unused components. This requires slightly different safety circuit handling.
        BOARD_2621B,    // Ultimaker 3.1/XL board, rev. B (2621 == 1000897; numbering scheme has changed)
        MAX_BOARD_ID
    } BoardType;

    /** @brief Determines the type of board
     *         Must be called before temperatureInit() to have ADC access.
     */
    static void detect();

    /** @brief Initializes the board
     *         Must be called before temperatureInit() to have ADC access.
     *  @return Returns the initialization result, 0 meaning ok, any other value is a STOP reason
     */
    static uint8_t init();

    /** @brief Returns the board id
     *  @return The id which is one of the values of @ref BoardType
     */
    static BoardType getId()
    {
        return board_id;
    }

    /** @brief Power down the board.
     */
    static void powerDown();

    /** @brief Power up the board
     */
    static void powerUp();

    // @brief  Returns True when this board supports case fans.
    // @return True when the board has case fans.
    static bool hasCaseFans();
private:
    /** @brief Determines if the give analog value is within the specified tolerance of the specified ADC constant
     *  @param analog_value The value to verify
     *  @param adc_constant The constant to verify against
     *  @param tolerance The tolerance to use during verification
     *  @return Returns true if the analog value is within the tolerance value of the ADC constant, otherwise false
     */
    static bool isWithinTolerance(const int16_t analog_value, const int16_t adc_constant, const int16_t tolerance=TOLERANCE);

    /** @brief The detected board id */
    static BoardType board_id;

    /** @brief The tolerance used to create a range in which a specific board is detected. */
    static const int16_t TOLERANCE = 50;

    /** @brief Perform a pre-init, for example setup debug leds, etc.
     *  @return 0 if everything is ok
     */
    static uint8_t executePreInit();

    /** @brief Perform a post-init, for example turning off debug leds, etc.
     *  @return 0 if everything is ok
     */
    static uint8_t executePostInit();

    /** @brief One of the board init functions.
     *  @return 0 if init finished without complications.
     */
    static uint8_t initBoard2621B();
};

#endif//BOARD_H