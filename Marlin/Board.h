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

// The following define selects which power supply you have.
// We need to set our line HIGH to keep our main relay on.
#define PS_ON_AWAKE  HIGH
#define PS_ON_ASLEEP LOW


/** @brief Class to determine the type of hardware being used (board identification)
 *  based on the voltage divider.
 */
class Board
{
public:
    /** @brief Enum for different board variations. There are a few slightly different variants on the boards, the board type can be used to react differently for these boards. */
    typedef enum
    {
        BOARD_NOT_YET_DETECTED=-2,
        BOARD_UNKNOWN=-1,   // Board type cannot be discovered. Newer board, or something is wrong with the board/power supply.
        BOARD_2621B=0,      // S5 board, rev. B (2621 == 1000897; numbering scheme has changed). Introduced TMC2130 stepper drivers and flow sensor.
        BOARD_V4=1,         // Ultimainboard v4 with SMARC SOM
        BOARD_E2=4,         // Ultimaker E1. Based on BOARD_V4 but removed SOM, 1 extruder, UM2+ type nozzles (no print cores), no bed leveling and optional flow sensor.
        BOARD_V2_X=7,       // Ultimainboard v2 for UM3. Has 16 microsteps on the Z. REV_I variant removed all unused components and requires slightly different safety circuit handling.
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

    /** @brief Disable all stepper motors and then switch of the 24V power supply
    */
    static void powerDownSafely();

    /** @brief Enable the 24V Power supply.
     */
    static void powerUp();

    // @brief  Returns True when this board supports case fans.
    // @return True when the board has case fans.
    static bool hasCaseFans();

    /// Set the default PID values for this board type.
    // Note that these defaults can be overruled by the M12005 command.
    static void setPidDefaults();

    /// Set the default Power Usage values for this board type.
    static void setPowerDefaults();

    /**
     * @brief Tests the high power 24V supply.
     * First this turns on the high power 24V (HP24) power supply and checks if the voltage is 24V (within a margin)
     *  Then it turns off HP24 and checks that the voltage is below 20V after 5 seconds.
     * If all tests passed, it leaves the HP24 on
     * @return Returns true if all the checks passed
     */
    static bool test24HP();

private:
    /** @brief Determines if the give analog value is within the specified tolerance of the specified ADC constant
     *  @param analog_value The value to verify
     *  @param adc_constant The constant to verify against
     *  @param tolerance The tolerance to use during verification
     *  @return Returns true if the analog value is within the tolerance value of the ADC constant, otherwise false
     */
    static bool isWithinTolerance(const int16_t analog_value, const int16_t adc_constant, const int16_t tolerance=TOLERANCE);

    static void enable_external_5v(bool enable);

    /** @brief The detected board id */
    static BoardType board_id;

    /** @brief The tolerance used to create a range in which a specific board is detected. */
    static const int16_t TOLERANCE = 50;

    /** @brief Perform a pre-init, for example setup debug leds, etc.
     *  @return 0 if everything is ok
     */
    static uint8_t executePreInit();

    /** @brief Perform a post-init, for example turning off debug leds, etc.
     */
    static void executePostInit();

    /** @brief One of the board init functions.
     *  @return 0 if init finished without complications.
     */
    static uint8_t initBoardE2();

    /** @brief Disable the 24V power supply.
     */
    static void powerDown();
};

#endif//BOARD_H

