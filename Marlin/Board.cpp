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
#include "Board.h"

#include <stdlib.h>

#include "Arduino.h"
#include "pins.h"
#include "watchdog.h"
#include "Marlin.h"

#define __USE_DEBUG_LEDS__
#include "DebugLeds.h"

// Calculations @ https://docs.google.com/spreadsheets/d/1qF5tl1OkxcSzZYmVoZcvR5AQCxsvpXnV5bHmrN79-qc
// Connected to 24V - 100k -|- 4k7  - GND = ADC ~221 on Ultimaker 2.0 board
#define MAIN_BOARD_ADC_V2_0 221
// Connected to 24V - 100k -|- 10k  - GND = ADC ~447 ADC on Ultimaker 2.x board
#define MAIN_BOARD_ADC_V2_x 447
// Connected to 24V - 100k -|- 14k7 - GND = ADC ~630 ADC on Ultimaker board revision I
#define MAIN_BOARD_ADC_REV_I 630
// Connected to 24V - 10k -|- 2k2 - GND = ADC ~869 on 2621 board (Ultimaker 3.1 board)
#define MAIN_BOARD_ADC_2621B 869

// Connected to 24V (after high-power switch) - 10k -|- 2k2 - GND = 4.328V (@ Vref=5.1V) = ADC ~869 on 2621 board (Ultimaker 3.1 board)
#define SWITCH_ON_LIMIT_VOLTAGE 200

// The following define selects which power supply you have.
// We need to set our line HIGH to keep our main relay on.
#define PS_ON_AWAKE  HIGH
#define PS_ON_ASLEEP LOW

Board::BoardType Board::board_id = BOARD_NOT_YET_DETECTED;

// Determine the printed circuit board revision type by reading the analog voltage on a defined pin.
// This needs to be done to identify the board.
// Note that we can only call analogRead() before temperatureInit() as temperatureInit takes over control of the ADC peripheral.
void Board::detect()
{
    // Only detect once, because detection will power down the board which makes sure nothing will be interfering with ADC reading
    if (board_id != BOARD_NOT_YET_DETECTED)
    {
        return;
    }

    Board::powerDown();

    int16_t main_board_voltage = analogRead(ADC_INPUT_VOLTAGE_PIN);
    if (isWithinTolerance(main_board_voltage, MAIN_BOARD_ADC_V2_0))
    {
        board_id = BOARD_V2_0;
    }
    else if (isWithinTolerance(main_board_voltage, MAIN_BOARD_ADC_V2_x))
    {
        board_id = BOARD_V2_X;
    }
    else if (isWithinTolerance(main_board_voltage, MAIN_BOARD_ADC_REV_I))
    {
        board_id = BOARD_REV_I;
    }
    else if (isWithinTolerance(main_board_voltage, MAIN_BOARD_ADC_2621B))
    {
        board_id = BOARD_2621B;
    }
    else
    {
        board_id = BOARD_UNKNOWN;
    }

    MSerial.print("LOG:Detected board with id #");
    MSerial.println(board_id);
}

// Note that we can only call analogRead() before temperatureInit() as temperatureInit takes over control of the ADC peripheral.
uint8_t Board::init()
{
    if (getId() == Board::BOARD_2621B)
    {
        return initBoard2621B();
    }

    return 0;
}

void Board::powerDown()
{
    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #endif

    if (getId() == Board::BOARD_REV_I)
    {
        // Board revision I and higher have removed the safety circuit. But only rev I needs to implement this pin hack!
        // This means SAFETY_TRIGGERED_PIN has to be turned as output and switched to HIGH.
        // As this switches on the high end of the relay circuit, while the PS_ON_PIN switches on the low side.
        SET_OUTPUT(SAFETY_TRIGGERED_PIN);
        WRITE(SAFETY_TRIGGERED_PIN, HIGH);
    }
}

void Board::powerUp()
{
    // Right now we cannot power up the BOARD_2621B because this would need the sequence like the one performed during init.
    // This sequence is now not possible since the ADC cannot be accessed at this time anymore.
    // TODO: EM-2760 [New] - fix this. We want to enable power on connection to Opinicus and not earlier
    if (getId() != Board::BOARD_2621B)
    {
        #if defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN); // GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);
        #endif
    }

    if (getId() == Board::BOARD_REV_I)
    {
        // Board revision I and higher have removed the safety circuit. But only rev I needs to implement this pin hack!
        // This means SAFETY_TRIGGERED_PIN has to be turned as output and switched to LOW.
        // As this switches on the high end of the relay circuit, while the PS_ON_PIN switches on the low side.
        SET_OUTPUT(SAFETY_TRIGGERED_PIN);
        WRITE(SAFETY_TRIGGERED_PIN, LOW);
    }
}


// Init sequences

uint8_t Board::executePreInit()
{
    SERIAL_ECHOLNPGM("INIT: pre init");
    uint8_t board_id = getId();

    if (board_id == Board::BOARD_2621B)
    {
        INITIALIZE_DEBUG_LEDS;
        SWITCH_ON_DEBUG_LED(DEBUG_LED0);

        // Ensure the High Power supply is switched off.
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);     // First setting the value before making the port an output prevents unwanted glitch.
        SET_OUTPUT(PS_ON_PIN);

        SET_INPUT(POWER_FAIL_N_PIN);        // set POWER_FAIL pin as input
        WRITE(POWER_FAIL_N_PIN, HIGH);      // initialize as pull up

        // Wait for Power Fail input to become high, i.e. no power error.
        unsigned long start_time = millis();
        while ( !READ(POWER_FAIL_N_PIN) )
        {
            // If after half a second, this pin is not high, something is wrong, discontinue.
            if ((millis() - start_time > 0.5 * 1000UL))
            {
                return STOP_REASON_PCB_INIT_POWERFAIL_N;
            }
            watchdog_reset();
        }

        SWITCH_ON_DEBUG_LED(DEBUG_LED1);
    }

    return 0;
}

uint8_t Board::executePostInit()
{
    SERIAL_ECHOLNPGM("INIT: post init");
    uint8_t board_id = getId();

    if (board_id == Board::BOARD_2621B)
    {
        SWITCH_OFF_DEBUG_LED(DEBUG_LED0);
        SWITCH_OFF_DEBUG_LED(DEBUG_LED1);
        SWITCH_OFF_DEBUG_LED(DEBUG_LED2);
    }
    return 0;
}

uint8_t Board::initBoard2621B()
{
    SERIAL_ECHOLNPGM("INIT: BOARD_2621B");
    uint8_t result = executePreInit();
    if (result != 0)
    {
        return result;
    }

    // The output voltage should be below 6V before the main switch is
    // turned on to make sure that the inrush current limiter operates
    // as designed. The voltage may be higher if the switch was on just
    // before we got here.
    unsigned long start_time = millis();
    int16_t high_power_voltage = analogRead(ADC_OUTPUT_VOLTAGE_PIN);
    do
    {
        high_power_voltage += analogRead(ADC_OUTPUT_VOLTAGE_PIN);
        high_power_voltage /= 2;
        watchdog_reset();

        if (millis() - start_time >= 6 * 1000UL)    // Discharging should take less than 6 seconds.
        {
            // error! voltage not low enough, risk of shoot-through
            MSerial.print("LOG:Error initial output voltage too high: ");
            MSerial.println(high_power_voltage);
            return STOP_REASON_PCB_INIT_PRE_CHARGE;
        }
    } while (high_power_voltage > SWITCH_ON_LIMIT_VOLTAGE);

    // turn on the main power supply switch
    SWITCH_ON_DEBUG_LED(DEBUG_LED1);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);

    // Wait for the output voltage to rise.
    _delay_ms(50);

    // Measure output voltage.
    high_power_voltage = analogRead(ADC_OUTPUT_VOLTAGE_PIN);
    if (!isWithinTolerance(high_power_voltage, MAIN_BOARD_ADC_2621B))
    {
        // It's not - meaning the high power safety switched turned off
        SWITCH_OFF_DEBUG_LED(DEBUG_LED1);
        MSerial.print("# ERROR: final voltage too low: ");
        MSerial.println(high_power_voltage);
        return STOP_REASON_PCB_INIT_HP_SW_SHUTDOWN;
    }

    return executePostInit();
}

bool Board::isWithinTolerance(const int16_t analog_value, const int16_t adc_constant, const int16_t tolerance)
{
    return abs(analog_value - adc_constant) < tolerance;
}

bool Board::hasCaseFans()
{
    return board_id == Board::BOARD_2621B;
}
