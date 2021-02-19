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
#include <stdio.h>

#include "Arduino.h"
#include "pins.h"
#include "watchdog.h"
#include "Marlin.h"
#include "temperature.h"
#include "stepper.h"

#define __USE_DEBUG_LEDS__
#include "DebugLeds.h"

// Connected to 24V - 10k -|- 2k2 - GND = ADC ~869 on 2621 board (Ultimaker 3.1 board)
#define MAIN_BOARD_24V  869

#define MAIN_BOARD_24V_VOLTAGE_DIVIDER_FACTOR (2.2 / (2.2 + 10.0))
#define MAIN_BOARD_24V_CONVERT_TO_RAW_ADC(v) (int)(v * MAIN_BOARD_24V_VOLTAGE_DIVIDER_FACTOR / 5.0 * 1024.0)

Board::BoardType Board::board_id = BOARD_NOT_YET_DETECTED;

// Determine the printed circuit board revision type by reading the hardware coded id-pins.
void Board::detect()
{
    // Configure the ID pins for reading.
    SET_INPUT(BOARD_REV_1_PIN);
    SET_INPUT(BOARD_REV_2_PIN);
    SET_INPUT(BOARD_REV_3_PIN);
    // Enable the internal pull up resistors.
    WRITE(BOARD_REV_1_PIN, HIGH);
    WRITE(BOARD_REV_2_PIN, HIGH);
    WRITE(BOARD_REV_3_PIN, HIGH);

    board_id = static_cast<BoardType>(READ(BOARD_REV_1_PIN) + (READ(BOARD_REV_2_PIN) << 1) + (READ(BOARD_REV_3_PIN) << 2));

    MSerial.print("LOG:Detected board with id #");
    MSerial.println(board_id);
}

uint8_t Board::init()
{
    INITIALIZE_DEBUG_LEDS;

    // Set the algorithm defaults before any error is detected, otherwise these
    // functions will never be called after an error is triggered.
    setPowerDefaults();
    setPidDefaults();

    uint8_t result = executePreInit();
    if (result != 0)
    {
        return result;
    }

    if (board_id == BOARD_E2)
    {
        result = initBoardE2();
    }
    else
    {
        MSerial.print("LOG:Unsupported board type detected. Aborting.");
        return STOP_REASON_UNSUPPORTED_BOARD_ID;
    }

    if (result != 0)
    {
        return result;
    }

    executePostInit();

    return result;
}

void Board::powerDownSafely()
{
    // A sudden drop in the power supply can damage the stepper drivers.
    // By disabling the steppers before switching off the 24V power supply we prevent this.
    disable_all_steppers();
    powerDown();
}

void Board::powerDown()
{
    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #endif

    enable_external_5v(false);
}

void Board::powerUp()
{
    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN); // GND
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif

    enable_external_5v(true);
    _delay_ms(100);     // Safety delay on request of Electronics department for not damaging the 24V_HP FET.
}


uint8_t Board::executePreInit()
{
    SERIAL_ECHOLNPGM("INIT: pre init");
    if (board_id == BOARD_E2)
    {
        SWITCH_ON_DEBUG_LED(DEBUG_LED0);

        // Enable the topcap inputs
        SET_INPUT(TOPCAP_PRESENT_PIN);

        // Ensure the High Power supply is switched off.
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);     // First setting the value before making the port an output prevents unwanted glitch.
        SET_OUTPUT(PS_ON_PIN);

        SET_INPUT(POWER_FAIL_N_PIN);        // set POWER_FAIL pin as input
        WRITE(POWER_FAIL_N_PIN, HIGH);      // initialize as pull up

        // Verify the Power Fail detection circuit.
        // Wait for Power Fail input to become high, i.e. power == ok.
        unsigned long start_time = millis();
        fprintf_P(stderr, PSTR("Waiting for nPWR_FAIL to negate"));
        while ( !READ(POWER_FAIL_N_PIN) )
        {
            // If after half a second, this pin is not high, something is wrong, discontinue.
            if ((millis() - start_time > 0.5 * 1000UL))
            {
                fprintf_P(stderr, PSTR("\nnPWR_FAIL did not negate in time\n"));
                return STOP_REASON_PCB_INIT_POWERFAIL_N;
            }

            _delay_ms(50);
            watchdog_reset();
            fprintf_P(stderr, PSTR("."));
        }
        fprintf_P(stderr, PSTR("\nnPWR_FAIL ok\n"));
    }

    return 0;
}

void Board::executePostInit()
{
    SWITCH_OFF_DEBUG_LED(DEBUG_LED0);
    SWITCH_OFF_DEBUG_LED(DEBUG_LED1);
    SWITCH_OFF_DEBUG_LED(DEBUG_LED2);
}

bool Board::test24HP()
{
    // Ensure 24V HP is on
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
    _delay_ms(1000);    // Very slow. Don't shorten this too much! Can take 100ms to rise and switching off during rise can break the FET.

    // Check the 24V HP voltage is correct.
    int16_t high_power_voltage = analogRead(ADC_HIGH_POWER_VOLTAGE_PIN);
    fprintf_P(stderr, PSTR("Self check high power voltage on: %d "), high_power_voltage);

    if (!isWithinTolerance(high_power_voltage, MAIN_BOARD_24V))
    {
        fprintf_P(stderr, PSTR("FAIL\n"));
        return false;
    } else {
        fprintf_P(stderr, PSTR("OK\n"));
    }

    // Now switch off the 24V_HP and verify that the voltage drops.
    WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    for (uint8_t n = 0; n < 50; n++)
    {
        _delay_ms(100);
        watchdog_reset();

        high_power_voltage = analogRead(ADC_HIGH_POWER_VOLTAGE_PIN);
        if (high_power_voltage < MAIN_BOARD_24V_CONVERT_TO_RAW_ADC(20.0))
        {
            break;
        }
    }
    fprintf_P(stderr, PSTR("Self check high power voltage off: %d "), high_power_voltage);
    if (high_power_voltage > MAIN_BOARD_24V_CONVERT_TO_RAW_ADC(20.0))
    {
        fprintf_P(stderr, PSTR("FAIL\n"));
        // Error is ignored on purpose since a short-circuited FET still results in a good working printer.
        SERIAL_ECHO_START
        SERIAL_ECHOLNPGM("High Power voltage FET fails to switch off");
    }
    else
    {
        fprintf_P(stderr, PSTR("OK\n"));
    }

    // Switch on the 24V_HP again.
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
    _delay_ms(100);     // Safety delay on request of Electronics department for not damaging the 24V_HP FET.
    return true;
}

uint8_t Board::initBoardE2()
{
    fprintf_P(stderr, PSTR("\nPowering up VCC24_HP\n"));
    // turn on the main power supply switch
    SWITCH_ON_DEBUG_LED(DEBUG_LED1);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);

    if(!Board::test24HP())
    {
        fprintf_P(stderr, PSTR("VCC24_HP failed\n"));
        return STOP_REASON_HP_SW_VOLTAGE_LOW;
    }
    fprintf_P(stderr, PSTR("VCC24_HP ok\n"));

    enable_external_5v(true);
    return 0;
}

bool Board::isWithinTolerance(const int16_t analog_value, const int16_t adc_constant, const int16_t tolerance)
{
    return abs(analog_value - adc_constant) < tolerance;
}

void Board::enable_external_5v(bool enable)
{
    if (board_id == BOARD_E2)
    {
        #if defined(V5_EXT_ENABLE_PIN) && V5_EXT_ENABLE_PIN > -1
        SET_OUTPUT(V5_EXT_ENABLE_PIN);
        WRITE(V5_EXT_ENABLE_PIN, enable);
        #endif
    }
}

bool Board::hasCaseFans()
{
    return board_id == BOARD_2621B || board_id == BOARD_V4;
}

void Board::setPidDefaults()
{
    if (board_id == BOARD_E2)
    {
        // M301 - Set nozzle PID parameters FF, P, I and D
        for (uint8_t extruder = 0; extruder < EXTRUDERS; extruder++)
        {
            hotend_pid[extruder].setKff(0.4);
            hotend_pid[extruder].setKp(10.00);
            hotend_pid[extruder].setKi(0.5);
            hotend_pid[extruder].setKd(40);
            hotend_pid[extruder].setKiMax(15);
            hotend_pid[extruder].setFunctionalRange(45);
            hotend_pid[extruder].setKpcf(0.06);
        }

        // M304 - Set bed PID parameters FF, P, I and D
        heated_bed_pid.setKff(2.5);
        heated_bed_pid.setKp(250);
        heated_bed_pid.setKi(2);
        heated_bed_pid.setKd(0);
        heated_bed_pid.setKiMax(500);
        heated_bed_pid.setFunctionalRange(2);
    }
}

void Board::setPowerDefaults()
{
    if (board_id == BOARD_E2)
    {
        // M290 P<total_power_budget> I<idle_power_consumption>
        pwr.setTotalPowerBudget(221);
        pwr.setIdlePowerConsumption(10);    // Note that engaged motors can eat a lot of power, so ensure those are disabled!

        // M291 R<nominal_bed_resistance> A<bed_resistance_per_degree> V<bed_voltage>
        pwr.setNominalBedResistance(3.678255);
        pwr.setBedResistancePerDegree(0.012285);
        pwr.setBedVoltage(24);

        // M292 T<hotend_nr> R<nominal_hotend_cartridge_resistance> V<hotend_slot_voltage> P<max_hotend_power>
        for (uint8_t extruder = 0; extruder < EXTRUDERS; extruder++)
        {
            pwr.setNominalHotendResistance(extruder, 16);
            pwr.setHotendVoltage(extruder, 24);
            pwr.setMaxPowerUsageForHeater(extruder, 40);
        }
    }
}
