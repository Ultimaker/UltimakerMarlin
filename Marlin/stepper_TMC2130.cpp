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
#include "stepper_TMC2130.h"
#include "usart_spi_driver.h"
#include "fastio.h"
#include "Marlin.h"


#define IHOLD_IRUN_INDEX 0x10
//Hold current configuration
#define IHOLD(n) (uint32_t(n) << 0ULL)
//Run current configuration
#define IRUN(n) (uint32_t(n) << 8ULL)
//Delay in current lowering from IRUN to IHOLD
#define IHOLDDELAY(n) (uint32_t(n) << 16ULL)


#define CHOPCONF_INDEX 0x6C
#define CHOPCONF_TOFF(n)  (n)
#define CHOPCONF_HSTRT(n) (uint32_t(n) << 4UL)
#define CHOPCONF_HEND(n)  (uint32_t(n) << 7UL)
#define CHOPCONF_TBL(n)   (uint32_t(n) << 15UL)
#define CHOPCONF_CHM      (1 << 14UL)
#define CHOPCONF_MS_256   0x00000000UL
#define CHOPCONF_MS_128   0x01000000UL
#define CHOPCONF_MS_64    0x02000000UL
#define CHOPCONF_MS_32    0x03000000UL
#define CHOPCONF_MS_16    0x04000000UL
#define CHOPCONF_MS_8     0x05000000UL
#define CHOPCONF_MS_4     0x06000000UL
#define CHOPCONF_MS_2     0x07000000UL
#define CHOPCONF_MS_1     0x08000000UL
#define CHOPCONF_INTPOL   0x10000000UL


static void setChipSelect(uint8_t chip_idx)
{
    if (chip_idx == 0) WRITE(X_SPI_CS, 0);
    if (chip_idx == 1) WRITE(Y_SPI_CS, 0);
    if (chip_idx == 2) WRITE(Z_SPI_CS, 0);
    if (chip_idx == 3) WRITE(E0_SPI_CS, 0);
    if (chip_idx == 4)
    {
        if (Board::getId() == Board::BOARD_V4)
        {
            WRITE(E1_SPI_CS_V4, 0);
        }
        else if (Board::getId() == Board::BOARD_2621B)
        {
            WRITE(E1_SPI_CS_V3, 0);
        }
    }
}

static void clearChipSelect(uint8_t chip_idx)
{
    if (chip_idx == 0) WRITE(X_SPI_CS, 1);
    if (chip_idx == 1) WRITE(Y_SPI_CS, 1);
    if (chip_idx == 2) WRITE(Z_SPI_CS, 1);
    if (chip_idx == 3) WRITE(E0_SPI_CS, 1);
    if (chip_idx == 4)
    {
        if (Board::getId() == Board::BOARD_V4)
        {
            WRITE(E1_SPI_CS_V4, 1);
        }
        else if (Board::getId() == Board::BOARD_2621B)
        {
            WRITE(E1_SPI_CS_V3, 1);
        }
    }
}

void StepperTMC2130::writeRegister(uint8_t stepper_index, uint8_t register_index, uint32_t value)
{
    setChipSelect(stepper_index);
    UsartSpiDriver::transceive(register_index | 0x80);
    UsartSpiDriver::transceive(value >> 24);
    UsartSpiDriver::transceive(value >> 16);
    UsartSpiDriver::transceive(value >> 8);
    UsartSpiDriver::transceive(value);
    clearChipSelect(stepper_index);
}

uint32_t StepperTMC2130::readRegister(uint8_t stepper_index, uint8_t register_index)
{
    setChipSelect(stepper_index);
    UsartSpiDriver::transceive(register_index);
    UsartSpiDriver::transceive(0x00);
    UsartSpiDriver::transceive(0x00);
    UsartSpiDriver::transceive(0x00);
    UsartSpiDriver::transceive(0x00);
    clearChipSelect(stepper_index);

    setChipSelect(stepper_index);
    uint32_t result = 0;
    UsartSpiDriver::transceive(register_index);
    result |= uint32_t(UsartSpiDriver::transceive(0x00)) << 24;
    result |= uint32_t(UsartSpiDriver::transceive(0x00)) << 16;
    result |= uint32_t(UsartSpiDriver::transceive(0x00)) << 8;
    result |= UsartSpiDriver::transceive(0x00);
    clearChipSelect(stepper_index);

    return result;
}

void StepperTMC2130::init()
{
    SET_OUTPUT(X_SPI_CS);
    SET_OUTPUT(Y_SPI_CS);
    SET_OUTPUT(Z_SPI_CS);
    SET_OUTPUT(E0_SPI_CS);

    if (Board::getId() == Board::BOARD_V4)
    {
        SET_OUTPUT(E1_SPI_CS_V4);
    }
    else if (Board::getId() == Board::BOARD_2621B)
    {
        SET_OUTPUT(E1_SPI_CS_V3);
    }

    for (uint8_t n = 0; n < NUM_MOTOR_DRIVERS; n++)
    {
        clearChipSelect(n);
    }

    for (uint8_t n = 0; n < NUM_MOTOR_DRIVERS; n++)
    {
        // Configure the motor drivers with some reasonable defaults right now.
        // - stealthChop off
        // - spreadCycle on
        //   TOFF=4, TBL=1, HSTRT=6, HEND=0
        // - microPlyer / interpolator on
        //   INTPOL=1
        //   sendgcode M12010 R0x6c V0x14008064
        // - coil current equal to Allegro setting (1.32A amplitude)
        //   sendgcode M12010 R0x10 V0xF0F
        writeRegister(n, IHOLD_IRUN_INDEX, IHOLDDELAY(0) | IRUN(15) | IHOLD(15));
        writeRegister(n, CHOPCONF_INDEX, CHOPCONF_INTPOL | CHOPCONF_MS_16 | CHOPCONF_TBL(1) |
                         CHOPCONF_HEND(0) | CHOPCONF_HSTRT(6) | CHOPCONF_TOFF(4));
    }
}
