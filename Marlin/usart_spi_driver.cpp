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
#include "usart_spi_driver.h"
#include "fastio.h"

/** Static driver configuration */
#define USART_SPI_SERIAL_INDEX 1
#define BITRATE 50000

#if USART_SPI_SERIAL_INDEX == 1
#define MISO_PIN D2
#define MOSI_PIN D3
#define SCK_PIN D5
#endif

/** Macro magic to convert the SERIAL_INDEX to the proper register names */
#define _REGNAME(base,num,post) base##num##post
#define REGNAME(base,num,post) _REGNAME(base,num,post)

//USART registers
#define UDRn    REGNAME(UDR,   USART_SPI_SERIAL_INDEX, )
#define UCSRnA  REGNAME(UCSR,  USART_SPI_SERIAL_INDEX, A)
#define UCSRnB  REGNAME(UCSR,  USART_SPI_SERIAL_INDEX, B)
#define UCSRnC  REGNAME(UCSR,  USART_SPI_SERIAL_INDEX, C)
#define UBRRn   REGNAME(UBRR,  USART_SPI_SERIAL_INDEX, )

//USART register bits
#define RXENn   REGNAME(RXEN,  USART_SPI_SERIAL_INDEX, )
#define TXENn   REGNAME(TXEN,  USART_SPI_SERIAL_INDEX, )
#define UMSELn0 REGNAME(UMSEL, USART_SPI_SERIAL_INDEX, 0)
#define UMSELn1 REGNAME(UMSEL, USART_SPI_SERIAL_INDEX, 1)
#define UCPOLn  REGNAME(UCPOL, USART_SPI_SERIAL_INDEX, )
#define UDREn   REGNAME(UDRE,  USART_SPI_SERIAL_INDEX, )
#define RXCn    REGNAME(RXC,   USART_SPI_SERIAL_INDEX, )
//This register bit definition missing from <avr/io.h>, but is needed to properly setup SPI mode 1 and 3 for the USART SPI
#define UCPHAn 1

void UsartSpiDriver::init(uint8_t mode)
{
    SET_OUTPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
    SET_OUTPUT(SCK_PIN);

    //Configure USARTn as SPI bus with proper mode
    UCSRnA = 0;
    UCSRnB = _BV(RXENn) | _BV(TXENn);
    switch(mode)
    {
    case 0:
        UCSRnC = _BV(UMSELn1) | _BV(UMSELn0);
        break;
    case 1:
        UCSRnC = _BV(UMSELn1) | _BV(UMSELn0) | _BV(UCPHAn);
        break;
    case 2:
        UCSRnC = _BV(UMSELn1) | _BV(UMSELn0) | _BV(UCPOLn);
        break;
    case 3:
        UCSRnC = _BV(UMSELn1) | _BV(UMSELn0) | _BV(UCPHAn) | _BV(UCPOLn);
        break;
    }
    UBRRn = (F_CPU / (2L * BITRATE)) - 1L;
}

void UsartSpiDriver::transceive(uint8_t* buffer, uint8_t buffer_size)
{
    while(buffer_size)
    {
        *buffer = transceive(*buffer);
        buffer++;
        buffer_size--;
    }
}

uint8_t UsartSpiDriver::transceive(uint8_t data)
{
    while( !( UCSRnA & _BV(UDREn)) ) {}
    UDRn = data;
    while( !(UCSRnA & _BV(RXCn)) ) {}
    return UDRn;
}

