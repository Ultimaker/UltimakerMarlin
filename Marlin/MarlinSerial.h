/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"

#if !defined(SERIAL_PORT)
#define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                            (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                            (port == 3 && defined(UBRR3H)))

// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (these are expanded
// depending on selected serial port
#define M_UCSRxA SERIAL_REGNAME(UCSR,SERIAL_PORT,A) // defines M_UCSRxA to be UCSRnA where n is the serial port number
#define M_UCSRxB SERIAL_REGNAME(UCSR,SERIAL_PORT,B)
#define M_RXENx SERIAL_REGNAME(RXEN,SERIAL_PORT,)
#define M_TXENx SERIAL_REGNAME(TXEN,SERIAL_PORT,)
#define M_RXCIEx SERIAL_REGNAME(RXCIE,SERIAL_PORT,)
#define M_UDREx SERIAL_REGNAME(UDRE,SERIAL_PORT,)
#define M_UDRx SERIAL_REGNAME(UDR,SERIAL_PORT,)
#define M_UBRRxH SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_U2Xx SERIAL_REGNAME(U2X,SERIAL_PORT,)



#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


// Define constants and variables for buffering incoming serial data.  We're using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail is the index of the location from which to read.
// 256 is the max limit due to uint8_t head and tail. Use only powers of 2. (...,16,32,64,128,256)
// 1 message has a maximum length of 66 bytes (start_of_packet[2] + sequence_number[1] + payload_size[1] + payload[MAX_CMD_SIZE] + crc[2], where MAC_CMD_SIZE=60)
// So with a 256 buffer we can buffer a maximum of 3 messages if the full lenght is used. This will never be the case! So an expected minimum of 4 can be used.
// Alltough this buffer could theoretically overrun, because of the fact that this buffer is also read very fast, it's very unlikely to have overruns.
// Therefore the maximum allowed packets to be in transit (on the wire, meaning that have not be acknowled yet) can be 1 higher then the expected minimum: 5 messages.
// Even in case of an overrun, the protocol implemented will correct itself!
#define RX_BUFFER_SIZE 256

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
};

#if UART_PRESENT(SERIAL_PORT)
  extern ring_buffer rx_buffer;
#endif

class MarlinSerial //: public Stream
{

  public:
    MarlinSerial();
    void begin(int32_t);
    void end();
    int16_t peek(void);
    int16_t read(void);
    void flush(void);

    FORCE_INLINE int16_t available(void)
    {
      return (uint16_t)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
    }

    FORCE_INLINE void write(const uint8_t c)
    {
      /* Wait for empty transmit buffer */
      while (!((M_UCSRxA) & (1 << M_UDREx)))
        ;

      /* Put data into buffer, sends the data */
      M_UDRx = c;
    }


    FORCE_INLINE void checkRx(void)
    {
      if((M_UCSRxA & (1<<M_RXCx)) != 0) {
        unsigned char c  =  M_UDRx;
        uint8_t i = (uint16_t(rx_buffer.head) + 1) % RX_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != rx_buffer.tail) {
          rx_buffer.buffer[rx_buffer.head] = c;
          rx_buffer.head = i;
        }
      }
    }


    private:
    void printNumber(uint32_t, uint8_t);
    void printFloat(double, uint8_t);


  public:

    FORCE_INLINE void write(const char *str)
    {
      while (*str)
        write(*str++);
    }


    FORCE_INLINE void write(const uint8_t *buffer, size_t size)
    {
      while (size--)
        write(*buffer++);
    }

    FORCE_INLINE void print(const String &s)
    {
      for (int16_t i = 0; i < (int16_t)s.length(); i++) {
        write(s[i]);
      }
    }

    FORCE_INLINE void print(const char* const str)
    {
      write(str);
    }
    void print(char, int16_t = BYTE);
    void print(unsigned char, int16_t = BYTE);
    void print(int16_t, int16_t = DEC);
    void print(uint16_t, int16_t = DEC);
    void print(int32_t, int16_t = DEC);
    void print(uint32_t, int16_t = DEC);
    void print(double, int16_t = 2);

    void println(const String &s);
    void println(const char[]);
    void println(char, int16_t = BYTE);
    void println(unsigned char, int16_t = BYTE);
    void println(int16_t, int16_t = DEC);
    void println(uint16_t, int16_t = DEC);
    void println(int32_t, int16_t = DEC);
    void println(uint32_t, int16_t = DEC);
    void println(double, int16_t = 2);
    void println(void);
};

extern MarlinSerial MSerial;

#endif
