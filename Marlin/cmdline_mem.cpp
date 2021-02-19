/*
    Copyright (c) 2019-2021 Ultimaker B.V. All rights reserved.

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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "cmdline.h"


/******************************************************************************/
/* Macros                                                                     */
/******************************************************************************/
#define STACK_MAGIC             0x55    /* Magic number marking free memory to detect stack growth */


/******************************************************************************/
/* Global Data                                                                */
/******************************************************************************/


/******************************************************************************/
/* Local Prototypes                                                           */
/******************************************************************************/


/******************************************************************************/
/* Local Implementations                                                      */
/******************************************************************************/

// Convert the given string to an integer value.
// Allowed are decimal values or hexadecimal values starting with '0x' or '0X'
// Returns 0 on success, -1 when the string contains non-value characters
static inline int8_t str2value(char *string, unsigned int *value)
{
    char  *end;

    // Convert the number part
    *value = strtoul(string, &end, 0);
    // Check for trailing garbage
    if (end != (string + strlen(string)))
        return -1;

    return 0;
}


uint8_t read_byte(uint8_t mem_type, uint8_t *addr)
{
    uint8_t data_byte;

    switch(mem_type)
    {
    case 'F':
        data_byte = pgm_read_byte(addr);
        break;
    case 'E':
        data_byte = eeprom_read_byte(addr);
        break;
    default:
        data_byte = *addr;
        break;
    }

    return data_byte;
}

// Dump the data for the given memory type, from the given address over the specified length.
// @mem_type: memory type to read. 'R' = RAM, 'F' = Flash, 'E' = EEPROM
// @addr: pointer to the first memory address to display
// @length: length of the data to display in bytes.
void hexdump(void *cmdline, char mem_type, uint8_t *addr, uint16_t len)
{
    size_t  index = 0;

    cmdline_printf_P(cmdline, PSTR("%c @%.4x, %d bytes:\n"), mem_type, addr, len);

    while (index < len) {
        size_t  byte;
        char    line[81] = {0};

        /* Address */
        sprintf_P(line, PSTR("addr[0x%.4x]: "), index);

        /* Bytes in HEX */
        byte = 0;
        while (byte < 16) {
            if ((index + byte) < len) {
                char    buffer[16];
                uint8_t data_byte = read_byte(mem_type, &addr[index + byte]);

                sprintf_P(buffer, PSTR(" %.2x"), data_byte);
                strcat(line, buffer);
            } else
                strcat(line, "   ");
            byte++;
        }

        /* Separator */
        strcat(line, "  ");

        /* Bytes in ASCII */
        byte = 0;
        while ((byte < 16) && ((index + byte) < len)) {
            char    buffer[16];
            uint8_t data_byte = read_byte(mem_type, &addr[index + byte]);

            if (data_byte >= ' ' && data_byte <= '~') {
                sprintf_P(buffer, PSTR("%c"), data_byte);
                strcat(line, buffer);
            } else
                strcat(line, ".");
            byte++;
        }

        /* Line ending */
        cmdline_printf_P(cmdline, PSTR("%s\n"), line);
        line[0] = 0;
        /* Next line */
        index += 16;
    }
}


/******************************************************************************/
/* Global Implementations                                                     */
/******************************************************************************/
void cmdline_initstackprotector(void)       // Function naming not to Ultimaker coding style
{
    extern unsigned char  __heap_start;   /* This variable's ADDRESS points to heap start */
    extern void           *__brkval;      /* This variable points to the heap end, but IS ZERO when the heap is empty */
    unsigned int          heap_end = __brkval?(unsigned int)__brkval:(unsigned int)&__heap_start;
    unsigned char         *stack_top = (unsigned char *)heap_end;

#ifdef DEBUG
//    cmdline_printf_P(cmdline, PSTR("__brkval     = @%p\n"), __brkval);
//    cmdline_printf_P(cmdline, PSTR("__heap_start = @%p\n"), &__heap_start);
//    cmdline_printf_P(cmdline, PSTR("stack protector @%p-%p\n"), stack_top, &stack_top);
#endif /* DEBUG */

    while (stack_top < (unsigned char *)&stack_top) {
        *stack_top = STACK_MAGIC;
        stack_top++;
    }
}


int cmdline_meminfo(void *cmdline, int argc, char *argv[] __attribute__((unused)))
/*
 * Usage
 * =====
 * Print memory usage:
 * mem
 */
{
    extern unsigned char  __bss_end;      /* This variable's ADDRESS points to the .bss section end */
    extern unsigned char  __heap_start;   /* This variable's ADDRESS points to heap start */
    extern void           *__brkval;      /* This variable points to the heap end, but IS ZERO when the heap is empty */
    unsigned int          heap_end = __brkval?(unsigned int)__brkval:(unsigned int)&__heap_start;
    unsigned char         *stack_top = (unsigned char *)heap_end;

    if (argc > 1)
        return ERR_SYNTAX;

    while (stack_top < (unsigned char *)&stack_top) {
        if (*stack_top != STACK_MAGIC)
            break;
        stack_top++;
    }

    cmdline_printf_P(cmdline, PSTR(".bss:  0x%.4x...0x%.4x = %d bytes\n"), 0x200, (unsigned int)&__bss_end -1, (unsigned int)&__bss_end - 0x200);
    if (heap_end != (unsigned int)&__heap_start)
        cmdline_printf_P(cmdline, PSTR("heap:  0x%.4x...0x%.4x = %d bytes\n"), (unsigned int)&__heap_start, heap_end -1, heap_end - (unsigned int)&__heap_start);
    cmdline_printf_P(cmdline, PSTR("free:  0x%.4x...0x%.4x = %d bytes\n"), heap_end, (unsigned int)stack_top -1, (unsigned int)stack_top - heap_end);
    cmdline_printf_P(cmdline, PSTR("stack: 0x%.4x...0x%.4x = %d bytes\n"), (unsigned int)stack_top, 0x2200 -1, 0x2200 - (unsigned int)stack_top);

    return ERR_OK;
}


int cmdline_memdump(void *cmdline, int argc, char *argv[])
/*
 * Usage
 * =====
 * Dump RAM contents:
 * memdump [0x0] [0x40]
 *
 * Dump FLASH contents:
 * flashdump [0x0] [0x40]
 *
 * Dump EEPROM contents:
 * eepromdump [0x0] [0x40]
 */
{
    unsigned int  start = 0;
    unsigned int  length = 0x40;

    if ((argc > 3) || (argc < 1))
        return ERR_SYNTAX;

    // First argument: Start address (optional)
    if ((argc > 1) && str2value(argv[1], &start))
        return ERR_SYNTAX;

    // Second argument: Dump length (optional)
    if ((argc > 2) && (str2value(argv[2], &length)))
        return ERR_SYNTAX;

    // Figure out the memory type to dump
    uint8_t mem_type = 'R';
    if (strcmp(argv[0], "flashdump") == 0) {mem_type = 'F';}
    if (strcmp(argv[0], "eepromdump") == 0) {mem_type = 'E';}

    // Do the actual dumping
    hexdump(cmdline, mem_type, (uint8_t *)start, length);

    return ERR_OK;
}
