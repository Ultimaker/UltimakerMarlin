/* Copyright (c) 2002, 2003, 2004, 2007 Marek Michalkiewicz
   Copyright (c) 2005, 2006 Bjoern Haase
   Copyright (c) 2008 Atmel Corporation
   Copyright (c) 2008 Wouter van Gulik
   Copyright (c) 2009 Dmitry Xmelkov
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: eeprom.h,v 1.21.2.13 2009/12/03 18:38:59 arcanum Exp $ */

#ifndef _AVR_EEPROM_H_
#define _AVR_EEPROM_H_ 1

#include <assert.h>
#include <stdint.h>
#include <sys/types.h>
#include <string.h>

//Using __ name mangling here, to prevent possible name collisions with the actual AVR code.
extern uint8_t avr_simulation_eeprom_storage[4096];

static inline uint8_t eeprom_read_byte (const uint8_t *__p)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint8_t));
    return avr_simulation_eeprom_storage[long(__p)];
}
static inline uint16_t eeprom_read_word (const uint16_t *__p)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint16_t));
    return *(uint16_t*)&avr_simulation_eeprom_storage[long(__p)];
}
static inline uint32_t eeprom_read_dword (const uint32_t *__p)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint32_t));
    return *(uint32_t*)&avr_simulation_eeprom_storage[long(__p)];
}
#define eeprom_read_float eeprom_read_float_
static inline float eeprom_read_float_ (const float *__p)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(float));
    return *(float*)&avr_simulation_eeprom_storage[long(__p)];
}
static inline void eeprom_read_block (void *__dst, const void *__src, size_t __n)
{
    assert(long(__src) >= 0 && (unsigned long)(__src) <= sizeof(avr_simulation_eeprom_storage) - __n);
    memcpy(__dst, &avr_simulation_eeprom_storage[long(__src)], __n);
}

static inline void eeprom_write_byte (uint8_t *__p, uint8_t __value)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint8_t));
    avr_simulation_eeprom_storage[long(__p)] = __value;
}
static inline void eeprom_write_word (uint16_t *__p, uint16_t __value)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint16_t));
    *(uint16_t*)&avr_simulation_eeprom_storage[long(__p)] = __value;
}

static inline void eeprom_write_dword (uint32_t *__p, uint32_t __value)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(uint32_t));
    *(uint32_t*)&avr_simulation_eeprom_storage[long(__p)] = __value;
}

#define eeprom_write_float eeprom_write_float_
static inline void eeprom_write_float_ (float *__p, float __value)
{
    assert(long(__p) >= 0 && (unsigned long)(__p) <= sizeof(avr_simulation_eeprom_storage) - sizeof(float));
    *(float*)&avr_simulation_eeprom_storage[long(__p)] = __value;
}

static inline void eeprom_write_block (const void *__src, void *__dst, size_t __n)
{
    assert(long(__src) >= 0 && (unsigned long)(__src) <= sizeof(avr_simulation_eeprom_storage) - __n);
    memcpy(&avr_simulation_eeprom_storage[long(__dst)], __src, __n);
}

//The update functions are a read&write if different. So we can just pass those on to the write functions.
static inline void eeprom_update_byte (uint8_t *__p, uint8_t __value) { eeprom_write_byte(__p, __value); }
static inline void eeprom_update_word (uint16_t *__p, uint16_t __value) { eeprom_write_word(__p, __value); }
static inline void eeprom_update_dword (uint32_t *__p, uint32_t __value) { eeprom_write_dword(__p, __value); }
static inline void eeprom_update_float (float *__p, float __value) { eeprom_write_float(__p, __value); }
static inline void eeprom_update_block (const void *__src, void *__dst, size_t __n) { eeprom_write_block(__src, __dst, __n); }

#endif	/* !_AVR_EEPROM_H_ */
