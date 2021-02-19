#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "../component/base.h"
#include "../frontend/frontend.h"
#include "../clock.h"

#include "../../Marlin/Configuration.h"
#include "../../Marlin/pins.h"
#include "../../Marlin/fastio.h"

AVRRegistor __reg_map[__REG_MAP_SIZE];
uint8_t avr_simulation_eeprom_storage[4096];

unsigned int __bss_end;
unsigned int __heap_start;
void *__brkval;

extern void TWI_vect();
extern void TIMER0_OVF_vect();
extern void TIMER0_COMPB_vect();
extern void TIMER1_COMPA_vect();

static unsigned int prevTicks = 0;

//After an interrupt we need to set the interrupt flag again, but do this without calling simUpdate so the interrupt does not fire recursively
#define _sei() do { SREG.forceValue(SREG | _BV(SREG_I)); } while(0)

void simUpdate()
{
    if (!(SREG & _BV(SREG_I)))
        return;

    unsigned int ticks = getMilliseconds();
    if (prevTicks == 0)
        prevTicks = ticks;
    int tickDiff = ticks - prevTicks;
    prevTicks = ticks;
    
    //if (tickDiff > 1)
    //    printf("Ticks slow! %i\n", tickDiff);
    if (tickDiff > 0)
    {
        SimBaseComponent::tickAllComponents();
        if (Frontend::instance)
            Frontend::instance->update();
    
        cli();
        //Timer0 is configured to trigger once per ms by Arduino.
        for(int n=0;n<tickDiff;n++)
        {
            if (TIMSK0 & _BV(OCIE0B))
                TIMER0_COMPB_vect();
            if (TIMSK0 & _BV(TOIE0))
                TIMER0_OVF_vect();
        }
        
        //Timer1 runs at 16Mhz / 8 ticks per second.
        unsigned int waveformMode = ((TCCR1B & (_BV(WGM13) | _BV(WGM12))) >> 1) | (TCCR1A & (_BV(WGM11) | _BV(WGM10)));
        unsigned int clockSource = TCCR1B & (_BV(CS12) | _BV(CS11) | _BV(CS10));
        unsigned int tickCount = F_CPU * tickDiff / 1000;
        unsigned int ticks = TCNT1;
        switch(clockSource)
        {
        case 0: tickCount = 0; break;
        case 1: break;
        case 2: tickCount /= 8; break;
        case 3: tickCount /= 64; break;
        case 4: tickCount /= 256; break;
        case 5: tickCount /= 1024; break;
        case 6: tickCount = 0; break;
        case 7: tickCount = 0; break;
        }
        
        if (tickCount > 0 && OCR1A > 0)
        {
            ticks += tickCount;
            while(ticks > int(OCR1A))
            {
                ticks -= int(OCR1A);
                if (TIMSK1 & _BV(OCIE1A))
                    TIMER1_COMPA_vect();
            }
            TCNT1 = ticks;
        }
        _sei();
    }
}

//Assignment opperator called on every register write.
AVRRegistor& AVRRegistor::operator = (const uint32_t v)
{
    uint8_t n = v;
    callback(value, n);
    value = n;
    simUpdate();
    return *this;
}
