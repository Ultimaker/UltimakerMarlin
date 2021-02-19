#include "heater.h"
#include "arduinoIO.h"
#include "../frontend/frontend.h"
#include "../clock.h"

#include "../../Marlin/thermistortables.h"

HeaterSim::HeaterSim(int heaterPinNr, float heater_strength)
{
    this->heaterPinNr = heaterPinNr;
    this->adc = adc;
    this->heater_strength = heater_strength;
    
    this->temperature_core = 20;
    this->temperature_sensor = 20;
    
    last_tick_count = getMilliseconds();
}

HeaterSim::~HeaterSim()
{
}

void HeaterSim::tick()
{
    unsigned long now = getMilliseconds();
    float delta = float(now - last_tick_count) / 1000.0f;
    last_tick_count = now;

    heater_output = float(readAnalogOutput(heaterPinNr)) / 255.0f;
    
    temperature_core -= temperature_decrease_on_no_heat_per_second * delta;
    temperature_core += (temperature_increase_on_full_heat_per_second * heater_strength + temperature_decrease_on_no_heat_per_second) * heater_output * delta;
    
    if (temperature_core < 22)
        temperature_core += (22 - temperature_core) * delta;
    
    temperature_sensor += (temperature_core - temperature_sensor) * delta / measuring_delay;
}

void HeaterSim::draw(int x, int y)
{
    char buffer[32];
    sprintf(buffer, "%.1fC %.1fC %i%%", temperature_core, temperature_sensor, int(heater_output * 100));
    Frontend::instance->drawString(x, y, buffer, 0xFFFFFF);
}

void HeaterSim::adcReadout(int max_value, int& output)
{
    if (max_value == 255) /* Avr ADC */
    {
        output = 231 + temperature_sensor * 81 / 100;//Not accurate, but accurate enough.
    }
    if (max_value == 0x7FFF) /* ADS101X ADC */
    {
#if (THERMISTORHEATER_0 == 21) || (THERMISTORHEATER_1 == 21) || (THERMISTORHEATER_2 == 21) || (THERMISTORBED == 21)
        for(unsigned int n=0; n<(sizeof(temptable_21)/sizeof(temptable_21[0]))-1; n++)
        {
            if (temptable_21[n+1][1] > temperature_sensor)
            {
                int raw = temptable_21[n][0] + (temptable_21[n+1][0] - temptable_21[n][0]) * (temperature_sensor - temptable_21[n][1]) / (temptable_21[n+1][1] - temptable_21[n][1]);
                output = raw / OVERSAMPLENR;
                return;
            }
        }
        output = temptable_21[0][0] / OVERSAMPLENR;
#endif
    }
}
