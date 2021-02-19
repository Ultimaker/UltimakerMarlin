#ifndef HEATER_SIM_H
#define HEATER_SIM_H

#include "base.h"
#include "adc.h"

class HeaterSim : public SimBaseComponent
{
public:
    HeaterSim(int heaterPinNr, float heater_strength = 1.0);
    virtual ~HeaterSim();
    
    virtual void tick();
    virtual void draw(int x, int y);
    
    void adcReadout(int max_value, int& output);
private:
    static constexpr float temperature_increase_on_full_heat_per_second = 12.0 / 5.0; /* this was measured over 5 second interval */
    static constexpr float temperature_decrease_on_no_heat_per_second = 4.0 / 5.0; /* this was measured over 5 second interval */
    static constexpr float measuring_delay = 5.0f;

    unsigned long last_tick_count;
    float temperature_core;
    float temperature_sensor;
    float heater_strength;
    float heater_output;

    int heaterPinNr;
    AvrAdcSim* adc;
};

#endif//I2C_SIM_H
