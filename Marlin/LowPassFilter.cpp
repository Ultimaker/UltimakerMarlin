#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float rc_constant)
{
    this->output = 0;
    this->rc_constant = rc_constant;
    this->previous_initialized = false;
}

void LowPassFilter::take_input(float value, float dt) {
    if(!this->previous_initialized)
    {
        this->output = value;
        this->previous_initialized = true;
    }

    float ratio = dt / (rc_constant + dt);

    this->output = ratio * value + (1 - ratio) * this->output;
}

float LowPassFilter::get_output()
{
    return this->output;
}
