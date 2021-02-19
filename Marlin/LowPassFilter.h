#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

/**
    Low pass filter (LPF) for floating point values.
    Each time you give this filter a new value it gives a weighted average of the latest output and the new value.
    The RC constant combined with the time difference between the inputs determines the weights of the new and old values.

    See en.wikipedia.org/wiki/Low-pass_filter and en.wikipedia.org/wiki/RC_time_constant for more information.
 */
class LowPassFilter {

private:
    float output;
    float rc_constant;
    bool previous_initialized;

public:

    /**
        Initializes a new LPF with a given RC constant.
        @param rc_constant The constant that together with the time difference determines the weight of the latest value relative to the previous output
     */
    LowPassFilter(float rc_constant);

    /**
     * Takes a new value as input
     * @param value The new value
     * @param dt The time difference between now and the previous call. In seconds.
     */
    void take_input(float value, float dt);

    /**
     * The current output value
     * @return The current output value. 0 if no value was given yet after initializing the filter
     */
    float get_output();
};

#endif
