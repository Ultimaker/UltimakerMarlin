/*
    Copyright (c) 2015-2021 Ultimaker B.V. All rights reserved.

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
/*
   Routines to do z-probing using the capacitive sensor. The bed is raised
   slowly until it hits the nozzle. At that point, the measured values will
   not change as much anymore. The trick is to find this point and then stop
   the motion of the bed. The data does not show a linear progression, as
   the capacitance between two conductors is proportional to the inverse of
   the distance.

   The math involved is documented at the appropriate places in this file.

   NOTE: most of the expressions in this file contain hand-crafted typecasts
   to prevent overflows while maintaining precision. Do not modify any of
   these without extensive testing.
*/

#include "Marlin.h"
#include "i2c_capacitance_FDC1004.h"
#include "cap_sense_probe.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "limits.h"

#define SAMPLE_BUFFER_1_SIZE CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT1
#define SAMPLE_BUFFER_2_SIZE CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT2

// Capacitive sensor samples will always have a bit of noise, but sometimes
// an outlier occurs (due to a bump or electrical noise such as ESD). Such
// samples can be rejected for reliability. They will be replaced by the
// average value of the three preceding samples. REJECT_THRESHOLD defines
// how much a sample is allowed to deviate from the mean value of the sample
// buffer, expressed in variance (square of standard deviation). A good
// starting point is 4 standard deviations, given as a variance of (4 * 4).
// 4 standard deviations may seem rather large, but there is a trend in the
// data (mostly increasing). It would be better to calculate the deviation
// from the trend, but the algorithm does not go that far.
#define REJECT_THRESHOLD (4*4)

// The impact of a rejected sample decreases over time. A single rejection
// should not matter, while a burst is an indication of trouble. The
// following values define when the data is considered to be so unreliable
// that the z-probe should be aborted. A counter is incremented by 1 for
// every rejected sample, while it is multiplied by REJECT_CONSTANT for
// every accepted sample (IIR filter). When the counter value reaches
// REJECT_COUNT_MAX, the run is aborted.
#define REJECT_COUNT_MAX 10
#define REJECT_CONSTANT 0.9

// ERROR_HEIGHT is a safe height that won't break the printer. Opinicus
// homes the bed after this for a retry (a number of times), since a move is
// planned to the detected height after probing (very useful for debugging,
// now it moves to ERROR_HEIGHT before homing which is also very clear
// behaviour indicating an error).
#define ERROR_HEIGHT 150

// buffer index handling (note % behavior for negative numbers)
#define BUFMOD(x) (((x) + SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE) % (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE))
#define RESIDUAL_BUFMOD(x) (((x) + CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE) % (CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE))

// work packet (storing global data)
struct WorkPacket
{
    enum probe_state_t state;
    const char* error_message;
    float z;
    int debuglevel;             // from V option
    // An inflection point is a point on a curve where the curve changes from being concave to convex or vice versa.
    int64_t inflection;
    int64_t inflection_max;    // the maximum inflection so far
    float reject_count;        // nr of rejected samples; decreased for every good sample by multiplication with REJECT_CONSTANT
};

static struct WorkPacket workPacket;

struct stat_t {
    int64_t sum_of_x;	    // sum of x values
    int64_t sum_of_squared_x;  // sum of squares of x values
    int64_t sum_of_xy;	    // sum of xy products
    int64_t sum_of_y;	    // sum of y values
    int64_t sum_of_squared_y;  // sum of squares of y values
};

static int16_t capacitive_baseline = 0;

// add a data point to a statistics struct
static void statPushSample(struct stat_t *stat, int64_t x, int64_t y)
{
    stat->sum_of_x += x;
    stat->sum_of_squared_x += x * x;
    stat->sum_of_xy += x * y;
    stat->sum_of_y  += y;
    stat->sum_of_squared_y += y * y;
}

// Overload of statPushSample for lower precision data
static void statPushSample(struct stat_t *stat, long x, long y)
{
    statPushSample(stat, int64_t(x), int64_t(y));
}

// remove a data point from a statistics struct
static void statPopSample(struct stat_t *stat, int64_t x, int64_t y)
{
    stat->sum_of_x  -= x;
    stat->sum_of_squared_x -= x * x;
    stat->sum_of_xy -= x * y;
    stat->sum_of_y  -= y;
    stat->sum_of_squared_y -= y * y;
}

// Overload of statPopSample for lower precision data
static void statPopSample(struct stat_t *stat, long x, long y)
{
    statPopSample(stat, int64_t(x), int64_t(y));
}

static void processSample(uint16_t sample_value, float sample_z)
{
    // sample counter, also used as abscissa data for regression
    static long counter = -1;

    // data buffers (used as ring buffers)
    static uint16_t cbuf[SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE]; // capacitive data sample buffer
    static float    zbuf[SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE]; // z height buffer.
    static float    residual_buffer[CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE];
    static uint16_t write_pointer;

    // statistics
    static struct stat_t stat1, stat2, stat_residual;
    static float estimated_sample_value;
    static float estimated_sample_velocity;

    // slope of lines determined by linear regression over buffers (see math comment below)
    int64_t a1, a2;
    // inflection slope based scaling logic
    static bool scaled_inflection_triggered_peak;
    static float inflection_scaling;

    // x coordinate of intersection in terms of counter value
    static float intersection_sample_count;

    // x data for new sample (note that counter starts at -1 to allow pre-increment)
    if (workPacket.state == INIT)
        counter = -1; // workPacket.state is re-initialized below

    counter++;

    // initialization
    if (counter == 0)
    {
        workPacket.state = FILLING;
        memset(&stat1, 0, sizeof(struct stat_t));
        memset(&stat2, 0, sizeof(struct stat_t));
        memset(&stat_residual, 0, sizeof(struct stat_t));
        write_pointer = 0;
        workPacket.inflection_max = 0;
        workPacket.reject_count = 0.0;

        estimated_sample_value = (float)sample_value;
        estimated_sample_velocity = 0.0;

        scaled_inflection_triggered_peak = false;
        inflection_scaling = 1.0;
    }

    // anything to do?
    if (workPacket.state == DONE || workPacket.state == ERROR)
        return;

    // Part 1 of the alpha beta filter, estimate what the new sample should be and compute how far off it is from the incoming sample (residual)
    // Because the filter estimates the velocity, the mean of the residuals should show no bias and be around 0
    // A stat_t struct can then be used to determine how far from 0 the average residual lies
    // If an incoming residual lies much further from 0 than the mean, this residual likely belongs to an outlier and should be rejected.
    float next_estimated_sample_value = estimated_sample_value + estimated_sample_velocity / CONFIG_BED_LEVELING_SAMPLE_FREQUENCY;
    float residual = (float)sample_value - next_estimated_sample_value;

    // sample rejection logic -- skip this while filling the residual_buffer
    if (counter > CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE)
    {
        // calculate variance (calculation carefully crafted to maintain precision (casting to float immediately
        // will be more imprecise than dividing as integer and then casting to float) and avoid overflows)
        float residual_variance = float(stat_residual.sum_of_squared_y - stat_residual.sum_of_y * stat_residual.sum_of_y / int64_t(CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE)) / float(CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE);
        // The values in stat_residual are multiplied with CONFIG_BED_LEVELING_RESIDUAL_MULTIPLIER for precision, and the variance is related to the square of the residuals so undo the square of this ratio
        residual_variance = residual_variance / CONFIG_BED_LEVELING_RESIDUAL_MULTIPLIER / CONFIG_BED_LEVELING_RESIDUAL_MULTIPLIER;

        // reject sample if it is an outlier
        if (residual * residual > REJECT_THRESHOLD * residual_variance)
        {
            int16_t sample_value_new;

            // replace aberrant sample with average of last three samples
            sample_value_new = int16_t((cbuf[BUFMOD(write_pointer - 1)] + cbuf[BUFMOD(write_pointer - 2)] + cbuf[BUFMOD(write_pointer - 3)]) / 3.0);

            if (workPacket.debuglevel > 0)
            {
                MSerial.print("# Outlier found: ");
                MSerial.print(sample_value);
                MSerial.print(" -> ");
                MSerial.println(sample_value_new);
            }
            sample_value = sample_value_new;
            workPacket.reject_count++;
            if (workPacket.reject_count > REJECT_COUNT_MAX)
            {
                workPacket.state = ERROR;
                workPacket.error_message = "WARNING:CAPACITIVE_SENSOR_PROBLEM:outlier limit exceeded - aborting";
                return;
            }
        }
        else
        {
            workPacket.reject_count *= REJECT_CONSTANT;
        }
    }

    //Part 2 of alpha beta filter: use the residual to update the estimates and store them for the next loop:
    estimated_sample_value = next_estimated_sample_value + CONFIG_BED_LEVELING_ALPHA * residual;
    estimated_sample_velocity += CONFIG_BED_LEVELING_BETA * residual / CONFIG_BED_LEVELING_SAMPLE_FREQUENCY;

    // update statistics: remove old samples
    if (counter >= SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE)
        statPopSample(&stat1, counter - (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE), cbuf[write_pointer]);
    if (counter >= SAMPLE_BUFFER_2_SIZE)
        statPopSample(&stat2, counter - SAMPLE_BUFFER_2_SIZE, cbuf[BUFMOD(write_pointer - SAMPLE_BUFFER_2_SIZE)]);
    if (counter >= CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE)
    {
        float residual_to_pop = residual_buffer[RESIDUAL_BUFMOD(write_pointer - CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE)];
        statPopSample(&stat_residual, (int64_t)(counter - CONFIG_BED_LEVELING_RESIDUAL_BUFFER_SIZE), (int64_t)residual_to_pop * CONFIG_BED_LEVELING_RESIDUAL_MULTIPLIER);
    }

    // add new sample to buffer, overwriting old value
    cbuf[write_pointer] = sample_value;
    zbuf[write_pointer] = sample_z;
    residual_buffer[RESIDUAL_BUFMOD(write_pointer)] = residual;
    write_pointer = BUFMOD(write_pointer + 1);

    // update statistics: add samples
    if (counter >= SAMPLE_BUFFER_2_SIZE)
        statPushSample(&stat1, counter - SAMPLE_BUFFER_2_SIZE, cbuf[BUFMOD(write_pointer - SAMPLE_BUFFER_2_SIZE - 1)]);
    statPushSample(&stat2, counter, sample_value);
    // Scale the residual to maintain precision when converting it to an int64_t, this has to be undone when popping and using stat_residual for computation
    int64_t rounded_scaled_residual = (int64_t)residual * CONFIG_BED_LEVELING_RESIDUAL_MULTIPLIER;
    statPushSample(&stat_residual, (int64_t)counter, rounded_scaled_residual);
    // check for overflows and abort if necessary
    if (workPacket.state == ERROR)
        return;

    // wait until buffer is filled
    if (counter < SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1)
        return;

    if (counter == SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1)
    {
        if (workPacket.debuglevel > 0)
            MSerial.print("# buffer filled\n");// (%ld)\n", counter);
        workPacket.state = WORKING;
    }

    /*
        Using linear least-squares regression, a (virtual) line is drawn
        through each buffer according to the equation

          y = a * x + b

        where a (slope) and b (intercept) are to be determined. Closed
        expressions exist for a and b:

              S(x*y) - 1/n * S(x) * S(y)   n * S(x*y) - S(x) * S(y)
          a = -------------------------- = ------------------------
               S(x^2) - 1/n * (S(x))^2      n * S(x^2) - (S(x))^2

              1/n * S(y) * S(x^2) - 1/n * S(x) * S(x*y)   S(y) * S(x^2) - S(x) * S(y)
          b = ----------------------------------------- = ---------------------------
                       S(x^2) - 1/n * (S(x))^2               n * S(x^2) - (S(x))^2

        where SAMPLE_BUFFER_1_SIZE and SAMPLE_BUFFER_2_SIZES are equivalent to n and
        S is to be read as sigma, the summation over a buffer, and x
        and y are x_i and y_i, respectively. This is convenient, because it
        allows us to calculate a and b based on streaming data:

          d1 = SAMPLE_BUFFER_1_SIZE * stat1.sum_of_squared_x - stat1.sum_of_x * stat1.sum_of_x;
          d2 = SAMPLE_BUFFER_2_SIZE * stat2.sum_of_squared_x - stat2.sum_of_x * stat2.sum_of_x;
          a1 = (SAMPLE_BUFFER_1_SIZE * stat1.sum_of_xy - stat1.sum_of_x * stat1.sum_of_y) / float(d1);
          a2 = (SAMPLE_BUFFER_2_SIZE * stat2.sum_of_xy - stat2.sum_of_x * stat2.sum_of_y) / float(d2);

        The denominators d1 and d2 represent n * Var(x). For the linear
        counter used here, these values are constant and equal for the two
        buffers when they are full. In this approach, the exact value is:

          n * Var(x) = n * S(x^2) - (S(x))^2 = 1/12 * n^2 * (n^2 - 1)

        For SAMPLE_BUFFER_1_SIZE = SAMPLE_BUFFER_2_SIZE = 50, the
        denominators are 520625. Since we are interested in the location of
        the intersection of the two lines, the denominators (and a good
        number of divisions) can be omitted from the analysis. See the next
        math comment for a proof.

        The numbers involved are large! The final results will usually fit
        in a long, so they can be cast before printing. The intermediate
        values can easily overflow a long, however.
    */

    // calculate slopes
    a1 = SAMPLE_BUFFER_1_SIZE * int64_t(stat1.sum_of_xy) - int64_t(stat1.sum_of_x) * stat1.sum_of_y;
    a2 = SAMPLE_BUFFER_2_SIZE * int64_t(stat2.sum_of_xy) - int64_t(stat2.sum_of_x) * stat2.sum_of_y;

    // inflection at point between SAMPLE_BUFFER_1_SIZE and SAMPLE_BUFFER_2_SIZE in buffer
    int64_t inflection = a1 - a2;
    workPacket.inflection = inflection;
    if (!scaled_inflection_triggered_peak)
    {
        // Inflection scaling logic: If the inflection is large, that does not mean that the the angle between a1 and a2 is actually large
        // If the two lines are co-linear, the ratio a1/a2 will be 1. The further away this ratio is from 1.0, the more obvious the angle.
        // This ratio should never amplify the inflection measurement (the results would be highly unpredictable) so it is limited to 1.0
        inflection_scaling = abs(float(a1) / a2 - 1.0);
        inflection_scaling = min(1, inflection_scaling);
        inflection = inflection_scaling * inflection;
        if (inflection > CONFIG_BED_LEVELING_PEAK_DET1)
            scaled_inflection_triggered_peak = true;
    }


    if (workPacket.inflection > workPacket.inflection_max)
    {
        // intercept (see math comment below)
        int64_t b1, b2;

        /*
            The math to calculate the intersection of two lines is:
              line 1: y1 = a1 * x + b1
              line 2: y2 = a2 * x + b2

              y1 = y2 ->
              a1 * x + b1 = a2 * x + b2 ->
              a1 * x - a2 * x = b2 - b1 ->
              (a1 - a2) * x = b2 - b1 ->
              x = (b2 - b1) / (a1 - a2)

            A common denominator in a and b drops out of the equation.
            The numbers involved can be huge: we're trying not to lose
            accuracy by using integers as long as possible. The difference
            between b2 and b1 will not be as large. The calculation below
            takes less than 150us on a 16-MHz ATmega2560.
        */

        // store inflection
        workPacket.inflection_max = workPacket.inflection;

        // calculate index of sample_z at peak, relative to x data (counter)
        b1 = stat1.sum_of_y * int64_t(stat1.sum_of_squared_x) - stat1.sum_of_x * int64_t(stat1.sum_of_xy);
        b2 = stat2.sum_of_y * int64_t(stat2.sum_of_squared_x) - stat2.sum_of_x * int64_t(stat2.sum_of_xy);
        intersection_sample_count = float(b2 - b1) / float(workPacket.inflection);
    }

    /*
       Two checks must be true before the algorithm determines the bed is found:
       The scaled inflection must exceed CONFIG_BED_LEVELING_PEAK_DET1 and must have dropped since then
       It must drop by the ratio CONFIG_BED_LEVELING_PEAK_DET2N / CONFIG_BED_LEVELING_PEAK_DET2D which should be below 1.0
       Fractional math does not always work nicely with int64_t which is why the fracion is split
    */
    if (scaled_inflection_triggered_peak &&
        workPacket.inflection < CONFIG_BED_LEVELING_PEAK_DET2N * (workPacket.inflection_max / CONFIG_BED_LEVELING_PEAK_DET2D))
    {
        float index;
        int offset, offset1;

        // peak found!
        workPacket.state = DONE;
        if (workPacket.debuglevel > 0)
            MSerial.print("# peak found\n");

        // find corresponding index in z buffer
        index = intersection_sample_count - (float(counter) - (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1));

        // index is offset from write_pointer in circular buffer
        offset  = BUFMOD(write_pointer + (int)index);
        offset1 = BUFMOD(write_pointer + (int)index + 1);

        if (index < 0)
        {
            workPacket.state = ERROR;
            workPacket.error_message = "WARNING:CAPACITIVE_SENSOR_PROBLEM:peak is no longer in buffer!";
            return;
        }

        if (index >= SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE)
        {
            workPacket.state = ERROR;
            workPacket.error_message = "WARNING:CAPACITIVE_SENSOR_PROBLEM:peak is in the future?!";
            return;
        }

        workPacket.z = zbuf[offset] + (zbuf[offset1] - zbuf[offset]) * (index - (int)index);
        if (workPacket.debuglevel > 0)
        {
            MSerial.print("# z ");
            MSerial.println(workPacket.z, 5);
        }

        return;
    }
}

static int16_t captureSample()
{
    int16_t value = 0;
    // This is a different reject count than the one in workPacket.reject_count.
    // This rejects completely bogus samples early on.
    uint8_t reject_count = 0;

    while (reject_count < 50)
    {
        if (i2cCapacitanceDone(value))
        {
            if (value != 0 && value != (int16_t)0xffff)
            {
                return value;
            }
            reject_count++;
        }
        manage_heater();
        manage_inactivity();
    }
    // 50 samples have returned with the same value. Most likely, the sensor
    // is not connected. Return error and stop all motion to prevent the bed
    // from colliding with the head.
    quickStop();

    return 0;
}

static void doTheProbing(const float start_position[], const int feedrate, const float extra_z_move)
{
    int t0 = millis();

    // Loop for as long as the Z-axis is moving.
    while (blocks_queued())
    {
        int t1 = millis();
        int16_t sample_value = captureSample();    // returns 0 on error.
        if (sample_value == 0)
        {
            workPacket.state = ERROR;
            workPacket.error_message = "WARNING:CAPACITIVE_SENSOR_PROBLEM:capacitive sensor malfunction - stopped";
            return;
        }
        /* The algorithm requires small positive numbers. To make the number small we subtract the capacitive_baseline.
           We expect the result to be positive because the capacitive_baseline is determined with the build plate
           far away from the sensor, while this function should start close to the build plate.
           However a sensor that has been hit by an Electro Static Discharge will be less sensitive, so the initial values
           in this function will be very close to those found when determining the capacitive_baseline.
           To make sure that subtracting the baseline doesn't cause negative numbers for ESD compromised sensors we add
           the sensor noise STD as an offset
        */
        sample_value -= capacitive_baseline;
        sample_value += CONFIG_BED_LEVEL_SENSOR_MAX_NOISE_STD;
        // If the sample is still negative after adding the noise STD, this sensor is so bad that we better stop probing
        if (sample_value < 0)
        {
            workPacket.state = ERROR;
            workPacket.error_message = "WARNING:Z_OFFSET_PROBE_FAILED:capacitive sensor value below baseline - stopped";
            return;
        }

        float zf = float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS];

        int t2 = millis();
        if (workPacket.state != CONTINUE)
            processSample(sample_value, zf);    // processSample expects positive input values. Since we checked sample_value to be positive this is true.

        int t3 = millis();
        if (workPacket.debuglevel)
        {
            MSerial.print(zf, 5);
            MSerial.print(" ");
            MSerial.print(sample_value);
            MSerial.print(" ");
            MSerial.print((long)workPacket.inflection);
            MSerial.print(" ");
            MSerial.print(workPacket.reject_count);
            MSerial.print(" ");
            MSerial.print(workPacket.state);
            MSerial.print(" ");
            MSerial.print(t3 - t0);
            MSerial.print(" ");
            MSerial.print(t2 - t1);
            MSerial.print(" ");
            MSerial.println(t3 - t2);
        }

        if (workPacket.state == DONE)
        {
            if (workPacket.debuglevel)
            {
                MSerial.print("# done\n");
            }

            // Was requested to perform extra Z-move after detecting the build plate? (debug/test feature)
            if (extra_z_move > 0)
            {
                workPacket.state = CONTINUE;
                plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], current_position[Z_AXIS] - extra_z_move,
                        current_position[E_AXIS], float(feedrate) / 60, active_extruder);
            }
            else
            {
                return;
            }
        }

        if (workPacket.state == ERROR)
        {
            return;
        }
        t0 = millis();
    }
}

ProbeResult probeWithCapacitiveSensor(const float start_position[], const int feedrate, const int verbosity, const float move_distance, const float extra_z_move)
{
    // Prepare for probing
    stop_heaters_pwm = true;
    manage_heater();
    memset(&workPacket, 0, sizeof(WorkPacket));
    workPacket.debuglevel = verbosity;
    workPacket.z = ERROR_HEIGHT;

    // move to start position
    plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], start_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS], active_extruder);
    st_synchronize();

    disable_x();
    disable_y();
    disable_e0();
    disable_e1();
    disable_e2();

    // A Q&D-hack to figure out if the distorted first few samples are caused by moving mechanics.
    // 20ms should be enough as only the first +/-10 samples are distorted.
    _delay_ms(20);

    // start Z-move
    plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], start_position[Z_AXIS] - move_distance, current_position[E_AXIS], float(feedrate)/60, active_extruder);

    // Do the actual probing.
    doTheProbing(start_position, feedrate, extra_z_move);

    // Restore printer to a defined state
    quickStop();    // Discard any possible movements still going on.

    enable_x();
    enable_y();
    stop_heaters_pwm = false;

    ProbeResult probe_result = ProbeResult();
    probe_result.state = workPacket.state;
    probe_result.z = workPacket.z;
    probe_result.error_message = workPacket.error_message;

    return probe_result;
}

void moveWithCapacitiveSensor(const int feedrate, const float move_distance)
{
    // prepare for move
    stop_heaters_pwm = true;
    manage_heater();

    st_synchronize();

    disable_x();
    disable_y();
    disable_e0();
    disable_e1();
    disable_e2();

    // start Z-move
    plan_buffer_line(current_position[X_AXIS],
                     current_position[Y_AXIS],
                     current_position[Z_AXIS] - move_distance,
                     current_position[E_AXIS], float(feedrate)/60, active_extruder);

    current_position[Z_AXIS] -= move_distance;
    unsigned long t0 = millis();
    while (blocks_queued())
    {
        unsigned long t1 = millis();
        int16_t sample_value = captureSample();
        float zf = float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS];

        MSerial.print(zf, 5);
        MSerial.print(" ");
        MSerial.print(sample_value);
        MSerial.print(" ");
        MSerial.println(t1 - t0);

        t0 = t1;
    }

    enable_x();
    enable_y();
    stop_heaters_pwm = false;
}

/** @brief Capture a sample from the capdac
 *  @param capdac_level The step/level to sample
 *  @return Returns the 3rd sample
 */
static int16_t getCAPDACSample(uint8_t capdac_level)
{
    i2cCapacitanceSetCAPDAC(capdac_level);

    /* throw away the first samples; the CAPDAC needs some time to settle */
    captureSample();
    captureSample();

    return captureSample();
}

/* The static capacitance seen by the sensor may be larger than the
   measurement range (15pF). This may easily happen when ESD protection is
   used. The measured value should not be too large, because it will
   increase when the head is close to the bed. The value may even clip at
   32767 (reading only the 16 most significant bits).

   The FDC1004 contains a so-called CAPDAC which can be enbled to compensate
   for a static capacitance. One step in the CAPDAC setting subtracts
   3.125pF from the measured value. With the reading as used here (bits
   23:8), one step decreases the value by about 6300. The CAPDAC provides 32
   steps (0 = off).

   This function finds the best CAPDAC setting by increasing it until the
   measured capacitance is less than can be offset using the CAPDAC. Make
   sure that the bed is in the lowest position that will be used for
   measuring, otherwise later measurements may give negative results (2's
   complement). The linear regression algorithm cannot deal with negative
   numbers. We don't need to worry about edge cases: in typical situations
   the noise on the sensor is about 25 (peak-peak), while the sensor value
   will increase by about 100 when the bed is raised from the calibration
   height to the starting height for Z probing.
*/
#define CAPDAC_MAX_STEPS 32

void calibrateCapacitanceOffset(int verbosity)
{
    int16_t value;
    uint8_t capdac_level;

    if (verbosity > 0)
    {
        /* debug only: cycle once through all available settings to check out the CAPDAC */
        for (capdac_level = 0; capdac_level < CAPDAC_MAX_STEPS; capdac_level++)
        {
            value = getCAPDACSample(capdac_level);
            MSerial.print("capdac ");
            MSerial.print(capdac_level, DEC);
            MSerial.print(" value ");
            MSerial.println(value, DEC);
        }
    }

    /* This loop attempts to find the maximum CAPDAC that will give a positive result at
       calibration height. It does this by increasing the setting until the measurement is
       below a noise margin, and then decrementing the setting by 1 before returning.
       This decrementing is not done if the setting is already at 0, perfect (future) sensors will use capdac level 0

       This loop can fail in two ways:

       - The measurement result is negative even without the compensating effect of the
         CAPDAC. In this case it is most likely that the FDC1004 is broken, probably due to
         ESD. Replace the printhead PCB and check again. Make sure to observe ESD-safe
         working practices. The FDC1004 is very sensitive.

       - The measurement cannot be brought into range even with the highest CAPDAC setting.
         This indicates faulty wiring. Check that the capactive sensor wires are connected
         correctly. Make sure that the sensor board is in place and that the front fan wires
         do not make contact with the fan bracket. Observe ESD-safe working practices.

       Note that the fan bracket is connected to the SHIELD pin of the FDC1004, not to
       ground. The front fan should not make electrical contact with the fan bracket.
    */
    for (capdac_level = 0; capdac_level < CAPDAC_MAX_STEPS; capdac_level++)
    {
        value = getCAPDACSample(capdac_level);

        /* continue until the measured capacitance is below 5 sigma noise (99.9999% reliable), then back up if we can*/
        if (value < 5 * CONFIG_BED_LEVEL_SENSOR_MAX_NOISE_STD)
        {
            if (value < 0 && capdac_level == 0)
            {
                /* the measured capacitance is already negative?! */
                MSerial.println("ERROR:CAPACITIVE_SENSOR_PROBLEM:negative capacitance without compensation");
                return;
            }
            if (capdac_level > 0)
            {
                --capdac_level;
            }
            value = getCAPDACSample(capdac_level);
            MSerial.print("capdac ");
            MSerial.print(capdac_level, DEC);
            MSerial.print(" value ");
            MSerial.println(value, DEC);
            return;
        }
    }

    /* the loop should never complete */
    MSerial.println("ERROR:CAPACITIVE_SENSOR_PROBLEM:cannot compensate static capacitance (sensor shorted?)");
}

void updateCapacitiveSensorBaseline(int verbosity)
{
    int32_t total_cap_value = int32_t(captureSample()) + captureSample() + captureSample();
    capacitive_baseline = total_cap_value / 3;

    if (verbosity > 0)
    {
        MSerial.print("capacitive_baseline ");
        MSerial.println(capacitive_baseline, DEC);
    }
}
