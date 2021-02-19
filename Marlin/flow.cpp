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
#include "flow.h"
#include "stepper.h"

#define SENSOR_WHEEL_DIAMETER               9       // Diameter in mm.
#define SENSOR_RESOLUTION                   4096    // The sensor's angle resolution. Where 4096 steps are one full revolution.
#define SENSOR_WHEEL_CIRCUM                 (SENSOR_WHEEL_DIAMETER * M_PI)
#define DEFAULT_ALLOWED_FLOW_ERROR_FACTOR   0.8     // The default allowed error in extruded volume

#define NO_SENSOR_DATA                      (UINT16_MAX)

FlowA1335 flowSensor[MAX_A1335_SENSOR_COUNT] = {0, 1, 2, 3};

Flow::Flow()
  : previous_e_position_in_steps(0)
  , current_e_position_in_steps(0)
  , allowed_flow_error_factor(DEFAULT_ALLOWED_FLOW_ERROR_FACTOR)
  , is_e_pos_reset(false)
  , current_state( STAGE_TRIGGER_NEW_SAMPLING )
{
}

void Flow::init()
{
    bool error_raised = false; 
    for (uint8_t sensor=0; sensor < NR_OF_FLOW_SENSORS; sensor++)
    {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("FILAMENT_FLOW_SENSOR_INIT:");
        MSerial.print(sensor, DEC);
        SERIAL_ECHOLNPGM(" initializing...");
        if (!initializeSensor(sensor))
        {
            SERIAL_ECHOPGM("WARNING:FILAMENT_FLOW_SENSOR_INIT:");
            MSerial.print(sensor, DEC);
            SERIAL_ECHOLNPGM(":Unable to initialize sensor");
            error_raised = true; 
        }
    }
    if (!error_raised)
    {
        initFlowData();
        setStepsPerMm(axis_steps_per_unit[E_AXIS]);
    }
}

void Flow::resetStateMachine()
{
    current_state = STAGE_TRIGGER_NEW_SAMPLING;
}

bool Flow::initializeSensor(uint8_t sensor_nr)
{
    if (flowSensor[sensor_nr].isDiscovered())
    {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Discovering flow sensor A1335 #");
        MSerial.println(sensor_nr, DEC);

        if (flowSensor[sensor_nr].init() == ERROR_SUCCESS)
        {
            return true;
        }
    }
    SERIAL_ECHO_START
    SERIAL_ECHOPGM("A1335 init failed for sensor #");
    MSerial.println(sensor_nr, DEC);
    return false;
}

void Flow::initFlowData()
{
    for (uint8_t sensor = 0; sensor < NR_OF_FLOW_SENSORS; sensor++)
    {
        initFlowBuckets(sensor);
    }

    previous_e_position_in_steps = st_get_position(E_AXIS);
}

void Flow::initFlowBuckets(uint8_t sensor)
{
    window_pointer[sensor] = 0;
    for (uint8_t bucket = 0; bucket < NR_OF_FLOW_BUCKETS; bucket++)
    {
        initFlowBucket(&flow_buckets[sensor][bucket]);
    }
    flow_buckets[sensor][0].last_sensor_angle = flowSensor[sensor].getAngleWait();      // Set initial angle position
}

void Flow::initFlowBucket(FlowBucket* bucket_ptr)
{
    bucket_ptr->last_sensor_angle = NO_SENSOR_DATA;
    bucket_ptr->last_e_position = st_get_position(E_AXIS);
    bucket_ptr->angle_change = 0;
    bucket_ptr->e_position_change = 0;
}

void Flow::setStepsPerMm(float stepsPerMm)
{
    sample_interval_e_steps = fabs(stepsPerMm * SAMPLE_INTERVAL_MM);
    bucket_size_e_steps = fabs(stepsPerMm * FLOW_BUCKET_SIZE_MM);
    flow_error_minimum_e_steps = fabs(stepsPerMm * MINIMUM_FLOW_ERROR_DISTANCE_MM);
}

void Flow::setAllOutputRates(uint8_t output_rate)
{
    for (uint8_t sensor_nr = 0; sensor_nr < MAX_A1335_SENSOR_COUNT; sensor_nr++)
    {
        if (flowSensor[sensor_nr].isSensorPresent())
        {
            flowSensor[sensor_nr].setOutputRate(output_rate);
        }
    }
}

uint16_t Flow::getSensorRaw(uint8_t sensor_nr)
{
    return getCurrentBucketPointer(sensor_nr)->last_sensor_angle;
}

int32_t Flow::getExtrusionPosition(uint8_t sensor_nr)
{
    return getCurrentBucketPointer(sensor_nr)->last_e_position;
}

int32_t Flow::getAngleChange(uint8_t sensor_nr)
{
    return getCurrentBucketPointer(sensor_nr)->angle_change;
}

int32_t Flow::getExtrusionPositionChange(uint8_t sensor_nr)
{
    return getCurrentBucketPointer(sensor_nr)->e_position_change;
}

void Flow::setExtrusionPosition(int32_t e_pos_in_steps)
{
    if (e_pos_in_steps == 0)
    {
        is_e_pos_reset = true;
    }

    // State machine stores a local copy of the stepper position
    // and therefore needs to be reset
    current_e_position_in_steps = e_pos_in_steps;
    previous_e_position_in_steps = e_pos_in_steps;
    resetStateMachine();

    // Update the flow sensing data
    updateExtrusionPosition();
}

// This function should be called at frequent intervals during printing (the more often is better for higher accuracy).
void Flow::update()
{
    static uint8_t sensor_nr;           // Made this a static so we save the 'sensor's number at start of sensor read' for use in next stages.

    // Reading the angle sensor for 1 sensor takes 0.53ms, longer for more sensors. We don't want to be blocking that
    // long, hence the introduction of a state machine where we first start the I2C reading in the background and
    // process the results later on.
    switch(current_state)
    {
        case STAGE_TRIGGER_NEW_SAMPLING:    // Is it time to start new sampling?
        {
            if (areAllSensorsReady())
            {
                current_e_position_in_steps = st_get_position(E_AXIS);
                sensor_nr = active_extruder;

                // In order to reduce the system load we will only be sampling every 1mm3 of extruded material.
                if (abs(current_e_position_in_steps - previous_e_position_in_steps) >= sample_interval_e_steps)
                {
                    startUpdatingAllSensors();
                    current_state = STAGE_READ_SENSOR_DATA;
                }
            }
            break;
        }
        case STAGE_READ_SENSOR_DATA:    // Wait for flow sensor data to arrive
        {
            if (areAllSensorsReady())
            {
                updateExtrusionPosition(sensor_nr, flowSensor[sensor_nr].getRawAngle(), current_e_position_in_steps);
                current_state = STAGE_TRIGGER_NEW_SAMPLING;
            }
            break;
        }
    }
}

void Flow::updateExtrusionPosition()
{
    uint16_t angle = flowSensor[active_extruder].getAngleWait();    // Note: blocking call to get the sensor position, not optimal.
    updateExtrusionPosition(active_extruder, angle, st_get_position(E_AXIS));
}

void Flow::updateExtrusionPosition(uint8_t sensor_nr, uint16_t current_angle_raw, int32_t current_e_pos)
{
    if (current_angle_raw == NO_SENSOR_DATA) // No point in measuring flow in case of a read time-out
    {
        return;
    }

    FlowBucket* flow_bucket = getCurrentBucketPointer(sensor_nr);
    int16_t previous_angle = flow_bucket->last_sensor_angle;
    int16_t actual_moved_steps = current_e_pos - previous_e_position_in_steps;
    bool has_flow_error = false;

    // Have we reached the trigger level to start a new bucket?
    if (flow_bucket->e_position_change >= bucket_size_e_steps)
    {
        flow_bucket = getNewBucket(sensor_nr);

        // Flow problems are not checked for the current bucket because it might contain a retract.
        // So, we only have to verify flow problems when a new bucket is created.
        CalculatedFlowData flowData = calculateFlowData(sensor_nr);
        has_flow_error = hasFlowError(flowData);
        if (has_flow_error)
        {
            sendFlowError(sensor_nr, flowData);

            // Reset the error counters.
            initFlowBuckets(sensor_nr);
        }
    }

    // Add new data to the current bucket.
    flow_bucket->last_sensor_angle = current_angle_raw;
    flow_bucket->last_e_position = current_e_pos;
    flow_bucket->angle_change += calculateAngleChange(previous_angle, current_angle_raw);
    flow_bucket->e_position_change += actual_moved_steps;

    previous_e_position_in_steps = current_e_pos;

    log(has_flow_error);
}

int16_t Flow::calculateAngleChange(uint16_t previous_angle, uint16_t current_angle)
{
    // Calculate the raw angle differences taking into account that the magnetic sensor might move through the zero position.
    // We assume the angle sensor didn't move more than 1/2 revolution from the previous time we got here.
    int16_t angle_change = current_angle - previous_angle;
    if (angle_change > SENSOR_RESOLUTION / 2)
    {
        angle_change -= SENSOR_RESOLUTION;
    }
    else if (angle_change < -SENSOR_RESOLUTION / 2)
    {
        angle_change += SENSOR_RESOLUTION;
    }

    return angle_change;
}

void Flow::sendFlowError(uint8_t sensor_nr, CalculatedFlowData flowData)
{
    SERIAL_ECHOPGM("WARNING:FILAMENT_FLOW:");
    SERIAL_ECHO(uint16_t(sensor_nr));
    SERIAL_ECHO(":");
    debugDump();
    SERIAL_ECHO(" : Total flow angle: ");
    SERIAL_ECHO(flowData.raw_angle_total);
    SERIAL_ECHO(", ");
    SERIAL_ECHO("Total steps: ");
    SERIAL_ECHO(flowData.e_steps_total);
    SERIAL_ECHO(", ");
    SERIAL_ECHO("Flow sensor distance: ");
    SERIAL_ECHO(flowData.flow_sensor_distance);
    SERIAL_ECHO(", ");
    SERIAL_ECHO("Stepper distance: ");
    SERIAL_ECHO(flowData.stepper_distance);
    SERIAL_ECHO(", ");
    SERIAL_ECHO("Distance difference: ");
    SERIAL_ECHO(flowData.distance_difference);
    SERIAL_ECHO(", ");
    SERIAL_ECHO("Max allowed difference: ");
    SERIAL_ECHO(flowData.maximum_allowed_difference);
    MSerial.println();
}

FlowBucket* Flow::getNewBucket(uint8_t sensor_nr)
{
    // Start the next flow data bucket.
    FlowBucket* flow_bucket = advanceBucketPointer(sensor_nr);
    initFlowBucket(flow_bucket);
    return flow_bucket;
}

Flow::CalculatedFlowData Flow::calculateFlowData(uint8_t sensor_nr)
{
    CalculatedFlowData flowData;
    flowData.raw_angle_total = 0;
    flowData.e_steps_total = 0;
    flowData.flow_sensor_distance = 0.0;
    flowData.stepper_distance = 0.0;

    // Count the total number of steps and magnetic sensor revolutions.
    for (uint8_t i = 0; i < NR_OF_FLOW_BUCKETS; i++)
    {
        FlowBucket *current_ptr = &flow_buckets[sensor_nr][i];
        flowData.raw_angle_total += current_ptr->angle_change;
        flowData.e_steps_total += current_ptr->e_position_change;
    }

    // Convert total counts to distances.
    // Note: could have used scaled integers here instead of float but speed isn't critical and this is easier reading.
    flowData.flow_sensor_distance = float(flowData.raw_angle_total * SENSOR_WHEEL_CIRCUM) / SENSOR_RESOLUTION;
    flowData.stepper_distance = flowData.e_steps_total / axis_steps_per_unit[E_AXIS];
    flowData.distance_difference = flowData.stepper_distance - flowData.flow_sensor_distance;
    flowData.maximum_allowed_difference = fabs(flowData.stepper_distance * allowed_flow_error_factor);

    return flowData;
}

bool Flow::hasFlowError(const CalculatedFlowData& flowData)
{
    // Quit when the minimum detection distance hasn't been reached yet
    if (flowData.e_steps_total < flow_error_minimum_e_steps)
    {
        return false;
    }

    // We only check under extrusion, over extrusion is ignored.
    if (flowData.distance_difference > flowData.maximum_allowed_difference )
    {
        return true;
    }

    return false;
}

void Flow::debugDump()
{
    for (uint8_t i=0; i < NR_OF_FLOW_BUCKETS; i++)
    {
        FlowBucket* bucket_ptr = &flow_buckets[active_extruder][i];

        // Quit loop when this bucket has no data yet.
        if (bucket_ptr->last_sensor_angle == NO_SENSOR_DATA)
        {
            break;
        }

        SERIAL_ECHO(" ");
        SERIAL_ECHO(bucket_ptr->last_sensor_angle);
        SERIAL_ECHO("/");
        SERIAL_ECHO(bucket_ptr->last_e_position);
        SERIAL_ECHO("/");
        SERIAL_ECHO(bucket_ptr->angle_change);
        SERIAL_ECHO("/");
        SERIAL_ECHO(bucket_ptr->e_position_change);
    }
}

FlowBucket* Flow::advanceBucketPointer(uint8_t sensor_nr)
{
    uint8_t* window_ptr = &window_pointer[sensor_nr];
    if (*window_ptr >= NR_OF_FLOW_BUCKETS - 1)
    {
        *window_ptr = 0;
    }
    else
    {
        (*window_ptr)++;
    }

    return getCurrentBucketPointer(sensor_nr);
}

FlowBucket* Flow::getCurrentBucketPointer(uint8_t sensor_nr)
{
    return &flow_buckets[sensor_nr][window_pointer[sensor_nr]];
}

// For debugging we will log a special tagged ('FLOW@') raw debug string that Opinicus can write to a file.
void Flow::log(bool has_flow_error)
{
    SERIAL_ECHOPGM("FLOW@");            // Start of debug message
    SERIAL_ECHOPGM("E");                // Log active extruder
    MSerial.print(active_extruder, DEC);

    for (uint8_t sensor_nr = 0; sensor_nr < NR_OF_FLOW_SENSORS; sensor_nr++)
    {
        SERIAL_ECHOPGM(", ");
        SERIAL_ECHO("W");
        MSerial.print(window_pointer[sensor_nr], DEC);
        SERIAL_ECHOPGM(", ");
        SERIAL_ECHOPGM("S");
        MSerial.print(sensor_nr, DEC);
        SERIAL_ECHOPGM(", ");
        MSerial.print(getSensorRaw(sensor_nr), DEC);
        SERIAL_ECHOPGM(", ");
        MSerial.print(getExtrusionPosition(sensor_nr), DEC);
        SERIAL_ECHOPGM(", ");
        MSerial.print(getAngleChange(sensor_nr), DEC);
        SERIAL_ECHOPGM(", ");
        MSerial.print(getExtrusionPositionChange(sensor_nr), DEC);
    }

    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO(is_e_pos_reset);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO(has_flow_error);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
    SERIAL_ECHOLN("");

    is_e_pos_reset = false;
}

void Flow::startUpdatingAllSensors()
{
    for (uint8_t sensor_nr = 0; sensor_nr < MAX_A1335_SENSOR_COUNT; sensor_nr++)
    {
        if (flowSensor[sensor_nr].isSensorPresent())
        {
            flowSensor[sensor_nr].start();
        }
    }
}

bool Flow::areAllSensorsReady()
{
    for (uint8_t sensor_nr = 0; sensor_nr < MAX_A1335_SENSOR_COUNT; sensor_nr++)
    {
        if (flowSensor[sensor_nr].isSensorPresent() && flowSensor[sensor_nr].isBusy())
        {
            return false;
        }
    }

    return true;
}
