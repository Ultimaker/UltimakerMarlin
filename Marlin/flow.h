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
#ifndef FLOW_H
#define FLOW_H

#include "FlowBase.h"
#include "Configuration.h"
#include "flow_A1335.h"


extern FlowA1335 *flowSensor[];

struct FlowBucket
{
    uint16_t    last_sensor_angle;  // The last known sensor angle position (angle is in binary units where 4096 steps are one full revolution).
    int32_t     last_e_position;    // The last known E_axis position that matches the magnetic sensor position above [steps].
    int16_t     angle_change;       // The angle change (angle is in binary units where 4096 steps are one full revolution).
    int16_t     e_position_change;  // The E position change [steps]
};

enum FlowSensorVersion
{
    FLOW_SENSOR_NONE,
    FLOW_SENSOR_VERSION_1,
    FLOW_SENSOR_VERSION_2
};

struct FlowSensorSettings
{
    float       minimum_extrusion_factor;      // The lower extrusion factor limit before triggering EOF
    float       maximum_extrusion_factor;      // The upper extrusion factor limit before triggering EOF
};

#define FILAMENT_DIAMETER                   2.85
#define FILAMENT_RADIUS                     (FILAMENT_DIAMETER / 2)
#define MM3_IN_1MM_OF_FILAMENT              (M_PI * FILAMENT_RADIUS * FILAMENT_RADIUS)
#define MM_OF_FILAMENT_FOR_1MM3             (1 / MM3_IN_1MM_OF_FILAMENT)
// This is a define and not a static class variable because a non integer type can't be a const in one compiler and it can't be a constexpr in the other compiler...
#define SENSOR_WHEEL_DIAMETER 9.0

#define SAMPLE_INTERVAL_MM                  0.2 // Number of mm extruder movement before we take a new sample [mm]
#define FLOW_BUCKET_SIZE_MM                 1.0 // Size of a flow bucket [mm]
#define NR_OF_FLOW_BUCKETS                  10  // Number of flow sample buckets.
#define MINIMUM_FLOW_ERROR_DISTANCE_MM      9.0 // Minimum distance that must have been extruded before we start reporting extrusion errors. [mm]

class Flow : public FlowBase
{
public:
    // @brief   Constructor.
    Flow();

    // @brief   Initialize the flow module.
    //          The system is scanned for flow sensors at known I2C addresses and discovered sensors will be initialized.
    virtual void init();

    // @brief  Initialize all flow position variables.
    virtual void initFlowData();

    // @brief   Configures the number of feeder steps required to move the filament 1mm.
    // @param   stepsPerMm is the number of steps to move the filament 1mm [float].
    virtual void setStepsPerMm(float stepsPerMm);

    // @brief Sets the minimal extrusion as a factor of the desired extrusion
    // @param sensor_index The index of the sensor to set the factor for.
    // @param factor Factor to use.
    virtual void setMinimumExtrusionFactor(uint8_t sensor_index, float factor);

    // @brief   Set output rate (averaging) for all flow sensors.
    // @param   output_rate configures the refresh rate of the angle data, i.e. how many samples are averaged before output.
    //          0 =   1 sample,   31.25 μs update
    //          1 =   2 samples,  62.5 μs update
    //          2 =   4 samples, 125 μs update
    //          3 =   8 samples, 250 μs update
    //          4 =  16 samples, 500 μs update
    //          5 =  32 samples, 1 ms update
    //          6 =  64 samples, 2 ms update
    //          7 = 128 samples, 4 ms update
    virtual void setAllOutputRates(uint8_t output_rate);

    // @brief   Get the raw angle sensor value for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The raw angle sensor value.
    virtual uint16_t getSensorRaw(uint8_t sensor_nr);

    // @brief   Get the last E-axis position (in steps) for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The E-axis position in steps.
    virtual int32_t getExtrusionPosition(uint8_t sensor_nr);

    // @brief   Get the angle sensor position change for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The angle sensor position compared to the previous storage location.
    virtual int32_t getAngleChange(uint8_t sensor_nr);

    // @brief   Get the extrusion position change for the specified sensor number.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  The number of steps changed compared to the previous storage location.
    virtual int32_t getExtrusionPositionChange(uint8_t sensor_nr);

    // @brief   Update the position tracking and flow sensor data.
    //          This function should be called at frequent intervals during printing (the more often is better for higher accuracy).
    //          In order to reduce system load this function will only do something when more than 1mm3 has been extruded.
    virtual uint8_t update();

    // @brief   Update extrusion flow position for active extruder. Calling this function impacts
    //          the flow detection algorithm, to prevent the algorithm from working with old data
    //          the state machine needs to be reset.
    // @param   e_pos_in_steps is the number of feeder steps you want to set. Used when there is a sudden E-change without actual
    //          physical movement, for example after a G92 (Set current position) command.
    virtual void setExtrusionPosition(int32_t e_pos_in_steps);

    // @brief   Update extrusion flow position for active extruder.
    //          Retrieves active feeder position and sensor position (blocking call, so not optimal)
    virtual void updateExtrusionPosition();

    static const int16_t SENSOR_RESOLUTION = 4096;    // The sensor's angle resolution. Where 4096 steps are one full revolution.
    const float SENSOR_WHEEL_CIRCUM;
    static const uint16_t NO_SENSOR_DATA = UINT16_MAX;

private:
    int16_t    bucket_size_e_steps;         // minimum size of a bucket in extruder steps
    int16_t    sample_interval_e_steps;     // minimum of extruder steps before we take a new sample
    int16_t    flow_error_minimum_e_steps;  // Minimum of extruder steps before starting to test for flow errors

    // @brief   A structure with consolidated flow data values
    struct CalculatedFlowData
    {
        int16_t  raw_angle_total;
        int16_t  e_steps_total;
        float    flow_sensor_distance;
        float    stepper_distance;
        float    extrusion_factor;
    };

    // State machine states
    enum SamplingState
    {
        STAGE_TRIGGER_NEW_SAMPLING,
        STAGE_READ_SENSOR_DATA
    };

    struct FlowBucket   flow_buckets[NR_OF_FLOW_SENSORS][NR_OF_FLOW_BUCKETS];   // These buckets track data from angle position in relation to flow sensor angle over time.
    struct FlowSensorSettings sensor_settings[NR_OF_FLOW_SENSORS];

    uint8_t     window_pointer[NR_OF_FLOW_SENSORS];         // The buckets are organized in a ring buffer. This window pointer indicates the current active bucket.
    int32_t     previous_e_position_in_steps;               // The e_position at the end of the previous bucket (== the start of this bucket). Tracked as a global i.s.o. per extruder since Marlin only tracks one E position.
    int32_t     current_e_position_in_steps;                // The e_position value that is matched to a flow sensor value
    bool        is_e_pos_reset;                             // Used in flow data logging. Tracking the logged data makes more sense when there is an indication for the Gcode resetting the extrusion position.

    FlowSensorVersion sensor_version; // The version of the flow sensors.
    SamplingState current_state; // The current state of the internal state machine

    // @brief   Detect the indicated flow sensor and identify the version.
    // @param   sensor_nr is the sensor index [0-1].
    // @return  Returns Enum indicating the sensor version or undetected.
    FlowSensorVersion detectFlowSensor(uint8_t sensor_nr);

    // @brief   Reset the internal state machine, this is important because the update function has a locally scoped
    //          variable tracking the e-stepper position. This value could be invalid before it is used when the extrusion
    //          position is set explicitly.
    void resetStateMachine();

    // @brief   Try to discover the indicated sensor on the I2C bus and when discovered the sensor will be initialized.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  Returns True when the sensor could be initialized
    bool initializeSensor(uint8_t sensor_nr);

    // @brief   Initialize all the flow position buckets for the specified sensor.
    // @param   sensor_nr is the sensor number [0-3].
    void initFlowBuckets(uint8_t sensor_nr);

    // @brief   Initialize the flow position bucket for the specified sensor.
    // @param   bucket_ptr is a pointer to the bucket to be initialized.
    void initFlowBucket(FlowBucket* bucket_ptr);

    // @brief   Sum the flow buckets and calculate the distances traveled
    // for both the flow sensor and the stepper
    // @param   sensor_nr is the sensor number [0-3].
    // @return  A structure with consolidated flow data values.
    CalculatedFlowData calculateFlowData(uint8_t sensor_nr);

    // @brief   Returns true when there is a flow error.
    // @param   flowData is a structure with consolidated flow data values.
    // @return  True when there is a flow error.
    bool hasFlowError(uint8_t sensor_nr, const CalculatedFlowData& flowData);

    // @brief   Dump a lot of debugging data
    void debugDump();

    // @brief   Update extrusion flow position for active extruder.
    // @param   sensor_nr is the sensor number [0-3].
    // @param   current_angle_raw is the current sensor angle [0-4095].
    // @param   current_e_pos  is the current E position in steps.
    virtual void updateExtrusionPosition(uint8_t sensor_nr, uint16_t current_angle_raw, int32_t current_e_pos);

    // @brief   Calculate the raw angle change between to measured angle values [-2048 to +2047].
    //          This function is limited in that it can handle maximum 1/2 revolution between the values to compare.
    // @param   previous_angle is the previous angle you want to compare with.
    // @param   current_angle_raw is the current measured angle value.
    // @return  The angle difference.
    static int16_t calculateAngleChange(uint16_t previous_angle, uint16_t current_angle_raw);

    // @brief   Sends the flow error code/message to the external print controller.
    // @param   sensor_nr is the sensor number [0-3].
    // @param   flowData is a structure with consolidated flow data values.
    void sendFlowError(uint8_t sensor_nr, CalculatedFlowData flowData);

    // @brief   Gets a new flow position storage bucket for tracking data.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  A pointer to the new storage bucket.
    FlowBucket* getNewBucket(uint8_t sensor_nr);

    // @brief   Advance the bucket pointer to the next storage bucket.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  A pointer to the new storage bucket.
    FlowBucket*  advanceBucketPointer(uint8_t sensor_nr);

    // @brief   Get a pointer to the current storage bucket for the specified sensor.
    // @param   sensor_nr is the sensor number [0-3].
    // @return  A pointer to the current storage bucket.
    FlowBucket* getCurrentBucketPointer(uint8_t sensor_nr);

    // @brief   This function writes a special tagged ('FLOW@') raw debug string that Opinicus can write to a file.
    // @param   has_flow_error is to be set when the algorithm detected a flow error.
    void log(bool has_flow_error);

    // @brief   Start sampling all flow sensors.
    void startUpdatingAllSensors();

    // @brief   Returns true when all flow sensors are ready with new data.
    // @return  True when all sensors have new data.
    bool areAllSensorsReady();
};

#endif // FLOW_H
