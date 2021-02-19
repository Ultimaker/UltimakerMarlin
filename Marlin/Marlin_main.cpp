/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 */
#define  __STDC_LIMIT_MACROS        // Required to get UINTxx_MAX macros to work in stdint.h

#include "Marlin.h"

#include <stdint.h>
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "flow.h"
#include "motion_control.h"
#include "watchdog.h"
#include "language.h"
#include "i2c_driver.h"
#include "i2c_capacitance_FDC1004.h"
#include "pca9635_driver.h"
#include "led_rgbw_pca9632.h"
#include "fan_driver.h"
#include "cap_sense_probe.h"
#include "i2c_onewire_ds2482.h"
#include "stepper_A4988.h"
#include "stepper_TMC2130.h"
#include "usart_spi_driver.h"
#include "oneWireDS2431_eeprom.h"
#include "SerialProtocol.h"
#include "Board.h"
#include "GainCompensatorFactory.h"

//Add the compile date and time to the begining of the .text section.
// What does this mean? It means the compile date and time is added at the start of code in the flash.
// Why do we do this? It makes it easier to check if the current firmware is the one we are expecting.
// As we only need to check if the begining of the hex file matches the flash in the system.
char compile_date_time[] __attribute__((section(".vectors"))) __attribute__((used)) = __DATE__ __TIME__;

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G28 - Home all Axis
// G30 - Probe Z at current position and report result.
// G31 - get/update capacitive sensor base level
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Get Firmware Version and Capabilities
// M140 - Set bed target temp
// M142 - Set system lights (M142 r[0-255] g[0-255] b[0-255] w[0-255])
// M143 - Set hotend light (M143 r[0-255] g[0-255] b[0-255] T[0-1])
// M144 - Set allow hotend removal (M144 T[0-1])
// M145 - Clear allow hotend removal (M145 T[0-1])
// M149 - Read hotend SERIAL (M149 T[0-1])
// M150 - Read raw hotend eeprom page as hex data. (M150 T[0-1] P[0-3])
// M151 - Write Raw hotend data per 8 bytes. (M151 T[0-1] A<register address> D<hex data(16 characters/nibbles)>)
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2.
// M205 - Advanced settings: minimum travel speed S=while printing, T=travel only, B=minimum segment time [us], X=maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - Set additional homing offset
// M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y> Z<offset_on_Z>
// M220 S<factor in percent> - Set speed factor override percentage
// M221 S<factor in percent> - Set extrude factor override percentage
// M290 P<powerBudget> I<idle_power_consumption>
// M291 R<nominal_bed_resistance> A<bed_resistance_per_degree> V<bed_voltage>
// M292 T<hotend_nr> R<nominal_hotend_cartridge_resistance> V<hotend_slot_voltage> P<max_hotend_power>
// M301 - Set nozzle PID parameters FF, P, I, D, i, C and R (Q: Debug dump enable/disable)
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>
// M304 - Set bed PID parameters FF, P, I, D, i, C and R (Q: Debug dump enable/disable)
// M310 - Single cap sensor read.
// M311 - calculate cap sensor noise
// M312 - reset capacitance chip
// M313 - re-init capacitance chip
// M400 - Finish all moves
// M401 - Quickstop - Abort all the planned moves. This will stop the head mid-move, so most likely the head will be out of sync with the stepper position after this
// M405 - Enable/disable the flow sensor hardware. S1=active A=averaging value
// M406 - Re-init flow sensor algorithm
// M407 - Read raw flow sensor value: S<sensor number>
// M907 - Set digital trimpot motor current using axis codes. Current in mA. X-axis sets Y-axis as well. (M907 X1300 Z1300 E1250)
// M998 - Intentionally stop the system as if by an error.
// M999 - Restart after being stopped by error
// M12000 - Set build volume maximum position (M12000 X200 Y200 Z200)
// M12001 - Set build volume minimum position (M12000 X0 Y0 Z0)
//          The minimum and maximum position combined define the build volume for the printer. They also define the default homing coordinates.
// M12003 - Get board type
// M12004 - Set the direction of each axis. M12004 X[0/1] Y[0/1] Z[0/1], To set direction per extruder: M12004 T[0/1] E[0/1]
// M12005 - Set the extruder temperature compensation
// M12006 - Set the build plate temperature compensation
// M12010 - Read/Write TMC2130 stepper driver registers. R[register index] (S[driver index]) (V[new register value]). A driver index of 0xFF will address all drivers.


//===========================================================================
//=============================public variables=============================
//===========================================================================
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = { false, false, false, false };
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply = 100;//ARRAY_BY_EXTRUDERS(100, 100, 100); //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homing[NUM_AXIS-1] = {0,0,0};
float min_pos[NUM_AXIS-1] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[NUM_AXIS-1] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
long extruder_offset[NUM_AXIS-1][EXTRUDERS];
#endif
uint8_t active_extruder = 0;
uint8_t target_fan_speed=0;

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0;

static const char bin_to_hex[] = "0123456789abcdef";

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static uint8_t reported_plan_buf_free_positions = 0;

/** @brief The sequence numbers received per gcode line, needed for ACK to sender */
static uint8_t gcodeline_sequence_number[BUFSIZE];

static char command_buffer[BUFSIZE][MAX_CMD_SIZE];
static uint8_t command_buffer_index_read = 0;
static uint8_t command_buffer_index_write = 0;
static uint8_t command_buffer_length = 0;

static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

static uint8_t tmp_extruder;

// We have a 'dummy flow' and a 'real flow' object to make it easy to change between machines with and without flow sensors
// and not having a lot of if-statements to test for the module being present or not.
Flow real_flow;
FlowBase dummy_flow;
FlowBase *flow = &dummy_flow;

#ifdef ENABLE_WATCHDOG_SERIAL_INPUT
static unsigned long start_time_empty_command_buffer = 0;

static void check_serial_input_timeout();
#endif

/* Define at what (>=) temperature threshold the Case FANs should be turned on, any lower temperature will turn them off */
#define CASE_FANS_MINIMAL_TEMP_THRESHOLD 50.0

#define PULSE_OFF 0
#define PULSE_2_COLORS 1
#define PULSE_ONLY_ONE_COLOR 2
static uint8_t pulse[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(PULSE_OFF);
static uint8_t blink_time = 5; // ms delay for each blink a blink consists of 2* 255
static uint8_t first_color[EXTRUDERS][3];
static uint8_t second_color[EXTRUDERS][3];

#define M12010_ALL_DRIVERS  255     // M12010 command parameter to address all motor drivers

//===========================================================================
//======================= PRIVATE ROUTINE DECLARATIONS ======================
//===========================================================================
static bool is_command_queued();
static void process_command();
static void get_coordinates();
static void get_arc_coordinates();
static void prepare_move();
static void prepare_arc_move(char isclockwise);
static int bin2hex(char* hex, const uint8_t* bin, int count);
static int hex2bin(uint8_t* bin, const char* hex, const int buf_size);
static bool setTargetedHotend();
static void pulseLeds();

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

/** @brief The callback function to be executed when a packet has been read
 *  @param packet The network packet received from the serial line
 *  @return Returns True if the packet could be processed and added to the command buffer
 */
bool handle_packet(SerialProtocol::network_packet_t packet);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

static bool is_command_queued()
{
    return command_buffer_length > 0;
}

static char* code_value_ptr()
{
    return strchr_pointer + 1;
}

static float code_value()
{
    return strtod(code_value_ptr(), NULL);
}

static long code_value_long()
{
    return strtol(code_value_ptr(), NULL, 10);
}

static bool code_seen(char code)
{
  strchr_pointer = strchr(command_buffer[command_buffer_index_read], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

/** @brief Detects and initializes the board.
 *  @return Returns true when board is initialized succesfully, otherwise false
 */
bool initBoard()
{
    Board::detect();      // Detect the PCB revision. Must be called before temperatureInit() to have ADC access.
    uint8_t result = Board::init();
    if (result > 0)
    {
        stop(result);
        return false;
    }
    return true;
}

void setup()
{
    MSerial.begin(BAUDRATE);

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("BUILD:" __DATE__ " " __TIME__);

    watchdog_init();
    st_init();    // Initialize stepper, this will automatically disable the steppers
    initBoard();
    if (Board::getId() == Board::BOARD_2621B)
    {
        UsartSpiDriver::init();
        StepperTMC2130::init();
    }
    StepperA4988::init(); // Set the default PWM for the steppers
    i2cDriverInit();
    initFans();
    initPCA9635();
    temperatureInit();
    plan_init();  // Initialize planner
    st_enable_interrupt(); // this enables interrupts!
#ifdef ENABLE_BED_LEVELING_PROBE
    i2cCapacitanceInit();
#endif
    i2cOneWireInit();
    ledRGBWInit();

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Marlin started");
}

void loop()
{
    SerialProtocol::readCommand(handle_packet);

#ifdef ENABLE_WATCHDOG_SERIAL_INPUT
  check_serial_input_timeout();
#endif
  if (reported_plan_buf_free_positions == 0) {
    uint8_t  free_positions = plan_buf_free_positions();

    if (free_positions) {
      reported_plan_buf_free_positions = free_positions;
      SerialProtocol::sendResponse('q', 0, free_positions);
    }
  }

  if (is_command_queued())
  {
    process_command();
    if (is_command_queued())
    {
      command_buffer_length--;
      command_buffer_index_read = (command_buffer_index_read + 1) % BUFSIZE;
    }
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
}

bool handle_packet(SerialProtocol::network_packet_t packet)
{
    if (command_buffer_length >= BUFSIZE)
    {
        // No space, sorry,
        return false;
    }

    memcpy((void*)command_buffer[command_buffer_index_write], (void*)packet.payload, packet.payload_size);
    command_buffer[command_buffer_index_write][packet.payload_size] = 0; //terminate string
    gcodeline_sequence_number[command_buffer_index_write] = packet.sequence_number;

    if (code_seen('G'))
    {
        switch (code_value_long())
        {
            case 0:
            case 1:
            case 2:
            case 3:
                if (isStopped())
                { // If printer is stopped by an error the G[0-3] codes are ignored.
                    SERIAL_ECHOLNPGM(MSG_ERR_STOPPED);
                }
                break;
            default:
                break;
        }
    }

    command_buffer_index_write = (command_buffer_index_write + 1) % BUFSIZE;
    command_buffer_length++;

    return true;
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis) {
    if (home_dir(axis) < 0)
        current_position[axis] = min_pos[axis] + add_homing[axis];
    else
        current_position[axis] = max_pos[axis] + add_homing[axis];
}

// Move the given axis to the home position.
// Movement is in two parts:
// 1) A quick move until the endstop switch is reached.
// 2) A short backup and then slow move to home for accurately determining the home position.
static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))
  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {

    // Do a fast run to the home position.
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 1.5 * (max_pos[axis] - min_pos[axis]) * home_dir(axis);
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    long home_dist_steps_quick_run = st_get_position(axis);
    if (!isEndstopHit())
    {
        if (axis == Z_AXIS)
        {
            current_position[axis] = 0;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            destination[axis] = -home_retract_mm(axis) * home_dir(axis) * 10.0;
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            st_synchronize();

            stop(STOP_REASON_Z_ENDSTOP_BROKEN_ERROR);
        }else{
            stop(STOP_REASON_XY_ENDSTOP_BROKEN_ERROR);
        }
        return;
    }

    // Move back a little bit.
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * home_dir(axis);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // Verify the endstop switch isn't stuck.
    bool endstop_pressed = false;
    switch(axis)
    {
    case X_AXIS:
        #if defined(X_MIN_PIN) && X_MIN_PIN > -1 && X_HOME_DIR == -1
        endstop_pressed = (READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
        #endif
        #if defined(X_MAX_PIN) && X_MAX_PIN > -1 && X_HOME_DIR == 1
        endstop_pressed = (READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
        #endif
        break;
    case Y_AXIS:
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1 && Y_HOME_DIR == -1
        endstop_pressed = (READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
        #endif
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1 && Y_HOME_DIR == 1
        endstop_pressed = (READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
        #endif
        break;
    case Z_AXIS:
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1 && Z_HOME_DIR == -1
        endstop_pressed = (READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
        #endif
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1 && Z_HOME_DIR == 1
        endstop_pressed = (READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
        #endif
        break;
    }
    if (endstop_pressed)
    {
        if (axis == Z_AXIS)
        {
            stop(STOP_REASON_Z_ENDSTOP_STUCK_ERROR);
        }else{
            stop(STOP_REASON_XY_ENDSTOP_STUCK_ERROR);
        }
        endstops_hit_on_purpose();
        return;
    }

    // Do a second run at the home position, but now slower to be more accurate.
    destination[axis] = 2*home_retract_mm(axis) * home_dir(axis);
    feedrate = homing_feedrate[axis]/3;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    long home_dist_steps_total = home_dist_steps_quick_run + st_get_position(axis);
    float h_dist = (float(home_dist_steps_total) / axis_steps_per_unit[axis]) * home_dir(axis);
    MSerial.print("home_distance = ");
    MSerial.println(h_dist, 5);

    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

// Process 1 gcode command line.
static void process_command()
{
  if(code_seen('G'))
  {
    switch(code_value_long())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(!isStopped()) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
      }
      break;
    case 2: // G2  - CW ARC
      if(!isStopped()) {
        get_arc_coordinates();
        prepare_arc_move(true);
      }
      break;
    case 3: // G3  - CCW ARC
      if(!isStopped()) {
        get_arc_coordinates();
        prepare_arc_move(false);
      }
      break;
    case 28: // G28 - Home all Axes one at a time
      float saved_feedrate;
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;

      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      #if defined(QUICK_HOME)
      if(home_all_axis)
      {
        current_position[X_AXIS] = 0; current_position[Y_AXIS] = 0; current_position[Z_AXIS] = 0;

        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

        destination[X_AXIS] = 1.5 * (max_pos[X_AXIS] - min_pos[X_AXIS]) * X_HOME_DIR;
        destination[Y_AXIS] = 1.5 * (max_pos[Y_AXIS] - min_pos[Y_AXIS]) * Y_HOME_DIR;
        destination[Z_AXIS] = 1.5 * (max_pos[Z_AXIS] - min_pos[Z_AXIS]) * Z_HOME_DIR;
        feedrate = homing_feedrate[X_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        st_synchronize();
        endstops_hit_on_purpose();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        axis_is_at_home(Z_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        destination[Z_AXIS] = current_position[Z_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        current_position[Z_AXIS] = destination[Z_AXIS];
      }
      #endif
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      #if defined(QUICK_HOME)
      if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
      {
        current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = 1.5 * (max_pos[X_AXIS] - min_pos[X_AXIS]) * X_HOME_DIR;
        destination[Y_AXIS] = 1.5 * (max_pos[Y_AXIS] - min_pos[Y_AXIS]) * Y_HOME_DIR;
        feedrate = homing_feedrate[X_AXIS];
        if(homing_feedrate[Y_AXIS]<feedrate)
          feedrate =homing_feedrate[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        current_position[Z_AXIS] = destination[Z_AXIS];
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS]=code_value()+add_homing[0];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS]=code_value()+add_homing[1];
        }
      }

      if(code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value()+add_homing[2];
        }
      }
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      endstops_hit_on_purpose();
      break;
#ifdef ENABLE_BED_LEVELING_PROBE
    case 30: // G30 - Probe Z at current position and report result.
      {
        int probe_feedrate = 48; // speed (mm/min)
        int verbosity = 1;  // verbosity (default 1)
        float move_distance = CONFIG_BED_LEVELING_Z_MOVE_DISTANCE;  // move distance
        float extra_z_move = 0; // extra z measurement distance for testing (used during debugging)

        if (code_seen('F')) probe_feedrate = code_value_long();
        if (code_seen('V')) verbosity = code_value_long();
        if (code_seen('D')) move_distance = code_value();
        if (code_seen('Z')) extra_z_move = code_value();

        ProbeResult probe_result = probeWithCapacitiveSensor(destination, probe_feedrate, verbosity, move_distance, extra_z_move);

        if (probe_result.state == ERROR)
        {
            MSerial.println(probe_result.error_message);
        }
        else
        {
          destination[Z_AXIS] = probe_result.z;
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], homing_feedrate[Z_AXIS], active_extruder);
          MSerial.println(destination[Z_AXIS], 5);
        }
      }
      break;
    case 31: // G31 - get/update capacitive sensor base level
      {
        int verbosity = 0;

        if (code_seen('V')) verbosity = code_value_long();

        i2cCapacitanceReset();
        i2cCapacitanceInit();
        calibrateCapacitanceOffset(verbosity);
        updateCapacitiveSensorBaseline(verbosity);
      }
      break;
#endif
    case 90: // G90 - Use Absolute Coordinates
      relative_mode = false;
      break;
    case 91: // G91 - Use Relative Coordinates
      relative_mode = true;
      break;
    case 92: // G92 - Set current position to coordinates given
      {
        for (int8_t i=0; i < NUM_AXIS; i++)
        {
          if (code_seen(axis_codes[i]))
          {
            current_position[i] = code_value();
            if (i == E_AXIS)
            {

              plan_set_e_position(current_position[E_AXIS]);

              flow->setExtrusionPosition(st_get_position(E_AXIS));   // Update the E position after the position jump.
            }
            else
            {
              plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]); // Calls st_synchronize()
            }
          }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
    case 17: // M17  - Enable/Power all stepper motors
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;
    case 42: // M42 - Change pin status via gcode
    {
        if (code_seen('S'))
        {
            int pin_pwm_value = code_value();
            int pin_number = -1;
            if (code_seen('P') && pin_pwm_value >= 0 && pin_pwm_value <= 255)
            {
                pin_number = code_value();
            }
            if (pin_number > -1)
            {
                pinMode(pin_number, OUTPUT);
                analogWrite(pin_number, pin_pwm_value);
            }
        }
        break;
    }
    case 142:   //M142 Set system LEDs
      {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        uint8_t w = 0;
        if (code_seen('r'))
          r = code_value();
        if (code_seen('g'))
          g = code_value();
        if (code_seen('b'))
          b = code_value();
        if (code_seen('w'))
          w = code_value();
        ledRGBWUpdate(r, g, b, w);
#if LED_PIN > -1
        if (Board::getId() != Board::BOARD_2621B)   // 2621 board uses this pin for the case fan
        {
            analogWrite(LED_PIN, w);
        }
#endif
      }
      break;
    case 143:   //M143 Set head LEDs
      {
        if (code_seen('B'))
          blink_time = code_value();

        uint8_t t = 0;
        if (code_seen('T'))
          t = code_value();
        if (t > EXTRUDERS - 1)
            break;

        if (code_seen('r'))
          first_color[t][0] = code_value();
        if (code_seen('g'))
          first_color[t][1] = code_value();
        if (code_seen('b'))
          first_color[t][2] = code_value();
        if (code_seen('Q'))
          second_color[t][0] = code_value();
        if (code_seen('R'))
          second_color[t][1] = code_value();
        if (code_seen('S'))
          second_color[t][2] = code_value();
        if (code_seen('P'))
          pulse[t] = code_value();
        if (pulse[t] == PULSE_OFF)
        {
          setPCA9635led(t, first_color[t][0], first_color[t][1], first_color[t][2]);
        }
      }
      break;
    case 144: // M144 - Set allow hotend removal
      if (code_seen('T')) setAllowHotendRemoval(code_value(), true);
      break;
    case 145: // M145 - Clear allow hotend removal
      if (code_seen('T')) setAllowHotendRemoval(code_value(), false);
      break;
    case 149: // M149 T[0-1] - Read hotend SERIAL
      {
        uint8_t index = 0;
        uint8_t serial[DS2431_SERIAL_SIZE];
        if (code_seen('T') && code_value()) index = 1;
        if (oneWireDS2431ReadSerial(index, serial))
        {
          SERIAL_ECHOPGM("SERIAL:");
          for(uint8_t n = 0; n < DS2431_SERIAL_SIZE; n++)
          {
            if (serial[n] < 16)
              MSerial.print('0');
            MSerial.print(serial[n], HEX);
          }
          MSerial.println();
        }
        else
        {
          SERIAL_ECHOLNPGM("NO-HOTEND");
        }
      }
      break;
    case 150: // M150 T[0-1] P[0-3] - Read raw hotend eeprom page as hex data.
      {
        uint8_t data[DS2431_PAGE_SIZE];
        char hex_data[DS2431_PAGE_SIZE*2+1];
        uint8_t index = 0;
        uint8_t page = 0;
        hex_data[DS2431_PAGE_SIZE*2] = '\0'; // add \0 at end of string
        if (code_seen('T') && code_value()) index = 1;
        if (code_seen('P')) page = code_value();
        if (page >= DS2431_NR_OF_PAGES)
        {
          SERIAL_ECHOLNPGM("PAGE INDEX OUT OF RANGE");
          break;
        }

        if (oneWireDS2431Read(index, page * sizeof(data), data, sizeof(data)))
        {
          bin2hex(hex_data, data, sizeof(data));
          SERIAL_ECHOPGM("DATA: 0x");
          MSerial.println(hex_data);
        }
        else
        {
          SERIAL_ECHOLNPGM("NO-HOTEND");
        }
      }
      break;
    case 151: // M151 T[0-1] A<register address> D<hex data(16 characters/nibbles)> - Write Raw hotend data per 8 bytes.
    {
        uint8_t index = 0;
        uint8_t chunk[DS2431_CHUNK_SIZE];
        uint8_t address = 0;
        char* hex_data = NULL;
        if (code_seen('T') && (code_value() == 1 || code_value() == 0))
        {
            index = code_value();
        }
        else
        {
            SERIAL_ECHOLNPGM("WARNING: NO TOOL PARAMETER FOUND");
            break;
        }
        if (code_seen('A'))
        {
            address = code_value();
        }
        else
        {
            SERIAL_ECHOLNPGM("WARNING: NO ADDRESS PARAMETER FOUND");
            break;
        }
        if (code_seen('D'))
        {
          hex_data = code_value_ptr();
        }
        else
        {
            SERIAL_ECHOLNPGM("WARNING: NO DATA PARAMETER FOUND");
            break;
        }

        if (hex2bin(chunk, hex_data, sizeof(chunk)) == sizeof(chunk) * 2)
        {
            SERIAL_ECHOLNPGM("WRITING");
            if (oneWireDS2431WritePage(index, address, chunk))
            {
                SERIAL_ECHOLNPGM("DATA_WRITTEN");
            }
            else
            {
                SERIAL_ECHOLNPGM("ERROR: DATA_NOT WRITTEN");
            }
        }
        else
        {
            SERIAL_ECHOLNPGM("WARNING: NO (OR NOT ENOUGH) DATA FOUND");
        }
        break;
    }
    case 104: // M104 - Set extruder target temp
      if(setTargetedHotend()){
        break;
      }
      if (code_seen('S'))
      {
        if (!allow_temperature_sensor_errors[tmp_extruder])
          hotend_pid[tmp_extruder].setTargetTemperature(code_value());
      }
      break;
    case 140: // M140 - Set bed target temp
      if (code_seen('S'))
      {
        float bed_temp = code_value();
        heated_bed_pid.setTargetTemperature(bed_temp);
        if (bed_temp >= CASE_FANS_MINIMAL_TEMP_THRESHOLD )
        {
          setCaseFanSpeed(255);
        }
        else
        {
          setCaseFanSpeed(0);
        }
      }
      break;
    case 105 : // M105 - Read current temp
        for(tmp_extruder=0; tmp_extruder<EXTRUDERS; tmp_extruder++)
        {
            SERIAL_ECHOPGM(" T");
            SERIAL_ECHO(int(tmp_extruder));
            SERIAL_ECHO(':');
            SERIAL_ECHO_F(degHotend(tmp_extruder), 1);
            SERIAL_ECHO('/');
            SERIAL_ECHO_F(hotend_pid[tmp_extruder].getTargetTemperature(), 1);
            SERIAL_ECHO('@');
            if (heater_output_accumulator_counter[tmp_extruder] > 0)
            {
                SERIAL_ECHO(heater_output_accumulator[tmp_extruder] / heater_output_accumulator_counter[tmp_extruder]);
                heater_output_accumulator[tmp_extruder] = 0;
                heater_output_accumulator_counter[tmp_extruder] = 0;
            }
            else
            {
                SERIAL_ECHO(int(pwr.getActualHeaterOutput(tmp_extruder)));
            }
            SERIAL_ECHO('p');
            SERIAL_ECHO(hotend_present[tmp_extruder]);
            SERIAL_ECHO('e');
            SERIAL_ECHO(!allow_temperature_sensor_errors[tmp_extruder]);
            SERIAL_ECHO('f');
            SERIAL_ECHO(flow->getSensorRaw(tmp_extruder));
            SERIAL_ECHO('/');
            SERIAL_ECHO(flow->getExtrusionPosition(tmp_extruder));
        }
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        SERIAL_ECHOPGM(" B");
        SERIAL_ECHO_F(degBed(), 1);
        SERIAL_ECHO('/');
        SERIAL_ECHO_F(heated_bed_pid.getTargetTemperature(), 1);
        SERIAL_ECHO('@');
        if (bed_heater_output_accumulator_counter > 0)
        {
            SERIAL_ECHO(bed_heater_output_accumulator / bed_heater_output_accumulator_counter);
            bed_heater_output_accumulator = 0;
            bed_heater_output_accumulator_counter = 0;
        }
        else
        {
            SERIAL_ECHO(int(pwr.getActualBedOutput()));
        }
        #endif //TEMP_BED_PIN
      break;
      case 106: // M106 - Fan on
        if (code_seen('S'))
        {
          target_fan_speed = constrain(code_value(), 0, 255);
        }
        else
        {
          target_fan_speed = 255;
        }
        break;
      case 107: // M107 - Fan off
        target_fan_speed = 0;
        break;

      case 80: // M80  - Turn on Power Supply (ATX Power On)
        Board::powerUp();
        break;

      case 81: // M81  - Turn off Power Supply (ATX Power Off)
        st_synchronize();
        Board::powerDown();
        break;

    case 82: // M82  - Set E codes absolute (default)
      axis_relative_modes[3] = false;
      break;
    case 83: // M83  - Set E codes relative while in Absolute Coordinates (G90) mode
      axis_relative_modes[3] = true;
      break;
    case 18: // Compatibility: M18  - Disable all stepper motors; same as M84
    case 84: // M84  - Disable steppers until next move
      {
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 92: // M92 - Set axis_steps_per_unit - same syntax as G92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if (i == E_AXIS) {
            float value = code_value();
            axis_steps_per_unit[i] = value;
            flow->setStepsPerMm(value);
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      reset_acceleration_rates();
      break;
    case 114: // M114 - Output current position to serial port
      SERIAL_ECHOPGM("X:");
      SERIAL_ECHO(current_position[X_AXIS]);
      SERIAL_ECHOPGM("Y:");
      SERIAL_ECHO(current_position[Y_AXIS]);
      SERIAL_ECHOPGM("Z:");
      SERIAL_ECHO(current_position[Z_AXIS]);
      SERIAL_ECHOPGM("E:");
      SERIAL_ECHO(current_position[E_AXIS]);

      SERIAL_ECHOPGM(" count X:");
      SERIAL_ECHO(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_ECHOPGM("Y:");
      SERIAL_ECHO(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_ECHOPGM("Z:");
      SERIAL_ECHO(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_ECHOLN("");
      break;
    case 115: // M115 - Get Firmware Version and Capabilities
        SERIAL_ECHOPGM(" MACHINE_TYPE:UM3");
        SERIAL_ECHOPGM(" PCB_ID:");
        SERIAL_ECHO(int(Board::getId()));
        SERIAL_ECHOLNPGM(" BUILD:\"" __DATE__ " " __TIME__ "\"");

        break;
    case 119: // M119 - Output Endstop status to serial port
        //SERIAL_ECHOLN(MSG_M119_REPORT);
#if (X_MIN_PIN > -1)
        SERIAL_ECHOPGM(MSG_X_MIN);
        SERIAL_ECHO(((READ(X_MIN_PIN) ^ X_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
#if (X_MAX_PIN > -1)
        SERIAL_ECHOPGM(MSG_X_MAX);
        SERIAL_ECHO(((READ(X_MAX_PIN) ^ X_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
#if (Y_MIN_PIN > -1)
        SERIAL_ECHOPGM(MSG_Y_MIN);
        SERIAL_ECHO(((READ(Y_MIN_PIN) ^ Y_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
#if (Y_MAX_PIN > -1)
        SERIAL_ECHOPGM(MSG_Y_MAX);
        SERIAL_ECHO(((READ(Y_MAX_PIN) ^ Y_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
#if (Z_MIN_PIN > -1)
        SERIAL_ECHOPGM(MSG_Z_MIN);
        SERIAL_ECHO(((READ(Z_MIN_PIN) ^ Z_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
#if (Z_MAX_PIN > -1)
        SERIAL_ECHOPGM(MSG_Z_MAX);
        SERIAL_ECHO(((READ(Z_MAX_PIN) ^ Z_ENDSTOPS_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
        SERIAL_ECHO(" ");
#endif
        SERIAL_ECHOLN("");
        break;
    case 201: // M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2.
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: // M205 - Advanced settings: minimum travel speed S=while printing, T=travel only, B=minimum segment time [us], X=maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 - Set additional homing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homing[i] = code_value();
      }
      break;
    #if EXTRUDERS > 1
    case 218: // M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y> Z<offset_on_Z>
    {
      // calculate the offsets in steps so there is no accumulation of floating point errors.
      if(setTargetedHotend()){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmp_extruder] = -lround(code_value() * axis_steps_per_unit[X_AXIS]);
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = -lround(code_value() * axis_steps_per_unit[Y_AXIS]);
      }
      if(code_seen('Z'))
      {
        extruder_offset[Z_AXIS][tmp_extruder] = -lround(code_value() * axis_steps_per_unit[Z_AXIS]);
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(float(extruder_offset[X_AXIS][tmp_extruder])/axis_steps_per_unit[X_AXIS]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(float(extruder_offset[Y_AXIS][tmp_extruder])/axis_steps_per_unit[Y_AXIS]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(float(extruder_offset[Z_AXIS][tmp_extruder])/axis_steps_per_unit[Z_AXIS]);
      }
      SERIAL_ECHOLN("");
    }break;
    #endif
    case 220: // M220 S<factor in percent> - Set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value();
      }
    }
    break;
    case 221: // M221 S<factor in percent> - Set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        extrudemultiply = code_value();
      }
    }
    break;

    case 290: // M290 P<total_power_budget> I<idle_power_consumption>
    {
        if (code_seen('P')) pwr.setTotalPowerBudget(code_value());
        if (code_seen('I')) pwr.setIdlePowerConsumption(code_value());
    }
    break;
    case 291: // M291 R<nominal_bed_resistance> A<bed_resistance_per_degree> V<bed_voltage>
    {
        if (code_seen('R')) pwr.setNominalBedResistance(code_value());
        if (code_seen('A')) pwr.setBedResistancePerDegree(code_value());
        if (code_seen('V')) pwr.setBedVoltage(code_value());
    }
    break;
    case 292: // M292 T<hotend_nr> R<nominal_hotend_cartridge_resistance> V<hotend_slot_voltage> P<max_hotend_power>
    {
        if (setTargetedHotend())
            break;
        if (code_seen('R')) pwr.setNominalHotendResistance(tmp_extruder, code_value());
        if (code_seen('V')) pwr.setHotendVoltage(tmp_extruder, code_value());
        if (code_seen('P')) pwr.setMaxPowerUsageForHeater(tmp_extruder, code_value());
    }
    break;
    case 301: // M301 - Set nozzle PID parameters FF, P, I and D
      {
        if (setTargetedHotend())
            break;
        if (code_seen('F')) hotend_pid[tmp_extruder].setKff(code_value());
        if (code_seen('P')) hotend_pid[tmp_extruder].setKp(code_value());
        if (code_seen('I')) hotend_pid[tmp_extruder].setKi(code_value());
        if (code_seen('D')) hotend_pid[tmp_extruder].setKd(code_value());
        if (code_seen('i')) hotend_pid[tmp_extruder].setKiMax(code_value());
        if (code_seen('R')) hotend_pid[tmp_extruder].setFunctionalRange(code_value());
        if (code_seen('C')) hotend_pid[tmp_extruder].setKpcf(code_value());
        if (code_seen('Q')) hotend_pid[tmp_extruder].setDebugDump(code_value() > 0);
      }
      break;
    case 304: // M304 - Set bed PID parameters FF, P, I and D
      {
        if (code_seen('F')) heated_bed_pid.setKff(code_value());
        if (code_seen('P')) heated_bed_pid.setKp(code_value());
        if (code_seen('I')) heated_bed_pid.setKi(code_value());
        if (code_seen('D')) heated_bed_pid.setKd(code_value());
        if (code_seen('i')) heated_bed_pid.setKiMax(code_value());
        if (code_seen('R')) heated_bed_pid.setFunctionalRange(code_value());
        if (code_seen('C')) heated_bed_pid.setKpcf(code_value());
        if (code_seen('Q')) heated_bed_pid.setDebugDump(code_value() > 0);
      }
      break;
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // M302 - Allow cold extrudes, or set the minimum extrude S<temperature>
    {
      float temp = .0;
      if (code_seen('S'))
      {
          temp = code_value();
      }
      set_extrude_min_temp(temp);
    }
    break;
    #endif
#ifdef ENABLE_BED_LEVELING_PROBE
    case 310: // M310 - Single cap sensor read.
      {
        uint16_t count = 1;
        if (code_seen('S'))
          count = code_value();

        for (uint16_t i = 0; i < count; i++) // 16 bits because 1000 samples should fit
        {
          int16_t value;
          while(!i2cCapacitanceDone(value))
          {
            manage_heater();
            manage_inactivity();
          }
          MSerial.print(value);
          MSerial.print(' ');
          MSerial.println(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS], 4);
        }
      }
    break;
    case 311: // M311, calculate cap sensor noise
      {
        uint16_t sample_count = 10000;
        if (code_seen('S'))
        {
            sample_count = code_value();
        }
        int16_t min_value = INT16_MAX;
        int16_t max_value = INT16_MIN;
        int32_t average = 0;

        for (uint16_t n = 0; n < sample_count; n++)
        {
            int16_t value;
            while (!i2cCapacitanceDone(value))
            {
                manage_heater();
                manage_inactivity();
            }
            average += value;
            if (value < min_value)
                min_value = value;
            if (value > max_value)
                max_value = value;
        }
        average /= sample_count;

        uint32_t std_dev = 0;   // Always positive, so can be unsigned.
        for (uint16_t n = 0; n < sample_count; n++)
        {
            int16_t value;
            while (!i2cCapacitanceDone(value))
            {
                manage_heater();
                manage_inactivity();
            }

            int32_t deviation_from_average = (average - value);
            std_dev += deviation_from_average * deviation_from_average;
        }
        std_dev /= sample_count;

        SERIAL_ECHOPGM("Average: ");
        MSerial.println(average);
        SERIAL_ECHOPGM("Min: ");
        MSerial.println(min_value);
        SERIAL_ECHOPGM("Max: ");
        MSerial.println(max_value);
        SERIAL_ECHOPGM("Stddev^2: ");
        MSerial.println(std_dev);
      }
    break;
    case 312: // M312 reset capacitance chip
        i2cCapacitanceReset();
    break;
    case 313: // M313 re-init capacitance chip
        i2cCapacitanceInit();
    break;
    case 314: // M314 move while measuring wih capacitive sensor
    {
        float move_distance = 0;
        int move_feedrate = 0;
        if (code_seen('Z')) move_distance = code_value();
        if (code_seen('F')) move_feedrate = code_value();

        moveWithCapacitiveSensor(move_feedrate, move_distance);
    }
#endif//ENABLE_BED_LEVELING_PROBE
    case 400: // M400 - Finish all moves
    {
      st_synchronize();
    }
    break;
    case 401: // M401 - Quickstop - Abort all the planned moves. This will stop the head mid-move, so most likely the head will be out of sync with the stepper position after this
    {
      quickStop();
      // quickStop synchronizes current position with the stepper position.
      // so this reports the position we stopped at back to opinicus, nozzle switching during printing should not
      // be a problem as the switching procedure is unabortable.
      SERIAL_ECHOPGM("X");
      MSerial.print(current_position[X_AXIS], 3);
      SERIAL_ECHOPGM(" Y");
      MSerial.print(current_position[Y_AXIS], 3);
      SERIAL_ECHOPGM(" Z");
      MSerial.print(current_position[Z_AXIS], 3);
      SERIAL_ECHOPGM(" E");
      MSerial.println(current_position[E_AXIS], 3);
    }
    break;
    case 405: //M405 - Set flow sensor feature enable/disable
    {
        if (code_seen('S'))
        {
            if (code_value())
            {
                flow = &real_flow;
            }
            else
            {
                flow = &dummy_flow;
            }
            flow->init();
        }
        if (code_seen('A'))
        {
            flow->setAllOutputRates(code_value_long());
        }
    }
    break;
    case 406: // M406 - Re-init flow sensor algorithm
    {
            flow->initFlowData();
    }
    break;
    case 407: //M407 - Read flow sensor raw value
    {
        if (code_seen('S'))
        {
            uint8_t nr = code_value();
            MSerial.println(flowSensor[nr].getAngleWait());
        }
    }
    break;
    case 907: // M907 - Set digital trimpot motor current using axis codes
    {
        if(code_seen('X')) StepperA4988::setCurrent(0, code_value());
        if(code_seen('Z')) StepperA4988::setCurrent(1, code_value());
        if(code_seen('E')) StepperA4988::setCurrent(2, code_value());
    }
    break;
    case 998: // M998 - Intentionally stop the system as if by an error.
    {
        stop(STOP_REASON_GCODE);
    }
    break;
    case 999: // M999 - Restart after being stopped by error
    {
        // TODO: EM-1379 [New] - Add powerup in clearStopReason?
        clearStopReason();
    }
    break;
    case 12000: // M12000 - Set build volume maximum size.
    {
        if (code_seen('X'))
        {
            max_pos[X_AXIS] = code_value();
        }
        if (code_seen('Y'))
        {
            max_pos[Y_AXIS] = code_value();
        }
        if (code_seen('Z'))
        {
            max_pos[Z_AXIS] = code_value();
        }
    }
    break;
    case 12001: // M12001 - Set build volume minimum position.
    {
        if (code_seen('X'))
        {
            min_pos[X_AXIS] = code_value();
        }
        if (code_seen('Y'))
        {
            min_pos[Y_AXIS] = code_value();
        }
        if (code_seen('Z'))
        {
            min_pos[Z_AXIS] = code_value();
        }
    }
    break;
    case 12003: //M12003 - Get board type for debugging goals
    {
        SERIAL_ECHOLN(int(Board::getId()));
    }
    break;
    case 12004: //M12004 - Set axis direction. Has direct effect.
    {
        if (code_seen('X'))
            invert_direction[X_AXIS] = code_value() != 0;
        if (code_seen('Y'))
            invert_direction[Y_AXIS] = code_value() != 0;
        if (code_seen('Z'))
            invert_direction[Z_AXIS] = code_value() != 0;
        if (code_seen('E'))
        {
            bool inverted = code_value() != 0;
            if (!setTargetedHotend())
            {
                invert_direction[E_AXIS + tmp_extruder] = inverted;
                flowSensor[tmp_extruder].setDirection(inverted);
            }
        }
    }
    break;
    case 12005: // M12005 - Set extruder temperature compensation
    {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("# Set hotend compensation");
        if (code_seen('T'))
        {
            // Get by reference so we can change the settings
            GainCompensator& compensator = GainCompensatorFactory::getInstance(GainCompensatorFactory::EXTRUDER, int(code_value()));
            SERIAL_ECHOPGM(" index=");
            MSerial.print(int(code_value()));

            if (code_seen('F'))
            {
                SERIAL_ECHOPGM(" factor=");
                compensator.setFactor(code_value());
                MSerial.print(compensator.getFactor());
            }
            if (code_seen('O'))
            {
                SERIAL_ECHOPGM(" offset=");
                compensator.setOffset(code_value());
                MSerial.print(compensator.getOffset());
            }
        }
        SERIAL_ECHOLNPGM(" #");
    }
    break;
    case 12006: // M12006 - Set buildplate temperature compensation
    {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("# Set buildplate compensation");
        // Get by reference so we can change the settings
        GainCompensator& compensator = GainCompensatorFactory::getInstance(GainCompensatorFactory::BUILDPLATE, 0);

        if (code_seen ('F'))
        {
            SERIAL_ECHOPGM(" factor=");
            compensator.setFactor(code_value());
            MSerial.print(code_value(), DEC);
        }
        if (code_seen ('O'))
        {
            SERIAL_ECHOPGM(" offset=");
            compensator.setOffset(code_value());
            MSerial.print(code_value(), DEC);
        }
        SERIAL_ECHOLNPGM(" #");
    }
    case 12010: //M12010 - Read/write TMC2130 register
        if (Board::getId() == Board::BOARD_2621B)
        {
            uint8_t chip_idx = M12010_ALL_DRIVERS;
            uint8_t reg_idx = 0;
            if (code_seen('S'))
            {
                chip_idx = code_value_long();
            }
            if (code_seen('R'))
            {
                reg_idx = code_value_long();
            }

            if (code_seen('V'))
            {
                if (chip_idx == M12010_ALL_DRIVERS)
                {
                    for (chip_idx = 0; chip_idx < NUM_MOTOR_DRIVERS; chip_idx++)
                    {
                        StepperTMC2130::writeRegister(chip_idx, reg_idx, code_value_long());
                    }
                }
                else
                {
                    StepperTMC2130::writeRegister(chip_idx, reg_idx, code_value_long());
                }
            }
            else
            {
                if (chip_idx == M12010_ALL_DRIVERS)
                {
                    for (chip_idx = 0; chip_idx < NUM_MOTOR_DRIVERS; chip_idx++)
                    {
                        uint32_t value = StepperTMC2130::readRegister(chip_idx, reg_idx);
                        MSerial.println(value, HEX);
                    }
                }
                else
                {
                    uint32_t value = StepperTMC2130::readRegister(chip_idx, reg_idx);
                    MSerial.println(value, HEX);
                }
            }
        }
    break;
    }
  }
  else if(code_seen('T'))
  // Normally the T-command is just an administrative change, but when F is specified
  // there will be an immediate physical motion planned to the new nozzle offset position.
  // When F <= 0 the given value is ignored and the old feedrate remains in effect.
  {
    tmp_extruder = code_value();
    if(tmp_extruder < EXTRUDERS)
    {
      bool make_move = false;
      if(code_seen('F')) {
        make_move = true;
        float next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        flow->updateExtrusionPosition();     // Save the current flow status before switching to another extruder.
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        // Don't Offset extruder, as this is now done in the planner!
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
        // Move to the old position which has changed due to the offsets if 'F' was in the parameters
        if(make_move && !isStopped()) {
           plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        }
      }
      #endif
    }
  }
  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(command_buffer[command_buffer_index_read]);
    SERIAL_ECHOLNPGM("\"");
  }

  reported_plan_buf_free_positions = plan_buf_free_positions();
  SerialProtocol::sendResponse('o', gcodeline_sequence_number[command_buffer_index_read], reported_plan_buf_free_positions);
}

static void get_coordinates()
{
    for(int8_t i=0; i < NUM_AXIS; i++)
    {
        if(code_seen(axis_codes[i]))
        {
            destination[i] = (float)code_value();
            if (axis_relative_modes[i] || relative_mode)
                destination[i] += current_position[i];

            //Check if we are trying to move outside of our normal building area. We allow for some extra area outside of the volume
            //Because we messed up with definitions of build volume and head movement area.
            //This is an extra safety check against corrupted GCode.
            if (i < E_AXIS)
            {
                if (destination[i] < min_pos[i] - ALLOWED_POSITION_ERROR || destination[i] > max_pos[i] + ALLOWED_POSITION_ERROR)
                {
                    stop(STOP_REASON_POSITION_ERROR);
                    destination[i] = current_position[i];
                }
            }
        }
        else
        {
            destination[i] = current_position[i];
        }
    }
    if(code_seen('F'))
    {
        float next_feedrate = code_value();
        if(next_feedrate > 0.0)
            feedrate = next_feedrate;
    }
}

static void get_arc_coordinates()
{
   get_coordinates();

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

static void prepare_move()
{
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination[X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS]))
  {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else
  {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }

  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

static void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void manage_inactivity()
{
#if defined(SAFETY_TRIGGERED_PIN) && SAFETY_TRIGGERED_PIN > -1
    // Check if the safety pin was triggered. Revision I or newer boards no longer have the safety circuit, and abuse this pin to turn on the relay.
    // So we do only check this for boards before the BOARD_REV_I
    if (Board::getId() < Board::BOARD_REV_I && READ(SAFETY_TRIGGERED_PIN))
    {
        stop(STOP_REASON_SAFETY_TRIGGER);
    }
#endif
    check_axes_activity();
    pulseLeds();
    updatePCA9635();

    flow->update();
}

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
        //Do not change timer 0, connected to millies and temperature sampling.
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
        //Do not change timer 1, connected to stepper generation.
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

// @brief Sets the global variable tmp_extruder with the value specified in the T parameter.
// When no T-value is specified, then the current active extruder is used.
// @returns true on error.
static bool setTargetedHotend()
{
    tmp_extruder = active_extruder;
    if (code_seen('T'))
    {
        tmp_extruder = code_value();
        if(tmp_extruder >= EXTRUDERS)
        {
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_INVALID_EXTRUDER);
            SERIAL_ECHOLN(tmp_extruder);
            return true;
        }
    }
    return false;
}

/**
 * @brief Fills hex string from binary data, does not add '\0' at the end of the string.
 * @param[out] hex pointer to buffer to write resulting string to.
 * @param[in] bin pointer to array of binary data to be converted.
 * @param[in] count Number of bytes in the bin array to be converted.
 * @returns Nr of bytes converted, resulting string length is twice as large.
 */
static int bin2hex(char* hex, const uint8_t* bin, int count)
{
    for (int i = 0; i < count; i++)
    {
        *hex++ = bin_to_hex[*bin >> 4];
        *hex++ = bin_to_hex[*bin++ & 0xf];
    }
    return count;
}

static int hexNibble2Bin(const char nibble)
{
    if (nibble >= 'a' && nibble <= 'f')
        return nibble - ('a' - 10);
    if (nibble >= 'A' && nibble <= 'F')
        return nibble - ('A' - 10);
    if (nibble >= '0' && nibble <= '9')
        return nibble - '0';
    return -1;
}

/**
 * @brief Fills binary array from hex string.
 * @param[out] bin pointer to buffer to write resulting binary data to.
 * @param[in] hex pointer to string to be converted.
 * @param[in] buf_size Maximum number of bytes the bin array is going to have.
 * @returns Nr of bytes converted, resulting binary array is half as large.
 */
static int hex2bin(uint8_t* bin, const char* hex, const int buf_size)
{
    int count;
    for (count = 0; count < buf_size * 2; count++)
    {
        int bin_nibble = hexNibble2Bin(*hex++);
        if (bin_nibble >= 0)
        {
            // If first nibble.
            if (count % 2 == 0)
            {
                *bin = bin_nibble << 4;
            }
            else    // Second nibble.
            {
                *bin += bin_nibble;
                bin++;
            }
        }
        else
        {
            // End of hex string reached.
            break;
        }
    }

    return count;
}

#ifdef ENABLE_WATCHDOG_SERIAL_INPUT
/**
 * @brief Safety feature to verify communication is still running when there are hot objects.
 * Checks if the input buffer is not empty; it resets the counter, otherwise it starts the counter (if necessary)
 * If there are heated parts (in this case, the bed or any of the hotends), the check will be performed.
 * If the total elapsed time is bigger than the defined time out, all heaters are stopped and the hotend led will change to an orange color
 * and the printer will enter the Stop state
 */
void check_serial_input_timeout()
{
    // Commands available, simply reset and return (no need to do anything else)
    if (is_command_queued())
    {
        start_time_empty_command_buffer = 0;
        return;
    }

    // Check the minimum temperature threshold for bed and hotend(s)
    bool bed_hot = degBed() > MONITOR_MINIMUM_BED_TEMPERATURE;
    bool hotends_hot = false;
    for (int hotend_index = 0; hotend_index < EXTRUDERS; hotend_index++)
    {
        hotends_hot = hotends_hot || (degHotend(hotend_index) > MONITOR_MINIMUM_HOTEND_TEMPERATURE);
    }
    if (!(bed_hot || hotends_hot))
    {
        return;
    }

    // We have hot stuff, but no commands in the buffer, start keeping track of this time (if this was not yet done)
    if (start_time_empty_command_buffer == 0)
    {
        start_time_empty_command_buffer = millis();
        return;
    }

    // Determine if the elapsed time has passed the timeout. UM3 requests the temperature nearly every 100ms, if
    // this is more than the timeout, it means the other board has stopped working.
    unsigned long elapsed_time = millis() - start_time_empty_command_buffer;
    if (elapsed_time < MONITOR_SERIAL_INPUT_TIMEOUT * 1000UL)
    {
        return;
    }

    // It's official, no communication received within the timeout limit, time to take safety measures and turn the hot stuff off.
    // To indicate this problem, set the hotend(s) color to some shade of orange.
    for (uint8_t hotend_index = 0; hotend_index < EXTRUDERS; hotend_index++)
    {
        setPCA9635led(hotend_index, 255, 30, 0);
    }
    stop(STOP_REASON_SERIAL_INPUT_TIMEOUT);
}
#endif


// This table represents the brightness output scaling expressed as fraction of a 16 bit integer.
// f(x) ~ (exp(sin(x)) - 1/e) * 100/(e-1/e)
// -1/e = -0.36787944117144233, and 100/(e - 1/e) = 42.54590641196608 are constants, and can be pre-calculated.
// pi/2 is there to make sure the sinusoid is applied to the right range
// max_output = 2^16-1 # since then it is the most accurate in integer calculation afterwards.
// and is calculated with new_brightness = (math.exp(math.sin((brightness - max_output / 2) * math.pi / max_output)) - 0.36787944117144233) * 42.5459064119660
// Through this function a linear increment is fed, the output is an e power to make the light appear more gradual to the human eye.

const PROGMEM uint16_t brightness_table[] = {
/* i 0.000000, o 0.000000, int_o */ 0,
/* i 0.392157, o 0.001188, int_o */ 0,
/* i 0.784314, o 0.004752, int_o */ 3,
/* i 1.176471, o 0.010693, int_o */ 7,
/* i 1.568627, o 0.019013, int_o */ 12,
/* i 1.960784, o 0.029714, int_o */ 19,
/* i 2.352941, o 0.042801, int_o */ 28,
/* i 2.745098, o 0.058276, int_o */ 38,
/* i 3.137255, o 0.076144, int_o */ 49,
/* i 3.529412, o 0.096411, int_o */ 63,
/* i 3.921569, o 0.119083, int_o */ 78,
/* i 4.313725, o 0.144167, int_o */ 94,
/* i 4.705882, o 0.171670, int_o */ 112,
/* i 5.098039, o 0.201601, int_o */ 132,
/* i 5.490196, o 0.233969, int_o */ 153,
/* i 5.882353, o 0.268783, int_o */ 176,
/* i 6.274510, o 0.306054, int_o */ 200,
/* i 6.666667, o 0.345793, int_o */ 226,
/* i 7.058824, o 0.388012, int_o */ 254,
/* i 7.450980, o 0.432724, int_o */ 283,
/* i 7.843137, o 0.479943, int_o */ 314,
/* i 8.235294, o 0.529681, int_o */ 347,
/* i 8.627451, o 0.581955, int_o */ 381,
/* i 9.019608, o 0.636779, int_o */ 417,
/* i 9.411765, o 0.694170, int_o */ 454,
/* i 9.803922, o 0.754146, int_o */ 494,
/* i 10.196078, o 0.816723, int_o */ 535,
/* i 10.588235, o 0.881921, int_o */ 577,
/* i 10.980392, o 0.949758, int_o */ 622,
/* i 11.372549, o 1.020255, int_o */ 668,
/* i 11.764706, o 1.093432, int_o */ 716,
/* i 12.156863, o 1.169311, int_o */ 766,
/* i 12.549020, o 1.247913, int_o */ 817,
/* i 12.941176, o 1.329263, int_o */ 871,
/* i 13.333333, o 1.413383, int_o */ 926,
/* i 13.725490, o 1.500297, int_o */ 983,
/* i 14.117647, o 1.590032, int_o */ 1042,
/* i 14.509804, o 1.682612, int_o */ 1102,
/* i 14.901961, o 1.778065, int_o */ 1165,
/* i 15.294118, o 1.876417, int_o */ 1229,
/* i 15.686275, o 1.977697, int_o */ 1296,
/* i 16.078431, o 2.081934, int_o */ 1364,
/* i 16.470588, o 2.189156, int_o */ 1434,
/* i 16.862745, o 2.299394, int_o */ 1506,
/* i 17.254902, o 2.412679, int_o */ 1581,
/* i 17.647059, o 2.529043, int_o */ 1657,
/* i 18.039216, o 2.648518, int_o */ 1735,
/* i 18.431373, o 2.771137, int_o */ 1816,
/* i 18.823529, o 2.896934, int_o */ 1898,
/* i 19.215686, o 3.025943, int_o */ 1983,
/* i 19.607843, o 3.158200, int_o */ 2069,
/* i 20.000000, o 3.293740, int_o */ 2158,
/* i 20.392157, o 3.432600, int_o */ 2249,
/* i 20.784314, o 3.574817, int_o */ 2342,
/* i 21.176471, o 3.720429, int_o */ 2438,
/* i 21.568627, o 3.869475, int_o */ 2535,
/* i 21.960784, o 4.021994, int_o */ 2635,
/* i 22.352941, o 4.178026, int_o */ 2738,
/* i 22.745098, o 4.337611, int_o */ 2842,
/* i 23.137255, o 4.500791, int_o */ 2949,
/* i 23.529412, o 4.667608, int_o */ 3058,
/* i 23.921569, o 4.838104, int_o */ 3170,
/* i 24.313725, o 5.012321, int_o */ 3284,
/* i 24.705882, o 5.190305, int_o */ 3401,
/* i 25.098039, o 5.372098, int_o */ 3520,
/* i 25.490196, o 5.557746, int_o */ 3642,
/* i 25.882353, o 5.747295, int_o */ 3766,
/* i 26.274510, o 5.940790, int_o */ 3893,
/* i 26.666667, o 6.138277, int_o */ 4022,
/* i 27.058824, o 6.339804, int_o */ 4154,
/* i 27.450980, o 6.545418, int_o */ 4289,
/* i 27.843137, o 6.755167, int_o */ 4427,
/* i 28.235294, o 6.969099, int_o */ 4567,
/* i 28.627451, o 7.187264, int_o */ 4710,
/* i 29.019608, o 7.409710, int_o */ 4856,
/* i 29.411765, o 7.636487, int_o */ 5004,
/* i 29.803922, o 7.867646, int_o */ 5156,
/* i 30.196078, o 8.103236, int_o */ 5310,
/* i 30.588235, o 8.343308, int_o */ 5467,
/* i 30.980392, o 8.587914, int_o */ 5628,
/* i 31.372549, o 8.837104, int_o */ 5791,
/* i 31.764706, o 9.090931, int_o */ 5957,
/* i 32.156863, o 9.349445, int_o */ 6127,
/* i 32.549020, o 9.612698, int_o */ 6299,
/* i 32.941176, o 9.880743, int_o */ 6475,
/* i 33.333333, o 10.153632, int_o */ 6654,
/* i 33.725490, o 10.431417, int_o */ 6836,
/* i 34.117647, o 10.714151, int_o */ 7021,
/* i 34.509804, o 11.001884, int_o */ 7210,
/* i 34.901961, o 11.294671, int_o */ 7402,
/* i 35.294118, o 11.592562, int_o */ 7597,
/* i 35.686275, o 11.895610, int_o */ 7795,
/* i 36.078431, o 12.203867, int_o */ 7997,
/* i 36.470588, o 12.517383, int_o */ 8203,
/* i 36.862745, o 12.836212, int_o */ 8412,
/* i 37.254902, o 13.160403, int_o */ 8624,
/* i 37.647059, o 13.490007, int_o */ 8840,
/* i 38.039216, o 13.825075, int_o */ 9060,
/* i 38.431373, o 14.165656, int_o */ 9283,
/* i 38.823529, o 14.511800, int_o */ 9510,
/* i 39.215686, o 14.863555, int_o */ 9740,
/* i 39.607843, o 15.220970, int_o */ 9975,
/* i 40.000000, o 15.584091, int_o */ 10213,
/* i 40.392157, o 15.952966, int_o */ 10454,
/* i 40.784314, o 16.327639, int_o */ 10700,
/* i 41.176471, o 16.708157, int_o */ 10949,
/* i 41.568627, o 17.094562, int_o */ 11203,
/* i 41.960784, o 17.486898, int_o */ 11460,
/* i 42.352941, o 17.885207, int_o */ 11721,
/* i 42.745098, o 18.289529, int_o */ 11986,
/* i 43.137255, o 18.699903, int_o */ 12255,
/* i 43.529412, o 19.116368, int_o */ 12528,
/* i 43.921569, o 19.538961, int_o */ 12805,
/* i 44.313725, o 19.967716, int_o */ 13086,
/* i 44.705882, o 20.402667, int_o */ 13371,
/* i 45.098039, o 20.843846, int_o */ 13660,
/* i 45.490196, o 21.291284, int_o */ 13953,
/* i 45.882353, o 21.745009, int_o */ 14250,
/* i 46.274510, o 22.205048, int_o */ 14552,
/* i 46.666667, o 22.671425, int_o */ 14857,
/* i 47.058824, o 23.144163, int_o */ 15167,
/* i 47.450980, o 23.623283, int_o */ 15481,
/* i 47.843137, o 24.108803, int_o */ 15799,
/* i 48.235294, o 24.600738, int_o */ 16122,
/* i 48.627451, o 25.099102, int_o */ 16448,
/* i 49.019608, o 25.603907, int_o */ 16779,
/* i 49.411765, o 26.115160, int_o */ 17114,
/* i 49.803922, o 26.632867, int_o */ 17454,
/* i 50.196078, o 27.157032, int_o */ 17797,
/* i 50.588235, o 27.687653, int_o */ 18145,
/* i 50.980392, o 28.224728, int_o */ 18497,
/* i 51.372549, o 28.768252, int_o */ 18853,
/* i 51.764706, o 29.318214, int_o */ 19213,
/* i 52.156863, o 29.874602, int_o */ 19578,
/* i 52.549020, o 30.437401, int_o */ 19947,
/* i 52.941176, o 31.006590, int_o */ 20320,
/* i 53.333333, o 31.582148, int_o */ 20697,
/* i 53.725490, o 32.164046, int_o */ 21079,
/* i 54.117647, o 32.752255, int_o */ 21464,
/* i 54.509804, o 33.346741, int_o */ 21854,
/* i 54.901961, o 33.947465, int_o */ 22247,
/* i 55.294118, o 34.554385, int_o */ 22645,
/* i 55.686275, o 35.167455, int_o */ 23047,
/* i 56.078431, o 35.786623, int_o */ 23453,
/* i 56.470588, o 36.411836, int_o */ 23862,
/* i 56.862745, o 37.043033, int_o */ 24276,
/* i 57.254902, o 37.680151, int_o */ 24694,
/* i 57.647059, o 38.323120, int_o */ 25115,
/* i 58.039216, o 38.971869, int_o */ 25540,
/* i 58.431373, o 39.626319, int_o */ 25969,
/* i 58.823529, o 40.286387, int_o */ 26402,
/* i 59.215686, o 40.951985, int_o */ 26838,
/* i 59.607843, o 41.623022, int_o */ 27278,
/* i 60.000000, o 42.299399, int_o */ 27721,
/* i 60.392157, o 42.981013, int_o */ 28168,
/* i 60.784314, o 43.667757, int_o */ 28618,
/* i 61.176471, o 44.359517, int_o */ 29071,
/* i 61.568627, o 45.056175, int_o */ 29528,
/* i 61.960784, o 45.757606, int_o */ 29987,
/* i 62.352941, o 46.463682, int_o */ 30450,
/* i 62.745098, o 47.174268, int_o */ 30916,
/* i 63.137255, o 47.889224, int_o */ 31384,
/* i 63.529412, o 48.608403, int_o */ 31856,
/* i 63.921569, o 49.331655, int_o */ 32329,
/* i 64.313725, o 50.058823, int_o */ 32806,
/* i 64.705882, o 50.789745, int_o */ 33285,
/* i 65.098039, o 51.524252, int_o */ 33766,
/* i 65.490196, o 52.262170, int_o */ 34250,
/* i 65.882353, o 53.003322, int_o */ 34736,
/* i 66.274510, o 53.747520, int_o */ 35223,
/* i 66.666667, o 54.494577, int_o */ 35713,
/* i 67.058824, o 55.244294, int_o */ 36204,
/* i 67.450980, o 55.996471, int_o */ 36697,
/* i 67.843137, o 56.750901, int_o */ 37192,
/* i 68.235294, o 57.507371, int_o */ 37688,
/* i 68.627451, o 58.265663, int_o */ 38184,
/* i 69.019608, o 59.025553, int_o */ 38682,
/* i 69.411765, o 59.786813, int_o */ 39181,
/* i 69.803922, o 60.549210, int_o */ 39681,
/* i 70.196078, o 61.312502, int_o */ 40181,
/* i 70.588235, o 62.076448, int_o */ 40682,
/* i 70.980392, o 62.840796, int_o */ 41183,
/* i 71.372549, o 63.605293, int_o */ 41684,
/* i 71.764706, o 64.369680, int_o */ 42185,
/* i 72.156863, o 65.133692, int_o */ 42686,
/* i 72.549020, o 65.897063, int_o */ 43186,
/* i 72.941176, o 66.659517, int_o */ 43685,
/* i 73.333333, o 67.420780, int_o */ 44184,
/* i 73.725490, o 68.180568, int_o */ 44682,
/* i 74.117647, o 68.938597, int_o */ 45179,
/* i 74.509804, o 69.694578, int_o */ 45675,
/* i 74.901961, o 70.448217, int_o */ 46168,
/* i 75.294118, o 71.199218, int_o */ 46661,
/* i 75.686275, o 71.947281, int_o */ 47151,
/* i 76.078431, o 72.692103, int_o */ 47639,
/* i 76.470588, o 73.433377, int_o */ 48125,
/* i 76.862745, o 74.170797, int_o */ 48608,
/* i 77.254902, o 74.904049, int_o */ 49089,
/* i 77.647059, o 75.632820, int_o */ 49566,
/* i 78.039216, o 76.356795, int_o */ 50041,
/* i 78.431373, o 77.075655, int_o */ 50512,
/* i 78.823529, o 77.789083, int_o */ 50979,
/* i 79.215686, o 78.496756, int_o */ 51443,
/* i 79.607843, o 79.198353, int_o */ 51903,
/* i 80.000000, o 79.893551, int_o */ 52359,
/* i 80.392157, o 80.582027, int_o */ 52810,
/* i 80.784314, o 81.263457, int_o */ 53256,
/* i 81.176471, o 81.937518, int_o */ 53698,
/* i 81.568627, o 82.603885, int_o */ 54135,
/* i 81.960784, o 83.262236, int_o */ 54566,
/* i 82.352941, o 83.912248, int_o */ 54992,
/* i 82.745098, o 84.553600, int_o */ 55413,
/* i 83.137255, o 85.185971, int_o */ 55827,
/* i 83.529412, o 85.809043, int_o */ 56235,
/* i 83.921569, o 86.422500, int_o */ 56637,
/* i 84.313725, o 87.026026, int_o */ 57033,
/* i 84.705882, o 87.619311, int_o */ 57422,
/* i 85.098039, o 88.202045, int_o */ 57804,
/* i 85.490196, o 88.773921, int_o */ 58178,
/* i 85.882353, o 89.334638, int_o */ 58546,
/* i 86.274510, o 89.883897, int_o */ 58906,
/* i 86.666667, o 90.421402, int_o */ 59258,
/* i 87.058824, o 90.946862, int_o */ 59602,
/* i 87.450980, o 91.459993, int_o */ 59939,
/* i 87.843137, o 91.960512, int_o */ 60267,
/* i 88.235294, o 92.448144, int_o */ 60586,
/* i 88.627451, o 92.922617, int_o */ 60897,
/* i 89.019608, o 93.383668, int_o */ 61199,
/* i 89.411765, o 93.831037, int_o */ 61493,
/* i 89.803922, o 94.264472, int_o */ 61777,
/* i 90.196078, o 94.683727, int_o */ 62051,
/* i 90.588235, o 95.088563, int_o */ 62317,
/* i 90.980392, o 95.478747, int_o */ 62572,
/* i 91.372549, o 95.854056, int_o */ 62818,
/* i 91.764706, o 96.214272, int_o */ 63054,
/* i 92.156863, o 96.559185, int_o */ 63281,
/* i 92.549020, o 96.888596, int_o */ 63496,
/* i 92.941176, o 97.202311, int_o */ 63702,
/* i 93.333333, o 97.500145, int_o */ 63897,
/* i 93.725490, o 97.781923, int_o */ 64082,
/* i 94.117647, o 98.047479, int_o */ 64256,
/* i 94.509804, o 98.296654, int_o */ 64419,
/* i 94.901961, o 98.529301, int_o */ 64572,
/* i 95.294118, o 98.745281, int_o */ 64713,
/* i 95.686275, o 98.944465, int_o */ 64844,
/* i 96.078431, o 99.126732, int_o */ 64963,
/* i 96.470588, o 99.291975, int_o */ 65071,
/* i 96.862745, o 99.440092, int_o */ 65169,
/* i 97.254902, o 99.570996, int_o */ 65254,
/* i 97.647059, o 99.684606, int_o */ 65329,
/* i 98.039216, o 99.780855, int_o */ 65392,
/* i 98.431373, o 99.859683, int_o */ 65444,
/* i 98.823529, o 99.921044, int_o */ 65484,
/* i 99.215686, o 99.964899, int_o */ 65512,
/* i 99.607843, o 99.991224, int_o */ 65530,
/* i 100.00000, o 100.00000, int_o */ 65535
    };


void pulseLeds()
{
    static uint16_t last_time = 0;
    static uint16_t led_linear_output = 0;
    static int8_t led_direction = 1;
    static int8_t led_current_color = 0;

    if (uint16_t(millis()) - last_time < blink_time)
        return;

    last_time = millis();

    for (int tool = 0; tool < EXTRUDERS; tool++)
    {
        if (pulse[tool])
        {
            uint8_t* color_ptr = NULL;
            if (led_current_color == 0 || pulse[tool] == PULSE_ONLY_ONE_COLOR)
                color_ptr = first_color[tool];
            else
                color_ptr = second_color[tool];

            uint32_t brightness = pgm_read_word(&brightness_table[led_linear_output]);
            uint16_t red   = (uint32_t(color_ptr[0]) * brightness) / 65536;
            uint16_t green = (uint32_t(color_ptr[1]) * brightness) / 65536;
            uint16_t blue  = (uint32_t(color_ptr[2]) * brightness) / 65536;

            setPCA9635led(tool, red, green, blue);
        }
    }
    led_linear_output += led_direction;
    if (led_linear_output == 0)
    {
        led_direction = 1;
        if (led_current_color != 0)
            led_current_color = 0;
        else
            led_current_color = 1;
    }
    if (led_linear_output >= sizeof(brightness_table) / sizeof(brightness_table[0])-1)
        led_direction = -1;
}
