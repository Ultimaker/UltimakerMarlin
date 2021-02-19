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

#include <stdio.h>
#include <stdint.h>

#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "flow.h"
#include "motion_control.h"
#include "watchdog.h"
#include "language.h"
#include "i2c_driver.h"
#include "fan_driver.h"
#include "stepper_TMC2130.h"
#include "usart_spi_driver.h"
#include "SerialProtocol.h"
#include "Board.h"
#include "GainCompensatorFactory.h"
#include "lifetime_stats.h"

#include "uart.h"
#include "cmdline.h"
#include "cmdline_mem.h"

#define CMDLINE

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
// G90 - Use Absolute Coordinates. Affects all axes, including the E-axis. See M82 for E-axis only.
// G91 - Use Relative Coordinates. Affects all axes, including the E-axis. See M83 for E-axis only.
// G92 - Set current position to coordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp and status info
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
// M142 - Set system lights (M142 w[0-255]) Note that the parameter letters are all lower case.
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
// M12020 - Activate the high power 24V self test. This will check if the 24VHP can be turned off and on. Triggers an error if it can't.
// M12030 - Set the Top Cap fan speed: S<speed 0-255>, without S-parameter the fan will be in 'auto' mode.
// M12031 - Read the topcap present signal
// M12032 - Read the topcap temperature

//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef __AVR
static FILE         std_uart;  /* Storage holding the UART file descriptor referred to by stdin, stdout and stderr */

int cmdline_board_id(void *cmdline, int argc, char *argv[] __attribute__((unused)));
int cmdline_fan24(void *cmdline, int argc, char *argv[] __attribute__((unused)));
int cmdline_temp(void *cmdline, int argc, char *argv[] __attribute__((unused)));
int cmdline_lifetime(void *cmdline, int argc, char *argv[] __attribute__((unused)));

#ifdef CMDLINE
void                *cmdline = NULL;
struct command_t    commands[] = {
    { "?",          cmdline_help },
    { "help",       cmdline_help },
    { "echo",       cmdline_echo },
    { "history",    cmdline_history },
    { "mem",        cmdline_meminfo },
    { "memdump",    cmdline_memdump },
    { "flashdump",  cmdline_memdump },
    { "eepromdump", cmdline_memdump },
    { "board_id",   cmdline_board_id},
    { "fan24",      cmdline_fan24},
    { "temp",       cmdline_temp},
    { "lifetime",   cmdline_lifetime},
    { "",           NULL }  /* Sentinel */
};
#endif /* CMDLINE */
#endif /* __AVR */

float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = { false, false, false, false };
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply = 100;  //100->1 200->2
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
int16_t target_topcap_fan_speed = TOP_CAP_FAN_AUTO_MODE;

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
static int16_t gcodeline_sequence_number[BUFSIZE];      // CKI: it works and is efficient, but also ugly from OO perspective. Better make this part of a new cmdbuffer object thingy.

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
static bool setTargetedHotend();

/** @brief The callback function to be executed when a packet has been read
 *  @param packet The network packet received from the serial line
 *  @return Returns True if the packet could be processed and added to the command buffer
 */
static bool handle_packet(SerialProtocol::network_packet_t packet);

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

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
 *  @return Returns true when board is initialized successful, otherwise false
 */
bool initBoard()
{
    Board::detect();      // Detect the PCB revision.
    uint8_t result = Board::init();
    if (result > 0)
    {
        stop(result);
        return false;
    }
    return true;
}

#ifdef __AVR
// stdout/stderr to UART glue function.
static int uart_putch(char c, FILE *stream)
{
    if (stream != &std_uart)
        return 0;

    if (c == '\n')
        c = '\r';

    uart0_putc(c);
    return 0;
}

// UART to stdin glue function.
static int uart_getch(FILE *stream)
{
    uint16_t  ch;

    if (stream != &std_uart)
        return EOF;

    ch = uart0_getc();

    if (ch == UART_NO_DATA)
        return EOF;

    if (ch & UART_BUFFER_OVERFLOW)
        fprintf_P(stderr, PSTR("Receive ringbuffer overflow\n"));
    if (ch & UART_OVERRUN_ERROR)
        fprintf_P(stderr, PSTR("Overrun condition by UART\n"));
    if (ch & UART_FRAME_ERROR)
        fprintf_P(stderr, PSTR("Framing Error by UART\n"));

    return ch & 0xff;
}

static void setup_AVR()
{
    // Initialize UART 0 for the command line interface.
    uart0_init(UART_BAUD_SELECT_DOUBLE_SPEED(115200, F_CPU));

    // Setup std_uart file descriptors with read and write glue function vectors.
    fdev_setup_stream(&std_uart, &uart_putch, &uart_getch, _FDEV_SETUP_RW);

    // Assign stdin, stdout and stderr to std_uart file descriptor for printf and friends.
    stdin  = &std_uart;
    stdout = &std_uart;
    stderr = &std_uart;

    // Announce ourselves
    printf_P(PSTR("\n\nStarting Marlin (built " __DATE__ " " __TIME__")\n"));

    // Report version the bootloader reported in GPIOR1, after announcing ourselfs to avoid
    // suggesting it is reported by the bootloader itself.
    printf_P(PSTR("Bootloader reported version v%d\n"), GPIOR1);

    // Bootloader v0 clears MCUSR, but does not copy the contents in GPIOR0.
    uint8_t reset_source;
    if (GPIOR1 == 0) {
        // GPIOR1 is also 0 if no bootloader is present, in that case MCUSR still holds the reset source.
        printf_P(PSTR("Warning: Bootloader v0 or absent! MCUSR reported reset source: "));
        reset_source = MCUSR;
    } else {
        // Report the reset cause, copied into GPIOR0 by bootloader V1 and up.
        printf_P(PSTR("Bootloader reported reset source: "));
        reset_source = GPIOR0;
    }
    if (reset_source & _BV(PORF)) {
        printf_P(PSTR("Power-on\n"));
    } else if (reset_source & _BV(EXTRF)) {
        printf_P(PSTR("External\n"));
    } else if (reset_source & _BV(BORF)) {
        printf_P(PSTR("Brown-out\n"));
    } else if (reset_source & _BV(WDRF)) {
        printf_P(PSTR("Watchdog\n"));
    } else if (reset_source & _BV(JTRF)) {
        printf_P(PSTR("JTAG\n"));
    } else
        printf_P(PSTR("Unknown (GPIOR0=0x%.2x, MCUSR=0x%.2x)\n"), GPIOR0, MCUSR);
    MCUSR = 0;  // Reset the reset source bits
}
#endif // __AVR

void setup()
{
#ifdef __AVR
    setup_AVR();
#endif /* __AVR */

    MSerial.begin(BAUDRATE);

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("BUILD:" __DATE__ " " __TIME__);

    watchdog_init();
    st_init();    // Initialize stepper, this will automatically disable the steppers
    initBoard();

    Board::BoardType board_id = Board::getId();
    if (board_id == Board::BOARD_2621B || board_id == Board::BOARD_V4 || board_id == Board::BOARD_E2)
    {
        UsartSpiDriver::init();
        StepperTMC2130::init();
    }
    i2cDriverInit();
    initFans();
    temperatureInit();
    plan_init();  // Initialize planner
    lifetime_stats_init();
    st_enable_interrupt(); // this enables interrupts!

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Marlin started");

#ifdef __AVR
#ifdef CMDLINE
    cmdline_initstackprotector();

    /* Spawn a command line interface */
    cmdline_new(&cmdline, commands);
#else
    printf_P(PSTR("Marlin started\n"));
#endif /* CMDLINE */
#endif /* __AVR */
}

void loop()
{
#ifdef __AVR
#ifdef CMDLINE
    /* Handle the command line interface */
    cmdline_thread(cmdline);
#endif /* CMDLINE */
#endif /* __AVR */

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
    command_buffer_length--;
    command_buffer_index_read = (command_buffer_index_read + 1) % BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
}

static bool handle_packet(SerialProtocol::network_packet_t packet)
{
    return add_command(packet.payload, packet.payload_size, packet.sequence_number);
}

// sequence_number  Use -1 when not applicable.
bool add_command(void* payload, uint8_t payload_size, int16_t sequence_number)
{
    if (command_buffer_length >= BUFSIZE)
    {
        // No space, sorry,
        return false;
    }

    memcpy((void*)command_buffer[command_buffer_index_write], payload, payload_size);
    command_buffer[command_buffer_index_write][payload_size] = 0; //terminate string
    gcodeline_sequence_number[command_buffer_index_write] = sequence_number;

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
        get_coordinates(); // For X Y Z E F
        prepare_move();
        break;
    case 2: // G2  - CW ARC
        get_arc_coordinates();
        prepare_arc_move(true);
        break;
    case 3: // G3  - CCW ARC
        get_arc_coordinates();
        prepare_arc_move(false);
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
    case 90: // G90 - Use Absolute Coordinates
      relative_mode = false;
      break;
    case 91: // G91 - Use Relative Coordinates
      relative_mode = true;
      break;
    case 92: // G92 - Set current position to coordinates given
      {
        for (uint8_t i=0; i < NUM_AXIS; i++)
        {
          if (code_seen(axis_codes[i]))
          {
            current_position[i] = code_value();
            if (i == E_AXIS)
            {
              plan_set_e_position(current_position[E_AXIS]);    // Calls st_synchronize, so planner will be flushed. This is the reason for not calling G92 with an E-position during prints, it causes blobs.

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
        uint8_t w = 0;
        if (code_seen('w'))
          w = code_value();
#if LED_PIN > -1
        if (Board::getId() != Board::BOARD_2621B && Board::getId() != Board::BOARD_V4)   // 2621 board uses this pin for the case fan
        {
            analogWrite(LED_PIN, w);
        }
#endif
      }
      break;
    case 104: // M104 - Set extruder target temp
      if(setTargetedHotend()){
        break;
      }
      if (code_seen('S'))
      {
        float target_temp = code_value();
        
        // If the hotend is turned off, force writing the lifetime_stats to eeprom
        if (hotend_pid[tmp_extruder].getTargetTemperature() > HOTEND_HUMAN_TOUCHABLE_TEMPERATURE && target_temp == 0.0)
        {
            lifetime_stats_force_write_to_eeprom();
        }

        hotend_pid[tmp_extruder].setTargetTemperature(target_temp);
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
            if (tmp_extruder > 0)
            {
                SERIAL_ECHO(' ');
            }
            SERIAL_ECHOPGM("T");
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
            SERIAL_ECHO('f');
            SERIAL_ECHO(flow->getSensorRaw(tmp_extruder));
            SERIAL_ECHO('/');
            SERIAL_ECHO(flow->getExtrusionPosition(tmp_extruder));
        }
        #if defined(HEATER_BED_PIN)
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
        // Topcap
        SERIAL_ECHO('t');
        SERIAL_ECHO(int(TOPCAP_IS_PRESENT));    // Topcap present yes/no
        SERIAL_ECHO('/');
        SERIAL_ECHO(int(degTopcap()));          // Topcap temperature
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
        Board::powerDownSafely();
        break;

    case 82: // M82  - Set E codes absolute (default)
      axis_relative_modes[E_AXIS] = false;
      break;
    case 83: // M83  - Set E codes relative while in Absolute Coordinates (G90) mode
      axis_relative_modes[E_AXIS] = true;
      break;
    case 18: // Compatibility: M18  - Disable all stepper motors; same as M84
    case 84: // M84  - Disable steppers until next move
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
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
        SERIAL_ECHOPGM(" MACHINE_TYPE:E2");
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
        // TODO: add code that changes the maximum current in the stepper drivers for the specified axis.
    }
    break;
    case 998: // M998 - Intentionally stop the system as if by an error.
    {
        stop(STOP_REASON_GCODE);
    }
    break;
    case 999: // M999 - Restart after being stopped by error
    {
        clearStopReason();
        Board::powerUp();
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
    break;
    case 12010: //M12010 - Read/write TMC2130 register
    {
        Board::BoardType board_id = Board::getId();
        if (board_id == Board::BOARD_2621B || board_id == Board::BOARD_V4 || board_id == Board::BOARD_E2)
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
                        cmdline_printf_P(cmdline, PSTR("Chip %d:x%02X=%08X\n"), chip_idx, reg_idx, value);
                    }
                }
                else
                {
                    uint32_t value = StepperTMC2130::readRegister(chip_idx, reg_idx);
                    MSerial.println(value, HEX);
                    cmdline_printf_P(cmdline, PSTR("Chip %d:x%02X=%08X\n"), chip_idx, reg_idx, value);
                }
            }
        }
    }
    break;
    case 12020: // M12020 - Test the 24V High Power safety switch
    {
        st_synchronize();
        if(!Board::test24HP())
        {
            stop(STOP_REASON_24HP_CHECK_FAILED);
        }
    }
    break;
    case 12030: // M12030 - Set the Top Cap fan speed: S<speed 0-255>, without S-parameter the fan will be in 'auto' mode.
    {
        if (code_seen('S'))
        {
            target_topcap_fan_speed = constrain(code_value(), 0, 255);
            setTopCapFanSpeed(0);       // Note: parameter is ignored but we call to have direct effect.
        }
        else
        {
            target_topcap_fan_speed = TOP_CAP_FAN_AUTO_MODE;
            setTopCapFanSpeed(0);
        }
    }
    break;
    case 12031: // M12031 - Read the topcap present signal
    {
        SERIAL_ECHO(TOPCAP_IS_PRESENT);
        cmdline_printf_P(cmdline, PSTR("Topcap present=%d\n"), TOPCAP_IS_PRESENT);
    }
    break;
    case 12032: // M12032 - Read the topcap temperature
    {
        SERIAL_ECHO_F(degTopcap(), 1);
        cmdline_printf_P(cmdline, PSTR("Topcap temperature=%.1f\n"), degTopcap());
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
      bool make_move __attribute__((unused)) = false;
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
        if (make_move) {
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

  // If this was an external gcode line, then report the gcode as executed successful.
  if (gcodeline_sequence_number[command_buffer_index_read] != LOCAL_COMMAND)
  {
    reported_plan_buf_free_positions = plan_buf_free_positions();
    SerialProtocol::sendResponse('o', gcodeline_sequence_number[command_buffer_index_read], reported_plan_buf_free_positions);
  }
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
    check_axes_activity();

    flow->update();
    lifetime_stats_update();
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
    stop(STOP_REASON_SERIAL_INPUT_TIMEOUT);
}
#endif

ISR(BADISR_vect)
{
    /* Wait for the watchdog reset */
    for(;;);
}

int cmdline_board_id(void *cmdline, int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
    cmdline_printf_P(cmdline, PSTR(" PCB_ID: %d\n"), int(Board::getId()));
    return ERR_OK;
}

int cmdline_fan24(void *cmdline, int argc, char *argv[] __attribute__((unused)))
{
    int8_t fan_speed = 0;

    if (argc >= 2)
    {
        fan_speed = strtol(argv[1], NULL, 10);
    }

    target_fan_speed = fan_speed;   // Sets the  Material Cooling fan, is activated by the planner-loop.
    cmdline_printf_P(cmdline, PSTR("Set fan speed to %d\n"), target_fan_speed);

    return ERR_OK;
}

int cmdline_lifetime(void *cmdline, int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
    cmdline_printf_P(cmdline, PSTR("version=%d\n"), lifetime.version);
    cmdline_printf_P(cmdline, PSTR("on_minutes=%d\n"), lifetime.on_minutes);
    cmdline_printf_P(cmdline, PSTR("print_minutes=%d\n"), lifetime.print_minutes);
    cmdline_printf_P(cmdline, PSTR("print_millimeters=%d\n"), lifetime.print_millimeters);
    cmdline_printf_P(cmdline, PSTR("topcap_fan_minutes=%d\n"), lifetime.topcap_fan_minutes);

    return ERR_OK;
}

// Shows the current temperature
// @param debug, when set to a value > 0 it will continuously show the temperature and PID values.
//        Turn off again by entering the 'temp' command without parameters or a 0 parameter.
int cmdline_temp(void *cmdline, int argc, char *argv[] __attribute__((unused)))
{
    bool set_debug_on = false;

    cmdline_printf_P(cmdline, PSTR("Temperatures are:"));
    for (uint8_t e=0; e < EXTRUDERS; e++)
    {
        cmdline_printf_P(cmdline, PSTR("%.2f/%u%% "), degHotend(e), pwr.getActualHeaterOutput(e));
    }

    cmdline_printf_P(cmdline, PSTR("bed=%.2f/%u%%\n"), degBed(), pwr.getActualBedOutput());

    if (argc >= 2)
    {
        set_debug_on = strtol(argv[1], NULL, 10);
    }
    hotend_pid[0].setDebugDump(set_debug_on);

    return ERR_OK;
}
