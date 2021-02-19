Marlin 3D Printer Firmware
==========================

[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/ErikZalm/Marlin&title=Marlin&language=&tags=github&category=software)

Quick Information
===================
This RepRap firmware is based on the <a href="https://github.com/Ultimaker/Ultimaker2Marlin">Ultimaker2Marlin</a> code.
The UM2 is based on the UMO and the UMO in turn is based on a version of Marlin from around 2013. The UMO is a mashup between <a href="https://github.com/kliment/Sprinter">Sprinter</a>, <a href="https://github.com/grbl/grbl">grbl</a> and many original parts.

Derived from Sprinter and Grbl by Erik van der Zalm.
Sprinters lead developers are Kliment and caru.
Grbls lead developer is Simen Svale Skogsrud. Sonney Jeon (Chamnit) improved some parts of grbl
A fork by bkubicek for the Ultimaker was merged, and further development was aided by him.
Some features have been added by:
Lampmaker, Bradley Feldman, and others...

Main difference from UM2:
* Removed SD Card support, this is now handled by the other CPU board.
* Removed all LCD support.

Features:
*   Interrupt based movement with real linear acceleration
*   High steprate
*   Look ahead (Keep the speed high when possible. High cornering speed)
*   Interrupt based temperature protection
*   Preliminary support for Matthew Roberts advance algorithm
    For more info see: http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*   Full endstop support
*   Many small but handy things originating from bkubicek's fork.
*   Arc support
*   Temperature oversampling
*   Support for QTMarlin, a very beta GUI for PID-tuning and velocity-acceleration testing. https://github.com/bkubicek/QTMarlin
*   Endstop trigger reporting to the host software.
*   Heater power reporting. Useful for PID monitoring.
*   PID tuning
*   Automatic operation of extruder/cold-end cooling fans based on nozzle temperature

The default baudrate is 250000. This baudrate has less jitter and hence errors than the usual 115200 baud, but is less supported by drivers and host-environments.


Differences and additions to the already good Sprinter firmware:
================================================================

*Look-ahead:*

Marlin has look-ahead. While sprinter has to break and re-accelerate at each corner,
lookahead will only decelerate and accelerate to a velocity,
so that the change in vectorial velocity magnitude is less than the xy_jerk_velocity.
This is only possible, if some future moves are already processed, hence the name.
It leads to less over-deposition at corners, especially at flat angles.

*Arc support:*

Old versions of Slic3r (before v2.0) can find curves that, although broken into segments, were meant to describe an arc.
Marlin is able to print those arcs. The advantage is the firmware can choose the resolution,
and can perform the arc with nearly constant velocity, resulting in a nice finish.
Also, less serial communication is needed.

*Endstop trigger reporting:*

If an endstop is hit while moving towards the endstop, the location at which the firmware thinks that the endstop was triggered is outputed on the serial port.
This is useful, because the user gets a warning message.
However, also tools like QTMarlin can use this for finding acceptable combinations of velocity+acceleration.

*Coding paradigm:*

Not relevant from a user side, but Marlin was split into thematic junks, and has tried to partially enforce private variables.
This is intended to make it clearer, what interacts with what, and leads to a higher level of modularization.
We think that this is a useful prestep for porting this firmware to e.g. an ARM platform in the future.
A lot of RAM (with enabled LCD ~2200 bytes) was saved by storing char []="some message" in Program memory.
In the serial communication, a #define based level of abstraction was enforced, so that it is clear that
some transfer is information (usually beginning with "LOG:"), an error "Error:", or just normal protocol,
necessary for backwards compatibility.

Serial Protocol Implementation:
===============================
The serial protocol implementation can be found in the griffin repository, docs/Serial_Protocol.odt

Implemented G Codes:
====================

*  G0  -> G1
*  G1  - Coordinated Movement X Y Z E
*  G2  - CW ARC
*  G3  - CCW ARC
*  G4  - Dwell S<seconds> or P<milliseconds>
*  G28 - Home all Axis
*  G90 - Use Absolute Coordinates
*  G91 - Use Relative Coordinates
*  G92 - Set current position to given coordinates

RepRap M Codes
*  M104 - Set extruder target temp
*  M105 - Read current temp
*  M106 - Fan on
*  M107 - Fan off

Custom M Codes
*  M17  - Enable/Power all stepper motors
*  M18  - Disable all stepper motors; same as M84
*  M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
*  M80  - Turn on Power Supply
*  M81  - Turn off Power Supply
*  M82  - Set E codes absolute (default)
*  M83  - Set E codes relative while in Absolute Coordinates (G90) mode
*  M84  - Disable steppers until next move.
*  M92  - Set axis_steps_per_unit - same syntax as G92
*  M114 - Output current position to serial port
*  M115 - Capabilities string
*  M119 - Output Endstop status to serial port
*  M120 - Disable endstops
*  M121 - Enable endstops
*  M140 - Set bed target temp
*  M142 - Set system LEDs
*  M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
*  M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
*  M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2.
*  M205 -  advanced settings: minimum travel speed S=while printing, T=travel only, B=minimum segment time [us], X=maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
*  M206 - Set additional homeing offset
*  M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
*  M220 - set speed factor override percentage: S<factor in percent>
*  M221 - set extrude factor override percentage: S<factor in percent>
*  M301 - Set nozzle PID parameters FF, P, I and D
*  M302 - Allow cold extrudes
*  M304 - Set bed PID parameters FF, P, I and D
*  M400 - Finish all moves
*  M401 - Quickstop - Abort all the planned moves
*  M405 - Enable/disable the flow sensor hardware. S1=active A=averaging value
*  M406 - Read raw flow sensor value: S<sensor number>
*  M907 - Set digital trimpot motor current using axis codes.
*  M998 - Intentionally stop the system as if by an error.
*  M999 - Restart after being stopped by error
*  M12000 - Set build volume maximum size.
*  M12001 - Set build volume minimum position.


Configuring and compilation:
============================
Install the Classic Arduino software IDE/toolset v1.0.5
   https://www.arduino.cc/en/Main/OldSoftwareReleases#1.0.x

Remove I2C driver from Arduino:
A one time change. Marlin made changes to the I2C driver. You will have to remove this driver function from the
Arduino IDE by removing the TWI_vect interrupt routine in arduino/libraries/Wire/utility/twi.c, line 364

Copy the Marlin firmware
   https://github.com/Ultimaker/UltimakerMarlin
   (Use the download button)

Start the arduino IDE.
Select Tools -> Board -> Arduino Mega 2560
Open Marlin.pde
Click the Verify/Compile button

Copy the generated hex file to your printer

Restart the printer and the firmware will be installed.

That's it.  Enjoy Silky Smooth Printing.
