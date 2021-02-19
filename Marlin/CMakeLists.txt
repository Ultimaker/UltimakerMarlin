project(jedi-marlin-firmware)
cmake_minimum_required(VERSION 2.8.12)

set(FIRMWARE_NAME jedi)
set(AVR_MCU atmega2560)
set(AVR_F_CPU 16000000)
set(ARDUINO_INSTALL_DIR /usr/share/arduino)

find_program(AVR_CC avr-gcc)
find_program(AVR_CXX avr-g++)
find_program(AVR_OBJCOPY avr-objcopy)
find_program(AVR_SIZE_TOOL avr-size)
find_program(AVR_OBJDUMP avr-objdump)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
set(CMAKE_C_COMPILER ${AVR_CC})
set(CMAKE_CXX_COMPILER ${AVR_CXX})
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(CMAKE_BUILD_TYPE Release)

set(SOURCES
	crc8.cpp
	crc16.cpp
	MarlinSerial.cpp
	Board.cpp
	GainCompensatorFactory.cpp
	Marlin_main.cpp
	PowerBudgetManagement.cpp
	PowerBudgetManagementBed.cpp
	PowerBudgetManagementHotendSlot.cpp
	Stop.cpp
	SerialProtocol.cpp
	cap_sense_probe.cpp
	fan_driver.cpp
	flow.cpp
	flow_A1335.cpp
	i2c_capacitance_FDC1004.cpp
	i2c_driver.cpp
	i2c_onewire_ds2482.cpp
	led_rgbw_pca9632.cpp
	motion_control.cpp
	oneWireDS2431_eeprom.cpp
	pca9635_driver.cpp
	planner.cpp
	stepper.cpp
	stepper_TMC2130.cpp
	stepper_A4988.cpp
	temperature.cpp
	temperaturePID.cpp
	temperature_ADS101X.cpp
	usart_spi_driver.cpp
	watchdog.cpp
)
set(ARDUINO_SOURCES
	${ARDUINO_INSTALL_DIR}/hardware/arduino/cores/arduino/wiring.c
	${ARDUINO_INSTALL_DIR}/hardware/arduino/cores/arduino/wiring_analog.c
	${ARDUINO_INSTALL_DIR}/hardware/arduino/cores/arduino/wiring_digital.c
	${ARDUINO_INSTALL_DIR}/hardware/arduino/cores/arduino/main.cpp
)


add_definitions(-DF_CPU=${AVR_F_CPU})
add_definitions(-DARDUINO=105)
# Setup the proper compiler flags for our build. We want maximum warnings, except for a few.
set(CMAKE_CXX_FLAGS_RELEASE "-Os -Wall -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -fdata-sections")

include_directories(${ARDUINO_INSTALL_DIR}/hardware/arduino/cores/arduino/)
include_directories(${ARDUINO_INSTALL_DIR}/hardware/arduino/variants/mega/)

set(ELF_FILE ${FIRMWARE_NAME}.elf)
set(HEX_FILE ${FIRMWARE_NAME}.hex)

add_executable(${ELF_FILE} ${SOURCES} ${ARDUINO_SOURCES})
set_target_properties(${ELF_FILE} PROPERTIES
	COMPILE_FLAGS "-mmcu=${AVR_MCU}"
	LINK_FLAGS "-mmcu=${AVR_MCU} -Wl,--gc-sections"
)
add_custom_command(OUTPUT ${HEX_FILE}
	COMMAND ${AVR_OBJCOPY} -j .text -j .data -O ihex ${ELF_FILE} ${HEX_FILE}
	DEPENDS ${ELF_FILE}
)
add_custom_target(
	${FIRMWARE_NAME}
	ALL
	DEPENDS ${HEX_FILE}
)
