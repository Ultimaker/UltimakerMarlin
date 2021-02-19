project(jedi-marlin-simulator)

cmake_minimum_required(VERSION 2.8.12)

option(USE_SDL "Use SDL to visually render the output of the simulator" OFF)

# Add warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/crc8.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/crc16.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/SerialProtocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/MarlinSerial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/Board.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/GainCompensatorFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/Marlin_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/Stop.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/cap_sense_probe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/fan_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/flow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/flow_A1335.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/i2c_capacitance_FDC1004.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/i2c_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/i2c_onewire_ds2482.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/led_rgbw_pca9632.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/motion_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/oneWireDS2431_eeprom.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/pca9635_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/stepper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/stepper_TMC2130.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/stepper_A4988.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/temperature.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/temperature_ADS101X.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/temperaturePID.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/PowerBudgetManagement.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/PowerBudgetManagementBed.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/PowerBudgetManagementHotendSlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/usart_spi_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/watchdog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/HardwareSerial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/LiquidCrystal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/Print.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/Stream.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/Tone.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/WString.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/new.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/wiring.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/wiring_analog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/wiring_digital.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/wiring_pulse.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim/wiring_shift.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/avr_sim/avr/sim_io.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/FDC1004.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/adc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/adc_ADS101X.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/arduinoIO.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/flux_AS5048B.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/heater.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/i2c.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/led_PCA9632.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/one_wire_DS2482.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/one_wire_DS2431_EEPROM.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/stepper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/frontend.cpp
)


if(USE_SDL)
	find_package(SDL REQUIRED)
	add_definitions(-DUSE_SDL)
	include_directories(${SDL_INCLUDE_DIR})
    set(SOURCES ${SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/frontend/sdl.cpp)
else()
    set(SOURCES ${SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/frontend/headless.cpp)
endif()

add_definitions(-D__AVR_ATmega2560__=1 -DARDUINO=100 -DF_CPU=16000000)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arduino_sim ${CMAKE_CURRENT_SOURCE_DIR}/avr_sim)
add_executable(jedi-marlin-simulator ${SOURCES})
target_link_libraries(jedi-marlin-simulator ${SDL_LIBRARY})

install(TARGETS jedi-marlin-simulator DESTINATION /usr/bin/)
