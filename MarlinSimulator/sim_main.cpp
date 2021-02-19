#include <getopt.h>
#include <Arduino.h>

#include <avr/io.h>

#include "component/adc.h"
#include "component/heater.h"
#include "component/serial.h"
#include "component/led_PCA9632.h"
#include "component/adc_ADS101X.h"
#include "component/FDC1004.h"
#include "component/flux_AS5048B.h"
#include "component/one_wire_DS2431_EEPROM.h"
#include "component/one_wire_DS2482.h"
#include "component/arduinoIO.h"
#include "component/stepper.h"

#ifdef USE_SDL
#include "frontend/sdl.h"
#else//!USE_SDL
#include "frontend/headless.h"
#endif//!USE_SDL

#include "../Marlin/stepper.h"


#define PRINTER_DOWN_SCALE 2
class printerSim : public SimBaseComponent
{
private:
    StepperSim* x;
    StepperSim* y;
    StepperSim* z;
    StepperSim* e0;
    StepperSim* e1;
    int e0stepPos, e1stepPos;
    int map[(X_MAX_POS - X_MIN_POS)/PRINTER_DOWN_SCALE+1][(Y_MAX_POS - Y_MIN_POS)/PRINTER_DOWN_SCALE+1];
public:
    printerSim(StepperSim* x, StepperSim* y, StepperSim* z, StepperSim* e0, StepperSim* e1)
    : x(x), y(y), z(z), e0(e0), e1(e1)
    {
        e0stepPos = e0->getPosition();
        e1stepPos = e1->getPosition();
        memset(map, 0, sizeof(map));
    }
    virtual ~printerSim()
    {
    }
    
    void draw(int _x, int _y)
    {
        Frontend::instance->drawRect(_x, _y, (X_MAX_POS - X_MIN_POS) / PRINTER_DOWN_SCALE + 1, (Y_MAX_POS - Y_MIN_POS) / PRINTER_DOWN_SCALE + 1, 0x202020);
        for(unsigned int py=0; py<(Y_MAX_POS - Y_MIN_POS)/PRINTER_DOWN_SCALE; py++)
            for(unsigned int px=0; px<(X_MAX_POS - X_MIN_POS)/PRINTER_DOWN_SCALE; px++)
                Frontend::instance->drawRect(_x+px, _y+py, 1, 1, map[px][py] + 0x202020);

        Frontend::instance->drawRect(_x + (X_MAX_POS - X_MIN_POS) / PRINTER_DOWN_SCALE + 2, _y, 1, (Z_MAX_POS - Z_MIN_POS) / PRINTER_DOWN_SCALE + 1, 0x202020);

        float pos[3];
        float stepsPerUnit[4] = DEFAULT_AXIS_STEPS_PER_UNIT;
        pos[0] = x->getPosition() / stepsPerUnit[X_AXIS];
        pos[1] = Y_MAX_POS - y->getPosition() / stepsPerUnit[Y_AXIS];
        pos[2] = z->getPosition() / stepsPerUnit[Z_AXIS];
        
        map[int(pos[0]/PRINTER_DOWN_SCALE)][int(pos[1]/PRINTER_DOWN_SCALE)] += e0->getPosition() - e0stepPos;
        map[int(pos[0]/PRINTER_DOWN_SCALE)][int(pos[1]/PRINTER_DOWN_SCALE)] += e1->getPosition() - e1stepPos;

        e0stepPos = e0->getPosition();
        e1stepPos = e1->getPosition();

        Frontend::instance->drawRect(_x + pos[0] / PRINTER_DOWN_SCALE, _y + pos[1] / PRINTER_DOWN_SCALE, 1, 1, 0xFFFFFF);
        Frontend::instance->drawRect(_x + (X_MAX_POS - X_MIN_POS) / PRINTER_DOWN_SCALE + 2, _y + pos[2] / PRINTER_DOWN_SCALE, 1, 1, 0xFFFFFF);
    }
};

enum MachineType
{
    Jedi,
    Watto
};
MachineType machine_type = Jedi;

bool setupSimulatorEnvironment(int argc, char** argv)
{
    int serial_listen_port_nr = 0;

    int c;
    while ((c=getopt(argc, argv, "p:m:")) != -1)
    switch (c)
    {
    case 'p'://Listen port
        serial_listen_port_nr = atoi(optarg);
        break;
    case 'm'://Machine type
        if (strcmp(optarg, "Jedi") == 0)
        {
            machine_type = Jedi;
        }
        else if (strcmp(optarg, "Watto") == 0)
        {
            machine_type = Watto;
        }
        else
        {
            fprintf(stderr, "Unknown machine type %s\n", optarg);
            return false;
        }
    case '?':
        fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        return false;
    default:
        return false;
    }
    for(int index=optind; index<argc; index++)
    {
        fprintf(stderr, "Unknown argument %s\n", argv[index]);
        return false;
    }

#ifdef USE_SDL
    Frontend::instance = new SDLFrontend();
#else//!USE_SDL
    Frontend::instance = new HeadlessFrontend();
#endif//!USE_SDL

    AvrAdcSim* adc = new AvrAdcSim();
    ArduinoIOSim* arduino_io = new ArduinoIOSim();
    
    I2CSim* i2c = new I2CSim();

    ///Default Printer stepper motor setup.
    StepperSim* xStep = new StepperSim(arduino_io, X_ENABLE_PIN, X_STEP_PIN, X_DIR_PIN, DEFAULT_INVERT_X_DIR);
    StepperSim* yStep = new StepperSim(arduino_io, Y_ENABLE_PIN, Y_STEP_PIN, Y_DIR_PIN, DEFAULT_INVERT_Y_DIR);
    StepperSim* zStep = new StepperSim(arduino_io, Z_ENABLE_PIN, Z_STEP_PIN, Z_DIR_PIN, DEFAULT_INVERT_Z_DIR);
    StepperSim* e0Step = new StepperSim(arduino_io, E0_ENABLE_PIN, E0_STEP_PIN, E0_DIR_PIN, DEFAULT_INVERT_E0_DIR);
    StepperSim* e1Step = new StepperSim(arduino_io, E1_ENABLE_PIN, E1_STEP_PIN, E1_DIR_PIN, DEFAULT_INVERT_E1_DIR);
    float stepsPerUnit[4] = DEFAULT_AXIS_STEPS_PER_UNIT;
    xStep->setRange(0, X_MAX_POS * stepsPerUnit[X_AXIS]);
    yStep->setRange(0, Y_MAX_POS * stepsPerUnit[Y_AXIS]);
    zStep->setRange(0, Z_MAX_POS * stepsPerUnit[Z_AXIS]);
    xStep->setEndstops(X_MIN_PIN, X_MAX_PIN);
    yStep->setEndstops(Y_MIN_PIN, Y_MAX_PIN);
    zStep->setEndstops(Z_MIN_PIN, Z_MAX_PIN);
    (new printerSim(xStep, yStep, zStep, e0Step, e1Step))->setDrawPosition(5, 70);
    e0Step->setDrawPosition(130, 100);
    e1Step->setDrawPosition(130, 110);

    //All printers have heaters.
    HeaterSim* heater_0 = new HeaterSim(HEATER_0_PIN);
    HeaterSim* heater_1 = new HeaterSim(HEATER_1_PIN);
    HeaterSim* heater_bed = new HeaterSim(HEATER_BED_PIN, 0.2);
    heater_0->setDrawPosition(130, 70);
    heater_1->setDrawPosition(130, 80);
    heater_bed->setDrawPosition(130, 90);

    (new SerialSim(0))->setDrawPosition(0, 0);
    (new SerialSim(1))->setDrawPosition(0, 0);
    SerialSim* jedi_serial = new SerialSim(2);
    jedi_serial->setDrawPosition(0, 0);
    if (serial_listen_port_nr != 0)
        jedi_serial->listenOnPort(serial_listen_port_nr);
    (new SerialSim(3))->setDrawPosition(0, 0);
    
    switch(machine_type)
    {
    case Jedi:
        {
            //Hotend EEPROMs
            OneWireDS2482* one_wire_0 = new OneWireDS2482(i2c, 0x18);
            OneWireDS2482* one_wire_1 = new OneWireDS2482(i2c, 0x19);
            one_wire_0->setDevice(new OneWireDS2431EEPROM(0x12345));
            one_wire_1->setDevice(new OneWireDS2431EEPROM(0x12347));
            
            //Active leveling sensor
            (new FDC1004(i2c, 0x50));
            
            //Hotend ADCs
            AdcADS101X* adc_ads101x = new AdcADS101X(i2c, 0x48);
            
            //LED driver in print head, connected to RGB LEDs and FANs
            (new LedPCA9635Sim(i2c, 0x44))->setDrawPosition(120, 0);
            //Case LEDs (RGBW)
            (new LedPCA9632Sim(i2c, 0x61))->setDrawPosition(120, 3);
            
            //Flow sensors in the feeders.
            FluxAS5048B* flow_0 = new FluxAS5048B(i2c, 0x40);
            FluxAS5048B* flow_1 = new FluxAS5048B(i2c, 0x41);
            float flow_sensor_radius = 9.10;//radius in mm.
            e0Step->attachFlowSensor(flow_0, stepsPerUnit[E_AXIS] / (float(1 << 14) / (M_PI * flow_sensor_radius)));
            e1Step->attachFlowSensor(flow_1, stepsPerUnit[E_AXIS] / (float(1 << 14) / (M_PI * flow_sensor_radius)));

            //Link the proper ADCs to our heater outputs.
            adc_ads101x->setReadCallback(0, DELEGATE(AdcDelegate, HeaterSim, *heater_0, adcReadout));
            adc_ads101x->setReadCallback(1, DELEGATE(AdcDelegate, HeaterSim, *heater_0, adcReadout));
            adc->setReadCallback(TEMP_BED_PIN, DELEGATE(AdcDelegate, HeaterSim, *heater_bed, adcReadout));
        }
        break;
    case Watto:
        {
            //ADC links to AVR ADCs for PT100 temperature measurements.
            adc->setReadCallback(TEMP_0_PIN, DELEGATE(AdcDelegate, HeaterSim, *heater_0, adcReadout));
            adc->setReadCallback(TEMP_1_PIN, DELEGATE(AdcDelegate, HeaterSim, *heater_1, adcReadout));
        }
        break;
    }

    FILE* f = fopen("eeprom.save", "rb");
    if (f)
    {
        if (fread(avr_simulation_eeprom_storage, sizeof(avr_simulation_eeprom_storage), 1, f) < 1)
            printf("Failed to read EEPROM, eeprom.save corrupt?");
        fclose(f);
    }

    return true;
}
