#ifndef STEPPER_SIM_H
#define STEPPER_SIM_H

#include "base.h"
#include "arduinoIO.h"

class FluxAS5048B;
class StepperSim : public SimBaseComponent
{
private:
    int minStepValue;
    int maxStepValue;
    int stepValue;
    bool invertDir;
    int enablePin, stepPin, dirPin;
    int minEndstopPin, maxEndstopPin;
    FluxAS5048B* flow_sensor;
    float flow_sensor_ratio;
public:
    StepperSim(ArduinoIOSim* arduinoIO, int enablePinNr, int stepPinNr, int dirPinNr, bool invertDir);
    virtual ~StepperSim();
    
    virtual void draw(int x, int y);
    
    void setRange(int minValue, int maxValue) { minStepValue = minValue; maxStepValue = maxValue; stepValue = (maxValue + minValue) / 2; }
    void setEndstops(int minEndstopPinNr, int maxEndstopPinNr);
    int getPosition() { return stepValue; }
    void attachFlowSensor(FluxAS5048B* sensor, float ratio) { flow_sensor = sensor; flow_sensor_ratio = ratio;}
private:
    void stepPinUpdate(int pinNr, bool high);
};

#endif//STEPPER_SIM_H
