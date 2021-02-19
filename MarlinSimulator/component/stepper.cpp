#include "stepper.h"
#include "flux_AS5048B.h"
#include "arduinoIO.h"
#include "../frontend/frontend.h"

StepperSim::StepperSim(ArduinoIOSim* arduinoIO, int enablePinNr, int stepPinNr, int dirPinNr, bool invertDir)
{
    this->minStepValue = -1;
    this->maxStepValue = -1;
    this->stepValue = 0;
    this->minEndstopPin = -1;
    this->maxEndstopPin = -1;
    
    this->invertDir = invertDir;
    this->enablePin = enablePinNr;
    this->stepPin = stepPinNr;
    this->dirPin = dirPinNr;
    
    this->flow_sensor = nullptr;
    this->flow_sensor_ratio = 0.0f;
    
    arduinoIO->registerPortCallback(stepPinNr, DELEGATE(ioDelegate, StepperSim, *this, stepPinUpdate));
}
StepperSim::~StepperSim()
{
}

void StepperSim::stepPinUpdate(int pinNr, bool high)
{
    if (high)//Only step on high->low transition.
        return;
    if (readOutput(enablePin))
        return;
    if (readOutput(dirPin) == invertDir)
        stepValue --;
    else
        stepValue ++;
    if (minStepValue != -1)
    {
        if (stepValue < minStepValue)
            stepValue = minStepValue;
        if (minEndstopPin > -1)
            writeInput(minEndstopPin, stepValue != minStepValue);
    }
    if (maxStepValue != -1)
    {
        if (stepValue > maxStepValue)
            stepValue = maxStepValue;
        if (maxEndstopPin > -1)
            writeInput(maxEndstopPin, stepValue != maxStepValue);
    }
    if (flow_sensor)
        flow_sensor->position = stepValue * flow_sensor_ratio;
}

void StepperSim::setEndstops(int minEndstopPinNr, int maxEndstopPinNr)
{
    minEndstopPin = minEndstopPinNr;
    maxEndstopPin = maxEndstopPinNr;
    
    writeInput(minEndstopPin, stepValue != minStepValue);
    writeInput(maxEndstopPin, stepValue != maxStepValue);
}

void StepperSim::draw(int x, int y)
{
    char buffer[32];
    sprintf(buffer, "%i steps", int(stepValue));
    Frontend::instance->drawString(x, y, buffer, 0xFFFFFF);
}
