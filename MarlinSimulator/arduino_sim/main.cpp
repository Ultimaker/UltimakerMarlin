#include <Arduino.h>

extern bool setupSimulatorEnvironment(int argc, char** argv);

int main(int argc, char** argv)
{
    if (!setupSimulatorEnvironment(argc, argv))
    {
        return 1;
    }

    init();

    setup();

    for (;;) {
        loop();
        if (serialEventRun) serialEventRun();
    }

    return 0;
}
