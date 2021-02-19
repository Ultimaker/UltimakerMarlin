#include "base.h"

std::vector<SimBaseComponent*> SimBaseComponent::simulator_components;

void SimBaseComponent::tickAllComponents()
{
    for(unsigned int n=0; n<simulator_components.size(); n++)
        simulator_components[n]->tick();
}
