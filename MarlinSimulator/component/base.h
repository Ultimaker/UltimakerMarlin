#ifndef SIM_BASE_H
#define SIM_BASE_H

#include <stdint.h>
#include <vector>

class SimBaseComponent
{
public:
    SimBaseComponent() : drawPosX(-1), drawPosY(-1) {simulator_components.push_back(this);}
    virtual ~SimBaseComponent() {}
    
    virtual void tick() {}//Called about every ms
    void doDraw() { if (drawPosX > -1) draw(drawPosX, drawPosY); }
    virtual void draw(int x, int y) {}//Called every screen draw (~25ms)

    void setDrawPosition(int x, int y) { drawPosX = x; drawPosY = y; }

    static std::vector<SimBaseComponent*> simulator_components;
    static void tickAllComponents();
private:
    int drawPosX, drawPosY;
};

#endif//SIM_BASE_H
