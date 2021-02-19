#ifndef HEADLESS_H
#define HEADLESS_H

#include "frontend.h"

class HeadlessFrontend : public Frontend
{
public:
    HeadlessFrontend();
    
    virtual void update() override;

private:
};

#endif//HEADLESS_H
