#ifndef FRONTEND_H
#define FRONTEND_H

#include <stdint.h>

class Frontend
{
public:
    Frontend();
    virtual ~Frontend();
    
    virtual void update() = 0;

    virtual void drawRect(const int x, const int y, const int w, const int h, uint32_t color) {}
    virtual void drawString(const int x, const int y, const char* str, uint32_t color) {}
    virtual void drawChar(const int x, const int y, const char c, uint32_t color) {}
    virtual void drawStringSmall(const int x, const int y, const char* str, uint32_t color) {}
    virtual void drawCharSmall(const int x, const int y, const char c, uint32_t color) {}

    static Frontend* instance;
};

#endif//FRONTEND_H
