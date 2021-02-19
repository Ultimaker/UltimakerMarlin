#ifndef FRONTEND_SDL_H
#define FRONTEND_SDL_H

#include "frontend.h"

struct SDL_Surface;
class SerialSim;

class SDLFrontend : public Frontend
{
public:
    SDLFrontend();
    
    virtual void update() override;

    virtual void drawRect(const int x, const int y, const int w, const int h, uint32_t color) override;
    virtual void drawString(const int x, const int y, const char* str, uint32_t color) override;
    virtual void drawChar(const int x, const int y, const char c, uint32_t color) override;
    virtual void drawStringSmall(const int x, const int y, const char* str, uint32_t color) override;
    virtual void drawCharSmall(const int x, const int y, const char c, uint32_t color) override;

private:
    SDL_Surface *screen;
    SerialSim* keyboard_input_serial;
};

#endif//FRONTEND_SDL_H
