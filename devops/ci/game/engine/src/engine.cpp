#include <iostream>
#include "engine.h"
#include "ai.h"
#include "graphics.h"

void engine(){
    ai();
    graphics();
    #ifdef NDEBUG
    std::cout << "engine/" << PKG_VERSION << ": Computing some game things (Release)!\n";
    #else
    std::cout << "engine/" << PKG_VERSION << ": Computing some game things (Debug)!\n";
    #endif
}
