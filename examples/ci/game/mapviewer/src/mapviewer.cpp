#include <iostream>
#include "mapviewer.h"
#include "graphics.h"

void mapviewer(){
    graphics();
    #ifdef NDEBUG
    std::cout << "mapviewer/" << PKG_VERSION << ":serving the game (Release)!\n";
    #else
    std::cout << "mapviewer/" << PKG_VERSION << ":serving the game (Debug)!\n";
    #endif
}
