#include <iostream>
#include "graphics.h"

void graphics(){
    #ifdef NDEBUG
    std::cout << "graphics/" << PKG_VERSION << ": Checking if things collide (Release)!\n";
    #else
    std::cout << "graphics/" << PKG_VERSION << ": Checking if things collide (Debug)!\n";
    #endif
}
