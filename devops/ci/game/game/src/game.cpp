#include <iostream>
#include "game.h"
#include "engine.h"

void game(){
    engine();
    #ifdef NDEBUG
    std::cout << "game/" << PKG_VERSION << ":fun game (Release)!\n";
    #else
    std::cout << "game/" << PKG_VERSION << ":fun game (Debug)!\n";
    #endif
}
