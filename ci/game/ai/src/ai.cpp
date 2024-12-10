#include <iostream>
#include "ai.h"
#include "mathlib.h"

void ai(int intelligence){
    mathlib();
    #ifdef NDEBUG
    std::cout << "ai/" << PKG_VERSION << ": Some Artificial Intelligence for aliens (Release)!\n";
    #else
    std::cout << "ai/" << PKG_VERSION << ": Some Artificial Intelligence for aliens (Debug)!\n";
    #endif

    std::cout << "ai/" << PKG_VERSION << ": Intelligence level="<< intelligence << "\n";
}
