#include <iostream>
#include "ai.h"
#include "matrix.h"

void ai(int intelligence){
    matrix();
    #ifdef NDEBUG
    std::cout << "ai/" << PKG_VERSION << ": Some Artificial Intelligence for enemies (Release)!\n";
    #else
    std::cout << "ai/" << PKG_VERSION << ": Some Artificial Intelligence for enemies (Debug)!\n";
    #endif

    std::cout << "ai/" << PKG_VERSION << ": Intelligence level="<< intelligence << "\n";
}
