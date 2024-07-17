#include <iostream>
#include "mathlib.h"

void mathlib(){
    #ifdef NDEBUG
    std::cout << "mathlib/" << PKG_VERSION << ": mathlib maths (Release)!\n";
    #else
    std::cout << "mathlib/" << PKG_VERSION << ": mathlib maths (Debug)!\n";
    #endif
}
