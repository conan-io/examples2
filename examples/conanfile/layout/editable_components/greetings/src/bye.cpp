#include <iostream>
#include "bye.h"

int bye(){
    #ifdef NDEBUG
    std::cout << "bye: Release!\n";
    #else
    std::cout << "bye: Debug!\n";
    #endif
    return 0;
}