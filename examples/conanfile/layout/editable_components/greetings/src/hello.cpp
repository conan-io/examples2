#include <iostream>
#include "hello.h"

int hello(){
    #ifdef NDEBUG
    std::cout << "hello: Release!\n";
    #else
    std::cout << "hello: Debug!\n";
    #endif 
    return 0;
}