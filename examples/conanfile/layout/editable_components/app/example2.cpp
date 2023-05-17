#include <iostream>

#include "hello/hello.h"


int main(){
    #ifdef NDEBUG
    std::cout << "main: Release!\n";
    #else
    std::cout << "main: Debug!\n";
    #endif

    hello();
    
    return 0;
}