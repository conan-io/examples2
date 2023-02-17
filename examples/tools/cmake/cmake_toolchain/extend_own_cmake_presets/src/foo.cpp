#include <iostream>
#include "foo.h"



void foo(){
    #ifdef NDEBUG
    std::cout << "foo/1.0: Hello World Release!\n";
    #else
    std::cout << "foo/1.0: Hello World Debug!\n";
    #endif
}
