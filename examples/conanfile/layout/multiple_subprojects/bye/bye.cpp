#include <iostream>

#include "../common/myheader.h"

int main(){
    #ifdef NDEBUG
    std::cout << "bye " <<  MYDEFINE << "\n";
    #else
    std::cout << "bye " <<  MYDEFINE << "\n";
    #endif
}
