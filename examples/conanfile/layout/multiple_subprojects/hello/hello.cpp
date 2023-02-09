#include <iostream>
#include "../common/myheader.h"

int main(){
    #ifdef NDEBUG
    std::cout << "hello " <<  MYDEFINE << "\n";
    #else
    std::cout << "hello " <<  MYDEFINE << "\n";
    #endif    
    return 0;
}
