#include <iostream>
#include <cstdlib>
#include <cstdint>

int main(int argc, char* argv[]) {
    #ifdef __SANITIZE_ADDRESS__
        std::cout << "Address sanitizer enabled\n";
    #else
        std::cout << "Address sanitizer not enabled\n";
    #endif

    int foo = 0x7fffffff;
    foo += argc; // Signed integer overflow

    return EXIT_SUCCESS;
}