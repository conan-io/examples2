#include <iostream>
#include <cstdlib>

int main() {
    #ifdef __SANITIZE_ADDRESS__
        std::cout << "Address sanitizer enabled\n";
    #else
        std::cout << "Address sanitizer not enabled\n";
    #endif

    int foo[100];
    foo[100] = 42; // Out-of-bounds write

    return EXIT_SUCCESS;
}