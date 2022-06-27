#include "secure_scanner.h"
#include <iostream>

int main(int argc, char *argv[]) {
    std::string path;
    if(argc < 2){
        std::cout << "ERROR: The argument 'path' is needed.\n";
        return -1;
    }
    path = std::string(argv[1]);
    return secure_scanner(path);
}
