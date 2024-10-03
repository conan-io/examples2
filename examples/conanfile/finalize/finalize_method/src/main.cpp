#include <fstream>
#include <iostream>

int main() {
    std::ifstream in("whoami.txt", std::ios_base::in);
    std::cout << in.rdbuf() << '\n';
}
