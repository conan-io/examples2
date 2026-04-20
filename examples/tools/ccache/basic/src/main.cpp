#include "basic.h"
#include <vector>
#include <string>

int main() {
    basic();

    std::vector<std::string> vec;
    vec.push_back("test_package");

    basic_print_vector(vec);
}
