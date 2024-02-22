#include "pkg.h"
#include <vector>
#include <string>

int main() {
    pkg();

    std::vector<std::string> vec;
    vec.push_back("test_package");

    pkg_print_vector(vec);
}
