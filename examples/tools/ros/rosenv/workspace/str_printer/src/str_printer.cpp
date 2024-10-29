#include <iostream>
#include <string>

#include "fmt/core.h"
#include "fmt/color.h"

#include "str_printer.h"

void str_printer(std::string str) {
    fmt::print(fmt::fg(fmt::color::gold) | fmt::emphasis::bold, "{}\n", str);
}
