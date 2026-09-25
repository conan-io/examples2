#pragma once

#include <vector>
#include <string>


#ifdef _WIN32
  #define BASIC_EXPORT __declspec(dllexport)
#else
  #define BASIC_EXPORT
#endif

BASIC_EXPORT void basic();
BASIC_EXPORT void basic_print_vector(const std::vector<std::string> &strings);
