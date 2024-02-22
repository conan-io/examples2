#pragma once

#include <vector>
#include <string>


#ifdef _WIN32
  #define PKG_EXPORT __declspec(dllexport)
#else
  #define PKG_EXPORT
#endif

PKG_EXPORT void pkg();
PKG_EXPORT void pkg_print_vector(const std::vector<std::string> &strings);
