#include <iostream>
#pragma once


#ifdef _WIN32
  #define SECURE_SCANNER_EXPORT __declspec(dllexport)
#else
  #define SECURE_SCANNER_EXPORT
#endif

SECURE_SCANNER_EXPORT int secure_scanner(std::string &);
