#pragma once

#include <string>

#ifdef WIN32
  #define FN_EXPORT __declspec(dllexport) 
#else
  #define FN_EXPORT  
#endif

FN_EXPORT void str_printer(std::string);
