#pragma once


#ifdef _WIN32
  #define MATH_EXPORT __declspec(dllexport)
#else
  #define MATH_EXPORT
#endif

MATH_EXPORT void math();
