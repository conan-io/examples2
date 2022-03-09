#pragma once


#ifdef _WIN32
  #define MATRIX_EXPORT __declspec(dllexport)
#else
  #define MATRIX_EXPORT
#endif

MATRIX_EXPORT void matrix();
