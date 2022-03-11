#pragma once


#ifdef _WIN32
  #define mathlib_EXPORT __declspec(dllexport)
#else
  #define mathlib_EXPORT
#endif

mathlib_EXPORT void mathlib();
