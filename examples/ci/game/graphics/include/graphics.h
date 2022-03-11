#pragma once


#ifdef _WIN32
  #define graphics_EXPORT __declspec(dllexport)
#else
  #define graphics_EXPORT
#endif

graphics_EXPORT void graphics();
