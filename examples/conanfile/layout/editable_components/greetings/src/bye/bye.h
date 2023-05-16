
#pragma once



#ifdef _WIN32
  #define BYE_EXPORT __declspec(dllexport)
#else
  #define BYE_EXPORT
#endif
BYE_EXPORT int bye();