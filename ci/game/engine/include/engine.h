#pragma once


#ifdef _WIN32
  #define ENGINE_EXPORT __declspec(dllexport)
#else
  #define ENGINE_EXPORT
#endif

ENGINE_EXPORT void engine();
