#pragma once


#ifdef _WIN32
  #define FOO_EXPORT __declspec(dllexport)
#else
  #define FOO_EXPORT
#endif

FOO_EXPORT void foo();
