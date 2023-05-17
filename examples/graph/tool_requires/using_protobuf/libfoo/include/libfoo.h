#pragma once


#ifdef _WIN32
  #define LIBFOO_EXPORT __declspec(dllexport)
#else
  #define LIBFOO_EXPORT
#endif

LIBFOO_EXPORT void libfoo();
