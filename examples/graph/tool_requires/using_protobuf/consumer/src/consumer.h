#pragma once


#ifdef _WIN32
  #define CONSUMER_EXPORT __declspec(dllexport)
#else
  #define CONSUMER_EXPORT
#endif

CONSUMER_EXPORT void consumer();
