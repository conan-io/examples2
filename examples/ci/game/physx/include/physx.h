#pragma once


#ifdef _WIN32
  #define PHYSX_EXPORT __declspec(dllexport)
#else
  #define PHYSX_EXPORT
#endif

PHYSX_EXPORT void physx();
