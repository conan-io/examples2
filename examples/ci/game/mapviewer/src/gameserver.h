#pragma once


#ifdef _WIN32
  #define mapviewer_EXPORT __declspec(dllexport)
#else
  #define mapviewer_EXPORT
#endif

mapviewer_EXPORT void mapviewer();
