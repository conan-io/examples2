#pragma once


#ifdef _WIN32
  #define AI_EXPORT __declspec(dllexport)
#else
  #define AI_EXPORT
#endif

AI_EXPORT void ai(int intelligence=0);
